use crate::{FrameType2D, FrameType3D, GridLine, SapModel};
use std::f64::consts::PI;

impl SapModel {
    /// Generates a continuous multi-span beam along the X-axis (`cFile.NewBeam`).
    pub fn generate_beam(
        &mut self,
        num_spans: usize,
        span_length: f64,
        restraint: bool,
        section: &str,
    ) {
        self.points.clear();
        self.frames.clear();
        self.areas.clear();
        self.solids.clear();
        self.links.clear();
        self.case_results.clear();
        self.steel_design_results.clear();
        self.concrete_design_results.clear();
        self.is_locked = false;

        let n_spans = num_spans.max(1);
        let mut x_coords = Vec::with_capacity(n_spans + 1);
        let start_x = -0.5 * (n_spans as f64) * span_length;
        for i in 0..=n_spans {
            x_coords.push(start_x + (i as f64) * span_length);
        }

        if let Some(cs) = self.coord_systems.get_mut("GLOBAL") {
            cs.grids.clear();
            for (i, &x) in x_coords.iter().enumerate() {
                cs.grids.push(GridLine {
                    axis: "X".to_string(),
                    id: format!("{}", (b'A' + (i as u8 % 26)) as char),
                    coordinate: x,
                    visible: true,
                    bubble_loc: "End".to_string(),
                });
            }
            cs.grids.push(GridLine {
                axis: "Z".to_string(),
                id: "Z1".to_string(),
                coordinate: 0.0,
                visible: true,
                bubble_loc: "Start".to_string(),
            });
        }

        for (i, &x) in x_coords.iter().enumerate() {
            let pt_name = format!("{}", i + 1);
            self.add_point(&pt_name, x, 0.0, 0.0);
            if restraint {
                if let Some(pt) = self.points.get_mut(&pt_name) {
                    if i == 0 {
                        // Pinned support (UX, UY, UZ, RX)
                        pt.restraint = [true, true, true, true, false, false];
                    } else {
                        // Roller support (UY, UZ, RX)
                        pt.restraint = [false, true, true, true, false, false];
                    }
                }
            }
        }

        let sec_name = if self.frame_sections.contains_key(section) {
            section.to_string()
        } else {
            self.frame_sections
                .keys()
                .next()
                .cloned()
                .unwrap_or_else(|| "FSEC1".to_string())
        };

        for i in 0..n_spans {
            let f_name = format!("{}", i + 1);
            self.add_frame(
                &f_name,
                format!("{}", i + 1),
                format!("{}", i + 2),
                &sec_name,
            );
        }
        self.model_name = format!("Continuous Beam ({n_spans} Spans)");
    }

    /// Generates a 2D Frame in the XZ plane (`cFile.New2DFrame`).
    pub fn generate_2d_frame(
        &mut self,
        frame_type: FrameType2D,
        num_stories: usize,
        story_height: f64,
        num_bays: usize,
        bay_width: f64,
        restraint: bool,
        beam_sec: &str,
        col_sec: &str,
        brace_sec: &str,
        eccentricity: f64,
    ) {
        self.points.clear();
        self.frames.clear();
        self.areas.clear();
        self.solids.clear();
        self.links.clear();
        self.case_results.clear();
        self.steel_design_results.clear();
        self.concrete_design_results.clear();
        self.is_locked = false;

        let n_stories = num_stories.max(1);
        let n_bays = num_bays.max(1);
        let start_x = -0.5 * (n_bays as f64) * bay_width;

        if let Some(cs) = self.coord_systems.get_mut("GLOBAL") {
            cs.grids.clear();
            for ix in 0..=n_bays {
                cs.grids.push(GridLine {
                    axis: "X".to_string(),
                    id: format!("{}", (b'A' + (ix as u8 % 26)) as char),
                    coordinate: start_x + (ix as f64) * bay_width,
                    visible: true,
                    bubble_loc: "End".to_string(),
                });
            }
            for iz in 0..=n_stories {
                cs.grids.push(GridLine {
                    axis: "Z".to_string(),
                    id: format!("Z{}", iz + 1),
                    coordinate: (iz as f64) * story_height,
                    visible: true,
                    bubble_loc: "Start".to_string(),
                });
            }
        }

        let default_sec = self
            .frame_sections
            .keys()
            .next()
            .cloned()
            .unwrap_or_else(|| "FSEC1".to_string());
        let b_sec = if self.frame_sections.contains_key(beam_sec) {
            beam_sec.to_string()
        } else {
            default_sec.clone()
        };
        let c_sec = if self.frame_sections.contains_key(col_sec) {
            col_sec.to_string()
        } else {
            default_sec.clone()
        };
        let br_sec = if self.frame_sections.contains_key(brace_sec) {
            brace_sec.to_string()
        } else {
            default_sec
        };

        let mut pt_id = 1usize;
        let mut grid_pts = vec![vec![String::new(); n_stories + 1]; n_bays + 1];
        for ix in 0..=n_bays {
            let x = start_x + (ix as f64) * bay_width;
            for iz in 0..=n_stories {
                let z = (iz as f64) * story_height;
                let name = format!("{pt_id}");
                pt_id += 1;
                self.add_point(&name, x, 0.0, z);
                if iz == 0 && restraint {
                    if let Some(p) = self.points.get_mut(&name) {
                        p.restraint = [true, true, true, true, true, true];
                    }
                }
                grid_pts[ix][iz] = name;
            }
        }

        let mut fr_id = 1usize;
        // Columns
        for ix in 0..=n_bays {
            for iz in 0..n_stories {
                let name = format!("{fr_id}");
                fr_id += 1;
                self.add_frame(&name, &grid_pts[ix][iz], &grid_pts[ix][iz + 1], &c_sec);
            }
        }
        // Beams & Braces
        for iz in 1..=n_stories {
            for ix in 0..n_bays {
                match frame_type {
                    FrameType2D::PortalFrame => {
                        let name = format!("{fr_id}");
                        fr_id += 1;
                        self.add_frame(&name, &grid_pts[ix][iz], &grid_pts[ix + 1][iz], &b_sec);
                    }
                    FrameType2D::ConcentricBraced => {
                        let mid_x = start_x + ((ix as f64) + 0.5) * bay_width;
                        let z = (iz as f64) * story_height;
                        let mid_pt = format!("{pt_id}");
                        pt_id += 1;
                        self.add_point(&mid_pt, mid_x, 0.0, z);

                        let b1 = format!("{fr_id}");
                        fr_id += 1;
                        self.add_frame(&b1, &grid_pts[ix][iz], &mid_pt, &b_sec);
                        let b2 = format!("{fr_id}");
                        fr_id += 1;
                        self.add_frame(&b2, &mid_pt, &grid_pts[ix + 1][iz], &b_sec);

                        // Chevron braces with pinned ends (M3 in-plane major bending released)
                        let br1 = format!("{fr_id}");
                        fr_id += 1;
                        self.add_frame(&br1, &grid_pts[ix][iz - 1], &mid_pt, &br_sec);
                        if let Some(f) = self.frames.get_mut(&br1) {
                            f.release_i[5] = true;
                            f.release_j[5] = true;
                        }
                        let br2 = format!("{fr_id}");
                        fr_id += 1;
                        self.add_frame(&br2, &grid_pts[ix + 1][iz - 1], &mid_pt, &br_sec);
                        if let Some(f) = self.frames.get_mut(&br2) {
                            f.release_i[5] = true;
                            f.release_j[5] = true;
                        }
                    }
                    FrameType2D::EccentricBraced => {
                        let e_half = (eccentricity.abs().max(0.1 * bay_width)).min(0.4 * bay_width) * 0.5;
                        let mid_x = start_x + ((ix as f64) + 0.5) * bay_width;
                        let z = (iz as f64) * story_height;
                        let p_left = format!("{pt_id}");
                        pt_id += 1;
                        self.add_point(&p_left, mid_x - e_half, 0.0, z);
                        let p_right = format!("{pt_id}");
                        pt_id += 1;
                        self.add_point(&p_right, mid_x + e_half, 0.0, z);

                        for (p1, p2) in [
                            (&grid_pts[ix][iz], &p_left),
                            (&p_left, &p_right),
                            (&p_right, &grid_pts[ix + 1][iz]),
                        ] {
                            let b = format!("{fr_id}");
                            fr_id += 1;
                            self.add_frame(&b, p1, p2, &b_sec);
                        }
                        for (p1, p2) in [
                            (&grid_pts[ix][iz - 1], &p_left),
                            (&grid_pts[ix + 1][iz - 1], &p_right),
                        ] {
                            let br = format!("{fr_id}");
                            fr_id += 1;
                            self.add_frame(&br, p1, p2, &br_sec);
                            if let Some(f) = self.frames.get_mut(&br) {
                                f.release_i[5] = true;
                                f.release_j[5] = true;
                            }
                        }
                    }
                }
            }
        }
        self.model_name = format!("2D {:?} ({n_stories} Stories, {n_bays} Bays)", frame_type);
    }

    /// Generates a 2D Pratt/Warren Truss in the XZ plane.
    pub fn generate_2d_truss(
        &mut self,
        num_bays: usize,
        bay_width: f64,
        height: f64,
        restraint: bool,
        chord_sec: &str,
        web_sec: &str,
    ) {
        self.points.clear();
        self.frames.clear();
        self.areas.clear();
        self.solids.clear();
        self.links.clear();
        self.case_results.clear();
        self.steel_design_results.clear();
        self.concrete_design_results.clear();
        self.is_locked = false;

        let n_bays = num_bays.max(2);
        let start_x = -0.5 * (n_bays as f64) * bay_width;
        let default_sec = self
            .frame_sections
            .keys()
            .next()
            .cloned()
            .unwrap_or_else(|| "FSEC1".to_string());
        let c_sec = if self.frame_sections.contains_key(chord_sec) {
            chord_sec.to_string()
        } else {
            default_sec.clone()
        };
        let w_sec = if self.frame_sections.contains_key(web_sec) {
            web_sec.to_string()
        } else {
            default_sec
        };

        let mut bot_pts = Vec::with_capacity(n_bays + 1);
        let mut top_pts = Vec::with_capacity(n_bays + 1);
        let mut pt_id = 1usize;
        for i in 0..=n_bays {
            let x = start_x + (i as f64) * bay_width;
            let b_name = format!("{pt_id}");
            pt_id += 1;
            self.add_point(&b_name, x, 0.0, 0.0);
            if restraint && (i == 0 || i == n_bays) {
                if let Some(p) = self.points.get_mut(&b_name) {
                    p.restraint = if i == 0 {
                        [true, true, true, true, false, true]
                    } else {
                        [false, true, true, true, false, true]
                    };
                }
            } else if let Some(p) = self.points.get_mut(&b_name) {
                p.restraint = [false, true, false, true, false, true];
            }
            bot_pts.push(b_name);

            let t_name = format!("{pt_id}");
            pt_id += 1;
            self.add_point(&t_name, x, 0.0, height);
            if let Some(p) = self.points.get_mut(&t_name) {
                p.restraint = [false, true, false, true, false, true];
            }
            top_pts.push(t_name);
        }

        let mut fr_id = 1usize;
        // Bottom & Top Chords
        for i in 0..n_bays {
            let b_fr = format!("{fr_id}");
            fr_id += 1;
            self.add_frame(&b_fr, &bot_pts[i], &bot_pts[i + 1], &c_sec);

            let t_fr = format!("{fr_id}");
            fr_id += 1;
            self.add_frame(&t_fr, &top_pts[i], &top_pts[i + 1], &c_sec);
        }
        // Verticals
        for i in 0..=n_bays {
            let v_fr = format!("{fr_id}");
            fr_id += 1;
            self.add_frame(&v_fr, &bot_pts[i], &top_pts[i], &w_sec);
        }
        // Diagonals
        for i in 0..n_bays {
            let d_fr = format!("{fr_id}");
            fr_id += 1;
            if i < n_bays / 2 {
                self.add_frame(&d_fr, &bot_pts[i + 1], &top_pts[i], &w_sec);
            } else {
                self.add_frame(&d_fr, &bot_pts[i], &top_pts[i + 1], &w_sec);
            }
            if let Some(f) = self.frames.get_mut(&d_fr) {
                f.release_i[5] = true;
                f.release_j[5] = true;
            }
        }
        self.model_name = format!("2D Truss ({n_bays} Bays)");
    }

    /// Generates a 3D Multi-Story Building Frame (`cFile.New3DFrame`).
    pub fn generate_3d_frame(
        &mut self,
        frame_type: FrameType3D,
        num_stories: usize,
        story_height: f64,
        num_bays_x: usize,
        bay_width_x: f64,
        num_bays_y: usize,
        bay_width_y: f64,
        restraint: bool,
        beam_sec: &str,
        col_sec: &str,
        area_sec: &str,
    ) {
        self.points.clear();
        self.frames.clear();
        self.areas.clear();
        self.solids.clear();
        self.links.clear();
        self.case_results.clear();
        self.steel_design_results.clear();
        self.concrete_design_results.clear();
        self.is_locked = false;

        let nz = num_stories.max(1);
        let nx = num_bays_x.max(1);
        let ny = num_bays_y.max(1);
        let x0 = -0.5 * (nx as f64) * bay_width_x;
        let y0 = -0.5 * (ny as f64) * bay_width_y;

        if let Some(cs) = self.coord_systems.get_mut("GLOBAL") {
            cs.grids.clear();
            for ix in 0..=nx {
                cs.grids.push(GridLine {
                    axis: "X".to_string(),
                    id: format!("{}", (b'A' + (ix as u8 % 26)) as char),
                    coordinate: x0 + (ix as f64) * bay_width_x,
                    visible: true,
                    bubble_loc: "End".to_string(),
                });
            }
            for iy in 0..=ny {
                cs.grids.push(GridLine {
                    axis: "Y".to_string(),
                    id: format!("{}", iy + 1),
                    coordinate: y0 + (iy as f64) * bay_width_y,
                    visible: true,
                    bubble_loc: "Start".to_string(),
                });
            }
            for iz in 0..=nz {
                cs.grids.push(GridLine {
                    axis: "Z".to_string(),
                    id: format!("Z{}", iz + 1),
                    coordinate: (iz as f64) * story_height,
                    visible: true,
                    bubble_loc: "Start".to_string(),
                });
            }
        }

        let default_fsec = self
            .frame_sections
            .keys()
            .next()
            .cloned()
            .unwrap_or_else(|| "FSEC1".to_string());
        let b_sec = if self.frame_sections.contains_key(beam_sec) {
            beam_sec.to_string()
        } else {
            default_fsec.clone()
        };
        let c_sec = if self.frame_sections.contains_key(col_sec) {
            col_sec.to_string()
        } else {
            default_fsec
        };
        let a_sec = if self.area_sections.contains_key(area_sec) {
            area_sec.to_string()
        } else {
            self.area_sections
                .keys()
                .next()
                .cloned()
                .unwrap_or_else(|| "ASEC1".to_string())
        };

        let mut pt_id = 1usize;
        let mut pts = vec![vec![vec![String::new(); nz + 1]; ny + 1]; nx + 1];
        for iz in 0..=nz {
            let z = (iz as f64) * story_height;
            for iy in 0..=ny {
                let y = y0 + (iy as f64) * bay_width_y;
                for ix in 0..=nx {
                    let x = x0 + (ix as f64) * bay_width_x;
                    if frame_type == FrameType3D::PerimeterFrame
                        && ix > 0
                        && ix < nx
                        && iy > 0
                        && iy < ny
                    {
                        continue;
                    }
                    let name = format!("{pt_id}");
                    pt_id += 1;
                    self.add_point(&name, x, y, z);
                    if iz == 0 && restraint {
                        if let Some(p) = self.points.get_mut(&name) {
                            p.restraint = [true; 6];
                        }
                    }
                    pts[ix][iy][iz] = name;
                }
            }
        }

        let mut fr_id = 1usize;
        // Columns
        for iz in 0..nz {
            for iy in 0..=ny {
                for ix in 0..=nx {
                    if pts[ix][iy][iz].is_empty() || pts[ix][iy][iz + 1].is_empty() {
                        continue;
                    }
                    let name = format!("{fr_id}");
                    fr_id += 1;
                    self.add_frame(&name, &pts[ix][iy][iz], &pts[ix][iy][iz + 1], &c_sec);
                }
            }
        }

        // Beams (unless FlatPlate)
        if frame_type != FrameType3D::FlatPlate {
            for iz in 1..=nz {
                // X-direction beams
                for iy in 0..=ny {
                    for ix in 0..nx {
                        if pts[ix][iy][iz].is_empty() || pts[ix + 1][iy][iz].is_empty() {
                            continue;
                        }
                        let name = format!("{fr_id}");
                        fr_id += 1;
                        self.add_frame(&name, &pts[ix][iy][iz], &pts[ix + 1][iy][iz], &b_sec);
                    }
                }
                // Y-direction beams
                for ix in 0..=nx {
                    for iy in 0..ny {
                        if pts[ix][iy][iz].is_empty() || pts[ix][iy + 1][iz].is_empty() {
                            continue;
                        }
                        let name = format!("{fr_id}");
                        fr_id += 1;
                        self.add_frame(&name, &pts[ix][iy][iz], &pts[ix][iy + 1][iz], &b_sec);
                    }
                }
            }
        }

        // Floor Slabs (for BeamSlab and FlatPlate)
        if matches!(frame_type, FrameType3D::BeamSlab | FrameType3D::FlatPlate) {
            let mut ar_id = 1usize;
            for iz in 1..=nz {
                for iy in 0..ny {
                    for ix in 0..nx {
                        let p1 = pts[ix][iy][iz].clone();
                        let p2 = pts[ix + 1][iy][iz].clone();
                        let p3 = pts[ix + 1][iy + 1][iz].clone();
                        let p4 = pts[ix][iy + 1][iz].clone();
                        let name = format!("{ar_id}");
                        ar_id += 1;
                        self.add_area(&name, vec![p1, p2, p3, p4], &a_sec);
                    }
                }
            }
        }

        self.model_name = format!(
            "3D {:?} ({nz} Stories, {nx}x{ny} Bays)",
            frame_type
        );
    }

    /// Generates a 2D Meshed Shear Wall in the XZ plane (`cFile.NewWall`).
    pub fn generate_wall(
        &mut self,
        num_x_divs: usize,
        div_width_x: f64,
        num_z_divs: usize,
        div_width_z: f64,
        restraint: bool,
        area_sec: &str,
    ) {
        self.points.clear();
        self.frames.clear();
        self.areas.clear();
        self.solids.clear();
        self.links.clear();
        self.case_results.clear();
        self.steel_design_results.clear();
        self.concrete_design_results.clear();
        self.is_locked = false;

        let nx = num_x_divs.max(1);
        let nz = num_z_divs.max(1);
        let x0 = -0.5 * (nx as f64) * div_width_x;
        let a_sec = if self.area_sections.contains_key(area_sec) {
            area_sec.to_string()
        } else {
            self.area_sections
                .keys()
                .next()
                .cloned()
                .unwrap_or_else(|| "ASEC1".to_string())
        };

        let mut pt_id = 1usize;
        let mut pts = vec![vec![String::new(); nz + 1]; nx + 1];
        for iz in 0..=nz {
            let z = (iz as f64) * div_width_z;
            for ix in 0..=nx {
                let x = x0 + (ix as f64) * div_width_x;
                let name = format!("{pt_id}");
                pt_id += 1;
                self.add_point(&name, x, 0.0, z);
                if iz == 0 && restraint {
                    if let Some(p) = self.points.get_mut(&name) {
                        p.restraint = [true; 6];
                    }
                }
                pts[ix][iz] = name;
            }
        }

        let mut ar_id = 1usize;
        for iz in 0..nz {
            for ix in 0..nx {
                let name = format!("{ar_id}");
                ar_id += 1;
                self.add_area(
                    &name,
                    vec![
                        pts[ix][iz].clone(),
                        pts[ix + 1][iz].clone(),
                        pts[ix + 1][iz + 1].clone(),
                        pts[ix][iz + 1].clone(),
                    ],
                    &a_sec,
                );
            }
        }
        self.model_name = format!("Shear Wall ({nx}x{nz} Mesh)");
    }

    /// Generates a 3D Spherical Dome Shell (`saptemplates.exe` `cGMDome`).
    pub fn generate_dome_shell(
        &mut self,
        radius: f64,
        roll_angle_deg: f64,
        num_angular_divs: usize,
        num_meridional_divs: usize,
        restraint: bool,
        area_sec: &str,
    ) {
        self.points.clear();
        self.frames.clear();
        self.areas.clear();
        self.solids.clear();
        self.links.clear();
        self.case_results.clear();
        self.steel_design_results.clear();
        self.concrete_design_results.clear();
        self.is_locked = false;

        let n_theta = num_angular_divs.max(6);
        let n_phi = num_meridional_divs.max(2);
        let max_phi = roll_angle_deg.clamp(10.0, 90.0).to_radians();
        let z_base = radius * max_phi.cos();

        let a_sec = if self.area_sections.contains_key(area_sec) {
            area_sec.to_string()
        } else {
            self.area_sections
                .keys()
                .next()
                .cloned()
                .unwrap_or_else(|| "ASEC1".to_string())
        };

        let apex_pt = "1".to_string();
        self.add_point(&apex_pt, 0.0, 0.0, radius - z_base);
        let mut pt_id = 2usize;

        let mut rings = vec![vec![String::new(); n_theta]; n_phi];
        for ip in 0..n_phi {
            let phi = ((ip + 1) as f64 / (n_phi as f64)) * max_phi;
            let r_ring = radius * phi.sin();
            let z = radius * phi.cos() - z_base;
            for it in 0..n_theta {
                let theta = (it as f64 / (n_theta as f64)) * 2.0 * PI;
                let x = r_ring * theta.cos();
                let y = r_ring * theta.sin();
                let name = format!("{pt_id}");
                pt_id += 1;
                self.add_point(&name, x, y, z);
                if ip + 1 == n_phi && restraint {
                    if let Some(p) = self.points.get_mut(&name) {
                        p.restraint = [true, true, true, false, false, false];
                    }
                }
                rings[ip][it] = name;
            }
        }

        let mut ar_id = 1usize;
        // Top cap triangles around apex
        for it in 0..n_theta {
            let next_it = (it + 1) % n_theta;
            let name = format!("{ar_id}");
            ar_id += 1;
            self.add_area(
                &name,
                vec![
                    apex_pt.clone(),
                    rings[0][it].clone(),
                    rings[0][next_it].clone(),
                ],
                &a_sec,
            );
        }
        // Meridional quad rings
        for ip in 0..(n_phi - 1) {
            for it in 0..n_theta {
                let next_it = (it + 1) % n_theta;
                let name = format!("{ar_id}");
                ar_id += 1;
                self.add_area(
                    &name,
                    vec![
                        rings[ip][it].clone(),
                        rings[ip + 1][it].clone(),
                        rings[ip + 1][next_it].clone(),
                        rings[ip][next_it].clone(),
                    ],
                    &a_sec,
                );
            }
        }
        self.model_name = format!("Spherical Dome Shell (R={radius})");
    }
}

