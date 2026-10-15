//! Named Cartesian load bases. Vectors rotate; coordinate-system origins do not
//! translate a force applied at a specified joint or a distributed load location.
use crate::{
    frame::{frame_local_axes, mat3_mul},
    SolverError,
};
use sap_model::{CoordSystem, SapModel};
use std::collections::BTreeSet;

type Rotation = [[f64; 3]; 3];

/// Intrinsic right-handed Z, Y, X rotations, with columns expressed globally.
fn rotation(system: &CoordSystem) -> Result<Rotation, SolverError> {
    if system
        .angles_deg
        .iter()
        .chain(&system.origin)
        .any(|x| !x.is_finite())
    {
        return Err(SolverError::InvalidInput(format!(
            "coordinate system '{}' has nonfinite data",
            system.name
        )));
    }
    let [z, y, x] = system.angles_deg.map(f64::to_radians);
    let (sz, cz) = z.sin_cos();
    let (sy, cy) = y.sin_cos();
    let (sx, cx) = x.sin_cos();
    Ok(mat3_mul(
        &mat3_mul(
            &[[cz, -sz, 0.], [sz, cz, 0.], [0., 0., 1.]],
            &[[cy, 0., sy], [0., 1., 0.], [-sy, 0., cy]],
        ),
        &[[1., 0., 0.], [0., cx, -sx], [0., sx, cx]],
    ))
}

fn rotate6(r: Rotation, vector: [f64; 6]) -> [f64; 6] {
    std::array::from_fn(|i| (0..3).map(|j| r[i % 3][j] * vector[3 * (i / 3) + j]).sum())
}

pub(crate) fn normalize_loads(model: &mut SapModel) -> Result<(), SolverError> {
    let used: BTreeSet<String> = model
        .load_cases
        .values()
        .filter(|c| c.run_flag)
        .flat_map(|c| {
            c.loads
                .iter()
                .filter(|l| l.scale_factor != 0.)
                .map(|l| l.load_name.clone())
                .chain(
                    c.history_loads
                        .iter()
                        .filter(|(_, _, s)| *s != 0.)
                        .map(|(_, p, _)| p.clone()),
                )
        })
        .collect();
    let systems = &model.coord_systems;
    let get = |name: &str, pattern: &str| -> Result<Option<Rotation>, SolverError> {
        if !used.contains(pattern)
            || name.eq_ignore_ascii_case("GLOBAL")
            || name.eq_ignore_ascii_case("LOCAL")
        {
            return Ok(None);
        }
        let system = systems.get(name).ok_or_else(|| {
            SolverError::InvalidInput(format!("missing load coordinate system '{name}'"))
        })?;
        Ok(Some(rotation(system)?))
    };
    for point in model.points.values_mut() {
        for load in &mut point.loads_force {
            if let Some(r) = get(&load.csys, &load.load_pat)? {
                load.force = rotate6(r, load.force);
                load.csys = "GLOBAL".into();
            }
        }
        for load in &mut point.loads_displ {
            if let Some(r) = get(&load.csys, &load.load_pat)? {
                load.displ = rotate6(r, load.displ);
                load.csys = "GLOBAL".into();
            }
        }
    }
    for frame in model.frames.values_mut() {
        let i = model
            .points
            .get(&frame.point_i)
            .ok_or_else(|| SolverError::PointNotFound(frame.point_i.clone()))?;
        let j = model
            .points
            .get(&frame.point_j)
            .ok_or_else(|| SolverError::PointNotFound(frame.point_j.clone()))?;
        let (_, axes) = frame_local_axes([i.x, i.y, i.z], [j.x, j.y, j.z], frame.angle_deg);
        let mut dist = Vec::new();
        for load in &frame.loads_dist {
            if let Some(r) = get(&load.csys, &load.load_pat)? {
                if let Some(vector) = direction(load.dir, r, Some(axes[0]), load.my_type == 2)? {
                    for (axis, component) in vector.into_iter().enumerate() {
                        let mut split = load.clone();
                        split.csys = "GLOBAL".into();
                        split.dir = 4 + axis as i32;
                        split.val1 *= component;
                        split.val2 *= component;
                        dist.push(split);
                    }
                } else {
                    let mut l = load.clone();
                    l.csys = "GLOBAL".into();
                    dist.push(l);
                }
            } else {
                dist.push(load.clone());
            }
        }
        frame.loads_dist = dist;
        let mut points = Vec::new();
        for load in &frame.loads_point {
            if let Some(r) = get(&load.csys, &load.load_pat)? {
                if let Some(vector) = direction(load.dir, r, None, false)? {
                    for (axis, component) in vector.into_iter().enumerate() {
                        let mut split = load.clone();
                        split.csys = "GLOBAL".into();
                        split.dir = 4 + axis as i32;
                        split.val *= component;
                        points.push(split);
                    }
                } else {
                    let mut l = load.clone();
                    l.csys = "GLOBAL".into();
                    points.push(l);
                }
            } else {
                points.push(load.clone());
            }
        }
        frame.loads_point = points;
    }
    for area in model.areas.values_mut() {
        let mut loads = Vec::new();
        for load in &area.loads_uniform {
            if let Some(r) = get(&load.csys, &load.load_pat)? {
                if (7..=9).contains(&load.dir) || load.dir == 11 {
                    return Err(SolverError::UnsupportedInput("projected uniform area loads require surface projection, which is not implemented".into()));
                }
                if let Some(vector) = direction(load.dir, r, None, false)? {
                    for (axis, component) in vector.into_iter().enumerate() {
                        let mut split = load.clone();
                        split.csys = "GLOBAL".into();
                        split.dir = 4 + axis as i32;
                        split.val *= component;
                        loads.push(split);
                    }
                } else {
                    let mut l = load.clone();
                    l.csys = "GLOBAL".into();
                    loads.push(l);
                }
            } else {
                loads.push(load.clone());
            }
        }
        area.loads_uniform = loads;
    }
    Ok(())
}

fn direction(
    dir: i32,
    r: Rotation,
    line: Option<[f64; 3]>,
    moment: bool,
) -> Result<Option<[f64; 3]>, SolverError> {
    // CSI 1..3 are element-local even when a named basis is supplied;
    // gravity always remains global -Z. Only 4..9 refer to the named basis.
    match dir {
        1..=3 | 10..=11 => Ok(None),
        4..=9 => {
            let axis = ((dir - 4) % 3) as usize;
            let mut v = [r[0][axis], r[1][axis], r[2][axis]];
            if dir >= 7 {
                if let Some(e1) = line {
                    let dot: f64 = e1.iter().zip(v).map(|(a, b)| a * b).sum();
                    let factor = if moment {
                        dot.abs()
                    } else {
                        (1. - dot * dot).max(0.).sqrt()
                    };
                    for x in &mut v {
                        *x *= factor;
                    }
                }
            }
            Ok(Some(v))
        }
        _ => Err(SolverError::InvalidInput(format!(
            "invalid load direction {dir}"
        ))),
    }
}
