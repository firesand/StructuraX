//! 3D Elastic Catenary Cable Element (CSI Chapter IX, Peyrot & Goulois 1979) and
//! Frame Tension/Compression Limit Constitutive Engine (CSI Chapter VII).
//!
//! This module provides:
//! 1. **3D Elastic Catenary Cable Element (`CableElement`, `CableState`)**:
//!    - Exact 2-node thermal-elastic catenary formulation (Peyrot & Goulois 1979) in the
//!      gravity/load plane $(\mathbf{e}_x, \mathbf{e}_z)$ with analytical $2 \times 2$
//!      symmetric positive-definite flexibility matrix $\mathbf{F}_{\text{flex}}(H, F_4)$,
//!      exact in-plane tangent stiffness $\mathbf{K}_{2\text{D}} = \mathbf{F}_{\text{flex}}^{-1}$,
//!      and out-of-plane geometric pendulum stiffness $K_{yy} = H / l_x$.
//!    - Seamless handling of vertical cables ($l_x \to 0$) across hanging-loop and
//!      taut-vertical regimes, as well as unloaded tension-only straight cables ($|\mathbf{w}| \to 0$).
//!    - Shape calculator constructors (`from_undeformed_length`, `from_horizontal_tension`,
//!      `from_relative_sag`) and a multi-element 3D Newton-Raphson equilibrium solver
//!      (`solve_cable_network`).
//! 2. **Frame Tension/Compression Limit Constitutive Helper (`tc_limits`)**:
//!    - Exact axial tension/compression cutoff (`tension_limit`, `compression_limit`) with
//!      small positive-definite tangent regularization for Newton-Raphson convergence.

use crate::linalg::solve_lu;
use crate::SolverError;

/// Definition of a 2-node 3D elastic catenary cable element between end `I` and end `J`.
#[derive(Debug, Clone, PartialEq)]
pub struct CableElement {
    /// Initial 3D coordinates of end `I` ($s = 0$).
    pub x_i_init: [f64; 3],
    /// Initial 3D coordinates of end `J` ($s = L_0$).
    pub x_j_init: [f64; 3],
    /// Cross-sectional area $A > 0$.
    pub area: f64,
    /// Effective Young's modulus $E > 0$.
    pub e: f64,
    /// Coefficient of thermal expansion $\alpha$.
    pub alpha: f64,
    /// Undeformed arc length $L_0 > 0$ at reference temperature ($\Delta T = 0, T = 0$).
    pub l0: f64,
    /// Uniform distributed load vector per unit undeformed length $\mathbf{w}$
    /// (e.g. $[0, 0, -w]$ for downward self-weight $w$ along $-Z$).
    pub w_vec: [f64; 3],
    /// Uniform temperature change $\Delta T$ relative to reference temperature.
    pub delta_t: f64,
}

/// Evaluated state of a 3D cable element at given end displacements $(\mathbf{u}_I, \mathbf{u}_J)$.
#[derive(Debug, Clone, PartialEq)]
pub struct CableState {
    /// Internal nodal force vector `[FX_I, FY_I, FZ_I]` at end `I` in global axes
    /// (also equals the support reaction exerted by a support at `I` on the cable).
    pub f_i: [f64; 3],
    /// Internal nodal force vector `[FX_J, FY_J, FZ_J]` at end `J` in global axes
    /// (also equals the support reaction exerted by a support at `J` on the cable).
    pub f_j: [f64; 3],
    /// $6 \times 6$ consistent tangent stiffness matrix in global coordinates
    /// for DOFs `[u_Ix, u_Iy, u_Iz, u_Jx, u_Jy, u_Jz]`.
    pub k_tangent: [[f64; 6]; 6],
    /// Tension at end `I`: $T_I = |\mathbf{f}_I|$.
    pub tension_i: f64,
    /// Tension at end `J`: $T_J = |\mathbf{f}_J|$.
    pub tension_j: f64,
    /// Horizontal component of tension $H \ge 0$ in the load plane.
    pub horizontal_tension: f64,
    /// Stretched deformed arc length $L_{\text{def}}$ of the cable.
    pub deformed_length: f64,
    /// Current chord length $|\mathbf{x}_J - \mathbf{x}_I|$.
    pub chord_length: f64,
}

impl CableElement {
    /// Create a cable element with explicitly specified undeformed length `l0`.
    pub fn from_undeformed_length(
        x_i_init: [f64; 3],
        x_j_init: [f64; 3],
        area: f64,
        e: f64,
        alpha: f64,
        l0: f64,
        w_vec: [f64; 3],
        delta_t: f64,
    ) -> Result<Self, SolverError> {
        if area <= 0.0
            || e <= 0.0
            || l0 <= 0.0
            || [area, e, alpha, l0, delta_t]
                .iter()
                .chain(x_i_init.iter())
                .chain(x_j_init.iter())
                .chain(w_vec.iter())
                .any(|x| !x.is_finite())
            || 1. + alpha * delta_t <= 0.
        {
            return Err(SolverError::InvalidInput(format!(
                "invalid cable parameters: area={area}, E={e}, L0={l0}"
            )));
        }
        Ok(Self {
            x_i_init,
            x_j_init,
            area,
            e,
            alpha,
            l0,
            w_vec,
            delta_t,
        })
    }

    /// Shape calculator: determine the undeformed length $L_0$ such that at the initial
    /// positions $(\mathbf{x}_I^{(0)}, \mathbf{x}_J^{(0)})$ under load `w_vec` and
    /// `delta_t = 0`, the horizontal tension component equals `h_target > 0`.
    pub fn from_horizontal_tension(
        x_i_init: [f64; 3],
        x_j_init: [f64; 3],
        area: f64,
        e: f64,
        alpha: f64,
        h_target: f64,
        w_vec: [f64; 3],
        delta_t: f64,
    ) -> Result<Self, SolverError> {
        if area <= 0.0
            || e <= 0.0
            || h_target <= 0.0
            || [area, e, alpha, h_target, delta_t]
                .iter()
                .chain(x_i_init.iter())
                .chain(x_j_init.iter())
                .chain(w_vec.iter())
                .any(|x| !x.is_finite())
        {
            return Err(SolverError::InvalidInput(format!(
                "invalid cable horizontal-tension parameters: area={area}, E={e}, H={h_target}"
            )));
        }
        let dx = [
            x_j_init[0] - x_i_init[0],
            x_j_init[1] - x_i_init[1],
            x_j_init[2] - x_i_init[2],
        ];
        let chord = norm3(dx);
        if chord <= 1e-12 {
            return Err(SolverError::InvalidInput(
                "zero-chord cable cannot be initialized from horizontal tension".to_string(),
            ));
        }
        let ea = e * area;
        let w = norm3(w_vec);
        if w <= 1e-12 {
            let l0 = chord / (1.0 + h_target / ea);
            return Self::from_undeformed_length(
                x_i_init, x_j_init, area, e, alpha, l0, w_vec, delta_t,
            );
        }
        let ez = [-w_vec[0] / w, -w_vec[1] / w, -w_vec[2] / w];
        let lz = dot3(dx, ez);
        let h_vec = [dx[0] - lz * ez[0], dx[1] - lz * ez[1], dx[2] - lz * ez[2]];
        let lx = norm3(h_vec);
        if lx <= 1e-12 {
            return Err(SolverError::InvalidInput(
                "vertical cable cannot be initialized from horizontal tension".to_string(),
            ));
        }

        // Solve for L0 > 0 by Newton-Raphson on H(L0) - h_target = 0
        let mut l0 = chord;
        for _ in 0..60 {
            let (_, f_j, _, _) = solve_catenary_2d(lx, lz, l0, ea, w, 1.0)?;
            let h_cur = f_j[0];
            let err = h_cur - h_target;
            if err.abs() <= 1e-12 * h_target {
                break;
            }
            let dl = (1e-7 * l0).max(1e-9);
            let (_, f_j_p, _, _) = solve_catenary_2d(lx, lz, l0 + dl, ea, w, 1.0)?;
            let dh_dl0 = (f_j_p[0] - h_cur) / dl;
            if dh_dl0.abs() <= 1e-30 {
                break;
            }
            let step = -err / dh_dl0;
            let mut damp = 1.0;
            while l0 + damp * step <= 0.1 * chord {
                damp *= 0.5;
            }
            l0 += damp * step;
        }
        let (_, f_j, _, _) = solve_catenary_2d(lx, lz, l0, ea, w, 1.0)?;
        let residual = (f_j[0] - h_target).abs();
        if !residual.is_finite() || residual > 1e-12 * h_target {
            return Err(SolverError::NonConvergence {
                case: "CableShape".into(),
                iterations: 60,
                residual,
                reason: "horizontal-tension initialization did not converge".into(),
            });
        }
        Self::from_undeformed_length(x_i_init, x_j_init, area, e, alpha, l0, w_vec, delta_t)
    }

    /// Shape calculator: determine the undeformed length $L_0$ from a specified
    /// parabolic/catenary vertical sag ratio `sag_ratio = f / l_x > 0` at mid-span
    /// ($H \approx w l_x / (8 \cdot \text{sag\_ratio})$).
    pub fn from_relative_sag(
        x_i_init: [f64; 3],
        x_j_init: [f64; 3],
        area: f64,
        e: f64,
        alpha: f64,
        sag_ratio: f64,
        w_vec: [f64; 3],
        delta_t: f64,
    ) -> Result<Self, SolverError> {
        if sag_ratio <= 0.0 {
            return Err(SolverError::InvalidInput(format!(
                "invalid relative sag ratio {sag_ratio}; must be > 0"
            )));
        }
        let w = norm3(w_vec);
        if w <= 1e-12 {
            return Err(SolverError::InvalidInput(
                "cannot initialize cable from relative sag when distributed weight is zero"
                    .to_string(),
            ));
        }
        let dx = [
            x_j_init[0] - x_i_init[0],
            x_j_init[1] - x_i_init[1],
            x_j_init[2] - x_i_init[2],
        ];
        let ez = [-w_vec[0] / w, -w_vec[1] / w, -w_vec[2] / w];
        let lz = dot3(dx, ez);
        let h_vec = [dx[0] - lz * ez[0], dx[1] - lz * ez[1], dx[2] - lz * ez[2]];
        let lx = norm3(h_vec);
        let h_est = w * lx / (8.0 * sag_ratio);
        Self::from_horizontal_tension(x_i_init, x_j_init, area, e, alpha, h_est, w_vec, delta_t)
    }
}

#[inline]
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn norm3(a: [f64; 3]) -> f64 {
    dot3(a, a).sqrt()
}

#[inline]
fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Construct an arbitrary unit vector orthogonal to unit vector `ez`.
fn any_perpendicular_unit(ez: [f64; 3]) -> [f64; 3] {
    let ref_axis = if ez[0].abs() < 0.8 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let c = cross3(ez, ref_axis);
    let n = norm3(c);
    [c[0] / n, c[1] / n, c[2] / n]
}

/// Solve the 2D thermal-elastic catenary compatibility equations (Peyrot & Goulois 1979)
/// in the load plane $(\mathbf{e}_x, \mathbf{e}_z)$ for horizontal span `lx > 0`,
/// vertical span `lz`, undeformed length `l0`, axial rigidity `ea`, downward load
/// per unit undeformed length `w > 0`, and thermal stretch factor `c_th = 1 + alpha * delta_t`.
///
/// Returns `(f_i_2d, f_j_2d, k_2d, deformed_length)` where:
/// - `f_i_2d = [-H, F2]` is the internal force at `I` in `(e_x, e_z)`
/// - `f_j_2d = [+H, F4]` is the internal force at `J` in `(e_x, e_z)`
/// - `k_2d = [[Kxx, Kxz], [Kxz, Kzz]]` is the exact $2 \times 2$ SPD in-plane tangent stiffness.
fn solve_catenary_2d(
    lx: f64,
    lz: f64,
    l0: f64,
    ea: f64,
    w: f64,
    c_th: f64,
) -> Result<([f64; 2], [f64; 2], [[f64; 2]; 2], f64), SolverError> {
    let total_w = w * l0;
    let chord = lx.hypot(lz);
    let s_th = (c_th * l0).max(1e-12 * l0);

    // Initial guess for (H, F4)
    let (mut h, mut f4) = if chord > s_th {
        // Stretched chord regime
        let t0 = ea * (chord / l0 - c_th).max(1e-6);
        let h_init = (t0 * (lx / chord)).max(1e-6 * total_w);
        let f4_init = t0 * (lz / chord) + 0.5 * total_w;
        (h_init, f4_init)
    } else {
        // Hanging catenary regime: approximate sag from arc-chord difference
        let slack_sq = (s_th * s_th - chord * chord).max(0.0);
        let sag_est = (0.5 * slack_sq.sqrt()).max(1e-4 * l0);
        let h_init = (w * lx * lx / (8.0 * sag_est)).max(1e-6 * total_w);
        let f4_init = 0.5 * total_w + h_init * (lz / lx);
        (h_init, f4_init)
    };

    // Avoid subtracting nearly equal tensions/inverse-hyperbolic sines for
    // highly prestressed cables. These are algebraically identical differences.
    let asinh_sum = |h: f64, f4: f64| {
        let f2 = total_w - f4;
        let ti = h.hypot(f2);
        let tj = h.hypot(f4);
        let dt = (f4 - f2) * total_w / (tj + ti);
        if f4 <= 0. {
            ((total_w - dt) / (-f4 + tj)).ln_1p()
        } else if f2 <= 0. {
            ((total_w + dt) / (-f2 + ti)).ln_1p()
        } else {
            (f4 / h).asinh() + (f2 / h).asinh()
        }
    };
    let tol = 1e-13 * l0.max(chord);
    let mut converged = false;

    for _iter in 0..120 {
        let f2 = total_w - f4;
        let t_i = h.hypot(f2);
        let t_j = h.hypot(f4);
        let asinh_total = asinh_sum(h, f4);

        let gx = h * (l0 / ea + (c_th / w) * asinh_total) - lx;
        let gz = (f4 - f2) * l0 / (2. * ea) + c_th * (f4 - f2) * l0 / (t_j + t_i) - lz;
        let res = gx.hypot(gz);
        if res <= tol {
            converged = true;
            break;
        }

        // Exact 2x2 symmetric positive definite flexibility matrix F_flex = d(lx, lz)/d(H, F4)
        let f_xx = l0 / ea + (c_th / w) * (asinh_total - f4 / t_j - f2 / t_i);
        let f_xz = -c_th * h * (f4 - f2) * l0 / ((t_j + t_i) * t_j * t_i);
        let f_zz = l0 / ea + (c_th / w) * (f4 / t_j + f2 / t_i);
        let det = f_xx * f_zz - f_xz * f_xz;
        if !det.is_finite() || det <= 1e-36 {
            break;
        }

        let dh = -(f_zz * gx - f_xz * gz) / det;
        let df4 = -(-f_xz * gx + f_xx * gz) / det;

        // Damped line search keeping H > 0 and decreasing residual norm
        let mut step = 1.0;
        while h + step * dh <= 0.1 * h {
            step *= 0.5;
        }
        for _ls in 0..20 {
            let h_try = h + step * dh;
            let f4_try = f4 + step * df4;
            let f2_try = total_w - f4_try;
            let ti_try = h_try.hypot(f2_try);
            let tj_try = h_try.hypot(f4_try);
            let gx_try = h_try * (l0 / ea + (c_th / w) * asinh_sum(h_try, f4_try)) - lx;
            let gz_try = (f4_try - f2_try) * l0 / (2. * ea)
                + c_th * (f4_try - f2_try) * l0 / (tj_try + ti_try)
                - lz;
            if gx_try.hypot(gz_try) < res || step <= 1e-4 {
                h = h_try;
                f4 = f4_try;
                break;
            }
            step *= 0.5;
        }
    }

    if !converged {
        let f2 = total_w - f4;
        let t_i = h.hypot(f2);
        let t_j = h.hypot(f4);
        let gx = h * (l0 / ea + (c_th / w) * asinh_sum(h, f4)) - lx;
        let gz = (f4 - f2) * l0 / (2. * ea) + c_th * (f4 - f2) * l0 / (t_j + t_i) - lz;
        if !gx.hypot(gz).is_finite() || gx.hypot(gz) > tol {
            return Err(SolverError::NonConvergence {
                case: "CableElement2D".to_string(),
                iterations: 120,
                residual: gx.hypot(gz),
                reason: "catenary flexibility compatibility iteration did not converge".to_string(),
            });
        }
    }

    let f2 = total_w - f4;
    let t_i = h.hypot(f2);
    let t_j = h.hypot(f4);
    let asinh_total = asinh_sum(h, f4);

    let f_xx = l0 / ea + (c_th / w) * (asinh_total - f4 / t_j - f2 / t_i);
    let f_xz = -c_th * h * (f4 - f2) * l0 / ((t_j + t_i) * t_j * t_i);
    let f_zz = l0 / ea + (c_th / w) * (f4 / t_j + f2 / t_i);
    let det = f_xx * f_zz - f_xz * f_xz;
    if !det.is_finite() || det <= 0. {
        return Err(SolverError::InvalidInput(
            "singular catenary flexibility".into(),
        ));
    }

    let k_xx = f_zz / det;
    let k_xz = -f_xz / det;
    let k_zz = f_xx / det;

    let l_def = c_th * l0 + (f2 * t_i + f4 * t_j + h * h * asinh_total) / (2.0 * ea * w);

    Ok(([-h, f2], [h, f4], [[k_xx, k_xz], [k_xz, k_zz]], l_def))
}

/// Evaluate the 3D internal nodal forces, $6 \times 6$ consistent tangent stiffness matrix,
/// end tensions, and deformed length of a [`CableElement`] at given joint displacements
/// `u_i` and `u_j`.
pub fn evaluate_cable_element(
    elem: &CableElement,
    u_i: [f64; 3],
    u_j: [f64; 3],
) -> Result<CableState, SolverError> {
    if elem.area <= 0.0
        || elem.e <= 0.0
        || elem.l0 <= 0.0
        || [elem.area, elem.e, elem.alpha, elem.l0, elem.delta_t]
            .iter()
            .chain(elem.x_i_init.iter())
            .chain(elem.x_j_init.iter())
            .chain(elem.w_vec.iter())
            .chain(u_i.iter())
            .chain(u_j.iter())
            .any(|x| !x.is_finite())
        || 1. + elem.alpha * elem.delta_t <= 0.
    {
        return Err(SolverError::InvalidInput(
            "cable element requires positive area, E, and L0".to_string(),
        ));
    }
    let x_i = [
        elem.x_i_init[0] + u_i[0],
        elem.x_i_init[1] + u_i[1],
        elem.x_i_init[2] + u_i[2],
    ];
    let x_j = [
        elem.x_j_init[0] + u_j[0],
        elem.x_j_init[1] + u_j[1],
        elem.x_j_init[2] + u_j[2],
    ];
    let dx = [x_j[0] - x_i[0], x_j[1] - x_i[1], x_j[2] - x_i[2]];
    let chord_length = norm3(dx);
    let ea = elem.e * elem.area;
    let c_th = 1.0 + elem.alpha * elem.delta_t;
    let w = norm3(elem.w_vec);

    // Case B: Unloaded straight tension-only cable (|w| <= 1e-12)
    if w <= 1e-12 {
        let l_th = (c_th * elem.l0).max(1e-12 * elem.l0);
        let k_elastic = ea / elem.l0;
        let k_reg = 1e-9 * k_elastic;
        if chord_length <= l_th || chord_length <= 1e-12 {
            // Slack cable: zero internal forces, small positive definite tangent regularization
            let mut k_tangent = [[0.0; 6]; 6];
            for d in 0..3 {
                k_tangent[d][d] = k_reg;
                k_tangent[d + 3][d + 3] = k_reg;
                k_tangent[d][d + 3] = -k_reg;
                k_tangent[d + 3][d] = -k_reg;
            }
            return Ok(CableState {
                f_i: [0.0; 3],
                f_j: [0.0; 3],
                k_tangent,
                tension_i: 0.0,
                tension_j: 0.0,
                horizontal_tension: 0.0,
                deformed_length: l_th,
                chord_length,
            });
        }
        let e_dir = [
            dx[0] / chord_length,
            dx[1] / chord_length,
            dx[2] / chord_length,
        ];
        let tension = k_elastic * (chord_length - l_th);
        let f_i = [
            -tension * e_dir[0],
            -tension * e_dir[1],
            -tension * e_dir[2],
        ];
        let f_j = [tension * e_dir[0], tension * e_dir[1], tension * e_dir[2]];
        let t_over_l = tension / chord_length;
        let mut k_3d = [[0.0; 3]; 3];
        for r in 0..3 {
            for c in 0..3 {
                let delta = if r == c { 1.0 } else { 0.0 };
                k_3d[r][c] =
                    k_elastic * e_dir[r] * e_dir[c] + t_over_l * (delta - e_dir[r] * e_dir[c]);
            }
        }
        let k_tangent = assemble_two_node_tangent(&k_3d);
        let h_horiz = tension * e_dir[0].hypot(e_dir[1]);
        return Ok(CableState {
            f_i,
            f_j,
            k_tangent,
            tension_i: tension,
            tension_j: tension,
            horizontal_tension: h_horiz,
            deformed_length: chord_length,
            chord_length,
        });
    }

    // Case A: Loaded 3D Elastic Catenary (|w| > 1e-12)
    let ez = [-elem.w_vec[0] / w, -elem.w_vec[1] / w, -elem.w_vec[2] / w];
    let lz = dot3(dx, ez);
    let h_vec = [dx[0] - lz * ez[0], dx[1] - lz * ez[1], dx[2] - lz * ez[2]];
    let lx = norm3(h_vec);

    let (ex, ey, f_i_loc, f_j_loc, k_loc_2d, k_yy, h_val, l_def) = if lx > 1e-11 * elem.l0 {
        let ex = [h_vec[0] / lx, h_vec[1] / lx, h_vec[2] / lx];
        let ey = cross3(ez, ex);
        let (f_i_2d, f_j_2d, k_2d, l_def) = solve_catenary_2d(lx, lz, elem.l0, ea, w, c_th)?;
        let h_val = f_j_2d[0];
        let k_yy = h_val / lx;
        (ex, ey, f_i_2d, f_j_2d, k_2d, k_yy, h_val, l_def)
    } else {
        // Purely vertical cable limit (lx -> 0, H = 0)
        let ex = any_perpendicular_unit(ez);
        let ey = cross3(ez, ex);
        let total_w = w * elem.l0;
        let l_lim = c_th * elem.l0 + total_w * elem.l0 / (2.0 * ea);
        let (f4, k_zz) = if lz.abs() <= l_lim {
            // Hanging vertical loop: both ends pull upward (F2 >= 0, F4 >= 0)
            let denom = 2.0 * elem.l0 * (1.0 / (2.0 * ea) + c_th / total_w);
            (0.5 * total_w + lz / denom, 1.0 / denom)
        } else if lz < -l_lim {
            // Taut vertical cable stretched downward
            let k_ax = ea / elem.l0;
            (0.5 * total_w + k_ax * (lz + c_th * elem.l0), k_ax)
        } else {
            // Taut vertical cable stretched upward
            let k_ax = ea / elem.l0;
            (0.5 * total_w + k_ax * (lz - c_th * elem.l0), k_ax)
        };
        let f2 = total_w - f4;
        let k_lat = (w).max(1e-6 * ea / elem.l0);
        let l_def = c_th * elem.l0 + (f2 * f2.abs() + f4 * f4.abs()) / (2.0 * ea * w);
        (
            ex,
            ey,
            [0.0, f2],
            [0.0, f4],
            [[k_lat, 0.0], [0.0, k_zz]],
            k_lat,
            0.0,
            l_def,
        )
    };

    // Transform local forces (f_x * ex + f_z * ez) to global Cartesian axes
    let mut f_i = [0.0; 3];
    let mut f_j = [0.0; 3];
    for r in 0..3 {
        f_i[r] = f_i_loc[0] * ex[r] + f_i_loc[1] * ez[r];
        f_j[r] = f_j_loc[0] * ex[r] + f_j_loc[1] * ez[r];
    }
    let tension_i = f_i_loc[0].hypot(f_i_loc[1]);
    let tension_j = f_j_loc[0].hypot(f_j_loc[1]);

    // Transform 3x3 local stiffness block Q * diag([K_2d, K_yy]) * Q^T to global
    // where columns of Q are [ex, ey, ez]
    let q_cols = [ex, ey, ez];
    let k_loc_3d = [
        [k_loc_2d[0][0], 0.0, k_loc_2d[0][1]],
        [0.0, k_yy, 0.0],
        [k_loc_2d[1][0], 0.0, k_loc_2d[1][1]],
    ];
    let mut k_3d = [[0.0; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            let mut s = 0.0;
            for a in 0..3 {
                for b in 0..3 {
                    s += q_cols[a][r] * k_loc_3d[a][b] * q_cols[b][c];
                }
            }
            k_3d[r][c] = s;
        }
    }
    let k_tangent = assemble_two_node_tangent(&k_3d);

    Ok(CableState {
        f_i,
        f_j,
        k_tangent,
        tension_i,
        tension_j,
        horizontal_tension: h_val,
        deformed_length: l_def,
        chord_length,
    })
}

/// Assemble a $3 \times 3$ relative-displacement stiffness block $\mathbf{K}_{\text{3D}}$ into
/// the $6 \times 6$ two-node tangent stiffness matrix $\begin{bmatrix} \mathbf{K} & -\mathbf{K} \\ -\mathbf{K} & \mathbf{K} \end{bmatrix}$.
fn assemble_two_node_tangent(k_3d: &[[f64; 3]; 3]) -> [[f64; 6]; 6] {
    let mut k_tan = [[0.0; 6]; 6];
    for r in 0..3 {
        for c in 0..3 {
            let v = k_3d[r][c];
            k_tan[r][c] = v;
            k_tan[r + 3][c + 3] = v;
            k_tan[r][c + 3] = -v;
            k_tan[r + 3][c] = -v;
        }
    }
    k_tan
}

/// Solve for the 3D nonlinear static equilibrium displacements `u` (one `[UX, UY, UZ]`
/// per node) of a network of [`CableElement`]s connected to `n_nodes` joints, given:
/// - `elements`: slice of `(node_i, node_j, CableElement)`
/// - `restraints`: slice of `[bool; 3]` per node (`true` = fixed DOF)
/// - `nodal_loads`: slice of `[f64; 3]` external concentrated loads per node
/// - `u_init`: optional initial displacement guess per node
pub fn solve_cable_network(
    n_nodes: usize,
    elements: &[(usize, usize, CableElement)],
    restraints: &[[bool; 3]],
    nodal_loads: &[[f64; 3]],
    u_init: Option<&[[f64; 3]]>,
    max_iter: usize,
    tol: f64,
) -> Result<(Vec<[f64; 3]>, Vec<CableState>), SolverError> {
    if max_iter == 0
        || !tol.is_finite()
        || tol <= 0.
        || restraints.len() != n_nodes
        || nodal_loads.len() != n_nodes
        || u_init.is_some_and(|u| u.len() != n_nodes)
        || nodal_loads
            .iter()
            .flatten()
            .chain(u_init.into_iter().flatten().flatten())
            .any(|x| !x.is_finite())
    {
        return Err(SolverError::InvalidInput("cable network needs finite loads/state, matching node counts and positive iteration controls".into()));
    }
    let mut u = match u_init {
        Some(init) => init.to_vec(),
        None => vec![[0.0; 3]; n_nodes],
    };

    let mut free_dofs = Vec::new();
    let mut dof_map = vec![[None; 3]; n_nodes];
    for i in 0..n_nodes {
        for d in 0..3 {
            if !restraints[i][d] {
                dof_map[i][d] = Some(free_dofs.len());
                free_dofs.push((i, d));
            }
        }
    }
    let nf = free_dofs.len();

    let eval_network =
        |u_cur: &[[f64; 3]]| -> Result<(Vec<f64>, Vec<f64>, Vec<CableState>), SolverError> {
            let mut r_free = vec![0.0; nf];
            let mut k_free = vec![0.0; nf * nf];
            let mut states = Vec::with_capacity(elements.len());

            for (i, d) in &free_dofs {
                let idx = dof_map[*i][*d].unwrap();
                r_free[idx] = -nodal_loads[*i][*d];
            }

            for &(ni, nj, ref elem) in elements {
                let st = evaluate_cable_element(elem, u_cur[ni], u_cur[nj])?;
                let node_pair = [ni, nj];
                let f_pair = [st.f_i, st.f_j];
                for a in 0..2 {
                    for d in 0..3 {
                        if let Some(gi) = dof_map[node_pair[a]][d] {
                            r_free[gi] += f_pair[a][d];
                        }
                    }
                }
                for a in 0..2 {
                    for da in 0..3 {
                        if let Some(gi) = dof_map[node_pair[a]][da] {
                            let row_local = 3 * a + da;
                            for b in 0..2 {
                                for db in 0..3 {
                                    if let Some(gj) = dof_map[node_pair[b]][db] {
                                        let col_local = 3 * b + db;
                                        k_free[gi * nf + gj] += st.k_tangent[row_local][col_local];
                                    }
                                }
                            }
                        }
                    }
                }
                states.push(st);
            }
            Ok((r_free, k_free, states))
        };

    if nf == 0 {
        let (_, _, states) = eval_network(&u)?;
        return Ok((u, states));
    }

    // Scale by the forces being balanced, before cancellation between cable
    // ends. Self-weight is inside element forces, not in nodal_loads.
    let force_scale = |states: &[CableState]| {
        let mut nodal = nodal_loads
            .iter()
            .map(|f| f.map(f64::abs))
            .collect::<Vec<_>>();
        for ((i, j, _), state) in elements.iter().zip(states) {
            for d in 0..3 {
                nodal[*i][d] += state.f_i[d].abs();
                nodal[*j][d] += state.f_j[d].abs();
            }
        }
        free_dofs
            .iter()
            .map(|&(i, d)| nodal[i][d])
            .fold(0., f64::max)
    };
    for iteration in 0..max_iter {
        let (r_free, k_free, states) = eval_network(&u)?;
        let res_norm = r_free.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !res_norm.is_finite() {
            return Err(SolverError::NonConvergence {
                case: "CableNetwork".into(),
                iterations: iteration,
                residual: res_norm,
                reason: "nonfinite cable network residual".into(),
            });
        }
        if res_norm <= tol * force_scale(&states) {
            return Ok((u, states));
        }

        let rhs: Vec<f64> = r_free.iter().map(|&v| -v).collect();
        let du = solve_lu(&k_free, &rhs, nf)?;

        // Backtracking line search on residual norm
        let mut alpha = 1.0;
        let mut u_next = u.clone();
        let mut accepted = false;
        for _ls in 0..40 {
            for (k, &(ni, d)) in free_dofs.iter().enumerate() {
                u_next[ni][d] = u[ni][d] + alpha * du[k];
            }
            if let Ok((r_try, _, _)) = eval_network(&u_next) {
                let res_try = r_try.iter().map(|v| v * v).sum::<f64>().sqrt();
                if res_try.is_finite() && res_try < res_norm {
                    accepted = true;
                    break;
                }
            }
            alpha *= 0.5;
        }
        if !accepted {
            return Err(SolverError::NonConvergence {
                case: "CableNetwork".into(),
                iterations: iteration,
                residual: res_norm,
                reason: "cable network line search did not decrease residual".into(),
            });
        }
        u = u_next;
    }

    let (r_free, _, states) = eval_network(&u)?;
    let res_norm = r_free.iter().map(|v| v * v).sum::<f64>().sqrt();
    if res_norm.is_finite() && res_norm <= tol * force_scale(&states) {
        return Ok((u, states));
    }
    Err(SolverError::NonConvergence {
        case: "CableNetwork".to_string(),
        iterations: max_iter,
        residual: res_norm,
        reason: "cable network Newton-Raphson did not reach tolerance".to_string(),
    })
}

/// Frame Tension/Compression (T/C) Limit constitutive helpers (CSI Chapter VII).
pub mod tc_limits {
    /// Axial tension and compression force limits for a Frame element (`FrameSection`).
    ///
    /// Sign convention: axial tension is positive ($P > 0$), axial compression is negative ($P < 0$).
    /// - `tension_limit`: maximum allowable tensile force $T_{\text{lim}} \ge 0$ (`Some(0.0)` = no tension allowed).
    /// - `compression_limit`: maximum allowable compressive force magnitude (`Some(0.0)` = no compression allowed;
    ///   either `-|C_lim|` or `+|C_lim|` is normalized to lower bound `-|C_lim|`).
    #[derive(Debug, Clone, Copy, PartialEq, Default)]
    pub struct FrameTcLimits {
        pub tension_limit: Option<f64>,
        pub compression_limit: Option<f64>,
    }

    /// Evaluate the clamped axial force $P$, consistent axial tangent stiffness $k_{T,\text{axial}}$,
    /// and active flag (`true` = elastic within limits, `false` = clamped at T/C limit) given:
    /// - `ea_over_l`: linear elastic axial stiffness $EA / L > 0$
    /// - `delta_u_axial`: axial elongation $u_{J,a} - u_{I,a}$
    /// - `p0_axial`: initial/thermal axial force offset $P_0$
    /// - `limits`: optional tension and compression limits
    /// - `eps_reg_scale`: small positive tangent regularization factor when clamped (e.g. `1e-9`)
    pub fn eval_frame_tc_axial(
        ea_over_l: f64,
        delta_u_axial: f64,
        p0_axial: f64,
        limits: FrameTcLimits,
        eps_reg_scale: f64,
    ) -> (f64, f64, bool) {
        let p_trial = ea_over_l * delta_u_axial + p0_axial;
        let k_reg = (eps_reg_scale.max(1e-12)) * ea_over_l;

        if let Some(t_lim) = limits.tension_limit {
            let p_max = t_lim.max(0.0);
            if p_trial > p_max {
                return (p_max, k_reg, false);
            }
        }
        if let Some(c_lim) = limits.compression_limit {
            let p_min = -c_lim.abs();
            if p_trial < p_min {
                return (p_min, k_reg, false);
            }
        }
        (p_trial, ea_over_l, true)
    }

    /// Apply Frame Tension/Compression limits directly to a local 12-DOF frame internal
    /// force vector `f_local` and $12 \times 12$ local stiffness matrix `k_local`
    /// (where DOF 0 is $u_{1,I}$ and DOF 6 is $u_{1,J}$, so axial force is $P = f_{\text{local}}[6] = -f_{\text{local}}[0]$).
    pub fn apply_frame_tc_limits_local(
        f_local: &mut [f64; 12],
        k_local: &mut [[f64; 12]; 12],
        limits: FrameTcLimits,
        eps_reg_scale: f64,
    ) -> bool {
        let ea_over_l = k_local[0][0];
        let p_trial = f_local[6];
        let (p_clamped, k_ax, active) =
            eval_frame_tc_axial(ea_over_l, 0.0, p_trial, limits, eps_reg_scale);
        if !active {
            f_local[0] = -p_clamped;
            f_local[6] = p_clamped;
            k_local[0][0] = k_ax;
            k_local[6][6] = k_ax;
            k_local[0][6] = -k_ax;
            k_local[6][0] = -k_ax;
        }
        active
    }
}

pub use tc_limits::{apply_frame_tc_limits_local, eval_frame_tc_axial, FrameTcLimits};

#[cfg(test)]
mod tests {
    use super::*;

    /// SAP2000 Verification Example 7-001:
    /// Single Catenary Cable (`L0 = 100 ft`, `A = 1 in^2`, `E = 30,000,000 lb/in^2`,
    /// `w = 1 lb/ft`, `alpha = 6.5e-6 /F`, `delta_T = +100 F`) with top end `I` at `(0, 0, 90)`
    /// and lower end `J` at `(x_low, 0, 30)` for six spans `C1..C6` (`x_low = 100, 80, 60, 40, 20, 0 ft`).
    #[test]
    fn sap2000_example_7_001_catenary_cables_c1_through_c6() {
        // Expected lower-support reactions (R_Jx, R_Jz) in lb from SAP2000 Example 7-001 table:
        // C1 (x=100): (4,258,491, -2,555,045), rel tol 1e-4
        // C2 (x=80):  (504.10, -328.87)
        // C3 (x=60):  (22.15,  +15.73)
        // C4 (x=40):  (9.17,   +19.24)
        // C5 (x=20):  (3.06,   +19.93)
        // C6 (x=0):   (0.00,   +20.02)
        let cases: [(f64, f64, f64, f64); 6] = [
            (100.0, 4_258_491.0, -2_555_045.0, 50.0),
            (80.0, 504.10, -328.87, 0.05),
            (60.0, 22.15, 15.73, 0.02),
            (40.0, 9.17, 19.24, 0.02),
            (20.0, 3.06, 19.93, 0.02),
            (0.0, 0.00, 20.02, 0.02),
        ];

        for &(x_low, exp_rx, exp_rz, tol) in &cases {
            let elem = CableElement::from_undeformed_length(
                [0.0, 0.0, 90.0],
                [x_low, 0.0, 30.0],
                1.0,
                30.0e6,
                6.5e-6,
                100.0,
                [0.0, 0.0, -1.0],
                100.0,
            )
            .unwrap();

            let state = evaluate_cable_element(&elem, [0.0; 3], [0.0; 3]).unwrap();
            let rx_j = state.f_j[0];
            let rz_j = state.f_j[2];

            assert!(
                (rx_j - exp_rx).abs() <= tol,
                "x_low={x_low}: Rx_J got {rx_j:.4}, expected {exp_rx:.4}"
            );
            assert!(
                (rz_j - exp_rz).abs() <= tol,
                "x_low={x_low}: Rz_J got {rz_j:.4}, expected {exp_rz:.4}"
            );

            // Global equilibrium check: f_I + f_J + w_vec * L0 == 0
            assert!((state.f_i[0] + state.f_j[0]).abs() < 1e-8);
            assert!((state.f_i[1] + state.f_j[1]).abs() < 1e-8);
            assert!((state.f_i[2] + state.f_j[2] - 100.0).abs() < 1e-8);

            // Verify tangent stiffness symmetry
            for i in 0..6 {
                for j in 0..6 {
                    assert!(
                        (state.k_tangent[i][j] - state.k_tangent[j][i]).abs()
                            <= 1e-9 * state.k_tangent[i][i].abs().max(1.0)
                    );
                }
            }
        }
    }

    /// SAP2000 Verification Example 7-002 (Tibert 1999):
    /// Two-Segment Suspended Cable under Self-Weight and Concentrated Point Load (`8 kips` downward at Joint 2).
    /// - Support Joint 1: `(0, 0, 0) ft`, Support Joint 3: `(1000, 0, 0) ft`
    /// - Initial Joint 2 position: `(400, 0, -150) ft` (arbitrary initial Z per SAP2000 verification problem)
    /// - Cable properties: `A = 0.85 in^2`, `E = 19,000 kips/in^2`, `w = 0.00316 kips/ft`
    /// - Undeformed lengths: `L0_12 = 412.8837 ft`, `L0_23 = 613.0422 ft`
    /// - Expected Point 2 displacement from self-weight state to self-weight + 8 kip load state:
    ///   `Ux = -2.819 ft`, `Uz = -18.457 ft`.
    #[test]
    fn sap2000_example_7_002_two_segment_cable_concentrated_load() {
        let x1 = [0.0, 0.0, 0.0];
        let x2_init = [400.0, 0.0, -150.0];
        let x3 = [1000.0, 0.0, 0.0];

        let area = 0.85;
        let e = 19_000.0; // kips/in^2
        let w = 0.00316; // kips/ft

        let seg_12 = CableElement::from_undeformed_length(
            x1,
            x2_init,
            area,
            e,
            0.0,
            412.8837,
            [0.0, 0.0, -w],
            0.0,
        )
        .unwrap();
        let seg_23 = CableElement::from_undeformed_length(
            x2_init,
            x3,
            area,
            e,
            0.0,
            613.0422,
            [0.0, 0.0, -w],
            0.0,
        )
        .unwrap();

        let elements = vec![(0, 1, seg_12), (1, 2, seg_23)];
        // Fix Joint 1 (node 0), Joint 3 (node 2), and out-of-plane Y of Joint 2 (node 1)
        let restraints = [[true, true, true], [false, true, false], [true, true, true]];

        // Step 1: Self-weight equilibrium
        let loads_sw = [[0.0; 3], [0.0, 0.0, 0.0], [0.0; 3]];
        let (u_sw, _) =
            solve_cable_network(3, &elements, &restraints, &loads_sw, None, 60, 1e-10).unwrap();

        // Step 2: Self-weight + 8 kips downward load at Joint 2, continuing from self-weight state
        let loads_pt = [[0.0; 3], [0.0, 0.0, -8.0], [0.0; 3]];
        let (u_pt, _) =
            solve_cable_network(3, &elements, &restraints, &loads_pt, Some(&u_sw), 60, 1e-10)
                .unwrap();

        let disp_ux = u_pt[1][0] - u_sw[1][0];
        let disp_uz = u_pt[1][2] - u_sw[1][2];

        // Compare against SAP2000 & Tibert (1999) exact benchmark: Ux = -2.819 ft, Uz = -18.457 ft
        assert!(
            (disp_ux - (-2.819439)).abs() < 5e-4,
            "Point 2 Ux = {disp_ux:.6}, expected -2.819"
        );
        assert!(
            (disp_uz - (-18.456747)).abs() < 5e-4,
            "Point 2 Uz = {disp_uz:.6}, expected -18.457"
        );
    }

    #[test]
    fn frame_tension_compression_limits_constitutive_check() {
        let ea_l = 1000.0;
        // Tension-only brace (compression_limit = Some(0.0), tension_limit = None)
        let t_only = FrameTcLimits {
            tension_limit: None,
            compression_limit: Some(0.0),
        };
        let (p_tens, k_tens, act_t) = eval_frame_tc_axial(ea_l, 0.05, 0.0, t_only, 1e-9);
        assert!(act_t);
        assert!((p_tens - 50.0).abs() < 1e-12);
        assert!((k_tens - ea_l).abs() < 1e-12);

        let (p_comp, k_comp, act_c) = eval_frame_tc_axial(ea_l, -0.05, 0.0, t_only, 1e-9);
        assert!(!act_c);
        assert_eq!(p_comp, 0.0);
        assert!((k_comp - 1e-6).abs() < 1e-15);

        // Bounded tension (+30) and compression (-20)
        let bounded = FrameTcLimits {
            tension_limit: Some(30.0),
            compression_limit: Some(-20.0),
        };
        let (p_hi, _, act_hi) = eval_frame_tc_axial(ea_l, 0.05, 0.0, bounded, 1e-9);
        assert!(!act_hi);
        assert!((p_hi - 30.0).abs() < 1e-12);

        let (p_lo, _, act_lo) = eval_frame_tc_axial(ea_l, -0.05, 0.0, bounded, 1e-9);
        assert!(!act_lo);
        assert!((p_lo - (-20.0)).abs() < 1e-12);
    }
}
