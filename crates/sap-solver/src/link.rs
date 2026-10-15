//! Six-spring links using the CSI internal deformation operator B (manual pp.
//! 255, 261-263, 267, 271-272). Stiffness and viscous damping use B^T D B.
use crate::{
    frame::{frame_local_axes, mat3_mul},
    SolverError,
};
use indexmap::IndexMap;
use sap_model::{LinkObj, LinkProp, PointObj, SapModel};

pub(crate) struct LinkOperator {
    pub dofs: Vec<usize>,
    pub b: Vec<Vec<f64>>,
}

pub(crate) fn operator(
    link: &LinkObj,
    prop: &LinkProp,
    points: &IndexMap<String, PointObj>,
    indices: &IndexMap<String, usize>,
    rotations: &[[[f64; 3]; 3]],
) -> LinkOperator {
    let pi = &points[&link.point_i];
    let i = indices[&link.point_i];
    let (length, axes) = if let Some(name) = &link.point_j {
        let pj = &points[name];
        let length = ((pj.x - pi.x).powi(2) + (pj.y - pi.y).powi(2) + (pj.z - pi.z).powi(2)).sqrt();
        if length > 1e-12 {
            frame_local_axes([pi.x, pi.y, pi.z], [pj.x, pj.y, pj.z], link.angle_deg)
        } else {
            (
                0.,
                frame_local_axes([0.; 3], [0., 0., 1.], link.angle_deg).1,
            )
        }
    } else {
        (
            0.,
            frame_local_axes([0.; 3], [0., 0., 1.], link.angle_deg).1,
        )
    };
    let mut local = [[0.; 12]; 6];
    for d in 0..6 {
        local[d][d] = -1.;
        local[d][6 + d] = 1.;
    }
    // CSI dr2 = r2i-r2j, unlike dr1 and dr3.
    local[4][4] = 1.;
    local[4][10] = -1.;
    local[1][5] = -(length - prop.dj2);
    local[1][11] = -prop.dj2;
    local[2][4] = length - prop.dj3;
    local[2][10] = prop.dj3;
    let nodes = if let Some(name) = &link.point_j {
        vec![(i, 0), (indices[name], 6)]
    } else {
        vec![(i, 6)]
    }; // single-joint object is the J end; I is ground
    let dofs: Vec<usize> = nodes
        .iter()
        .flat_map(|(p, _)| (0..6).map(move |d| 6 * p + d))
        .collect();
    let mut b = vec![vec![0.; dofs.len()]; 6];
    for (node, (index, offset)) in nodes.iter().enumerate() {
        let t = mat3_mul(&axes, &rotations[*index]);
        for d in 0..6 {
            for c in 0..6 {
                b[d][node * 6 + c] = (0..3)
                    .map(|r| local[d][offset + 3 * (c / 3) + r] * t[r][c % 3])
                    .sum();
            }
        }
    }
    LinkOperator { dofs, b }
}

pub(crate) fn stiffness(prop: &LinkProp) -> [f64; 6] {
    std::array::from_fn(|d| {
        if prop.dof_fixed[d] {
            1e10
        } else if prop.dof_active[d] {
            prop.ke[d]
        } else {
            0.
        }
    })
}

pub(crate) fn assemble(matrix: &mut [f64], ndof: usize, op: &LinkOperator, diagonal: [f64; 6]) {
    for (d, coefficient) in diagonal.into_iter().enumerate() {
        for (a, i) in op.dofs.iter().enumerate() {
            for (b, j) in op.dofs.iter().enumerate() {
                matrix[i * ndof + j] += coefficient * op.b[d][a] * op.b[d][b];
            }
        }
    }
}

pub(crate) fn damping(
    model: &SapModel,
    points: &IndexMap<String, PointObj>,
    indices: &IndexMap<String, usize>,
    rotations: &[[[f64; 3]; 3]],
    ndof: usize,
) -> Vec<f64> {
    let mut c = vec![0.; ndof * ndof];
    for link in model.links.values() {
        let p = &model.link_props[&link.prop];
        let diagonal = std::array::from_fn(|d| {
            if p.dof_active[d] && !p.dof_fixed[d] {
                p.ce[d]
            } else {
                0.
            }
        });
        assemble(
            &mut c,
            ndof,
            &operator(link, p, points, indices, rotations),
            diagonal,
        );
    }
    c
}

fn spring_forces(
    prop: &LinkProp,
    op: &LinkOperator,
    u: &[f64],
    velocity: Option<&[f64]>,
    nonlinear: bool,
) -> [f64; 6] {
    let effective = stiffness(prop);
    std::array::from_fn(|d| {
        let deformation: f64 = op.dofs.iter().zip(&op.b[d]).map(|(i, b)| u[*i] * b).sum();
        if nonlinear && prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d] {
            crate::nonlinear::law(prop.link_type, prop.k_nl[d], prop.param1[d], deformation).0
        } else {
            let speed: f64 = velocity.map_or(0., |v| {
                op.dofs.iter().zip(&op.b[d]).map(|(i, b)| v[*i] * b).sum()
            });
            effective[d] * deformation
                + if !nonlinear && prop.dof_active[d] && !prop.dof_fixed[d] {
                    prop.ce[d] * speed
                } else {
                    0.
                }
        }
    })
}

pub(crate) fn section_at_j(prop: &LinkProp, mut forces: [f64; 6]) -> [f64; 6] {
    forces[4] -= prop.dj3 * forces[2];
    forces[5] -= prop.dj2 * forces[1];
    forces
}

/// Constitutive snapshot forces before conversion to J-end section moments.
/// Nonlinear DOFs replace ke exactly; ce acts through the same viscous matrix
/// used by the stepper. Rayleigh stiffness damping uses only the linear base.
pub(crate) fn dynamic_spring_forces(
    prop: &LinkProp,
    op: &LinkOperator,
    u: &[f64],
    velocity: &[f64],
    nonlinear: &[Option<f64>; 6],
    beta: f64,
) -> [f64; 6] {
    let k = stiffness(prop);
    std::array::from_fn(|d| {
        let displacement: f64 = op.dofs.iter().zip(&op.b[d]).map(|(i, b)| u[*i] * b).sum();
        let speed: f64 = op
            .dofs
            .iter()
            .zip(&op.b[d])
            .map(|(i, b)| velocity[*i] * b)
            .sum();
        let viscous = if prop.dof_active[d] && !prop.dof_fixed[d] {
            prop.ce[d] * speed
        } else {
            0.
        };
        nonlinear[d].unwrap_or(k[d] * displacement)
            + viscous
            + if nonlinear[d].is_none() {
                beta * k[d] * speed
            } else {
                0.
            }
    })
}

pub(crate) fn forces_for_case(
    prop: &LinkProp,
    op: &LinkOperator,
    u: &[f64],
    nonlinear: bool,
) -> [f64; 6] {
    section_at_j(prop, spring_forces(prop, op, u, None, nonlinear))
}

/// Support force applied by one-joint grounded links, expressed at the modeled
/// joint. Include these in base reactions even when that joint is unrestrained.
pub(crate) fn grounded_reactions(
    model: &SapModel,
    points: &IndexMap<String, PointObj>,
    indices: &IndexMap<String, usize>,
    rotations: &[[[f64; 3]; 3]],
    u: &[f64],
    dynamics: Option<(&[f64], f64)>,
    nonlinear: bool,
) -> IndexMap<String, [f64; 6]> {
    let mut result: IndexMap<String, [f64; 6]> = IndexMap::new();
    for object in model
        .links
        .values()
        .filter(|object| object.point_j.is_none())
    {
        let prop = &model.link_props[&object.prop];
        let op = operator(object, prop, points, indices, rotations);
        let mut force = spring_forces(prop, &op, u, dynamics.map(|(v, _)| v), nonlinear);
        if let Some((velocity, beta)) = dynamics {
            let k = stiffness(prop);
            for d in 0..6 {
                force[d] += beta
                    * k[d]
                    * op.dofs
                        .iter()
                        .zip(&op.b[d])
                        .map(|(i, b)| velocity[*i] * b)
                        .sum::<f64>();
            }
        }
        let reaction = result.entry(object.point_i.clone()).or_insert([0.; 6]);
        for (c, value) in reaction.iter_mut().enumerate() {
            *value -= (0..6).map(|d| op.b[d][c] * force[d]).sum::<f64>();
        }
    }
    result
}

pub(crate) fn validate(model: &SapModel) -> Result<(), SolverError> {
    for link in model.links.values() {
        let prop = model.link_props.get(&link.prop).ok_or_else(|| {
            SolverError::InvalidInput(format!(
                "link '{}' references missing property '{}'",
                link.name, link.prop
            ))
        })?;
        for name in std::iter::once(&link.point_i).chain(link.point_j.iter()) {
            let point = model
                .points
                .get(name)
                .ok_or_else(|| SolverError::PointNotFound(name.clone()))?;
            if [point.x, point.y, point.z].iter().any(|x| !x.is_finite()) {
                return Err(SolverError::InvalidInput(format!(
                    "link '{}' has nonfinite geometry",
                    link.name
                )));
            }
        }
        if !link.angle_deg.is_finite()
            || [prop.dj2, prop.dj3, prop.mass, prop.weight]
                .iter()
                .any(|x| !x.is_finite())
            || prop.mass < 0.
            || prop
                .ke
                .iter()
                .chain(&prop.ce)
                .any(|x| !x.is_finite() || *x < 0.)
        {
            return Err(SolverError::InvalidInput(format!(
                "link '{}' needs finite geometry, nonnegative stiffness, damping and mass",
                link.name
            )));
        }
    }
    Ok(())
}
