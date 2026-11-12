//! Typed native link editing. Full vendor COM method coverage is not implied.
use super::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::{LinkObj, LinkProp, LinkPropType, SapModel};

/// Validate the represented linear/Damper/Gap/Hook/plastic Wen subset before changing a model.
pub fn set_link_property(model: &mut SapModel, mut property: LinkProp) -> Result<(), String> {
    if model.is_locked {
        return Err("Model is locked.".into());
    }
    if !super::is_valid_name(&property.name) {
        return Err(
            "Provide a nonempty, trimmed property name without quotes or control characters."
                .into(),
        );
    }
    if !matches!(
        property.link_type,
        LinkPropType::Linear
            | LinkPropType::Damper
            | LinkPropType::Gap
            | LinkPropType::Hook
            | LinkPropType::PlasticWen
    ) {
        return Err(
            "This editor supports linear, damper, gap, hook and plastic Wen link properties only."
                .into(),
        );
    }
    let wen = property.link_type == LinkPropType::PlasticWen;
    let valid_param2 = if property.link_type == LinkPropType::Damper {
        property
            .param2
            .iter()
            .all(|v| v.is_finite() && (*v == 0.0 || (*v - 1.0).abs() <= 1e-9))
    } else if wen {
        // Post-yield stiffness ratio.
        property.param2.iter().all(|v| (0.0..=1.0).contains(v))
    } else {
        property.param2.iter().all(|v| v.is_finite() && *v == 0.0)
    };
    // The third parameter row is the Wen yield exponent; other laws have none.
    let valid_param3 = property
        .param3
        .iter()
        .all(|v| *v == 0.0 || (wen && (1.0..=20.0).contains(v)));
    if !valid_param3 {
        return Err("The third nonlinear parameter is the plastic Wen yield exponent: 1 to 20 for a Wen DOF, zero otherwise.".into());
    }
    if property
        .ke
        .iter()
        .chain(&property.ce)
        .chain(&property.k_nl)
        .chain(&property.param1)
        .any(|v| !v.is_finite() || *v < 0.)
        || !valid_param2
        || [property.dj2, property.dj3, property.mass, property.weight]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.)
    {
        return Err("Stiffness, damping, opening, distances, mass and weight must be finite and nonnegative; damper exponents must be linear (1.0) and post-yield ratios are not supported for these laws.".into());
    }
    for d in 0..6 {
        if property.dof_fixed[d] && !property.dof_active[d] {
            return Err("A fixed DOF must be active.".into());
        }
        if property.dof_nonlinear[d]
            && (!property.dof_active[d]
                || property.dof_fixed[d]
                || property.link_type == LinkPropType::Linear)
        {
            return Err(
                "Nonlinear flags require an active, unfixed Damper/Gap/Hook/plastic Wen DOF."
                    .into(),
            );
        }
        if wen
            && property.dof_nonlinear[d]
            && (property.k_nl[d] <= 0.0 || property.param1[d] <= 0.0 || property.param3[d] < 1.0)
        {
            return Err("A nonlinear plastic Wen DOF needs positive stiffness k_nl, positive yield force (param1), post-yield ratio in [0, 1] (param2) and yield exponent in [1, 20] (param3).".into());
        }
        if property.link_type == LinkPropType::Damper && property.dof_nonlinear[d] {
            if property.k_nl[d] <= 0.0 {
                return Err(
                    "Active nonlinear Maxwell damper DOFs require positive series stiffness k_nl."
                        .into(),
                );
            }
            property.param2[d] = 1.0;
        }
    }
    if property.link_type != LinkPropType::Linear && !property.nonlinear_data_complete {
        return Err(
            "Source nonlinear parameters are incomplete; supply a fully specified property.".into(),
        );
    }
    model.link_props.insert(property.name.clone(), property);
    model.invalidate_results();
    Ok(())
}
/// Add or replace a fully specified link object. One-joint links use I as the active joint; the other end is grounded.
pub fn set_link_object(model: &mut SapModel, link: LinkObj) -> Result<(), String> {
    if model.is_locked {
        return Err("Model is locked.".into());
    }
    if !super::is_valid_name(&link.name) || !link.angle_deg.is_finite() {
        return Err("Provide a valid name and finite roll angle.".into());
    }
    if !model.link_props.contains_key(&link.prop) {
        return Err("Link property does not exist.".into());
    }
    if link.point_j.as_ref() == Some(&link.point_i) {
        return Err(
            "Use distinct point identities for a two-joint link, or omit J for a grounded link."
                .into(),
        );
    }
    for name in std::iter::once(&link.point_i).chain(link.point_j.as_ref()) {
        let point = model
            .points
            .get(name)
            .ok_or_else(|| format!("Missing joint '{name}'."))?;
        if [point.x, point.y, point.z].iter().any(|v| !v.is_finite()) {
            return Err("Link joints need finite coordinates.".into());
        }
    }
    model.links.insert(link.name.clone(), link);
    model.invalidate_results();
    Ok(())
}
pub struct cPropLink<'a> {
    pub(crate) api: &'a mut cSapModel,
}
impl cPropLink<'_> {
    pub fn set_definition(&mut self, property: LinkProp) -> i32 {
        if set_link_property(&mut self.api.inner, property).is_ok() {
            OAPI_OK
        } else {
            OAPI_ERR
        }
    }
    pub fn get_definition(&self, name: &str) -> Option<LinkProp> {
        self.api.inner.link_props.get(name).cloned()
    }
}
pub struct cLinkObj<'a> {
    pub(crate) api: &'a mut cSapModel,
}
impl cLinkObj<'_> {
    pub fn set_definition(&mut self, link: LinkObj) -> i32 {
        if set_link_object(&mut self.api.inner, link).is_ok() {
            OAPI_OK
        } else {
            OAPI_ERR
        }
    }
    pub fn get_definition(&self, name: &str) -> Option<LinkObj> {
        self.api.inner.links.get(name).cloned()
    }
    pub fn add_by_point(
        &mut self,
        point_i: &str,
        point_j: Option<&str>,
        property: &str,
        user_name: &str,
    ) -> (i32, String) {
        if !super::is_valid_optional_user_name(user_name)
            || (!user_name.is_empty() && self.api.inner.links.contains_key(user_name))
        {
            return (OAPI_ERR, String::new());
        }
        let name = if user_name.is_empty() {
            sap_model::next_numeric_name(self.api.inner.links.keys())
        } else {
            user_name.into()
        };
        let link = LinkObj {
            name: name.clone(),
            point_i: point_i.into(),
            point_j: point_j.map(str::to_owned),
            prop: property.into(),
            angle_deg: 0.,
            selected: false,
        };
        if set_link_object(&mut self.api.inner, link).is_ok() {
            (OAPI_OK, name)
        } else {
            (OAPI_ERR, String::new())
        }
    }
    pub fn delete(&mut self, name: &str) -> i32 {
        if self.api.inner.is_locked || !self.api.inner.links.contains_key(name) {
            return OAPI_ERR;
        }
        self.api.inner.links.shift_remove(name);
        self.api.inner.invalidate_results();
        OAPI_OK
    }
    pub fn count(&self) -> usize {
        self.api.inner.links.len()
    }
    pub fn get_name_list(&self) -> (i32, Vec<String>) {
        (OAPI_OK, self.api.inner.links.keys().cloned().collect())
    }
}
