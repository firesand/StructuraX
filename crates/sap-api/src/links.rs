//! Typed native link editing. Full vendor COM method coverage is not implied.
use super::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::{LinkObj, LinkProp, LinkPropType, SapModel};

/// Validate the represented linear/Gap/Hook subset before changing a model.
pub fn set_link_property(model: &mut SapModel, property: LinkProp) -> Result<(), String> {
    if model.is_locked {
        return Err("Model is locked.".into());
    }
    if property.name.trim().is_empty() || property.name.chars().any(char::is_control) {
        return Err("Provide a nonempty property name without control characters.".into());
    }
    if !matches!(
        property.link_type,
        LinkPropType::Linear | LinkPropType::Gap | LinkPropType::Hook
    ) {
        return Err("This editor supports linear, gap and hook link properties only.".into());
    }
    if property
        .ke
        .iter()
        .chain(&property.ce)
        .chain(&property.k_nl)
        .chain(&property.param1)
        .any(|v| !v.is_finite() || *v < 0.)
        || property.param2.iter().any(|v| !v.is_finite() || *v != 0.)
        || [property.dj2, property.dj3, property.mass, property.weight]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.)
    {
        return Err("Stiffness, damping, opening, distances, mass and weight must be finite and nonnegative; post-yield ratios are not supported for these laws.".into());
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
            return Err("Nonlinear flags require an active, unfixed Gap/Hook DOF.".into());
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
    if link.name.trim().is_empty()
        || link.name.chars().any(char::is_control)
        || !link.angle_deg.is_finite()
    {
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
        if !user_name.trim().is_empty() && self.api.inner.links.contains_key(user_name) {
            return (OAPI_ERR, String::new());
        }
        let name = if user_name.trim().is_empty() {
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
}
