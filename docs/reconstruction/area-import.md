# Area mesh, mass and modifier import, and the corrections it exposed

Date: 2026-11-12. Scope note for one change set. Vendor-runtime acceptance and
broad parity remain unestablished.

## Imported from native files

| Data | Native carrier | Model field | Corpus files |
| --- | --- | --- | --- |
| N x N area auto-mesh | `ShMesh` (`TYShMesh`) | `AreaObj::auto_mesh` | 7 imported, 12 recorded and refused |
| Area added mass per area | `ShMass` (`TYSMass`) | `AreaObj::mass_per_area` (new) | 7 |
| Per-object area modifiers | `ShModifier` (`TYShModifier`) | `AreaObj::modifiers` | 3 |

The importer previously read the mesh counts from area-object fields that are
zero in all 336 files, and did not read the other two at all.

**Auto-mesh rule.** A mesh record is imported only when the solver's meshing
means the same thing: mesh type 1 (N x N), equal counts, a four-joint area, no
point-on-edge, cookie-cut, sub-mesh or local-axes options, and restraint
options that agree with what the solver does. The solver restrains a new edge
joint in the degrees of freedom restrained at both ends of the edge, and an
interior joint in those restrained at all four corners. That equals the vendor
"restraints on edge" and "restraints on face" options when they are on, or when
there is nothing for them to act on. Otherwise the record becomes an
unmodeled-source entry (all cases). Unequal counts are refused because the
vendor edge order for them is not established by any file that could be checked.

Of the 19 corpus files: the seven Example 2-008 files import. The six
Example 2-016 files are refused because all four corners are restrained in Uz
while the vendor option restrains only the new edge joints. The six Example
2-019 files are refused for unequal counts and edge restraints off.

**Native export** writes the three assignments back to the same sections (class
tags observed in the corpus: `TYShMesh` 19, `TYSMass` 7, `TYShModifier` 5
records, all version 1). Unequal mesh counts are refused on native export. S2K
carries the mass as a `MassPerArea` column on the area connectivity row.

## Corrections to the native area and solid section import

These were found while checking Example 2-008 and are corrected in both
directions (import and native export).

| Field | Was read as | Evidence | Now |
| --- | --- | --- | --- |
| Shell `m_SType` | the OAPI enumeration (1 thin, 2 thick, 3 plate thin, 4 plate thick, ...) | all 49 "thin" verification files and all 135 untouched default sections (vendor default Shell-Thin) store 4; all 53 "thick" files store 1; no other value occurs | 4 = Shell-Thin, 1 = Shell-Thick; any other shell code is recorded and refused |
| `m_ElementType` | ignored: every area section became a shell | 31 plane files store 1, 11 axisymmetric-solid files store 2; the six "strain" files have section type 2, the other plane files 1 | 1 = plane (section type 1 stress, 2 strain), 2 = axisymmetric solid |
| `m_Incompatible` (area and solid sections) | 1 = incompatible modes on | all 18 "comp" files store 1, all 19 "incomp" files and all defaults store 0 (vendor default on) | 0 = on, 1 = off |

Before the correction every thin shell example was solved with the thick-plate
formulation and no membrane stiffness, every thick one as a thin shell, every
plane-strain and axisymmetric model as a shell, and every "incompatible modes"
solid model with the compatible element.

Native export refuses the section types whose code has not been observed
(plate thin, plate thick, membrane, layered).

**Models read before this correction keep the old meaning.** A model that was
imported earlier and saved as S2K or as a private snapshot has the old section
types stored in it and must be re-imported from the vendor file. A native file
that an earlier version of this project synthesized carries the old codes and
has no marker that distinguishes it from a vendor file; it now reads back with
thin and thick swapped and the incompatible-modes flag flipped.

## Solver changes

- **Area added mass** is lumped to the translations of the element joints. It
  is not scaled by the mass modifier, which applies to the section's own mass.
- **Axisymmetric-solid sections are refused.** They were assembled as
  plane-stress membranes.
- **Plane sections with incompatible bending modes are refused.** The area
  element has no such modes; the vendor default asks for them.
- **Cases that use geometric stiffness are refused when the model has area or
  solid elements**: buckling, nonlinear static with P-Delta, and linear static
  or modal cases with an initial case (the solver takes geometric stiffness
  from that case's frame forces whatever its type). Geometric stiffness is
  assembled from frame axial forces only, so shell membrane and solid stresses
  would be left out. This refuses ordinary frame-plus-slab P-Delta models.
- **The singularity test of the linear static factorization is no longer
  relative to constraint penalties.** A constrained degree of freedom carries a
  penalty about 1e6 times the largest structural stiffness on its diagonal, and
  a pivot was compared with that. A sound but flexible constrained structure
  (the Example 5-002 beam with its rigid tip) was reported as singular. The
  pivot is now compared with the structural diagonal, and never with less than
  the roundoff of the penalty arithmetic, so a real mechanism is still caught.
  A degree of freedom with no element stiffness at all (penalty-only, and also
  one held only by a joint spring) is compared with that roundoff floor alone.
  This applies to the linear static path only; the modal, buckling, nonlinear
  and history paths still use the original test.

## Evidence

Published values are from the CSI Software Verification manual; files are the
vendor's own, imported natively.

| Check | Published | Result |
| --- | --- | --- |
| Example 2-008, cantilever plate periods, modes 1-5, 10 x 10 mesh, thin (kip, in, s) | 0.01743 0.00717 0.00288 0.00228 0.00200 | within 1.5 % (largest -1.3 %) |
| same, thick | 0.01746 0.00721 0.00290 0.00230 0.00202 | within 1.5 % (largest -1.0 %) |
| same, 20 x 20 mesh, thin and thick | 0.01737 ... 0.00196 / 0.01740 ... 0.00198 | within 0.6 % (ignored by default: about a minute per model) |
| thick periods exceed thin periods, every mode | stated conclusion | holds |
| Model D (thick, V13 = V23 = 1000) equals the thin plate | stated as identical | within 1e-4 relative, on the 10 x 10 mesh |
| Example 3-001, plane-stress patch test | sigma_xx = sigma_yy = 1333.33 psi, tau_xy = 400 psi | joint displacements to 1e-12 in, membrane forces to 1e-9 |
| Example 5-002 Model A, tip Ux / Uz / Uy with incompatible modes (lb, in) | 2.926e-5 / 0.1057 / 0.4203 | within 1 % |
| same without incompatible modes | 2.914e-5 / 0.0100 / 0.0109 | within 1 % |

Portable tests: closed-form rigid-translation period of a plate on springs
with added mass, its independence from the mass modifier and from units; the
four refusals above; the factorization test on a three-DOF penalty system with
and without a mechanism; native and S2K roundtrips of mesh, mass, modifiers and
section types.

## Limits

- The 40 x 40 models of Example 2-008 (including Model D as filed) are not run:
  the dense solver would need about 10 000 degrees of freedom. Model D is
  checked by solving its imported properties on the 10 x 10 mesh.
- The incompatible-modes plane files of Example 3-001 and the other plane
  examples that ask for incompatible modes are refused, not reproduced.
- The vendor "restraints on edge/face" options are not model fields; a mesh that
  needs them off is refused.
- The Example 5-002 "incomp" check exercises the corrected factorization test
  only in linear static analysis.
- No vendor file has an area section with any other shell-type code, so plate,
  membrane and layered sections cannot be imported from or written to native
  files.
