//! End-to-end tests of wave 5: the rows of the tutorial video that the wave
//! closes (Grid Only, the seismic eccentricity, the load read-out, a section with
//! unequal flanges, Create Video), the repairs of the solver seen through the
//! application's windows, and the vendor examples that touch them.
//!
//! The application is run frame by frame through `render_ui` with synthetic
//! pointer and key input, as in `video_results_e2e`; what is checked is what the
//! application paints or the model and result it changed. The independent side of
//! each comparison (a closed form written out in the test, a manual equation with
//! its page, a printed vendor value) is named in the test, never read from the
//! code under test.

#[path = "video_results_e2e/harness.rs"]
#[allow(dead_code, clippy::duplicate_mod)]
mod harness;
mod design_files;
mod ext;
mod ortho;
mod repairs;
mod rows;
mod solver_repairs;
mod vendor;
mod video;
