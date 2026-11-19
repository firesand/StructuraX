//! End-to-end tests of the result displays of the tutorial video (shell
//! stress components and faces, displacement contours, plot functions and the
//! modal tables), driven through the GUI's own functions. They live inside
//! the crate because the application is a binary without a library target.
//!
//! Two kinds of "through the application" are used and each test says which:
//! (a) the state is set by the application's own functions or by synthetic
//! pointer and key input sent through its frame function (`render_ui`), and
//! (b) the checked values are the ones its pure display functions return or
//! the texts and colours it paints in that frame. The independent side of each
//! comparison (a closed form, a model built in code, a printed vendor number)
//! is named in the test.

mod cylinder;
mod vendor;
mod frame;
mod harness;
mod strip;
mod ten_storey;
mod tower;
