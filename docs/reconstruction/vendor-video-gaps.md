# The vendor's video page: what its videos show and what StructuraX lacks

Date: 2026-11-18. Reading notes for the tutorial-video work, kept beside
[tutorial-video-progress.md](tutorial-video-progress.md). This file changes
no status of the project: vendor-runtime acceptance and broad parity remain
unestablished, and no percentage of compatibility is claimed.

The vendor's page for SAP2000 lists 46 videos (`video-catalog.json`). Seven
are in the tracker itself (five "CSI Spotlight" videos, "Static Pushover
Analysis" and "P-Delta Analysis"). This file holds the other 39.

How they were read:

- Each video was read twice by reading agents: once from its transcript,
  listing every command, form and option the speaker names, and once from
  frames of the screen at the moments where a form is shown (about 290
  frames in all). Where the two readings differ, the frames are used.
- For a part of four videos the first frame reading could not be trusted
  (the reader no longer had the pictures when it wrote); those frames were
  taken again and only the second reading is used.
- Every feature was looked up in the v19 help ("H/" below is the folder
  `Menus/` of the unpacked v19 help; the vendor material is not in the
  repository) and in the StructuraX code and documents. A status here is the
  readers' finding with the evidence they gave. The reviewer checked a
  sample: fifteen "absent" findings against the code, and one frame against
  the video. It is a list of leads, not a test result.
- No number shown or spoken in a video is used anywhere as a reference.
- No screenshot or transcript text is kept.

Status words in the tables: **present**, **partial** (what is missing is
said), **absent**, **refused** (the program rejects the input with a
message), **not determined**. "Ledger" names the item of the tracker that
already holds a gap; "new" means it had none when this file was written.
Since then the tracker groups the new ones by area as G58 to G68.

## 1. Table of the 39 videos

| # | Video | Length | Program version on screen | What it is about | present / partial / absent / refused / not determined | What stops or limits it most |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 01 Introductory Tutorial | 30:17 | v15.0.0 Advanced | build, load, run and steel-design a frame plus shell model | 23 / 6 / 4 / 0 / 0 | [absent] Select > Specified Coordinate Range; Automatic Area Mesh only partial |
| 2 | 02 Select Commands | 6:48 | v15.0.0 Advanced | the Select menu | 5 / 0 / 12 / 0 / 0 | [absent] most Select commands |
| 3 | 03 Draw Commands | 9:28 | v15.0.0 Advanced | drawing controls, point edit, align, snaps | 6 / 3 / 8 / 0 / 0 | [absent] Drawing Control Type hotkeys, Align Points, Reshape, Trim/Extend, snaps |
| 4 | 06 Nonprismatic Sections | 6:22 | v15.0.0 Advanced | tapered section definition and use | 4 / 2 / 2 / 0 / 0 | [partial] no form to make a nonprismatic section; extruded view has no taper |
| 5 | 13 Cardinal Points | 9:51 | v15.0.0 Ultimate | frame insertion point, beam with slab | 3 / 3 / 0 / 0 / 0 | [partial] no GUI form for Insertion Point; API refuses mirror |
| 6 | 12 Section Designer | 13:22 | v15.0.0 Ultimate | SD section, rebar, concrete check | 0 / 1 / 8 / 0 / 0 | [absent] Section Designer as a whole |
| 7 | TECH TIPS Auto vs Manual Mesh | 3:45 | not read (title bar cropped; menus newer than v19) | Divide Areas against Automatic Area Mesh on walls | 2 / 2 / 2 / 0 / 0 | [absent] Edit > Edit Areas > Divide Areas; mesh modes partial |
| 8 | 08 Area Edge Constraints | 3:13 | v15.0.0 Ultimate | joining two mismatched meshes | 3 / 0 / 1 / 0 / 0 | [absent] Generate Edge Constraints (flag stored and ignored) |
| 9 | 09 Tension-only Bracing | 4:21 | v15.0.0 Ultimate | T/C limits on braces, nonlinear static | 4 / 0 / 0 / 1 / 0 | [refused] T/C limits with subdivision, offsets, releases; a linear case ignores them |
| 10 | 16 Cable Objects | 18:40 | v15.0.0 Ultimate | cable geometry, target force, guyed tower | 4 / 2 / 7 / 1 / 0 | [absent] Edit Cable Geometry; mixed cable-frame models refused |
| 11 | 17 Tendons | 10:32 | v15.0.0 Ultimate | tendons as loads and elements | 1 / 2 / 6 / 1 / 0 | [absent] tendon sections, objects, loads |
| 12 | 18 Gap Elements | 17:46 | v15.0.0 Ultimate | pounding: gap links, Ritz, FNA, direct integration | 6 / 6 / 2 / 0 / 0 | [absent] Nonlinear Modal History (FNA); damping by period |
| 13 | Foundation Properties and Assemblies | 4:08 | not read (title bar hidden; newer UI) | pile group foundation property and assembly | 1 / 0 / 3 / 0 / 0 | [absent] Foundation Properties (v24 only) |
| 14 | 28 Varying Mass in Seismic Analyses | 17:12 | v16.1.0 Ultimate | several mass sources, quake case on nonlinear stiffness | 5 / 2 / 4 / 0 / 0 | [absent] several named mass sources; IBC 2006 auto lateral |
| 15 | 33 Beam-Column Joint Flexibility | 12:05 | v19.1.0 Ultimate 32-bit | end length offsets and panel zones | 3 / 2 / 3 / 0 / 0 | [absent] Assign > Joint > Panel Zones; automatic offsets |
| 16 | 20 Nonlinear Shear Walls | 29:48 | v15.1.0 Ultimate | layered nonlinear shell wall, pushover, history, section cuts | 9 / 2 / 9 / 0 / 0 | [absent] layered shell, nonlinear material, generalized displacements, section cuts |
| 17 | 23 Wave Loading | 5:23 | v16.0.0 Ultimate | wave load pattern on a tower | 0 / 2 / 6 / 0 / 0 | [absent] wave load generation (type enum only) |
| 18 | 31 Automated Wind Loads | 18:43 | v19.0.0 Ultimate 32-bit | ASCE 7-10 wind pattern on diaphragms | 3 / 3 / 8 / 0 / 0 | [absent] automatic wind loads (G25) |
| 19 | TECH TIPS Joint Patterns | 4:45 | not read (version digits hidden; 64-bit Ultimate, newer forms) | joint patterns for hydrostatic pressure | 0 / 3 / 4 / 1 / 0 | [absent] Joint Patterns forms (data only by import) |
| 20 | 04 Mass and Modal Analysis | 5:45 | v15.0.0 Advanced | mass source, eigen modes, tables | 9 / 2 / 2 / 0 / 0 | [absent] Assembled Joint Masses table; modal frequency shift options |
| 21 | 10 Response Spectrum Analysis | 5:31 | v15.0.0 Ultimate | IBC 2009 spectrum and RS case | 2 / 4 / 6 / 0 / 0 | [absent] code spectrum, directional combination |
| 22 | 11 Modal Time History Analysis | 14:04 | v15.0.0 Ultimate | function types, linear modal history, video | 7 / 4 / 4 / 0 / 0 | [partial] time history function generators; envelopes absent |
| 23 | 29 Fast Nonlinear Analysis | 17:58 | v16.1.0 Ultimate | FNA with dampers and isolators | 3 / 5 / 6 / 0 / 0 | [absent] FNA case, triple pendulum isolator |
| 24 | 32 Steady-State Vibration | 17:36 | v19.1.0 Ultimate 32-bit | steady-state case with a pump function | 1 / 1 / 6 / 1 / 0 (1 row not needed) | [refused] Steady State load case |
| 25 | 22 Power Spectral Density | 12:50 | v15.1.0 Ultimate | PSD case on a pole | 7 / 0 / 3 / 1 / 0 (1 row not needed) | [refused] Power Spectral Density load case |
| 26 | 19 Nonlinear Staged Construction | 7:48 | v15.0.0 Ultimate | staged construction case | 3 / 1 / 3 / 0 / 0 | [absent] Nonlinear Staged Construction case type |
| 27 | 24 Model Alive | 4:52 | v15.1.0 Ultimate | Model Alive and table editing | 3 / 1 / 1 / 0 / 0 | [absent] Analyze > Model Alive |
| 28 | 27 Buckling Factors and Modes | 7:22 | v16.0.2 Ultimate | buckling case after a P-Delta case | 3 / 2 / 3 / 0 / 0 | [partial] stiffness at end of nonlinear case; buckling-mode member forces absent |
| 29 | TECH TIPS Merge Analysis Results | 4:05 | not read (title bar cut; recent UI) | merge results of cases run separately | 1 / 1 / 4 / 0 / 1 | [absent] Analyze > Merge Analysis Results |
| 30 | 26 CSiLoadOptimizer | 10:35 | v15.0.0 Ultimate | optimizer for lift-off displacements | 2 / 0 / 3 / 0 / 0 | [absent] Tools > CSiLoadOptimizer; ground displacement loads |
| 31 | 34 Concrete Shell Reinforcement Design | 16:56 | v19.2.1 Ultimate 32-bit | concrete shell design parameters and display | 4 / 0 / 3 / 0 / 0 | [absent] concrete shell design |
| 32 | TECH TIPS Multi-Step Animation Video | 4:06 | not read (cropped, no title bar) | Multi-step Video File Creation | 2 / 1 / 3 / 0 / 0, counts unsure (one row, nonlinear modal history, is "present" in D notes and "absent" in B and C frames) | [partial] video options Wire Frame, Absolute Displacements, Show Graph |
| 33 | 15 Creating Reports | 11:17 | v16.0.0 Plus | Create Report, Advanced Report Writer | 0 / 1 / 12 / 0 / 0 | [absent] all report writing |
| 34 | 25 Open API | 13:53 | v15.0.0 Ultimate | OAPI from Excel VBA builds a model | 6 / 0 / 3 / 0 / 1 (1 row not applicable) | [partial] Rust facade only; no COM control, no shared GUI session |
| 35 | 30 Web Activated Licensing | 13:58 | not read (installer frame names SAP2000 17; no program window) | license activation | 0 / 0 / 0 / 1 / 0 | [not a goal] licensing |
| 36 | 07 Interactive Database Editing | 7:59 | v15.1.0 Ultimate | edit tables, calculator, Excel | 3 / 2 / 13 / 0 / 0 | [absent] Interactive Database Editing window |
| 37 | 14 Displaying Tabular Data | 9:13 | v16.0.0 Plus | table format, filter, sort | 2 / 2 / 13 / 0 / 0 | [absent] table Format, Filter, Sort, saved formats |
| 38 | TECH TIPS Interactive Database Editing | 4:56 | not read (cropped; current UI) | joint load table edit | 1 / 1 / 3 / 0 / 0 | [absent] Interactive Database Editing window |
| 39 | TECH TIPS Import of DXF Models | 3:03 | not read (cropped) | DXF import with layers | 0 / 0 / 8 / 0 / 0 | [absent] File > Import > AutoCAD .dxf |

Totals of the last column: [absent] 29 videos, [partial] 6 (4, 5, 22, 28, 32, 34), [refused] 3 (9, 24, 25), [not a goal] 1 (35). About 410 rows are counted in all.

## 2. Features that are absent or partial (deduplicated, by area)

Columns: Feature | Videos | Status | v19 help topic | Ledger id. "new" = no ledger row found.

### 2.1 Selection
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Select > Select > Poly; Intersecting Poly; Intersecting Line | 2 | absent | H/Select/Select/Poly.htm, Intersecting_Poly.htm, Intersecting_Line.htm | new |
| Select > Coordinate Specification > Specified Coordinate Range... (Coordinate System, Units, Entirely Inside / Entirely Or Partially Inside, Point/Line/Area/Solid/Link boxes, per axis Not Limited / Between Two Values / Outside Of Two Values / At A Single Value, Operator, Value, Tolerance) | 1, 2 | absent | H/Select/Select/Coordinate_Specification/Specified_Coordinate_Range.htm | new |
| Select > Coordinate Specification > 3D Box | 2 | absent | .../Select_3D_Box.htm | new |
| Select > Coordinate Specification > Click Joint in XY Plane / XZ Plane / YZ Plane | 2 | absent | .../Coordinate_Specification/Select_XY_Plane.htm etc. | new |
| Select > Select Lines Parallel To > Click Straight Line Object; Coordinate Axes or Plane... | 2 | absent | .../Select_Lines_Parallel_To/Coordinate_Axes_or_Plane.htm | new |
| Select > Properties > Material Properties...; Cable Properties...; Tendon Properties...; Solid Properties...; Link Properties...; Frequency Dependent Link Properties... (Frame Sections and Area Sections exist) | 1, 2 | absent | .../Properties/Material_Properties.htm | new |
| Select > Assignments > Joint Supports...; Joint Constraints... | 2 | absent | .../Assignments/Joint_Supports.htm | new |
| Select > Select Using Tables (Choose Tables for Display, Filter tab) | 2 | absent | H/Select/Select_Using_Tables.htm | new |
| Select > Invert Selection | 2 | absent | H/Select/Select_Invert_Selection.htm | new |
| Select > Deselect > (submenu) | 2 | absent | H/Select/Deselect (folder) | new |
| Select > Select > Groups...; Labels... | 2 | absent (model has no group object) | Groups.htm, Labels.htm (names to confirm) | new |
| Define > Groups; Assign > Assign to Group... (Ctrl+Shift+G); use of a group in a staged case, section cut, plot | 16, 26 | absent (no group object in sap-model) | Define/Groups.htm; Assign/Assign_to_Groups.htm | new |

### 2.2 Drawing, editing and display of the model
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Drawing Control Type list: None <space bar>, Horizontal <H>, Vertical <V>, Parallel to Angle <A>, Fixed Length <L>, Fixed Length and Angle <S>, Fixed dh and dv <D> | 3 | absent | H/Draw/Drawing_Controls.htm | G27 |
| Snap To: Ends and Midpoints, Intersections, perpendicular, lines or edges (StructuraX snaps joints and grid intersections only) | 1, 3 | partial: joints and grid intersections only, 10 px | H/Draw/Snap_To.htm | G27 |
| Drawing in a 3-D view | 3 | partial: points snap to joints only, a free start is refused | H/Draw/Draw_Frame_Cable_Tendon.htm | new |
| Point Information form with Joint Coordinates edit (double-click a cell) | 3 | absent (Object Information is read-only) | Information_Forms/Point_Information_Location_Tab.htm | new |
| Edit > Edit Points > Align Points (X-, Y-, Z-Ordinate, Nearest Line) | 3 | absent | H/Edit/Edit_Points/Align_Points.htm | new |
| Draw > Set Reshape Element Mode | 3 | absent | H/Draw/Set_Reshape_Element_Mode.htm | new |
| Edit > Edit Lines > Trim/Extend Frames | 3 | absent | H/Edit/Edit_Lines/Trim_Extend_Frames.htm | new |
| Quick Draw Secondary Beams, orientation "normal to the nearest edge" | 3 | partial: spans in X or Y only | H/Draw/Quick_Draw_Secondary_Beams.htm | new (G29 is a preview, not this) |
| Draw Poly Area with more than four corners | 1, 3 | partial: more than four refused | H/Draw/Draw_Poly_Area.htm | G10 |
| Edit > Show Duplicates; Merge Duplicates... | 3 | absent | H/Edit/Show_Duplicates.htm, Merge_Duplicates.htm | new |
| Edit > Change Labels... (renames and fixes references) | 3, 36 | absent | H/Edit/Change_Labels.htm | new |
| Edit > Extrude > linear extrusion of frames to areas, Delete Source | 34 | absent (only radial extrusion) | H/Edit/Extrude.htm | G53 |
| Two-window tiled display, each with its own case | 26, 28 | absent (at best one workspace of synchronized 3-D and 2-D views) | Menus/Window (not opened) | new |
| Assign > Clear Display of Assigns | 1 | absent | H/Assign/Clear_Display_of_Assigns.htm | new |
| View > Set Display Options (Fill Objects, Shrink Objects, Extrude View, Show Edges, Show Ref. Lines, Show Bounding Boxes, View by Colors of, Show Analysis Model, Apply to All Windows) | 1, 4 | absent (one "Show Extruded Profile View" box and a few View toggles) | H/View/Set_Display_Options.htm | new |
| Set Display Options > Frames/Cables/Tendons > Local Axes | 4 | absent | same | new |
| Set Display Options: hide links (Links "Not in View") | 12 | partial: extruded view exists; only link labels toggle, no hide-links switch (not settled) | H/Options/Set Display Options | new |
| Extruded view of a nonprismatic member (taper, segments) | 4 | partial: constant depth t3, no taper | H/View/Set_Display_Options.htm | new |

### 2.3 Meshing and area objects
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Assign > Area > Automatic Area Mesh: modes No Auto Meshing / number along edges 1-2 and 1-3 / maximum size / based on points on area edges / cookie cut by lines (Extend All Lines To Intersect Area Edges) / cookie cut by points (Rotation of Cut Lines) / General Divide Tool; Meshing Group; Sub-Mesh Option; Local Axes and Restraints For Added Points; "Use user mesh for analysis, if assigned" (video 7 only) | 1, 7 | partial: "No Auto Meshing" and N1 x N2 quads only; vendor mode says "Quads and Triangles" | H/Assign/Area/Area_Automatic_Area_Mesh.htm | new |
| Edit > Edit Areas > Divide Areas (Divide Selected Areas form, six modes, Reset Form to Default Values) | 7 | absent (divide exists for frames only) | H/Edit/Edit_Areas/Divide_Areas.htm | new |
| Automatic mesh that connects slabs to beams, columns and walls | 7 | absent (mesh is a grid inside one area) | same | new |
| Assign > Area > Generate Edge Constraints (Do NOT Create Edge Constraints / Create Constraints around Object Edges) | 8 | absent; flag `edge_constraint` stored from S2K and ignored | H/Assign/Area/Generate_Edge_Constraints.htm | D73 |
| Assign > Frame > Automatic Frame Mesh... ("at intersections with other objects", maximum length) | 1 | partial: API `set_auto_mesh` has segments and "at joints"; no GUI command; other options refused | H/Assign/Frame/Frame_Automatic_Frame_Mesh.htm | new |

### 2.4 Sections, materials and Section Designer
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Define > Frame Sections > Add New > Other > Nonprismatic form (Start Section, End Section, Length, Length Type, EI33 and EI22 Variation; Add, Insert, Modify, Delete; Display Color) | 4 | partial: model, solver, files carry it; no form | H/Define/Section_Properties/Frame_Sections/Nonprismatic.htm | new (D61 is its mass) |
| Section Designer section: SD Section Data form (Base Material, Design Type No Check/Design, General Steel Section, Concrete Column; Section Designer..., Properties..., Set Modifiers...) | 6 | absent (type name `SD` only) | .../Section_Designer_Section.htm | new |
| SD shape library: steel shapes, solid concrete shapes, Caltrans shapes, poly shape | 6 | absent | not in the help (utility named only) | new |
| SD Shape Properties - Solid (Name, Material, X/Y Center, Height, Width, Rotation, Reinforcing, Conc. Model Mander-Unconfined, C Model, S Model); reshaper; corner coordinates | 6 | absent | not in the help | new |
| SD combined shapes converted by equivalent base material | 6 | absent | not in the help | new |
| SD rebar: Edge Reinforcing (Bar Size, Bar Area, Bar Spacing, Bar Cover, Apply to All Edges), corner bars, Draw Reinforcing Shape (single, line, rectangular, circular) | 6 | absent | not in the help | new |
| SD display: section properties, interaction surface, moment-curvature | 6 | absent (P-M2-M3 hinges also refused) | H/Design/Concrete_Frame_Design/CF_Interaction_Surface_for_Section_Form.htm (design side) | new (G12 related) |
| Concrete Frame Design of an SD section (Concrete Column Check Information) | 6 | partial: not refused by name, but no geometry or bar layout is stored | H/Design/Concrete_Frame_Design | new (G11 related) |
| Define > Section Properties > Reinforcement Bar Sizes | 6 | absent (no named bar size) | H/Define/Section_Properties/Reinforcement_Bar_Sizes.htm | new |
| Cable section (Define > Section Properties > Cable Sections, area only) | 10 | absent as an object; area taken from the frame section | .../Cable_Sections/Cable_Section_Data_Form.htm | new |
| Tendon section (Tendon Section Data: Model Tendon as Loads / as Elements, Prestress Type, Material, diameter or area) | 11 | absent | .../Tendon_Sections/Tendon_Section_Data_Form.htm | new |
| Material type Tendon | 11 | partial: enum value `Tendon = 7` only; claim of rejection rests on editing-wave2.md:29 (not re-opened) | H/Define/Materials | new |
| Nonlinear Material Data (rebar, confined and unconfined concrete stress-strain, Material Stress-Strain Curve Plot) | 16 | absent | Define/Material (Nonlinear Material Data) | new |
| Area section "Shell - Layered/Nonlinear": Layer Definition (Layer Name, Distance, Thickness, Type, Num Int. Points, Material, Material Angle, S11 S22 S12 behavior), Quick Start Parameters | 16 | absent (enum value only) | .../Area_Sections/Shell_Section_Layer_Definition_Form.htm, Concrete_Shell_Section_Quick_Start_Parameters_Form.htm | new |
| Concrete Shell Section Design Parameters (Rebar Material, Default / One Layer / Two Layers, covers Top/Bottom Bar Direction 1/2) | 31 | absent | .../Area_Sections/Concrete_Shell_Section_Design_Parameters_Form.htm | new |
| Define > Foundation Properties... (Isolated Footing, Combined Footing, Pile Group, Pile Pier, Pile Shaft); Foundation Assemblies...; Draw Isolated Foundation; Foundation Assembly Data | 13, 19 (menu entries) | absent | v24 only (Foundation_Assembly_Operations_form_-_For_SAP2000.htm) | new |
| Soil and pile springs generated by a foundation assembly; Assign > Area > Area Springs | 13 | absent | v24 only; H/Assign/Area/Area_Springs.htm | new |

### 2.5 Frame assignments
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Assign > Frame > Insertion Point... (Cardinal Point 1-11, Mirror about Local 2, Coord System, End-I/End-J offsets, "Do not transform frame stiffness for offsets from centroid", Reset Defaults) | 5 | partial: no GUI form; API `set_insertion_point`; solver does points 1-10 of doubly symmetric shapes, equal offsets, linear static; mirror refused; 11 treated as centroid | H/Assign/Frame/Frame_Insertion_Point.htm | new |
| Assign > Frame > End (Length) Offsets...: Automatic from Connectivity / User Defined Lengths, rigid zone factor | 15 | partial: model, API, solver (D21, D28 done); no GUI form | H/Assign/Frame/Frame_End_Length_Offsets.htm | new |
| End Length Offsets "Automatic from Connectivity" | 15 | absent | same | new |
| Assign > Joint > Panel Zones (Elastic Properties from Column / and Doubler Plate, Specified Spring Stiffnesses, Specified Link Property, connectivity, local axes) | 15 | absent | H/Assign/Joint/Joint_Panel_Zones.htm | new |
| Display of the panel zone assign (right-click spring property) | 15 | absent | H/Display/Show_Miscellaneous_Assigns/Show_Miscellaneous_Assigns_Joints.htm | new |
| Object Information (Line / Point Information) listing offsets, rigid zone factor, stations, P-Delta force, T/C limits, hinges, line springs, panel zone | 15 | partial: window shows loads only | Information_Forms/Line_Information_Assignments_Tab.htm | G07 (loads done) |
| Assign > Frame > Tension/Compression Limits... combined with subdivision, end offsets, releases, some geometric options | 9 | refused | H/Assign/Frame/Frame_Tension_Compression_Limits.htm | new (D68 related) |
| Assign > Frame > Tension/Compression Limits... in a linear case | 9 | partial: accepted, applied only in nonlinear static cases; a linear case ignores it without a message | same | new |

### 2.6 Special objects: cables, tendons, links
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Cable as an object (cable is a frame with `cable_params`; three modes) | 10 | partial: cable-only models; no separate cable object | H/Assign/Cable/Cable_Properties.htm | new |
| Cable mixed with frame objects (mast and guys) | 10 | refused (CAPABILITIES l.11) | Cable_Sections | new |
| Edit > Edit Lines > Edit Cable Geometry: Cable Type list (Minimum Tension At I-End, Tension At I-End, Tension At J-End, Horizontal Tension Component, Maximum Vertical Sag, Low-Point Vertical Sag, Undeformed Length, Relative Undeformed Length), point table, Planar View, "Model Cable Using Straight Frame Objects", meshing options | 10 | absent (only length, horizontal tension, I-end tension on a frame) | H/Edit/Edit_Lines/Edit_Cable_Geometry.htm | new |
| Assign > Cable Loads > Target Force... (Axial Force P, Target Force, Relative Location) | 10 | absent | H/Assign/Cable_Loads/Cable_Loads_Target_Force.htm | new |
| Assign > Cable Loads > Strain...; Deformation... | 10 | absent (not found) | Cable_Loads_Strain.htm, Cable_Loads_Deformation.htm | new |
| Target Force Iteration (Maximum Iterations per Stage, Convergence Tolerance, Acceleration Factor, Continue Analysis If No Convergence) | 10 | absent | Define/Load_Cases/Static/Nonlinear/Target-Force_Iteration.htm | new |
| Right-click "Diagrams for Cable Object / Frame Object" window (Scroll for Values, free body diagram) | 10, 11 | absent | H/Display/Show_Forces_Stresses | new |
| Draw tendon objects | 11 | refused (native files with tendons recorded unmodeled; model refused) | Draw_Frame_Cable_Tendon.htm | new |
| Assign > Tendon Loads > Tendon Force/Stress (Jack From This Location, Force/Stress, curvature, wobble, anchorage set slip, elastic shortening, creep, shrinkage, steel relaxation) | 11 | absent | .../Tendon_Loads/Tendon_Loads_Tendon_Force_Stress.htm | new |
| Prestress load pattern type | 11 | partial: enum `Prestress = 34`; no solver use | Define/Load_Pattern | new |
| Element tendon with elastic shortening | 11 | absent | .../Tendon/Tendon_Properties.htm | new |
| Tendon target force, strain, deformation, temperature, gravity loads | 11 | absent | Assign/Tendon_Loads/*.htm | new |
| Tendon Tension/Compression Limits | 11 | absent | .../Tendon_Tension_Compression_Limits.htm | new |
| Edit > Edit Lines > Edit Tendon Profile (Tendon Data: Quick Start, Parabolic Calculator, Insert Above/Below) | 11 | absent | H/Edit/Edit_Lines/Edit_Tendon_Profile.htm | new |
| Draw > Draw 2 Joint Link / 1 Joint Link as draw tools | 23 | absent per C frames; B notes (video 12) say present: see part 5 | H/Draw/ | new |
| Link types Triple Pendulum Isolator; friction and rubber isolators; other nonlinear link types | 23 | absent in the solver (enum and reader exist) | Link_Support properties | new |
| Frequency-dependent link properties | 24 (help only) | absent | H/Assign/Link_Support/Frequency_Dep._Link_Props.htm | new |
| Ritz starting vector Load Type "Link" / Name "All" (all link nonlinear degrees of freedom) | 12, 23 | absent in the GUI (accel UX, UY, UZ only); refused on native import | H/Define/Load_Cases/Modal_Form.htm | G41 (D36) |
| Nonlinear direct-integration history with link dampers or gaps | 12, 23 | partial: rest start only; no continuation, no geometric nonlinearity | .../Time_History/NonLinear_Direct_Integration_History_Form.htm | new |

### 2.7 Load patterns and automatic loads
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Load pattern type Wave | 17 | partial: enum `Wave = 14`; nothing generates loads | Define_Load_Patterns.htm | new |
| Wave Load Pattern form (Wave Characteristics, Current Profile, Marine Growth, Drag and Inertia Coefficients, Wind Load, Include Buoyant Loads, discretization, crest position, datum, water density); Modify Lateral Load Pattern | 17 | absent | .../Wave_Load/Wave_Load_Pattern_Form.htm, Wave_Wind_Load_Pattern_Form.htm, Seastate_Load_Pattern_Form.htm | new |
| Wave Characteristics form (Airy or User Defined, Wave Kinematics Factor, Storm Water Depth) | 17 | absent | .../Wave_Load/Wave_Characteristics_Form.htm | new |
| Current Profile, Marine Growth, Drag and Inertia Coefficients forms | 17 | absent | .../Wave_Load/Current_Profile_Data_Form.htm, Marine_Growth_Data_Form.htm, Drag_and_Inertia_Coefficients_Form.htm | new |
| Show Wave Plot (Click for Values, Contours, Arrows, Vertical Cut, Horizontal Cut) and Show Wave Table (nine columns) | 17 | absent | Wave_Load_Pattern_Form.htm | new |
| Assign > Frame Loads > Auto Wave Loading Parameters | 17 | absent | H/Assign/Fame_Loads/Frame_Loads_Auto_Wave_Loading_Parameters.htm | new |
| Wind load on the dry part of a wave pattern | 17 | absent | Wave_Wind_Load_Pattern_Form.htm | new |
| Load pattern type Wind with default pattern WIND | 18 | partial: type and name exist; generates no loads | Define_Load_Patterns.htm | new |
| Auto Lateral Load ASCE 7-10 wind: "ASCE 7-10 Wind Load Pattern" form (Wind Direction Angle, Windward / Leeward Cp, Wind Speed, Exposure Type, Kzt, Gust Factor, Kd, Solid / Gross Area Ratio) | 18 | absent | H/Define/Load_Pattern/Wind_Load_Pattern_Form.htm | G25 |
| Exposure from Extents of Rigid Diaphragms; Modify/Show Exposure Widths...; Exposure Height | 18 | absent | same | G25 |
| Exposure from Frame and Area Objects | 18 | absent | same | G25 |
| Case "Create All Cases" (12 patterns WIND-n), e1 and e2 ratios | 18 | absent for wind (seismic eccentricity exists, G24) | same | G25 |
| Other wind codes (NBCC, AS/NZS and others) | 18 | absent | same | G55 |
| Assign > Area Loads > Wind Pressure Coefficients (Shell)... (Windward / Other, Cp) | 18 | absent | H/Assign/Area_Loads/Area_Loads_Wind_Pressure_Coefficients.htm | G25 |
| Assign > Frame Loads > Open Structure Wind Parameters (Element is Loaded by Wind, Ice Thickness, Net Force/Shape Coefficient) | 18 | absent | H/Assign/Fame_Loads/Frame_Loads_Open_Structure_Wind_Parameters.htm | G25 |
| Display > Show Load Assigns > Joint... / Frame/Cable/Tendon... / Area... (pattern choice, Resolve Forces in Coordinate System, Show Loading Values; surface pressure, pore pressure, temperature, gradient 3-3) | 17, 18, 19 | partial: "Show Applied Loads" check box and right-click Loads tab; no form | H/Display/Show_Load_Assigns/ | new (G07 read-out done) |
| Define > Joint Patterns... (Define Pattern Names: Add New Pattern Name, Change Pattern Name, Delete Pattern) | 19 | absent in the GUI; model has `pattern_values` on points | H/Define/Joint_Patterns.htm | new |
| Assign > Joint Patterns... (Assign Joint Pattern Data: X, Y, Z Multipliers; Z Coordinate at Zero Pressure and Weight per Unit Volume; Restrictions; Add to / Replace / Delete Existing Values) | 19 | absent; values arrive only by import | H/Assign/Assign_Joint_Patterns.htm | new |
| Area surface pressure by joint pattern | 19 | absent | .../Area_Loads_Surface_Pressure_(All).htm | G42 |
| Area temperature scaled by joint pattern | 19 | refused ("not implemented") | Area_Loads_Temperature_(All).htm | G33 |
| Frame or cable temperature or load by joint pattern | 19 | partial: refused for cables; frames not checked | Frame loads topics | new |
| Assign > Area Loads > Uniform (Shell)..., direction Gravity projected (projected directions) | 1 | partial: projected directions refused by the solver | .../Area_Loads_Uniform_(Shell).htm | new |
| Assign > Joint Loads > Displacements... (Ground Displacements; display "Joint Ground Displs") | 30 | absent | Menus/Assign/Joint_Loads (not opened) | new |
| Define > Mass Source with several named sources (Mass Source Name, add copy, default source) | 14 | absent (one source per model) | H/Define/Mass_Source.htm | new |
| Mass source chosen per load case | 14 | absent | .../Load_Cases/Load_Case_Data_Form.htm | new |
| Mass Source form: check boxes and multiplier table | 20, 24 | partial: one source, no name | H/Define/Mass_Source.htm | new |
| Define Load Patterns, Auto Lateral Load Pattern "IBC 2006" | 14 | absent (2012 IBC / ASCE 7-10 only) | Define/Load_Pattern | new (G21 to G23 are other items) |
| Design load combination names UDSTLn (vendor) against DSTLn (StructuraX) | 18 | partial: names differ | H/Design/ | new |

### 2.8 Load and analysis cases
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Load case type Nonlinear Staged Construction (operations Add Structure, Change Section, Load Objects, Load Objects If Added, Remove Structure; Duration (Days), Provide Output, Output Label, Age At Add, Show Stages In Tree View) | 26 | absent | H/Define/Load_Cases/Static/Nonlinear/Nonlinear_Static_Staged_Construction_Form.htm | new |
| Results Saved for Nonlinear Static Load Cases: End of Final Stage / End of Each Stage / Start and End of Each Stage / Two or More Times In Each Stage | 26 | partial: saved states of nonlinear static exist; no stage options | .../Results_Saved_for_Nonlinear_Static_Load_Cases_Form.htm | new (V2.14 saved states) |
| Add Copy of Load Case (and of mass source) | 14, 28 | absent | H/Define/Load_Cases/Define_Load_Cases.htm | G44 |
| Stiffness at End of Nonlinear Case (modal, static, buckling) | 14, 20, 23, 24, 28 | partial: only where no hinge has yielded | .../Stiffness_to_Use.htm | G48 (D72 for buckling) |
| Modal case: Frequency Shift (Center), Cutoff Frequency (Radius), Convergence Tolerance, Allow Automatic Frequency Shifting | 20, 24 | absent | .../Load_Cases/Modal_Form.htm | new |
| Response Spectrum case: Modal Combination GMC (f1, f2), NRC 10 Percent, Double Sum, Periodic + Rigid Type (CQC, SRSS, Absolute exist) | 21 | partial: three of six | .../Load_Cases/Response_Spectrum_Form.htm | new |
| Response Spectrum case: Directional Combination SRSS / CQC3 / Absolute | 21 | absent | same | new |
| Modal Damping: Interpolated Damping by Period or Frequency; Mass and Stiffness Proportional by Coefficient; Modal Damping Overrides | 21, 22 | partial: constant ratio only | .../Damping.htm, Modal_Damping_Form.htm | G39 |
| Modal damping above 0.99 (0.999 with a ramp load) | 23 | partial: range 0 to 0.99 rejects it | Modal_Damping_Form.htm | new |
| Response Spectrum IBC 2009 Function Definition (Ss, S1 from USGS lat/long or zip, Site Class, Fa, Fv, SDS, SD1, Convert to User Defined) | 21 | absent | not in v19 or v24 help; v19 has IBC 2006 (Response_Spectrum_IBC_2006_Function_Definition_Form.htm) | new |
| Other code response spectra (UBC, Eurocode 8, NBCC and others) | 21 | absent | H/Define/Functions/Response_Spectrum/*.htm (18 files) | new |
| Response spectrum from file | 21 | partial: user table only; file not checked | Spectrum_from_File_..._Form.htm | new |
| Spectrum plot with cursor coordinates (Display Graph) | 21 | absent | Define_Response_Spectrum_Functions.htm | new |
| Scale factor calculator (Shift + double click); SAP2000 Calculator in tables (View, Edit, Angles, Close) | 21, 36, 38 | absent | Keyboard_Commands_and_Special_Features/Calculator.htm | new |
| Time history function types Cosine, Ramp, Sawtooth, Sine, Triangular, User Periodic (parameters, Convert to User Defined, Display Graph) | 22, 23 | partial: user pairs and from file only | Time_History_Sine_Function_Definition_Form.htm and siblings | new |
| User Periodic function; Time History Motion Type Periodic | 22 | absent | Time_History_User_Periodic_Function.htm | G40 |
| Case type "Nonlinear Modal History (FNA)" | 12, 23 | absent; refused by name on native import | .../Time_History/Nonlinear_Modal_History_Form.htm | G41 (D26) |
| Modal history "Continue from State at End of" another case | 23 | partial: chaining only for nonlinear static | Initial_Conditions.htm | new |
| Time Integration Parameters: Collocation; Chung and Hulbert (Newmark, Wilson, Hilber-Hughes-Taylor exist) | 12 | partial | .../Time_History/Time_Integration_Parameters_Form.htm | new |
| Proportional damping "Specify Damping by Period" / "by Frequency", Recalculate Coefficients | 12 | absent (GUI has mass and stiffness coefficients only) | .../Load_Cases/Damping.htm | new |
| Load case type Steady State (Frequency Step Data, Set Additional Frequencies, Add Modal Frequencies, Signed Fractional Deviation, advanced load columns Phase Angle / Coord Sys / Angle, Solution Type Modal/Direct) | 24 | refused (UnsupportedInput) | .../Load_Cases/Steady_State_Form.htm, Additional_Frequencies_Form.htm | new |
| Steady State Function Definition (Frequency, Value; Hz or RPM) | 24 | absent | .../Functions/Steady_State/*.htm | new |
| Hysteretic Damping form (steady state, PSD) | 24, 25 | absent | .../Load_Cases/Hysteretic_Damping_Form.htm | new |
| Load case type Power Spectral Density; Power Spectral Density Function Definition (User, From File; UNIFPSD) | 25 | refused (case); absent (function) | .../Load_Cases/Power_Spectral_Density_Form.htm; .../Functions/Power_Spectral_Density/*.htm | new |
| Analyze > Model Alive (toolbar toggle; "Model-Alive" box in Set Load Cases to Run) | 27, 29 | absent | H/Analyze/Model_Alive.htm | new |
| Analyze > Merge Analysis Results | 29 | absent | not in v19 help, not in v24 help (grep) | new |
| Save analysis results with the model | 29 | absent (`semantic_model` clears case_results) | not a help topic | new |
| Set Analysis Options > Advanced SAPFire Options (Standard / Advanced / Multi-threaded Solver; Auto / GUI / Separate Process; Run Load Cases in Parallel; threads; response file size) | 29, 30 | absent | H/Analyze/Set_Analysis_Options.htm does not name them; "separate process" only in Starting_SAP2000_from_the_Command_Line.htm | new |
| Set Load Cases to Run form (Run/Do Not Run All, Delete Results, named sets, Analysis Monitor Options, Show Messages after Run) | 29 | partial: per-case "Run this case" box only | H/Analyze/Set_Load_Cases_to_Run_and_Run_Analysis.htm | new |
| Output Options form (Mode Shapes range, Base Reactions Location, Envelopes / Step-by-Step / Last Step for histories, steady state, PSD RMS, Multi-Valued Response Combos) | 20 | absent | Database_Tables/Output_Options_Form.htm | new (D05 for combos) |
| Tools > CSiLoadOptimizer (variables, goals Joint Displacement / Joint Reaction / Frame Force, Sense, Relative Benefit, new case "<case>_<name>") | 30 | absent | H/Tools/CSiLoadOptimizer.htm | new |

### 2.9 Results display, plots, video
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Envelopes of a time history | 22 | absent (step by step only) | Display/Show_Forces_Stresses | D25 |
| Show Plot Functions: Plot Function Trace Display Definition, horizontal function other than TIME, named sets | 16 | partial: joint responses and history loads only; not for generalized displacements or section cuts | .../Show_Plot_Function/Plot_Functions_Form.htm | G05 (sets in file) |
| Plot Functions types Add Load / Energy / Base / Joint Disps/Forces / Frame Forces / Link Forces/Deformations ("Input Energy") | 12, 23 | partial: input record plots as a trace; no six-type choice | same | G03 |
| Plot of absolute acceleration and velocity | 22 | partial: displacement and absolute acceleration only | .../Joint_Plot_Functions_Form.htm | G03, G04 |
| Plot functions against frequency (steady state); PSD plots; RMS displacement and force display | 24, 25 | absent | .../Joint_Plot_Functions_Form.htm | G03 |
| Display > Show Response Spectrum Curves; Response Spectrum Generation form (tabs Define, Options, Frequency, Damping, Axes, Direction) | 22 | absent | H/Display/Show_Response_Spectrum_Curves.htm | new |
| Define > Generalized Displacements; plot of a generalized displacement | 16 | absent | H/Define/Generalized_Displacements.htm; Generalized_Displacement_Plot_Functions.htm | new |
| Define > Section Cuts (Group or Quadrilateral Cutting Planes) | 16 | absent | H/Define/Section_Cuts.htm | new |
| Section-cut plot function (moment against rotation, function against function) | 16 | absent | .../Section_Cut_Plot_Function.htm | new |
| Display > Show Forces/Stresses > Shells: Component Types Shell Strains and Shell Layer Stresses; full component list (F11 to VMax) | 7, 16 | partial (video 7) / absent (layer stresses, video 16): one value per element | .../Show_Forces_Stresses_Shells.htm | G01 |
| Display Shell Stresses > Component Type "Concrete Design" (Output Type; N11, N22, N12, NMax, NMin, NDes1, NDes2, ASt1, ASt2, Fc, Sc) | 31 | absent | Display shell stresses form | new |
| Member forces for a buckling mode (title "Axial Force Diagram (Buckling) - Mode 6") | 28 | absent | Display > Show Forces/Stresses > Frames | new (G02 is shells per mode) |
| Buckling Factors table (OutputCase, StepType, StepNum, ScaleFactor) | 28 | partial: two columns "Buckling Mode", "Critical Factor" | Database Tables | new |
| Show Tables > Assembled Joint Masses | 20 | absent | H/Display/Show_Tables.htm | new |
| Multi-step Video File Creation: Wire Frame, Cubic Curve, Absolute Displacements, Show Graph with Named Set, Delete Temporary BMP files, Plot Type | 12, 22, 23, 32 | partial: kind, start, end, increment, size, fps, magnification, path only | .../Create_Video/Create_Multi_Step_Animation_Video.htm | G08, D45 (fields done) |
| Video compression (Microsoft Video 1, Compression Quality) | 22, 32 | absent (uncompressed AVI by design; Windows codec dialog) | .../Create_Video/Video_Compression_Form.htm | G08 |

### 2.10 Tables, reports, export, import, API
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Display > Show Tables form "Choose Tables for Display" (Select Load Cases, Modify/Show Options, Show Unformatted, Table Formats File..., Named Sets) | 20, 37 | partial: window with result and design tabs; no selection form | H/Display/Show_Tables.htm, Database_Tables/Choose_Tables_For_Form.htm | new (G49 related) |
| Input tables displayed (joint coordinates, connectivity) | 37 | partial: API tables only (19) | Database_Table_Form.htm | new |
| Modify/Show Database Table Format > Format (Include Field in Report, Repeat Field if Table is Split, Force a Table Split After this Field, Field Alignment, Units for Field Width, Field Width, display order, Reset Display Order) | 37 | absent | .../Modify_Show_Database_Table_Format_Form.htm | new |
| Table Filter (None, Quick, Basic, Advanced SQL string; operators) | 2, 37 | absent | Database_Table_Form.htm | new |
| Table Sort (up to five keys) | 37 | absent | same | new |
| Save Table Format to File; Apply Format from File; Apply Program Default Format | 37 | absent | same; Database_Table_Named_Sets_Form.htm | new |
| File > Display Current Table in Word; Print Tables | 37 | absent | H/File/Print_Tables.htm | G49 |
| Named sets of tables | 37 | absent | Database_Table_Named_Sets_Form.htm | new |
| "Add Tables..." button and record navigator in a table window; column information by right click | 37 | absent | Database_Table_Form.htm | new |
| Edit > Interactive Database Editing... (Ctrl+E); Choose Tables for Interactive Editing (Show All Fields, Expose All Input Tables, only ticked groups, only selected objects) | 27, 36, 38 | absent in the GUI | H/Edit/Interactive_Database_Editing.htm | new |
| Interactive Database Editing table editor (table list, grid, Apply to Model, Done) | 27, 36, 38 | partial: API only (`get/set_table_for_editing_array`, `apply_edited_tables`, 19 tables) | .../Interactive_Database_Editing_Form.htm | new |
| Editor buttons Copy, Paste, Paste Insert, Paste Append, Delete Record, Append Blank Rows, Find..., Replace...; row selection blinks the object | 36 | absent | same | new |
| Advanced Options..., Undo Last Apply, Cancel Excel, Overwrite Mode | 36 | absent in the GUI (API has `cancel_table_editing`) | same | new |
| To Excel / From Excel | 36, 38 | absent | Database_Table_Form.htm | G49 |
| Units button inside the table editor | 36 | absent | same | new |
| Joint Loads - Force table editing | 36, 38 | absent (Joint Restraint Assignments covered) | Database_Table_Form.htm | new |
| Renamed row on import becomes a new item | 38 | partial: read from code, no test | Interactive_Database_Editing.htm | new |
| File > Export > Excel spreadsheet; File > Import > Excel, Access, Tabular Database | 36 | absent | H/File/Export/Export_SAP2000_MS_Excel_Spreadsheet_xls_file.htm; H/File/Import/ | G35 (Excel not named) |
| File > Create Report | 33 | absent | H/File/Create_Report.htm | new |
| File > Report Setup (Report Setup Data: Report Contents File, Table Format File, five output types, Report Group, orientation, cover, hyperlinked contents) | 33 | absent | H/File/Report_Setup.htm | new |
| Report contents XML file | 33 | absent | not a help page | new |
| File > Advanced Report Writer (Create Custom Report: Find Existing DB File, Convert Excel File, Convert Text File, Create New DB File, Section, Page Break, Text, Blank Line(s), Picture, Apply Formats, Save Formats) | 33 | absent | H/File/Advanced_Report_Writer/Advanced_Report_Writer.htm | new |
| Report Page Setup..., Font Setup... (nine tabs), Cover Page Layout... | 33 | absent | Report_Item_Page_Setup_Form.htm, Report_Item_Font_Form.htm, Report_Item_Cover_Page_Form.htm | new |
| Save Named Report; Open Named Report | 33 | absent | Named_Report_Form.htm | new |
| Report Picture form (caption, location, maximum size, page breaks, blank lines) | 33 | absent | Report_Picture_Form.htm | new |
| Report output RTF, HTML, text | 33 | absent | Report_Item_Filename.htm | new |
| File > Capture Picture (bitmap of the deformed shape) | 33 | absent | H/File/Capture_Picture.htm, Capture_Enhanced_Metafile.htm | new |
| Database tables as the source of a report (46 + 9 tables against 19 in the API) | 33 | partial | Choose_Tables_For_Form.htm | new |
| OAPI driven from Excel VBA or another program (COM) | 34 | absent (Rust facade, "not a COM server") | not in v19 help | new |
| GUI and API session sharing one running program | 34 | absent | not in v19 help | new |
| File > Import > AutoCAD .dxf: Import Information (Global Up Direction X, Y, Z, -X, -Y, -Z, Units, Length Tolerance, Max Arc Segment Length) | 39 | absent | H/File/Import/Import_AutoCAD_dxf_dwg_File.htm | new |
| DXF Import layer assignment (Special Joints, Frames, NL Links, Shells, Solid) and entity mapping | 39 | absent | same | new |
| File > Export > AutoCAD .dxf/.dwg | 39 | absent | H/File/Export/Export_AutoCAD_dxf_dwg_File.htm | G35 |
| Other imports (NASTRAN, STAAD, CIS/2, IFC, Excel, Access) | 39 | absent (no import beyond S2K/SDB) | H/File/Import/ | G35 (IFC blocked) |
| Web-activated licensing (installation wizard, NetKey, WlmAdmin) | 35 | refused by design | not in v19 help | G55 |

### 2.11 Design
| Feature | Videos | Status | v19 help | Ledger |
| --- | --- | --- | --- | --- |
| Steel design code AISC360-05/IBC2006 (Steel Frame Design Preferences with Direct Analysis rows) | 1 | partial: AISC 360-10 and EC3 only | not searched | new (G20 is 360-16; G26 preference rows) |
| Automatic concrete shell design (N11, N12, NDes, ASt, Fc, Sc) with design combinations | 31 | absent | Shell_Section_Data_Form.htm | new |

Rows in part 2: 191 ("new": 156; with a ledger id: 35).

## 3. Inputs that are accepted and not used

1. The area edge constraint (video 8): read from an S2K file and stored,
   used by neither the solver nor the windows, with no message. Two meshes
   that do not share joints along an edge run as two unconnected parts.
   Ledger D73.
2. Stored as a type name only, with nothing behind it: the load pattern
   types Wave (video 17), Wind (video 18) and Prestress (video 11); the frame
   section type "SD" of the Section Designer (video 6); the layered shell
   type (video 16). For these the readers did not establish whether a
   message appears when such an input is analyzed; that is to be checked
   when their area is planned.

Not a gap: frame tension and compression limits are not applied in a linear
case (video 9). The video shows the vendor program doing the same.

## 4. Versions

| Version on screen | Videos | Count |
| --- | --- | --- |
| v15.0.0 | 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 20, 21, 22, 26, 30, 34 | 17 |
| v15.1.0 | 16, 25, 27, 36 | 4 |
| v16.0.0, v16.0.2, v16.1.0 | 17, 33, 37; 28; 14, 23 | 6 |
| v17 (an installer only) | 35 | 1 |
| v19.0.0, v19.1.0, v19.2.1 | 18; 15, 24; 31 | 4 |
| not read (title bar cropped or hidden) | 7, 13, 19, 29, 32, 38, 39 | 7 |

No title bar shows a version above v19.2.1. The seven videos whose version
could not be read look newer than v19 by their dialogs and menus; that is a
reading of the frames, not a version number. Features that need a form of
those newer versions: Foundation Properties and Assemblies (in the v24 help
only), Merge Analysis Results (in neither help) and the advanced solver
options of the same video.

## 5. Unresolved

- Rows still not determined: the Set Analysis Options form (available
  degrees of freedom); the cable load commands Gravity, Distributed and
  Temperature; the buttons of the concrete check form; the parameter sets of
  the pushover curve window; the table catalogue of the vendor program (45
  to 48 model tables and 9 result tables) against the 19 tables of the API,
  one by one.
- Readings that contradict each other and were not settled: whether a
  nonlinear modal history counts as present (one reader) or absent (two
  readers; the tracker has it as G41 and D26, refused on native import);
  whether drawing a one-joint or two-joint link exists as a draw tool.
- Forms that were asked for and never appeared within the frame limits,
  among them: the Snap To menu, the Section Designer's interaction-surface
  and moment-curvature windows, the wave characteristics form, the exposure
  widths form of the wind load, the response spectrum generation tabs, the
  damper and isolator property forms, the hysteretic damping form of the
  steady-state case, the page setup form of the report, the filter operator
  list of the table format window, and several right-click menus.
- Versions not read: videos 7, 13, 19, 29, 32, 38, 39.
