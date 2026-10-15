use sap_model::*;
use sap_format::{sdb::native::pack_model_to_native_sdb,extract_sdb_to_model,parse_s2k};
use sap_api::cSapModel;
fn rt(m:&SapModel)->SapModel {extract_sdb_to_model(&pack_model_to_native_sdb(m).unwrap()).unwrap()}
fn main(){
 let mut m=SapModel::default();m.add_point("A",0.,0.,0.);m.add_point("B",10.,0.,0.);m.add_frame("F","A","B","FSEC1");
 m.project_info.insert("Owner".into(),"Do not lose".into());m.points["B"].spring_coupled=Some([2.;21]);m.points["B"].spring_is_local=true;m.points["B"].local_axes_matrix=Some([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]);m.frame_sections["FSEC1"].file_name=Some("own.pro".into());m.materials["A992Fy50"].e_ortho=Some([29000.,22000.,13000.]);
 let r=rt(&m);println!("LOSS project_info={:?} coupled={:?} localaxes={:?} ortho={:?} section_file={:?}",r.project_info,r.points["B"].spring_coupled,r.points["B"].local_axes_matrix,r.materials["A992Fy50"].e_ortho,r.frame_sections["FSEC1"].file_name);
 let mut bad=m.clone();bad.frames["F"].point_j="MISSING".into();bad.frames["F"].section="MISSING".into();let r=rt(&bad);println!("INVALID_REF export OK point_j={} section={}",r.frames["F"].point_j,r.frames["F"].section);
 let mut p=LinkProp{name:"G".into(),link_type:LinkPropType::Gap,nonlinear_data_complete:false,..LinkProp::default()};p.dof_nonlinear[0]=true;p.k_nl[0]=100.;p.param1[0]=1.;m.link_props.insert("G".into(),p);let r=rt(&m);println!("COMPLETENESS before={} after={}",m.link_props["G"].nonlinear_data_complete,r.link_props["G"].nonlinear_data_complete);
 let mut api=cSapModel::default();api.prop_solid().set_prop("SSEC1","4000Psi",0.,0.,0.,true);api.inner.add_point("A",0.,0.,0.);let(status,id)=api.solid_obj().add_by_point(std::array::from_fn(|_|"A".into()),"SSEC1","ZERO");println!("DEGENERATE_SOLID status={status} id={id}");
 let corners=[[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.],[0.,0.,1.],[1.,0.,1.],[1.,1.,1.],[0.,1.,1.]];
 for (i,c) in corners.iter().enumerate(){api.inner.add_point(format!("C{i}"),c[0],c[1],c[2]);}
 api.solid_obj().add_by_point(std::array::from_fn(|i|format!("C{i}")),"SSEC1","CUBE");
 api.solid_obj().set_load_temperature("CUBE","DEAD",f64::MAX,true);api.inner.case_results.insert("OLD".into(),CaseResults::default());let status=api.solid_obj().set_load_temperature("CUBE","DEAD",f64::MAX,false);println!("TEMP_OVERFLOW status={status} stored={} results={}",api.inner.solids["CUBE"].loads_temp["DEAD"],api.inner.case_results.len());
 let mut api2=cSapModel::default();let mut history=LoadCase::linear_static("NLH","DEAD",1.);history.case_type=LoadCaseType::NonlinearHistory;let status=api2.load_cases().set_definition(history);println!("NONLINEAR_HISTORY_API status={status}");let mut continuation=LoadCase::linear_static("CONT","DEAD",1.);continuation.case_type=LoadCaseType::NonlinearStatic;continuation.initial_case=Some("DEAD".into());let status=api2.load_cases().set_definition(continuation);println!("CONTINUATION_API status={status}");
 let source="TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0 VendorTag=\"keep\"\nTABLE: \"OPAQUE VENDOR\"\n Value=21\n";let source_model=parse_s2k(source).unwrap();let r=rt(&source_model);println!("OPAQUE_LOSS source_metadata={} exported_metadata={}",source_model.s2k_preservation.is_some(),r.s2k_preservation.is_some());
}
