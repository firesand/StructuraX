# SAP2000 v19 File Formats & Section Property Databases Technical Specification

This document provides the reverse-engineered binary and text format specifications for **CSI SAP2000 v19.0.0**, derived from static analysis and IL/native disassembly of:
- `/home/edo/REA/extracted/files/sap2000.exe` (PE32 `.NET` assembly, obfuscated top-level application, `cFileIO` at `TypeDef 0x02000080 qq`, `.pro` reader at `0x0600a614`–`0x0600a61b`, and `.s2k` / database table engine at `0x06000cd1`–`0x06001cf3`)
- `/home/edo/REA/extracted/files/sapsharedstructures.dll` (PE32 `.NET` assembly defining 856 `CSI.CSICommon.IFileIO` model structs in namespace `CSIMDIForm.SAP2000Form`)
- `/home/edo/REA/extracted/files/csicommon.dll` (PE32 `.NET` assembly defining `CSI.CSICommon.IFileIO` and `CSI.CSICommon.cBinaryStream`)
- `/home/edo/REA/extracted/files/sdbthumbnailviewer.dll` (PE32 `.NET` Shell preview handler reading the `.sdb` outer container)
- `/home/edo/REA/extracted/files/sapimport.dll` (PE32 `.NET` importer for external formats and P/Invoke wrapper for `saptran.dll`)
- `/home/edo/REA/extracted/files/saptran.dll` (32-bit native Lahey Fortran 77 DLL `TOPDLL.FOR` translating legacy SAP90/SAP2000 v6/v7 text models)
- `/home/edo/REA/extracted/files/*.pro`, `*.sdb`, `csitext01.xml`, `csimateriallibrary*.xml`, `csimateriallibrary.xsd`, and `csidefaulttablenamefile.xsd`

All binary layouts have been verified against the extracted SAP2000 v19 distribution:
- **336 / 336 `.sdb` files** in `/home/edo/REA/extracted/files/` parse from byte `0` to the final byte of the decompressed `gzip` stream (`0` bytes remaining) using `/home/edo/.gemini/antigravity/brain/93a71218-fc85-41f1-84cc-d6032276e1c4/scratch/parse_sdb.py` and the extracted 856-struct schema table `/home/edo/.gemini/antigravity/brain/93a71218-fc85-41f1-84cc-d6032276e1c4/scratch/sapshared_writeself.json`.
- **All 23 `.pro` section property databases** (`aisc14.pro`, `euro.pro`, `sections8.pro`, `sections.pro`, `aisc.pro`, `aa6061_t6.pro`, etc.) have been verified against the `sap2000.exe` reader IL and AISC 14th Edition / Eurocode published tables.
- **All 2,292 `.s2k` (`$2k`) database table schemas** and field lists have been extracted to `/home/edo/.gemini/antigravity/brain/93a71218-fc85-41f1-84cc-d6032276e1c4/scratch/s2k_tables_and_fields.json`.

---

## 1. Architecture & Component Overview

| Binary / File | Type | Primary Role | Key Entry Points / Types |
|---|---|---|---|
| `sap2000.exe` | Managed `.NET` (`x86`) | Main executable; orchestrates `.sdb` I/O, `.pro` section database reader, and `.s2k`/`$2k`/XML/Excel/Access database table import/export | `qq` (`0x02000080`): `.sdb` reader/writer (`0x06001dc9`–`0x06001dfe`)<br>`ao` (`0x02000049`): `.s2k`/DB table engine (`0x06000cd1`–`0x06001cf3`)<br>`hy` (`0x0200018b`): `.pro` reader (`0x0600a614`–`0x0600a61b`, `0x0600a5e5`) |
| `sapsharedstructures.dll` | Managed `.NET` (`AnyCPU`) | Defines all 856 in-memory model data structures implementing `CSI.CSICommon.IFileIO` (`ReadSelf`, `WriteSelf`, `ClassID`, `ClassVersion`, `Initialize`, `Clone`) | `CSIMDIForm.SAP2000Form.S_POINTOBJECTS`, `S_LINEOBJECTS`, `S_AREAOBJECTS`, `S_SOLIDOBJECTS`, `S_NLLINKOBJECTS`, `S_Material`, `SECTIONARRAY`, `AREASECTIONARRAY`, `LOADPATTERNARRAY`, `ANALYSISCASEARRAY`, `COMBOARRAY`, etc. |
| `csicommon.dll` | Managed `.NET` (`AnyCPU`) | Implements `CSI.CSICommon.cBinaryStream` (underlying `System.IO.BinaryReader` / `BinaryWriter` + `GZipStream` serialization primitives) and `IFileIO` | `CSI.CSICommon.cBinaryStream` (`0x0200001c`)<br>`CSI.CSICommon.IFileIO` (`0x0200001e`) |
| `sdbthumbnailviewer.dll` | Managed `.NET` (`AnyCPU`) | Windows Explorer thumbnail handler for `.sdb` files; parses outer 52-byte header and extracts embedded PNG | `SDBThumbnailViewer.SDBThumbnailViewer.GetVisibleThumbnailImage` (`0x0600003a`) |
| `sapimport.dll` | Managed `.NET` (`x86`) | Imports StruCAD*3D (`cImportStruCAD3D`), STAAD/GTSTRUDL (`cImportSTAAD`), RISA-3D (`cImportRISA`), SACS (`cImportSACS`), LandXML (`cImportAlignmentFile`), and invokes `saptran.dll` | `SAPImport.cSAPTranslator.SAPTRAN` (`P/Invoke` into `saptran.dll`) |
| `saptran.dll` | Native PE32 (`x86`, Lahey Fortran 77) | Translates legacy SAP90 v5.5 input files into SAP2000 v7.40 `.s2k` block-keyword text files | Export #1 `SAPTRAN` (`RVA 0x00001000`), compiled from `D:\ Sap2000v7\Saptran\DLL\TOPDLL.FOR` |

---

## 2. `.sdb` Binary Model File Specification

### 2.1 Outer Container Layout
Every SAP2000 v19 `.sdb` file consists of a 48-byte ASCII identification header, a 4-byte little-endian PNG byte length, an embedded PNG thumbnail image, and a standard RFC 1952 `gzip` compressed stream starting immediately after the PNG image and continuing to EOF.

Verified in `sap2000.exe` (`qq::b(0x06001dc9)` writer and `qq::a(0x06001dca)` reader) and `SDBThumbnailViewer.dll` (`0x0600003a`):

| Byte Offset | Size (Bytes) | Type | Field Name in `sap2000.exe` | Description / Example (`example_1_001.sdb`) |
|---|---|---|---|---|
| `0x00` | `16` | `ASCII [u8; 16]` | `NameInFile` | Space-padded program identifier: `"SAP2000         "` (or `"SAP2000E        "` for evaluation models) |
| `0x10` | `16` | `ASCII [u8; 16]` | `ProgramVersionInFile` | Space-padded program version string: `"19.0.0          "` |
| `0x20` | `16` | `ASCII [u8; 16]` | `FileVersionInFile` | Space-padded file format version string (`f32` parsed via `Val()`): `"19.00           "` |
| `0x30` | `4` | `i32` LE (`u32`) | `ThumbnailLength` | Length in bytes $L_{\text{png}}$ of the embedded PNG thumbnail image (`0` if no thumbnail; `4438` (`0x00001156`) in `example_1_001.sdb`) |
| `0x34` | $L_{\text{png}}$ | `[u8; L_png]` | `ThumbnailPNG` | Standard PNG image (`89 50 4E 47 0D 0A 1A 0A ... IEND`) |
| `0x34 + L_png` | $\text{EOF} - (0x34 + L_{\text{png}})$ | `GZipStream` | `CompressedModelStream` | Standard RFC 1952 `gzip` stream (`1F 8B 08 00 ...`) decompressing to the `cBinaryStream` byte stream (`70,245` bytes in `example_1_001.sdb`) |

> [!NOTE]
> Legacy `.sdb` files prior to v12 (`FileVersionInFile < 12.0`) used Microsoft Compound File Binary (OLE2 Structured Storage `D0 CF 11 E0 A1 B1 1A E1`). SAP2000 v19 rejects `.sdb` files with `FileVersionInFile < 12.0` or `> 19.0` (`qq::a(0x06001dca)` at `+00bd..+00eb`). All v12–v19 `.sdb` files use the container above.

---

### 2.2 Inner `cBinaryStream` Wire Encoding (`csicommon.dll`)
Once the `gzip` payload at `0x34 + L_png` is decompressed, the resulting byte stream is a sequential `.NET` `System.IO.BinaryReader` stream managed by `CSI.CSICommon.cBinaryStream` (`TypeDef 0x0200001c` in `csicommon.dll`).

Crucially, **field names (`ItemID` strings passed to `Read`/`Write` in IL) are NOT written to the binary stream**—they are ignored by `cBinaryStream` in binary mode. Only the values and `IFileIO` struct headers (`ClassID` + `ClassVersion`) are serialized on the wire.

#### 2.2.1 Primitive Wire Types
All multi-byte numeric types are **little-endian**:

| Type Code | `.NET` Type | `cBinaryStream` Method | Wire Encoding |
|---|---|---|---|
| `bool` | `System.Boolean` | `Read(ref bool)` / `Write(bool)` | 1 byte (`u8`): `0x00` = `false`, `0x01` = `true` |
| `u1` | `System.Byte` | `Read(ref byte)` / `Write(byte)` | 1 byte (`u8`) |
| `i2` | `System.Int16` | `Read(ref short)` / `Write(short)` | 2 bytes (`i16` LE) |
| `i4` | `System.Int32` (and `Enum`) | `Read(ref int)` / `Write(int)` | 4 bytes (`i32` LE) |
| `i8` | `System.Int64` | `Read(ref long)` / `Write(long)` | 8 bytes (`i64` LE) |
| `r4` | `System.Single` | `Read(ref float)` / `Write(float)` | 4 bytes (IEEE-754 `f32` LE) |
| `r8` | `System.Double` | `Read(ref double)` / `Write(double)` | 8 bytes (IEEE-754 `f64` LE) |
| `string` | `System.String` | `Read(ref string)` / `Write(string)` | `.NET BinaryWriter` 7-bit LEB128 unsigned byte length prefix followed by UTF-8 bytes. **Null strings** are written as the 16-byte sequence `0F 7E 4E 6F 74 68 69 6E 67 53 74 72 69 6E 67 7E` (`"\x0f~NothingString~"`), which `Read(ref string)` converts back to `""` |
| `nullable_bool` | `Nullable<bool>` | `Read(ref bool?)` | `i32` LE flag (`-1` = `None`; `0` = `Some` followed by 1-byte `bool`) |
| `nullable_i4` | `Nullable<int>` | `Read(ref int?)` | `i32` LE flag (`-1` = `None`; `0` = `Some` followed by 4-byte `i32` LE) |
| `nullable_r8` | `Nullable<double>` | `Read(ref double?)` | `i32` LE flag (`-1` = `None`; `0` = `Some` followed by 8-byte `f64` LE) |

#### 2.2.2 Primitive Array & `List<T>` Wire Types
All primitive 1D arrays (`T[]`), `List<T>`, and multi-dimensional arrays (`T[,]`, `T[,,]`, `T[,,,]`) begin with a **1-byte presence flag** (`bool` `0x00` or `0x01`):
- **1D Primitive Array / `List<T>` (`arr_<prim>`)**:
  - `u8` `has_data`: if `0x00`, array is null/empty (0 additional bytes).
  - If `0x01`: `i32` LE `count` ($N \ge 0$), followed immediately by $N$ elements of type `<prim>`.
  - *Note*: In VB.NET, 1-based arrays allocated with `ReDim arr(N)` have CLR length `N + 1` (elements `0..N`), so `cBinaryStream.Write(T[])` writes `count = N + 1` elements where index `0` is the default/unused slot and indices `1..N` hold the 1-based items.
- **2D Primitive Array (`arr2_<prim>`, `T[,]`)**:
  - `u8` `has_data`: if `0x00`, stop.
  - If `0x01`: `i32` LE `count0` ($N_0$), `i32` LE `count1` ($N_1$), followed by $N_0 \times N_1$ elements of `<prim>` in row-major order (`for i in 0..N0 { for j in 0..N1 { ... } }`).
- **3D Primitive Array (`arr3_<prim>`, `T[,,]`)**:
  - `u8` `has_data`: if `0x00`, stop.
  - If `0x01`: `i32` LE `count0` ($N_0$), `i32` LE `count1` ($N_1$), `i32` LE `count2` ($N_2$), followed by $N_0 \times N_1 \times N_2$ elements of `<prim>` in row-major order.
- **4D Primitive Array (`arr4_<prim>`, `T[,,,]`)**:
  - `u8` `has_data`: if `0x00`, stop.
  - If `0x01`: `i32` LE `count0`, `count1`, `count2`, `count3`, followed by $N_0 \times N_1 \times N_2 \times N_3$ elements of `<prim>`.

#### 2.2.3 Structured Object (`IFileIO`) Wire Types
Every model structure in `sapsharedstructures.dll` implements `CSI.CSICommon.IFileIO`:
- **Single Struct (`struct`, `cBinaryStream.WriteStr(string, IFileIO)` / `ReadStr(string, ref IFileIO)`)**:
  1. `u8` `has_data`: if `0x00`, struct is null/uninitialized (0 additional bytes).
  2. If `0x01`:
     - `string` `class_id` (7-bit LEB128 length + UTF-8 string, e.g. `"CSIMDIForm.SAP2000Form+S_POINTOBJECTS"`).
     - `i32` LE `class_version` (e.g. `1900` = `0x0000076c` for SAP2000 v19.0.0).
     - Struct body: exact sequence of fields written by `<StructType>.WriteSelf(cBinaryStream)`.
- **1D Struct Array (`arr_struct`, `cBinaryStream.WriteStr<T>(string, T[], int Count)` / `ReadStr<T>(string, ref T[], int Count)`)**:
  1. `u8` `has_data`: if `0x00`, array is empty (0 additional bytes).
  2. If `0x01`:
     - `i32` LE `count` ($N$).
     - Followed by $N$ consecutive **Single Struct (`struct`)** encodings (each starting with its own `u8` `0x01` + `string class_id` + `i32 class_version` + body, corresponding to `arr[1] .. arr[N]`!).
     - *Important*: Unlike primitive arrays (which serialize index `0`), `WriteStr<T>(string, T[], int Count)` only serializes the **`Count` active 1-based elements `arr[1..Count]`**!
- **2D Struct Array (`arr2_struct`, `cBinaryStream.WriteStr2<T>(string, T[,], int Count0, int Count1)`)**:
  1. `u8` `has_data`: if `0x00`, array is empty (0 additional bytes).
  2. If `0x01`:
     - `i32` LE `count0` ($N_0$).
     - `i32` LE `count1` ($N_1$).
     - Followed by $N_0 \times N_1$ consecutive **Single Struct (`struct`)** encodings (`arr[1..N0, 1..N1]`).

---

### 2.3 Exact Top-Level `.sdb` Stream Sequence (`qq` in `sap2000.exe`)
When `qq::b(0x06001dc9)` writes or `qq::a(0x06001dca)` reads the decompressed `cBinaryStream`, it executes a fixed sequence of **21 helper methods** (`s`, `r`, `u`, `t`, `q`, `p`, `o`, `n`, `m`, `l`, `k`, `j`, `i`, `h`, `g`, `d`, `c`, `b`, `a`, `f`, `e`).

Every top-level field below is written in the exact order shown. Full verification across all 336 `.sdb` files in `/home/edo/REA/extracted/files/` is implemented in `/home/edo/.gemini/antigravity/brain/93a71218-fc85-41f1-84cc-d6032276e1c4/scratch/parse_sdb.py`.

#### Section 1: File Header (`s` — `0x06001dcd` / `0x06001deb`)
1. `string` `NameInFile` (`"SAP2000"` or `"SAP2000E"`)
2. `string` `ProgramVersionInFile` (`"19.0.0"`)
3. `r4` `FileVersionInFile` (`19.0f32`)
4. `string` `ProgLevel` (`"ADVANCED"`, `"ULTIMATE"`, `"PLUS"`, `"BASIC"`, etc.)
5. `string` `LicenseNum` (`"3010*16..."`)
6. `bool` `LicenseOS` (Offshore license flag)
7. `bool` `LicenseSC` (Staged Construction license flag)
8. `bool` `LicenseBR` (Bridge license flag)
9. `bool` `LicenseHT` (Heat Transfer license flag)

#### Section 2: Project Startup & Preferences (`r` — `0x06001dce` / `0x06001dec`)
1. `struct` `ProjectStartUp` (`CSIMDIForm.SAP2000Form+S_ProjectStartUp` — contains `m_Units` (`i4`), active degrees of freedom `m_ux..m_rz`, solver options, design code strings `m_SteelCode`, `m_ConcCode`, `m_AlumCode`, `m_ColdCode`, `m_BridgeCode`, etc.)
2. `arr_struct` `ProjectInfo` (`ProjectInfoData[]`: key-value project metadata such as `"Company Name"`, `"Model Name"`, `"Model Description"`, `"ProjectNumber"`, `"Engineer"`, etc.)
3. `struct` `UserCommentSessionLog` (`S_UserCommentSessionLog`)
4. `struct` `Dimensions` (`S_Dimensions`)
5. `struct` `SpringOptions` (`S_SpringOptions`)
6. `struct` `MassOptions` (`S_MassOptions`)
7. `arr_struct` `MassSource` (`MASSSOURCEARRAY[]`)
8. `i4` `DefaultMassSource` (1-based index into `MassSource`)
9. `i4` `ModalCase`
10. `arr_struct` `CoordSystem` (`COORDSYSTEMARRAY[]`: coordinate systems and grid lines, index 1 = `"GLOBAL"`)
11. `i4` `PresentCoordSystem` (1-based index into `CoordSystem`)

#### Section 3: Reference Lines & Named Property Modifiers (`u` — `0x06001dcb` / `0x06001de9`)
1. `arr_struct` `GeneralReferenceLines` (`S_GeneralReferenceLines[]`)
2. `arr_struct` `LineElementNamedPropMod` (`S_LineElementNamedPropMod[]`)
3. `arr_struct` `AreaElementNamedPropMod` (`S_AreaElementNamedPropMod[]`)
4. `arr_struct` `SolidElementNamedPropMod` (`S_SolidElementNamedPropMod[]`)
5. `arr_struct` `LinkNamedPropMod` (`S_LinkNamedPropMod[]`)

#### Section 4: Constraints, Patterns, Weld/Bolt & Materials (`t` — `0x06001dcc` / `0x06001dea`)
1. `arr_struct` `Constraint` (`TYConstraint[]`)
2. `arr_struct` `PatternName` (`TYPattern[]`: joint patterns, index 1 = `"Default"`)
3. `arr_struct` `Weld` (`S_Weld[]`)
4. `arr_struct` `Bolt` (`S_Bolt[]`)
5. `arr_struct` `Material` (`S_Material[]`: material definitions, e.g. `"4000Psi"`, `"A992Fy50"`)

#### Section 5: Frame, Cable, Tendon, Area, Solid & Link Sections (`q` — `0x06001dcf` / `0x06001ded`)
1. `bool` `StagedConstructionInFile`
2. `arr_struct` `RebarList` (`S_RebarList[]`: standard reinforcing bar sizes `#3`..`#18`, `10M`..`55M`, etc.)
3. `arr_struct` `Section` (`SECTIONARRAY[]`: **Frame section properties**)
4. `arr_struct` `CableSection` (`S_CableSection[]`)
5. `arr_struct` `TendonSection` (`S_TendonSection[]`)
6. `arr_struct` `NamedLineReleases` (`S_NamedLineReleases[]`)
7. `arr_struct` `AreaSection` (`AREASECTIONARRAY[]`: **Shell / Plane / Asolid section properties**)
8. `arr_struct` `SolidSection` (`S_SolidSection[]`)
9. `arr_struct` `LinkProps` (`TYNLLinkProps[]`: **Link / Support properties**)
10. `arr_struct` `SoilProfile` (`SOILPROFILEARRAY[]`)
11. `arr_struct` `Footing` (`FOOTINGARRAY[]`)
12. `arr_struct` `HingeProps` (`S_HingeProps[]`)
13. `struct` `MediaControl` (`S_MediaControl`)

#### Section 6: Functions, Load Patterns, Load Cases, Combinations (`p` — `0x06001dd0` / `0x06001dee`)
1. `arr_struct` `RSFunc` (`TYRSFunc[]`: Response Spectrum & Time History functions)
2. `arr_struct` `LoadPattern` (`LOADPATTERNARRAY[]`: Load Patterns, e.g. `"DEAD"`, `"LIVE"`)
3. `arr_struct` `TendonObjects` (`S_TENDONOBJECTS[]`)
4. `arr_struct` `AnalysisCase` (`ANALYSISCASEARRAY[]`: Load Cases, e.g. `"DEAD"`, `"MODAL"`)
5. `arr_struct` `Combo` (`COMBOARRAY[]`: Load Combinations)
6. `arr_struct` `ExplicitOptions` (`S_ExplicitOptions[]`)
7. `struct` `AutoComboOptions` (`S_AutoComboOptions`)
8. `bool` `AddingBridgeAutoCombos`
9. `arr_bool` `DefaultDesignCombos` (`bool[9]`)
10. `arr2_i4` `CodeMovedGeneratedCombos` (`int[,]`)
11. `arr_struct` `AutoSaveLongTerm` (`S_AutoSaveLongTerm[]`)
12. `arr_struct` `View` (`S_View[]`: saved 3D/2D camera views)
13. `arr_struct` `GeneralDispl` (`S_GeneralDispl[]`: generalized displacements)
14. `arr_struct` `SectionCut` (`S_SectionCut[]`: section cuts)

#### Section 7: Groups, Pushover, Function Plots & Named Sets (`o` — `0x06001dd1` / `0x06001def`)
1. `arr_struct` `Groups` (`GROUPARRAY[]`: object groups, index 1 = `"ALL"`)
2. `struct` `PushOverParams` (`S_PushOverParams`)
3. `arr_struct` `FuncPlot` (`S_FuncPlot[]`)
4. `arr_struct` `TablesNamedSet` (`S_TablesNamedSet[]`)
5. `arr_struct` `VirtualWorkNamedSet` (`S_VirtualWorkNamedSet[]`)
6. `arr_struct` `PSDNamedSet` (`S_PSDNamedSet[]`)
7. `arr_struct` `SteadyStateNamedSet` (`S_SteadyStateNamedSet[]`)
8. `arr_struct` `THNamedSet` (`S_THNamedSet[]`)
9. `arr_struct` `PushoverNamedSet` (`S_PushoverNamedSet[]`)
10. `arr_struct` `Color` (`COLORARRAY[]`: color palette)
11. `struct` `DisplayOptions` (`S_DisplayOptions`)
12. `struct` `Set2dView` (`S_Set2dView`)
13. `struct` `OtherGraphics` (`S_OtherGraphics`)

#### Section 8: Offshore / API Wave, Wind, Current & Fatigue (`n` — `0x06001dd8` / `0x06001df6`)
1. `arr_struct` `WaveChars` (`S_WaveChars[]`)
2. `arr_struct` `WaveWind` (`S_WaveWind[]`)
3. `arr_struct` `WaveCurrent` (`S_WaveCurrent[]`)
4. `arr_struct` `WaveGrowth` (`S_WaveGrowth[]`)
5. `arr_struct` `Wave` (`S_Wave[]`)
6. `arr_struct` `JointWaveMod` (`S_JointWaveMod[]`)
7. `arr_struct` `LineCDCM` (`S_LineCDCM[]`)
8. `arr_struct` `LineWaveMod` (`S_LineWaveMod[]`)
9. `bool` `ConsiderWaveAddedMass`
10. `bool` `AdjustAddedMassAboveWater`
11. `i4` `WaveCaseForAddedMass`
12. `bool` `ConsiderFatigue`
    - **Conditional Branch (`if ConsiderFatigue == true` ONLY)**:
      - `struct` `APIFatigueParameters` (`S_APIFatigueParameters`)
      - `bool` `FatigueAPIResultsAvailable`
      - `bool` `FatigueAPIDefaultsSet`
      - `arr_struct` `FatigueAPIData` (`S_FatigueAPIData[]`)
      - `arr_struct` `FatigueAPIEnvelopes` (`S_FatigueAPIEnvelopes[]`)
    - *(When `ConsiderFatigue == false`, those 5 items are completely omitted from the stream!)*

#### Section 9: Design Overwrites & Design Options (`m` — `0x06001dd2` / `0x06001df0`)
1. `struct` `SteelOptions` (`S_SteelOptions`)
2. `struct` `ConcreteOptions` (`S_ConcreteOptions`)
3. `struct` `AlumOptions` (`S_AlumOptions`)
4. `struct` `ColdOptions` (`S_ColdOptions`)
5. `arr_struct` `SteelOverWrites` (`S_SteelOverWrites[]`)
6. `arr_struct` `ConcOverWrites` (`S_ConcOverWrites[]`)
7. `arr_struct` `AlumOverWrites` (`S_AlumOverWrites[]`)
8. `arr_struct` `ColdOverWrites` (`S_ColdOverWrites[]`)
9. `arr2_i4` `ListOfElementDesignSections` (`int[,]`)
10. `arr_i4` `DesignSectionTemp` (`int[]`)
11. `bool` `HaveOptimizedSinceAnalysis`
12. `arr_struct` `SteelDeflection` (`S_SteelDeflection[]`)
13. `bool` `APIPunchResultsAvailable`
14. `arr_struct` `APIPunchData` (`S_APIPunchData[]`)
15. `arr_struct` `APIPunchEnvelopes` (`S_APIPunchEnvelopes[]`)
16. `bool` `AllActiveSteelDesigned`
17. `bool` `AllActiveConcDesigned`
18. `bool` `AllActiveAlumDesigned`
19. `bool` `AllActiveColdDesigned`
20. `struct` `LastEnergyPlotSetup` (`S_LastEnergyPlotSetup`)
21. `arr_struct` `JointRespSpecNamedSet` (`S_JointRespSpecNamedSet[]`)

#### Section 10: Bridge Analysis & Bridge Design (`l` — `0x06001dd3` / `0x06001df1`)
1. `struct` `BridgeSelector` (`S_BridgeSelector`)
2. `struct` `BridgePrefs` (`S_BridgePrefs`)
3. `struct` `BridgeOptions` (`S_BridgeOptions`)
4. `struct` `BridgeParams` (`S_BridgeParams`)
5. `bool` `MoveLoadAllLanesSaved`
6. `r8` `MaxLengthDiscAlongBridge`
7. `arr_struct` `Bridge` (`S_Bridge[]`)
8. `arr_struct` `LayoutLine` (`S_LayoutLine[]`)
9. `arr_struct` `BridgeSection` (`S_BridgeSection[]`)
10. `arr_struct` `BridgeDiaphragm` (`S_BridgeDiaphragm[]`)
11. `arr_struct` `BridgeSuperelevation` (`S_BridgeSuperelevation[]`)
12. `arr_struct` `BridgePrestressPatterns` (`S_BridgePrestressPatterns[]`)
13. `arr_struct` `BridgeAbutment` (`S_BridgeAbutment[]`)
14. `arr_struct` `BridgeBent` (`S_BridgeBent[]`)
15. `arr_struct` `BridgeBearing` (`S_BridgeBearing[]`)
16. `arr_struct` `BridgeRestrainer` (`S_BridgeRestrainer[]`)
17. `arr_struct` `BridgeFoundationSpring` (`S_BridgeFoundationSpring[]`)
18. `arr_struct` `BridgeLineLoad` (`S_BridgeLineLoad[]`)
19. `arr_struct` `BridgeAreaLoad` (`S_BridgeAreaLoad[]`)
20. `arr_struct` `BridgeTempGradientLoad` (`S_BridgeTempGradientLoad[]`)
21. `arr_struct` `BridgeLossDefs` (`S_BridgeLossDefs[]`)
22. `arr_struct` `Lanes` (`S_Lanes[]`)
23. `arr_struct` `Vehicles` (`S_Vehicles[]`)
24. `bool` `StandardTypesAdded`
25. `arr_struct` `Vehicles General` (`S_VehiclesGeneral[]`)
26. `arr_struct` `VehicleClasses` (`S_VehicleClasses[]`)
27. `arr_struct` `BridgeResp` (`S_BridgeResp[]`)
28. `arr_struct` `BridgeSuperDesReq` (`S_BridgeSuperDesReq[]`)
29. `arr_struct` `BridgeSuperRatReq` (`S_BridgeSuperRatReq[]`)
30. `arr_struct` `BridgeSeismicDesReq` (`S_BridgeSeismicDesReq[]`)
31. `i4` `DisplayBridgeSuperDesReqItem`
32. `struct` `BridgeResultsDisplayData` (`S_BridgeResultsDisplayData`)
33. `struct` `BridgeUpdateData` (`S_BridgeUpdateData`)
34. `arr_struct` `ScheduledBridgeObjects` (`S_ScheduledBridgeObjects[]`)
35. `string` `BridgeScheduleFilePath`
36. `arr_struct` `TendonJack` (`TYTendonJack[]`)

#### Section 11: Point Objects (Joints) & Point Assignments (`k` — `0x06001dd4` / `0x06001df2`)
1. `arr_struct` `PointObjects` (`S_POINTOBJECTS[]`: **Joint coordinates & point objects**)
2. `arr_struct` `Restraint` (`TYRestraint[]`: **Joint restraint assignments**, `U1..R3`)
3. `arr_struct` `Spring` (`TYSpring[]`: Joint uncoupled/coupled springs)
4. `arr_struct` `JMass` (`TYJMass[]`: Joint added masses)
5. `arr_struct` `JLocalAxes` (`TYJLocalAxes[]`: Joint local axes)
6. `arr_struct` `PanelZone` (`TYPanelZone[]`: Joint panel zones)
7. `arr_struct` `JPattern` (`TYJPattern[]`: Joint pattern values)
8. `arr_struct` `PointsoilProfile` (`TYPointSoilProfile[]`)
9. `arr_struct` `PointFooting` (`TYPointFooting[]`)
10. `arr_struct` `JointMergeOpt` (`S_JointMergeOpt[]`)

#### Section 12: Point Loads (Joint Forces & Displacements) (`j` — `0x06001dd5` / `0x06001df3`)
1. `arr_struct` `JFLoad` (`TYJFLoad[]`: **Joint concentrated forces/moments**, `F1, F2, F3, M1, M2, M3`)
2. `arr_struct` `JDLoad` (`TYJDLoad[]`: **Joint ground displacements**, `U1, U2, U3, R1, R2, R3`)
3. `arr_struct` `PointTHLoad` (`TYPointTHLoad[]`)
4. `arr_struct` `PointHeatSource` (`S_PointHeatSource[]`)
5. `arr_struct` `PointTempConstant` (`S_PointTempConstant[]`)

#### Section 13: Line Objects (Frames / Cables) & Line Assignments (`i` — `0x06001dd6` / `0x06001df4`)
1. `arr_struct` `LineObjects` (`S_LINEOBJECTS[]`: **Frame/Cable connectivity & section assignment**)
2. `arr_struct` `LineHinges` (`S_LineHinges[]`)
3. `arr_struct` `LineEndOffsets` (`S_LineEndOffsets[]`)
4. `arr_struct` `LineEndSkew` (`S_LineEndSkew[]`)
5. `arr_struct` `LineInsertPoint` (`S_LineInsertPoint[]`)
6. `arr_struct` `LineLocalAxes` (`S_LineLocalAxes[]`)
7. `arr_struct` `LineMass` (`S_LineMass[]`)
8. `arr_struct` `LineMatTemp` (`S_LineMatTemp[]`)
9. `arr_struct` `LineSubdivide` (`S_LineSubdivide[]`)
10. `arr_struct` `LineMonitor` (`S_LineMonitor[]`)
11. `arr_struct` `LinePropMod` (`S_LinePropMod[]`)
12. `arr_struct` `LineRelease` (`S_LineRelease[]`: **Frame end releases & partial fixity**)
13. `arr_struct` `LineSpandrel` (`S_LineSpandrel[]`)
14. `arr_struct` `LineSpring` (`S_LineSpring[]`)
15. `arr_struct` `LineTC Limits` (`S_LineTCLimits[]`)
16. `arr_struct` `LineNonPrismatic` (`S_LineNonPrismatic[]`)
17. `arr_struct` `LineFireproofing` (`S_LineFireproofing[]`)

#### Section 14: Line Loads (Frame / Cable Loads) (`h` — `0x06001dd7` / `0x06001df5`)
1. `arr_struct` `LineGravityLoad` (`TYLineGravityLoad[]`)
2. `arr_struct` `LinePointLoad` (`TYLinePointLoad[]`: **Frame point loads**)
3. `arr_struct` `LineDistLoad` (`TYLineDistLoad[]`: **Frame distributed / trapezoidal loads**)
4. `arr_struct` `LineTempLoad` (`TYLineTempLoad[]`)
5. `arr_struct` `LinePrestressLoad` (`TYLinePrestressLoad[]`)
6. `arr_struct` `LineStrainLoad` (`TYLineStrainLoad[]`)
7. `arr_struct` `LineDefLoad` (`TYLineDefLoad[]`)
8. `arr_struct` `LineTargetForce` (`TYLineTargetForce[]`)
9. `arr_struct` `LinePorePressure` (`TYLinePorePressure[]`)
10. `arr_struct` `LineOpeningsWindLoad` (`S_LineOpeningsWindLoad[]`)
11. `arr_struct` `LineHeatSource` (`S_LineHeatSource[]`)
12. `arr_struct` `LineConvecBound` (`S_LineConvecBound[]`)
13. `arr_struct` `LineRadBound` (`S_LineRadBound[]`)

#### Section 15: Area Objects (Shells / Planes / Asolids) & Area Assignments (`g` — `0x06001dd9` / `0x06001df7`)
1. `arr_struct` `AreaObjects` (`S_AREAOBJECTS[]`: **Area connectivity & section assignment**)
2. `arr_struct` `AreaLocalAxes` (`TYAreaLocalAxes[]`)
3. `arr_struct` `AreaPropMod` (`TYAreaPropMod[]`)
4. `arr_struct` `AreaMass` (`TYAreaMass[]`)
5. `arr_struct` `AreaMatTemp` (`TYAreaMatTemp[]`)
6. `arr_struct` `AreaSpring` (`TYAreaSpring[]`)
7. `arr_struct` `AreaThickness` (`TYAreaThickness[]`)
8. `arr_struct` `AreaOffsets` (`TYAreaOffsets[]`)
9. `arr_struct` `AreaAutoMesh` (`TYAreaAutoMesh[]`)
10. `arr_struct` `AreaEdgeConstraint` (`TYAreaEdgeConstraint[]`)
11. `arr_struct` `AreaMonitor` (`TYAreaMonitor[]`)

#### Section 16: Area Loads (`d` — `0x06001ddc` / `0x06001dfa`)
1. `arr_struct` `AreaGravityLoad` (`TYAreaGravityLoad[]`)
2. `arr_struct` `AreaUniformLoad` (`TYAreaUniformLoad[]`: **Area uniform loads**)
3. `arr_struct` `AreaUniformToFrame` (`TYAreaUniformToFrame[]`)
4. `arr_struct` `AreaSurfacePressure` (`TYAreaSurfacePressure[]`: **Area surface pressure loads**)
5. `arr_struct` `AreaPorePressure` (`TYAreaPorePressure[]`)
6. `arr_struct` `AreaTempLoad` (`TYAreaTempLoad[]`)
7. `arr_struct` `AreaStrainLoad` (`TYAreaStrainLoad[]`)
8. `arr_struct` `AreaRotateLoad` (`TYAreaRotateLoad[]`)
9. `arr_struct` `AreaWindPressureLoad` (`S_AreaWindPressureLoad[]`)
10. `arr_struct` `AreaHeatSource` (`S_AreaHeatSource[]`)
11. `arr_struct` `AreaConvecBound` (`S_AreaConvecBound[]`)
12. `arr_struct` `AreaRadBound` (`S_AreaRadBound[]`)

#### Section 17: Solid Objects & Solid Assignments (`c` — `0x06001ddd` / `0x06001dfb`)
1. `arr_struct` `SolidObjects` (`S_SOLIDOBJECTS[]`: **8-node Solid connectivity & section assignment**)
2. `arr_struct` `SolidLocalAxes` (`TYSolidLocalAxes[]`)
3. `arr_struct` `SolidMatTemp` (`TYSolidMatTemp[]`)
4. `arr_struct` `SolidSpring` (`TYSolidSpring[]`)
5. `arr_struct` `SolidAutoMesh` (`TYSolidAutoMesh[]`)
6. `arr_struct` `SolidEdgeConstraint` (`TYSolidEdgeConstraint[]`)
7. `arr_struct` `SolidPropMod` (`S_SolidPropMod[]`)
8. `arr_struct` `SolidMass` (`TYSolidMass[]`)

#### Section 18: Solid Loads (`b` — `0x06001dde` / `0x06001dfc`)
1. `arr_struct` `SolidGravityLoad` (`TYSolidGravityLoad[]`)
2. `arr_struct` `SolidSurfacePressure` (`TYSolidSurfacePressure[]`)
3. `arr_struct` `SolidPorePressure` (`TYSolidPorePressure[]`)
4. `arr_struct` `SolidTempLoad` (`TYSolidTempLoad[]`)
5. `arr_struct` `SolidStrainLoad` (`TYSolidStrainLoad[]`)
6. `arr_struct` `SolidRotateLoad` (`TYSolidRotateLoad[]`)

#### Section 19: Link Objects & Link Assignments/Loads (`a` — `0x06001ddf` / `0x06001dfd`)
1. `arr_struct` `NLLinkObjects` (`S_NLLINKOBJECTS[]`: **1-joint or 2-joint Link connectivity & property assignment**)
2. `arr_struct` `NLLinkLocalAxes` (`TYNLLinkLocalAxes[]`)
3. `arr_struct` `LinkGravityLoad` (`TYLinkGravityLoad[]`)
4. `arr_struct` `LinkDefLoad` (`TYLinkDefLoad[]`)
5. `arr_struct` `LinkTargetForce` (`TYLinkTargetForce[]`)
6. `arr_struct` `LinkPropMod` (`S_LinkPropMod[]`)

#### Section 20: Database Table Customization & Results Setup (`f` — `0x06001dda` / `0x06001df8`)
1. `bool` `AllInputAccessDBTablesChosen`
2. `struct` `DBTableControl` (`S_DBTableControl`)
3. `struct` `CustomReportSetup` (`S_CustomReportSetup`)
4. `arr_struct` `ResultsSummarySetup` (`RESULTSUMMARYSETUPARRAY[]`)
5. `struct` `InDBFormat` (`S_InDBFormat`)
6. `string` `DBNameFile`
7. `arr_struct` `DBTableName` (`DBTABLENAMEARRAY[]`)
8. `bool` `AnyTableOrFieldNamesUserDefined`
9. `bool` `IsModelNew`
10. `arr_struct` `AutoDB` (`AUTODBARRAY[]`)
11. `bool` `AutoExportedAfterAnalysis`
12. `arr_struct` `DisplayNamedSet` (`S_DisplayNamedSet[]`)
13. `arr_struct` `LoadCaseTreeFilter` (`S_LoadCaseTreeFilter[]`)
14. `arr_struct` `GroupAssignTable` (`S_GroupAssignTable[]`)
15. `arr_struct` `SelectionList` (`S_SelectionList[]`)

#### Section 21: Model Log & Timestamp (`e` — `0x06001de0` / `0x06001dfe`)
1. `string` `ModelLog` (multi-line session log text)
2. `i4` `DateTimeStamp` (`Field:ff(0x4005c43)`, 4-byte `i32` LE timestamp, followed immediately by EOF of the decompressed stream)

---

### 2.4 Core `.sdb` Struct Field Schemas (`sapsharedstructures.dll`)
All 856 `IFileIO` struct definitions are stored in machine-readable JSON in `/home/edo/.gemini/antigravity/brain/93a71218-fc85-41f1-84cc-d6032276e1c4/scratch/sapshared_writeself.json`.

**Cross-Reference Indexing Convention**:
All integer references between top-level `.sdb` arrays are **1-based array indices** into the corresponding top-level `arr_struct` array:
- `m_point[1..2]` in `S_LINEOBJECTS` indexes `PointObjects[1..N]`
- `m_jn` in `TYRestraint`, `TYJFLoad`, `TYJDLoad`, `TYSpring`, `TYJMass` indexes `PointObjects[1..N]`
- `m_fn` in `S_LineRelease`, `TYLinePointLoad`, `TYLineDistLoad`, `S_LineLocalAxes` indexes `LineObjects[1..N]`
- `m_an` in `TYAreaUniformLoad`, `TYAreaSurfacePressure`, `TYAreaLocalAxes` indexes `AreaObjects[1..N]`
- `m_sanalysis` / `m_sdesign` in `S_LINEOBJECTS` indexes `Section[1..N]`
- `m_section` in `S_AREAOBJECTS` indexes `AreaSection[1..N]`
- `m_Mindex` in `SECTIONARRAY` and `AREASECTIONARRAY` indexes `Material[1..N]`
- `m_LC` in load structs (`TYJFLoad`, `TYLineDistLoad`, `TYAreaUniformLoad`, etc.) indexes `LoadPattern[1..N]`
- `m_csys` / `m_csysdef` indexes `CoordSystem[1..N]` (`1` = `"GLOBAL"`)

#### 2.4.1 `S_ProjectStartUp` (Units & Active DOFs)
| Order | Field Name | Wire Type | Description / Values |
|---|---|---|---|
| 1 | `m_ProgramName` | `string` | `"SAP2000"` |
| 2 | `m_Version` | `string` | `"19.0.0"` |
| 3 | `m_Units` | `i4` | **Units Code (`1..16`)**:<br>`1`=`lb, in, F` \| `2`=`lb, ft, F` \| `3`=`kip, in, F` \| `4`=`kip, ft, F`<br>`5`=`kN, mm, C` \| `6`=`kN, m, C` \| `7`=`kgf, mm, C` \| `8`=`kgf, m, C`<br>`9`=`N, mm, C` \| `10`=`N, m, C` \| `11`=`Tonf, mm, C` \| `12`=`Tonf, m, C`<br>`13`=`kN, cm, C` \| `14`=`kgf, cm, C` \| `15`=`N, cm, C` \| `16`=`Tonf, cm, C` |
| 4..9 | `m_ux`, `m_uy`, `m_uz`, `m_rx`, `m_ry`, `m_rz` | `bool` $\times 6$ | Active global degrees of freedom (`UX, UY, UZ, RX, RY, RZ`) |
| 10..20 | `m_SteelCode`, `m_ConcCode`, `m_AlumCode`, `m_ColdCode`, `m_BridgeCode`, `m_BridgeRatingCode`, `m_BridgeSeismicCode`, `m_SolverType`, `m_SolverProcessType`, `m_Force32BitSolver`, `m_StiffCase` | `string`/`i4`/`bool` | Design codes & solver configuration (`m_SolverType`: `0`=Standard, `1`=Advanced, `2`=Multi-threaded) |
| 21..26 | `m_NonlinearGeomType`, `m_HingeOption`, `m_AutoRegenHingesAfterImport`, `m_BridgeScheduleGUID`, `m_HingeLinkOption`, `m_LinkF0` | `i4`/`bool`/`string`/`r8` | Hinge & geometric nonlinearity options |

#### 2.4.2 `S_POINTOBJECTS` (`PointObjects[]` — Joint Coordinates)
| Order | Field Name | Wire Type | Description |
|---|---|---|---|
| 1 | `m_alive` | `bool` | `true` if active |
| 2 | `m_ID` | `string` | User-visible Joint ID (e.g. `"1"`, `"2"`) |
| 3..5 | `m_gx`, `m_gy`, `m_gz` | `r8` $\times 3$ | Global Cartesian coordinates $(X, Y, Z)$ in model length units |
| 6 | `m_SpecialPoint` | `bool` | Special joint flag |
| 7 | `m_RestraintPtr` | `i4` | 1-based index into `Restraint[]` (`0` if unrestrained) |
| 8 | `m_SpringPtr` | `i4` | 1-based index into `Spring[]` (`0` if none) |
| 9 | `m_MassPtr` | `i4` | 1-based index into `JMass[]` (`0` if none) |
| 10 | `m_LocalAxesPtr` | `i4` | 1-based index into `JLocalAxes[]` (`0` if default global) |
| 11 | `m_PanelZonePtr` | `i4` | 1-based index into `PanelZone[]` (`0` if none) |
| 12 | `m_ConstraintPtr` | `arr_i4` | 1-based indices into `Constraint[]` |
| 13..18 | `m_Selected`, `m_CoordSysType`, `m_JointCoordType`, `m_csysdef`, `m_CX`, `m_CY`, `m_Cz` | `bool`/`i4`/`r8` | Defining coordinate system (`m_csysdef`) and defining coordinates (`m_CX`, `m_CY`, `m_Cz`) |
| 19..42 | `m_GUID` (at field #36), `m_OriginalGX..GZ`, `m_TargetGX..GZ`, etc. | various | GUID, P-Delta / shape-finding target coordinates |

#### 2.4.3 `TYRestraint` (`Restraint[]` — Joint Restraints)
| Order | Field Name | Wire Type | Description |
|---|---|---|---|
| 1 | `m_alive` | `bool` | `true` if active |
| 2 | `m_jn` | `i4` | 1-based index into `PointObjects[]` |
| 3 | `m_res` | `arr_bool` (`len=7`) | `m_res[1..6]` = `[U1, U2, U3, R1, R2, R3]` (`true` = restrained / fixed) |

#### 2.4.4 `S_LINEOBJECTS` (`LineObjects[]` — Frame & Cable Connectivity)
| Order | Field Name | Wire Type | Description |
|---|---|---|---|
| 1 | `m_alive` | `bool` | `true` if active |
| 2 | `m_ID` | `string` | User-visible Frame/Cable ID (e.g. `"1"`, `"2"`) |
| 3 | `m_LineType` | `i4` | `0` = Straight Frame, `1` = Curved Frame, `2` = Cable |
| 4 | `m_point` | `arr_i4` (`len=5`) | `m_point[1]` = `JointI` (1-based index into `PointObjects`), `m_point[2]` = `JointJ` |
| 5 | `m_sanalysis` | `i4` | 1-based index into `Section[]` (analysis frame section) or `CableSection[]` |
| 6 | `m_sdesign` | `i4` | 1-based index into `Section[]` (design frame section) |
| 7..36 | `m_ReleasePtr` (#14), `m_EndOffsetPtr` (#8), `m_LocalAxesPtr` (#10), `m_PropModPtr` (#13), `m_NonPrismaticPtr` (#16), `m_GUID` (#23), `m_Length` (#34), `m_CentroidX..Z` (#31..33) | various | Pointers (1-based indices) into `LineRelease[]`, `LineEndOffsets[]`, `LineLocalAxes[]`, `LinePropMod[]`, `LineNonPrismatic[]`, etc. |

#### 2.4.5 `S_Material` (`Material[]` — Material Properties)
| Order | Field Name | Wire Type | Description |
|---|---|---|---|
| 1 | `m_alive` | `bool` | `true` if active |
| 2 | `m_mname` | `string` | Material name (e.g. `"4000Psi"`, `"A992Fy50"`, `"STEEL"`) |
| 3 | `m_mtype` | `i4` | Material type: `1`=Steel, `2`=Concrete, `3`=NoDesign/Other, `4`=Aluminum, `5`=ColdFormed, `6`=Rebar, `7`=Tendon |
| 4 | `m_symtype` | `i4` | Symmetry type: `0`=Isotropic, `1`=Orthotropic, `2`=Anisotropic, `3`=Uniaxial |
| 5 | `m_nTemps` | `i4` | Number of temperature-dependent property rows ($\ge 1$) |
| 6 | `m_t` | `arr_r8` | Temperatures `m_t[1..nTemps]` |
| 7 | `m_w` | `arr_r8` | Weight per unit volume $\gamma$ (`m_w[1..nTemps]`) |
| 8 | `m_m` | `arr_r8` | Mass per unit volume $\rho$ (`m_m[1..nTemps]`) |
| 9 | `m_e` | `arr2_r8` | Young's moduli $E_1, E_2, E_3$ (`[nTemps+1, 4]`: `m_e[t, 1..3]`) |
| 10 | `m_g` | `arr2_r8` | Shear moduli $G_{12}, G_{13}, G_{23}$ (`[nTemps+1, 4]`: `m_g[t, 1..3]`) |
| 11 | `m_u` | `arr2_r8` | Poisson's ratios $\nu_{12}, \nu_{13}, \nu_{23}$ (`m_u[t, 1..3]`) |
| 12 | `m_a` | `arr2_r8` | Thermal expansion coefficients $\alpha_1, \alpha_2, \alpha_3$ (`m_a[t, 1..3]`) |
| 13..18 | `m_fy`, `m_fu`, `m_fye`, `m_fue`, `m_fc`, `m_fcs` | `arr_r8` | Yield strength $F_y$, ultimate strength $F_u$, expected $F_{ye}, F_{ue}$, concrete $f'_c$ |

#### 2.4.6 `SECTIONARRAY` (`Section[]` — Frame Section Properties)
| Order | Field Name | Wire Type | Description |
|---|---|---|---|
| 1 | `m_alive` | `bool` | `true` if active |
| 2 | `m_sname` | `string` | Section name (e.g. `"W18X35"`, `"FSEC1"`) |
| 3 | `m_SNInFile` | `string` | Name inside `.pro` file if imported from file |
| 4 | `m_FileName` | `string` | `.pro` file name if imported |
| 5 | `m_Mindex` | `i4` | 1-based index into `Material[]` |
| 6 | `m_type` | `i4` | Section definition type (`1`=From `.pro` file, `2`=User/Analytic, `3`=Auto-select list, `4`=Section Designer, `5`=Nonprismatic) |
| 7 | `m_sh` | `i4` | **Shape Code**: `1`=I/Wide Flange, `2`=Channel, `3`=Tee, `4`=Angle, `5`=Double Angle, `6`=Box/Tube, `7`=Pipe, `8`=Rectangular, `9`=Circle, `10`=General, `11`=Double Channel, `12`..=Cold-Formed/SD/Variable |
| 8..16 | `m_t3`, `m_t2`, `m_TF`, `m_TW`, `m_t2b`, `m_tfb`, `m_dis`, `m_rad`, `m_ld` | `r8` $\times 9$ | Cross-section geometric dimensions ($t_3, t_2, t_f, t_w, t_{2b}, t_{fb}, \text{dis}, r, d_{\text{lip}}$) |
| 17..28 | `m_Area`, `m_Torsion`, `m_I33`, `m_I22`, `m_I23`, `m_As2`, `m_As3`, `m_S33`, `m_S22`, `m_Z33`, `m_Z22`, `m_r33`, `m_r22` (#30) | `r8` | Section mechanical properties ($A, J, I_{33}, I_{22}, I_{23}, A_{s2}, A_{s3}, S_{33}, S_{22}, Z_{33}, Z_{22}, r_{33}, r_{22}$) |
| 36 | `m_smod` | `arr_r8` (`len=9`) | Property modifiers `m_smod[1..8]` = `[AMod, A2Mod, A3Mod, JMod, I2Mod, I3Mod, MMod, WMod]` |

#### 2.4.7 Concrete Walkthrough: `example_1_001.sdb` & `example_1_002a.sdb`
Decoded directly by `parse_sdb.py`:
- **`example_1_001.sdb`** (Plane Frame with 3 joints, 3 members, `kip, in, F` units):
  - Outer header: `NameInFile="SAP2000"`, `ProgramVersion="19.0.0"`, `FileVersion="19.00"`, `png_len = 4,438` bytes (`0x34..0x118a`), `gzip` offset = `0x118a`, uncompressed `cBinaryStream` = **`70,245` bytes** (`0` bytes leftover).
  - `ProjectStartUp`: `m_Units = 3` (`kip, in, F`), active DOFs = `UX=true, UY=false, UZ=true, RX=false, RY=true, RZ=false`.
  - `PointObjects` (3 joints):
    - Joint `"1"`: $(0, 0, 0)$ — `Restraint[1]`: `[U1=true, U2=true, U3=true, R1=false, R2=false, R3=false]`
    - Joint `"2"`: $(0, 0, 120)$ — `Restraint[2]`: `[U1=true, U2=true, U3=true, R1=false, R2=false, R3=false]`
    - Joint `"3"`: $(120, 0, 120)$ — `JFLoad`: `LoadPat=1 ("DEAD": F3=-100 kip)`, `LoadPat=2 ("TEMP": F1=100, F3=-200 kip)`
  - `LineObjects` (3 frame elements):
    - Frame `"1"` (`Joint 1 -> Joint 3`, `Section 1`), Frame `"2"` (`Joint 2 -> Joint 3`, `Section 2`), Frame `"3"` (`Joint 2 -> Joint 3`, `Section 1`).
- **`example_1_002a.sdb`** (`kip, in, F` units, uncompressed `cBinaryStream` = **`59,322` bytes**, `0` bytes leftover):
  - 9 `PointObjects` (`"1"`..`"9"`), 10 `LineObjects` (`"1"`..`"10"` using `W18X35` and `W10X30` from `Section[1..2]`), 3 `LoadPattern`s (`"DEAD"`, `"LIVE"`, `"QUAKE"`), 3 `AnalysisCase`s (`"DEAD"`, `"LIVE"`, `"QUAKE"`).

---

## 3. `.pro` Section Property Database File Specification

SAP2000 stores precomputed structural steel, aluminum, and cold-formed section properties in binary `.pro` files (23 files in `/home/edo/REA/extracted/files/`, including `aisc14.pro`, `aisc14m.pro`, `euro.pro`, `sections8.pro`, `sections.pro`, `aisc.pro`, `cisc9.pro`, `chinese.pro`, `british.pro`, `indian.pro`, `australian.pro`, `japanese.pro`, `russian.pro`, `aa6061_t6.pro`, `aisi96.pro`).

Reverse-engineered from `sap2000.exe` methods:
- `hy::a(0x0600a614, fo=23410544)`: Populates available shapes/sections from `.pro` file
- `hy::a(0x0600a615, fo=23411912)`: Reads a 120-byte `.pro` section record and converts units/derived properties into `SECTIONARRAY`
- `hy::a(0x0600a61a, fo=23414100)`: Reads a single 120-byte section record at 1-based record index `recIdx` (`offset = 120 * recIdx`)
- `hy::a(0x0600a61b, fo=23414444)`: Reads Record 0 (the 120-byte file header at `offset = 0`)
- `hy::b(0x0600a5e5, fo=23358984)`: Computes analytical geometric/shear-center offsets (`m_t2off`, `m_t3off`, `m_t2offShearCenter`, `m_t3offShearCenter`)

### 3.1 File Layout: Fixed 120-Byte (`0x78`) Records
Every `.pro` file is a flat array of $N + 1$ fixed-size **120-byte (`0x78`)** records:
$$\text{File Size in Bytes} = 120 \times (N + 1)$$
- **Record `0`** (byte offset `0x00..0x78`): Header record specifying format version (`6.0` or `7.0`), base length unit code (`1.0`..`5.0`), and section count $N$.
- **Records `1..N`** (byte offset `120 * r .. 120 * (r + 1)` for $r \in [1, N]$): Individual section property records, sorted by shape/name.

### 3.2 Record 0: 120-Byte Header Record (`0x0600a61b`)
| Byte Offset | Size | Type | Meaning in `0x0600a61b` | Values Across Extracted `.pro` Files |
|---|---|---|---|---|
| `0x00..0x28` | `40` | `ASCII [u8; 40]` | Header padding / ruler string | `"0123456789012345678901234567890123456789"` in v7 `.pro` files (`aisc14.pro`, `euro.pro`, `sections8.pro`); 40 spaces `"   ..."` in v6 `.pro` files (`sections.pro`, `aisc.pro`) |
| `0x28..0x2C` | `4` | `f32` LE (`hdr[0]`) | **Format Version** | `7.0` (`0x40e00000`) = v7 format (36-char single label per record)<br>`6.0` (`0x40c00000`) = v6 format (two 18-char labels `Label1` US + `Label2` SI per record) |
| `0x2C..0x30` | `4` | `f32` LE (`hdr[1]`) | **Base Length Unit Code** (`Field:bf(0x4005d45)`) | `1.0` = **Inches (`in`)** (`aisc14.pro`, `sections8.pro`, `aisc.pro`, `aa6061_t6.pro`, `aisi96.pro`)<br>`2.0` = **Feet (`ft`)**<br>`3.0` = **Millimeters (`mm`)** (`aisc14m.pro`, `euro.pro`, `cisc9.pro`, `chinese.pro`, `british.pro`, `australian.pro`)<br>`4.0` = **Centimeters (`cm`)** (`indian.pro`, `japanese.pro`, `russian.pro`)<br>`5.0` = **Meters (`m`)** |
| `0x30..0x34` | `4` | `f32` LE (`hdr[2]`) | **Section Record Count $N$** (`Field:ba(0x4005d40)`) | Exact number of section records following Record 0 (e.g. `1998.0` in `aisc14.pro` $\rightarrow 1999 \times 120 = 239,880$ bytes; `4390.0` in `euro.pro` $\rightarrow 4391 \times 120 = 526,920$ bytes) |
| `0x34..0x78` | `68` | `[u8; 68]` | Unused / creation date | In v7 files: ASCII ruler `"0123456789..."`. In v6 files: `0x34..0x38` is `f32` creation date `YYMMDD.0` (e.g. `960829.0`), followed by `0.0` floats. |

**Unit Scaling in `0x0600a615` (`+0106..+01cc`)**:
When SAP2000 imports a `.pro` section into a model whose active length unit has conversion factor $s = \frac{L_{\text{pro}}}{L_{\text{model}}}$:
- Linear dimensions ($L^1$): multiplied by $s^1$
- Area properties ($L^2$): multiplied by $s^2$
- Section moduli ($L^3$): multiplied by $s^3$
- Moments of inertia & torsional constant ($L^4$): multiplied by $s^4$

### 3.3 Records `1..N`: 120-Byte Section Record Layout (`0x0600a61a` & `0x0600a615`)
Each 120-byte section record consists of a **36-byte ASCII name field**, a **4-byte ASCII shape code field**, and **20 IEEE-754 `f32` little-endian floats** (`80` bytes):

| Byte Offset | Float Index | Type | `SECTIONARRAY` Target | Physical / Engineering Meaning | Dimensional Power |
|---|---|---|---|---|---|
| `0x00..0x24` | — | `ASCII [u8; 36]` | `m_sname` / `m_SNInFile` | **Section Name(s)** (space-padded right):<br>- If `hdr[0] == 7.0`: 36-byte section name (`Label1`, e.g. `"W44X335                             "`)<br>- If `hdr[0] == 6.0`: `0x00..0x12` (18 bytes) is US name (`Label1`, e.g. `"W44x285           "`), `0x12..0x24` (18 bytes) is SI metric name (`Label2`, e.g. `"W1100x424         "`) | — |
| `0x24..0x28` | — | `ASCII [u8; 4]` | `m_sh` | **Shape Designator (`dsg`)**: `0x0600a614` reads the first 2 chars (`0x24..0x26`); v7 `.pro` files repeat the 2-char code twice (`"W W "`, `"2L2L"`):<br>- `"W "` / `"WW"` / `"I "` / `"M "` / `"S "` / `"HP"` $\rightarrow$ `m_sh = 1` (I / Wide Flange)<br>- `"C "` / `"MC"` $\rightarrow$ `m_sh = 2` (Channel)<br>- `"T "` / `"TW"` / `"WT"` / `"MT"` / `"ST"` $\rightarrow$ `m_sh = 3` (Structural Tee)<br>- `"L "` $\rightarrow$ `m_sh = 4` (Single Angle)<br>- `"2L"` $\rightarrow$ `m_sh = 5` (Double Angle)<br>- `"B "` / `"TS"` $\rightarrow$ `m_sh = 6` (Box / Rectangular HSS Tube)<br>- `"P "` $\rightarrow$ `m_sh = 7` (Pipe / Round HSS)<br>- `"R "` $\rightarrow$ `m_sh = 8` (Solid Rectangular)<br>- `"CR"` $\rightarrow$ `m_sh = 9` (Solid Round / Circle)<br>- `"G "` $\rightarrow$ `m_sh = 10` (General Section)<br>- `"2C"` $\rightarrow$ `m_sh = 11` (Double Channel) | — |
| `0x28..0x2C` | `f[0]` (`loc.11[1]`) | `f32` LE | `m_t3` | **Outside Depth $t_3$ ($d$)**: vertical depth of section (or outside diameter $D$ for Pipe `"P "` / Circle `"CR"`) | $L^1$ |
| `0x2C..0x30` | `f[1]` (`loc.11[2]`) | `f32` LE | `m_t2` | **Outside Width $t_2$ ($b_f$)**: top flange width for I/Channel/Tee; horizontal leg length for Angle `"L "`; total back-to-back width ($2 b_{\text{horiz}} + \text{dis}$) for Double Angle `"2L"`; outside diameter $D$ for Pipe `"P "` | $L^1$ |
| `0x30..0x34` | `f[2]` (`loc.11[3]`) | `f32` LE | `m_TF` | **Flange Thickness $t_f$**: top flange thickness; horizontal leg thickness for `"L "`; wall thickness $t_{\text{des}}$ for Box `"B "` and Pipe `"P "` | $L^1$ |
| `0x34..0x38` | `f[3]` (`loc.11[4]`) | `f32` LE | `m_TW` | **Web Thickness $t_w$**: web thickness; vertical leg thickness for `"L "`; wall thickness $t_{\text{des}}$ for Box `"B "` and Pipe `"P "` | $L^1$ |
| `0x38..0x3C` | `f[4]` (`loc.11[5]`) | `f32` LE | `m_XK` | **Fillet Design Distance $k_{\text{des}}$**: distance from outer face of flange to web toe of fillet ($0$ for welded/HSS/Pipe) | $L^1$ |
| `0x3C..0x40` | `f[5]` (`loc.11[6]`) | `f32` LE | `m_Area` | **Cross-Sectional Area $A$** | $L^2$ |
| `0x40..0x44` | `f[6]` (`loc.11[7]`) | `f32` LE | `m_Torsion` | **Torsional Constant $J$** ($I_t$) | $L^4$ |
| `0x44..0x48` | `f[7]` (`loc.11[8]`) | `f32` LE | `m_I33` | **Major-Axis Moment of Inertia $I_{33}$** ($I_x$ / $I_y$ in Eurocode) about the horizontal 3-axis | $L^4$ |
| `0x48..0x4C` | `f[8]` (`loc.11[9]`) | `f32` LE | `m_I22` | **Minor-Axis Moment of Inertia $I_{22}$** ($I_y$ / $I_z$ in Eurocode) about the vertical 2-axis | $L^4$ |
| `0x4C..0x50` | `f[9]` (`loc.11[10]`) | `f32` LE | `m_As2` | **Major Shear Area $A_{s2}$** (effective shear area parallel to the 2-axis / web direction) | $L^2$ |
| `0x50..0x54` | `f[10]` (`loc.11[11]`) | `f32` LE | `m_As3` | **Minor Shear Area $A_{s3}$** (effective shear area parallel to the 3-axis / flange direction) | $L^2$ |
| `0x54..0x58` | `f[11]` (`loc.11[12]`) | `f32` LE | `m_Z33` | **Major-Axis Plastic Section Modulus $Z_{33}$** ($Z_x$ / $W_{\text{pl},y}$) | $L^3$ |
| `0x58..0x5C` | `f[12]` (`loc.11[13]`) | `f32` LE | `m_Z22` | **Minor-Axis Plastic Section Modulus $Z_{22}$** ($Z_y$ / $W_{\text{pl},z}$) | $L^3$ |
| `0x5C..0x60` | `f[13]` (`loc.11[14]`) | `f32` LE | `loc.2` ($\bar{x}$) | **Horizontal Centroid Distance $\bar{x}$**: distance from outside vertical face to centroid along 3-axis (populated for Channel `"C "` and Single Angle `"L "`; `0.0` for horizontally symmetric shapes where $\bar{x} = t_2 / 2$) | $L^1$ |
| `0x60..0x64` | `f[14]` (`loc.11[15]`) | `f32` LE | `loc.3` ($\bar{y}$) | **Vertical Centroid Distance $\bar{y}$**: distance from outside horizontal face to centroid along 2-axis (populated for Tee `"T "`, Angle `"L "`, Double Angle `"2L"`, and asymmetric I-shapes; `0.0` for vertically symmetric shapes where $\bar{y} = t_3 / 2$) | $L^1$ |
| `0x64..0x68` | `f[15]` (`loc.11[16]`) | `f32` LE | ($r_z$) | **Principal Minor Radius of Gyration $r_z$ ($r_{\min}$)**: populated in v6 `.pro` files (`sections.pro`, `aisc.pro`) for Single Angles `"L "` (e.g. `1.56` in for `L8x8x1`); `0.0` in v7 `.pro` files where SAP2000 computes principal angle properties analytically | $L^1$ |
| `0x68..0x6C` | `f[16]` (`loc.11[17]`) | `f32` LE | `m_dis` | **Back-to-Back Separation Distance $\text{dis}$**: gap between back-to-back angles (`"2L"`) or channels (`"2C"`), e.g. `0.375` or `0.75` in; `0.0` for other shapes | $L^1$ |
| `0x6C..0x70` | `f[17]` (`loc.11[18]`) | `f32` LE | ($e_0$) | **Shear Center Distance $e_0$**: distance from back of channel web to shear center in v6 `.pro` files (`sections.pro`, `aisc.pro`, e.g. `0.583` in for `C15x50`); `0.0` in v7 `.pro` files where `0x0600a5e5` computes $e_0$ analytically | $L^1$ |
| `0x70..0x74` | `f[18]` (`loc.11[19]`) | `f32` LE | `m_t2b` | **Bottom Flange Width $t_{2b}$ ($b_{fb}$)**: populated for I/Wide-Flange shapes (`"W "` / `"I "`, equals $t_2$ for doubly-symmetric I-shapes); `0.0` for non-I shapes | $L^1$ |
| `0x74..0x78` | `f[19]` (`loc.11[20]`) | `f32` LE | `m_tfb` | **Bottom Flange Thickness $t_{fb}$**: populated for I/Wide-Flange shapes (`"W "` / `"I "`, equals $t_f$ for doubly-symmetric I-shapes); `0.0` for non-I shapes | $L^1$ |

### 3.4 Exact Formulas for Derived Section Properties (`0x0600a615` & `0x0600a5e5`)
When `0x0600a615` loads a record from a `.pro` file, it computes the elastic section moduli ($S_{33}, S_{22}$), radii of gyration ($r_{33}, r_{22}$), and centroid/shear-center offsets (`m_t2off`, `m_t3off`, `m_t2offShearCenter`, `m_t3offShearCenter`) as follows:

1. **Extreme Fiber Distances $c_2$ (horizontal) and $c_3$ (vertical)**:
   $$c_2 = \begin{cases} t_2 / 2 & \text{if } f[13] = 0 \\ \max(f[13],\; t_2 - f[13]) & \text{if } f[13] \ne 0 \end{cases}$$
   $$c_3 = \begin{cases} t_3 / 2 & \text{if } f[14] = 0 \\ \max(f[14],\; t_3 - f[14]) & \text{if } f[14] \ne 0 \end{cases}$$
2. **Elastic Section Moduli $S_{33}$ and $S_{22}$**:
   $$S_{33} = \frac{I_{33}}{c_3}, \qquad S_{22} = \frac{I_{22}}{c_2}$$
3. **Radii of Gyration $r_{33}$ and $r_{22}$**:
   $$r_{33} = \sqrt{\frac{I_{33}}{A}}, \qquad r_{22} = \sqrt{\frac{I_{22}}{A}}$$
4. **Centroid & Shear-Center Offsets (`0x0600a615` at `+0305..+03fc` and `0x0600a5e5`)**:
   - **Channel (`m_sh = 2`)**:
     - `m_t2off` $= f[13] - 0.5\, t_2$, `m_t3off` $= 0$
     - Shear center distance from web centerline:
       $$e_{\text{sc}} = \frac{3 b_f^2 t_f}{6 b_f t_f + (d - t_f) t_w} \quad \text{where } b_f = t_2 - 0.5 t_w,\; d = t_3 - t_f$$
     - `m_t2offShearCenter` $= -(e_{\text{sc}} + 0.5 t_w - f[13])$
   - **Structural Tee (`m_sh = 3`)**:
     - `m_t2off` $= 0$, `m_t3off` $= 0.5 t_3 - f[14]$
     - `m_t3offShearCenter` $= f[14] - 0.5 t_f$
   - **Single Angle (`m_sh = 4`)**:
     - `m_t2off` $= f[13] - 0.5 t_2$, `m_t3off` $= f[14] - 0.5 t_3$
     - `m_t2offShearCenter` $= 0.5 t_w - f[13]$, `m_t3offShearCenter` $= 0.5 t_f - f[14]$
   - **Double Angle (`m_sh = 5`)**:
     - `m_t2off` $= 0$, `m_t3off` $= 0.5 t_3 - f[14]$
     - `m_t3offShearCenter` $= f[14] - 0.5 t_f$

### 3.5 Verified Concrete Section Records Across `.pro` Databases
The table below shows actual decoded records from `aisc14.pro` (`hdr = [7.0, 1.0 (in), 1998.0]`) and `euro.pro` (`hdr = [7.0, 3.0 (mm), 4390.0]`):

| Section Name (`.pro` File) | Shape (`dsg`) | $t_3$ (`f[0]`) | $t_2$ (`f[1]`) | $t_f$ (`f[2]`) | $t_w$ (`f[3]`) | $k_{\text{des}}$ (`f[4]`) | $A$ (`f[5]`) | $J$ (`f[6]`) | $I_{33}$ (`f[7]`) | $I_{22}$ (`f[8]`) | $A_{s2}$ (`f[9]`) | $A_{s3}$ (`f[10]`) | $Z_{33}$ (`f[11]`) | $Z_{22}$ (`f[12]`) | $\bar{x}$ (`f[13]`) | $\bar{y}$ (`f[14]`) | $\text{dis}$ (`f[16]`) | $t_{2b}$ (`f[18]`) | $t_{fb}$ (`f[19]`) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **`W44X335`** (`aisc14.pro` #1) | `"W W "` (`1`) | `44.0` in | `15.9` in | `1.77` in | `1.03` in | `2.56` in | `98.5` $\text{in}^2$ | `74.4` $\text{in}^4$ | `31100` $\text{in}^4$ | `1200` $\text{in}^4$ | `45.32` $\text{in}^2$ | `46.905` $\text{in}^2$ | `1620` $\text{in}^3$ | `236` $\text{in}^3$ | `0` | `0` | `0` | `15.9` in | `1.77` in |
| **`W14X90`** (`aisc14.pro` #163) | `"W W "` (`1`) | `14.0` in | `14.5` in | `0.71` in | `0.44` in | `1.31` in | `26.5` $\text{in}^2$ | `4.06` $\text{in}^4$ | `999` $\text{in}^4$ | `362` $\text{in}^4$ | `6.16` $\text{in}^2$ | `17.158` $\text{in}^2$ | `157` $\text{in}^3$ | `75.6` $\text{in}^3$ | `0` | `0` | `0` | `14.5` in | `0.71` in |
| **`C15X50`** (`aisc14.pro` #315) | `"C C "` (`2`) | `15.0` in | `3.72` in | `0.65` in | `0.716` in | `1.3125` in | `14.7` $\text{in}^2$ | `2.65` $\text{in}^4$ | `404` $\text{in}^4$ | `11.0` $\text{in}^4$ | `10.74` $\text{in}^2$ | `4.028` $\text{in}^2$ | `68.5` $\text{in}^3$ | `6.14` $\text{in}^3$ | `0.799` in | `0` | `0` | `0` | `0` |
| **`WT22X167.5`** (`aisc14.pro` #812) | `"T T "` (`3`) | `22.0` in | `15.9` in | `1.77` in | `1.03` in | `2.56` in | `49.3` $\text{in}^2$ | `37.1` $\text{in}^4$ | `2120` $\text{in}^4$ | `599` $\text{in}^4$ | `22.66` $\text{in}^2$ | `23.453` $\text{in}^2$ | `202` $\text{in}^3$ | `118` $\text{in}^3$ | `0` | `5.02` in | `0` | `0` | `0` |
| **`L8X8X1-1/8`** (`aisc14.pro` #385) | `"L L "` (`4`) | `8.0` in | `8.0` in | `1.125` in | `1.125` in | `1.75` in | `16.8` $\text{in}^2$ | `6.92` $\text{in}^4$ | `98.1` $\text{in}^4$ | `98.1` $\text{in}^4$ | `9.0` $\text{in}^2$ | `9.0` $\text{in}^2$ | `31.6` $\text{in}^3$ | `31.6` $\text{in}^3$ | `2.4` in | `2.4` in | `0` | `0` | `0` |
| **`2L8X8X1-1/8`** (`aisc14.pro` #1200) | `"2L2L"` (`5`) | `8.0` in | `16.0` in | `1.125` in | `1.125` in | `1.75` in | `33.5` $\text{in}^2$ | `13.8` $\text{in}^4$ | `196` $\text{in}^4$ | `390` $\text{in}^4$ | `18.0` $\text{in}^2$ | `18.0` $\text{in}^2$ | `63.2` $\text{in}^3$ | `79.9` $\text{in}^3$ | `0` | `2.4` in | `0` | `0` | `0` |
| **`HSS20X12X.625`** (`aisc14.pro` #1436) | `"B B "` (`6`) | `20.0` in | `12.0` in | `0.581` in | `0.581` in | `0` | `34.6` $\text{in}^2$ | `1200` $\text{in}^4$ | `1830` $\text{in}^4$ | `839` $\text{in}^4$ | `23.24` $\text{in}^2$ | `13.944` $\text{in}^2$ | `227` $\text{in}^3$ | `162` $\text{in}^3$ | `0` | `0` | `0` | `0` | `0` |
| **`HSS20X.500`** (`aisc14.pro` #1836) | `"P P "` (`7`) | `20.0` in | `20.0` in | `0.465` in | `0.465` in | `0` | `28.5` $\text{in}^2$ | `2720` $\text{in}^4$ | `1360` $\text{in}^4$ | `1360` $\text{in}^4$ | `14.25` $\text{in}^2$ | `14.25` $\text{in}^2$ | `177` $\text{in}^3$ | `177` $\text{in}^3$ | `0` | `0` | `0` | `0` | `0` |
| **`IPE200`** (`euro.pro` #9) | `"I I "` (`1`) | `200` mm | `100` mm | `8.5` mm | `5.6` mm | `12` mm | `2848` $\text{mm}^2$ | `69800` $\text{mm}^4$ | $1.943 \times 10^7$ $\text{mm}^4$ | $1.424 \times 10^6$ $\text{mm}^4$ | `1120` $\text{mm}^2$ | `1416.7` $\text{mm}^2$ | $2.206 \times 10^5$ $\text{mm}^3$ | $4.461 \times 10^4$ $\text{mm}^3$ | `0` | `0` | `0` | `100` mm | `8.5` mm |
| **`HE100A`** (`euro.pro` #31) | `"I I "` (`1`) | `96` mm | `100` mm | `8.0` mm | `5.0` mm | `12` mm | `2124` $\text{mm}^2$ | `52400` $\text{mm}^4$ | $3.492 \times 10^6$ $\text{mm}^4$ | $1.338 \times 10^6$ $\text{mm}^4$ | `480` $\text{mm}^2$ | `1333.3` $\text{mm}^2$ | $8.301 \times 10^4$ $\text{mm}^3$ | $4.114 \times 10^4$ $\text{mm}^3$ | `0` | `0` | `0` | `100` mm | `8.0` mm |

---

## 4. `.s2k` / `.$2k` Text Model Format Specification

SAP2000 supports two distinct generations of `.s2k` / `.$2k` plain-text model files:
1. **Modern Tabular Database Format (SAP2000 v8.0.0 through v19.0.0)**: Uses `TABLE:  "<TableName>"` blocks containing `Key=Value` record lines, handled by `sap2000.exe` (`TypeDef 0x02000049 ao`, methods `0x06000cd1`–`0x06001cf3`).
2. **Legacy Keyword-Block Format (SAP2000 v6 / v7.40)**: Uses section headers like `SYSTEM`, `JOINT`, `RESTRAINT`, `MATERIAL`, `FRAME SECTION`, `FRAME`, `LOAD`, `COMBO`, generated by `saptran.dll` (`TOPDLL.FOR`) when translating SAP90 models and imported by `sap2000.exe`'s legacy v6/v7 translator.

### 4.1 Modern v8–v19 `.s2k` / `.$2k` Tabular Format (`sap2000.exe`)

#### 4.1.1 Lexical & Syntactic Rules (`0x06000cf2`, `0x06000cff`, `0x06000d01`, `0x06000cf6`)
1. **File Header Banner**:
   ```text
   File <path> was saved on m/d/yy at h:mm:ss
   ```
2. **Table Header (`0x06000cff`)**:
   Each table begins with a blank line followed by `TABLE:  "<TableName>"` (note the two spaces after `TABLE:`):
   ```text
   TABLE:  "Program Control"
   ```
   The first table in every valid `.s2k` / `.$2k` file **must** be `TABLE:  "Program Control"` (`0x06000cee` at `+0553`), which establishes `CurrUnits` (`"Kip, in, F"`, `"kN, m, C"`, etc.) for all subsequent tables.
3. **Record Lines & Line Continuation (`0x06000d01` at `+01b2..+024c`)**:
   - Each record row starts with 3 leading spaces (`"   "`), followed by space-separated `FieldName=Value` pairs (3 spaces between fields).
   - Values containing spaces or special characters (or string fields) are enclosed in double quotes (`FieldName="Value"`), while numeric/identifier tokens without spaces may be unquoted (`FieldName=Value`).
   - When a line exceeds `S_Dimensions.TextFileMaxChar` (default ~75–120 chars), SAP2000 appends `" _"` (space + underscore) at the end of the line and continues the **same record** on the next line indented by 5 leading spaces (`"     "`).
4. **End of File Marker (`0x06000cf6` at `+006c`)**:
   ```text
   END TABLE DATA
   ```
5. **Comments**:
   Lines starting with `$` are treated as comments/log lines.

#### 4.1.2 Core `.s2k` (`$2k`) Model Definition & Result Tables
Extracted directly from `sap2000.exe` (`0x06000cda..0x06000ce1` and table builders `0x060015a1..0x06001cf3`; full 2,292-table catalog saved in `/home/edo/.gemini/antigravity/brain/93a71218-fc85-41f1-84cc-d6032276e1c4/scratch/s2k_tables_and_fields.json`):

| `TABLE:  "<TableName>"` | Builder Method | Exact Field Keys in `.s2k` Order |
|---|---|---|
| `"Program Control"` | `0x0600188f` | `ProgramName`, `Version`, `ProgLevel`, `LicenseNum`, `LicenseOS`, `LicenseSC`, `LicenseBR`, `LicenseHT`, `CurrUnits`, `SteelCode`, `ConcCode`, `AlumCode`, `ColdCode`, `BridgeCode`, `BridgeRat`, `BridgeSeism`, `RegenHinge`, `BSchedGUID` |
| `"Active Degrees of Freedom"` | `0x060015a1` | `UX`, `UY`, `UZ`, `RX`, `RY`, `RZ` (`Yes`/`No`) |
| `"Analysis Options"` | `0x060015a2` | `Solver`, `SolverProc`, `Force32Bit`, `StiffCase`, `GeomMod`, `HingeOpt`, `HingeLnkOpt`, `LinkF0` |
| `"Coordinate Systems"` | `0x060016e4` | `Name`, `Type`, `X`, `Y`, `Z`, `AboutZ`, `AboutY`, `AboutX` |
| `"Grid Lines"` | `0x06001770` | `CoordSys`, `AxisDir`, `GridID`, `XRYZCoord`, `TAngle`, `LineType`, `LineColor`, `Visible`, `BubbleLoc`, `AllVisible`, `BubbleSize` |
| `"Material Properties 01 - General"` | `0x060017e3` | `Material`, `Type` (`Steel`/`Concrete`/`Aluminum`/`ColdFormed`/`Rebar`/`Tendon`/`Other`), `SymType` (`Isotropic`/`Orthotropic`/`Anisotropic`/`Uniaxial`), `TempDepend`, `Color`, `GUID`, `Notes` |
| `"Material Properties 02 - Basic Mechanical Properties"` | `0x060017e5` | `Material`, `Temp`, `UnitWeight`, `UnitMass`, `E1`, `E2`, `E3`, `G12`, `G13`, `G23`, `U12`, `U13`, `U23`, `A1`, `A2`, `A3` |
| `"Material Properties 03a - Steel Data"` | `0x060017e6` | `Material`, `Temp`, `Fy`, `Fu`, `EffFy`, `EffFu`, `SSCurveOpt`, `SSHysType`, `SHard`, `SMax`, `SRup`, `FinalSlope` |
| `"Material Properties 03b - Concrete Data"` | `0x060017e7` | `Material`, `Temp`, `Fc`, `eFc`, `LtWtConc`, `LtWtFact`, `SSCurveOpt`, `SSHysType`, `SFc`, `SCap`, `FinalSlope`, `FAngle`, `DAngle` |
| `"Frame Section Properties 01 - General"` | `0x0600171a` | `SectionName`, `Material`, `Shape` (`"I/Wide Flange"`, `"Channel"`, `"Tee"`, `"Angle"`, `"Double Angle"`, `"Box/Tube"`, `"Pipe"`, `"Rectangular"`, `"Circle"`, `"General"`), `t3`, `t2`, `tf`, `tw`, `t2b`, `tfb`, `dis`, `Radius`, `LipDepth`, `LipAngle`, `Area`, `TorsConst`, `I33`, `I22`, `I23`, `AS2`, `AS3`, `S33`, `S22`, `Z33`, `Z22`, `R33`, `R22`, `EccV2`, `ConcCol`, `ConcBeam`, `Color`, `TotalWt`, `TotalMass`, `FromFile`, `AMod`, `A2Mod`, `A3Mod`, `JMod`, `I2Mod`, `I3Mod`, `MMod`, `WMod`, `SectInFile`, `FileName`, `GUID`, `Notes` |
| `"Area Section Properties"` | `0x060015b7` | `Section`, `Material`, `MatAngle`, `AreaType` (`Shell`/`Plane`/`ASolid`), `Type` (`Shell-Thin`/`Shell-Thick`/`Plate-Thin`/`Plate-Thick`/`Membrane`/`Plane-Stress`/`Plane-Strain`), `DrillDOF`, `Thickness`, `BendThick`, `Arc`, `InComp`, `CoordSys`, `Color`, `TotalWt`, `TotalMass`, `F11Mod`, `F22Mod`, `F12Mod`, `M11Mod`, `M22Mod`, `M12Mod`, `V13Mod`, `V23Mod`, `MMod`, `WMod`, `GUID`, `Notes` |
| `"Joint Coordinates"` | `0x060017b0` | `Joint`, `CoordSys`, `CoordType` (`Cartesian`/`Cylindrical`/`Spherical`), `XorR`, `Y`, `T`, `Z`, `SpecialJt`, `GlobalX`, `GlobalY`, `GlobalZ`, `GUID` |
| `"Joint Restraint Assignments"` | `0x060017ba` | `Joint`, `U1`, `U2`, `U3`, `R1`, `R2`, `R3` (`Yes`/`No`) |
| `"Joint Spring Assignments 1 - Uncoupled"` | `0x060017bb` | `Joint`, `CoordSys`, `U1`, `U2`, `U3`, `R1`, `R2`, `R3` |
| `"Joint Added Mass Assignments"` | `0x060017ab` | `Joint`, `CoordSys`, `Mass1`, `Mass2`, `Mass3`, `MMI1`, `MMI2`, `MMI3` |
| `"Connectivity - Frame"` | `0x060016d6` | `Frame`, `JointI`, `JointJ`, `IsCurved`, `Length`, `CentroidX`, `CentroidY`, `CentroidZ`, `GUID` |
| `"Connectivity - Area"` | `0x060016d4` | `Area`, `NumJoints`, `Joint1`, `Joint2`, `Joint3`, `Joint4`, `Perimeter`, `AreaArea`, `Volume`, `CentroidX`, `CentroidY`, `CentroidZ`, `Order`, `GUID` |
| `"Connectivity - Solid"` | `0x060016d8` | `Solid`, `Joint1`, `Joint2`, `Joint3`, `Joint4`, `Joint5`, `Joint6`, `Joint7`, `Joint8`, `Volume`, `CentroidX`, `CentroidY`, `CentroidZ`, `GUID` |
| `"Connectivity - Link"` | `0x060016d7` | `Link`, `JointI`, `JointJ`, `Length`, `CentroidX`, `CentroidY`, `CentroidZ`, `GUID` |
| `"Connectivity - Cable"` | `0x060016d5` | `Cable`, `JointI`, `JointJ`, `Length`, `GUID` |
| `"Connectivity - Tendon"` | `0x060016d9` | `Tendon`, `JointI`, `JointJ`, `Length`, `GUID` |
| `"Frame Section Assignments"` | `0x06001718` | `Frame`, `SectionType`, `AutoSelect`, `AnalSect`, `DesignSect`, `MatProp`, `NPSectType`, `NPSectLen`, `NPSectRD` |
| `"Frame Release Assignments 1 - General"` | `0x06001716` | `Frame`, `PI`, `V2I`, `V3I`, `TI`, `M2I`, `M3I`, `PJ`, `V2J`, `V3J`, `TJ`, `M2J`, `M3J`, `PartialFix` |
| `"Frame Release Assignments 2 - Partial Fixity"` | `0x06001717` | `Frame`, `PI`, `V2I`, `V3I`, `TI`, `M2I`, `M3I`, `PJ`, `V2J`, `V3J`, `TJ`, `M2J`, `M3J` |
| `"Frame Local Axes Assignments 1 - Typical"` | `0x0600170d` | `Frame`, `Angle`, `AdvanceAxes` |
| `"Area Section Assignments"` | `0x060015b6` | `Area`, `Section`, `MatProp` |
| `"Load Pattern Definitions"` | `0x060017de` | `LoadPat`, `DesignType` (`Dead`/`Super Dead`/`Live`/`Reducible Live`/`Quake`/`Wind`/`Snow`/`Temp`/`Other`), `SelfWtMult`, `AutoLoad`, `GUID`, `Notes` |
| `"Load Case Definitions"` | `0x060017dd` | `Case`, `Type` (`LinStatic`/`NonStatic`/`LinModal`/`LinRespSpec`/`LinHist`/`NonHist`/`LinBuckling`/`LinSteady`/`LinPSD`/`Move`), `InitialCond`, `ModalCase`, `BaseCase`, `MassSource`, `DesTypeOpt`, `DesignType`, `DesActOpt`, `DesignAct`, `AutoType`, `RunCase`, `CaseStatus`, `GUID`, `Notes` |
| `"Case - Static 1 - Load Assignments"` | `0x060016c7` | `Case`, `LoadType` (`"Load pattern"`/`"Accel"`/`"Mode"`), `LoadName`, `LoadSF`, `TransAccSF`, `RotAccSF` |
| `"Case - Modal 1 - General"` | `0x060016ad` | `Case`, `ModeType` (`Eigen`/`Ritz`), `MaxNumModes`, `MinNumModes`, `EigenShift`, `EigenCutoff`, `EigenTol`, `AutoShift` |
| `"Combination Definitions"` | `0x060016d3` | `ComboName`, `ComboType` (`"Linear Add"`/`"Envelope"`/`"Abs Add"`/`"SRSS"`/`"Range Add"`), `AutoDesign`, `CaseType`, `CaseName`, `ScaleFactor`, `SteelDesign`, `ConcDesign`, `AlumDesign`, `ColdDesign`, `GUID`, `Notes` |
| `"Joint Loads - Force"` | `0x060017b1` | `Joint`, `LoadPat`, `CoordSys`, `F1`, `F2`, `F3`, `M1`, `M2`, `M3`, `GUID` |
| `"Joint Loads - Ground Displacement"` | `0x060017b2` | `Joint`, `LoadPat`, `CoordSys`, `U1`, `U2`, `U3`, `R1`, `R2`, `R3` |
| `"Frame Loads - Distributed"` | `0x06001705` | `Frame`, `LoadPat`, `CoordSys`, `Type` (`Force`/`Moment`), `Dir` (`Gravity`/`Local 1`/`Local 2`/`Local 3`/`X`/`Y`/`Z`/`Proj X`/`Proj Y`/`Proj Z`), `DistType` (`RelDist`/`AbsDist`), `RelDistA`, `RelDistB`, `AbsDistA`, `AbsDistB`, `FOverLA`, `FOverLB`, `MOverLA`, `MOverLB`, `GUID` |
| `"Frame Loads - Point"` | `0x06001708` | `Frame`, `LoadPat`, `CoordSys`, `Type` (`Force`/`Moment`), `Dir`, `DistType`, `RelDist`, `AbsDist`, `Force`, `Moment`, `GUID` |
| `"Area Loads - Uniform"` | `0x060015ad` | `Area`, `LoadPat`, `CoordSys`, `Dir`, `UnifLoad` |
| `"Groups 1 - Definitions"` | `0x06001771` | `GroupName`, `Selection`, `SectionCut`, `Steel`, `Concrete`, `Aluminum`, `ColdFormed`, `Stage`, `Bridge`, `AutoSeismic`, `AutoWind`, `SelDesSteel`, `SelDesAlum`, `SelDesCold`, `MassWeight`, `Color` |
| `"Joint Displacements"` *(Output)* | `0x06001c73` | `Joint`, `OutputCase`, `CaseType`, `StepType`, `StepNum`, `StepLabel`, `U1`, `U2`, `U3`, `R1`, `R2`, `R3` |
| `"Joint Reactions"` *(Output)* | `0x06001c75` | `Joint`, `OutputCase`, `CaseType`, `StepType`, `StepNum`, `StepLabel`, `F1`, `F2`, `F3`, `M1`, `M2`, `M3` |
| `"Element Forces - Frames"` *(Output)* | `0x06001c60` | `Frame`, `Station`, `OutputCase`, `CaseType`, `StepType`, `StepNum`, `StepLabel`, `P`, `V2`, `V3`, `T`, `M2`, `M3`, `FrameElem`, `ElemStation` |
| `"Element Forces - Area Shells"` *(Output)* | `0x06001c5f` | `Area`, `AreaElem`, `ShellType`, `Joint`, `OutputCase`, `CaseType`, `StepType`, `StepNum`, `StepLabel`, `F11`, `F22`, `F12`, `FMax`, `FMin`, `FAngle`, `FVM`, `M11`, `M22`, `M12`, `MMax`, `MMin`, `MAngle`, `V13`, `V23`, `VMax`, `VAngle` |
| `"Modal Periods And Frequencies"` *(Output)* | `0x06001c7d` | `OutputCase`, `StepType`, `StepNum`, `Period`, `Frequency`, `CircFreq`, `Eigenvalue` |

#### 4.1.3 Example Modern `.s2k` (`$2k`) Model (`example_1_001.sdb` Equivalent)
```text
File C:\SAP2000\Example_1_001.$2k was saved on 10/14/26 at 23:30:00
 
TABLE:  "Program Control"
   ProgramName=SAP2000   Version=19.0.0   ProgLevel=Advanced   CurrUnits="Kip, in, F" _
     SteelCode="AISC 360-10"   ConcCode="ACI 318-14"   AlumCode="AA-ASD 2000" _
     ColdCode="AISI-ASD96"   RegenHinge=Yes
 
TABLE:  "Active Degrees of Freedom"
   UX=Yes   UY=No   UZ=Yes   RX=No   RY=Yes   RZ=No
 
TABLE:  "Joint Coordinates"
   Joint=1   CoordSys=GLOBAL   CoordType=Cartesian   XorR=0   Y=0   Z=0   SpecialJt=Yes
   Joint=2   CoordSys=GLOBAL   CoordType=Cartesian   XorR=0   Y=0   Z=120   SpecialJt=Yes
   Joint=3   CoordSys=GLOBAL   CoordType=Cartesian   XorR=120   Y=0   Z=120   SpecialJt=Yes
 
TABLE:  "Joint Restraint Assignments"
   Joint=1   U1=Yes   U2=Yes   U3=Yes   R1=No   R2=No   R3=No
   Joint=2   U1=Yes   U2=Yes   U3=Yes   R1=No   R2=No   R3=No
 
TABLE:  "Connectivity - Frame"
   Frame=1   JointI=1   JointJ=3   IsCurved=No
   Frame=2   JointI=2   JointJ=3   IsCurved=No
   Frame=3   JointI=2   JointJ=3   IsCurved=No
 
TABLE:  "Load Pattern Definitions"
   LoadPat=DEAD   DesignType=Dead   SelfWtMult=0
   LoadPat=TEMP   DesignType=Temp   SelfWtMult=0
 
TABLE:  "Joint Loads - Force"
   Joint=3   LoadPat=DEAD   CoordSys=GLOBAL   F1=0   F2=0   F3=-100   M1=0   M2=0   M3=0
   Joint=3   LoadPat=TEMP   CoordSys=GLOBAL   F1=100   F2=0   F3=-200   M1=0   M2=0   M3=0
 
END TABLE DATA
```

---

### 4.2 Legacy SAP2000 v6/v7 `.s2k` Block-Keyword Format (`saptran.dll`)
`saptran.dll` (`TOPDLL.FOR`, exported function `SAPTRAN`) translates SAP90 files into SAP2000 v7.40 `.s2k` files using keyword blocks (where `;` starts a comment and `ADD=` continues a multi-line entry):
- `SYSTEM`: `DOF=UX,UY,UZ,RX,RY,RZ`, `LENGTH=IN|FT|MM|CM|M`, `FORCE=LB|KIP|KN|N|KGF|TON`, `TMP=F|C`, `MODES=n`, `SHIFT=f`, `TOL=f`, `RITZ=n`
- `JOINT`: `<id>  X=<x>  Y=<y>  Z=<z>`
- `RESTRAINT`: `ADD=<id>  DOF=U1,U2,U3,R1,R2,R3`
- `SPRING`: `ADD=<id>  U1=<k1>  U2=<k2>  U3=<k3>  R1=<kr1>  R2=<kr2>  R3=<kr3>`
- `MASS`: `ADD=<id>  U1=<m1>  U2=<m2>  U3=<m3>  R1=<mr1>  R2=<mr2>  R3=<mr3>`
- `CONSTRAINT`: `NAME=<name>  TYPE=BODY|DIAPH|PLATE|ROD|BEAM|EQUAL  DOF=...`
- `MATERIAL`: `NAME=<name>  TYPE=ISO|ORTHO  IDES=S|C|N  M=<rho>  W=<gamma>` followed by `T=<temp>  E=<E>  U=<nu>  A=<alpha>  G=<G>  FY=<fy>  FC=<fc>`
- `FRAME SECTION`: `NAME=<name>  MAT=<mat>  SH=R|P|B|I|C|T|L|2L|G  T=<t3>,<t2>,<tf>,<tw>,<t2b>,<tfb>  A=<A>  J=<J>  I=<I33>,<I22>  AS=<As2>,<As3>`
- `SHELL SECTION`: `NAME=<name>  MAT=<mat>  TYPE=SHELL|PLATE|MEMBRANE  TH=<t>  THB=<tb>`
- `NLPROP`: `NAME=<name>  TYPE=DAMPER|GAP|HOOK|PLASTIC1|ISOLATOR1|ISOLATOR2`
- `FRAME`: `<id>  J=<j1>,<j2>  SEC=<sec>  NSEG=<n>  ANG=<ang>  REL=<r>`
- `SHELL`: `<id>  J=<j1>,<j2>,<j3>(,<j4>)  SEC=<sec>  TYPE=SHELL|PLANE|ASOLID`
- `SOLID`: `<id>  J=<j1>,<j2>,<j3>,<j4>,<j5>,<j6>,<j7>,<j8>  MAT=<mat>`
- `NLLINK`: `<id>  J=<j1>(,<j2>)  NLP=<prop>`
- `LOAD`: `NAME=<name>  CSYS=0  SW=<selfwt>` followed by sub-blocks:
  - `TYPE=FORCE`: `ADD=<jid>  F=<fx>,<fy>,<fz>  M=<mx>,<my>,<mz>`
  - `TYPE=DISPLACEMENT`: `ADD=<jid>  U=<ux>,<uy>,<uz>  R=<rx>,<ry>,<rz>`
  - `TYPE=DISTRIBUTED`: `ADD=<fid>  RD=0,1  F=<w1>,<w2>`
  - `TYPE=CONCENTRATED`: `ADD=<fid>  RD=<d>  F=<p>`
  - `TYPE=UNIFORM`: `ADD=<aid>  UZ=<q>`
  - `TYPE=PRESSURE`: `ADD=<aid>  FACE=<f>  P=<p>`
  - `TYPE=TEMPERATURE`: `ADD=<id>  T=<dt>`
- `FUNCTION`: `NAME=<name>  TYPE=SPECTRUM|HISTORY  NPL=1  DT=<dt>`
- `SPEC`: `NAME=<name>  DAMP=<zeta>  ACC=U1,U2,U3  FUNC=<f1>,<f2>,<f3>  SF=<s1>,<s2>,<s3>`
- `HIST`: `NAME=<name>  TYPE=LIN|NONLIN  NSTEP=<n>  DT=<dt>  DAMP=<zeta>`
- `COMBO`: `NAME=<name>  TYPE=ADD|ENV|ABS|SRSS` followed by `LOAD=<lname>  SF=<sf>`

---

## 5. XML Configuration, Material Library & Report Files

### 5.1 `csimateriallibrary*.xml` & `csimateriallibrary.xsd`
Six regional XML material libraries are included in `/home/edo/REA/extracted/files/`:
- `csimateriallibrarychina.xml`, `csimateriallibraryeurope.xml`, `csimateriallibraryindia.xml`, `csimateriallibraryitaly.xml`, `csimateriallibraryrussia.xml`, `csimateriallibraryspain.xml` (validated against `csimateriallibrary.xsd`).

**Schema Structure**:
- Root `<propertyFile xmlns="http://www.csiberkeley.com">`:
  - `<control>`: `<version>`, `<lengthUnits>` (`mm`, `cm`, `m`, `in`, `ft`), `<forceUnits>` (`N`, `kN`, `kgf`, `tonf`, `lb`, `kip`), `<temperatureUnits>` (`C`, `F`).
  - `<region name="Europe">`:
    - `<collection type="Materials">`:
      - `<standard name="EN 1993-1-1" ...>` / `<standard name="EN 1992-1-1">`:
        - `<material type="Steel|Concrete|Rebar|Aluminum|Tendon">` with `<name>`, `<grade>`, `<symmetryType>` (`Isotropic`/`Uniaxial`), `<weightPerUnitVolume>`, `<massPerUnitVolume>`, and type-specific `<isotropic>` (`<E>`, `<U>`, `<A>`, `<G>`), `<steel>` (`<Fy>`, `<Fu>`, `<Fye>`, `<Fue>`, `<tMax>` thickness-dependent tables), `<concrete>` (`<fc>`, `<fcsFactor>`, `<strainAtfc>`, `<strainUltimate>`), `<rebar>`, `<aluminum>`, or `<tendon>` elements.

### 5.2 `csitext01.xml`, `sapreportcontents.xml` & `csidefaulttablenamefile.xsd`
- **`csitext01.xml`**: Root `<report_texts>` containing 46 `<text>` entries (`<label>` such as `"Introduction"`, `"Layout Line"`, `"Materials"`, `"Frame Sections"`, `"Bearings"`, `"Abutments"`, `"Bents"`, `"Diaphragms"`, `"Hinges"`, `"Links"`, `"Tendons"`, `"Loads"`, `"Load Cases"`, `"Combos"`, and `<narrative>` containing Base64-encoded RTF report text templates).
- **`sapreportcontents.xml`**: Standard report generator template hierarchy referencing database table keys via `<table_id><key>...</key></table_id>` (matching the `tkey` strings in `s2k_tables_and_fields.json`).
- **`csidefaulttablenamefile.xsd`**: XML Schema for custom database table/field name mapping files (`<DBTableNameFile>` $\rightarrow$ `<tableItem><tableKey>`, `<tableName>`, `<fieldItem><fieldKey>`, `<fieldName>`), used by `sap2000.exe` (`0x06000cea`) to map internal table/field keys to localized or user-defined `.s2k`/XML/Excel names.
