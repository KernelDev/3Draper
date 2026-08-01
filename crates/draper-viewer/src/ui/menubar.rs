// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Menu bar — 21 cascading menus per ROADMAP_UI Phase 1.
//!
//! Each menu is a function that returns `Option<MenuAction>` when an item is clicked.
//! Backend wiring: actions are dispatched to dispatcher.rs → 3Draper engine.

use eframe::egui;

/// Action emitted by the menu bar / ribbon / command palette.
#[derive(Clone, Debug, PartialEq)]
pub enum MenuAction {
    /// No action (stub).
    None,

    // ── File actions ──
    FileNew,
    FileOpen,
    FileSave,
    FileSaveAs,
    FileExportStep,
    FileExportStl,
    FileExportObj,
    FileExportGltf,
    FileExportPdf,
    FileExportDxf,
    FileImportStep,
    FileImportStl,
    FileImportObj,
    FileImportPly,
    FileImportDxf,
    FileImportPointCloud,
    FilePrint,
    FileQuit,

    // ── Edit actions ──
    EditUndo,
    EditRedo,
    EditCut,
    EditCopy,
    EditPaste,
    EditDuplicate,
    EditFind,

    // ── View actions ──
    ViewFit,
    ViewZoomIn,
    ViewZoomOut,
    ViewZoomWindow,
    ViewZoomSelection,
    ViewIso,
    ViewFront,
    ViewBack,
    ViewTop,
    ViewBottom,
    ViewLeft,
    ViewRight,
    ViewDimetric,
    ViewWireframe,
    ViewShaded,
    ViewShadedEdges,
    ViewToggleGrid,
    ViewToggleAxis,
    ViewToggleTriad,
    ViewToggleViewCube,
    ViewToggleShadows,
    ViewToggleAo,
    ViewToggleAa,
    ViewToggleEdges,
    ViewToggleNormals,
    ViewToggleSilhouette,
    ViewSectionCut,
    ViewTimeline,
    ViewPerspective,
    ViewOrthographic,
    ViewSaveLayout,
    ViewLoadLayout,

    // ── Insert actions ──
    InsertBox,
    InsertSphere,
    InsertCylinder,
    InsertCone,
    InsertTorus,
    InsertPlane,
    InsertAxis,
    InsertPoint,
    InsertCs,
    InsertSketch,
    InsertMesh,
    InsertMeshFromSolid,
    InsertRemesh,
    InsertComponent,
    InsertLinearPattern,
    InsertCircularPattern,
    InsertMirror,

    // ── Sketch actions ──
    SketchEnter,
    SketchLine,
    SketchCircle,
    SketchArc3,
    SketchArcTangent,
    SketchRectangle,
    SketchSpline,
    SketchPolygon,
    SketchPoint,
    SketchConstraintCoincident,
    SketchConstraintCollinear,
    SketchConstraintConcentric,
    SketchConstraintParallel,
    SketchConstraintPerpendicular,
    SketchConstraintTangent,
    SketchConstraintHorizontal,
    SketchConstraintVertical,
    SketchConstraintEqual,
    SketchDimLinear,
    SketchDimAngular,
    SketchDimRadial,
    SketchDimDiameter,
    SketchTrim,
    SketchExtend,
    SketchSplit,
    SketchOffset,
    SketchMirror,
    SketchPattern,
    SketchFillet,
    SketchExit,

    // ── Modify actions ──
    ModifyUnion,
    ModifySubtract,
    ModifyIntersect,
    ModifyFillet,
    ModifyChamfer,
    ModifyLoft,
    ModifySweep,
    ModifyMove,
    ModifyRotate,
    ModifyScale,
    ModifyLinearPattern,
    ModifyCircularPattern,
    ModifyMirror,
    ModifyMoveFace,
    ModifyOffsetFace,
    ModifyDeleteFace,
    ModifyReplaceFace,
    ModifySplitFace,
    ModifyMergeFaces,
    ModifySimplify,
    ModifyThicken,
    ModifyBend,
    ModifyTwist,
    ModifyTaper,
    ModifyStretch,

    // ── Sheet Metal actions ──
    SmBaseFlange,
    SmEdgeFlange,
    SmBend,
    SmHem,
    SmJog,
    SmRectRelief,
    SmTearRelief,
    SmUnfold,
    SmFold,
    SmFlatPattern,
    SmExportDxf,
    SmGaugeTable,

    // ── Assembly actions ──
    AsmAddComponent,
    AsmMateCoincident,
    AsmMateConcentric,
    AsmMateDistance,
    AsmMateAngle,
    AsmMateParallel,
    AsmMatePerpendicular,
    AsmMateTangent,
    AsmMateWidth,
    AsmMateSymmetric,
    AsmSolve,
    AsmBom,
    AsmExplode,
    AsmMotion,
    AsmDiagnostics,

    // ── CAM actions ──
    CamStockSetup,
    CamCoordinateSystem,
    CamToolLibrary,
    CamFacing,
    CamProfile,
    CamPocket,
    CamDrilling,
    CamEngraving,
    CamSurfacing,
    CamSim2d,
    CamSim3d,
    CamPostFanuc,
    CamPostSiemens,
    CamPostHaas,
    CamPostHeidenhain,
    CamPostMach3,
    CamPostLinuxCnc,
    CamPostGrbl,

    // ── Drawing actions ──
    DrwNewSheet,
    DrwViewStandard,
    DrwViewSection,
    DrwViewDetail,
    DrwViewProjected,
    DrwViewBrokenOut,
    DrwViewCrop,
    DrwViewAuxiliary,
    DrwViewExploded,
    DrwDimLinear,
    DrwDimAngular,
    DrwDimRadial,
    DrwDimDiameter,
    DrwDimOrdinate,
    DrwAnnotationNote,
    DrwAnnotationBalloon,
    DrwAnnotationSurfaceFinish,
    DrwAnnotationWelding,
    DrwAnnotationDatum,
    DrwAnnotationTolerance,
    DrwTemplateA0,
    DrwTemplateA1,
    DrwTemplateA2,
    DrwTemplateA3,
    DrwTemplateA4,
    DrwExportPdf,
    DrwExportDxf,
    DrwExportDwg,
    DrwExportSvg,

    // ── Simulation actions ──
    SimMesh,
    SimStudyStatic,
    SimStudyModal,
    SimStudyThermal,
    SimStudyBuckling,
    SimStudyFatigue,
    SimStudyNonlinear,
    SimStudyCfd,
    SimStudyEm,
    SimStudyOptimization,
    SimSolve,
    SimValidate,
    SimResultsVonMises,
    SimResultsDisplacement,
    SimResultsStrain,
    SimResultsStressXX,
    SimAnimate,

    // ── Parametric actions ──
    ParamParameters,
    ParamEquations,
    ParamDesignTable,
    ParamDependencyGraph,
    ParamVariants,

    // ── Optimize actions ──
    OptTopologyLightweight,
    OptTopologyStiff,
    OptTopologyBalanced,
    OptGenVariantA,
    OptGenVariantB,
    OptGenVariantC,
    OptGenVariantD,

    // ── GD&T actions ──
    GdtDatum,
    GdtFlatness,
    GdtStraightness,
    GdtCircularity,
    GdtCylindricity,
    GdtParallelism,
    GdtPerpendicularity,
    GdtAngularity,
    GdtPosition,
    GdtProfileLine,
    GdtProfileSurface,
    GdtCircularRunout,
    GdtTotalRunout,
    GdtAnalyze,
    GdtReports,
    GdtStackup,

    // ── Heal actions ──
    HealStitch,
    HealGapFill,
    HealRemoveDuplicates,
    HealFixOrientation,
    HealFixDegenerate,
    HealSimplify,
    HealRemoveSliver,
    HealCloseHoles,
    HealRepairTJunctions,
    MeasureDistance,
    MeasureAngle,
    MeasureLength,
    MeasureArea,
    MeasureVolume,
    MeasureMass,
    MeasureDiameter,
    MeasureRadius,
    MeasureCenter,
    AnalysisWatertight,
    AnalysisManifold,
    AnalysisCurvature,
    AnalysisDraft,
    AnalysisThickness,
    AnalysisInterference,
    AnalysisEdgeConsistency,
    AnalysisGaussianCurvature,

    // ── Mold actions ──
    MoldBaseCatalog,
    MoldRunner,
    MoldCooling,
    MoldEjection,
    MoldCavityCore,
    MoldFlow,
    MoldCoolingAnalysis,
    MoldWarpage,

    // ── Tools actions ──
    ToolsOptions,
    ToolsCustomize,
    ToolsPlugins,
    ToolsScriptingConsole,
    ToolsAiSettings,
    ToolsMacroRecorder,
    ToolsPerformance,
    ToolsTheme,
    ToolsUiLayout,

    // ── Scripting actions ──
    ScrScriptList,
    ScrLoadScript,
    ScrRecordMacro,
    ScrRunWithParams,
    ScrDebugStep,
    ScrProfile,
    ScrLibraryBrowser,
    ScrApiReference,

    // ── AI actions ──
    AiShapeFromText,
    AiChat,
    AiDesignReview,
    AiCostEstimate,
    AiSuggestFeature,
    AiAutoFillet,
    AiAutoPattern,
    AiAutoRepair,
    AiAutoDimension,
    AiAutoConstrain,
    AiGenVariantA,
    AiGenVariantB,
    AiGenVariantC,
    AiGenVariantD,
    AiOptLightweight,
    AiOptStiff,
    AiOptBalanced,
    AiOptCustom,
    AiSettings,

    // ── Window actions ──
    WinCloseAll,
    WinCascade,
    WinTileH,
    WinTileV,
    WinNextTab,
    WinPrevTab,
    WinSaveLayout,

    // ── Help actions ──
    HelpAbout,
    HelpDocs,
    HelpForum,
    HelpReportBug,
    HelpAssetsLibrary,
    HelpCheckUpdates,
    HelpTutorialGettingStarted,
    HelpTutorialSketch,
    HelpTutorialAssembly,
    HelpExampleBracket,
    HelpExampleBolt,
    HelpExampleGear,
    HelpExampleEngine,
    HelpExampleMold,
    HelpExampleSheetMetal,
    HelpExampleAssembly,
}

impl Default for MenuAction {
    fn default() -> Self { MenuAction::None }
}

/// Render the complete menu bar with 21 menus.
/// Returns the action if a menu item was clicked.
pub fn render_menu_bar(ctx: &egui::Context) -> Option<MenuAction> {
    let mut action = None;

    egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
        egui::menu::bar(ui, |ui| {
            if action.is_none() { action = render_file_menu(ui).take(); }
            if action.is_none() { action = render_edit_menu(ui).take(); }
            if action.is_none() { action = render_view_menu(ui).take(); }
            if action.is_none() { action = render_insert_menu(ui).take(); }
            if action.is_none() { action = render_sketch_menu(ui).take(); }
            if action.is_none() { action = render_modify_menu(ui).take(); }
            if action.is_none() { action = render_sheetmetal_menu(ui).take(); }
            if action.is_none() { action = render_assembly_menu(ui).take(); }
            if action.is_none() { action = render_cam_menu(ui).take(); }
            if action.is_none() { action = render_drawing_menu(ui).take(); }
            if action.is_none() { action = render_simulation_menu(ui).take(); }
            if action.is_none() { action = render_parametric_menu(ui).take(); }
            if action.is_none() { action = render_optimize_menu(ui).take(); }
            if action.is_none() { action = render_gdt_menu(ui).take(); }
            if action.is_none() { action = render_heal_menu(ui).take(); }
            if action.is_none() { action = render_mold_menu(ui).take(); }
            if action.is_none() { action = render_tools_menu(ui).take(); }
            if action.is_none() { action = render_scripting_menu(ui).take(); }
            if action.is_none() { action = render_ai_menu(ui).take(); }
            if action.is_none() { action = render_window_menu(ui).take(); }
            if action.is_none() { action = render_help_menu(ui).take(); }
        });
    });

    action
}

fn render_file_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("File", |ui| {
        if ui.button("New").clicked() { action = Some(MenuAction::FileNew); ui.close_menu(); return; }
        if ui.button("Open…").clicked() { action = Some(MenuAction::FileOpen); ui.close_menu(); return; }
        ui.separator();
        if ui.button("Save").clicked() { action = Some(MenuAction::FileSave); ui.close_menu(); return; }
        if ui.button("Save As…").clicked() { action = Some(MenuAction::FileSaveAs); ui.close_menu(); return; }
        ui.separator();
        ui.menu_button("Import", |ui| {
            if ui.button("STEP (*.stp, *.step)").clicked() { action = Some(MenuAction::FileImportStep); ui.close_menu(); return; }
            if ui.button("STL (*.stl)").clicked() { action = Some(MenuAction::FileImportStl); ui.close_menu(); return; }
            if ui.button("OBJ (*.obj)").clicked() { action = Some(MenuAction::FileImportObj); ui.close_menu(); return; }
            if ui.button("PLY (*.ply)").clicked() { action = Some(MenuAction::FileImportPly); ui.close_menu(); return; }
            if ui.button("DXF (*.dxf)").clicked() { action = Some(MenuAction::FileImportDxf); ui.close_menu(); return; }
            if ui.button("Point Cloud (*.xyz, *.las)").clicked() { action = Some(MenuAction::FileImportPointCloud); ui.close_menu(); return; }
        });
        ui.menu_button("Export", |ui| {
            if ui.button("STEP (AP214)").clicked() { action = Some(MenuAction::FileExportStep); ui.close_menu(); return; }
            if ui.button("STEP (AP242)").clicked() { action = Some(MenuAction::FileExportStep); ui.close_menu(); return; }
            if ui.button("STL (*.stl)").clicked() { action = Some(MenuAction::FileExportStl); ui.close_menu(); return; }
            if ui.button("OBJ (*.obj)").clicked() { action = Some(MenuAction::FileExportObj); ui.close_menu(); return; }
            if ui.button("GLTF (*.gltf)").clicked() { action = Some(MenuAction::FileExportGltf); ui.close_menu(); return; }
            if ui.button("PDF (*.pdf)").clicked() { action = Some(MenuAction::FileExportPdf); ui.close_menu(); return; }
            if ui.button("DXF (*.dxf)").clicked() { action = Some(MenuAction::FileExportDxf); ui.close_menu(); return; }
        });
        ui.separator();
        ui.menu_button("Recent", |ui| {
            ui.label("(no recent files)");
        });
        ui.separator();
        if ui.button("Print / Plot…").clicked() { action = Some(MenuAction::FilePrint); ui.close_menu(); return; }
        ui.separator();
        if ui.button("Exit").clicked() { action = Some(MenuAction::FileQuit); ui.close_menu(); return; }
    });
    action
}

fn render_edit_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Edit", |ui| {
        if ui.button("Undo  ⌘Z").clicked() { action = Some(MenuAction::EditUndo); ui.close_menu(); return; }
        if ui.button("Redo  ⌘⇧Z").clicked() { action = Some(MenuAction::EditRedo); ui.close_menu(); return; }
        ui.separator();
        ui.menu_button("History", |ui| {
            ui.label("Snapshot / Branch / Diff / Tree");
        });
        ui.separator();
        if ui.button("Cut  ⌘X").clicked() { action = Some(MenuAction::EditCut); ui.close_menu(); return; }
        if ui.button("Copy  ⌘C").clicked() { action = Some(MenuAction::EditCopy); ui.close_menu(); return; }
        if ui.button("Paste  ⌘V").clicked() { action = Some(MenuAction::EditPaste); ui.close_menu(); return; }
        if ui.button("Duplicate  ⌘D").clicked() { action = Some(MenuAction::EditDuplicate); ui.close_menu(); return; }
        ui.separator();
        if ui.button("Find…").clicked() { action = Some(MenuAction::EditFind); ui.close_menu(); return; }
    });
    action
}

fn render_view_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("View", |ui| {
        ui.menu_button("Orient", |ui| {
            let orients = [
                ("ISO", MenuAction::ViewIso),
                ("Front", MenuAction::ViewFront),
                ("Back", MenuAction::ViewBack),
                ("Top", MenuAction::ViewTop),
                ("Bottom", MenuAction::ViewBottom),
                ("Left", MenuAction::ViewLeft),
                ("Right", MenuAction::ViewRight),
                ("Dimetric", MenuAction::ViewDimetric),
            ];
            for (label, act) in &orients {
                if ui.button(*label).clicked() { action = Some(act.clone()); ui.close_menu(); return; }
            }
        });
        ui.menu_button("Zoom", |ui| {
            if ui.button("Fit").clicked() { action = Some(MenuAction::ViewFit); ui.close_menu(); return; }
            if ui.button("Window").clicked() { action = Some(MenuAction::ViewZoomWindow); ui.close_menu(); return; }
            if ui.button("In").clicked() { action = Some(MenuAction::ViewZoomIn); ui.close_menu(); return; }
            if ui.button("Out").clicked() { action = Some(MenuAction::ViewZoomOut); ui.close_menu(); return; }
            if ui.button("Selection").clicked() { action = Some(MenuAction::ViewZoomSelection); ui.close_menu(); return; }
        });
        ui.menu_button("Display Style", |ui| {
            if ui.button("Wireframe").clicked() { action = Some(MenuAction::ViewWireframe); ui.close_menu(); return; }
            if ui.button("Shaded").clicked() { action = Some(MenuAction::ViewShaded); ui.close_menu(); return; }
            if ui.button("Shaded + Edges").clicked() { action = Some(MenuAction::ViewShadedEdges); ui.close_menu(); return; }
        });
        ui.menu_button("Options", |ui| {
            if ui.button("Toggle Grid").clicked() { action = Some(MenuAction::ViewToggleGrid); ui.close_menu(); return; }
            if ui.button("Toggle Axis").clicked() { action = Some(MenuAction::ViewToggleAxis); ui.close_menu(); return; }
            if ui.button("Toggle Triad").clicked() { action = Some(MenuAction::ViewToggleTriad); ui.close_menu(); return; }
            if ui.button("Toggle View Cube").clicked() { action = Some(MenuAction::ViewToggleViewCube); ui.close_menu(); return; }
            if ui.button("Toggle Shadows").clicked() { action = Some(MenuAction::ViewToggleShadows); ui.close_menu(); return; }
            if ui.button("Toggle Ambient Occlusion").clicked() { action = Some(MenuAction::ViewToggleAo); ui.close_menu(); return; }
            if ui.button("Toggle Anti-alias").clicked() { action = Some(MenuAction::ViewToggleAa); ui.close_menu(); return; }
            if ui.button("Toggle Edges").clicked() { action = Some(MenuAction::ViewToggleEdges); ui.close_menu(); return; }
            if ui.button("Toggle Normals").clicked() { action = Some(MenuAction::ViewToggleNormals); ui.close_menu(); return; }
            if ui.button("Toggle Silhouette").clicked() { action = Some(MenuAction::ViewToggleSilhouette); ui.close_menu(); return; }
            ui.separator();
            if ui.button("Section Cut…").clicked() { action = Some(MenuAction::ViewSectionCut); ui.close_menu(); return; }
            if ui.button("Feature Timeline…").clicked() { action = Some(MenuAction::ViewTimeline); ui.close_menu(); return; }
        });
        ui.menu_button("Camera", |ui| {
            if ui.button("Perspective").clicked() { action = Some(MenuAction::ViewPerspective); ui.close_menu(); return; }
            if ui.button("Orthographic").clicked() { action = Some(MenuAction::ViewOrthographic); ui.close_menu(); return; }
        });
        ui.menu_button("Layouts", |ui| {
            if ui.button("Save Layout…").clicked() { action = Some(MenuAction::ViewSaveLayout); ui.close_menu(); return; }
            if ui.button("Load Layout…").clicked() { action = Some(MenuAction::ViewLoadLayout); ui.close_menu(); return; }
        });
    });
    action
}

fn render_insert_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Insert", |ui| {
        ui.menu_button("Primitives", |ui| {
            if ui.button("Box").clicked() { action = Some(MenuAction::InsertBox); ui.close_menu(); return; }
            if ui.button("Sphere").clicked() { action = Some(MenuAction::InsertSphere); ui.close_menu(); return; }
            if ui.button("Cylinder").clicked() { action = Some(MenuAction::InsertCylinder); ui.close_menu(); return; }
            if ui.button("Cone").clicked() { action = Some(MenuAction::InsertCone); ui.close_menu(); return; }
            if ui.button("Torus").clicked() { action = Some(MenuAction::InsertTorus); ui.close_menu(); return; }
        });
        ui.menu_button("Reference Geometry", |ui| {
            if ui.button("Plane").clicked() { action = Some(MenuAction::InsertPlane); ui.close_menu(); return; }
            if ui.button("Axis").clicked() { action = Some(MenuAction::InsertAxis); ui.close_menu(); return; }
            if ui.button("Point").clicked() { action = Some(MenuAction::InsertPoint); ui.close_menu(); return; }
            if ui.button("Coordinate System").clicked() { action = Some(MenuAction::InsertCs); ui.close_menu(); return; }
        });
        if ui.button("Sketch").clicked() { action = Some(MenuAction::InsertSketch); ui.close_menu(); return; }
        ui.menu_button("Mesh", |ui| {
            if ui.button("Import Mesh").clicked() { action = Some(MenuAction::InsertMesh); ui.close_menu(); return; }
            if ui.button("Mesh from Solid").clicked() { action = Some(MenuAction::InsertMeshFromSolid); ui.close_menu(); return; }
            if ui.button("Remesh").clicked() { action = Some(MenuAction::InsertRemesh); ui.close_menu(); return; }
        });
        if ui.button("Component").clicked() { action = Some(MenuAction::InsertComponent); ui.close_menu(); return; }
        ui.menu_button("Pattern", |ui| {
            if ui.button("Linear Pattern").clicked() { action = Some(MenuAction::InsertLinearPattern); ui.close_menu(); return; }
            if ui.button("Circular Pattern").clicked() { action = Some(MenuAction::InsertCircularPattern); ui.close_menu(); return; }
            if ui.button("Mirror").clicked() { action = Some(MenuAction::InsertMirror); ui.close_menu(); return; }
        });
    });
    action
}

fn render_sketch_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Sketch", |ui| {
        ui.menu_button("Draw", |ui| {
            if ui.button("Line").clicked() { action = Some(MenuAction::SketchLine); ui.close_menu(); return; }
            if ui.button("Circle").clicked() { action = Some(MenuAction::SketchCircle); ui.close_menu(); return; }
            if ui.button("Arc (3-Point)").clicked() { action = Some(MenuAction::SketchArc3); ui.close_menu(); return; }
            if ui.button("Arc (Tangent)").clicked() { action = Some(MenuAction::SketchArcTangent); ui.close_menu(); return; }
            if ui.button("Rectangle").clicked() { action = Some(MenuAction::SketchRectangle); ui.close_menu(); return; }
            if ui.button("Spline").clicked() { action = Some(MenuAction::SketchSpline); ui.close_menu(); return; }
            if ui.button("Polygon").clicked() { action = Some(MenuAction::SketchPolygon); ui.close_menu(); return; }
            if ui.button("Point").clicked() { action = Some(MenuAction::SketchPoint); ui.close_menu(); return; }
        });
        ui.menu_button("Constrain", |ui| {
            if ui.button("Coincident").clicked() { action = Some(MenuAction::SketchConstraintCoincident); ui.close_menu(); return; }
            if ui.button("Collinear").clicked() { action = Some(MenuAction::SketchConstraintCollinear); ui.close_menu(); return; }
            if ui.button("Concentric").clicked() { action = Some(MenuAction::SketchConstraintConcentric); ui.close_menu(); return; }
            if ui.button("Parallel").clicked() { action = Some(MenuAction::SketchConstraintParallel); ui.close_menu(); return; }
            if ui.button("Perpendicular").clicked() { action = Some(MenuAction::SketchConstraintPerpendicular); ui.close_menu(); return; }
            if ui.button("Tangent").clicked() { action = Some(MenuAction::SketchConstraintTangent); ui.close_menu(); return; }
            if ui.button("Horizontal").clicked() { action = Some(MenuAction::SketchConstraintHorizontal); ui.close_menu(); return; }
            if ui.button("Vertical").clicked() { action = Some(MenuAction::SketchConstraintVertical); ui.close_menu(); return; }
            if ui.button("Equal").clicked() { action = Some(MenuAction::SketchConstraintEqual); ui.close_menu(); return; }
        });
        ui.menu_button("Dimension", |ui| {
            if ui.button("Linear").clicked() { action = Some(MenuAction::SketchDimLinear); ui.close_menu(); return; }
            if ui.button("Angular").clicked() { action = Some(MenuAction::SketchDimAngular); ui.close_menu(); return; }
            if ui.button("Radial").clicked() { action = Some(MenuAction::SketchDimRadial); ui.close_menu(); return; }
            if ui.button("Diameter").clicked() { action = Some(MenuAction::SketchDimDiameter); ui.close_menu(); return; }
        });
        ui.menu_button("Modify", |ui| {
            if ui.button("Trim").clicked() { action = Some(MenuAction::SketchTrim); ui.close_menu(); return; }
            if ui.button("Extend").clicked() { action = Some(MenuAction::SketchExtend); ui.close_menu(); return; }
            if ui.button("Split").clicked() { action = Some(MenuAction::SketchSplit); ui.close_menu(); return; }
            if ui.button("Offset").clicked() { action = Some(MenuAction::SketchOffset); ui.close_menu(); return; }
            if ui.button("Mirror").clicked() { action = Some(MenuAction::SketchMirror); ui.close_menu(); return; }
            if ui.button("Pattern").clicked() { action = Some(MenuAction::SketchPattern); ui.close_menu(); return; }
            if ui.button("Fillet").clicked() { action = Some(MenuAction::SketchFillet); ui.close_menu(); return; }
        });
        ui.separator();
        if ui.button("Exit Sketch").clicked() { action = Some(MenuAction::SketchExit); ui.close_menu(); return; }
    });
    action
}

fn render_modify_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Modify", |ui| {
        ui.menu_button("Boolean", |ui| {
            if ui.button("Union").clicked() { action = Some(MenuAction::ModifyUnion); ui.close_menu(); return; }
            if ui.button("Subtract").clicked() { action = Some(MenuAction::ModifySubtract); ui.close_menu(); return; }
            if ui.button("Intersect").clicked() { action = Some(MenuAction::ModifyIntersect); ui.close_menu(); return; }
        });
        ui.menu_button("Edge", |ui| {
            if ui.button("Fillet").clicked() { action = Some(MenuAction::ModifyFillet); ui.close_menu(); return; }
            if ui.button("Chamfer").clicked() { action = Some(MenuAction::ModifyChamfer); ui.close_menu(); return; }
        });
        ui.menu_button("Surface", |ui| {
            if ui.button("Loft").clicked() { action = Some(MenuAction::ModifyLoft); ui.close_menu(); return; }
            if ui.button("Sweep").clicked() { action = Some(MenuAction::ModifySweep); ui.close_menu(); return; }
        });
        ui.menu_button("Transform", |ui| {
            if ui.button("Move").clicked() { action = Some(MenuAction::ModifyMove); ui.close_menu(); return; }
            if ui.button("Rotate").clicked() { action = Some(MenuAction::ModifyRotate); ui.close_menu(); return; }
            if ui.button("Scale").clicked() { action = Some(MenuAction::ModifyScale); ui.close_menu(); return; }
        });
        ui.menu_button("Pattern", |ui| {
            if ui.button("Linear").clicked() { action = Some(MenuAction::ModifyLinearPattern); ui.close_menu(); return; }
            if ui.button("Circular").clicked() { action = Some(MenuAction::ModifyCircularPattern); ui.close_menu(); return; }
            if ui.button("Mirror").clicked() { action = Some(MenuAction::ModifyMirror); ui.close_menu(); return; }
        });
        ui.menu_button("Direct Modeling", |ui| {
            if ui.button("Move Face").clicked() { action = Some(MenuAction::ModifyMoveFace); ui.close_menu(); return; }
            if ui.button("Offset Face").clicked() { action = Some(MenuAction::ModifyOffsetFace); ui.close_menu(); return; }
            if ui.button("Delete Face").clicked() { action = Some(MenuAction::ModifyDeleteFace); ui.close_menu(); return; }
            if ui.button("Replace Face").clicked() { action = Some(MenuAction::ModifyReplaceFace); ui.close_menu(); return; }
            if ui.button("Split Face").clicked() { action = Some(MenuAction::ModifySplitFace); ui.close_menu(); return; }
            if ui.button("Merge Faces").clicked() { action = Some(MenuAction::ModifyMergeFaces); ui.close_menu(); return; }
            if ui.button("Simplify").clicked() { action = Some(MenuAction::ModifySimplify); ui.close_menu(); return; }
            if ui.button("Thicken").clicked() { action = Some(MenuAction::ModifyThicken); ui.close_menu(); return; }
        });
        ui.menu_button("Deform", |ui| {
            if ui.button("Bend").clicked() { action = Some(MenuAction::ModifyBend); ui.close_menu(); return; }
            if ui.button("Twist").clicked() { action = Some(MenuAction::ModifyTwist); ui.close_menu(); return; }
            if ui.button("Taper").clicked() { action = Some(MenuAction::ModifyTaper); ui.close_menu(); return; }
            if ui.button("Stretch").clicked() { action = Some(MenuAction::ModifyStretch); ui.close_menu(); return; }
        });
    });
    action
}

fn render_sheetmetal_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Sheet Metal", |ui| {
        ui.menu_button("Flange", |ui| {
            if ui.button("Base Flange").clicked() { action = Some(MenuAction::SmBaseFlange); ui.close_menu(); return; }
            if ui.button("Edge Flange").clicked() { action = Some(MenuAction::SmEdgeFlange); ui.close_menu(); return; }
        });
        ui.menu_button("Bend", |ui| {
            if ui.button("Bend").clicked() { action = Some(MenuAction::SmBend); ui.close_menu(); return; }
            if ui.button("Hem").clicked() { action = Some(MenuAction::SmHem); ui.close_menu(); return; }
            if ui.button("Jog").clicked() { action = Some(MenuAction::SmJog); ui.close_menu(); return; }
        });
        ui.menu_button("Relief", |ui| {
            if ui.button("Rectangular Relief").clicked() { action = Some(MenuAction::SmRectRelief); ui.close_menu(); return; }
            if ui.button("Tear Relief").clicked() { action = Some(MenuAction::SmTearRelief); ui.close_menu(); return; }
        });
        ui.menu_button("Flatten", |ui| {
            if ui.button("Unfold").clicked() { action = Some(MenuAction::SmUnfold); ui.close_menu(); return; }
            if ui.button("Fold").clicked() { action = Some(MenuAction::SmFold); ui.close_menu(); return; }
            if ui.button("Flat Pattern").clicked() { action = Some(MenuAction::SmFlatPattern); ui.close_menu(); return; }
            if ui.button("Export DXF").clicked() { action = Some(MenuAction::SmExportDxf); ui.close_menu(); return; }
        });
        ui.separator();
        if ui.button("Gauge Table").clicked() { action = Some(MenuAction::SmGaugeTable); ui.close_menu(); return; }
    });
    action
}

fn render_assembly_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Assembly", |ui| {
        if ui.button("Add Component").clicked() { action = Some(MenuAction::AsmAddComponent); ui.close_menu(); return; }
        ui.menu_button("Mate", |ui| {
            if ui.button("Coincident").clicked() { action = Some(MenuAction::AsmMateCoincident); ui.close_menu(); return; }
            if ui.button("Concentric").clicked() { action = Some(MenuAction::AsmMateConcentric); ui.close_menu(); return; }
            if ui.button("Distance").clicked() { action = Some(MenuAction::AsmMateDistance); ui.close_menu(); return; }
            if ui.button("Angle").clicked() { action = Some(MenuAction::AsmMateAngle); ui.close_menu(); return; }
            if ui.button("Parallel").clicked() { action = Some(MenuAction::AsmMateParallel); ui.close_menu(); return; }
            if ui.button("Perpendicular").clicked() { action = Some(MenuAction::AsmMatePerpendicular); ui.close_menu(); return; }
            if ui.button("Tangent").clicked() { action = Some(MenuAction::AsmMateTangent); ui.close_menu(); return; }
            if ui.button("Width").clicked() { action = Some(MenuAction::AsmMateWidth); ui.close_menu(); return; }
            if ui.button("Symmetric").clicked() { action = Some(MenuAction::AsmMateSymmetric); ui.close_menu(); return; }
        });
        if ui.button("Solve").clicked() { action = Some(MenuAction::AsmSolve); ui.close_menu(); return; }
        if ui.button("BOM Editor").clicked() { action = Some(MenuAction::AsmBom); ui.close_menu(); return; }
        if ui.button("Explode").clicked() { action = Some(MenuAction::AsmExplode); ui.close_menu(); return; }
        if ui.button("Motion Study").clicked() { action = Some(MenuAction::AsmMotion); ui.close_menu(); return; }
        if ui.button("Constraint Diagnostics").clicked() { action = Some(MenuAction::AsmDiagnostics); ui.close_menu(); return; }
    });
    action
}

fn render_cam_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("CAM", |ui| {
        ui.menu_button("Setup", |ui| {
            if ui.button("Stock Setup").clicked() { action = Some(MenuAction::CamStockSetup); ui.close_menu(); return; }
            if ui.button("Coordinate System").clicked() { action = Some(MenuAction::CamCoordinateSystem); ui.close_menu(); return; }
        });
        ui.menu_button("Tools", |ui| {
            if ui.button("Tool Library").clicked() { action = Some(MenuAction::CamToolLibrary); ui.close_menu(); return; }
        });
        ui.menu_button("Operations", |ui| {
            if ui.button("Facing").clicked() { action = Some(MenuAction::CamFacing); ui.close_menu(); return; }
            if ui.button("Profile").clicked() { action = Some(MenuAction::CamProfile); ui.close_menu(); return; }
            if ui.button("Pocket").clicked() { action = Some(MenuAction::CamPocket); ui.close_menu(); return; }
            if ui.button("Drilling").clicked() { action = Some(MenuAction::CamDrilling); ui.close_menu(); return; }
            if ui.button("Engraving").clicked() { action = Some(MenuAction::CamEngraving); ui.close_menu(); return; }
            if ui.button("3D Surfacing").clicked() { action = Some(MenuAction::CamSurfacing); ui.close_menu(); return; }
        });
        ui.menu_button("Simulate", |ui| {
            if ui.button("2D Sim").clicked() { action = Some(MenuAction::CamSim2d); ui.close_menu(); return; }
            if ui.button("3D Sim").clicked() { action = Some(MenuAction::CamSim3d); ui.close_menu(); return; }
        });
        ui.menu_button("Post Process", |ui| {
            if ui.button("Fanuc").clicked() { action = Some(MenuAction::CamPostFanuc); ui.close_menu(); return; }
            if ui.button("Siemens").clicked() { action = Some(MenuAction::CamPostSiemens); ui.close_menu(); return; }
            if ui.button("Haas").clicked() { action = Some(MenuAction::CamPostHaas); ui.close_menu(); return; }
            if ui.button("Heidenhain").clicked() { action = Some(MenuAction::CamPostHeidenhain); ui.close_menu(); return; }
            if ui.button("Mach3").clicked() { action = Some(MenuAction::CamPostMach3); ui.close_menu(); return; }
            if ui.button("LinuxCNC").clicked() { action = Some(MenuAction::CamPostLinuxCnc); ui.close_menu(); return; }
            if ui.button("GRBL").clicked() { action = Some(MenuAction::CamPostGrbl); ui.close_menu(); return; }
        });
    });
    action
}

fn render_drawing_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Drawing", |ui| {
        if ui.button("New Sheet").clicked() { action = Some(MenuAction::DrwNewSheet); ui.close_menu(); return; }
        ui.menu_button("Views", |ui| {
            if ui.button("Standard").clicked() { action = Some(MenuAction::DrwViewStandard); ui.close_menu(); return; }
            if ui.button("Section").clicked() { action = Some(MenuAction::DrwViewSection); ui.close_menu(); return; }
            if ui.button("Detail").clicked() { action = Some(MenuAction::DrwViewDetail); ui.close_menu(); return; }
            if ui.button("Projected").clicked() { action = Some(MenuAction::DrwViewProjected); ui.close_menu(); return; }
            if ui.button("Broken-out").clicked() { action = Some(MenuAction::DrwViewBrokenOut); ui.close_menu(); return; }
            if ui.button("Crop").clicked() { action = Some(MenuAction::DrwViewCrop); ui.close_menu(); return; }
            if ui.button("Auxiliary").clicked() { action = Some(MenuAction::DrwViewAuxiliary); ui.close_menu(); return; }
            if ui.button("Exploded").clicked() { action = Some(MenuAction::DrwViewExploded); ui.close_menu(); return; }
        });
        ui.menu_button("Dimensions", |ui| {
            if ui.button("Linear").clicked() { action = Some(MenuAction::DrwDimLinear); ui.close_menu(); return; }
            if ui.button("Angular").clicked() { action = Some(MenuAction::DrwDimAngular); ui.close_menu(); return; }
            if ui.button("Radial").clicked() { action = Some(MenuAction::DrwDimRadial); ui.close_menu(); return; }
            if ui.button("Diameter").clicked() { action = Some(MenuAction::DrwDimDiameter); ui.close_menu(); return; }
            if ui.button("Ordinate").clicked() { action = Some(MenuAction::DrwDimOrdinate); ui.close_menu(); return; }
        });
        ui.menu_button("Annotations", |ui| {
            if ui.button("Note").clicked() { action = Some(MenuAction::DrwAnnotationNote); ui.close_menu(); return; }
            if ui.button("Balloon").clicked() { action = Some(MenuAction::DrwAnnotationBalloon); ui.close_menu(); return; }
            if ui.button("Surface Finish").clicked() { action = Some(MenuAction::DrwAnnotationSurfaceFinish); ui.close_menu(); return; }
            if ui.button("Welding").clicked() { action = Some(MenuAction::DrwAnnotationWelding); ui.close_menu(); return; }
            if ui.button("Datum").clicked() { action = Some(MenuAction::DrwAnnotationDatum); ui.close_menu(); return; }
            if ui.button("Tolerance").clicked() { action = Some(MenuAction::DrwAnnotationTolerance); ui.close_menu(); return; }
        });
        ui.menu_button("Templates", |ui| {
            if ui.button("A0").clicked() { action = Some(MenuAction::DrwTemplateA0); ui.close_menu(); return; }
            if ui.button("A1").clicked() { action = Some(MenuAction::DrwTemplateA1); ui.close_menu(); return; }
            if ui.button("A2").clicked() { action = Some(MenuAction::DrwTemplateA2); ui.close_menu(); return; }
            if ui.button("A3").clicked() { action = Some(MenuAction::DrwTemplateA3); ui.close_menu(); return; }
            if ui.button("A4").clicked() { action = Some(MenuAction::DrwTemplateA4); ui.close_menu(); return; }
        });
        ui.menu_button("Export", |ui| {
            if ui.button("PDF").clicked() { action = Some(MenuAction::DrwExportPdf); ui.close_menu(); return; }
            if ui.button("DXF").clicked() { action = Some(MenuAction::DrwExportDxf); ui.close_menu(); return; }
            if ui.button("DWG").clicked() { action = Some(MenuAction::DrwExportDwg); ui.close_menu(); return; }
            if ui.button("SVG").clicked() { action = Some(MenuAction::DrwExportSvg); ui.close_menu(); return; }
        });
    });
    action
}

fn render_simulation_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Simulation", |ui| {
        if ui.button("Mesh").clicked() { action = Some(MenuAction::SimMesh); ui.close_menu(); return; }
        ui.menu_button("Study", |ui| {
            if ui.button("Static").clicked() { action = Some(MenuAction::SimStudyStatic); ui.close_menu(); return; }
            if ui.button("Modal").clicked() { action = Some(MenuAction::SimStudyModal); ui.close_menu(); return; }
            if ui.button("Thermal").clicked() { action = Some(MenuAction::SimStudyThermal); ui.close_menu(); return; }
            if ui.button("Buckling").clicked() { action = Some(MenuAction::SimStudyBuckling); ui.close_menu(); return; }
            if ui.button("Fatigue").clicked() { action = Some(MenuAction::SimStudyFatigue); ui.close_menu(); return; }
            if ui.button("Nonlinear").clicked() { action = Some(MenuAction::SimStudyNonlinear); ui.close_menu(); return; }
            if ui.button("CFD").clicked() { action = Some(MenuAction::SimStudyCfd); ui.close_menu(); return; }
            if ui.button("Electromagnetic").clicked() { action = Some(MenuAction::SimStudyEm); ui.close_menu(); return; }
            if ui.button("Optimization").clicked() { action = Some(MenuAction::SimStudyOptimization); ui.close_menu(); return; }
        });
        ui.menu_button("Run", |ui| {
            if ui.button("Solve").clicked() { action = Some(MenuAction::SimSolve); ui.close_menu(); return; }
            if ui.button("Validate").clicked() { action = Some(MenuAction::SimValidate); ui.close_menu(); return; }
        });
        ui.menu_button("Results", |ui| {
            if ui.button("Von Mises").clicked() { action = Some(MenuAction::SimResultsVonMises); ui.close_menu(); return; }
            if ui.button("Displacement").clicked() { action = Some(MenuAction::SimResultsDisplacement); ui.close_menu(); return; }
            if ui.button("Strain").clicked() { action = Some(MenuAction::SimResultsStrain); ui.close_menu(); return; }
            if ui.button("Stress XX").clicked() { action = Some(MenuAction::SimResultsStressXX); ui.close_menu(); return; }
            if ui.button("Animate").clicked() { action = Some(MenuAction::SimAnimate); ui.close_menu(); return; }
        });
    });
    action
}

fn render_parametric_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Parametric", |ui| {
        if ui.button("Parameters").clicked() { action = Some(MenuAction::ParamParameters); ui.close_menu(); return; }
        if ui.button("Equations").clicked() { action = Some(MenuAction::ParamEquations); ui.close_menu(); return; }
        if ui.button("Design Table").clicked() { action = Some(MenuAction::ParamDesignTable); ui.close_menu(); return; }
        if ui.button("Dependency Graph").clicked() { action = Some(MenuAction::ParamDependencyGraph); ui.close_menu(); return; }
        if ui.button("Variants").clicked() { action = Some(MenuAction::ParamVariants); ui.close_menu(); return; }
    });
    action
}

fn render_optimize_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Optimize", |ui| {
        ui.menu_button("Topology Optimization", |ui| {
            if ui.button("Lightweight").clicked() { action = Some(MenuAction::OptTopologyLightweight); ui.close_menu(); return; }
            if ui.button("Stiff").clicked() { action = Some(MenuAction::OptTopologyStiff); ui.close_menu(); return; }
            if ui.button("Balanced").clicked() { action = Some(MenuAction::OptTopologyBalanced); ui.close_menu(); return; }
        });
        ui.menu_button("Generative Design", |ui| {
            if ui.button("Variant A").clicked() { action = Some(MenuAction::OptGenVariantA); ui.close_menu(); return; }
            if ui.button("Variant B").clicked() { action = Some(MenuAction::OptGenVariantB); ui.close_menu(); return; }
            if ui.button("Variant C").clicked() { action = Some(MenuAction::OptGenVariantC); ui.close_menu(); return; }
            if ui.button("Variant D").clicked() { action = Some(MenuAction::OptGenVariantD); ui.close_menu(); return; }
        });
    });
    action
}

fn render_gdt_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("GD&T", |ui| {
        if ui.button("Datum").clicked() { action = Some(MenuAction::GdtDatum); ui.close_menu(); return; }
        ui.menu_button("Form", |ui| {
            if ui.button("Flatness").clicked() { action = Some(MenuAction::GdtFlatness); ui.close_menu(); return; }
            if ui.button("Straightness").clicked() { action = Some(MenuAction::GdtStraightness); ui.close_menu(); return; }
            if ui.button("Circularity").clicked() { action = Some(MenuAction::GdtCircularity); ui.close_menu(); return; }
            if ui.button("Cylindricity").clicked() { action = Some(MenuAction::GdtCylindricity); ui.close_menu(); return; }
        });
        ui.menu_button("Orientation", |ui| {
            if ui.button("Parallelism").clicked() { action = Some(MenuAction::GdtParallelism); ui.close_menu(); return; }
            if ui.button("Perpendicularity").clicked() { action = Some(MenuAction::GdtPerpendicularity); ui.close_menu(); return; }
            if ui.button("Angularity").clicked() { action = Some(MenuAction::GdtAngularity); ui.close_menu(); return; }
        });
        if ui.button("Position").clicked() { action = Some(MenuAction::GdtPosition); ui.close_menu(); return; }
        ui.menu_button("Profile", |ui| {
            if ui.button("Profile of Line").clicked() { action = Some(MenuAction::GdtProfileLine); ui.close_menu(); return; }
            if ui.button("Profile of Surface").clicked() { action = Some(MenuAction::GdtProfileSurface); ui.close_menu(); return; }
        });
        ui.menu_button("Runout", |ui| {
            if ui.button("Circular Runout").clicked() { action = Some(MenuAction::GdtCircularRunout); ui.close_menu(); return; }
            if ui.button("Total Runout").clicked() { action = Some(MenuAction::GdtTotalRunout); ui.close_menu(); return; }
        });
        ui.separator();
        if ui.button("Analyze").clicked() { action = Some(MenuAction::GdtAnalyze); ui.close_menu(); return; }
        if ui.button("Reports").clicked() { action = Some(MenuAction::GdtReports); ui.close_menu(); return; }
        if ui.button("Stackup Analysis").clicked() { action = Some(MenuAction::GdtStackup); ui.close_menu(); return; }
    });
    action
}

fn render_heal_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Heal", |ui| {
        ui.menu_button("Heal", |ui| {
            if ui.button("Stitch").clicked() { action = Some(MenuAction::HealStitch); ui.close_menu(); return; }
            if ui.button("Gap Fill").clicked() { action = Some(MenuAction::HealGapFill); ui.close_menu(); return; }
            if ui.button("Remove Duplicates").clicked() { action = Some(MenuAction::HealRemoveDuplicates); ui.close_menu(); return; }
            if ui.button("Fix Orientation").clicked() { action = Some(MenuAction::HealFixOrientation); ui.close_menu(); return; }
            if ui.button("Fix Degenerate").clicked() { action = Some(MenuAction::HealFixDegenerate); ui.close_menu(); return; }
            if ui.button("Simplify").clicked() { action = Some(MenuAction::HealSimplify); ui.close_menu(); return; }
            if ui.button("Remove Sliver").clicked() { action = Some(MenuAction::HealRemoveSliver); ui.close_menu(); return; }
            if ui.button("Close Holes").clicked() { action = Some(MenuAction::HealCloseHoles); ui.close_menu(); return; }
            if ui.button("Repair T-Junctions").clicked() { action = Some(MenuAction::HealRepairTJunctions); ui.close_menu(); return; }
        });
        ui.menu_button("Measure", |ui| {
            if ui.button("Distance").clicked() { action = Some(MenuAction::MeasureDistance); ui.close_menu(); return; }
            if ui.button("Angle").clicked() { action = Some(MenuAction::MeasureAngle); ui.close_menu(); return; }
            if ui.button("Length").clicked() { action = Some(MenuAction::MeasureLength); ui.close_menu(); return; }
            if ui.button("Area").clicked() { action = Some(MenuAction::MeasureArea); ui.close_menu(); return; }
            if ui.button("Volume").clicked() { action = Some(MenuAction::MeasureVolume); ui.close_menu(); return; }
            if ui.button("Mass").clicked() { action = Some(MenuAction::MeasureMass); ui.close_menu(); return; }
            if ui.button("Diameter").clicked() { action = Some(MenuAction::MeasureDiameter); ui.close_menu(); return; }
            if ui.button("Radius").clicked() { action = Some(MenuAction::MeasureRadius); ui.close_menu(); return; }
            if ui.button("Center").clicked() { action = Some(MenuAction::MeasureCenter); ui.close_menu(); return; }
        });
        ui.menu_button("Analysis", |ui| {
            if ui.button("Watertight Check").clicked() { action = Some(MenuAction::AnalysisWatertight); ui.close_menu(); return; }
            if ui.button("Manifold Check").clicked() { action = Some(MenuAction::AnalysisManifold); ui.close_menu(); return; }
            if ui.button("Curvature").clicked() { action = Some(MenuAction::AnalysisCurvature); ui.close_menu(); return; }
            if ui.button("Draft Analysis").clicked() { action = Some(MenuAction::AnalysisDraft); ui.close_menu(); return; }
            if ui.button("Thickness").clicked() { action = Some(MenuAction::AnalysisThickness); ui.close_menu(); return; }
            if ui.button("Interference").clicked() { action = Some(MenuAction::AnalysisInterference); ui.close_menu(); return; }
            if ui.button("Edge Consistency").clicked() { action = Some(MenuAction::AnalysisEdgeConsistency); ui.close_menu(); return; }
            if ui.button("Gaussian Curvature").clicked() { action = Some(MenuAction::AnalysisGaussianCurvature); ui.close_menu(); return; }
        });
    });
    action
}

fn render_mold_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Mold", |ui| {
        if ui.button("Mold Base Catalog").clicked() { action = Some(MenuAction::MoldBaseCatalog); ui.close_menu(); return; }
        if ui.button("Runner System").clicked() { action = Some(MenuAction::MoldRunner); ui.close_menu(); return; }
        if ui.button("Cooling System").clicked() { action = Some(MenuAction::MoldCooling); ui.close_menu(); return; }
        if ui.button("Ejection System").clicked() { action = Some(MenuAction::MoldEjection); ui.close_menu(); return; }
        if ui.button("Cavity/Core").clicked() { action = Some(MenuAction::MoldCavityCore); ui.close_menu(); return; }
        if ui.button("Flow Analysis").clicked() { action = Some(MenuAction::MoldFlow); ui.close_menu(); return; }
        if ui.button("Cooling Analysis").clicked() { action = Some(MenuAction::MoldCoolingAnalysis); ui.close_menu(); return; }
        if ui.button("Warpage Analysis").clicked() { action = Some(MenuAction::MoldWarpage); ui.close_menu(); return; }
    });
    action
}

fn render_tools_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Tools", |ui| {
        if ui.button("Options").clicked() { action = Some(MenuAction::ToolsOptions); ui.close_menu(); return; }
        if ui.button("Customize").clicked() { action = Some(MenuAction::ToolsCustomize); ui.close_menu(); return; }
        if ui.button("Plugins Manager").clicked() { action = Some(MenuAction::ToolsPlugins); ui.close_menu(); return; }
        if ui.button("Scripting Console").clicked() { action = Some(MenuAction::ToolsScriptingConsole); ui.close_menu(); return; }
        if ui.button("AI Settings").clicked() { action = Some(MenuAction::ToolsAiSettings); ui.close_menu(); return; }
        if ui.button("Macro Recorder").clicked() { action = Some(MenuAction::ToolsMacroRecorder); ui.close_menu(); return; }
        if ui.button("Performance Monitor").clicked() { action = Some(MenuAction::ToolsPerformance); ui.close_menu(); return; }
        if ui.button("Theme").clicked() { action = Some(MenuAction::ToolsTheme); ui.close_menu(); return; }
        if ui.button("UI Layout").clicked() { action = Some(MenuAction::ToolsUiLayout); ui.close_menu(); return; }
    });
    action
}

fn render_scripting_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Scripting", |ui| {
        if ui.button("Script List").clicked() { action = Some(MenuAction::ScrScriptList); ui.close_menu(); return; }
        if ui.button("Load Script").clicked() { action = Some(MenuAction::ScrLoadScript); ui.close_menu(); return; }
        if ui.button("Record Macro").clicked() { action = Some(MenuAction::ScrRecordMacro); ui.close_menu(); return; }
        if ui.button("Run with Parameters").clicked() { action = Some(MenuAction::ScrRunWithParams); ui.close_menu(); return; }
        if ui.button("Debug Step").clicked() { action = Some(MenuAction::ScrDebugStep); ui.close_menu(); return; }
        if ui.button("Profile").clicked() { action = Some(MenuAction::ScrProfile); ui.close_menu(); return; }
        if ui.button("Library Browser").clicked() { action = Some(MenuAction::ScrLibraryBrowser); ui.close_menu(); return; }
        if ui.button("API Reference").clicked() { action = Some(MenuAction::ScrApiReference); ui.close_menu(); return; }
    });
    action
}

fn render_ai_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("AI", |ui| {
        if ui.button("Shape from Text").clicked() { action = Some(MenuAction::AiShapeFromText); ui.close_menu(); return; }
        ui.menu_button("Assistant", |ui| {
            if ui.button("Chat").clicked() { action = Some(MenuAction::AiChat); ui.close_menu(); return; }
            if ui.button("Design Review").clicked() { action = Some(MenuAction::AiDesignReview); ui.close_menu(); return; }
            if ui.button("Cost Estimate").clicked() { action = Some(MenuAction::AiCostEstimate); ui.close_menu(); return; }
            if ui.button("Suggest Feature").clicked() { action = Some(MenuAction::AiSuggestFeature); ui.close_menu(); return; }
        });
        ui.menu_button("Smart", |ui| {
            if ui.button("Auto-Fillet").clicked() { action = Some(MenuAction::AiAutoFillet); ui.close_menu(); return; }
            if ui.button("Auto-Pattern").clicked() { action = Some(MenuAction::AiAutoPattern); ui.close_menu(); return; }
            if ui.button("Auto-Repair").clicked() { action = Some(MenuAction::AiAutoRepair); ui.close_menu(); return; }
            if ui.button("Auto-Dimension").clicked() { action = Some(MenuAction::AiAutoDimension); ui.close_menu(); return; }
            if ui.button("Auto-Constrain").clicked() { action = Some(MenuAction::AiAutoConstrain); ui.close_menu(); return; }
        });
        ui.menu_button("Generate", |ui| {
            if ui.button("Variant A").clicked() { action = Some(MenuAction::AiGenVariantA); ui.close_menu(); return; }
            if ui.button("Variant B").clicked() { action = Some(MenuAction::AiGenVariantB); ui.close_menu(); return; }
            if ui.button("Variant C").clicked() { action = Some(MenuAction::AiGenVariantC); ui.close_menu(); return; }
            if ui.button("Variant D").clicked() { action = Some(MenuAction::AiGenVariantD); ui.close_menu(); return; }
        });
        ui.menu_button("Optimize", |ui| {
            if ui.button("Lightweight").clicked() { action = Some(MenuAction::AiOptLightweight); ui.close_menu(); return; }
            if ui.button("Stiff").clicked() { action = Some(MenuAction::AiOptStiff); ui.close_menu(); return; }
            if ui.button("Balanced").clicked() { action = Some(MenuAction::AiOptBalanced); ui.close_menu(); return; }
            if ui.button("Custom").clicked() { action = Some(MenuAction::AiOptCustom); ui.close_menu(); return; }
        });
        if ui.button("AI Settings").clicked() { action = Some(MenuAction::AiSettings); ui.close_menu(); return; }
    });
    action
}

fn render_window_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Window", |ui| {
        if ui.button("Close All").clicked() { action = Some(MenuAction::WinCloseAll); ui.close_menu(); return; }
        if ui.button("Cascade").clicked() { action = Some(MenuAction::WinCascade); ui.close_menu(); return; }
        if ui.button("Tile Horizontal").clicked() { action = Some(MenuAction::WinTileH); ui.close_menu(); return; }
        if ui.button("Tile Vertical").clicked() { action = Some(MenuAction::WinTileV); ui.close_menu(); return; }
        ui.separator();
        if ui.button("Next Tab").clicked() { action = Some(MenuAction::WinNextTab); ui.close_menu(); return; }
        if ui.button("Previous Tab").clicked() { action = Some(MenuAction::WinPrevTab); ui.close_menu(); return; }
        ui.separator();
        if ui.button("Save Layout").clicked() { action = Some(MenuAction::WinSaveLayout); ui.close_menu(); return; }
        ui.separator();
        ui.label("Open Documents:");
        ui.label("  • model.step");
    });
    action
}

fn render_help_menu(ui: &mut egui::Ui) -> Option<MenuAction> {
    let mut action = None;
    ui.menu_button("Help", |ui| {
        if ui.button("Check for Updates").clicked() { action = Some(MenuAction::HelpCheckUpdates); ui.close_menu(); return; }
        if ui.button("About BRepCAD").clicked() { action = Some(MenuAction::HelpAbout); ui.close_menu(); return; }
        ui.separator();
        if ui.button("Documentation").clicked() { action = Some(MenuAction::HelpDocs); ui.close_menu(); return; }
        if ui.button("Forum").clicked() { action = Some(MenuAction::HelpForum); ui.close_menu(); return; }
        if ui.button("Report Bug").clicked() { action = Some(MenuAction::HelpReportBug); ui.close_menu(); return; }
        if ui.button("Assets Library").clicked() { action = Some(MenuAction::HelpAssetsLibrary); ui.close_menu(); return; }
        ui.separator();
        ui.menu_button("Tutorials", |ui| {
            if ui.button("Getting Started").clicked() { action = Some(MenuAction::HelpTutorialGettingStarted); ui.close_menu(); return; }
            if ui.button("Sketch Tutorial").clicked() { action = Some(MenuAction::HelpTutorialSketch); ui.close_menu(); return; }
            if ui.button("Assembly Tutorial").clicked() { action = Some(MenuAction::HelpTutorialAssembly); ui.close_menu(); return; }
        });
        ui.menu_button("Examples", |ui| {
            if ui.button("Bracket").clicked() { action = Some(MenuAction::HelpExampleBracket); ui.close_menu(); return; }
            if ui.button("Bolt").clicked() { action = Some(MenuAction::HelpExampleBolt); ui.close_menu(); return; }
            if ui.button("Gear").clicked() { action = Some(MenuAction::HelpExampleGear); ui.close_menu(); return; }
            if ui.button("Engine Block").clicked() { action = Some(MenuAction::HelpExampleEngine); ui.close_menu(); return; }
            if ui.button("Mold Cavity").clicked() { action = Some(MenuAction::HelpExampleMold); ui.close_menu(); return; }
            if ui.button("Sheet Metal Part").clicked() { action = Some(MenuAction::HelpExampleSheetMetal); ui.close_menu(); return; }
            if ui.button("Assembly").clicked() { action = Some(MenuAction::HelpExampleAssembly); ui.close_menu(); return; }
        });
    });
    action
}
