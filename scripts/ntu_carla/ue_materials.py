"""Roadmap 072 C7, run inside the CARLA editor after `make import` (UE 4.26's Python plugin):

    UE4Editor-Cmd CarlaUE4.uproject -run=pythonscript -script=<this file> -unattended -nullrhi

The NTU package's props, trees and buildings carry the scan's colour as vertex colour (linear
RGB, de-lit) and the trees' cut-outs as its alpha (`carla_meshes.py`). UE's FBX import gives
them a plain material that ignores both, so this makes one master material,
/Game/NTU/Static/Materials/M_NTU_ScanColour, and puts it on every static mesh of the package
whose name holds Prop_, Tree_ or Building_ (the import prefixes the package's name):
- BaseColor is the vertex colour (RGB);
- OpacityMask is its alpha: masked, clip at 0.5, two-sided, as 059's Isaac crowns;
- Roughness is 0.9 and Specular 0.2, like a rough natural surface.

Then it moves those meshes out of /Game/NTU/Static/Terrain, where CARLA's import puts everything
that is not road, into the folders CARLA's tagger reads as their semantic class (Tagger.cpp):
Tree_* to Static/Vegetation, Building_* to Static/Building, Prop_* to Static/Static (the scan's
other objects mix walls, shrubs and parked cars). Redirectors stay behind for the level's
actors; the cook follows them.
"""
import unreal

PACKAGE = "/Game/NTU"
MAT_DIR = PACKAGE + "/Static/Materials"
MAT_NAME = "M_NTU_ScanColour"
PREFIXES = ("Prop_", "Tree_", "Building_")
FOLDER = {"Tree_": "Vegetation", "Building_": "Building", "Prop_": "Static"}

lib = unreal.MaterialEditingLibrary
assets = unreal.EditorAssetLibrary
path = f"{MAT_DIR}/{MAT_NAME}"
if assets.does_asset_exist(path):
    mat = assets.load_asset(path)
else:
    mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        MAT_NAME, MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
mat.set_editor_property("two_sided", True)
mat.set_editor_property("opacity_mask_clip_value", 0.5)
lib.delete_all_material_expressions(mat)
vc = lib.create_material_expression(mat, unreal.MaterialExpressionVertexColor, -400, 0)
lib.connect_material_property(vc, "", unreal.MaterialProperty.MP_BASE_COLOR)
lib.connect_material_property(vc, "A", unreal.MaterialProperty.MP_OPACITY_MASK)
rough = lib.create_material_expression(mat, unreal.MaterialExpressionConstant, -400, 200)
rough.set_editor_property("r", 0.9)
lib.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
spec = lib.create_material_expression(mat, unreal.MaterialExpressionConstant, -400, 300)
spec.set_editor_property("r", 0.2)
lib.connect_material_property(spec, "", unreal.MaterialProperty.MP_SPECULAR)
lib.recompile_material(mat)
assets.save_loaded_asset(mat)

n = 0
for a in assets.list_assets(PACKAGE, recursive=True, include_folder=False):
    name = a.split("/")[-1].split(".")[0]
    if not any(p in name for p in PREFIXES):
        continue
    mesh = assets.load_asset(a)
    if not isinstance(mesh, unreal.StaticMesh):
        continue
    for i in range(len(mesh.get_editor_property("static_materials"))):
        mesh.set_material(i, mat)
    assets.save_loaded_asset(mesh)
    n += 1
moved = 0
for a in assets.list_assets(PACKAGE + "/Static/Terrain", recursive=True, include_folder=False):
    name = a.split("/")[-1].split(".")[0]
    for p, folder in FOLDER.items():
        if p in name:
            sub = a.split("/Static/Terrain/", 1)[1].rsplit("/", 1)[0]
            if assets.rename_asset(a.split(".")[0], f"{PACKAGE}/Static/{folder}/{sub}/{name}"):
                moved += 1
            break
unreal.log_warning(f"ue_materials: {MAT_NAME} on {n} meshes; {moved} moved to their class folders")
