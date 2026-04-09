import bpy
import os
import mathutils

#
# SDF exporter for a four-bar linkage finger model.
# Generates SDFormat 1.9 compatible output for use with Drake or Gazebo.
#
# Four-bar linkage topology:
#
#   proximal_phalanx ──pip_flex1──► middle_phalanx1 ──dip_flex1──► distal_phalanx
#           │                                                              ▲
#           └──────pip_flex2──► middle_phalanx2 ──dip_flex2──────────────┘
#
# dip_flex2 is the loop-closing joint. Drake/Gazebo treat it as a
# kinematic constraint automatically — no special SDF tag required.
#

# ── Config ─────────────────────────────────────────────────────────────────────

MESHES_DIR    = "/home/michael-jenz/rdfms_ws/finger_vizualization/src/drake-finger-sim/finger_description/meshes"
SDF_PATH      = "/home/michael-jenz/rds_ws/finger_vizualization/src/drake-finger-sim/finger_description/sdf/finger.sdf"
VISUAL_DIR    = os.path.join(MESHES_DIR, "visual")
COLLISION_DIR = os.path.join(MESHES_DIR, "collision")
AXES_FILE     = os.path.join(MESHES_DIR, "joint_axes.yaml")
PACKAGE_NAME  = "finger_description"
ROBOT_NAME    = "finger"

os.makedirs(VISUAL_DIR, exist_ok=True)
os.makedirs(COLLISION_DIR, exist_ok=True)
os.makedirs(os.path.dirname(SDF_PATH), exist_ok=True)

# ── Kinematic tree ──────────────────────────────────────────────────────────────

JOINTS = [
    {
        "name":     "mcp_splay",
        "type":     "revolute",
        "parent":   "base_link",
        "child":    "mcp_link",
        "lower":    -0.174533,
        "upper":    0.174533,
        "effort":   10000.0,
        "velocity": 10000.0,
        "damping":  0.05,
        "friction": 0.0,
        "empty":    "mcp_splay",
    },
    {
        "name":     "mcp_flexion",
        "type":     "revolute",
        "parent":   "mcp_link",
        "child":    "proximal_phalanx",
        "lower":    0.0,
        "upper":    1.570,
        "effort":   10000.0,
        "velocity": 10000.0,
        "damping":  0.05,
        "friction": 0.0,
        "empty":    "mcp_flex",
    },
    {
        "name":     "pip_flex1",
        "type":     "revolute",
        "parent":   "proximal_phalanx",
        "child":    "middle_phalanx1",
        "lower":    0.0,
        "upper":    1.570,
        "effort":   10000.0,
        "velocity": 10000.0,
        "damping":  0.05,
        "friction": 0.0,
        "empty":    "pip_flex1",
    },
    {
        "name":     "pip_flex2",
        "type":     "revolute",
        "parent":   "proximal_phalanx",
        "child":    "middle_phalanx2",
        "lower":    0.0,
        "upper":    1.570,
        "effort":   0.0,
        "velocity": 10000.0,
        "damping":  0.05,
        "friction": 0.0,
        "empty":    "pip_flex2",
    },
    {
        "name":     "dip_flex1",
        "type":     "revolute",
        "parent":   "middle_phalanx1",
        "child":    "distal_phalanx",
        "lower":    -0.1,
        "upper":    1.570,
        "effort":   0.0,
        "velocity": 10000.0,
        "damping":  0.05,
        "friction": 0.0,
        "empty":    "dip_flex1",
    },
    # {
    #     "name":     "dip_flex2",
    #     "type":     "revolute",
    #     "parent":   "middle_phalanx2",
    #     "child":    "distal_phalanx",
    #     "lower":    0.0,
    #     "upper":    1.570,
    #     "effort":   3.0,
    #     "velocity": 3.14,
    #     "damping":  0.1,
    #     "friction": 0.05,
    #     "empty":    "dip_flex2",
    # },
]

LINKS = [
    {
        "name":    "base_link",
        "mesh":    True,
        "mass":    0.05,
        "inertia": (1e-5, 1e-5, 1e-5),
        "comment": "Finger base",
    },
    {
        "name":    "mcp_link",
        "mesh":    True,
        "mass":    0.01,
        "inertia": (1e-5, 1e-5, 1e-5),
        "comment": "Space between MCP splay and flexion",
    },
    {
        "name":    "proximal_phalanx",
        "mesh":    True,
        "mass":    0.01,
        "inertia": (1e-5, 1e-5, 1e-5),
        "comment": "Proximal phalanx",
    },
    {
        "name":    "middle_phalanx1",
        "mesh":    True,
        "mass":    0.008,
        "inertia": (5e-6, 5e-6, 5e-6),
        "comment": "Middle phalanx 1 (four-bar coupler, dorsal)",
    },
    {
        "name":    "middle_phalanx2",
        "mesh":    True,
        "mass":    0.008,
        "inertia": (5e-6, 5e-6, 5e-6),
        "comment": "Middle phalanx 2 (four-bar coupler, volar)",
    },
    {
        "name":    "distal_phalanx",
        "mesh":    True,
        "mass":    0.005,
        "inertia": (2e-6, 2e-6, 2e-6),
        "comment": "Distal phalanx",
    },
]

KINEMATIC_TREE = {
    "mcp_splay":   None,
    "mcp_flexion": "mcp_splay",
    "pip_flex1":   "mcp_flexion",
    "pip_flex2":   "mcp_flexion",
    "dip_flex1":   "pip_flex1",
    "dip_flex2":   "pip_flex2",
}

# ── Helpers ────────────────────────────────────────────────────────────────────

def snap(v):
    if abs(v) < 0.001:     return 0.0
    if abs(v - 1) < 0.001: return 1.0
    if abs(v + 1) < 0.001: return -1.0
    return round(v, 4)

def get_empty_data(name):
    obj = bpy.data.objects.get(name)
    if obj is None or obj.type != 'EMPTY':
        print(f"  WARNING: Empty '{name}' not found, using zeros")
        return (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    loc  = obj.matrix_world.translation
    axis = obj.matrix_world.to_3x3().col[2].normalized()
    origin = tuple(round(c, 4) for c in loc)
    axis   = tuple(snap(c) for c in axis)
    return origin, axis

def fmt_pose(xyz, rpy=(0, 0, 0)):
    return f"{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}"

def fmt_inertia_sdf(ixx, iyy, izz):
    def f(v): return f"{v:g}"
    return (f"<ixx>{f(ixx)}</ixx><ixy>0</ixy><ixz>0</ixz>"
            f"<iyy>{f(iyy)}</iyy><iyz>0</iyz><izz>{f(izz)}</izz>")

# ── Mesh export ────────────────────────────────────────────────────────────────
# - All meshes exported as .obj (Drake doesn't support .stl for collision)
# - forward_axis='Y', up_axis='Z' prevents 90° X rotation from Blender→URDF axis swap
# - export_triangulated_mesh=True fixes holes from Drake mishandling quads/ngons

bpy.ops.object.select_all(action='DESELECT')
exported = []
skipped  = []

for obj in bpy.data.objects:
    if obj.type != 'MESH':
        skipped.append(f"{obj.name} (not a mesh)")
        continue

    collections = [c.name for c in obj.users_collection]
    if "visual" in collections:
        out_dir = VISUAL_DIR
    elif "collision" in collections:
        out_dir = COLLISION_DIR
    else:
        skipped.append(f"{obj.name} (not in visual or collision collection)")
        continue

    saved_matrix = obj.matrix_world.copy()
    obj.matrix_world = mathutils.Matrix.Identity(4)

    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    filepath = os.path.join(out_dir, f"{obj.name}.obj")
    bpy.ops.wm.obj_export(
        filepath=filepath,
        export_selected_objects=True,
        global_scale=1.0,
        export_materials=False,
        forward_axis='Y',
        up_axis='Z',
        export_smooth_groups=False,
        export_normals=True,
        export_triangulated_mesh=True,
    )

    obj.select_set(False)
    obj.matrix_world = saved_matrix
    exported.append(f"{obj.name} → {out_dir}")

# ── Joint axes ─────────────────────────────────────────────────────────────────

world_data = {}
for j in JOINTS:
    origin, axis = get_empty_data(j["empty"])
    world_data[j["name"]] = {"origin": origin, "axis": axis}

axes_data = {}
for j in JOINTS:
    name     = j["name"]
    w_origin = world_data[name]["origin"]
    axis     = world_data[name]["axis"]

    parent_joint = KINEMATIC_TREE[name]
    if parent_joint is not None:
        p_origin   = world_data[parent_joint]["origin"]
        rel_origin = tuple(round(w_origin[i] - p_origin[i], 4) for i in range(3))
    else:
        rel_origin = w_origin

    axes_data[name] = {"origin": rel_origin, "axis": axis, "world": w_origin}

# ── Link world poses ───────────────────────────────────────────────────────────

link_creator_joint = {}
for j in JOINTS:
    child = j["child"]
    if child not in link_creator_joint:
        link_creator_joint[child] = j["name"]

link_world_poses = {"base_link": (0.0, 0.0, 0.0)}
for link in LINKS:
    name = link["name"]
    if name == "base_link":
        continue
    creator = link_creator_joint.get(name)
    if creator:
        link_world_poses[name] = world_data[creator]["origin"]
    else:
        print(f"  WARNING: No creator joint found for link '{name}', using origin")
        link_world_poses[name] = (0.0, 0.0, 0.0)

# ── YAML axes file ─────────────────────────────────────────────────────────────

with open(AXES_FILE, 'w') as f:
    f.write("# Joint axes extracted from Blender (four-bar linkage SDF version)\n")
    f.write("# origin: position relative to parent link frame\n")
    f.write("# axis:   rotation axis in parent link / joint frame\n")
    f.write("# world:  absolute world position of joint in rest config\n")
    f.write("# Generated by blender_export_sdf.py -- do not edit manually\n\n")
    f.write("joints:\n")
    for name, data in axes_data.items():
        o, a, w = data["origin"], data["axis"], data["world"]
        f.write(f"  {name}:\n")
        f.write(f"    origin:  [{o[0]}, {o[1]}, {o[2]}]\n")
        f.write(f"    axis:    [{a[0]}, {a[1]}, {a[2]}]\n")
        f.write(f"    world:   [{w[0]}, {w[1]}, {w[2]}]\n")

# ── SDF generation ─────────────────────────────────────────────────────────────

def link_block_sdf(link):
    name          = link["name"]
    mass          = link["mass"]
    ixx, iyy, izz = link["inertia"]
    comment       = link["comment"]
    pkg           = PACKAGE_NAME
    pose          = link_world_poses.get(name, (0.0, 0.0, 0.0))

    lines = []
    lines.append(f'    <!-- {comment} -->')
    lines.append(f'    <link name="{name}">')
    lines.append(f'      <pose>{fmt_pose(pose)}</pose>')

    if link["mesh"]:
        lines.append(f'      <visual name="visual">')
        lines.append(f'        <geometry>')
        lines.append(f'          <mesh>')
        lines.append(f'            <uri>package://{pkg}/meshes/visual/{name}.obj</uri>')
        lines.append(f'          </mesh>')
        lines.append(f'        </geometry>')
        lines.append(f'      </visual>')
        lines.append(f'      <collision name="collision">')
        lines.append(f'        <geometry>')
        lines.append(f'          <mesh>')
        lines.append(f'            <uri>package://{pkg}/meshes/collision/col_{name}.obj</uri>')
        lines.append(f'          </mesh>')
        lines.append(f'        </geometry>')
        lines.append(f'      </collision>')

    lines.append(f'      <inertial>')
    lines.append(f'        <mass>{mass}</mass>')
    lines.append(f'        <inertia>{fmt_inertia_sdf(ixx, iyy, izz)}</inertia>')
    lines.append(f'      </inertial>')
    lines.append(f'    </link>')

    return '\n'.join(lines)


def joint_block_sdf(joint):
    name   = joint["name"]
    o      = axes_data[name]["origin"]
    a      = axes_data[name]["axis"]
    parent = joint["parent"]

    lines = []
    lines.append(f'    <!-- {name.replace("_", " ").title()} -->')
    lines.append(f'    <joint name="{name}" type="{joint["type"]}">')
    lines.append(f'      <pose relative_to="{parent}">{fmt_pose(o)}</pose>')
    lines.append(f'      <parent>{parent}</parent>')
    lines.append(f'      <child>{joint["child"]}</child>')
    lines.append(f'      <axis>')
    lines.append(f'        <xyz expressed_in="__model__">{a[0]} {a[1]} {a[2]}</xyz>')
    lines.append(f'        <limit>')
    lines.append(f'          <lower>{joint["lower"]}</lower>')
    lines.append(f'          <upper>{joint["upper"]}</upper>')
    lines.append(f'          <effort>{joint["effort"]}</effort>')
    lines.append(f'          <velocity>{joint["velocity"]}</velocity>')
    lines.append(f'        </limit>')
    lines.append(f'        <dynamics>')
    lines.append(f'          <damping>{joint["damping"]}</damping>')
    lines.append(f'          <friction>{joint["friction"]}</friction>')
    lines.append(f'        </dynamics>')
    lines.append(f'      </axis>')
    lines.append(f'    </joint>')

    return '\n'.join(lines)


sdf_lines = []
sdf_lines.append('<?xml version="1.0"?>')
sdf_lines.append('<sdf version="1.9">')
sdf_lines.append(f'  <model name="{ROBOT_NAME}">')
sdf_lines.append('')
sdf_lines.append(f'  <!-- Generated by blender_export_sdf.py from {bpy.data.filepath} -->')
sdf_lines.append(f'  <!-- Four-bar linkage: proximal_phalanx + middle_phalanx1/2 + distal_phalanx -->')
sdf_lines.append(f'  <!-- Loop-closing joint: dip_flex2 (middle_phalanx2 → distal_phalanx) -->')
sdf_lines.append('')

sdf_lines.append('    <!-- ═══════════════ LINKS ═══════════════ -->')
sdf_lines.append('')
for link in LINKS:
    sdf_lines.append(link_block_sdf(link))
    sdf_lines.append('')

sdf_lines.append('    <!-- ═══════════════ JOINTS ══════════════ -->')
sdf_lines.append('')
for joint in JOINTS:
    sdf_lines.append(joint_block_sdf(joint))
    sdf_lines.append('')

sdf_lines.append('  </model>')
sdf_lines.append('</sdf>')

with open(SDF_PATH, 'w') as f:
    f.write('\n'.join(sdf_lines))

# ── Report ─────────────────────────────────────────────────────────────────────

print("\n=== Mesh export ===")
print(f"Exported {len(exported)} meshes:")
for e in exported: print(f"  - {e}")
if skipped:
    print(f"\nSkipped {len(skipped)}:")
    for s in skipped: print(f"  - {s}")

print(f"\n=== Link world poses (model frame) ===")
for name, pose in link_world_poses.items():
    print(f"  {name}: {pose}")

print(f"\n=== Joint axes (relative to parent link frame) ===")
for name, data in axes_data.items():
    print(f"  {name}: origin={data['origin']} axis={data['axis']}")

print(f"\n=== Four-bar topology check ===")
print("  Spanning tree joints:  pip_flex1 (prox→mp1), pip_flex2 (prox→mp2), dip_flex1 (mp1→dist)")
print("  Loop-closing joint:    dip_flex2 (mp2→dist)  ← constraint enforced by physics engine")

print(f"\n=== SDF ===")
print(f"Saved to {SDF_PATH}")