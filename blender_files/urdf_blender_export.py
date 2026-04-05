import bpy
import os
import mathutils

#
# The following uses the powerhouse finger as an example, change the joints
# and links to match your robot.
#

# ── Config ────────────────────────────────────────────────────────────────────

MESHES_DIR    = "/home/michael-jenz/rds_ws/finger_vizualization/src/drake-finger-sim/finger_description/meshes"
URDF_PATH     = "/home/michael-jenz/rds_ws/finger_vizualization/src/drake-finger-sim/finger_description/urdf/finger.urdf"
VISUAL_DIR    = os.path.join(MESHES_DIR, "visual")
COLLISION_DIR = os.path.join(MESHES_DIR, "collision")
AXES_FILE     = os.path.join(MESHES_DIR, "joint_axes.yaml")
PACKAGE_NAME  = "finger_description"
ROBOT_NAME    = "finger"

os.makedirs(VISUAL_DIR, exist_ok=True)
os.makedirs(COLLISION_DIR, exist_ok=True)
os.makedirs(os.path.dirname(URDF_PATH), exist_ok=True)

# ── Kinematic tree ────────────────────────────────────────────────────────────
# Define the URDF structure
# Each entry: joint_name, joint_type, parent_link, child_link, limits, dynamics
JOINTS = [
    {
        "name":     "mcp_splay",
        "type":     "revolute",
        "parent":   "base_link",
        "child":    "mcp_link",
        "lower":    -0.174533,
        "upper":    0.174533,
        "effort":   5.0,
        "velocity": 3.14,
        "damping":  0.1,
        "friction": 0.0,
        "empty":    "mcp_splay",   # name of the ARROWS empty in Blender
    },
    {
        "name":     "mcp_flexion",
        "type":     "revolute",
        "parent":   "mcp_link",
        "child":    "proximal_phalanx",
        "lower":    0.0,
        "upper":    1.570,
        "effort":   5.0,
        "velocity": 3.14,
        "damping":  0.1,
        "friction": 0.0,
        "empty":    "mcp_flex",
    },
    {
        "name":     "pip_flexion",
        "type":     "revolute",
        "parent":   "proximal_phalanx",
        "child":    "middle_phalanx",
        "lower":    0.0,
        "upper":    1.570,
        "effort":   5.0,
        "velocity": 3.14,
        "damping":  0.1,
        "friction": 0.0,
        "empty":    "pip_flex",
    },
    {
        "name":     "dip_flexion",
        "type":     "revolute",
        "parent":   "middle_phalanx",
        "child":    "distal_phalanx",
        "lower":    0.0,
        "upper":    1.570,
        "effort":   3.0,
        "velocity": 3.14,
        "damping":  0.1,
        "friction": 0.0,
        "empty":    "dip_flex",
        "mimic":    {"joint": "pip_flexion", "multiplier": "0.85", "offset": 0.0},
    },
]

# Links in order. mesh=True means use STL, mesh=False means primitive (virtual link)
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
        "name":    "middle_phalanx",
        "mesh":    True,
        "mass":    0.008,
        "inertia": (5e-6, 5e-6, 5e-6),
        "comment": "Middle phalanx",
    },
    {
        "name":    "distal_phalanx",
        "mesh":    True,
        "mass":    0.005,
        "inertia": (2e-6, 2e-6, 2e-6),
        "comment": "Distal phalanx",
    },
]

# ── Helpers ───────────────────────────────────────────────────────────────────

def snap(v):
    if abs(v) < 0.001:     return 0.0
    if abs(v - 1) < 0.001: return 1.0
    if abs(v + 1) < 0.001: return -1.0
    return round(v, 4)

def get_empty_data(name):
    """Return (origin_xyz, axis_xyz) from a named Empty in the joint_axes collection."""
    obj = bpy.data.objects.get(name)
    if obj is None or obj.type != 'EMPTY':
        print(f"  WARNING: Empty '{name}' not found, using zeros")
        return (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    loc  = obj.matrix_world.translation
    axis = obj.matrix_world.to_3x3().col[2].normalized()   # Get forward axis on empty obj
    origin = tuple(round(c, 4) for c in loc)
    axis   = tuple(snap(c) for c in axis)
    return origin, axis

def fmt_inertia(ixx, iyy, izz):
    def f(v): return f"{v:g}"
    return f'ixx="{f(ixx)}" ixy="0" ixz="0" iyy="{f(iyy)}" iyz="0" izz="{f(izz)}"'

# ── Mesh export ───────────────────────────────────────────────────────────────

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

    # Temporarily zero the world transform so vertices export in the link's local frame
    # For some reason this is required by URDF? Couldn't think of a better way to do this
    # saved_matrix = obj.matrix_world.copy()
    # obj.matrix_world = mathutils.Matrix.Identity(4)

    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Duplicate cleanly
    bpy.ops.object.duplicate()
    dup = bpy.context.active_object

    # Apply scale on duplicate to fix normals
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    # Zero the transform
    dup.matrix_world = mathutils.Matrix.Identity(4)

    # Export the duplicate
    filepath = os.path.join(out_dir, f"{obj.name}.obj")  # use original obj.name before it goes stale
    dup.select_set(True)
    bpy.context.view_layer.objects.active = dup
    bpy.ops.wm.obj_export(
        filepath=filepath,
        export_selected_objects=True,
        global_scale=1.0,
        export_materials=False,
        forward_axis='Y',
        up_axis='Z',
        export_smooth_groups=False,      # add this
        export_normals=True,             # add this - explicitly include normals
        export_triangulated_mesh=True,
    )

    # Delete the duplicate
    bpy.data.objects.remove(dup, do_unlink=True)

    # Restore original (no longer need to restore matrix since we didn't touch it)
    exported.append(f"{obj.name} → {out_dir}")

# ── Joint axes YAML ───────────────────────────────────────────────────────────

# Build a lookup of joint name -> parent joint name for relative transform calculation
joint_parent_map = {}
link_to_parent_joint = {}
for j in JOINTS:
    link_to_parent_joint[j["child"]] = j["name"]

for j in JOINTS:
    parent_link = j["parent"]
    if parent_link in link_to_parent_joint:
        joint_parent_map[j["name"]] = link_to_parent_joint[parent_link]
    else:
        joint_parent_map[j["name"]] = None  # root joint, already relative to base

# Build axes_data with relative origins
axes_data = {}

# First pass: get all world positions of joints
world_data = {}
for j in JOINTS:
    origin, axis = get_empty_data(j["empty"])
    world_data[j["name"]] = {"origin": origin, "axis": axis}

# Second pass: subtract parent joint world position
for j in JOINTS:
    name = j["name"]
    w_origin = world_data[name]["origin"]
    axis = world_data[name]["axis"]

    parent_joint = joint_parent_map[name]
    if parent_joint is not None:
        p_origin = world_data[parent_joint]["origin"]
        rel_origin = tuple(round(w_origin[i] - p_origin[i], 4) for i in range(3))
    else:
        rel_origin = w_origin  # Root joint origin is already in base_link frame

    axes_data[name] = {"origin": rel_origin, "axis": axis}

with open(AXES_FILE, 'w') as f:
    f.write("# Joint axes extracted from Blender\n")
    f.write("# origin: world-space position of the joint (meters)\n")
    f.write("# axis:   world-space rotation axis (unit vector)\n")
    f.write("# Generated by blender_export.py -- do not edit manually\n\n")
    f.write("joints:\n")
    for name, data in axes_data.items():
        o, a = data["origin"], data["axis"]
        f.write(f"  {name}:\n")
        f.write(f"    origin:\n")
        f.write(f"      x: {o[0]}\n")
        f.write(f"      y: {o[1]}\n")
        f.write(f"      z: {o[2]}\n")
        f.write(f"    axis:\n")
        f.write(f"      x: {a[0]}\n")
        f.write(f"      y: {a[1]}\n")
        f.write(f"      z: {a[2]}\n")
        f.write(f"    urdf_snippets:\n")
        f.write(f'      origin: \'<origin xyz="{o[0]} {o[1]} {o[2]}" rpy="0 0 0"/>\'\n')
        f.write(f'      axis:   \'<axis xyz="{a[0]} {a[1]} {a[2]}"/>\'\n')

# ── URDF generation ───────────────────────────────────────────────────────────

def link_block(link):
    name    = link["name"]
    mass    = link["mass"]
    ixx, iyy, izz = link["inertia"]
    comment = link["comment"]
    pkg     = PACKAGE_NAME
    lines   = []

    lines.append(f'  <!-- {comment} -->')
    lines.append(f'  <link name="{name}">')

    if link["mesh"]:
        lines.append(f'    <visual>')
        lines.append(f'      <geometry>')
        lines.append(f'        <mesh filename="package://{pkg}/meshes/visual/{name}.obj"/>')
        lines.append(f'      </geometry>')
        lines.append(f'    </visual>')
        lines.append(f'    <collision>')
        lines.append(f'      <geometry>')
        lines.append(f'        <mesh filename="package://{pkg}/meshes/collision/col_{name}.obj"/>')
        lines.append(f'      </geometry>')
        lines.append(f'    </collision>')
    # virtual links (mcp_link etc.) have no visual/collision

    lines.append(f'    <inertial>')
    lines.append(f'      <mass value="{mass}"/>')
    lines.append(f'      <origin xyz="0 0 0" rpy="0 0 0"/>')
    lines.append(f'      <inertia {fmt_inertia(ixx, iyy, izz)}/>')
    lines.append(f'    </inertial>')
    lines.append(f'  </link>')

    return '\n'.join(lines)

def joint_block(joint):
    name   = joint["name"]
    o, a   = axes_data[name]["origin"], axes_data[name]["axis"]
    lines  = []

    lines.append(f'  <!-- {name.replace("_", " ").title()} -->')
    lines.append(f'  <joint name="{name}" type="{joint["type"]}">')
    lines.append(f'    <parent link="{joint["parent"]}"/>')
    lines.append(f'    <child link="{joint["child"]}"/>')
    lines.append(f'    <origin xyz="{o[0]} {o[1]} {o[2]}" rpy="0 0 0"/>')
    lines.append(f'    <axis xyz="{a[0]} {a[1]} {a[2]}"/>')
    lines.append(f'    <limit lower="{joint["lower"]}" upper="{joint["upper"]}" '
                 f'effort="{joint["effort"]}" velocity="{joint["velocity"]}"/>')
    lines.append(f'    <dynamics damping="{joint["damping"]}" friction="{joint["friction"]}"/>')

    if "mimic" in joint:
        m = joint["mimic"]
        lines.append(f'    <mimic joint="{m["joint"]}" '
                     f'multiplier="{m["multiplier"]}" offset="{m["offset"]}"/>')

    lines.append(f'  </joint>')
    return '\n'.join(lines)

# Build link lookup for ordering: emit link, then its outgoing joint
link_to_joint = {j["parent"]: j for j in JOINTS}

urdf_lines = []
urdf_lines.append('<?xml version="1.0"?>')
urdf_lines.append(f'<robot name="{ROBOT_NAME}">')
urdf_lines.append('')
urdf_lines.append(f'  <!-- Generated by urdf_blender_export.py from Blender scene {bpy.data.filepath} -->')
urdf_lines.append('')

# Materials
materials = [
    ("blue",   "0 0 1 1"),
    ("green",  "0 1 0 1"),
    ("red",    "1 0 0 1"),
    ("purple", "1 0 1 1"),
    ("grey",   "0.5 0.5 0.5 1"),
    ("yellow", "1 1 0 1"),
]
for mat_name, rgba in materials:
    urdf_lines.append(f'  <material name="{mat_name}">')
    urdf_lines.append(f'    <color rgba="{rgba}"/>')
    urdf_lines.append(f'  </material>')
urdf_lines.append('')

# Interleave links and joints in kinematic order
for link in LINKS:
    urdf_lines.append(link_block(link))
    urdf_lines.append('')
    if link["name"] in link_to_joint:
        urdf_lines.append(joint_block(link_to_joint[link["name"]]))
        urdf_lines.append('')

urdf_lines.append('</robot>')

with open(URDF_PATH, 'w') as f:
    f.write('\n'.join(urdf_lines))

# ── Report ────────────────────────────────────────────────────────────────────

print("\n=== Mesh export ===")
print(f"Exported {len(exported)} meshes:")
for e in exported: print(f"  - {e}")
if skipped:
    print(f"\nSkipped {len(skipped)}:")
    for s in skipped: print(f"  - {s}")

print(f"\n=== Joint axes ===")
print(f"Saved to {AXES_FILE}")
for name, data in axes_data.items():
    print(f"  - {name}: origin={data['origin']} axis={data['axis']}")

print(f"\n=== URDF ===")
print(f"Saved to {URDF_PATH}")
