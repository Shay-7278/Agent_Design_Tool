# End-Effector Framework

This directory contains xacro macros for all mountable end-effectors.  Each
tool is described by one macro file (`ee_<name>_macro.xacro`) and one entry in
the central registry ([`../../config/tools.yaml`](../../config/tools.yaml)).

---

## Two TCP frames per tool

Every end-effector adds two TF frames to each arm:

| Frame | Name in TF tree | Description |
|---|---|---|
| **vis_tcp** | `${prefix}vis_tcp` | Nominal TCP from CAD geometry (`nominal_tcp` in tools.yaml). Use for simulation and visualisation. |
| **tcp** | `${prefix}tcp` | Calibrated TCP — `vis_tcp` plus a small measured correction (`calibrated_tcp` in tools.yaml). Use for motion planning and workcell_api. |

The frame hierarchy is:

```
${prefix}tool0  (flange — existing robot link)
├── ${prefix}ee_body            (rigid tool body with visual + collision meshes)
│   └── ...joints if any...    (e.g. left_finger / right_finger for gripper)
└── ${prefix}vis_tcp            (nominal TCP)
      └── ${prefix}tcp          (calibrated TCP)
```

`workcell_api.WorkcellClient` always targets `${prefix}tcp` for motion planning.
`${prefix}vis_tcp` is published in TF for reference (you can inspect it in Foxglove).

---

## Calibrated TCP workflow

1. **Before calibration** — both `calibrated_tcp.quat` and `calibrated_tcp.rpy`
   in `tools.yaml` are set to their identity/zero values.  The `tcp` frame
   coincides with `vis_tcp`.

2. **Run calibration** — use the ABB robot's built-in TCP calibration routine.
   The robot outputs a correction as a quaternion **`[w, x, y, z]`**.

3. **Update tools.yaml** — run the helper script with the measured values:

   ```bash
   python3 scripts/update_tcp_calibration.py \
       --tool gripper \
       --xyz  0.001 -0.002 0.003 \
       --quat 0.9999 0.0010 -0.0020 0.0003
   ```

   The script writes both `quat` and the derived `rpy` to `tools.yaml`.
   You can also edit `tools.yaml` manually — just keep `quat` and `rpy` in sync.

4. **Rebuild** — `colcon build --symlink-install` re-processes the xacro
   with the new values.  Restart the launch file to update TF.

> **Quaternion convention:** `[w, x, y, z]` throughout this package.
> `tf_transformations.euler_from_quaternion` uses `[x, y, z, w]` — the launch
> helpers reorder automatically.

---

## How to add a new tool

### 1 · Add an entry to `config/tools.yaml`

Copy the commented-out template at the bottom of the file.  At minimum you
need `nominal_tcp`, `calibrated_tcp`, `meshes`, and `joints: {}`.  Example:

```yaml
mill:
  description: "Spindle milling tool"
  nominal_tcp:
    xyz: [0.0, 0.0, 0.220]
    rpy: [0.0, 0.0, 0.0]
  calibrated_tcp:
    xyz:  [0.0, 0.0, 0.0]
    quat: [1.0, 0.0, 0.0, 0.0]
    rpy:  [0.0, 0.0, 0.0]
  meshes:
    visual:    "meshes/visual/end_effectors/mill.stl"
    collision: "meshes/collision/end_effectors/mill_collision.stl"
  joints: {}
```

### 2 · Create the xacro macro

Create `urdf/end_effectors/ee_mill_macro.xacro`.  Use
`ee_gripper_macro.xacro` as the template if the tool has articulated joints,
or `ee_none_macro.xacro` if it is rigid (just swap the key `'none'` → `'mill'`
in the `load_yaml` lookup and add the `ee_body` link with mesh).

Rigid tool template (copy and adapt):

```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">
  <xacro:macro name="ee_mill" params="prefix">

    <xacro:property name="tools_cfg"
      value="${xacro.load_yaml('$(find abb_irb4600_workcell_description)/config/tools.yaml')}"/>
    <xacro:property name="cfg"    value="${tools_cfg['tools']['mill']}"/>
    <xacro:property name="nom"    value="${cfg['nominal_tcp']}"/>
    <xacro:property name="cal"    value="${cfg['calibrated_tcp']}"/>
    <xacro:property name="meshes" value="${cfg['meshes']}"/>

    <link name="${prefix}ee_body">
      <visual>
        <geometry>
          <mesh filename="package://abb_irb4600_workcell_description/${meshes['visual']}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://abb_irb4600_workcell_description/${meshes['collision']}"/>
        </geometry>
      </collision>
    </link>
    <joint name="${prefix}tool0_to_ee_body" type="fixed">
      <parent link="${prefix}tool0"/>
      <child  link="${prefix}ee_body"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>

    <link name="${prefix}vis_tcp"/>
    <joint name="${prefix}tool0_to_vis_tcp" type="fixed">
      <parent link="${prefix}tool0"/>
      <child  link="${prefix}vis_tcp"/>
      <origin xyz="${nom['xyz'][0]} ${nom['xyz'][1]} ${nom['xyz'][2]}"
              rpy="${nom['rpy'][0]} ${nom['rpy'][1]} ${nom['rpy'][2]}"/>
    </joint>

    <link name="${prefix}tcp"/>
    <joint name="${prefix}vis_tcp_to_tcp" type="fixed">
      <parent link="${prefix}vis_tcp"/>
      <child  link="${prefix}tcp"/>
      <origin xyz="${cal['xyz'][0]} ${cal['xyz'][1]} ${cal['xyz'][2]}"
              rpy="${cal['rpy'][0]} ${cal['rpy'][1]} ${cal['rpy'][2]}"/>
    </joint>

  </xacro:macro>
</robot>
```

### 3 · Register the macro in the workcell macro

In `urdf/abb_irb4600_workcell_macro.xacro`, add:

```xml
<!-- include at top of file, alongside other EE includes -->
<xacro:include filename="$(find abb_irb4600_workcell_description)/urdf/end_effectors/ee_mill_macro.xacro"/>
```

And in the macro body (once per arm):

```xml
<xacro:if value="${tool_ll == 'mill'}"><xacro:ee_mill prefix="LL_"/></xacro:if>
<xacro:if value="${tool_rr == 'mill'}"><xacro:ee_mill prefix="RR_"/></xacro:if>
```

### 4 · Update the SRDF

In `abb_irb4600_workcell_moveit/config/abb_irb4600_workcell.srdf.xacro`, add a
conditional block for each arm.  For a rigid tool (no joints):

```xml
<xacro:if value="${tool_ll_val == 'mill'}">
  <group name="LL_mill"/>
  <end_effector name="LL_mill_ee" parent_link="LL_tool0"
                group="LL_mill" parent_group="LL_with_track"/>
  <disable_collisions link1="LL_link_6" link2="LL_ee_body" reason="Adjacent"/>
</xacro:if>
<xacro:if value="${tool_rr_val == 'mill'}">
  <group name="RR_mill"/>
  <end_effector name="RR_mill_ee" parent_link="RR_tool0"
                group="RR_mill" parent_group="RR_with_track"/>
  <disable_collisions link1="RR_link_6" link2="RR_ee_body" reason="Adjacent"/>
</xacro:if>
```

For tools with articulated joints, also add the joint names to the group and
add `group_state` entries for named positions (see the gripper block for an
example).

### 5 · Add joint limits (articulated tools only)

If the tool has joints, add limits to
`abb_irb4600_workcell_moveit/config/joint_limits.yaml` for every joint of
every arm (LL\_ and RR\_ prefixes):

```yaml
LL_mill_spindle_joint:
  has_position_limits: false
  has_velocity_limits: true
  max_velocity: 10.0
  ...
```

### 6 · Add ros2_control interfaces (articulated tools only)

Add the joints to
`abb_irb4600_workcell_moveit/config/abb_irb4600_workcell.ros2_control.xacro`
inside a conditional block, and a new controller entry to
`ros2_controllers.yaml` and `moveit_controllers.yaml`.

### 7 · Add mesh files

Place your STL files in:

```
meshes/visual/end_effectors/<name>.stl
meshes/collision/end_effectors/<name>_collision.stl
```

Ensure the STL origin coincides with the `tool0` frame (flange face centre).
The collision mesh should be a simplified convex decomposition of the visual
mesh for efficient collision checking.

---

## File naming conventions

| File | Convention |
|---|---|
| Macro file | `ee_<name>_macro.xacro` |
| Visual mesh | `meshes/visual/end_effectors/<name>.stl` |
| Collision mesh | `meshes/collision/end_effectors/<name>_collision.stl` |
| `tools.yaml` key | `<name>` (snake\_case, matches macro name and mesh prefix) |
| Macro xacro name | `ee_<name>` |
| `ToolName` enum value | `<name>` (add to `workcell_api/types.py`) |
