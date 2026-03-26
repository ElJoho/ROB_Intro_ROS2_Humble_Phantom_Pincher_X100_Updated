# Adding a Suction Cup Link to the End Effector

This document explains how to add a fixed suction cup link to the end effector of the PhantomX Pincher arm, including collision geometry and updating the MoveIt IK target (TCP) to the tip of the suction cup.

## Overview

Three files must be modified:

| File | Purpose |
|------|---------|
| `phantomx_pincher_description/urdf/phantomx_pincher_arm.xacro` | Add the new link and joints to the URDF |
| `phantomx_pincher_moveit_config/srdf/phantomx_pincher.xacro` | Update the IK chain tip in the SRDF template |
| `phantomx_pincher_moveit_config/srdf/phantomx_pincher.srdf` | Mirror the same changes with hardcoded link names |

## Mesh files

- **Visual**: `phantomx_pincher_description/meshes/DAE/acopleSuctionCupGripper.dae`
- **Collision**: `phantomx_pincher_description/meshes/STL/acopleSuctionCupGripper.stl`

---

## File 1: `phantomx_pincher_arm.xacro`

Locate the current `end_effector` virtual link block (around line 362) and add the following **after** it.

### A) Suction cup link with visual and collision geometry

```xml
<!-- Suction cup adapter mounted on end effector -->
<link name="${prefix}suction_cup_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://phantomx_pincher_description/meshes/DAE/acopleSuctionCupGripper.dae"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://phantomx_pincher_description/meshes/STL/acopleSuctionCupGripper.stl"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="${prefix}end_effector_to_suction_cup_joint" type="fixed">
  <parent link="${prefix}end_effector"/>
  <child link="${prefix}suction_cup_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

> **Note on inertia**: The values are approximate placeholders. Update `mass` and the inertia tensor once you have the real physical properties of the suction cup.

> **Note on offset**: The joint origin is currently `0 0 0`. Once you know the physical offset from the end effector frame to the suction cup base, update the `xyz` value accordingly.

### B) Virtual TCP link at the suction cup tip

This is an empty link (no geometry) that marks the tip of the suction cup. MoveIt will use it as the IK target. The `phantomx_pincher_virtual_link` macro is already defined in `phantomx_pincher_utils.xacro`.

```xml
<!-- Virtual TCP frame at the tip of the suction cup -->
<xacro:phantomx_pincher_virtual_link
    parent="${prefix}suction_cup_link"
    link_name="${prefix}suction_cup_tip"
    joint_origin_xyz="0.0 0.0 0.0"
    joint_origin_rpy="0.0 0.0 0.0"
    gazebo_preserve_fixed_joint="${gazebo_preserve_fixed_joint}"
    />
```

> **Note on tip offset**: The `joint_origin_xyz` here is `0 0 0` as a placeholder. Once you measure the physical length of the suction cup, set the `z` value to the distance from the suction cup base to its tip so the IK target lands exactly at the contact point.

---

## File 2: `phantomx_pincher.xacro` (SRDF template)

### Change the IK chain tip link

Find line 11 and update `tip_link`:

```xml
<!-- Before -->
<chain base_link="${prefix}arm_base_link" tip_link="${prefix}end_effector" />

<!-- After -->
<chain base_link="${prefix}arm_base_link" tip_link="${prefix}suction_cup_tip" />
```

### Add collision disable rules

Add the following entries near the existing `disable_collisions` blocks. These prevent MoveIt from raising false collision warnings between the suction cup and the links it is always in contact with.

```xml
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}gripper_finger_base_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}gripper_finger1_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}gripper_finger2_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}gripper_servo_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}arm_wrist_F3_0_link" reason="Never"/>
<disable_collisions link1="${prefix}suction_cup_link" link2="${prefix}arm_wrist_flex_link" reason="Never"/>
```

---

## File 3: `phantomx_pincher.srdf` (deployed SRDF)

This file mirrors the xacro but uses hardcoded link names (prefix = `phantomx_pincher_`). Apply the same two changes.

### Change the IK chain tip link

Find line 9 and update `tip_link`:

```xml
<!-- Before -->
<chain base_link="phantomx_pincher_arm_base_link" tip_link="phantomx_pincher_end_effector"/>

<!-- After -->
<chain base_link="phantomx_pincher_arm_base_link" tip_link="phantomx_pincher_suction_cup_tip"/>
```

### Add collision disable rules

```xml
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_gripper_finger_base_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_gripper_finger1_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_gripper_finger2_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_gripper_servo_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_arm_wrist_F3_0_link" reason="Never"/>
<disable_collisions link1="phantomx_pincher_suction_cup_link" link2="phantomx_pincher_arm_wrist_flex_link" reason="Never"/>
```

---

## Resulting kinematic chain

```
... → arm_wrist_flex_link
    → arm_wrist_F3_0_link
    → gripper_servo_link
    → gripper_finger_base_link
    → end_effector          (old TCP — kept as reference frame)
    → suction_cup_link      (mesh with visual and collision geometry)
    → suction_cup_tip       (new TCP used by MoveIt for IK planning)
```

---

## Pending tasks before finalizing

- [ ] Measure the physical offset from the `end_effector` frame to the base of the suction cup adapter and update the `xyz` in `end_effector_to_suction_cup_joint`.
- [ ] Measure the length of the suction cup and update `joint_origin_xyz` in the `suction_cup_tip` virtual link joint to place the TCP at the actual contact point.
- [ ] Verify the mesh origin in a 3D viewer (Blender or MeshLab) to confirm the `.dae` and `.stl` files are aligned with the expected frame before running.
