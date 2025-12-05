---
title: URDF Robot Modeling
sidebar_position: 4
description: Learn how URDF describes humanoid robot structure, kinematics, and physics for simulation and control
---

# URDF Robot Modeling: Describing Humanoid Structure

## Introduction

You've learned how ROS 2 nodes communicate via topics, services, and actions. But how does ROS 2 know what your robot **is**? How does it understand that your humanoid has two arms, two legs, a torso, and a head—and how they're all connected?

The answer is **URDF (Unified Robot Description Format)**—an XML-based language for describing a robot's:

- **Physical structure** (what parts exist)
- **Kinematics** (how parts move relative to each other)
- **Dynamics** (mass, inertia, friction)
- **Visual appearance** (3D models for rendering)
- **Collision geometry** (shapes for physics simulation)

URDF is the "blueprint" that tells simulators, visualizers, and control systems how your robot is built.

## Why URDF Matters for Humanoid Robots

A humanoid robot is a complex **kinematic chain**—interconnected rigid bodies (links) connected by movable joints. URDF provides a standardized way to describe this structure so that:

1. **Simulators** (Gazebo, Isaac Sim) can accurately model physics
2. **Visualizers** (RViz) can display the robot's current pose
3. **Motion planners** can compute valid joint trajectories
4. **Controllers** know which actuators to command

The same URDF file can be used across simulation, visualization, and real hardware—ensuring consistency.

## URDF Core Concepts

### 1. Links: Rigid Body Parts

A **link** represents a rigid body part of the robot that doesn't deform. For a humanoid:

- `base_link` (torso/pelvis)
- `head`
- `left_upper_arm`, `left_forearm`, `left_hand`
- `right_upper_arm`, `right_forearm`, `right_hand`
- `left_thigh`, `left_shin`, `left_foot`
- `right_thigh`, `right_shin`, `right_foot`

Each link can have:
- **Visual representation**: 3D mesh or primitive shape (box, cylinder, sphere)
- **Collision geometry**: Simplified shape for physics calculations
- **Inertial properties**: Mass, center of mass, inertia tensor

### 2. Joints: Connections Between Links

A **joint** defines how two links connect and move relative to each other. Joint types include:

| Joint Type | Degrees of Freedom | Example |
|------------|-------------------|---------|
| **revolute** | 1 (rotation, limited range) | Elbow, knee |
| **continuous** | 1 (rotation, unlimited) | Wheel axle |
| **prismatic** | 1 (translation) | Telescoping antenna |
| **fixed** | 0 (rigid connection) | Head to camera mount |
| **floating** | 6 (full 3D pose) | Mobile base |
| **planar** | 3 (2D translation + rotation) | Sliding door |

For humanoid robots, most joints are **revolute** (shoulders, elbows, hips, knees, ankles).

### 3. Parent-Child Relationships

URDF describes robots as a **tree structure**:

```
base_link (root)
  ├── head (via neck joint)
  ├── left_upper_arm (via left_shoulder joint)
  │     └── left_forearm (via left_elbow joint)
  │           └── left_hand (via left_wrist joint)
  ├── right_upper_arm (via right_shoulder joint)
  │     └── right_forearm (via right_elbow joint)
  │           └── right_hand (via right_wrist joint)
  ├── left_thigh (via left_hip joint)
  │     └── left_shin (via left_knee joint)
  │           └── left_foot (via left_ankle joint)
  └── right_thigh (via right_hip joint)
        └── right_shin (via right_knee joint)
              └── right_foot (via right_ankle joint)
```

Each joint connects a **parent link** to a **child link**, forming a kinematic tree.

## URDF XML Structure

Let's examine a simplified humanoid arm in URDF:

### Example: Humanoid Left Arm

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <!-- Base link (torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>  <!-- 30cm x 20cm x 50cm torso -->
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.3 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>  <!-- 10 kg torso -->
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Upper arm link -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>  <!-- 30cm long, 4cm radius -->
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>  <!-- Cylinder centered at joint -->
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>  <!-- 2 kg arm -->
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0"
               iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Shoulder joint (connects torso to upper arm) -->
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.0 0.2" rpy="0 0 0"/>  <!-- Offset from torso center -->
    <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="2.0"/>
    <!-- Limits: -90° to +90°, max torque 50 Nm, max speed 2 rad/s -->
  </joint>

  <!-- Forearm link -->
  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>  <!-- 25cm forearm -->
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>  <!-- 1 kg forearm -->
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Elbow joint (connects upper arm to forearm) -->
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>  <!-- End of upper arm -->
    <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
    <limit lower="0.0" upper="2.35" effort="30.0" velocity="2.0"/>
    <!-- Limits: 0° to 135°, max torque 30 Nm -->
  </joint>

</robot>
```

### Breaking Down the URDF

#### Link Definition
```xml
<link name="left_upper_arm">
  <visual>        <!-- How it looks (rendering) -->
  <collision>     <!-- Simplified shape for physics collisions -->
  <inertial>      <!-- Mass and inertia for dynamics -->
</link>
```

- **Visual**: What you see in RViz or simulators (can be detailed 3D mesh)
- **Collision**: Simplified geometry for fast collision detection
- **Inertial**: Physical properties for realistic simulation

#### Joint Definition
```xml
<joint name="left_shoulder" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0.0 0.2" rpy="0 0 0"/>  <!-- Position offset -->
  <axis xyz="0 1 0"/>                        <!-- Rotation axis -->
  <limit lower="-1.57" upper="1.57" .../>    <!-- Joint limits -->
</joint>
```

- **Parent/Child**: Defines the kinematic tree
- **Origin**: Where the child link attaches to the parent (translation and rotation)
- **Axis**: Which direction the joint rotates/translates
- **Limit**: Safety constraints (angle range, max torque, max velocity)

## URDF in Action: Visualization and Simulation

### Visualization in RViz

RViz uses URDF's **visual** elements to display the robot:

```bash
# Load URDF and visualize
ros2 launch robot_state_publisher view_robot.launch.py
```

You can interactively move joints using a GUI, and RViz updates the 3D model in real-time by reading joint states from the `/joint_states` topic.

### Simulation in Gazebo

Gazebo uses URDF's **collision**, **inertial**, and **visual** elements to:

- Render the robot in 3D
- Simulate physics (gravity, collisions, friction)
- Compute realistic dynamics based on mass and inertia

When you command a joint to move, Gazebo calculates the resulting motion considering:
- Joint limits
- Torque constraints
- Gravity effects
- Collisions with environment

## Advanced URDF Features

### 1. Using Meshes for Realistic Appearance

Instead of primitive shapes, you can reference 3D mesh files (STL, DAE, OBJ):

```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/left_arm.stl" scale="1.0 1.0 1.0"/>
  </geometry>
</visual>
```

This allows photorealistic rendering while keeping collision geometry simple:

```xml
<collision>
  <geometry>
    <cylinder radius="0.04" length="0.3"/>  <!-- Simplified for performance -->
  </geometry>
</collision>
```

### 2. Transmissions for Actuators

**Transmissions** map joints to actuators (motors):

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

This tells the control system how to command the physical motor.

### 3. Gazebo-Specific Extensions

URDF can include Gazebo-specific tags for sensors and plugins:

```xml
<gazebo reference="head">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
      </image>
    </camera>
  </sensor>
</gazebo>
```

This adds a camera sensor to the head link in simulation.

## Xacro: Parameterized URDF

For complex robots, writing raw URDF XML is tedious and error-prone. **Xacro** (XML Macros) lets you:

- Define constants and variables
- Create reusable macros (e.g., "leg" macro used for both legs)
- Use mathematical expressions

Example Xacro snippet:

```xml
<xacro:property name="arm_length" value="0.3"/>
<xacro:property name="arm_radius" value="0.04"/>

<xacro:macro name="arm" params="prefix">
  <link name="${prefix}_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="${arm_radius}" length="${arm_length}"/>
      </geometry>
    </visual>
  </link>
</xacro:macro>

<!-- Use the macro for both arms -->
<xacro:arm prefix="left"/>
<xacro:arm prefix="right"/>
```

Xacro files (`.urdf.xacro`) are processed into standard URDF at build time.

## Best Practices for Humanoid URDF

### 1. Start Simple, Iterate
Begin with primitive shapes (boxes, cylinders), validate kinematics, then add detailed meshes.

### 2. Accurate Inertial Properties
Incorrect mass or inertia causes unrealistic simulation behavior (falling over, unstable walking). Measure or estimate carefully.

### 3. Consistent Coordinate Frames
Follow ROS conventions:
- X: forward
- Y: left
- Z: up

### 4. Test in Simulation First
Validate URDF in Gazebo before deploying to hardware. Check for:
- Joint limits preventing self-collision
- Realistic mass distribution (center of mass)
- Proper coordinate frame alignment

### 5. Use Meaningful Names
```xml
<!-- Good -->
<link name="left_shin"/>
<joint name="left_knee"/>

<!-- Avoid -->
<link name="link_7"/>
<joint name="joint_3"/>
```

## URDF and ROS 2 Integration

URDF files are published to the `/robot_description` topic by the `robot_state_publisher` node:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf.xacro)"
```

Other nodes (visualizers, planners, controllers) subscribe to this topic to learn the robot's structure.

The `joint_state_publisher` node sends current joint positions to the `/joint_states` topic, and `robot_state_publisher` computes forward kinematics to determine where each link is in 3D space.

## Summary

**URDF** is the standardized language for describing robot structure in ROS 2:

- **Links**: Rigid body parts (arms, legs, torso, head)
- **Joints**: Connections enabling relative motion (revolute, prismatic, fixed)
- **Kinematics**: Parent-child tree structure defining how parts connect
- **Dynamics**: Mass, inertia, friction for realistic physics simulation
- **Visualization**: 3D meshes or primitive shapes for rendering
- **Collision**: Simplified geometry for efficient collision detection

For humanoid robots, URDF enables:
- Accurate simulation in Gazebo and Isaac Sim
- Real-time visualization in RViz
- Motion planning with collision avoidance
- Consistency between simulation and real hardware

The same URDF file flows through your entire development pipeline—from simulation to visualization to real robot control.

---

**Next Steps**: You now understand ROS 2's core concepts (nodes, topics, services, actions) and how URDF describes robot structure. Next, explore [Module 2: Simulation and Digital Twins](../module-2-simulation/index.md) to see these concepts in action!

## References

Open Robotics. (2024). *ROS 2 Documentation: URDF*. https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html

Open Robotics. (2024). *URDF Specification*. http://wiki.ros.org/urdf/XML

Open Robotics. (2024). *Using Xacro to Clean Up Your URDF*. https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html
