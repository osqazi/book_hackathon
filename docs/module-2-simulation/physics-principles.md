---
title: Physics Principles
sidebar_position: 2
description: Understand gravity, inertia, friction, and collisions in humanoid robot simulation
---

# Physics Principles for Humanoid Simulation

## Introduction

Simulating a humanoid robot isn't like simulating a wheeled robot or a stationary arm. Humanoids must maintain balance on two legs, manage dynamic movements without tipping over, and interact with environments through contact. These challenges require understanding the physics that govern bipedal locomotion and manipulation.

This section explores the four critical physics principles that make humanoid simulation unique:

1. **Gravity**: The constant force pulling the robot down
2. **Inertia**: Resistance to changes in motion based on mass distribution
3. **Friction**: Forces at contact points (feet, hands, objects)
4. **Collisions**: Detecting and responding to impacts

Understanding these principles helps you:
- Design stable walking gaits
- Predict robot behavior in simulation
- Debug unexpected simulation results
- Bridge the gap between simulation and reality

## Gravity: The Relentless Challenge

### What Gravity Does

Gravity exerts a constant downward force on every link of the robot:

```
F_gravity = m * g
```

Where:
- `m` = mass of the link (kg)
- `g` = gravitational acceleration (9.81 m/s² on Earth)

For a humanoid with total mass 50 kg:

```
F_gravity = 50 kg * 9.81 m/s² = 490.5 Newtons downward
```

This force never stops—even when the robot is stationary, it must continuously resist gravity to remain upright.

### Center of Mass (CoM) and Balance

A robot balances when its **Center of Mass (CoM)** projection falls within its **support polygon** (the area bounded by contact points with the ground).

For bipedal humanoids:

**Standing on Both Feet**:
```
Support Polygon = area between both feet
CoM must project inside this rectangle
```

**Walking (Single Support Phase)**:
```
Support Polygon = area of one foot
CoM must project inside this smaller area (harder to balance!)
```

**In Simulation**:
- Gazebo computes CoM from link masses and positions
- If CoM projection leaves the support polygon → robot tips over
- Controller must shift CoM to maintain balance

### Why Humanoids Are Harder to Balance Than Wheeled Robots

| Robot Type | Support Polygon | Stability |
|------------|-----------------|-----------|
| **Wheeled** | Large rectangle (all wheels) | Very stable |
| **Quadruped** | Large polygon (4 legs) | Stable |
| **Bipedal** | Small rectangle (2 feet) | Unstable—requires active control! |

Humanoid balance requires:
- Continuous monitoring of CoM position
- Active joint adjustments to shift weight
- Predictive control (move CoM before stepping)

### Gravity in Gazebo Configuration

Gazebo lets you adjust gravity for testing:

```xml
<world name="default">
  <physics type="ode">
    <gravity>0 0 -9.81</gravity>  <!-- Standard Earth gravity -->
  </physics>
</world>
```

Useful variations:
- **Moon gravity**: `0 0 -1.62` (easier balancing, longer hang time)
- **Zero gravity**: `0 0 0` (test manipulation without balance constraints)
- **Increased gravity**: `0 0 -15` (stress-test stability algorithms)

## Inertia: Resistance to Motion Changes

### What Is Inertia?

**Inertia** is resistance to changes in motion. It depends on:

1. **Mass**: Heavier objects resist acceleration more
2. **Mass distribution**: How mass is distributed around the center of rotation

For rotational motion, we use the **inertia tensor**:

```xml
<inertial>
  <mass value="2.0"/>  <!-- kg -->
  <inertia ixx="0.05" ixy="0.0" ixz="0.0"
           iyy="0.05" iyz="0.0" izz="0.01"/>
</inertial>
```

- **ixx, iyy, izz**: Moments of inertia around X, Y, Z axes
- **ixy, ixz, iyz**: Products of inertia (usually zero for symmetric objects)

### Why Inertia Matters for Humanoids

#### 1. Arm Swinging

When a humanoid swings its arm:

```
Torque_required = I * angular_acceleration
```

- **Large I (heavy, extended arm)**: More torque needed, slower motion
- **Small I (light, compact arm)**: Less torque needed, faster motion

**Simulation Impact**: If you specify incorrect inertia in URDF, the simulated arm will move too fast/slow compared to reality.

#### 2. Dynamic Walking

During walking, the robot's **angular momentum** changes as legs swing:

```
L = I * ω  (angular momentum = inertia * angular velocity)
```

The body must counterrotate to conserve momentum. If simulation uses wrong inertia:
- Robot may spin unexpectedly
- Gait becomes unstable
- Real hardware behaves differently

#### 3. Fast Motions

High-speed movements (punching, jumping, quick turns) amplify inertia effects. Small URDF inertia errors cause large behavior differences.

### Computing Accurate Inertia

For simple shapes:

**Solid Cylinder** (radius r, height h, mass m):
```
Ixx = Iyy = (1/12) * m * (3*r² + h²)
Izz = (1/2) * m * r²
```

**Rectangular Box** (width w, depth d, height h, mass m):
```
Ixx = (1/12) * m * (d² + h²)
Iyy = (1/12) * m * (w² + h²)
Izz = (1/12) * m * (w² + d²)
```

For complex shapes, use CAD software (SolidWorks, Fusion 360) to compute inertia tensors automatically.

### Common Inertia Mistakes

❌ **Using placeholder values** (`ixx="1.0" iyy="1.0" izz="1.0"`): Causes unrealistic dynamics

❌ **Forgetting to update after geometry changes**: Changing link size without recalculating inertia

❌ **Zero inertia**: `ixx="0"` crashes some physics engines

✅ **Use accurate values**: Measure or compute from CAD models

✅ **Validate in simulation**: Compare joint accelerations to expected values

## Friction: The Contact Challenge

### Types of Friction

Friction resists sliding motion at contact points. For humanoids, this affects:

- **Foot-ground contact**: Walking without slipping
- **Hand-object contact**: Grasping without dropping
- **Joint friction**: Internal resistance in motors (usually modeled separately)

### Coulomb Friction Model

Most simulators use the **Coulomb friction model**:

```
F_friction_max = μ * F_normal
```

Where:
- `μ` = coefficient of friction (material property)
- `F_normal` = normal force (perpendicular to surface)

If applied force < `F_friction_max` → object doesn't slip
If applied force > `F_friction_max` → object slides

### Friction Coefficients (μ)

| Surface Pair | μ (static) | μ (kinetic) |
|--------------|------------|-------------|
| Rubber on concrete | 0.7–1.0 | 0.5–0.8 |
| Metal on metal | 0.15–0.25 | 0.1–0.2 |
| Ice on ice | 0.02–0.05 | 0.01–0.03 |
| Soft rubber on wood | 0.9–1.2 | 0.7–0.9 |

**Static friction** (μ_s): Resistance before motion starts (higher)
**Kinetic friction** (μ_k): Resistance during motion (lower)

### Configuring Friction in Gazebo

In SDF/URDF collision elements:

```xml
<collision name="foot_collision">
  <geometry>
    <box size="0.1 0.05 0.02"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>      <!-- First friction direction -->
        <mu2>1.0</mu2>    <!-- Second friction direction (anisotropic) -->
      </ode>
    </friction>
  </surface>
</collision>
```

### Friction in Humanoid Walking

**Problem**: If foot friction is too low, the robot slips during push-off:

```
Foot pushes backward → Ground should push forward (Newton's 3rd law)
If μ is low → Foot slips instead → Robot doesn't move forward
```

**Solution**: Set foot friction high enough to prevent slipping:

```xml
<mu>1.0</mu>  <!-- High friction for rubber feet -->
```

**Testing**: Gradually decrease friction until robot slips, then add safety margin.

### Friction and Grasping

When grasping objects:

```
Required grip force = (object_weight / μ) / 2  (for two-finger grasp)
```

Example: Grasp 1 kg object (9.81 N weight) with μ = 0.5:

```
Grip force per finger = (9.81 N / 0.5) / 2 = 9.81 N per finger
```

If simulation uses wrong μ, you'll either:
- **Over-grip**: Waste energy, risk crushing delicate objects
- **Under-grip**: Object slips and falls

## Collisions: Detection and Response

### What Are Collisions?

A **collision** occurs when two objects' geometries intersect. Simulators must:

1. **Detect** when collision happens (computationally expensive)
2. **Compute contact points** (where surfaces touch)
3. **Apply contact forces** to prevent penetration
4. **Resolve dynamics** (bounce, slide, or stick together)

### Collision Geometry vs Visual Geometry

URDF separates visual and collision meshes:

```xml
<link name="arm">
  <visual>
    <geometry>
      <mesh filename="arm_detailed.stl"/>  <!-- High-poly for looks -->
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>  <!-- Simple for speed -->
    </geometry>
  </collision>
</link>
```

**Why separate?**
- **Visual**: Detailed mesh for realistic rendering (slow to test collisions)
- **Collision**: Simplified shape for fast physics (primitives like boxes, cylinders, spheres)

### Collision Response Parameters

**Restitution** (bounciness):

```xml
<surface>
  <bounce>
    <restitution_coefficient>0.5</restitution_coefficient>
  </bounce>
</surface>
```

- **e = 0**: Perfectly inelastic (no bounce, like clay)
- **e = 1**: Perfectly elastic (perfect bounce, like superballs)
- **e = 0.3–0.7**: Typical for robot-object interactions

**Contact Stiffness** (how "hard" surfaces are):

```xml
<surface>
  <contact>
    <ode>
      <kp>1e6</kp>  <!-- Stiffness (N/m) -->
      <kd>100</kd>  <!-- Damping (N·s/m) -->
    </ode>
  </contact>
</surface>
```

- Higher `kp`: Stiffer contacts, less penetration
- Higher `kd`: More damping, less oscillation

### Humanoid Collision Scenarios

#### Self-Collision
Humanoids can collide with themselves (e.g., arm hits torso):

```xml
<gazebo>
  <self_collide>true</self_collide>  <!-- Enable self-collision checking -->
</gazebo>
```

Useful for:
- Validating arm trajectories don't hit body
- Testing compact poses (crouching, folding)

Performance cost: High (many link pairs to check)

#### Environment Collision
Robot colliding with obstacles, walls, furniture:

- Use simplified collision meshes for environment
- Set appropriate friction (wall = low, carpet = high)
- Enable contact sensors to detect when collision occurs

#### Contact Forces
When humanoid foot touches ground, simulator computes **contact force**:

```
F_contact = k_p * penetration_depth + k_d * penetration_velocity
```

This force:
- Prevents foot from sinking into floor
- Provides normal force for friction calculations
- Feeds back to controller (some robots have force sensors in feet)

## Physics Engines in Gazebo

Gazebo supports multiple physics engines with different characteristics:

| Engine | Speed | Accuracy | Best For |
|--------|-------|----------|----------|
| **ODE** | Fast | Moderate | Real-time sim, wheeled robots |
| **Bullet** | Fast | Moderate | Grasping, contact-rich tasks |
| **DART** | Medium | High | Humanoid walking, precise dynamics |
| **Simbody** | Slow | Very high | Biomechanics, research |

For humanoid robots, **DART** or **Bullet** are recommended for better contact handling.

## Simulation Timestep and Stability

Physics simulation advances in discrete **timesteps**:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1 ms per step -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

**Timestep tradeoffs**:
- **Smaller (0.0001s)**: More accurate, slower computation, higher stability
- **Larger (0.01s)**: Faster, less accurate, may become unstable

**Instability symptoms**:
- Robot "vibrates" or "explodes"
- Joints oscillate wildly
- Simulation crashes

**Fix**: Reduce timestep, increase contact stiffness damping, check URDF inertias.

## Best Practices for Physics Simulation

### 1. Start with Accurate URDF
- Measure real robot masses
- Compute inertia tensors from CAD
- Validate link lengths match hardware

### 2. Use Appropriate Friction
- Rubber feet: μ = 0.8–1.2
- Metal slides: μ = 0.1–0.2
- Test edge cases (wet floor, polished surfaces)

### 3. Simplify Collision Geometry
- Use primitives (box, cylinder, sphere) when possible
- Keep polygon count low for meshes
- Disable self-collision if not needed

### 4. Tune Physics Parameters
- Start with default timestep (0.001s)
- Increase contact stiffness if penetration occurs
- Add damping if oscillations appear

### 5. Validate Against Reality
- Compare joint torques between sim and hardware
- Measure foot slip in real walking vs simulation
- Tune parameters to match observed behavior

## Summary

Physics simulation for humanoids requires understanding:

- **Gravity**: Constant downward force requiring active balance control (CoM within support polygon)
- **Inertia**: Mass distribution affects rotational dynamics (specify accurate inertia tensors)
- **Friction**: Contact forces enable walking and grasping (μ = 0.8–1.2 for feet)
- **Collisions**: Detection and response with simplified geometry for performance

**Key Takeaways**:
- Humanoids are inherently unstable—simulation reveals balance issues early
- Accurate URDF parameters (mass, inertia, friction) are critical for realistic behavior
- Simplified collision geometry balances performance and accuracy
- Validate simulation against real hardware to tune parameters

Next, we'll explore how to equip your simulated humanoid with virtual sensors for perception.

---

**Continue to**: [Sensor Simulation](./sensors.md)

## References

Open Robotics. (2024). *Gazebo Documentation: Physics Engines*. https://gazebosim.org/api/sim/7/physicsplugin.html

Open Robotics. (2024). *SDF Specification: Surface Properties*. http://sdformat.org/spec?elem=collision&ver=1.9
