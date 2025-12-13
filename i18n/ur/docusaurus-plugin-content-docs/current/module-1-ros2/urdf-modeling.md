---
title: URDF Robot Modeling
sidebar_position: 4
description: جانیں کہ URDF کیسے سمولیشن اور کنٹرول کے لیے انسان نما روبوٹ کی ڈھانچہ، kinematics، اور طبیعیات کی تفصیل کرتا ہے
---

# URDF Robot Modeling: انسان نما ڈھانچے کی تفصیل

## تعارف

آپ نے سیکھ لیا ہے کہ ROS 2 nodes topics، services، اور actions کے ذریعے کیسے بات چیت کرتے ہیں۔ لیکن ROS 2 کو کیسے معلوم ہوتا ہے کہ آپ کا روبوٹ **کیا ہے**؟ یہ کیسے سمجھتا ہے کہ آپ کے انسان نما کے دو بازو، دو ٹانگیں، ایک دھڑ، اور ایک سر ہے—اور یہ سب کیسے جڑے ہوئے ہیں؟

جواب **URDF (Unified Robot Description Format)** ہے—ایک XML پر مبنی زبان جو روبوٹ کی تفصیل کرتی ہے:

- **Physical structure** (کون سے حصے موجود ہیں)
- **Kinematics** (حصے ایک دوسرے کے مطابق کیسے حرکت کرتے ہیں)
- **Dynamics** (mass، inertia، friction)
- **بصری ظہور** (rendering کے لیے 3D ماڈلز)
- **Collision geometry** (طبیعیات سمولیشن کے لیے shapes)

URDF وہ "نقشہ" ہے جو simulators، visualizers، اور کنٹرول نظاموں کو بتاتا ہے کہ آپ کا روبوٹ کیسے بنایا گیا ہے۔

## انسان نما روبوٹوں کے لیے URDF کیوں اہم ہے

ایک انسان نما روبوٹ ایک پیچیدہ **kinematic chain** ہے—آپس میں جڑے ہوئے rigid bodies (links) جو movable joints سے جڑے ہوتے ہیں۔ URDF اس ڈھانچے کی تفصیل کا معیاری طریقہ فراہم کرتا ہے تاکہ:

1. **Simulators** (Gazebo، Isaac Sim) طبیعیات کو درست طریقے سے model کر سکیں
2. **Visualizers** (RViz) روبوٹ کی موجودہ pose دکھا سکیں
3. **Motion planners** درست joint trajectories compute کر سکیں
4. **Controllers** جانیں کہ کن actuators کو کمانڈ دینی ہے

ایک ہی URDF فائل سمولیشن، visualization، اور حقیقی ہارڈ ویئر میں استعمال ہو سکتی ہے—مطابقت کو یقینی بناتے ہوئے۔

## URDF بنیادی تصورات

### 1. Links: Rigid Body حصے

ایک **link** روبوٹ کا ایک rigid body حصہ represent کرتا ہے جو deform نہیں ہوتا۔ انسان نما کے لیے:

- `base_link` (دھڑ/pelvis)
- `head`
- `left_upper_arm`، `left_forearm`، `left_hand`
- `right_upper_arm`، `right_forearm`، `right_hand`
- `left_thigh`، `left_shin`، `left_foot`
- `right_thigh`، `right_shin`، `right_foot`

ہر link میں ہو سکتا ہے:
- **Visual representation**: 3D mesh یا primitive shape (box، cylinder، sphere)
- **Collision geometry**: طبیعیات calculations کے لیے simplified shape
- **Inertial properties**: Mass، center of mass، inertia tensor

### 2. Joints: Links کے درمیان Connections

ایک **joint** تعریف کرتا ہے کہ دو links کیسے جڑتے اور ایک دوسرے کے مطابق حرکت کرتے ہیں۔ Joint کی اقسام:

| Joint Type | Degrees of Freedom | مثال |
|------------|-------------------|---------|
| **revolute** | 1 (گردش، محدود رینج) | کہنی، گھٹنا |
| **continuous** | 1 (گردش، لامحدود) | پہیے کا axle |
| **prismatic** | 1 (translation) | Telescoping antenna |
| **fixed** | 0 (rigid connection) | سر سے کیمرا mount |
| **floating** | 6 (مکمل 3D pose) | Mobile base |
| **planar** | 3 (2D translation + rotation) | Sliding دروازہ |

انسان نما روبوٹوں کے لیے، زیادہ تر joints **revolute** ہیں (کندھے، کہنیاں، کولہے، گھٹنے، ٹخنے)۔

### 3. Parent-Child تعلقات

URDF روبوٹوں کو **tree structure** کے طور پر بیان کرتا ہے:

```
base_link (root)
  ├── head (بذریعہ neck joint)
  ├── left_upper_arm (بذریعہ left_shoulder joint)
  │     └── left_forearm (بذریعہ left_elbow joint)
  │           └── left_hand (بذریعہ left_wrist joint)
  ├── right_upper_arm (بذریعہ right_shoulder joint)
  │     └── right_forearm (بذریعہ right_elbow joint)
  │           └── right_hand (بذریعہ right_wrist joint)
  ├── left_thigh (بذریعہ left_hip joint)
  │     └── left_shin (بذریعہ left_knee joint)
  │           └── left_foot (بذریعہ left_ankle joint)
  └── right_thigh (بذریعہ right_hip joint)
        └── right_shin (بذریعہ right_knee joint)
              └── right_foot (بذریعہ right_ankle joint)
```

ہر joint ایک **parent link** کو **child link** سے جوڑتا ہے، ایک kinematic tree بناتے ہوئے۔

## URDF XML ڈھانچہ

آئیں URDF میں ایک simplified انسان نما بازو کا جائزہ لیں:

### مثال: انسان نما بایاں بازو

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <!-- Base link (دھڑ) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>  <!-- 30cm x 20cm x 50cm دھڑ -->
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
      <mass value="10.0"/>  <!-- 10 kg دھڑ -->
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- اوپری بازو کا link -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <inertial>
      <mass value="2.0"/>  <!-- 2 kg بازو -->
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0"
               iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- کندھے کا joint (دھڑ کو اوپری بازو سے جوڑتا ہے) -->
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Y-axis کے گرد گھومیں -->
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="2.0"/>
  </joint>

</robot>
```

## خلاصہ

**URDF** ROS 2 میں روبوٹ ڈھانچے کی تفصیل کی معیاری زبان ہے:

- **Links**: Rigid body حصے (بازو، ٹانگیں، دھڑ، سر)
- **Joints**: نسبی حرکت کو قابل بنانے والے connections (revolute، prismatic، fixed)
- **Kinematics**: Parent-child tree structure جو یہ بتاتا ہے کہ حصے کیسے جڑتے ہیں
- **Dynamics**: حقیقی طبیعیات سمولیشن کے لیے mass، inertia، friction
- **Visualization**: Rendering کے لیے 3D meshes یا primitive shapes
- **Collision**: موثر collision detection کے لیے simplified geometry

انسان نما روبوٹوں کے لیے، URDF قابل بناتا ہے:
- Gazebo اور Isaac Sim میں درست سمولیشن
- RViz میں real-time visualization
- Collision avoidance کے ساتھ motion planning
- سمولیشن اور حقیقی ہارڈ ویئر کے درمیان مطابقت

ایک ہی URDF فائل آپ کی پوری ترقیاتی pipeline سے بہتی ہے—سمولیشن سے visualization سے حقیقی روبوٹ کنٹرول تک۔

---

**اگلے اقدامات**: اب آپ ROS 2 کے بنیادی تصورات (nodes، topics، services، actions) اور URDF کیسے روبوٹ ڈھانچے کو بیان کرتا ہے سمجھتے ہیں۔ اگلا، ان تصورات کو عمل میں دیکھنے کے لیے [ماڈیول 2: سمولیشن اور Digital Twins](../module-2-simulation/index.md) کو تلاش کریں!

## حوالہ جات

Open Robotics. (2024). *ROS 2 Documentation: URDF*. https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
