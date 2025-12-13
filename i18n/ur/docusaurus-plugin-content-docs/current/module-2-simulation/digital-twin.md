---
title: Digital Twin ورک فلوز
sidebar_position: 4
description: انسان نما روبوٹ ترقی کے لیے Gazebo اور Unity کے ساتھ Digital Twins بنانا اور استعمال کرنا سیکھیں
---

# Digital Twin ورک فلوز

## تعارف

آپ نے طبیعیات سمولیشن اور سینسر ماڈلز کے بارے میں سیکھ لیا ہے۔ اب آئیں یہ سب کچھ اکٹھا کریں: انسان نما روبوٹ ترقی کے لیے **Digital Twins** بنانا۔ ایک Digital Twin ایک virtual replica ہے جو حقیقی روبوٹ کی طرح برتاؤ کرتا ہے، آپ کو ہارڈ ویئر deployment سے پہلے محفوظ طریقے سے الگورتھم test کرنے کی اجازت دیتا ہے۔

یہ حصہ احاطہ کرتا ہے:
- Gazebo سمولیشن میں URDF ماڈلز load کرنا
- اعلیٰ معیار کی visualization کے لیے Unity جوڑنا
- Virtually چلنے کی gaits test کرنا
- Manipulation الگورتھم validate کرنا
- Sim-to-real منتقلی کی حکمت عملی

## Digital Twin Pipeline

ایک مکمل Digital Twin ورک فلو تین اہم مراحل پر مشتمل ہے:

```
URDF ماڈل → Gazebo سمولیشن → ROS 2 Topics → Unity Rendering → Robot Controller
```

**بہاؤ**:
1. **URDF ماڈل**: روبوٹ ڈھانچہ کی تعریف (ماڈیول 1)
2. **Gazebo سمولیشن**: طبیعیات اور سینسر ڈیٹا compute کرتا ہے
3. **ROS 2 Topics**: اجزاء کے درمیان ڈیٹا منتقل کرتا ہے
4. **Unity Rendering** (اختیاری): اعلیٰ معیار میں visualize کرتا ہے
5. **Robot Controller**: آپ کے الگورتھم (چلنا، پکڑنا، navigation)

## Gazebo میں URDF load کرنا

### مرحلہ 1: اپنا URDF تیار کریں

یقینی بنائیں کہ آپ کے URDF میں شامل ہے:
- تمام links کے لیے درست masses اور inertias
- طبیعیات کے لیے collision geometries
- Joint limits اور dynamics
- Gazebo مخصوص سینسر plugins (LiDAR، کیمرے، IMU)

### مرحلہ 2: Launch فائل بنائیں

ROS 2 launch فائلیں Gazebo startup کو orchestrate کرتی ہیں:

```python
# launch/gazebo_humanoid.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join('path', 'to', 'humanoid.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'humanoid'],
        output='screen'
    )

    return LaunchDescription([spawn_entity])
```

### مرحلہ 3: سمولیشن Launch کریں

```bash
ros2 launch my_robot_description gazebo_humanoid.launch.py
```

## Visualization کے لیے Unity جوڑنا

Unity حقیقت پسندانہ rendering فراہم کرتا ہے جو Gazebo match نہیں کر سکتا۔ ورک فلو:

```
Gazebo (طبیعیات) → ROS 2 Topics → Unity (Visuals)
```

### مرحلہ 1: Unity Robotics Hub انسٹال کریں

1. Unity Hub اور Unity Editor ڈاؤن لوڈ کریں
2. Unity Robotics Hub packages انسٹال کریں

### مرحلہ 2: Unity میں URDF Import کریں

Unity کا URDF Importer آپ کی روبوٹ تفصیل پڑھتا ہے:

```
Assets → Import Robot from URDF
  → اپنی humanoid.urdf فائل منتخب کریں
  → Unity URDF ڈھانچے سے مماثل GameObjects بناتا ہے
```

## سمولیشن میں چلنے کی Gaits test کرنا

### ورک فلو: تیار کریں → Simulate کریں → Validate کریں → Deploy کریں

#### 1. Walking Controller تیار کریں

Python میں gait planner لکھیں:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleGaitController(Node):
    def __init__(self):
        super().__init__('gait_controller')
        
        # Balance feedback کے لیے IMU subscribe کریں
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid/imu/data', self.imu_callback, 10
        )
        
        # Joint commands publish کریں
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/humanoid/joint_commands', 10
        )
        
        self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        # موجودہ gait phase کے لیے مطلوبہ joint positions compute کریں
        joint_commands = self.compute_gait_phase()
        self.joint_pub.publish(joint_commands)
```

#### 2. Gazebo میں Test کریں

```bash
# Terminal 1: سمولیشن شروع کریں
ros2 launch my_robot_description gazebo_humanoid.launch.py

# Terminal 2: Gait controller چلائیں
ros2 run my_robot_control simple_gait_controller

# Terminal 3: روبوٹ state کی نگرانی کریں
ros2 topic echo /humanoid/imu/data
```

## Sim-to-Real منتقلی

### Reality Gap

سمولیشن ≠ حقیقت کی وجہ سے:

1. **طبیعیات کے Approximations**
   - Contact dynamics simplified
   - Friction ماڈلز تمام surface variations capture نہیں کرتے

2. **سینسر اختلافات**
   - حقیقی سینسرز میں systematic biases ہیں
   - Noise ماڈلز سمولیشن میں idealized ہیں

### Gap کو پاٹنے کی حکمت عملیاں

#### 1. Domain Randomization

Training کے دوران سمولیشن parameters vary کریں:

```python
# ہر episode میں friction randomize کریں
friction = random.uniform(0.5, 1.5)

# چیز کی masses randomize کریں
object_mass = random.uniform(0.1, 0.5)

# سینسر noise randomize کریں
imu_noise_stddev = random.uniform(0.001, 0.01)
```

#### 2. درست Parameter Identification

حقیقی روبوٹ parameters ناپیں:
- ہر link کو تولیں
- Joint friction ناپیں
- سینسر noise calibrate کریں

URDF کو حقیقت سے match کرنے کے لیے اپ ڈیٹ کریں۔

## خلاصہ

Digital Twin ورک فلوز محفوظ، موثر انسان نما روبوٹ ترقی کو قابل بناتے ہیں:

**Pipeline**:
1. **URDF ماڈل** → روبوٹ ڈھانچہ کی تعریف کرتا ہے
2. **Gazebo سمولیشن** → طبیعیات اور سینسرز compute کرتا ہے
3. **ROS 2 Topics** → ڈیٹا منتقل کرتا ہے
4. **Unity** (اختیاری) → اعلیٰ معیار کی visualization
5. **Controllers** → آپ کے الگورتھم

**ترقیاتی چکر**:
- سمولیشن میں الگورتھم تیار کریں
- حقیقی طبیعیات اور سینسرز کے ساتھ test کریں
- تیزی سے iterate کریں (ہارڈ ویئر نقصان کا کوئی خطرہ نہیں)
- حقیقی ہارڈ ویئر پر validate کریں
- حقیقی ڈیٹا کی بنیاد پر سمولیشن اپ ڈیٹ کریں

ایک اچھی طرح calibrated Digital Twin کے ساتھ، آپ ہارڈ ویئر چھونے سے پہلے اپنے روبوٹ کی 80% صلاحیتیں تیار کر سکتے ہیں—وقت، پیسے، اور روبوٹوں کی بچت!

---

**اگلے اقدامات**: اعلیٰ ادراک کے لیے تیار ہیں؟ حقیقت پسندانہ سمولیشن اور مصنوعی ڈیٹا جنریشن کے لیے [ماڈیول 3: AI سے چلنے والی ادراک (NVIDIA Isaac)](../module-3-isaac/index.md) پر جاری رکھیں۔

## حوالہ جات

Open Robotics. (2024). *Gazebo Documentation: Spawning Models*. https://gazebosim.org/api/gazebo/7/spawn_model.html

Unity Technologies. (2024). *Unity Robotics Hub: URDF Importer*. https://github.com/Unity-Technologies/URDF-Importer
