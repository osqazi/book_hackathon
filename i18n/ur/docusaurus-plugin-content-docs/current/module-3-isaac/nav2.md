---
title: Nav2 راستہ کی منصوبہ بندی
sidebar_position: 4
description: دو پیروں والے انسان نما روبوٹوں کے لیے Nav2 navigation
---

# Nav2: دو پیروں والے روبوٹوں کے لیے راستہ کی منصوبہ بندی

## تعارف

**Nav2** ROS 2 navigation stack ہے جو خودکار راستہ کی منصوبہ بندی اور رکاوٹ سے بچنا فراہم کرتا ہے۔ انسان نما روبوٹوں کے لیے، Nav2 کو دو پیروں پر چلنے کے خاص تحفظات کو handle کرنے کے لیے customize کیا جا سکتا ہے۔

## اہم اجزاء

### 1. Costmap Generator
ماحول کی رکاوٹوں کی نمائندگی:
- سینسرز (LiDAR، کیمرے) سے ڈیٹا
- Static نقشے
- Dynamic رکاوٹیں

### 2. Global Planner
شروع سے منزل تک راستہ find کریں:
- A* الگورتھم
- Dijkstra الگورتھم

### 3. Local Planner
حقیقی وقت میں راستے کو execute کریں:
- Dynamic Window Approach (DWA)
- Timed Elastic Band (TEB)

## Nav2 Setup

```bash
# Nav2 انسٹال کریں
sudo apt install ros-humble-navigation2

# Navigation launch کریں
ros2 launch nav2_bringup navigation_launch.py
```

### انسان نما کے لیے ترتیب

```yaml
# nav2_params.yaml
local_costmap:
  footprint: [[0.15, 0.1], [0.15, -0.1], [-0.15, -0.1], [-0.15, 0.1]]
  
controller_server:
  max_vel_x: 0.5  # آہستہ walking speed
  min_turning_radius: 0.3  # دو پیروں کے تحفظات
```

---

**جاری رکھیں**: [مصنوعی ڈیٹا جنریشن](./synthetic-data.md)

## حوالہ جات

Open Robotics. (2024). *Nav2 Documentation*. https://nav2.org/
