---
title: Visual SLAM
sidebar_position: 3
description: انسان نما navigation کے لیے Visual Simultaneous Localization and Mapping
---

# Visual SLAM: بصری نقشہ سازی اور Localization

## تعارف

**Visual SLAM (Simultaneous Localization and Mapping)** روبوٹ کو بیک وقت ایک نقشہ بنانے اور اس نقشے کے اندر اپنے آپ کو localize کرنے کی اجازت دیتا ہے—صرف کیمرہ ڈیٹا استعمال کرتے ہوئے۔

## کیسے کام کرتا ہے

1. **Feature Detection**: کیمرہ تصاویر میں distinctive points تلاش کریں
2. **Feature Matching**: مسلسل frames میں features match کریں
3. **Motion Estimation**: کیمرے کی حرکت calculate کریں
4. **Map Building**: ماحول کا 3D نقشہ بنائیں

## Isaac ROS Visual SLAM

NVIDIA GPU سے تیز کردہ Visual SLAM package فراہم کرتا ہے:

```bash
# Isaac ROS Visual SLAM انسٹال کریں
sudo apt install ros-humble-isaac-ros-visual-slam

# Node launch کریں
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### فوائد

✅ **تیز processing**: GPU acceleration
✅ **درست localization**: سینٹی میٹر level درستگی
✅ **Real-time mapping**: حرکت کرتے ہوئے نقشے بنائیں

## انسان نما کے لیے استعمال کے معاملات

- Indoor navigation
- نامعلوم ماحول میں تلاش
- GPS کے بغیر localization

---

**جاری رکھیں**: [Nav2 راستہ کی منصوبہ بندی](./nav2.md)

## حوالہ جات

NVIDIA. (2024). *Isaac ROS Visual SLAM*. https://nvidia-isaac-ros.github.io/
