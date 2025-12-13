---
title: سینسر سمولیشن
sidebar_position: 3
description: انسان نما روبوٹ ادراک کے لیے LiDAR، depth کیمرے، IMUs، اور RGB کیمرے simulate کرنا سیکھیں
---

# انسان نما روبوٹوں کے لیے سینسر سمولیشن

## تعارف

سینسرز روبوٹ کی آنکھیں، کان، اور balance کی حس ہیں۔ وہ ادراک کا ڈیٹا فراہم کرتے ہیں جو navigation، manipulation، اور ماحول کے ساتھ interaction کو قابل بناتا ہے۔ حقیقی ہارڈ ویئر پر deploy کرنے سے پہلے مضبوط ادراک الگورتھم تیار کرنے کے لیے سینسرز کو درست طریقے سے simulate کرنا اہم ہے۔

یہ حصہ انسان نما روبوٹکس کے لیے چار ضروری سینسر اقسام کا احاطہ کرتا ہے:

1. **LiDAR**: رکاوٹ detection اور mapping کے لیے laser range finding
2. **Depth کیمرے**: 3D scene سمجھنے کے لیے RGB-D ڈیٹا
3. **IMU**: balance اور orientation کے لیے accelerometer اور gyroscope
4. **RGB کیمرے**: چیز کی شناخت کے لیے بصری ادراک

## LiDAR: Laser Range Finding

### LiDAR کیسے کام کرتا ہے

**LiDAR (Light Detection and Ranging)** laser pulses خارج کرتا ہے اور reflections کے واپس آنے کا وقت ناپتا ہے:

```
فاصلہ = (c * Δt) / 2
```

جہاں:
- `c` = روشنی کی رفتار (3 × 10⁸ m/s)
- `Δt` = round-trip وقت
- 2 سے تقسیم کریں کیونکہ روشنی چیز تک اور واپس سفر کرتی ہے

**Scanning Pattern**:
- **2D LiDAR**: افقی plane scan (مثلاً 270° field of view، 0.25° resolution)
- **3D LiDAR**: متعدد scanning planes (مثلاً 64 عمودی beams، 360° افقی)

### Gazebo میں LiDAR

انسان نما کے سر پر 2D LiDAR کی ترتیب کی مثال:

```xml
<gazebo reference="head">
  <sensor type="ray" name="head_lidar">
    <pose>0 0 0.15 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-2.356</min_angle>
          <max_angle>2.356</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
  </sensor>
</gazebo>
```

## Depth کیمرے: RGB-D ادراک

### Depth کیمرے کیسے کام کرتے ہیں

Depth کیمرے **RGB رنگ کی تصاویر** اور **depth معلومات** (ہر pixel تک فاصلہ) دونوں فراہم کرتے ہیں۔ عام technologies:

1. **Structured Light** (مثلاً Intel RealSense D435): infrared pattern project کرتا ہے، distortion ناپتا ہے
2. **Time-of-Flight** (مثلاً Microsoft Kinect v2): infrared روشنی کا سفر کا وقت ناپتا ہے
3. **Stereo Vision** (مثلاً ZED کیمرہ): دو کیمرہ views سے depth compute کرتا ہے

### Gazebo میں Depth کیمرہ

انسان نما کے سر میں depth کیمرے کی ترتیب کی مثال:

```xml
<gazebo reference="head">
  <sensor type="depth" name="head_depth_camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>10.0</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

## IMU: Inertial Measurement Unit

### IMUs کیسے کام کرتے ہیں

ایک **IMU** جمع کرتا ہے:

1. **Accelerometer**: 3 axes میں linear acceleration ناپتا ہے (m/s²)
2. **Gyroscope**: 3 axes کے گرد angular velocity ناپتا ہے (rad/s)
3. **Magnetometer** (اختیاری): مطلق heading کے لیے magnetic field ناپتا ہے

**اہم بصیرت**: IMU براہ راست position نہیں، **قوتوں اور گردشوں** کو ناپتا ہے۔

### Gazebo میں IMU

انسان نما کے دھڑ میں IMU کی ترتیب کی مثال:

```xml
<gazebo reference="torso">
  <sensor name="torso_imu" type="imu">
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0001</stddev>
          </noise>
        </x>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### انسان نما روبوٹکس کے لیے استعمال کے معاملات

✅ **Balance کنٹرول**
- پتہ لگائیں جب روبوٹ گر رہا ہو (angular velocity تبدیلی)
- Balance corrections کے لیے جسم کا جھکاؤ ناپیں
- استحکام کے لیے تیز feedback loop (100+ Hz)

✅ **گرنے کا پتہ لگانا**
- Acceleration میں اچانک تبدیلیوں کی نگرانی کریں
- حفاظتی ردعمل trigger کریں

## RGB کیمرے: بصری ادراک

### RGB کیمرے کیسے کام کرتے ہیں

معیاری کیمرے رنگ کی معلومات (Red، Green، Blue channels) کے ساتھ 2D تصاویر capture کرتے ہیں۔

### Gazebo میں RGB کیمرہ

انسان نما کے سر میں RGB کیمرے کی ترتیب کی مثال:

```xml
<gazebo reference="head">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>1920</width>
        <height>1080</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

## صحیح سینسر کا انتخاب

### فیصلے کا میٹرکس

| کام | بہترین سینسر | کیوں |
|------|-------------|-----|
| **رکاوٹ سے بچنا (navigation)** | LiDAR | تیز، درست فاصلہ، 360° coverage |
| **چیز پکڑنا** | Depth کیمرہ | 3D شکل + رنگ، قریبی رینج |
| **Balance کنٹرول** | IMU | اعلیٰ تعدد، براہ راست orientation ناپتا ہے |
| **چیز کی شناخت** | RGB کیمرہ | classification کے لیے رنگ/texture |

## خلاصہ

اس ماڈیول نے انسان نما روبوٹکس کے لیے چار ضروری سینسرز کا احاطہ کیا:

1. **LiDAR**: رکاوٹ detection اور mapping کے لیے laser پر مبنی فاصلے کی پیمائش
2. **Depth کیمرے**: manipulation اور scene سمجھنے کے لیے رنگ اور depth جمع کرنے والا RGB-D ڈیٹا
3. **IMU**: balance کنٹرول اور state estimation کے لیے accelerometer + gyroscope
4. **RGB کیمرے**: چیز کی شناخت اور انسانی interaction کے لیے بصری ادراک

---

**جاری رکھیں**: [Digital Twin ورک فلوز](./digital-twin.md)

## حوالہ جات

Open Robotics. (2024). *Gazebo Documentation: Sensors*. https://gazebosim.org/api/sensors/8/namespaceignition_1_1sensors.html
