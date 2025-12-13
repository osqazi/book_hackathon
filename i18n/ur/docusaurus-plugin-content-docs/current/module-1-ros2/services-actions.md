---
title: Services اور Actions
sidebar_position: 3
description: انسان نما روبوٹ کنٹرول میں synchronous requests کے لیے ROS 2 services اور طویل چلنے والے کاموں کے لیے actions کب استعمال کریں
---

# Services اور Actions: Request-Response اور ہدف پر مبنی مواصلات

## تعارف

پچھلے حصے میں، ہم نے **topics** کے بارے میں سیکھا—asynchronous streams جو مسلسل ڈیٹا بہاؤ کے لیے مثالی ہیں۔ لیکن ان حالات کے بارے میں کیا جہاں آپ کو ضرورت ہے:

- سوال کا فوری جواب ("موجودہ بیٹری لیول کیا ہے؟")
- تصدیق کہ کمانڈ عملدرآمد ہو گئی ("بازو کو پوزیشن X پر لے جائیں—کیا یہ کامیاب ہوا؟")
- طویل چلنے والے کاموں کے لیے پیشرفت کی اپ ڈیٹس ("10 میٹر آگے چلیں—ہم کتنی دور آئے ہیں؟")

ان منظرناموں کے لیے، ROS 2 **services** اور **actions** فراہم کرتا ہے۔

## Services: Synchronous Request-Response

ایک **service** client-server نمونہ implement کرتی ہے: ایک node معلومات یا عمل کی درخواست کرتا ہے، اور دوسرا node جواب دیتا ہے۔

### Services کی اہم خصوصیات

1. **Synchronous**: Client جواب کا انتظار کرتا ہے
2. **One-to-One**: ایک client فی request ایک server سے بات کرتا ہے
3. **قلیل مدت**: فوری آپریشنز کے لیے بہترین (milliseconds سے چند سیکنڈ تک)
4. **Request-Reply ڈھانچہ**: Client request ڈیٹا بھیجتا ہے، server response ڈیٹا بھیجتا ہے

### Service تعریف کا ڈھانچہ

Services دو حصوں کے ساتھ تعریف کی جاتی ہیں:

```
Request   → ڈیٹا جو client server کو بھیجتا ہے
Response  ← ڈیٹا جو server client کو واپس بھیجتا ہے
```

مثال service type `std_srvs/SetBool`:
```
Request:
  bool data      # true یا false

Response:
  bool success   # کیا آپریشن کامیاب ہوا؟
  string message # اختیاری status پیغام
```

### Services کب استعمال کریں

✅ **مثالی برائے:**
- روبوٹ کی حالت پوچھنا ("موجودہ joint angles حاصل کریں")
- منفرد اعمال trigger کرنا ("موٹر فعال/غیر فعال کریں"، "Odometry ری سیٹ کریں")
- ترتیب کی تبدیلیاں ("Controller gains سیٹ کریں"، "کیمرا mode سوئچ کریں")
- مختصر calculations ("Pose کے لیے inverse kinematics"، "Footstep کی منصوبہ بندی")

❌ **مثالی نہیں برائے:**
- مسلسل ڈیٹا streams (topics استعمال کریں)
- طویل چلنے والے کام (actions استعمال کریں)
- یک طرفہ fire-and-forget پیغامات (topics استعمال کریں)

## انسان نما Service مثالیں

| کام | Service Type | استعمال کا معاملہ |
|------|--------------|----------|
| بیٹری لیول حاصل کریں | `GetBatteryStatus` | باقی power پوچھیں |
| چلنے کی رفتار سیٹ کریں | `SetFloat64` | Gait velocity limit تبدیل کریں |
| Inverse kinematics compute کریں | `ComputeIK` | End-effector pose کے لیے joint angles calculate کریں |
| IMU calibration ری سیٹ کریں | `Trigger` | Balance sensors دوبارہ calibrate کریں |
| Joints پر torque فعال کریں | `SetBool` | Motor controllers کو activate/deactivate کریں |

## Actions: طویل چلنے والے، Preemptable کام Feedback کے ساتھ

Services فوری requests کے لیے اچھی طرح کام کرتی ہیں، لیکن ان کاموں کے بارے میں کیا جو وقت لیتے ہیں؟

- "10 میٹر آگے چلیں" (کئی سیکنڈ لیتا ہے)
- "چیز اٹھائیں" (کثیر مرحلہ manipulation)
- "Waypoint تک navigate کریں" (منصوبہ بندی اور رکاوٹوں سے بچنا شامل ہے)

ان کے لیے، ROS 2 **actions** فراہم کرتا ہے—ہدف پر مبنی کام جن میں:

1. **Goal**: آپ کیا حاصل کرنا چاہتے ہیں
2. **Feedback**: عملدرآمد کے دوران متواتر اپ ڈیٹس
3. **Result**: مکمل ہونے پر حتمی نتیجہ
4. **Preemption**: درمیان میں منسوخ کرنے کی صلاحیت

### Action ڈھانچہ

```
Goal     → Client مطلوبہ نتیجہ بھیجتا ہے (مثلاً "Position [x, y, z] پر جائیں")
Feedback ← Server متواتر اپ ڈیٹس بھیجتا ہے (مثلاً "50% مکمل، موجودہ position [x, y, z]")
Result   ← Server حتمی status بھیجتا ہے (مثلاً "کامیاب" یا "ناکام: رکاوٹ کا پتہ چلا")
```

### Action مثال: ہدف Position تک چلنا

```
Client (Behavior Planner) → Goal: "[x=5.0, y=2.0] تک چلیں"
                             ↓
Server (Gait Controller)
                             ↓
                          Feedback: "باقی فاصلہ: 3.2m"
                          Feedback: "باقی فاصلہ: 1.5m"
                             ↓
Client ← Result: "ہدف کامیابی سے پہنچ گیا"
```

اگر client چلتے وقت منسوخ کرنے کا فیصلہ کرتا ہے (مثلاً رکاوٹ کا پتہ چلا)، تو وہ action کو **preempt** کر سکتا ہے۔

## Actions بمقابلہ Services بمقابلہ Topics کب استعمال کریں

| استعمال کا معاملہ | مواصلات کی قسم | مثال |
|----------|-------------------|---------|
| مسلسل سینسر ڈیٹا | **Topic** | کیمرے کی تصاویر، IMU readings |
| فوری query | **Service** | "بیٹری لیول حاصل کریں" |
| فوری کمانڈ | **Service** | "موٹر torque فعال کریں" |
| پیشرفت کے ساتھ طویل کام | **Action** | "Waypoint تک چلیں"، "چیز پکڑیں" |
| منسوخ کرنے کی ضرورت، طویل کام | **Action** | "ہدف تک navigate کریں" (رکاوٹ نظر آنے پر منسوخ کر سکتے ہیں) |
| Feedback کی ضرورت، طویل کام | **Action** | "Trajectory عملی جامہ پہنائیں" (پیشرفت monitor کریں) |

## فیصلے کا درخت: صحیح نمونہ کا انتخاب

```
مواصلات کی ضرورت ہے؟
  ├─ صرف مسلسل ڈیٹا بھیج رہے ہیں؟ → **Topic (Publisher)**
  ├─ جواب کی ضرورت ہے؟
      ├─ فوری (< 1 سیکنڈ)؟ → **Service**
      ├─ طویل چلنے والا (> 1 سیکنڈ)؟
          ├─ پیشرفت کی اپ ڈیٹس کی ضرورت؟ → **Action**
          ├─ منسوخ کرنے کی ضرورت ہو سکتی ہے؟ → **Action**
          ├─ Fire اور forget؟ → **Topic (Publisher)**
          └─ صرف synchronous نتیجہ؟ → **Service** (اگر کافی مختصر ہو)
```

## انسان نما روبوٹکس میں Actions

### مثال: چیز پکڑنا

```
Goal:
  object_id: "cup_123"
  approach_vector: [0, 0, -1]  # اوپر سے approach کریں
  grasp_force: 5.0  # Newtons

Feedback (ہر 0.5 سیکنڈ):
  stage: "approaching"      # موجودہ مرحلہ
  hand_position: [x, y, z]  # موجودہ end-effector pose
  distance_to_object: 0.15  # میٹرز

Result:
  success: True
  final_grasp_force: 4.8
  message: "چیز محفوظ طریقے سے پکڑی گئی"
```

## خلاصہ

ROS 2 تین مواصلاتی نمونے فراہم کرتا ہے، ہر ایک مختلف استعمال کے معاملات کے لیے بہتر:

| نمونہ | سمت | Timing | استعمال کا معاملہ |
|---------|-----------|--------|----------|
| **Topic** | Many-to-many | Asynchronous | مسلسل ڈیٹا streams |
| **Service** | One-to-one | Synchronous | فوری request-response |
| **Action** | One-to-one | Asynchronous | Feedback کے ساتھ طویل کام |

انسان نما روبوٹکس کے لیے:
- **Topics**: سینسر streams (کیمرے، IMU، joint states)، مسلسل موٹر کمانڈز
- **Services**: فوری queries (بیٹری لیول، joint positions)، motors فعال/غیر فعال کریں
- **Actions**: پیچیدہ رویے (چلنا، پکڑنا، navigate کرنا)، کثیر مرحلہ کام

صحیح نمونہ کب استعمال کریں یہ سمجھنا مضبوط، موثر روبوٹ کنٹرول نظام بنانے کے لیے اہم ہے۔

---

**جاری رکھیں**: [URDF Robot Modeling](./urdf-modeling.md)

## حوالہ جات

Open Robotics. (2024). *ROS 2 Documentation: Understanding Services*. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html

Open Robotics. (2024). *ROS 2 Documentation: Understanding Actions*. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html
