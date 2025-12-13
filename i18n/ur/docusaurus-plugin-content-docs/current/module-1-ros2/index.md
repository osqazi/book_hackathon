---
title: ماڈیول 1 - ROS 2 بنیادی باتیں
sidebar_position: 1
description: روبوٹک اعصابی نظام سیکھیں - ROS 2 فن تعمیر، مواصلاتی نمونے، اور انسان نما روبوٹوں کے لیے URDF ماڈلنگ
---

# ماڈیول 1: انسان نما روبوٹکس کے لیے ROS 2 بنیادی باتیں

## آپ کیا سیکھیں گے

جدید روبوٹکس سافٹ ویئر کی بنیاد میں خوش آمدید! یہ ماڈیول **ROS 2 (Robot Operating System 2)** متعارف کراتا ہے، وہ middleware جو انسان نما روبوٹوں کے لیے "اعصابی نظام" کے طور پر کام کرتا ہے۔ جس طرح آپ کا اعصابی نظام آپ کے دماغ، سینسرز، اور پٹھوں کے درمیان سگنلز کو مربوط کرتا ہے، ROS 2 سافٹ ویئر کمپوننٹس کے درمیان مواصلات کو مربوط کرتا ہے جو روبوٹک حرکات کو سمجھتے، منصوبہ بندی کرتے، اور کنٹرول کرتے ہیں۔

اس ماڈیول کے اختتام تک، آپ سمجھیں گے:

- **ROS 2 کور فن تعمیر**: Nodes، Topics، Services، اور Actions
- **مواصلاتی نمونے**: Asynchronous topics بمقابلہ synchronous services بمقابلہ طویل چلنے والے actions کب استعمال کریں
- **Python Integration**: `rclpy` لائبریری استعمال کرتے ہوئے ROS 2 پروگرام لکھنا
- **Robot Modeling**: URDF (Unified Robot Description Format) استعمال کرتے ہوئے انسان نما روبوٹ ڈھانچے کی تفصیل
- **System Integration**: کیسے Python کوڈ روبوٹ کنٹرولرز اور actuators سے جڑتا ہے

## انسان نما روبوٹوں کے لیے ROS 2 کیوں؟

انسان نما روبوٹ پیچیدہ نظام ہیں جن میں دسیوں سینسرز، actuators، اور computational processes بیک وقت چل رہے ہوتے ہیں۔ ایک عام انسان نما کو چاہیے:

- کیمروں، IMUs، force sensors، اور joint encoders سے **سینسر ڈیٹا پروسیس کریں** تیز تعدد پر
- 20+ degrees of freedom (ٹانگیں، بازو، دھڑ، سر) کے لیے **موٹر کمانڈز کو عملی جامہ پہنائیں**
- بصارت، توازن، اور ماحول کی سمجھ کے لیے **ادراک کے الگورتھم چلائیں**
- چلنے کے دوران توازن برقرار رکھتے ہوئے اشیاء کو ٹریک کرنے جیسے **کاموں کو مربوط کریں**

یہ پیچیدگی دستی طور پر سنبھالنا بہت مشکل ہوگا۔ ROS 2 فراہم کرتا ہے:

1. **Modularity**: پیچیدہ نظاموں کو آزاد، قابل انتظام اجزاء (nodes) میں توڑیں
2. **Interoperability**: معیاری انٹرفیس مختلف سافٹ ویئر کمپوننٹس کو بغیر رکاوٹ بات چیت کرنے دیتے ہیں
3. **Scalability**: متعدد پروسیسرز یا کمپیوٹرز میں computation تقسیم کریں
4. **Reliability**: Quality-of-Service (QoS) پالیسیاں یقینی بناتی ہیں کہ اہم ڈیٹا وقت پر پہنچے
5. **Community**: نیویگیشن، ادراک، manipulation، اور مزید کے لیے ہزاروں پہلے سے بنے packages

### ROS 2 بمقابلہ ROS 1 (سیاق و سباق)

اگر آپ نے پہلے "ROS" کے بارے میں سنا ہے، تو آپ "2" کے بارے میں سوچ رہے ہوں گے۔ ROS 2 ایک مکمل نیا ڈیزائن ہے جو اصل ROS کی حدود کو حل کرتا ہے:

- **Real-time قابل**: وقت کے لحاظ سے اہم کنٹرول کے لیے deterministic مواصلات کی حمایت
- **Production-ready**: تعینات روبوٹوں کے لیے سیکیورٹی، quality-of-service، اور lifecycle مینجمنٹ
- **Multi-platform**: Linux، Windows، macOS، اور embedded systems پر چلتا ہے
- **DDS middleware**: قابل اعتماد نیٹ ورکنگ کے لیے صنعتی معیار Data Distribution Service

یہ کتاب خاص طور پر ROS 2 پر توجہ مرکوز کرتی ہے (ہم فرض کرتے ہیں کہ آپ نئے شروع کر رہے ہیں)۔ اگر آپ ROS 1 سے واقف ہیں، تو آپ کو بہت سے تصورات ایک جیسے ملیں گے لیکن بہتر implementations کے ساتھ۔

## سیکھنے کے مقاصد

اس ماڈیول کو مکمل کرنے کے بعد، آپ قابل ہوں گے:

1. **ROS 2 تصورات کی وضاحت کریں** انسان نما روبوٹکس کی مثالوں کا استعمال کرتے ہوئے
   - Nodes، topics، services، اور actions کی تعریف کریں
   - شناخت کریں کہ انسان نما کنٹرول نظام متعدد آزاد nodes کیوں استعمال کرتے ہیں

2. **مناسب مواصلاتی نمونے منتخب کریں** روبوٹکس کاموں کے لیے
   - مسلسل سینسر streams کے لیے topics کا انتخاب کریں
   - فوری queries کے لیے services کا انتخاب کریں
   - feedback کے ساتھ ہدف پر مبنی کاموں کے لیے actions کا انتخاب کریں

3. **URDF روبوٹ تفصیلات کی تشریح کریں**
   - URDF XML ڈھانچے کو پڑھیں
   - Links (rigid body parts) اور joints (connections) کی شناخت کریں
   - سمجھیں کہ URDF کیسے simulated اور حقیقی روبوٹوں سے map ہوتا ہے

4. **ROS 2 نظاموں میں ڈیٹا فلو کا سراغ لگائیں**
   - Python scripts سے middleware کے ذریعے controllers تک کمانڈز کی پیروی کریں
   - سینسر ڈیٹا کے لیے feedback loops کو سمجھیں

## پیشگی تقاضے

یہ ماڈیول فرض کرتا ہے کہ آپ کے پاس ہے:

- **بنیادی Python پروگرامنگ**: Variables، functions، loops، classes، اور object-oriented تصورات
- **Command-line سے واقفیت**: Directories navigate کرنا، scripts چلانا، packages انسٹال کرنا
- **روبوٹکس کا شوق**: ROS یا روبوٹکس کا پہلے سے تجربہ ضروری نہیں!

**جس کی آپ کو ضرورت نہیں**: پہلے سے ROS 1 کا تجربہ، C++ کا علم، یا ہارڈ ویئر روبوٹکس کا پس منظر۔

## ماڈیول کا ڈھانچہ

یہ ماڈیول چار اہم حصوں پر مشتمل ہے:

### 1. [Nodes اور Topics](./nodes-topics.md)
**Nodes** (آزاد processes) اور **topics** (asynchronous مواصلاتی چینلز) کے بارے میں جانیں۔ آپ دیکھیں گے کہ سینسر ڈیٹا publish-subscribe نمونے استعمال کرتے ہوئے انسان نما روبوٹ کے سافٹ ویئر stack میں کیسے بہتا ہے۔

**کلیدی تصورات**: Process isolation، publish-subscribe، many-to-many مواصلات، asynchronous messaging

### 2. [Services اور Actions](./services-actions.md)
**Services** (synchronous request-response) اور **actions** (feedback کے ساتھ طویل چلنے والے کام) کو سمجھیں۔ جانیں کہ انسان نما کنٹرول کاموں کے لیے ہر نمونے کو کب استعمال کریں۔

**کلیدی تصورات**: Client-server نمونہ، synchronous بمقابلہ asynchronous، goal-oriented execution، preemptable tasks

### 3. [URDF Robot Modeling](./urdf-modeling.md)
تلاش کریں کہ **URDF** (Unified Robot Description Format) کیسے انسان نما روبوٹ کی physical structure، kinematics، اور dynamics کی تفصیل کرتا ہے۔ دیکھیں کہ کیسے ایک ہی URDF فائل simulation، visualization، اور حقیقی روبوٹ کنٹرول کو چلاتی ہے۔

**کلیدی تصورات**: Links، joints، kinematic chains، collision geometry، visual representation

## تنصیب اور سیٹ اپ

یہ ماڈیول **تصورات اور فن تعمیر** پر توجہ مرکوز کرتا ہے، تنصیب کے طریقہ کار پر نہیں۔ کوڈ کی مثالوں کے ساتھ یا اپنے تجربات چلانے کے لیے، آپ کو ROS 2 کی ضرورت ہوگی۔

**سرکاری تنصیب گائیڈ**: [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

ہم تجویز کرتے ہیں:
- **ROS 2 Humble Hawksbill** (LTS release، 2027 تک supported)
- **Ubuntu 22.04** (سب سے عام platform، بہترین community support)
- متبادل طور پر، فوری سیٹ اپ کے لیے [Docker containers](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html) استعمال کریں

اس ماڈیول میں تمام کوڈ کی مثالیں Python (`rclpy`) استعمال کرتی ہیں، C++ نہیں۔ اگر آپ `python3` چلا سکتے ہیں، تو آپ سیکھنے کے لیے تیار ہیں!

## آگے کیا ہے؟

ROS 2 بنیادی باتوں میں مہارت حاصل کرنے کے بعد، آپ تیار ہوں گے:

- **ماڈیول 2**: Gazebo اور Unity میں انسان نما روبوٹوں کو simulate کریں
- **ماڈیول 3**: NVIDIA Isaac کے ساتھ AI سے چلنے والی ادراک اور نیویگیشن شامل کریں
- **ماڈیول 4**: Vision-Language-Action ماڈلز استعمال کرتے ہوئے خودکار نظام بنائیں

ROS 2 ہر چیز کی بنیاد ہے جو آگے آتی ہے۔ آئیں یہ سمجھ کر شروع کریں کہ nodes اور topics کیسے modular روبوٹ سافٹ ویئر کو قابل بناتے ہیں!

---

**شروع کرنے کے لیے تیار؟** ROS 2 کے بنیادی building blocks سیکھنے کے لیے [Nodes اور Topics](./nodes-topics.md) پر جاری رکھیں۔

## حوالہ جات

Open Robotics. (2024). *ROS 2 Documentation: Humble Hawksbill*. https://docs.ros.org/en/humble/
