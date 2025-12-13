---
title: ماڈیول 3 - AI سے چلنے والی ادراک (NVIDIA Isaac)
sidebar_position: 1
description: NVIDIA Isaac پلیٹ فارم کے ساتھ حقیقت پسندانہ سمولیشن، Visual SLAM، اور AI سے چلنے والی navigation میں مہارت حاصل کریں
---

# ماڈیول 3: NVIDIA Isaac کے ساتھ AI سے چلنے والی ادراک

## تعارف

آپ نے ROS 2 بنیادی باتوں اور Gazebo کے ساتھ طبیعیات پر مبنی سمولیشن میں مہارت حاصل کر لی ہے۔ اب **NVIDIA Isaac** کے ساتھ اپنی انسان نما روبوٹکس صلاحیتوں کو بلند کرنے کا وقت ہے—ایک پلیٹ فارم جو آپ کے روبوٹوں میں حقیقت پسندانہ سمولیشن، GPU سے تیز کردہ AI ادراک، اور production کے لیے تیار navigation لاتا ہے۔

جبکہ Gazebo بہترین طبیعیات فراہم کرتا ہے، NVIDIA Isaac شامل کرتا ہے:
- Ray-traced روشنی اور سائے کے ساتھ **حقیقت پسندانہ rendering**
- حقیقی کیمرہ اور LiDAR رویے سے مماثل **اعلیٰ معیار کا سینسر سمولیشن**
- Real-time AI inference کے لیے **GPU سے تیز کردہ ادراک**
- بڑے پیمانے پر vision ماڈلز کی تربیت کے لیے **مصنوعی ڈیٹا جنریشن**
- پیچیدہ ماحول کے لیے بہتر **Production-grade navigation** Nav2 کے ساتھ

یہ ماڈیول بنیادی سمولیشن اور AI سے چلنے والے خودکار نظاموں کے درمیان gap کو پاٹتا ہے۔

## آپ کیا سیکھیں گے

ماڈیولز 1 اور 2 پر تعمیر کرتے ہوئے، یہ ماڈیول تلاش کرتا ہے:

- **Isaac Sim**: Omniverse سے powered NVIDIA کا حقیقت پسندانہ روبوٹ simulator
- **Isaac ROS**: Visual SLAM اور object detection کے لیے GPU سے تیز کردہ ادراک packages
- **Visual SLAM**: انسان نما navigation کے لیے Simultaneous Localization and Mapping
- **Nav2 Integration**: دو پیروں والے روبوٹوں کے لیے راستہ کی منصوبہ بندی اور رکاوٹ سے بچنا
- **مصنوعی ڈیٹا جنریشن**: Domain randomization کے ساتھ training datasets بنانا
- **Sim-to-Real منتقلی**: سمولیشن سے حقیقی ہارڈ ویئر پر AI ماڈلز deploy کرنا

## انسان نما روبوٹکس کے لیے NVIDIA Isaac کیوں؟

### روایتی سمولیشن سے آگے

روایتی simulators جیسے Gazebo طبیعیات میں بہترین ہیں، لیکن جدید AI روبوٹکس کی ضرورت ہے:

**حقیقت پسندانہ Visuals**
- حقیقی مصنوعی تصاویر پر vision ماڈلز train کریں
- مختلف روشنی کی حالتوں میں ادراک الگورتھم test کریں
- خودکار طور پر labeled datasets (bounding boxes، segmentation masks) بنائیں

**GPU Acceleration**
- ایک GPU پر متعدد سمولیشن متوازی طور پر چلائیں
- حقیقی کیمروں اور LiDAR کے لیے ray tracing تیز کریں
- AI inference (object detection، SLAM) real-time میں execute کریں

## ماڈیول کا ڈھانچہ

### 1. [Isaac Sim: حقیقت پسندانہ سمولیشن](./isaac-sim.md)
NVIDIA کے flagship روبوٹ simulator کو تلاش کریں:
- Omniverse USD: Universal Scene Description format
- حقیقی کیمروں کے لیے ray-traced rendering
- جسمانی طور پر درست سینسر ماڈلز
- ایک GPU پر کثیر روبوٹ سمولیشن
- ROS 2 integration اور ورک فلو

### 2. [انسان نما Navigation کے لیے Visual SLAM](./vslam.md)
Simultaneous Localization and Mapping میں مہارت حاصل کریں:
- Visual odometry: کیمرے کی حرکت کو track کرنا
- Feature extraction اور matching
- کیمرے کے مشاہدات سے نقشہ بنانا
- Isaac ROS Visual SLAM package

### 3. [دو پیروں والے روبوٹوں کے لیے Nav2](./nav2.md)
خودکار طور پر پیچیدہ ماحول میں navigate کریں:
- Nav2 architecture اور plugins
- سینسرز سے costmap جنریشن
- Global راستہ کی منصوبہ بندی
- دو پیروں کے تحفظات

### 4. [مصنوعی ڈیٹا جنریشن](./synthetic-data.md)
بڑے پیمانے پر AI training datasets بنائیں:
- Domain randomization تکنیک
- روشنی اور texture variation
- چیز کی جگہ کی حکمت عملی
- خودکار labeling
- Sim-to-real منتقلی کے بہترین طریقے

## پیشگی تقاضے

اس ماڈیول کو شروع کرنے سے پہلے، آپ کو سمجھنا چاہیے:

- **ROS 2 بنیادی باتیں** (ماڈیول 1): Nodes، topics، services، URDF
- **سمولیشن بنیادی باتیں** (ماڈیول 2): طبیعیات، سینسرز، Gazebo ورک فلوز
- **بنیادی AI تصورات**: Neural networks، training بمقابلہ inference

**ہارڈ ویئر تجویزات**:
- NVIDIA GPU (GTX 1070+ یا RTX series تجویز کردہ)
- 16GB+ RAM
- Ubuntu 20.04 یا 22.04

## Isaac بمقابلہ Gazebo: ہر ایک کب استعمال کریں

| خصوصیت | Gazebo | Isaac Sim | بہترین برائے |
|---------|--------|-----------|----------|
| **طبیعیات کی درستگی** | اچھی | بہترین | Gazebo: فوری prototyping |
| **بصری حقیقت** | بنیادی | حقیقت پسندانہ | Isaac: Vision AI training |
| **سینسر کی درستگی** | تخمینہ | Ray-traced | Isaac: Sim-to-real deployment |
| **GPU acceleration** | محدود | مکمل | Isaac: متوازی سمولیشن |

**تجویز**:
- ابتدائی ترقی کے لیے **Gazebo** استعمال کریں
- جب حقیقی سینسرز، AI training، یا GPU acceleration کی ضرورت ہو تو **Isaac Sim** استعمال کریں

---

**شروع کرنے کے لیے تیار؟** Omniverse اور حقیقی سینسر ماڈلز کے بارے میں جاننے کے لیے [Isaac Sim: حقیقت پسندانہ سمولیشن](./isaac-sim.md) پر جاری رکھیں۔

## حوالہ جات

NVIDIA. (2024). *Isaac Sim Documentation*. https://docs.omniverse.nvidia.com/isaacsim/latest/

NVIDIA. (2024). *Isaac ROS Documentation*. https://nvidia-isaac-ros.github.io/
