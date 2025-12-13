---
title: ماڈیول 2 - سمولیشن اور Digital Twins
sidebar_position: 1
description: محفوظ انسان نما روبوٹ ترقی کے لیے Gazebo کے ساتھ طبیعیات سمولیشن اور Unity کے ساتھ اعلیٰ معیار کی visualization میں مہارت حاصل کریں
---

# ماڈیول 2: سمولیشن اور Digital Twins

## تعارف

تصور کریں کہ حقیقی ہارڈ ویئر پر انسان نما چلنے کے الگورتھم کو test کر رہے ہیں—پھر اپنے مہنگے روبوٹ کو گرتے ہوئے دیکھ رہے ہیں کیونکہ آپ نے center of mass کی غلط calculation کی۔ یا ایک vision system deploy کر رہے ہیں جو ناکام ہو جاتا ہے کیونکہ آپ نے اسے مختلف روشنی کی حالتوں میں test نہیں کیا۔ حقیقی دنیا میں testing مہنگی، وقت طلب، اور ممکنہ طور پر خطرناک ہے۔

**سمولیشن** اس مسئلے کو حل کرتا ہے virtual ماحول بنا کر جہاں آپ physical ہارڈ ویئر کو چھونے سے پہلے روبوٹ کے رویے کو test، iterate، اور validate کر سکتے ہیں۔ یہ ماڈیول **Digital Twins** متعارف کراتا ہے—روبوٹوں کی virtual replicas جو محفوظ، موثر ترقی کو قابل بناتی ہیں۔

## آپ کیا سیکھیں گے

ماڈیول 1 کی ROS 2 بنیادی باتوں پر تعمیر کرتے ہوئے، یہ ماڈیول تلاش کرتا ہے:

- **طبیعیات سمولیشن**: کیسے gravity، inertia، friction، اور collisions انسان نما روبوٹ کے رویے کو متاثر کرتے ہیں
- **سینسر ماڈلنگ**: ادراک testing کے لیے LiDAR، depth کیمرے، IMUs، اور RGB کیمرے simulate کرنا
- **Gazebo Integration**: طبیعیات پر مبنی سمولیشن ماحول میں URDF ماڈلز load کرنا
- **Unity Visualization**: حقیقت پسندانہ testing کے لیے اعلیٰ معیار کی 3D rendering
- **Digital Twin ورک فلوز**: deployment سے پہلے پیچیدہ رویوں (چلنا، پکڑنا، navigation) کو test کرنا

## انسان نما روبوٹکس کے لیے سمولیشن کیوں؟

انسان نما روبوٹ منفرد چیلنجز پیش کرتے ہیں جو سمولیشن کو ضروری بناتے ہیں:

### 1. حفاظت اور لاگت
- **ہارڈ ویئر نقصان سے بچیں**: گرنے کا خطرہ مول لینے سے پہلے غیر مستحکم چلنے کی gaits virtually test کریں
- **تیزی سے iterate کریں**: ہارڈ ویئر سیٹ اپ کے گھنٹوں کے بجائے سیکنڈوں میں الگورتھم میں تبدیلیاں کریں
- **لاگت کم کریں**: متعدد physical روبوٹوں یا مہنگے سینسر arrays کی ضرورت نہیں

### 2. دہرائی جانے کی صلاحیت
- **Deterministic testing**: یکساں ابتدائی حالات کے ساتھ ایک ہی scenario سینکڑوں بار چلائیں
- **Edge case کی تلاش**: physical خطرے کے بغیر نایاب scenarios (پھسلنے والے فرش، سینسر کی ناکامیاں) test کریں
- **Regression testing**: تصدیق کریں کہ نیا کوڈ موجودہ رویوں کو نہیں توڑتا

### 3. توسیع پذیری
- **متوازی testing**: cloud infrastructure پر بیک وقت متعدد سمولیشن چلائیں
- **متنوع ماحول**: physical سیٹ اپ کے بغیر factories، گھروں، outdoor terrains میں test کریں
- **سینسر variations**: فوری طور پر کیمرے swap کریں، LiDAR شامل کریں، IMU noise ماڈلز تبدیل کریں

## Digital Twin تصور

ایک **Digital Twin** ایک physical system کی virtual replica ہے جو:

1. **ڈھانچے کی عکاسی کرتا ہے**: حقیقی روبوٹ کی طرح ایک ہی URDF تفصیل استعمال کرتا ہے
2. **طبیعیات simulate کرتا ہے**: gravity، collisions، joint dynamics، سینسر noise کو model کرتا ہے
3. **کمانڈز وصول کرتا ہے**: حقیقی ہارڈ ویئر کی طرح ایک ہی ROS 2 پیغامات قبول کرتا ہے
4. **فیڈبیک فراہم کرتا ہے**: حقیقی دنیا کے فارمیٹس کے مطابق سینسر ڈیٹا publish کرتا ہے

## ماڈیول کا ڈھانچہ

### 1. [طبیعیات کے اصول](./physics-principles.md)
وہ طبیعیات سمجھیں جو انسان نما سمولیشن کو چیلنجنگ بناتی ہے:
- دو پیروں پر چلنے کے لیے gravity اور balance
- متحرک حرکات میں inertia اور momentum
- پاؤں-زمین contact کے لیے friction ماڈلز
- محفوظ manipulation کے لیے collision detection

### 2. [سینسر سمولیشن](./sensors.md)
جانیں کہ روبوٹ ادراک کو کیسے model کریں:
- **LiDAR**: رکاوٹ detection اور mapping کے لیے laser range finding
- **Depth کیمرے**: 3D scene سمجھنے کے لیے RGB-D ڈیٹا
- **IMU**: balance اور orientation کے لیے accelerometer اور gyroscope
- **RGB کیمرے**: چیز کی شناخت کے لیے بصری ادراک

### 3. [Digital Twin ورک فلوز](./digital-twin.md)
حقیقی ترقیاتی scenarios پر سمولیشن لاگو کریں:
- Gazebo میں URDF ماڈلز load کرنا
- Visualization کے لیے Unity جوڑنا
- ہارڈ ویئر deployment سے پہلے چلنے کی gaits test کرنا
- سمولیشن میں grasping الگورتھم validate کرنا

## آگے کیا ہے؟

سمولیشن میں مہارت حاصل کرنے کے بعد، آپ تیار ہوں گے:

- **ماڈیول 3**: NVIDIA Isaac Sim کے ساتھ اعلیٰ ادراک (حقیقت پسندانہ rendering، مصنوعی ڈیٹا)
- **ماڈیول 4**: سمولیشن میں test کیے گئے Vision-Language-Action نظام

سمولیشن وہ جگہ ہے جہاں الگورتھم حقیقی ہارڈ ویئر کو چھونے سے پہلے پختہ ہوتے ہیں۔ آئیں یہ سمجھنے سے شروع کریں کہ طبیعیات انسان نما سمولیشن کو منفرد کیوں بناتی ہے!

---

**شروع کرنے کے لیے تیار؟** جانیں کہ gravity، inertia، اور collisions انسان نما روبوٹوں کو کیسے متاثر کرتے ہیں [طبیعیات کے اصول](./physics-principles.md) پر جاری رکھیں۔

## حوالہ جات

Open Robotics. (2024). *Gazebo Documentation*. https://gazebosim.org/docs

Unity Technologies. (2024). *Unity Robotics Hub*. https://github.com/Unity-Technologies/Unity-Robotics-Hub
