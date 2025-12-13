---
title: ماڈیول 4 - Vision-Language-Action
sidebar_position: 1
description: LLMs اور کثیر الوجوہی ادراک استعمال کرتے ہوئے قدرتی زبان کی کمانڈز کو سمجھنے اور عمل میں لانے والے خودکار انسان نما نظام بنائیں
---

# ماڈیول 4: Vision-Language-Action Integration

## تعارف

آپ نے ROS 2، سمولیشن، اور AI سے چلنے والی ادراک میں مہارت حاصل کر لی ہے۔ اب حتمی integration کا وقت ہے: **Vision-Language-Action (VLA)** نظام جو انسان نما روبوٹوں کو قدرتی زبان کی کمانڈز سمجھنے اور خودکار طور پر پیچیدہ کام عمل میں لانے کے قابل بناتے ہیں۔

تصور کریں اپنے انسان نما روبوٹ کو بتا رہے ہیں:
- "مجھے پانی کا گلاس لاؤ"
- "میز صاف کرو"
- "میرے پیچھے kitchen تک آؤ"

اور اسے بغیر رکاوٹ ماحول کو سمجھتے، اعمال کی ترتیب plan کرتے، اور انہیں execute کرتے دیکھنا—سب کچھ ایک سادہ بولی گئی کمانڈ سے۔ یہ VLA نظاموں کا وعدہ ہے۔

## Vision-Language-Action کیا ہے؟

**VLA** تین modalities کو یکجا کرتا ہے cognitive طور پر قابل روبوٹ بنانے کے لیے:

1. **Vision**: ماحول کو دیکھیں اور سمجھیں (کیمرے، depth سینسرز)
2. **Language**: انسانی کمانڈز اور ارادوں کو سمجھیں (تقریر کی شناخت، LLMs)
3. **Action**: حقیقی دنیا میں جسمانی کام execute کریں (navigation، manipulation)

### VLA انقلاب

روایتی روبوٹوں کو صریح programming کی ضرورت ہوتی ہے:
```python
# روایتی طریقہ
robot.navigate_to(x=5.0, y=2.0)
robot.detect_object("cup")
robot.grasp(object_id="cup_123")
```

VLA روبوٹ اعلیٰ سطح کا ارادہ سمجھتے ہیں:
```python
# VLA طریقہ
robot.execute_command("مجھے میز سے کپ لاؤ")
# روبوٹ خود سمجھتا ہے: navigate → detect → grasp → return
```

## انسان نما روبوٹوں کے لیے VLA کیوں؟

### قدرتی انسان-روبوٹ Interaction

انسان زبان کے ذریعے بات چیت کرتے ہیں، coordinate systems کے ذریعے نہیں:
- "kitchen میں جاؤ" (نہ کہ "x=10.5، y=3.2 تک navigate کرو")
- "لال کپ اٹھاؤ" (نہ کہ "object_id=cup_1234 grasp کرو")

### کام کی Generalization

VLA نظام نئے کاموں کو دوبارہ programming کے بغیر handle کر سکتے ہیں:
```
Training پر: "مجھے کپ لاؤ"، "مجھے کتاب لاؤ"
Generalizes: "مجھے stapler لاؤ" ← پہلے کبھی نہیں دیکھا، پھر بھی کام کرتا ہے!
```

## ماڈیول کا ڈھانچہ

### 1. [LLM سے چلنے والی Action Planning](./llm-planning.md)
جانیں کہ Large Language Models قدرتی زبان کو روبوٹ actions میں کیسے توڑتے ہیں:
- روبوٹ کاموں کے لیے prompt engineering
- کام کی تقسیم کی حکمت عملی
- روبوٹ صلاحیتوں میں زبان grounding کرنا

### 2. [Whisper تقریر کی شناخت](./whisper.md)
مضبوط speech-to-text کے لیے OpenAI Whisper کو integrate کریں:
- Whisper architecture اور صلاحیتیں
- Real-time transcription کے لیے ROS 2 integration
- شور والے ماحول کو handle کرنا

### 3. [کثیر الوجوہی ادراک](./multimodal.md)
بصری ادراک اور روبوٹ state میں زبان ground کریں:
- Vision-language models (CLIP، OWL-ViT)
- Referring expressions ground کرنا
- بصارت، زبان، اور state کو یکجا کرنا

### 4. [Capstone: خودکار انسان نما Architecture](./architecture.md)
تمام اجزاء کو integrate کرتے ہوئے مکمل VLA نظام ڈیزائن کریں:
- End-to-end VLA pipeline
- ROS 2 node architecture
- قدرتی زبان کی تقسیم کے ساتھ تین مکمل مثالیں

## پیشگی تقاضے

اس ماڈیول کو شروع کرنے سے پہلے، آپ کو سمجھنا چاہیے:

- **ROS 2 بنیادی باتیں** (ماڈیول 1): Topics، services، actions
- **سمولیشن** (ماڈیول 2): VLA نظاموں کو virtually test کرنا
- **AI ادراک** (ماڈیول 3): Visual SLAM، object detection، navigation
- **بنیادی AI تصورات**: Neural networks، language models

## VLA Technology Stack

### بنیادی اجزاء

VLA نظام استعمال کرتے ہیں:

**Speech Recognition**:
- **Whisper**: OpenAI کا مضبوط multilingual ASR
- **Google Speech-to-Text**: Cloud پر مبنی

**Large Language Models**:
- **GPT-4**: تجارتی، اعلیٰ صلاحیت
- **Claude 3**: مضبوط استدلال
- **Llama 3**: Open-source

**Vision-Language Models**:
- **CLIP**: تصاویر اور متن align کریں
- **OWL-ViT**: Open-vocabulary object detection
- **SAM**: Universal image segmentation

## حقیقی دنیا میں VLA اطلاقات

### گھر میں مدد
```
صارف: "bathroom سے میری دوائی لاؤ"
روبوٹ:
  1. Bathroom تک navigate کریں (SLAM + Nav2)
  2. دوائی کی بوتل detect کریں
  3. بوتل grasp کریں
  4. صارف کے پاس واپس navigate کریں
  5. دوائی hand over کریں
```

### Warehouse Logistics
```
صارف: "تمام لال boxes کو Zone B میں منتقل کرو"
روبوٹ:
  1. تمام لال boxes detect کریں
  2. ہر box کے لیے:
     a. Box تک navigate کریں
     b. Grasp اور lift کریں
     c. Zone B تک navigate کریں
     d. نیچے رکھیں
```

---

**شروع کرنے کے لیے تیار؟** جانیں کہ language models کیسے کاموں کو توڑتے ہیں [LLM سے چلنے والی Action Planning](./llm-planning.md) پر جاری رکھیں۔

## حوالہ جات

OpenAI. (2024). *Whisper*. https://github.com/openai/whisper

Ahn, M., et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. *arXiv:2204.01691*.
