---
title: مصنوعی ڈیٹا جنریشن
sidebar_position: 5
description: AI model training کے لیے Isaac Sim میں مصنوعی ڈیٹا بنائیں
---

# مصنوعی ڈیٹا جنریشن

## تعارف

**مصنوعی ڈیٹا** سمولیشن میں بنایا گیا labeled training ڈیٹا ہے۔ Isaac Sim کے ساتھ، آپ حقیقی دنیا کا ڈیٹا جمع کیے بغیر vision models کی تربیت کے لیے ہزاروں labeled تصاویر بنا سکتے ہیں۔

## Domain Randomization

ماحولیاتی parameters vary کریں robustness کے لیے:

### 1. روشنی Variation
```python
# مختلف روشنی کی حالتیں
light_intensity = random.uniform(0.2, 2.0)
light_color = random_color()
```

### 2. Texture Randomization
```python
# چیزوں پر random textures
texture = random.choice(texture_library)
apply_texture(object, texture)
```

### 3. چیز کی جگہ
```python
# Random positions اور orientations
position = random_position(workspace)
rotation = random_rotation()
spawn_object(type, position, rotation)
```

## خودکار Labeling

Isaac Sim خودکار طور پر فراہم کرتا ہے:
- **Bounding boxes**: چیز detection کے لیے
- **Segmentation masks**: instance segmentation کے لیے
- **Depth maps**: 3D reconstruction کے لیے
- **Pose annotations**: 6D pose estimation کے لیے

## مثال: تصویری Dataset بنائیں

```python
from omni.isaac.synthetic_utils import SyntheticDataHelper

# Synthetic data helper setup کریں
sd_helper = SyntheticDataHelper()

# 1000 مختلف scenes generate کریں
for i in range(1000):
    # ماحول randomize کریں
    randomize_lighting()
    randomize_object_poses()
    randomize_textures()
    
    # تصویر اور labels capture کریں
    rgb = sd_helper.get_rgb()
    bbox = sd_helper.get_bounding_boxes()
    
    # محفوظ کریں
    save_image(rgb, f"image_{i}.png")
    save_labels(bbox, f"labels_{i}.json")
```

## Sim-to-Real منتقلی

مصنوعی ڈیٹا پر train کیے گئے models حقیقی روبوٹوں پر deploy کریں:

1. **سمولیشن میں Train کریں**: ہزاروں varied examples
2. **Real ڈیٹا پر Fine-tune کریں**: کچھ حقیقی تصاویر کے ساتھ
3. **حقیقی روبوٹ پر Deploy کریں**: بہتر generalization

## فوائد

✅ **Scalability**: لامحدود ڈیٹا بنائیں
✅ **لاگت موثر**: حقیقی ڈیٹا جمع کرنے کی کوئی ضرورت نہیں
✅ **کامل labels**: کوئی انسانی labeling errors نہیں
✅ **Edge cases**: نایاب scenarios آسانی سے بنائیں

---

**اگلے اقدامات**: مبارک ہو! آپ نے AI سے چلنے والی ادراک مکمل کر لی۔ [ماڈیول 4: Vision-Language-Action](../module-4-vla/index.md) پر جائیں قدرتی زبان کے ساتھ intelligent automation کے لیے۔

## حوالہ جات

NVIDIA. (2024). *Isaac Sim Synthetic Data Generation*. https://docs.omniverse.nvidia.com/isaacsim/latest/
