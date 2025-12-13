---
title: کثیر الوجوہی ادراک
sidebar_position: 4
description: بصری ادراک اور روبوٹ state میں زبان ground کریں
---

# کثیر الوجوہی ادراک

## تعارف

**کثیر الوجوہی ادراک** vision، language، اور روبوٹ state کو یکجا کرتی ہے۔

## Vision-Language Models

### CLIP

```python
import clip

model, preprocess = clip.load("ViT-B/32")
image = preprocess(Image.open("scene.jpg"))
text = clip.tokenize(["ایک لال کپ"])

similarity = (image_features @ text_features.T)
```

### OWL-ViT

```python
from transformers import OwlViTProcessor

processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
texts = [["ایک لال کپ"]]
outputs = model(text=texts, images=image)
```

## Grounding Pipeline

```python
class MultimodalGrounding:
    def ground_command(self, command, image):
        object_desc = extract_object(command)
        boxes = self.owlvit.detect(image, object_desc)
        return boxes[0]
```

## فوائد

✅ **Open-vocabulary**
✅ **لچکدار**
✅ **درست**

---

**جاری رکھیں**: [Capstone: Architecture](./architecture.md)

## حوالہ جات

Radford, A., et al. (2021). Learning Transferable Visual Models. *arXiv:2103.00020*.
