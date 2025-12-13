---
title: LLM سے چلنے والی Action Planning
sidebar_position: 2
description: روبوٹ کاموں میں قدرتی زبان کو توڑنے کے لیے Large Language Models استعمال کریں
---

# LLM سے چلنے والی Action Planning

## تعارف

**Large Language Models (LLMs)** جیسے GPT-4، Claude، اور Llama انسانی کمانڈز کو عملی روبوٹ اعمال میں تبدیل کر سکتے ہیں۔ LLM کو ایک "task planner" کے طور پر سوچیں جو high-level ارادوں کو قابل عمل اقدامات میں توڑتا ہے۔

## کیسے کام کرتا ہے

### 1. صارف کمانڈ
```
"میز سے لال کپ لاؤ"
```

### 2. LLM تقسیم
```python
[
  {"action": "navigate_to", "params": {"location": "table"}},
  {"action": "detect_object", "params": {"description": "red cup"}},
  {"action": "grasp_object", "params": {"object_id": "detected_cup"}},
  {"action": "navigate_to", "params": {"location": "user"}}
]
```

## Prompt Engineering

```python
SYSTEM_PROMPT = """
آپ ایک انسان نما روبوٹ کے لیے task decomposition expert ہیں۔

دستیاب Actions:
1. navigate_to(location: str)
2. detect_object(description: str)
3. grasp_object(object_id: str)
4. place_object(location: str)
"""
```

## فوائد

✅ **لچک**: نئے کام programming کے بغیر
✅ **قدرتی interface**: آسان زبان میں
✅ **ذہین تقسیم**: پیچیدہ کام

---

**جاری رکھیں**: [Whisper تقریر کی شناخت](./whisper.md)

## حوالہ جات

Ahn, M., et al. (2022). Do As I Can, Not As I Say. *arXiv:2204.01691*.
