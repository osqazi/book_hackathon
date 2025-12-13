---
title: Whisper تقریر کی شناخت
sidebar_position: 3
description: مضبوط speech-to-text کے لیے OpenAI Whisper کو ROS 2 کے ساتھ integrate کریں
---

# Whisper: تقریر کی شناخت

## تعارف

**OpenAI Whisper** ایک state-of-the-art تقریر کی شناخت ماڈل ہے جو متعدد زبانوں اور شور والے ماحول کو سنبھالتا ہے۔

## اہم خصوصیات

### 1. Multilingual Support
- 99 زبانیں
- خودکار زبان detection
- اردو، انگریزی، عربی

### 2. شور کے خلاف مضبوطی
- پس منظر کے شور
- متعدد بولنے والے
- مختلف accents

## ROS 2 Integration

```python
import whisper

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.model = whisper.load_model("base")

    def listen(self):
        audio_data = self.stream.read(16000)
        result = self.model.transcribe(audio_data)
        return result['text']
```

## فوائد

✅ **اعلیٰ درستگی**
✅ **کوئی training نہیں**
✅ **آسان integration**

---

**جاری رکھیں**: [کثیر الوجوہی ادراک](./multimodal.md)

## حوالہ جات

Radford, A., et al. (2022). Robust Speech Recognition. *arXiv:2212.04356*.
