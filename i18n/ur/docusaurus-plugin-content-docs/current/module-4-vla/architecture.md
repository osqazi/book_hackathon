---
title: خودکار انسان نما Architecture
sidebar_position: 5
description: مکمل Vision-Language-Action نظام کے لیے end-to-end architecture
---

# Capstone: خودکار انسان نما Architecture

## تعارف

اب ہم تمام اجزاء کو ایک مکمل خودکار نظام میں integrate کرتے ہیں۔

## مکمل VLA Pipeline

```
انسانی آواز → Whisper → LLM → Grounding → Actions
```

## System Architecture

```python
class AutonomousHumanoid:
    def __init__(self):
        self.whisper = WhisperNode()
        self.llm = LLMPlanner()
        self.vision = MultimodalGrounding()

    def execute_command(self, audio):
        text = self.whisper.transcribe(audio)
        plan = self.llm.decompose(text)

        for step in plan:
            self.execute_action(step)
```

## مثال: "میز صاف کرو"

```
صارف: "میز صاف کرو"
   ↓
LLM Plan:
  1. navigate_to("table")
  2. detect_objects("clutter")
  3. grasp_and_store(each_object)
   ↓
Execution: روبوٹ عمل میں لاتا ہے
```

## ROS 2 Integration

```python
class VLAMainNode(Node):
    def __init__(self):
        super().__init__('vla_main')

        self.voice_sub = self.create_subscription(
            Audio, '/microphone', self.callback, 10
        )
```

## خلاصہ

مبارک ہو! آپ نے مکمل stack مکمل کر لیا:
1. ROS 2
2. سمولیشن
3. AI ادراک
4. Vision-Language-Action

---

**اگلے اقدامات**: حقیقی ہارڈ ویئر پر deploy کریں!

## حوالہ جات

Driess, D., et al. (2023). PaLM-E. *arXiv:2303.03378*.
