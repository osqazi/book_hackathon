---
title: Multimodal Perception
sidebar_position: 4
description: Ground natural language commands in visual perception using vision-language models
---

# Multimodal Perception

## Introduction

Language alone isn't enough for robots. When a user says "pick up the red cup on the left," the robot must:
1. **See** the environment (vision)
2. **Understand** the command (language)
3. **Connect** language to visual objects (grounding)
4. **Reason** about spatial relationships (left, on, near)

**Multimodal perception** bridges language and vision, enabling robots to understand referring expressions and execute visually-grounded commands.

## The Grounding Problem

### What is Grounding?

**Grounding** maps linguistic references to physical entities in the world.

**Example:**
```
Command: "Pick up the red cup"

Vision: [Object 1: Blue plate], [Object 2: Red cup], [Object 3: Green book]

Grounding: "red cup" → Object 2 → Bounding box [320, 240, 150, 200]

Action: grasp(object_id=2, bbox=[320, 240, 150, 200])
```

Without grounding, the LLM knows what a "red cup" is conceptually but can't locate it in the camera image.

### Challenges

**1. Referring Expressions**
- "The cup" (which cup? there are 3)
- "The cup on the left" (left relative to robot or user?)
- "The big red cup" (how big is "big"?)
- "My cup" (requires tracking ownership)

**2. Partial Observability**
- Objects occluded by other objects
- Object partially out of frame
- Object behind the robot (not visible)

**3. Ambiguity**
- "Pick up the book" when there are 5 books on the table
- "The red one" when there are 3 red objects

**Solution:** Vision-language models that learn joint representations of images and text.

## Vision-Language Models

### CLIP: Contrastive Language-Image Pre-training

**CLIP** learns a shared embedding space where similar images and text have similar representations.

**Training:**
```
Image: [Photo of a red cup]
Text: "A red ceramic cup on a table"

CLIP learns: embed(image) ≈ embed(text)
```

**Usage for Robotics:**
```python
import torch
import clip
from PIL import Image

# Load CLIP model
model, preprocess = clip.load("ViT-B/32", device="cuda")

# Image from robot camera
image = Image.open("robot_view.jpg")
image_input = preprocess(image).unsqueeze(0).to("cuda")

# Possible objects
text_queries = ["a red cup", "a blue plate", "a green book"]
text_inputs = clip.tokenize(text_queries).to("cuda")

# Compute similarities
with torch.no_grad():
    image_features = model.encode_image(image_input)
    text_features = model.encode_text(text_inputs)

    # Cosine similarity
    similarity = (image_features @ text_features.T).softmax(dim=-1)

print(similarity)
# Output: [[0.85, 0.10, 0.05]]  # 85% confident it's a red cup
```

**Application:** Zero-shot object classification without training on robotics data.

### OWL-ViT: Open-Vocabulary Object Detection

**OWL-ViT** extends CLIP to detect objects described by text queries.

**Query:**
```python
from transformers import OwlViTProcessor, OwlViTForObjectDetection

processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")

image = Image.open("robot_view.jpg")
text_queries = ["a red cup", "a blue plate"]

inputs = processor(text=text_queries, images=image, return_tensors="pt")
outputs = model(**inputs)

# Get bounding boxes
boxes = outputs.pred_boxes[0].detach().cpu().numpy()
scores = outputs.logits[0].softmax(-1).max(-1).values.detach().cpu().numpy()

for query, box, score in zip(text_queries, boxes, scores):
    if score > 0.3:
        print(f"{query}: {box} (confidence: {score:.2f})")

# Output:
# a red cup: [0.45, 0.32, 0.15, 0.20] (confidence: 0.87)
```

**Application:** Detect objects from natural language descriptions without retraining.

### SAM: Segment Anything Model

**SAM** generates pixel-perfect segmentation masks for objects.

**Usage:**
```python
from segment_anything import SamPredictor, sam_model_registry

sam = sam_model_registry["vit_h"](checkpoint="sam_vit_h.pth")
predictor = SamPredictor(sam)

# Set image
image = cv2.imread("robot_view.jpg")
predictor.set_image(image)

# Provide bounding box prompt (from OWL-ViT)
input_box = np.array([320, 240, 470, 440])  # [x1, y1, x2, y2]

masks, scores, _ = predictor.predict(box=input_box)

# masks[0] is binary mask for the object
segmentation_mask = masks[0]
```

**Application:** Precise object boundaries for grasp planning and collision avoidance.

## ROS 2 Multimodal Pipeline

### Architecture

```mermaid
graph TB
    A[Camera Image] --> B[Object Detection<br/>OWL-ViT]
    C[Voice Command<br/>"Pick up red cup"] --> D[LLM Planner<br/>Decompose task]

    D --> E[Grounding Node<br/>Match language to vision]
    B --> E

    E --> F[Segmentation<br/>SAM]
    F --> G[Grasp Planner<br/>Compute gripper pose]

    G --> H[Robot Executor<br/>Execute grasp]

    style A fill:#e1f5ff
    style C fill:#e1f5ff
    style B fill:#fff4e1
    style D fill:#fff4e1
    style E fill:#ffe1e1
    style F fill:#e1ffe1
    style G fill:#f0e1ff
    style H fill:#76b900
```

### Grounding Node Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import torch
from transformers import OwlViTProcessor, OwlViTForObjectDetection

class GroundingNode(Node):
    def __init__(self):
        super().__init__('grounding_node')

        # Load OWL-ViT
        self.processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
        self.model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Subscribe to grounding queries
        self.query_sub = self.create_subscription(
            String, '/grounding/query', self.query_callback, 10
        )

        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/grounding/detections', 10
        )

        self.bridge = CvBridge()
        self.latest_image = None

    def image_callback(self, msg):
        # Store latest camera frame
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def query_callback(self, msg):
        query = msg.data
        self.get_logger().info(f"Grounding query: {query}")

        if self.latest_image is None:
            self.get_logger().warn("No camera image available")
            return

        # Detect objects matching query
        detections = self.ground_query(query, self.latest_image)

        # Publish detections
        detection_msg = Detection2DArray()
        detection_msg.detections = detections
        self.detection_pub.publish(detection_msg)

    def ground_query(self, query, image):
        # Run OWL-ViT
        inputs = self.processor(text=[query], images=image, return_tensors="pt")
        outputs = self.model(**inputs)

        # Extract boxes and scores
        boxes = outputs.pred_boxes[0].detach().cpu().numpy()
        scores = outputs.logits[0].softmax(-1).max(-1).values.detach().cpu().numpy()

        detections = []
        for box, score in zip(boxes, scores):
            if score > 0.3:
                detection = Detection2D()
                detection.bbox.center.x = box[0]
                detection.bbox.center.y = box[1]
                detection.bbox.size_x = box[2]
                detection.bbox.size_y = box[3]
                detection.results[0].score = float(score)
                detections.append(detection)

        return detections
```

### LLM Integration with Vision

```python
class MultimodalPlanner(Node):
    def __init__(self):
        super().__init__('multimodal_planner')

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String, '/voice/command', self.command_callback, 10
        )

        # Grounding query publisher
        self.grounding_pub = self.create_publisher(
            String, '/grounding/query', 10
        )

        # Detection subscriber
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/grounding/detections', self.detection_callback, 10
        )

        self.llm = LLMClient()
        self.current_detections = None

    def command_callback(self, msg):
        command = msg.data  # "Pick up the red cup"

        # Ask LLM to extract grounding query
        prompt = f"""
        User command: "{command}"

        Extract the object to be detected. Return only the object description.

        Example:
        Command: "Pick up the red cup"
        Object: "a red cup"

        Command: "Navigate to the table"
        Object: "a table"

        Command: "{command}"
        Object:
        """

        grounding_query = self.llm.generate(prompt).strip()
        self.get_logger().info(f"Grounding query: {grounding_query}")

        # Request grounding
        self.grounding_pub.publish(String(data=grounding_query))

    def detection_callback(self, msg):
        self.current_detections = msg.detections

        if len(self.current_detections) == 0:
            self.get_logger().warn("No objects detected")
            return

        # Select best detection
        best_detection = max(self.current_detections, key=lambda d: d.results[0].score)

        self.get_logger().info(f"Grounded object at: {best_detection.bbox}")

        # Execute grasp action
        self.execute_grasp(best_detection.bbox)
```

## Spatial Reasoning

### Handling Referring Expressions

**Left/Right:**
```python
def resolve_spatial_reference(detections, reference):
    """
    detections: List of bounding boxes
    reference: "left", "right", "center"
    """
    if reference == "left":
        # Sort by x-coordinate, return leftmost
        return min(detections, key=lambda d: d.bbox.center.x)
    elif reference == "right":
        return max(detections, key=lambda d: d.bbox.center.x)
    elif reference == "center":
        image_center_x = 640  # Image width / 2
        return min(detections, key=lambda d: abs(d.bbox.center.x - image_center_x))
```

**Size Comparisons:**
```python
def resolve_size_reference(detections, reference):
    """
    reference: "big", "small", "biggest", "smallest"
    """
    if reference in ["big", "biggest"]:
        return max(detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
    elif reference in ["small", "smallest"]:
        return min(detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
```

**Distance from Robot:**
```python
def resolve_distance_reference(detections, depth_map, reference):
    """
    reference: "closest", "farthest"
    """
    distances = []
    for det in detections:
        # Get depth at object center
        x, y = int(det.bbox.center.x), int(det.bbox.center.y)
        distance = depth_map[y, x]
        distances.append(distance)

    if reference == "closest":
        idx = np.argmin(distances)
    elif reference == "farthest":
        idx = np.argmax(distances)

    return detections[idx]
```

### Combining Constraints

```python
command = "Pick up the small red cup on the left"

# Parse command
attributes = parse_command(command)
# {"color": "red", "size": "small", "position": "left", "object": "cup"}

# Detect all cups
all_cups = detect_objects("a cup")

# Filter by color
red_cups = filter_by_text(all_cups, "red cup")

# Filter by size
small_red_cups = resolve_size_reference(red_cups, "small")

# Filter by position
target_cup = resolve_spatial_reference([small_red_cups], "left")

# Execute grasp
execute_grasp(target_cup)
```

## State Estimation and Context

### Tracking Objects Over Time

```python
class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        self.tracked_objects = {}  # {object_id: DetectionHistory}

    def update(self, new_detections):
        for det in new_detections:
            # Match to existing object or create new
            object_id = self.match_detection(det)

            if object_id is None:
                # New object
                object_id = self.create_new_track()

            # Update track
            self.tracked_objects[object_id].add_detection(det)

    def match_detection(self, det):
        # Simple IOU-based matching
        for obj_id, track in self.tracked_objects.items():
            if iou(det.bbox, track.latest_bbox) > 0.5:
                return obj_id
        return None
```

**Application:** Resolve "it", "that", "the same object" references.

### Contextual Understanding

```python
# User says: "Pick up the cup"
# Robot sees 3 cups

# Use context to disambiguate:
context = {
    "last_mentioned_object": "red cup",
    "user_gaze_direction": [0.5, 0.3],  # User looking at center-left
    "recent_actions": ["navigate to table"]
}

# Prioritize object in user's gaze direction
def select_object_with_context(detections, context):
    gaze_x, gaze_y = context["user_gaze_direction"]

    # Find object closest to gaze
    best_match = min(detections, key=lambda d:
        distance([d.bbox.center.x, d.bbox.center.y], [gaze_x, gaze_y])
    )

    return best_match
```

## Complete Example: "Bring me the red cup"

### Step 1: Voice Command

```
User: "Bring me the red cup"
Whisper → Text: "bring me the red cup"
```

### Step 2: LLM Decomposition

```python
llm_plan = [
    {"action": "detect", "query": "a red cup"},
    {"action": "navigate", "target": "detected_object"},
    {"action": "grasp", "object": "detected_object"},
    {"action": "navigate", "target": "user"},
    {"action": "place", "location": "table"}
]
```

### Step 3: Visual Grounding

```python
# Execute detect action
grounding_query = "a red cup"
detections = owl_vit_detect(camera_image, grounding_query)

# detections[0]: bbox=[320, 240, 150, 200], score=0.89
```

### Step 4: Segmentation

```python
# Get precise mask for grasp planning
mask = sam_segment(camera_image, bbox=detections[0].bbox)
```

### Step 5: Grasp Planning

```python
# Compute 3D grasp pose from mask and depth
depth = get_depth_at_bbox(detections[0].bbox)
grasp_pose = compute_grasp_pose(mask, depth)
```

### Step 6: Execution

```python
# Navigate to object
navigate(grasp_pose.position)

# Grasp
grasp(grasp_pose)

# Navigate to user
navigate(user_position)

# Place
place(table_position)
```

## Summary

Multimodal perception enables robots to understand visually-grounded commands:

**Key Technologies:**
- **CLIP**: Zero-shot image-text similarity
- **OWL-ViT**: Open-vocabulary object detection
- **SAM**: Pixel-perfect segmentation masks

**Grounding Pipeline:**
1. Parse natural language command
2. Extract object description
3. Detect object in camera image
4. Segment object for grasp planning
5. Execute action with 3D pose

**Challenges:**
- Ambiguous referring expressions ("the cup" → which cup?)
- Partial observability (occluded objects)
- Spatial reasoning (left/right, near/far)

**Solutions:**
- Multi-constraint filtering (color + size + position)
- Context tracking (user gaze, recent mentions)
- Object tracking over time

**Next:** Integrate everything into a complete VLA architecture.

---

**Continue to**: [Capstone: Autonomous Humanoid Architecture](./architecture.md)

## References

Radford, A., et al. (2021). Learning Transferable Visual Models From Natural Language Supervision. *arXiv preprint* arXiv:2103.00020.

Minderer, M., et al. (2022). Simple Open-Vocabulary Object Detection with Vision Transformers. *arXiv preprint* arXiv:2205.06230.

Kirillov, A., et al. (2023). Segment Anything. *arXiv preprint* arXiv:2304.02643.
