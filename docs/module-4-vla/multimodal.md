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

## Advanced Grounding Techniques

### Visual Question Answering (VQA) for Verification

After detecting an object, verify it matches the command using VQA:

```python
from transformers import ViltProcessor, ViltForQuestionAnswering

class GroundingVerifier(Node):
    def __init__(self):
        super().__init__('grounding_verifier')

        # Load VQA model
        self.vqa_processor = ViltProcessor.from_pretrained("dandelin/vilt-b32-finetuned-vqa")
        self.vqa_model = ViltForQuestionAnswering.from_pretrained("dandelin/vilt-b32-finetuned-vqa")

    def verify_detection(self, image, bbox, expected_object):
        """
        Verify that detected bounding box contains the expected object.

        Example:
        Command: "Pick up the red cup"
        Detected: bbox at [320, 240, 150, 200]
        Verify: Is this actually a red cup?
        """

        # Crop to bounding box
        x, y, w, h = bbox
        cropped_image = image[y:y+h, x:x+w]

        # Ask VQA model
        questions = [
            f"Is this a {expected_object}?",
            f"What color is this {expected_object.split()[-1]}?",  # "red cup" → "cup"
            "What object is this?"
        ]

        answers = []
        for question in questions:
            inputs = self.vqa_processor(cropped_image, question, return_tensors="pt")
            outputs = self.vqa_model(**inputs)
            logits = outputs.logits
            answer_idx = logits.argmax(-1).item()
            answer = self.vqa_model.config.id2label[answer_idx]
            answers.append(answer)

        # Verify answers match expectations
        if expected_object in answers[0].lower() and answers[0].lower() == "yes":
            self.get_logger().info(f"Verified: {expected_object}")
            return True
        else:
            self.get_logger().warn(f"Verification failed: expected '{expected_object}', but VQA says '{answers[2]}'")
            return False
```

**Application**: Reduces false positives from object detection, especially in cluttered environments.

### Scene Graphs for Complex Spatial Reasoning

Build a scene graph to understand object relationships:

```python
from scene_graph_benchmark.scene_parser import SceneParser

class SceneGraphGrounding(Node):
    def __init__(self):
        super().__init__('scene_graph_grounding')
        self.parser = SceneParser()

    def parse_scene(self, image):
        """
        Generate scene graph with object relationships.

        Scene Graph:
        - cup [on] table
        - book [next to] cup
        - person [behind] table
        """

        scene_graph = self.parser.parse(image)

        # Example output:
        # {
        #   "objects": [
        #     {"id": 0, "label": "cup", "bbox": [320, 240, 80, 120]},
        #     {"id": 1, "label": "table", "bbox": [100, 300, 600, 200]},
        #     {"id": 2, "label": "book", "bbox": [450, 250, 100, 80]}
        #   ],
        #   "relationships": [
        #     {"subject": 0, "predicate": "on", "object": 1},  # cup on table
        #     {"subject": 2, "predicate": "next_to", "object": 0}  # book next to cup
        #   ]
        # }

        return scene_graph

    def ground_with_relationships(self, command, scene_graph):
        """
        Ground referring expression using relationships.

        Command: "Pick up the cup on the table"
        → Find cup object
        → Verify it has "on" relationship with table
        """

        # Parse command for relationships
        if "on the table" in command:
            # Find table object
            table = next(o for o in scene_graph["objects"] if o["label"] == "table")

            # Find objects on table
            on_table_ids = [
                r["subject"]
                for r in scene_graph["relationships"]
                if r["predicate"] == "on" and r["object"] == table["id"]
            ]

            # Filter for cup
            cups_on_table = [
                o for o in scene_graph["objects"]
                if o["label"] == "cup" and o["id"] in on_table_ids
            ]

            return cups_on_table[0]  # Return first matching cup
```

### Handling Negations and Exclusions

Support commands like "pick up the cup that's NOT red":

```python
def ground_with_negation(command, image):
    """
    Handle negative constraints in commands.

    Examples:
    - "Pick up the cup that's not red"
    - "Bring me a book, but not the blue one"
    - "Navigate to any table except the one in the kitchen"
    """

    # Parse negation
    if "not" in command or "except" in command:
        # Extract base object and negated property
        if "not red" in command:
            base_object = "cup"
            excluded_property = "red"

            # Detect all cups
            all_cups = owl_vit.detect(image, "a cup")

            # Filter by color exclusion
            non_red_cups = []
            for cup in all_cups:
                # Check color
                color = clip.classify(
                    crop_bbox(image, cup.bbox),
                    labels=["red", "blue", "white", "black", "green"]
                )

                if color != "red":
                    non_red_cups.append(cup)

            return non_red_cups[0]  # Return first non-red cup
```

### Temporal Reasoning for Dynamic Scenes

Track objects across time for temporal commands:

```python
class TemporalGrounding(Node):
    def __init__(self):
        super().__init__('temporal_grounding')

        self.object_history = {}  # {object_id: [detection_t0, detection_t1, ...]}

    def update(self, detections, timestamp):
        """
        Track objects over time.

        Enables commands like:
        - "Pick up the cup that just moved"
        - "Bring me the object that was on the table a minute ago"
        - "Navigate to where you saw the person last"
        """

        for det in detections:
            obj_id = det.id

            if obj_id not in self.object_history:
                self.object_history[obj_id] = []

            self.object_history[obj_id].append({
                "timestamp": timestamp,
                "bbox": det.bbox,
                "position_3d": det.position_3d
            })

    def ground_temporal_query(self, query):
        """
        Query: "the cup that just moved"
        → Find object with highest recent position change
        """

        if "just moved" in query:
            # Calculate recent motion for all objects
            motion_scores = {}

            for obj_id, history in self.object_history.items():
                if len(history) < 2:
                    continue

                # Compare last 2 positions
                recent_positions = [h["position_3d"] for h in history[-5:]]
                motion = np.std(recent_positions, axis=0)  # Standard deviation
                motion_magnitude = np.linalg.norm(motion)

                motion_scores[obj_id] = motion_magnitude

            # Return object with most motion
            moving_object_id = max(motion_scores, key=motion_scores.get)
            return self.get_current_detection(moving_object_id)

        elif "was on the table" in query and "ago" in query:
            # Find object that was on table in past
            time_delta = parse_time_delta(query)  # "a minute ago" → 60 seconds
            target_timestamp = time.time() - time_delta

            # Search history for object on table at target time
            for obj_id, history in self.object_history.items():
                for h in history:
                    if abs(h["timestamp"] - target_timestamp) < 5:  # Within 5s
                        if self.was_on_table(h["position_3d"]):
                            return self.get_current_detection(obj_id)
```

### Compositional Grounding for Complex Descriptions

Handle multi-attribute descriptions:

```python
def ground_compositional(command, image):
    """
    Complex description: "the small red cup next to the blue plate on the left side of the table"

    Strategy:
    1. Parse into components: [small, red, cup, next_to, blue plate, left side, table]
    2. Ground each component independently
    3. Combine with logical AND
    """

    components = parse_description(command)
    # {
    #   "object": "cup",
    #   "attributes": ["small", "red"],
    #   "spatial": "left side of table",
    #   "relations": ["next to blue plate"]
    # }

    # Step 1: Detect all cups
    candidates = owl_vit.detect(image, "a cup")

    # Step 2: Filter by attributes
    for attr in components["attributes"]:
        if attr == "small":
            candidates = [c for c in candidates if is_small(c)]
        elif attr == "red":
            color_candidates = []
            for c in candidates:
                color = classify_color(crop_bbox(image, c.bbox))
                if color == "red":
                    color_candidates.append(c)
            candidates = color_candidates

    # Step 3: Filter by spatial constraints
    if "left side" in components["spatial"]:
        table = detect_table(image)
        table_center_x = table.bbox.center_x
        candidates = [c for c in candidates if c.bbox.center_x < table_center_x]

    # Step 4: Filter by relations
    if "next to blue plate" in components["relations"]:
        blue_plate = detect_blue_plate(image)
        candidates = [
            c for c in candidates
            if is_next_to(c.bbox, blue_plate.bbox, threshold=100)  # pixels
        ]

    # Return best match
    return candidates[0] if candidates else None
```

## Production Deployment Considerations

### Latency Optimization

Vision-language models can be slow. Optimize for real-time performance:

```python
class OptimizedGroundingPipeline(Node):
    def __init__(self):
        super().__init__('optimized_grounding')

        # Pre-load all models at startup
        self.owl_vit = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")
        self.owl_vit.to("cuda")  # Move to GPU
        self.owl_vit.eval()  # Inference mode

        self.sam = SamPredictor(sam_model_registry["vit_h"](checkpoint="sam_vit_h.pth"))

        # Cache recent detections (avoid recomputing)
        self.detection_cache = {}
        self.cache_ttl = 2.0  # seconds

    def detect_with_cache(self, image, query):
        """Use cached detections if available and recent"""

        cache_key = f"{hash_image(image)}_{query}"

        if cache_key in self.detection_cache:
            cached_detection, timestamp = self.detection_cache[cache_key]
            if time.time() - timestamp < self.cache_ttl:
                self.get_logger().info("Using cached detection")
                return cached_detection

        # Run detection
        detection = self.owl_vit.detect(image, query)
        self.detection_cache[cache_key] = (detection, time.time())

        return detection
```

**Performance Targets:**
- Object detection: &lt;200ms
- Segmentation: &lt;100ms
- Total grounding latency: &lt;500ms

### Failure Recovery

Handle grounding failures gracefully:

```python
class RobustGroundingNode(Node):
    def ground_with_retry(self, command, max_attempts=3):
        """Retry grounding with progressively relaxed constraints"""

        for attempt in range(max_attempts):
            try:
                # Attempt 1: Strict matching
                if attempt == 0:
                    result = self.ground_strict(command)

                # Attempt 2: Relaxed constraints
                elif attempt == 1:
                    self.get_logger().warn("Strict grounding failed, relaxing constraints")
                    result = self.ground_relaxed(command)

                # Attempt 3: Ask user for help
                else:
                    self.get_logger().error("Grounding failed, requesting user help")
                    self.speak("I can't find that object. Could you point to it or describe it differently?")
                    return None

                if result:
                    return result

            except Exception as e:
                self.get_logger().error(f"Grounding attempt {attempt+1} failed: {e}")

        return None

    def ground_relaxed(self, command):
        """Lower confidence thresholds and try multiple query phrasings"""

        # Try multiple phrasings
        queries = generate_query_variations(command)
        # "red cup" → ["a red cup", "red ceramic cup", "red mug", "cup that is red"]

        for query in queries:
            detections = self.owl_vit.detect(image, query, confidence_threshold=0.2)  # Lower threshold
            if detections:
                return detections[0]

        return None
```

## Summary

Multimodal perception enables robots to understand visually-grounded commands:

**Key Technologies:**
- **CLIP**: Zero-shot image-text similarity
- **OWL-ViT**: Open-vocabulary object detection
- **SAM**: Pixel-perfect segmentation masks
- **VQA**: Visual question answering for verification
- **Scene Graphs**: Understanding object relationships

**Grounding Pipeline:**
1. Parse natural language command
2. Extract object description and constraints
3. Detect object in camera image
4. Verify detection with VQA or secondary checks
5. Segment object for grasp planning
6. Execute action with 3D pose

**Advanced Techniques:**
- Compositional grounding for complex multi-attribute descriptions
- Temporal reasoning for dynamic scenes and motion
- Negation handling ("not red", "except")
- Scene graph reasoning for spatial relationships
- Multi-modal verification to reduce false positives

**Challenges:**
- Ambiguous referring expressions ("the cup" → which cup?)
- Partial observability (occluded objects)
- Spatial reasoning (left/right, near/far)
- Latency constraints for real-time interaction

**Solutions:**
- Multi-constraint filtering (color + size + position)
- Context tracking (user gaze, recent mentions)
- Object tracking over time
- Caching and model optimization for speed
- Graceful degradation with retry strategies

**Production Considerations:**
- Target &lt;500ms total grounding latency
- Cache recent detections
- Implement retry with relaxed constraints
- Always verify detections before execution

**Next:** Integrate everything into a complete VLA architecture.

---

**Continue to**: [Capstone: Autonomous Humanoid Architecture](./architecture.md)

## References

Radford, A., et al. (2021). Learning Transferable Visual Models From Natural Language Supervision. *arXiv preprint* arXiv:2103.00020.

Minderer, M., et al. (2022). Simple Open-Vocabulary Object Detection with Vision Transformers. *arXiv preprint* arXiv:2205.06230.

Kirillov, A., et al. (2023). Segment Anything. *arXiv preprint* arXiv:2304.02643.
