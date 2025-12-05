---
title: Synthetic Data Generation
sidebar_position: 5
description: Generate AI training datasets using Isaac Sim with domain randomization for sim-to-real transfer
---

# Synthetic Data Generation

## Introduction

Training AI models for robotics requires **massive labeled datasets**—millions of images with bounding boxes, segmentation masks, or depth maps. Collecting and labeling real-world data is expensive and time-consuming.

**Synthetic data** generated in simulation solves this problem: create photorealistic scenes in Isaac Sim, randomize parameters (lighting, textures, object positions), and automatically generate labels. Train models on synthetic data, then deploy to real robots.

This section covers synthetic data workflows, domain randomization techniques, and sim-to-real transfer strategies.

## Why Synthetic Data?

### Real-World Data Challenges

**Manual Labeling is Slow**:
```
1 person × 8 hours/day × 100 images/hour = 800 labeled images/day
Need 100,000 images → 125 days of work
```

**Expensive**:
- Annotation services: $0.10 - $1.00 per image
- 100,000 images = $10,000 - $100,000

**Limited Diversity**:
- Hard to capture rare scenarios (fire, flooding)
- Expensive to stage variations (100 lighting conditions)

### Synthetic Data Advantages

**Automatic Labeling**:
```
Isaac Sim: 1,000 images/hour with perfect labels (bounding boxes, segmentation, depth)
Need 100,000 images → 100 hours of GPU time (~$50 on cloud)
```

**Perfect Ground Truth**:
- Exact 3D positions
- Pixel-perfect segmentation
- Occlusion-aware labels

**Unlimited Diversity**:
- Randomize lighting, weather, backgrounds
- Generate rare scenarios easily
- Test edge cases systematically

## Domain Randomization

**Domain randomization** varies simulation parameters to create diverse training data that generalizes to real-world variations.

### What to Randomize?

**1. Lighting**
- Light intensity (100 - 5000 lux)
- Color temperature (warm/cool)
- Number and position of lights
- HDR environment maps

**2. Camera Parameters**
- Exposure, gain, white balance
- Lens distortion, chromatic aberration
- Motion blur (for dynamic scenes)
- Sensor noise

**3. Object Properties**
- Textures and materials
- Colors (hue, saturation, brightness)
- Positions and orientations
- Scales (within realistic bounds)

**4. Scene Composition**
- Background clutter
- Distractors (irrelevant objects)
- Number of target objects
- Occlusions

### Domain Randomization Example

```python
import random
import omni

def randomize_scene():
    # Randomize lighting
    dome_light = "/World/DomeLight"
    intensity = random.uniform(500, 3000)
    omni.kit.commands.execute("ChangePrimProperty",
        prim_path=dome_light,
        property="inputs:intensity",
        value=intensity
    )

    # Randomize object positions
    for obj in ["/World/Cup", "/World/Book", "/World/Pen"]:
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-0.5, 0.5)
        z = 1.0  # On table
        set_position(obj, [x, y, z])

    # Randomize textures
    materials = ["Wood", "Metal", "Plastic", "Ceramic"]
    apply_material("/World/Table", random.choice(materials))

    # Randomize camera
    camera = "/World/Humanoid/Camera"
    exposure = random.uniform(-2, 2)  # EV stops
    set_camera_exposure(camera, exposure)
```

Run this function 100,000 times → 100,000 diverse training images.

## Synthetic Data Pipeline in Isaac Sim

### Step 1: Create Base Scene

Build realistic environment:

```python
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid

world = World()

# Add table
table = VisualCuboid(
    prim_path="/World/Table",
    size=[1.0, 0.6, 0.9],
    position=[1.0, 0.0, 0.45],
    color=[0.7, 0.5, 0.3]  # Wood color
)

# Add target objects
cup = DynamicCuboid(
    prim_path="/World/Cup",
    size=[0.08, 0.08, 0.12],
    position=[1.0, 0.2, 1.0],
    color=[1.0, 1.0, 1.0]  # White
)
```

### Step 2: Configure Replicator

Isaac Sim includes **Omniverse Replicator** for data generation:

```python
import omni.replicator.core as rep

# Define camera
camera = rep.create.camera(position=(2, 0, 1.5), look_at="/World/Table")

# Configure rendering
render_product = rep.create.render_product(camera, (1280, 720))

# Enable annotators (labels)
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
bbox_2d = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
semantic_seg = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")

# Attach to camera
rgb.attach(render_product)
bbox_2d.attach(render_product)
semantic_seg.attach(render_product)
```

### Step 3: Randomization Graph

Define randomization logic:

```python
def randomize():
    with rep.new_layer():
        # Randomize light intensity
        lights = rep.get.prims(path_pattern="/World/.*Light")
        with lights:
            rep.modify.attribute("inputs:intensity", rep.distribution.uniform(1000, 5000))

        # Randomize object positions
        objects = rep.get.prims(path_pattern="/World/Cup|/World/Book")
        with objects:
            rep.modify.pose(
                position=rep.distribution.uniform((-1, -0.5, 1.0), (1, 0.5, 1.0)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
            )

        # Randomize textures
        with objects:
            rep.randomizer.materials(
                materials=rep.get.prims(path_pattern="/World/Looks/.*")
            )

    return True

rep.randomizer.register(randomize)
```

### Step 4: Generate Data

Run data generation loop:

```python
# Generate 10,000 frames
rep.orchestrator.run_until_complete(num_frames=10000)

# Data automatically saved to:
# - RGB images: /_isaac_sim/rgb/*.png
# - Bounding boxes: /_isaac_sim/bounding_box_2d_tight/*.json
# - Segmentation: /_isaac_sim/semantic_segmentation/*.png
```

## Annotation Formats

### Bounding Box (COCO Format)

```json
{
  "images": [
    {"id": 1, "file_name": "frame_0001.png", "width": 1280, "height": 720}
  ],
  "annotations": [
    {
      "id": 1,
      "image_id": 1,
      "category_id": 1,
      "bbox": [320, 180, 150, 200],  # [x, y, width, height]
      "area": 30000,
      "iscrowd": 0
    }
  ],
  "categories": [
    {"id": 1, "name": "cup"},
    {"id": 2, "name": "book"}
  ]
}
```

Compatible with YOLOv8, Faster R-CNN, etc.

### Semantic Segmentation

```python
# segmentation.png (grayscale image)
# Pixel value = class ID
# 0: background
# 1: cup
# 2: book
# 3: table
```

Convert to color-coded visualization:

```python
import numpy as np
import cv2

seg_map = cv2.imread("segmentation.png", cv2.IMREAD_GRAYSCALE)
color_map = {
    0: [0, 0, 0],       # Background: black
    1: [255, 0, 0],     # Cup: red
    2: [0, 255, 0],     # Book: green
    3: [0, 0, 255]      # Table: blue
}

colored_seg = np.zeros((seg_map.shape[0], seg_map.shape[1], 3), dtype=np.uint8)
for class_id, color in color_map.items():
    colored_seg[seg_map == class_id] = color

cv2.imwrite("segmentation_colored.png", colored_seg)
```

### Depth Maps

```python
# depth.npy (NumPy array)
depth = np.load("depth.npy")  # Shape: (720, 1280), values in meters
```

Visualize as grayscale image:

```python
depth_normalized = (depth - depth.min()) / (depth.max() - depth.min())
depth_vis = (depth_normalized * 255).astype(np.uint8)
cv2.imwrite("depth_vis.png", depth_vis)
```

## Training on Synthetic Data

### Object Detection (YOLOv8)

```bash
# Install Ultralytics
pip install ultralytics

# Train on synthetic COCO dataset
yolo detect train \
    data=/path/to/coco_synthetic.yaml \
    model=yolov8n.pt \
    epochs=100 \
    imgsz=640
```

**coco_synthetic.yaml**:
```yaml
path: /isaac_sim_output
train: images/train
val: images/val

names:
  0: cup
  1: book
  2: pen
```

After training, deploy to Isaac ROS for real-time inference.

### Depth Estimation

Train depth prediction model:

```python
# Pseudo-code for depth training
model = DepthNet()
optimizer = Adam(model.parameters())

for epoch in epochs:
    for rgb, depth_gt in dataloader:
        # Predict depth from RGB
        depth_pred = model(rgb)

        # Loss: L1 distance
        loss = nn.L1Loss()(depth_pred, depth_gt)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
```

Synthetic depth maps provide perfect ground truth.

## Sim-to-Real Transfer

### The Reality Gap

Models trained purely on synthetic data may fail on real data due to:

**Visual Differences**:
- Simulated textures don't match real materials exactly
- Lighting models approximate real-world illumination
- Camera sensor noise differs

**Physics Differences**:
- Contact dynamics simplified
- Object behaviors (deformation, friction) approximated

### Bridging the Gap

**1. Domain Randomization** (covered above)
- Vary synthetic data parameters widely
- Model learns to ignore irrelevant variations

**2. Domain Adaptation**
- Fine-tune on small real-world dataset
- Use transfer learning (pre-train on synthetic, fine-tune on real)

**3. Sensor Noise Modeling**
- Add realistic camera noise to synthetic images
- Match ISO, exposure, compression artifacts

**4. Progressive Training**
```
Phase 1: Train on 100% synthetic data (100k images)
Phase 2: Fine-tune on 10% real data (10k images)
Phase 3: Validate on held-out real data
```

**5. Visual Style Transfer**
- Use CycleGAN to make synthetic images look more realistic
- Or use real images as backgrounds, composite synthetic objects

### Validation Strategy

**Simulation Testing**:
```
Synthetic validation set: 10k images
Measure: mAP@0.5 = 0.92 (great!)
```

**Real-World Testing**:
```
Real validation set: 1k images
Measure: mAP@0.5 = 0.78 (good, acceptable gap)
```

If gap > 20%, revisit domain randomization or add real data.

## Best Practices

### 1. Match Real-World Distribution

Ensure synthetic data reflects real deployment:

```python
# If real robot operates in warehouses:
environments = ["warehouse_1", "warehouse_2", "warehouse_3"]

# If real robot sees 80% boxes, 20% people:
object_distribution = {
    "box": 0.8,
    "person": 0.2
}
```

### 2. Balance Randomization

**Too little**: Model overfits to specific conditions
**Too much**: Model can't learn meaningful patterns

Start conservative, increase randomization if real-world performance is poor.

### 3. Validate Early

Test on real data after every 10k synthetic images:

```
Iteration 1: 10k synthetic → Test on 100 real → mAP = 0.60
Iteration 2: 20k synthetic → Test on 100 real → mAP = 0.70
Iteration 3: 30k synthetic → Test on 100 real → mAP = 0.75 (diminishing returns)
```

Stop when adding more data doesn't improve performance.

### 4. Use Pre-trained Models

Don't train from scratch:

```python
# Start with ImageNet pre-trained model
model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)

# Fine-tune on synthetic data
fine_tune(model, synthetic_dataset)
```

Pre-training on real images (ImageNet) helps bridge the sim-to-real gap.

## Summary

Synthetic data generation in Isaac Sim enables:

**Scalable Training Data**:
- Generate 100k+ labeled images in hours
- Perfect annotations (bounding boxes, segmentation, depth)
- Unlimited diversity via domain randomization

**Domain Randomization**:
- Vary lighting, textures, positions, camera parameters
- Model learns to generalize across variations
- Improves real-world performance

**Sim-to-Real Transfer**:
- Domain randomization reduces reality gap
- Fine-tune on small real-world datasets
- Validate early and often on real data

**Integration**:
- Train models on synthetic data
- Deploy with Isaac ROS for GPU-accelerated inference
- Use in Nav2 for perception-based navigation

Synthetic data democratizes AI robotics—no longer need massive labeled datasets to train production-quality models.

---

**Next Steps**: You've completed Module 3! Continue to [Module 4: Vision-Language-Action](../module-4-vla/index.md) to learn how humanoids understand and execute natural language commands.

## References

NVIDIA. (2024). *Omniverse Replicator Documentation*. https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html

Tobin, J., et al. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. *IEEE/RSJ IROS*.
