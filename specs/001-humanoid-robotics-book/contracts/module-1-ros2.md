# Module Contract: The Robotic Nervous System (ROS 2)

**Module ID**: P1 (Priority 1)
**Status**: Planned
**Dependencies**: None (foundation module)
**Target Completion**: Week 1

## Overview

This module serves as the foundation for all subsequent modules, introducing students to ROS 2 architecture, middleware concepts, Python integration (rclpy), and URDF modeling for humanoid robots. No prior ROS experience required; assumes basic Python literacy.

## Specifications

**Word Count**: 4,000–6,000 words
**Code Examples**: Minimum 4 Python examples using rclpy
**Diagrams**: Minimum 1 high-level architecture diagram
**Citations**: All ROS 2 technical claims must cite official documentation

## Learning Objectives

1. **Explain ROS 2 Core Concepts**: Define and differentiate nodes, topics, services, and actions with real-world humanoid control examples
2. **Map Humanoid Control to ROS 2**: Identify appropriate communication patterns (topic vs service vs action) for sensor data, motor commands, and task execution
3. **Interpret URDF Robot Descriptions**: Read URDF XML and identify links (body parts) and joints (connections) in a humanoid model
4. **Trace Data Flow**: Follow the path of commands and sensor data from Python scripts → ROS 2 middleware → robot controllers

## Deliverables

### Pages (4 minimum)
1. **index.md**: Module introduction, learning objectives, prerequisites
2. **nodes-topics.md**: ROS 2 nodes and publish-subscribe pattern with topics
3. **services-actions.md**: Synchronous services and long-running actions
4. **urdf-modeling.md**: URDF structure for humanoid robot description

### Code Examples (4 minimum, rclpy)
1. **Minimal Publisher**: Simple node publishing string messages to a topic
2. **Minimal Subscriber**: Node subscribing to and printing topic messages
3. **Service Server/Client**: Request-response pattern for immediate tasks
4. **Action Server/Client** (or URDF snippet): Long-running tasks with feedback OR URDF example showing humanoid links/joints

### Diagrams (1 minimum)
1. **Python → ROS 2 → Controllers Architecture**: High-level data flow showing Python code, ROS 2 topics, controller nodes, and robot actuators with bidirectional feedback

### Citations
- ROS 2 official documentation (nodes, topics, services, actions)
- URDF specification
- rclpy API reference

## Acceptance Criteria

Maps to spec.md User Story 1 acceptance scenarios:

1. **Node Understanding**: After reading, student can explain what a ROS 2 node is and why humanoid control uses multiple nodes (modularity, fault isolation)
2. **Pattern Selection**: Given a scenario like "move arm to position," student can identify whether to use a topic (continuous sensor data), service (immediate command), or action (motion with progress feedback)
3. **URDF Interpretation**: Shown a simple URDF snippet, student can identify `<link>` elements (torso, left_arm, head) and `<joint>` elements (shoulder_joint, elbow_joint)
4. **Data Flow Tracing**: Using the architecture diagram, student can explain how a Python `publish()` call reaches a motor controller and how sensor data returns

## Content Outline

### 1. Introduction (index.md) - 800 words
- What is ROS 2 and why it matters for humanoid robotics
- Key differences from ROS 1 (not a migration guide, just context)
- Module learning objectives and structure
- Prerequisites: Python basics, command-line familiarity
- Link to ROS 2 installation docs (external)

### 2. Nodes and Topics (nodes-topics.md) - 1,400 words
- **Nodes**: Independent processes, single responsibility, fault isolation
- **Topics**: Asynchronous publish-subscribe, many-to-many communication
- **Example**: Sensor data streaming (IMU, joint states) via topics
- **Code Example 1**: Minimal Publisher (rclpy)
- **Code Example 2**: Minimal Subscriber (rclpy)
- **Diagram Element**: Nodes connected by topics

### 3. Services and Actions (services-actions.md) - 1,200 words
- **Services**: Synchronous request-response, client-server pattern
- **Actions**: Long-running tasks with feedback, preemptable, goal-oriented
- **When to Use Each**: Decision tree for communication pattern selection
- **Example**: Service for "get joint angle," Action for "execute grasp"
- **Code Example 3**: Service Server/Client (rclpy)
- **Code Example 4** (option): Action Server/Client skeleton

### 4. URDF Modeling (urdf-modeling.md) - 1,000 words
- **URDF Basics**: XML format for robot kinematic/dynamic description
- **Links**: Rigid bodies (torso, head, arms, legs)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Humanoid Example**: Simplified biped URDF snippet
- **Visualization**: How URDF maps to 3D model in RViz/Gazebo
- **Code Example 4** (alternative): URDF XML snippet with comments

### 5. Architecture Integration (index.md or separate) - 600 words
- **Overall System**: Python scripts → ROS 2 → Controllers → Actuators
- **Diagram**: Python → ROS 2 → Controllers data flow
- **Feedback Loop**: Sensor data → Topics → Python (closed-loop control)
- **Next Steps**: Simulation (Module 2) builds on this foundation

## Testing and Validation

### Content Validation
- [ ] Word count: 4,000 ≤ total ≤ 6,000
- [ ] Code examples: ≥4 Python (rclpy)
- [ ] Diagrams: ≥1 architecture diagram
- [ ] Citations: All ROS 2 claims cited

### Technical Accuracy
- [ ] Python code examples syntactically valid
- [ ] rclpy API usage matches current Humble/Iron docs
- [ ] URDF XML valid against specification
- [ ] All links to ROS 2 docs accessible

### Learning Outcome Testing
- [ ] Sample quiz: Node definition, topic vs service vs action
- [ ] URDF reading exercise: Identify links/joints in provided snippet
- [ ] Data flow exercise: Trace message path in architecture diagram

## Dependencies

**Prerequisite Modules**: None
**Prerequisite Knowledge**: Basic Python (variables, functions, classes), command-line usage
**External Resources**: ROS 2 installation guide (linked, not included)

## Notes

- Keep examples simple and focused on concepts, not production code
- Pseudocode acceptable if clearly marked and explanatory
- Emphasize "why" (design decisions) not just "how" (syntax)
- Link to official docs for deep dives (installation, advanced topics)
