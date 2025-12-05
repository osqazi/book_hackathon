---
title: LLM-Driven Action Planning
sidebar_position: 2
description: Learn how Large Language Models decompose natural language commands into executable robot actions
---

# LLM-Driven Action Planning

## Introduction

Large Language Models (LLMs) like GPT-4, Claude, and Llama have revolutionized how robots understand and execute natural language commands. Instead of programming every possible task, we can leverage LLMs to **decompose high-level instructions into executable action sequences**.

This section covers prompt engineering for robotics, task decomposition strategies, and grounding language in robot capabilities.

## Why LLMs for Robot Planning?

### Traditional Approach: Explicit Programming

```python
# Every task requires explicit code
if command == "clean_table":
    navigate_to_table()
    detect_objects_on_table()
    for obj in detected_objects:
        grasp_object(obj)
        navigate_to_bin()
        place_object()
        navigate_to_table()
elif command == "bring_drink":
    # Another 20+ lines of code...
elif command == "follow_person":
    # Another 20+ lines of code...
# 100+ commands = 1000+ lines of brittle code
```

**Problems:**
- Requires programming every scenario
- Can't handle variations ("clean the kitchen table" vs "tidy up the table")
- Breaks with unexpected commands
- No reasoning about novel situations

### LLM Approach: Natural Language Decomposition

```python
# Universal command executor
user_command = "Clean the table"

# LLM decomposes into action sequence
action_plan = llm.decompose(user_command, robot_capabilities)

# Execute actions
for action in action_plan:
    robot.execute(action)
```

**LLM Output:**
```json
[
  {"action": "navigate", "target": "table"},
  {"action": "detect_objects", "surface": "table"},
  {"action": "grasp", "object": "cup"},
  {"action": "place", "location": "counter"},
  {"action": "grasp", "object": "plate"},
  {"action": "place", "location": "counter"}
]
```

**Advantages:**
- Handles novel commands without reprogramming
- Generalizes to variations
- Reasons about feasibility
- Explains its decisions

## Prompt Engineering for Robotics

### The Core Prompt Structure

A robotics prompt must include:

1. **Robot Capabilities**: What actions can the robot perform?
2. **Environment Context**: What does the robot know about its surroundings?
3. **Task Constraints**: Safety rules, physics limitations
4. **Output Format**: Structured action sequence

### Example: Humanoid Assistant Prompt

```python
SYSTEM_PROMPT = """
You are a motion planner for a humanoid robot with these capabilities:

ACTIONS:
- navigate(location: str) - Walk to named location
- detect_objects(area: str) - Find objects in area using vision
- grasp(object: str) - Pick up object with gripper
- place(location: str) - Put held object down
- speak(message: str) - Say something to user

KNOWN LOCATIONS:
- kitchen, living_room, bedroom, bathroom
- table, counter, shelf, floor

CONSTRAINTS:
- Can only hold one object at a time
- Must navigate before grasping
- Cannot grasp objects heavier than 5kg
- Must maintain balance while carrying objects

TASK: Decompose the user's command into a sequence of actions.

OUTPUT FORMAT (JSON):
[
  {"action": "navigate", "location": "kitchen"},
  {"action": "detect_objects", "area": "counter"},
  {"action": "grasp", "object": "cup"}
]

If the task is impossible, explain why.
"""
```

### User Command Example

```python
user_command = "Bring me a glass of water from the kitchen"

response = llm.chat([
    {"role": "system", "content": SYSTEM_PROMPT},
    {"role": "user", "content": user_command}
])
```

**LLM Response:**
```json
{
  "plan": [
    {"action": "speak", "message": "I'll get you water from the kitchen"},
    {"action": "navigate", "location": "kitchen"},
    {"action": "detect_objects", "area": "counter"},
    {"action": "grasp", "object": "glass"},
    {"action": "navigate", "location": "sink"},
    {"action": "place", "location": "sink"},
    {"action": "speak", "message": "Filling glass with water"},
    {"action": "grasp", "object": "glass"},
    {"action": "navigate", "location": "user"},
    {"action": "place", "location": "table"}
  ],
  "reasoning": "I need to navigate to kitchen, find a glass, fill it with water at the sink, then bring it to you."
}
```

## Task Decomposition Strategies

### Hierarchical Decomposition

Break complex tasks into subtasks recursively:

```
"Clean the house"
├── Clean the living room
│   ├── Detect objects on floor
│   ├── Pick up each object
│   └── Place in appropriate location
├── Clean the kitchen
│   ├── Clear the counter
│   └── Wipe surfaces
└── Clean the bedroom
    └── ...
```

**Prompt Technique:**
```python
HIERARCHICAL_PROMPT = """
First, decompose the high-level task into major subtasks.
Then, for each subtask, generate the specific action sequence.

Example:
Task: "Prepare breakfast"
Subtasks:
1. Get ingredients (eggs, bread, butter)
2. Cook eggs
3. Toast bread
4. Plate and serve

Actions for subtask 1:
- navigate(kitchen)
- detect_objects(refrigerator)
- grasp(eggs)
- place(counter)
...
"""
```

### Sequential vs Parallel Planning

**Sequential (safe but slow):**
```json
[
  {"action": "navigate", "location": "kitchen"},
  {"action": "grasp", "object": "cup"},
  {"action": "navigate", "location": "bedroom"},
  {"action": "place", "location": "table"}
]
```

**Parallel (efficient but complex):**
```json
{
  "plan": [
    {
      "parallel_group": [
        {"action": "navigate", "location": "kitchen"},
        {"action": "detect_objects", "area": "living_room"}
      ]
    },
    {"action": "grasp", "object": "cup"}
  ]
}
```

Most humanoid robots execute sequentially due to hardware limitations.

## Grounding Language in Robot Capabilities

### The Grounding Problem

LLMs can generate infeasible plans:

**Bad LLM Output:**
```json
{"action": "teleport", "location": "kitchen"}  // No teleportation!
{"action": "grasp", "object": "elephant"}  // Too heavy!
{"action": "fly", "location": "ceiling"}  // Can't fly!
```

### Solution 1: Constrained Action Space

Only allow LLM to select from valid actions:

```python
VALID_ACTIONS = {
    "navigate": {"params": ["location"], "type": "string"},
    "grasp": {"params": ["object"], "type": "string"},
    "place": {"params": ["location"], "type": "string"},
    "detect_objects": {"params": ["area"], "type": "string"}
}

def validate_plan(plan):
    for action in plan:
        if action["action"] not in VALID_ACTIONS:
            return False, f"Invalid action: {action['action']}"
        # Validate parameters...
    return True, "Plan is valid"

llm_plan = llm.generate_plan(user_command)
valid, message = validate_plan(llm_plan)

if not valid:
    # Ask LLM to regenerate with error feedback
    llm_plan = llm.regenerate_plan(user_command, error=message)
```

### Solution 2: Physics Validation

Check if plan obeys physics before execution:

```python
def is_physically_feasible(action):
    if action["action"] == "grasp":
        object_weight = get_object_weight(action["object"])
        if object_weight > MAX_GRIPPER_WEIGHT:
            return False, "Object too heavy"

        robot_pose = get_current_pose()
        object_pose = get_object_pose(action["object"])
        if distance(robot_pose, object_pose) > ARM_REACH:
            return False, "Object out of reach"

    return True, "Feasible"
```

### Solution 3: Iterative Refinement

Use LLM to self-correct:

```python
def execute_with_feedback(plan):
    for action in plan:
        feasible, reason = is_physically_feasible(action)

        if not feasible:
            # Ask LLM to fix the plan
            revised_plan = llm.chat([
                {"role": "user", "content": f"Action {action} failed: {reason}. Revise the plan."}
            ])
            return execute_with_feedback(revised_plan)

        result = robot.execute(action)
        if result.failed:
            # LLM handles execution failures
            revised_plan = llm.handle_failure(action, result.error)
            return execute_with_feedback(revised_plan)
```

## Example: "Clean the Table"

### Step 1: User Command

```python
command = "Clean the table"
```

### Step 2: LLM Decomposes Task

**Prompt:**
```
User command: "Clean the table"

Generate a plan using these actions:
- navigate(location)
- detect_objects(area)
- grasp(object)
- place(location)

The robot is currently in the living room.
```

**LLM Response:**
```json
{
  "plan": [
    {"action": "navigate", "location": "table", "reasoning": "Move to table to see objects"},
    {"action": "detect_objects", "area": "table", "reasoning": "Identify what needs cleaning"},
    {"action": "grasp", "object": "cup", "reasoning": "Pick up cup"},
    {"action": "navigate", "location": "counter"},
    {"action": "place", "location": "counter"},
    {"action": "navigate", "location": "table"},
    {"action": "grasp", "object": "plate"},
    {"action": "navigate", "location": "counter"},
    {"action": "place", "location": "counter"}
  ]
}
```

### Step 3: Validation

```python
valid, msg = validate_plan(llm_plan)
print(f"Plan valid: {valid}")  # True
```

### Step 4: Execution

```python
for action in llm_plan:
    print(f"Executing: {action}")
    robot.execute(action)
```

**Output:**
```
Executing: navigate to table
Executing: detect_objects on table
  Found: cup, plate, book
Executing: grasp cup
Executing: navigate to counter
Executing: place on counter
Executing: navigate to table
Executing: grasp plate
Executing: navigate to counter
Executing: place on counter
```

## Best Practices

### 1. Provide Rich Context

Include environment state, robot capabilities, and constraints in every prompt.

### 2. Use Structured Output

Always request JSON or structured formats—easier to parse and validate.

### 3. Few-Shot Examples

Include 2-3 example decompositions in the prompt:

```python
FEW_SHOT_EXAMPLES = """
Example 1:
Command: "Bring me a book"
Plan:
[
  {"action": "navigate", "location": "shelf"},
  {"action": "detect_objects", "area": "shelf"},
  {"action": "grasp", "object": "book"},
  {"action": "navigate", "location": "user"},
  {"action": "place", "location": "table"}
]

Example 2:
Command: "Put away the toys"
Plan: [...]
"""
```

### 4. Safety Constraints

Always include safety rules in the prompt:

```
SAFETY RULES:
- Never navigate while holding fragile objects
- Stop immediately if person is detected in path
- Do not grasp sharp objects without confirmation
```

### 5. Graceful Failure

Prompt LLM to explain when tasks are impossible:

```json
{
  "plan": null,
  "impossible": true,
  "reason": "Cannot grasp the car—it exceeds the 5kg weight limit."
}
```

## Summary

LLM-driven action planning enables humanoid robots to:

- **Understand** natural language commands
- **Decompose** high-level tasks into action sequences
- **Reason** about feasibility and constraints
- **Adapt** to novel commands without reprogramming

**Key Techniques:**
- Prompt engineering with robot capabilities and constraints
- Structured JSON output for parsing
- Validation layers (action space, physics, safety)
- Iterative refinement with error feedback

**Limitations:**
- LLMs can hallucinate infeasible actions
- Require validation before execution
- Need rich context to generate good plans
- Prompt engineering is task-specific

**Next:** Integrate speech recognition with Whisper to enable voice commands.

---

**Continue to**: [Whisper Speech Recognition](./whisper.md)

## References

Ahn, M., et al. (2022). Do As I Can, Not As I Say: Grounding Language in Robotic Affordances. *arXiv preprint* arXiv:2204.01691.

Huang, W., et al. (2022). Inner Monologue: Embodied Reasoning through Planning with Language Models. *arXiv preprint* arXiv:2207.05608.
