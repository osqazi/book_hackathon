---
title: Services and Actions
sidebar_position: 3
description: Learn when to use ROS 2 services for synchronous requests and actions for long-running tasks in humanoid robot control
---

# Services and Actions: Request-Response and Goal-Oriented Communication

## Introduction

In the previous section, we learned about **topics**—asynchronous streams ideal for continuous data flow. But what about situations where you need:

- An immediate answer to a question ("What's the current battery level?")
- Confirmation that a command was executed ("Move arm to position X—did it succeed?")
- Progress updates for long-running tasks ("Walk 10 meters forward—how far have we gone?")

For these scenarios, ROS 2 provides **services** and **actions**.

## Services: Synchronous Request-Response

A **service** implements a client-server pattern: one node requests information or action, and another node responds.

### Key Characteristics of Services

1. **Synchronous**: The client waits for the response
2. **One-to-One**: One client talks to one server per request
3. **Short-duration**: Best for quick operations (milliseconds to a few seconds)
4. **Request-Reply Structure**: Client sends request data, server sends response data

### Service Definition Structure

Services are defined with two parts:

```
Request   → Data the client sends to the server
Response  ← Data the server sends back to the client
```

Example service type `std_srvs/SetBool`:
```
Request:
  bool data      # true or false

Response:
  bool success   # Did the operation succeed?
  string message # Optional status message
```

### When to Use Services

✅ **Ideal for:**
- Querying robot state ("Get current joint angles")
- Triggering discrete actions ("Enable/disable motor", "Reset odometry")
- Configuration changes ("Set controller gains", "Switch camera mode")
- Short computations ("Inverse kinematics for pose", "Plan footstep")

❌ **Not ideal for:**
- Continuous data streams (use topics)
- Long-running tasks (use actions)
- One-way fire-and-forget messages (use topics)

## Humanoid Service Examples

| Task | Service Type | Use Case |
|------|--------------|----------|
| Get battery level | `GetBatteryStatus` | Query remaining power |
| Set walk speed | `SetFloat64` | Change gait velocity limit |
| Compute inverse kinematics | `ComputeIK` | Calculate joint angles for end-effector pose |
| Reset IMU calibration | `Trigger` | Recalibrate balance sensors |
| Enable torque on joints | `SetBool` | Activate/deactivate motor controllers |

### Service Example: Getting Current Joint Angles

```
Client (Path Planner)  →  Request: "Get left arm joint positions"
                          ↓
Server (Joint State Manager)
                          ↓
Client ← Response: [shoulder: 0.5, elbow: 1.2, wrist: 0.3]
```

The client **blocks** until it receives the response (or times out).

## Code Example 3: Service Server (Query Joint Position)

Let's create a service that returns the current position of a specific joint.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Using standard service type for demo

class JointQueryService(Node):
    def __init__(self):
        super().__init__('joint_query_service')

        # Create a service server
        self.srv = self.create_service(
            SetBool,                  # Service type
            '/get_joint_enabled',     # Service name
            self.handle_query         # Callback function
        )

        # Simulate joint state (in reality, this would come from hardware)
        self.joint_enabled = True
        self.get_logger().info('Joint query service ready')

    def handle_query(self, request, response):
        # This callback is called when a client makes a request
        self.get_logger().info(f'Incoming request: data={request.data}')

        # Process the request and prepare response
        if request.data:
            self.joint_enabled = True
            response.message = 'Joint motor enabled'
        else:
            self.joint_enabled = False
            response.message = 'Joint motor disabled'

        response.success = True
        self.get_logger().info(f'Responding: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = JointQueryService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server Key Points

- **`create_service()`**: Declares this node provides a service
- **Callback function**: `handle_query()` processes each request and returns a response
- **Blocking on server side**: Server processes one request at a time (sequential)
- **Response structure**: Must match the service definition

## Service Client (Calling a Service)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class JointQueryClient(Node):
    def __init__(self):
        super().__init__('joint_query_client')

        # Create a service client
        self.client = self.create_client(SetBool, '/get_joint_enabled')

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service available, sending request')
        self.send_request(True)  # Enable the joint

    def send_request(self, enable):
        request = SetBool.Request()
        request.data = enable

        # Call the service (blocks until response arrives)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(f'Service call succeeded: {response.message}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = JointQueryClient()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Key Points

- **`create_client()`**: Declares this node will call a service
- **`wait_for_service()`**: Ensures the server is running before calling
- **`call_async()`**: Sends the request and returns immediately (non-blocking)
- **`spin_until_future_complete()`**: Waits for the response
- **Timeout handling**: Can specify maximum wait time to avoid infinite blocking

## Actions: Long-Running, Preemptable Tasks with Feedback

Services work well for quick requests, but what about tasks that take time?

- "Walk 10 meters forward" (takes several seconds)
- "Pick up object" (multi-step manipulation)
- "Navigate to waypoint" (involves planning and obstacle avoidance)

For these, ROS 2 provides **actions**—goal-oriented tasks with:

1. **Goal**: What you want to achieve
2. **Feedback**: Periodic updates during execution
3. **Result**: Final outcome when complete
4. **Preemption**: Ability to cancel mid-execution

### Action Structure

```
Goal     → Client sends desired outcome (e.g., "Move to position [x, y, z]")
Feedback ← Server sends periodic updates (e.g., "50% complete, current position [x, y, z]")
Result   ← Server sends final status (e.g., "Success" or "Failed: obstacle detected")
```

### Action Example: Walking to a Target Position

```
Client (Behavior Planner) → Goal: "Walk to [x=5.0, y=2.0]"
                             ↓
Server (Gait Controller)
                             ↓
                          Feedback: "Distance remaining: 3.2m"
                          Feedback: "Distance remaining: 1.5m"
                             ↓
Client ← Result: "Goal reached successfully"
```

If the client decides to cancel mid-walk (e.g., detected obstacle), it can **preempt** the action.

## When to Use Actions vs Services vs Topics

| Use Case | Communication Type | Example |
|----------|-------------------|---------|
| Continuous sensor data | **Topic** | Camera images, IMU readings |
| Quick query | **Service** | "Get battery level" |
| Quick command | **Service** | "Enable motor torque" |
| Long task with progress | **Action** | "Walk to waypoint", "Grasp object" |
| Long task, need to cancel | **Action** | "Navigate to goal" (can cancel if obstacle appears) |
| Long task, need feedback | **Action** | "Execute trajectory" (monitor progress) |

## Decision Tree: Choosing the Right Pattern

```
Need communication?
  ├─ Just sending data continuously? → **Topic (Publisher)**
  ├─ Need a reply?
      ├─ Quick (< 1 second)? → **Service**
      ├─ Long-running (> 1 second)?
          ├─ Need progress updates? → **Action**
          ├─ Might need to cancel? → **Action**
          ├─ Fire and forget? → **Topic (Publisher)**
          └─ Synchronous result only? → **Service** (if short enough)
```

## Actions in Humanoid Robotics

### Example: Grasping an Object

```
Goal:
  object_id: "cup_123"
  approach_vector: [0, 0, -1]  # Approach from above
  grasp_force: 5.0  # Newtons

Feedback (every 0.5 seconds):
  stage: "approaching"      # Current stage
  hand_position: [x, y, z]  # Current end-effector pose
  distance_to_object: 0.15  # Meters

Result:
  success: True
  final_grasp_force: 4.8
  message: "Object grasped securely"
```

### Example: Bipedal Walking

```
Goal:
  target_pose: [x: 5.0, y: 2.0, theta: 1.57]  # Target position
  max_velocity: 0.5  # m/s

Feedback (every 0.2 seconds):
  current_pose: [x: 2.3, y: 1.1, theta: 0.8]
  steps_taken: 12
  estimated_time_remaining: 8.5  # seconds

Result:
  success: True
  final_pose: [x: 5.01, y: 2.03, theta: 1.58]
  total_steps: 25
  message: "Goal reached"
```

## Action Client Code Pattern (Simplified)

While action implementation is more complex than services, here's the general pattern:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_msgs.action import Walk  # Example custom action

class WalkActionClient(Node):
    def __init__(self):
        super().__init__('walk_action_client')
        self._action_client = ActionClient(self, Walk, '/walk_to_goal')

    def send_goal(self, x, y):
        goal_msg = Walk.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal and register callback for feedback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}m')

# Usage
client = WalkActionClient()
client.send_goal(x=5.0, y=2.0)
```

### Action Key Features

- **Feedback callback**: Receives progress updates asynchronously
- **Result callback**: Receives final outcome
- **Cancellation**: Client can call `cancel_goal()` to abort mid-execution
- **Goal status**: Can query whether goal is active, succeeded, aborted, or canceled

## Best Practices

### For Services

1. **Keep them fast**: Aim for < 100ms response time
2. **Timeout handling**: Always specify maximum wait time on client side
3. **Avoid blocking**: Don't call services from time-critical control loops
4. **Clear naming**: Service names should indicate what they do
   ```
   /arm/get_joint_state     ✅
   /query                   ❌
   ```

### For Actions

1. **Provide meaningful feedback**: Give clients insight into progress
   ```python
   # Good feedback
   feedback.stage = "approaching"
   feedback.percent_complete = 65
   feedback.estimated_time_remaining = 3.2

   # Poor feedback
   feedback.status = "running"
   ```

2. **Handle preemption gracefully**: Stop safely when canceled
3. **Clear success/failure states**: Result should unambiguously indicate outcome
4. **Use actions for > 1 second tasks**: Anything shorter can often be a service

### Pattern Selection Guidelines

**Use Topics when:**
- Publishing sensor data continuously
- Broadcasting state information
- Don't need confirmation

**Use Services when:**
- Need immediate response
- Task completes quickly (< 1 second)
- One-off queries or configurations

**Use Actions when:**
- Task takes significant time (> 1 second)
- Need progress updates
- Task might need to be canceled
- Clear success/failure states required

## Summary

ROS 2 provides three communication patterns, each optimized for different use cases:

| Pattern | Direction | Timing | Use Case |
|---------|-----------|--------|----------|
| **Topic** | Many-to-many | Asynchronous | Continuous data streams |
| **Service** | One-to-one | Synchronous | Quick request-response |
| **Action** | One-to-one | Asynchronous | Long tasks with feedback |

For humanoid robotics:
- **Topics**: Sensor streams (cameras, IMU, joint states), continuous motor commands
- **Services**: Quick queries (battery level, joint positions), enable/disable motors
- **Actions**: Complex behaviors (walking, grasping, navigating), multi-step tasks

Understanding when to use each pattern is crucial for building robust, efficient robot control systems. Next, we'll explore how to describe the robot's physical structure using URDF.

---

**Continue to**: [URDF Robot Modeling](./urdf-modeling.md)

## References

Open Robotics. (2024). *ROS 2 Documentation: Understanding Services*. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html

Open Robotics. (2024). *ROS 2 Documentation: Understanding Actions*. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

Open Robotics. (2024). *ROS 2 Documentation: Writing an Action Server and Client (Python)*. https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
