# Module 1 – Chapter 2: ROS 2 Core Concepts - Nodes, Topics, Services, and Actions

## 2.1 Introduction to ROS 2 Core Concepts

Building upon the understanding of ROS 2 as a vital middleware for distributed robot control, This Chapter delves into the fundamental building blocks of any ROS 2 application. These core concepts—Nodes, Topics, Services, Actions, and Parameters—orchestrate the communication and computational architecture of a robot, much like the specialized organs and pathways of a nervous system. Mastering these concepts is critical for designing, implementing, and debugging robust and scalable robot software architectures, especially for complex platforms like humanoids.

**Learning Goals for this Chapter:**

*   Define and understand the purpose of ROS 2 Nodes and their lifecycles.
*   Grasp the publish/subscribe communication model using ROS 2 Topics, including message types and Quality of Service (QoS) policies.
*   Comprehend the request/response communication of ROS 2 Services and their appropriate use cases.
*   Learn about ROS 2 Actions for managing long-running, goal-oriented tasks with feedback.
*   Understand the role of the ROS 2 parameter system for dynamic configuration.
*   Visualize and interpret the ROS 2 Graph to analyze data flow within a robotic system.

## 2.2 ROS 2 Nodes

A **Node** is an executable process within a ROS 2 system, designed to perform a single, well-defined task. Examples include a node dedicated to reading data from a specific sensor (e.g., a camera driver node), a node for controlling a single motor, or a node executing a complex navigation algorithm. The modularity of nodes promotes code reusability, fault isolation, and overall system robustness.

### Node Lifecycle

ROS 2 introduces a managed node lifecycle, which provides a deterministic way to control the state of nodes. This is crucial for real-time and safety-critical applications. Key states include:
*   **Unconfigured:** Initial state after creation.
*   **Inactive:** Configured and ready to be activated.
*   **Active:** Performing its primary function (e.g., publishing data, processing requests).
*   **Finalized:** Shut down and cleaned up.

Nodes transition between these states through specific commands, allowing for graceful startup, shutdown, and error recovery.

**Example:** A `camera_publisher` node captures images and publishes them, while an `image_processor` node subscribes to these images to detect objects. Each of these would typically be a separate ROS 2 node.

## 2.3 Topics: The Publish/Subscribe Model

**Topics** are the most common method of communication in ROS 2, implementing a publish/subscribe (pub/sub) messaging pattern. This asynchronous, many-to-many communication mechanism is ideal for continuous streams of data. A node that sends data to a topic is a **publisher**, and a node that receives data from a topic is a **subscriber**.

### Message Types

Data exchanged over topics must conform to predefined **message types** (e.g., `sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`, `std_msgs/msg/String`). These types are language-agnostic data structures that define the exact format and fields of the data, ensuring interoperability between nodes written in different programming languages (e.g., Python, C++).

### Quality of Service (QoS) Policies

QoS settings are a powerful feature of ROS 2, leveraging DDS capabilities to provide fine-grained control over communication behavior. Developers can tune QoS policies to match the specific requirements of their data streams. Key QoS policies include:

*   **Reliability:**
    *   `RELIABLE`: Guarantees delivery of every message, retransmitting if necessary. Suitable for critical data where no loss is acceptable (e.g., robot commands).
    *   `BEST_EFFORT`: Attempts to deliver messages but may drop them if the network is congested. Prioritizes low latency over guaranteed delivery. Suitable for high-frequency sensor data where missing a few samples is acceptable (e.g., camera feeds).
*   **Durability:**
    *   `TRANSIENT_LOCAL`: New subscribers receive the last message published before they connected. Useful for non-changing data like map updates.
    *   `VOLATILE`: Only messages published *after* a subscriber connects are received. Default behavior.
*   **History:**
    *   `KEEP_LAST`: Stores a fixed number of the most recent messages.
    *   `KEEP_ALL`: Stores all messages up to resource limits.
*   **Liveliness:** Detects whether a publisher is still active. If a publisher becomes unresponsive, subscribers can be notified.

**Example (Python - Minimal Publisher):**

This example demonstrates a basic `rclpy` node publishing a `String` message to a topic named `chatter`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher on topic 'chatter' with String message type and QoS depth 10
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 client library
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher) # Keep node alive until Ctrl+C
    minimal_publisher.destroy_node() # Clean up resources
    rclpy.shutdown() # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
```

## 2.4 Services: The Request/Response Model

**Services** provide a synchronous request/response communication mechanism in ROS 2. They are used for operations that have a clear start and end, and where the client expects an immediate response. A node acts as a **service client** to send a request, and another node acts as a **service server** to process the request and send back a response.

*   **Use Cases:** Querying a specific sensor value (e.g., current battery level), triggering a one-shot action (e.g., "take a picture"), or performing a specific calculation.
*   **Latency Concerns:** Due to their synchronous nature, services can introduce latency as the client waits for the server's response. They are generally not recommended for high-frequency, continuous data streams, for which topics are more appropriate.

**Example (Conceptual):**
A `robot_manager` node (client) requests a `reset_pose` service from a `motion_controller` node (server). The `motion_controller` executes the reset, and upon completion, sends a success/failure response back to the `robot_manager`.

## 2.5 Actions: Long-Running Goal-Oriented Tasks

**Actions** in ROS 2 are designed for long-running, goal-oriented tasks that require periodic feedback on their progress and the ability to be preempted or cancelled. They combine aspects of both topics (for feedback and results) and services (for goal and result). An action client sends a **goal**, receives **feedback** periodically as the goal is being processed, and ultimately gets a **result** upon completion.

*   **Use Cases:** Typical applications include navigation to a target (where continuous feedback on remaining distance is useful), complex manipulation sequences, or long-duration gait execution in humanoids where intermediate status updates are critical.

**Example (Conceptual):**
A `mission_planner` node (client) sends a "navigate to kitchen" goal to a `humanoid_navigator` node (server). The `humanoid_navigator` periodically sends feedback (e.g., current location, progress percentage) to the `mission_planner`. Once the robot reaches the kitchen, the `humanoid_navigator` sends a final result (e.g., "goal achieved").

## 2.6 Parameter System

The ROS 2 **Parameter System** allows nodes to store and retrieve configuration parameters dynamically at runtime. This provides a flexible mechanism for adjusting the behavior of a robot system without recompiling code. Parameters can be set from the command line, loaded from YAML files, or modified programmatically by other nodes.

*   **Benefits:** Enables easy tuning of algorithms (e.g., PID gains for a motor controller), switching between operational modes, or setting thresholds for sensor processing.

## 2.7 The ROS 2 Graph

The **ROS 2 Graph** is a powerful conceptual and visual representation of all the active nodes, topics, services, and actions within a running ROS 2 system, illustrating how they are interconnected. It maps the communication pathways and data flow, providing invaluable insight for:

*   **Debugging:** Identifying communication bottlenecks or broken connections.
*   **System Understanding:** Visualizing the overall architecture and interactions.
*   **Monitoring:** Observing the dynamic relationships as nodes come online or go offline.

Tools like `rqt_graph` provide a graphical representation of the ROS 2 Graph, allowing developers to see the "nervous system" of their robot in action. Understanding the ROS 2 Graph is fundamental to comprehending the holistic operation of a distributed robotic system.
