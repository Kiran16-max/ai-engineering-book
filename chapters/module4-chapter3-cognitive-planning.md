---
id: module4-chapter3-cognitive-planning
title: 'Module 4 – Chapter 3: Cognitive Planning with LLMs'
sidebar_label: 'Module 4 – Chapter 3: Cognitive Planning'
---

## Chapter Overview

In this chapter, we will explore how to use Large Language Models (LLMs) for **cognitive planning**. This involves translating high-level natural language commands into a sequence of concrete, executable actions for our humanoid robot. This is a crucial step towards creating truly autonomous and intelligent robots.

### Learning Goals

- Understand the concept of cognitive planning in robotics.
- Learn how to use LLMs to generate plans from natural language commands.
- Integrate the generated plans with a ROS 2-based robotic system.

## From Language to Action

The core idea behind cognitive planning is to leverage the reasoning capabilities of LLMs to bridge the gap between human language and robot actions. We can give our robot a high-level command like "get me the red ball from the table," and the LLM will generate a step-by-step plan for how to achieve this goal.

### Example Plan Generation

**User Command**: "Get me the red ball from the table."

**LLM-Generated Plan**:
1.  `find_object('table')`
2.  `move_to('table')`
3.  `find_object('red ball')`
4.  `pick_up('red ball')`
5.  `find_person()`
6.  `move_to('person')`
7.  `give_object('red ball')`

This plan is a sequence of simple, atomic actions that the robot can execute using its existing capabilities (e.g., navigation, manipulation).

## Implementing a Cognitive Planner

To implement a cognitive planner, we need a system that can:

1.  **Receive a natural language command**: This can be from a text input or from the voice-to-action system we built in the previous chapter.
2.  **Prompt an LLM to generate a plan**: The prompt should include the user's command, as well as information about the robot's capabilities and the current state of the environment.
3.  **Parse the LLM's output**: The generated plan needs to be parsed into a machine-readable format.
4.  **Execute the plan**: The system should execute the actions in the plan one by one, monitoring for success or failure at each step.

### Conceptual ROS 2 Node for Cognitive Planning

Here is a conceptual example of a ROS 2 node that implements a cognitive planner:

```python
# conceptual_planner_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_robot_interfaces.srv import ExecutePlan  # Custom service definition

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')
        self.subscription = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10)
        self.plan_executor_client = self.create_client(ExecutePlan, 'execute_plan')

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # 1. Prompt LLM to generate a plan
        plan = self.generate_plan_with_llm(command)
        self.get_logger().info(f"Generated plan: {plan}")

        # 2. Parse the plan
        parsed_plan = self.parse_plan(plan)

        # 3. Execute the plan
        self.execute_plan(parsed_plan)

    def generate_plan_with_llm(self, command):
        # In a real implementation, this would involve calling an LLM API
        # with a carefully crafted prompt.
        prompt = f"Given the command '{command}', generate a plan of executable actions."
        # ... call LLM API ...
        return "1. find_object('table')\n2. move_to('table')" # Dummy response

    def parse_plan(self, plan_text):
        # Parse the text-based plan into a list of actions
        return plan_text.split('\n')

    def execute_plan(self, plan):
        req = ExecutePlan.Request()
        req.plan = plan
        self.plan_executor_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    planner_node = CognitivePlannerNode()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1.  **Prompt Engineering**: Design a prompt for an LLM that encourages it to generate safe and efficient plans for a humanoid robot. The prompt should include information about the robot's capabilities, constraints, and the objects in its environment.
2.  **Plan Parser**: Write a Python script that can parse a text-based plan (like the one in the example above) into a list of actions and their arguments.
3.  **ROS 2 Service**: Create a ROS 2 package that defines a service for executing a plan. The service should take a list of actions as input and return a boolean indicating whether the plan was successfully executed.
