# **Social Marketplace ROS 2 Package**
The **Social Marketplace** is a ROS 2 package designed for **multi-agent collaboration**.  
It enables agents to **register**, **analyze scenes**, **place bids**, and **interact** in a simulated marketplace environment.

## **How It Works**
1️⃣ Agents **register** in the marketplace.  
2️⃣ The environment is **analyzed** to extract scene parameters.  
3️⃣ Agents **submit bids** based on the scene parameters.  
4️⃣ The system selects a **winning bid** and assigns a task.  
5️⃣ Agents receive **feedback** to improve future performance.  

This package integrates **bidding, agent interaction, scene analysis, and feedback collection** to facilitate **autonomous decision-making** in a dynamic marketplace. 🚀


## **KEY FEATURES**
✅ Dynamic **bidding system** for agents  
✅ **Scene analysis** with adjustable parameters  
✅ **Agent registration** and interaction management  
✅ **Feedback collection** for improving agent behavior  
✅ Seamless integration with **ROS 2 action, services, and topics**  

---

## **INSTALLATION**
To install and build the package, run:
```bash
cd ~/ros2_ws/src
git clone https://git.faps.uni-erlangen.de/forsocialrobots/forsocialrobots_architecture/social_marketplace.git -b main
cd ~/ros2_ws/src/social_marketplace
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## **USAGE**

## *Launching the Package*
To start the required ROS 2 nodes, execute:
```bash
ros2 launch social_marketplace_bringup social.launch.xml
```

## *Triggering Scene Analysis*
To manually trigger scene analysis, call the ROS 2 service:
```bash
ros2 service call /trigger_scene_analysis social_marketplace_interfaces/srv/TriggerSceneAnalysis "{trigger: true}"
```

## **Example Workflow**

1️⃣ **Launch the Marketplace**
```bash
ros2 launch social_marketplace_bringup social.launch.xml
```

2️⃣ **Trigger Scene Analysis**
```bash
ros2 service call /trigger_scene_analysis social_marketplace_interfaces/srv/TriggerSceneAnalysis "{trigger: true}"
```

3️⃣ Receive and Respond to Bids

4️⃣ Collect Agent Feedback

## **SYSTEM ARCHITECTURE**
A high-level overview of how nodes interact in the system.

## *ROS2 Interfaces*

### **Publishers**
- **`/bid_offers`** (`social_marketplace_msgs/BidOffers`): Publishes bid offers in the system.
- **`/scene_parameters`** (`social_marketplace_msgs/SceneParams`): Publishes updated scene parameters.

### **Subscribers**
- **`/bids`** (`social_marketplace_msgs/Bids`): Subscribes to incoming bid messages.
- **`/scene_parameters`** (`social_marketplace_msgs/SceneParams`): Subscribes to scene analysis parameters.

### **Services**
This package provides the following ROS 2 services:

- **`/trigger_scene_analysis`** (`social_marketplace_interfaces/srv/TriggerSceneAnalysis`): Triggers scene analysis on request.
- **`/agent_registration`** (`social_marketplace_interfaces/srv/AgentRegistration`): Handles registration of marketplace agents.
- **`/simple_agent_interaction`** (`social_marketplace_interfaces/srv/SimpleAgentInteraction`): Facilitates interactions between agents.
- **`/simple_feedback_collection`** (`social_marketplace_interfaces/srv/SimpleFeedbackCollection`): Collects feedback from agents.
