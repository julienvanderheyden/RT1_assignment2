# Assignment 2: Action/Service Implementation

This project is part of the second assignment for the **"Research Track 1"** course. It involves the implementation of a ROS package called `assignment2_rt`. The package provides two nodes to interact with an action server from another package (`assignment_2_2024`) and manage robot goals and target tracking.

## **Project Overview**

The `assignment2_rt` package contains two nodes:

### **1. User Node (`user.py`)**
- Implements a user interface to communicate with the `Planning` action from the `assignment_2_2024` package.
- Allows the user to:
  - Send a goal to the robot by typing coordinates in the format `x y`.
  - Cancel the current goal by typing `cancel`.
- Publishes the robot's position and velocity on the topic `/posvel` using a custom message type `Posvel` (defined in `msg/Posvel.msg`).
- Publishes the current target coordinates on the topic `/target` using the `Float64MultiArray` message type.

### **2. Target Service Node (`TSN.py`)**
- Implements a service (`get_target`) to retrieve the last target sent by the user.
- Subscribes to the `/target` topic to keep track of the latest goal.
- Service request and response are defined in `srv/GetTarget.srv`.

  ---

## **Running the Project**

There are two ways to run the project:

### **1. Run Nodes Separately**
Launch the action server and simulation, then start each node in a separate terminal:
```bash
roslaunch assignment_2_2024 assignment1.launch
rosrun assignment2_rt user.py
rosrun assignment2_rt TSN.py
```

### **2. Use the Launch File**
Alternatively, you can use the dedicated launch file to start all components:
```bash
roslaunch assignment2_rt assignment2_rt.launch
```
**Note**: Using this method merges logs from all nodes into one terminal, which may reduce clarity.

---

## Package Structure

```plaintext
assignment2_rt/
├── src/
│   ├── user.py               # Implementation of the user node
│   ├── TSN.py                # Implementation of the TargetServiceNode
├── msg/
│   ├── Posvel.msg            # Definition of the custom message type for /posvel topic
├── srv/
│   ├── GetTarget.srv         # Definition of the service request and response
├── launch/
│   ├── assignment2_rt.launch # Custom launch file to start all components
└── README.md                 # Project documentation
└── CMakeLists.txt	      # Building files 
└── package.xml			
```

---


## **Dependencies**

To run this project, the following dependencies are required:

### **Core ROS Packages**
- `rospy`
- `std_msgs`
- `nav_msgs`
- `actionlib`
- `geometry_msgs`

### **Custom Packages**
- `assignment_2_2024` (provides the `PlanningAction` definition).
- `assignment2_rt` (this package, includes `Posvel.msg` and `GetTarget.srv`).

### **Message Types**
- `Float64MultiArray` (from `std_msgs`).
- `geometry_msgs/PoseStamped` (used in `PlanningAction`).

### **Standard Python Libraries**
- `threading`: Used in the `user.py` script to manage terminal input concurrently.

---

## **Custom Message and Service Definitions**

### **Posvel.msg**
Defines the custom message type used by the `/posvel` topic:
```plaintext
float64 x
float64 y
float64 vel_x
float64 vel_y
```

### **GetTarget.srv**
Defines the request and response structure for the `get_target` service:
```plaintext
---
float64 x
float64 y
```

## **How the Nodes Work**

### **User Node (`user.py`)**
- Subscribes to:
  - `/odom`: Reads the robot's current position and velocity.
- Publishes to:
  - `/posvel`: Sends robot position and velocity data using `Posvel`.
  - `/target`: Publishes the target coordinates using `Float64MultiArray`.
- Interacts with the `Planning` action server to send goals and handle feedback.

### **Target Service Node (`TSN.py`)**
- Subscribes to:
  - `/target`: Reads the most recent goal coordinates.
- Provides the `get_target` service to retrieve the last goal sent by the user.






