#Chapter 3 -- Writing ROS programs

##3.4 A subscriber program

---
We will subscribe to the /turtle/pose topic, on which turtlesim_node publishes.
An example: *subpose.cpp*
```
// This program subscribes to turtle1/pose and shows its messages on the screen.
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip> // for std::setprecision and std::fixed

// a callback function. executed each time a new pose message arrives.
void poseMessageReceived(const turtlesim::Post& msg) {
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "Position=(" << msg.x << "," << msg.y <<")" << " direction=" << msg.theta);
}

int main(int argc, chr **argv) {
    //initialize the ROS system and become a node
    ros::init(argc, argv, "subscribe_to_pose");
    ros::NodeHandle nh;
    
    //create a subscriber object
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);
    //let ROS take over
    ros::spin();
}

```
###**_writing a callback function_**

A subscriber node doesn't know when message will arive. To deal with this fact, we must implement a **callback function**, which ROS calls once for each arriving message.
```
void function_name(const package_name::type_name &msg) {
    ...
}
```
Node that subscriber callback function have a void return value.

###**_Creating a subscriber object_**
To subscribe to a object, we create a ros::Subscriber object:
```
ros::Subscriber sub = node_handle.subscribe(topic_name, queue_size, pointer_to_callback_function);
```
###**_Giving ROS contrl_**
ROS will only execute our callback function when we give it explicit permission to do so. There are actually two slightly different ways to accomplish this:
```
ros::spinOnce();
```
The other option looks like this:
```
ros::spin();
```
this function is roughly equivalent to this loop:
```
while(ros::ok()) {
    ros::spinOnce();
}
```

###**_Compiling and executing subpose_**
Modify CMakeLists.txt and package.xml, then compiling subpose.cpp.

When both turtlesim_node and pubvel were running, you can see a sample of this program's output.






---



