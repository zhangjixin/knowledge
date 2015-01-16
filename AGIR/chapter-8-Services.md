#Chapter 8 -- Services
> In which we call services and respond to service requests.
1. **Service calls** are *bi-derectional*.
2. **Service calls** implement *one-to-one* communication.

##8.1 Terminology for services
---
The **client** node send some data called a **request** to a server node.
The **server** node reply some data called a **response** back to the client.

##8.2 Finding and calling services from the command line
---
####**_Listing all services_**
```
rosservece list
```
Each line of the outputs shows the name of one service that is currently available to call.
####**_Listing services by node_**
To see the services offered by one particular node, use this command:
```
rosnode info node-name
```
####**_Finding the node offering a service_**
To see which node offers a given service, use this command:
```
rosservice node service-name
```
####**_Finding the data type of a service_**
```
rosservice info service-name
```
####**_Inspecting service data types_**
```
rossrv show service-data-type-name
```
####**_Calling services from the command line_**
To get a feel for how services work, you can call them from the command line using this command:
```
rosservice call service-name request-content
```
##8.3 A client program
---
```
#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "spawn_turtle");
    ros::NodeHandle nh;

    //create a client object for the spawn service.
    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
    //create the request and response objects
    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response resp;
    req.x = 2;
    req.y = 3;
    req.theta = M_PI / 2;
    req.name = "Leo";
    bool success = spawnClient.call(req, resp);
    if (success) {
        ROS_INFO_STREAM("Spawned a turtle named" << resp.name);
    } else {
        ROS_ERROR_STREAM("Faild to spawn");
    }
    return 0;
}
```
##8.4 A server program
---
```
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

bool forward = true;
bool toggleForward(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp
) {
    forward = !forward;
    ROS_INFO_STREAM("Now sending"<<(forward?"forward":"rotate")<<" commands");
    return true;
}

int main(argc, char **argv) {
    ros::init(argc, argv, "pubvel_toggle");
    ros::NodeHandle nh;
    
    ros::ServiceServer server = nh.advertiseService("toggle_forward",&toggleForward);
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    ros::Rate rate(2);
    while(ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x = forward ? 1.0 : 0.0;
        msg.angular.z = forward ? 0.0 : 1.0;
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}
```
###Running and improving the server program

To test the pubvel_toggle example program, compile it and run both turtlesim_node and pubvel_toggle. with both running, you can switch the motion commands from translation to rotation and back by calling the toggle_froward from the command line:
```
rosservice call /toggle_forward
```

---



