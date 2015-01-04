#Chapter 3 -- Writing ROS programs

##3.1 Creating a workspace and a package

---
###**_Creating a workspace_**

Before we write any programs, the first steps are to create a workspace to hold out packages, and then to create the package itself.

####**_Creating a workspace_**

Packages that you create should live together in a directory called a **workspace**. Use the normal *mkdir* command to create a directory. We'll refer to this new directory as your **workspace directory**.

One final step is needed to set up the workspace. Create a subdirectory called *src* inside the workspace directory.

####**_Creating a package_**

The command to create a new ROS package, which should be run form the src directory of your workspace, looks like this :
```
catkin_create_pkg package-name
```
It creates a directory to hold the package and creates two configuration files (package.xml and CMakeLists.txt) inside that directory.
####**_Editing the manifest_**
The following content shows the manifest from our agitr package.
```
<?xml version = "1.0"?>
<package>
    <name>agitr</name>
    <version>0.01</version>
    <description>
        Example from A Gentle Introduction to ROS
    </description>
    <maintainer email="jokane@cse.sc.edu">
        Jason O'Kane
    </maintainer>
    <license>TODO</license>
    <buildtool_depend>catkin</buildtool_depend>
    <build_depend>geometry_msgs</build_depend>
    <run_depend>geometry_msgs</run_depend>
    <build_depend>turtlesim</build_depend>
    <run_depend>turtlesim</run_depend>
</package>
```
##3.2 Hello,ROS!
---
###**_A simple program_**
hello.cpp
```
//This header defines the standard ROS classes.
#include <ros/ros.h>
int main(int argc, char **argv) {
    //Initialize the ROS system.
    ros::init(argc, argv, "hello_ros");
    //Establish this program as a ROS node
    ros::NodeHandle nh;
    //send some output as a log message
    ROS_INFO_STREAM("Hello, ROS");
}
```
###**_Compiling the Hello program_**
four steps

**One.** Declaring dependencies

To list dependencies, edit the CMakeLists.txt. The default version of this file has this line:
```
find_package(catkin REQUIRED)
```
Dependencies on other catkin packages can be added in a COMPONENTS section :
```
find_package(catkin REQUIRED COMPONENTS package-names)
```
For the hello example, we need one dependency on a package called *roscpp*:
```
find_package(catkin REQUIRED COMPONENTS roscpp)
```

We should also list dependencies in the package manifest(package.xml), using the build_depend and run_depend elements:
```
<build_depend> package-name</build_depend>
<run_depend>package-name</run_depend>
```
In our example:
```
<build_depend>roscpp</build_depend>
<run_depend>roscpp</run_depend>
```
**Two.** Declaring an executable
we need to add two lines to CMakeLists.txt declaring the excutable we would like to create.
```
add_executable(executable-name source-files)

target_link_libraries(executable-name ${catkin_LIBRARIES})
```
In our example, we should add these two lines to CMakeLists.txt:
```
add_executable(hello hello.cpp)
target_link_libraries(hello ${catkin_LIBRARIES})
```
example:
```
cmake_minimum_required(VERSION 2.8.3)
project(agitr)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp)

#declare our catkin package
catkin_package()

#specify locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

#declare the executable, along with its source files
add_executable(hello hello.cpp)

#specify libraries against which to link
target_link_libraries(hello ${catkin_LIBRARIES})

```
**Three.** Building the workspace
You can build your workspace-including compiling all of the executables in all of its packages-using this command:
```
catkin_make
```
Because it's designed to build all of the packages in your workspace, this command must be run from your workspace directory.

It will perform several configuration steps and create subdirectories called *devel* and *build* within your workspace.

*Four.* Sourcing setup.bash
The final step is to execute a script called *setup.bash*, which is created by *catkin_make* inside the devel subdirectory of your workspace:
```
source devel/setup.bash
```
This automatically-generated script sets several environment variables theat enable ROS to find your package and its newly-generated executables.

###**_Executing the hello program_**
using the command *rosrun* to execute the hello program:
```
rosrun agitr hello
```
Don't forget to start roscore first: This program is a node, and nodes need a master to run correctly.

The program should produce output that looks something like this:
```
[INFO][1416432122.659693753]:Hello, ROS!
```
## 3.3 A publisher program

The main differences between pubvel and hello all stem from the need to publish messages.

pubvel.cpp
```
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    //initialize the ROS system and become a node
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;
    //create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtlel/cmd_vel",1000);
    //seed the random number generator
    srand(time(0));
    //look at 2Hz until the node is shut down
    ros::Rate rate(2);
    while(ros::ok() ) {
        //create and fill in the message.
        geometry_msgs::Twist msg;
        msg.linear.x = double(rand())/double(RAND_MAX);
        msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
        //publish the message
        pub.publish(msg);
        //send a message to rosout with the details
        ROS_INFO_STREAM("sending random velocity command:"
                << " linear=" << msg.linear.x
                << " angular=" << msg.angular.z);
        //wait until it's time for another iteration
        rate.sleep();
    }
}

```

###**_Publishing messages_**

####**_Including the message type declaration_**

```
#include <package_name/type_name.h>
```
####**_Creating a publisher object_**
```
ros::Publisher pub = node_handle.advertise<message_type>(topic_name, queue_size);
```
####**_Creating and filling in the message object_**

geometry_msgs/Twist message type has two top-level fields(liner and angular), each of which contains three sub-fields(x, y and z). Each of these sub-fields is a 64bit floating point number, called a double by most c++ compilers.
```
geometry_msgs::Twist msg;
msg.linear.x = double(rand())/double(RAND_MAX);
msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;
```
####**_Publishing the message_**
```
pub.publish(msg);
```
###**_The publishing loop_**
####**_Checking for node shutdown_**
The condition of pubvel's while loop is:
```
ros::ok()
```
####**_Controlling the publishing rate_**
```
ros::Rate rate(2);
```
This object controls how rapidly the loop runs. The parameter in its constructor is in units of Hz, that is, in cycles per second.

Near the end of each loop iteration, we call the sleep method of this object:
```
rate.sleep();
```
Each call of the this method causes a delay in the program.
###**_Compiling pubvel_**
In CMakeLists.txt 
```
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
```
Note that we are modifying the existing find_package line, rather than creating a new one.

In package.xml
```
<build_depend>geometry_msgs</build_depend>
<run_depend>geometry_msgs</run_depend>
```
Then
```
catkin_make
```
###**_Executing pubvel_**
```
source devel/setup.bash
rosrun agitr pubvel
```
You'll also want to run a turtlesim simulator, so that you can see the turtle respond to the motion commands that pubvel publishes:
```
rosrun turtlesim turtlesim_node
```


---



