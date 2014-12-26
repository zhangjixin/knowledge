#Chapter 2 -- Getting started

##2.3 A minimal example using turtlesim

---
###**_Starting turtlesim_**

In three separate terminals, execute these three commands:

```
roscore

rosrun turtlesim turtlesim_node

rosrun turtlesim turtle_teleop_key
```
If everything works correctly, you should see a graphical window similar to the left part of Figure 2.1.

Give the third terminal(the one executing command turtle_teleop_key ) the input focus and press the Up, Down, Left or Right keys, the turtle will move in response to your commands, leaving a trail behind it.

##2.4 Packages
---
All ROS software is organized into **packages**. A ROS packages is a coherent collection of files, generally including both executables and supporting files, that serves a specific purpose.

Each package is defined by a **manifest**, which is a file called *package.xml*. This file defines some details about the package, including its name, version, maintainer, and dependencies. The directory containing *package.xml* is called the **package directory**.

###Commands for interacting with packages

####**_Listing and locating packages_**
```
rospack list
```
You can obtain a list of all of the installed ROS packages using this command.

####**_Find the directory of a package_**
```
rospack find package-name
```
rospack supports **tab complete** for package name.

####**_Inspecting a package_**

To view the files in a package directory:
```
rosls package-name
```
To go to a package directory:
```
roscd package-name
```
##2.5 The master
---
One of the basic goals of ROS is to enable roboticists to design software as a collection of small, mostly independent programs called **nodes** that all run at the same time. For this to work, those nodes must be able to communicate with one another. The part of ROS that facilitates this communication is called the **ROS  master**. TO start the master, use this command:
```
roscore
```
You should allow the master to continue running for the entire time that you're using ROS. One reasonable workflow is to start roscore in one terminal. When you've finished working with ROS, you can stop the master by typing **Ctrl+C** in its terminal.

Most ROS nodes connect to the master when they start up, if you stop *roscore*, any other nodes running at the time wil be unable to establish new connections, even if you restart *roscore* later.

##2.6 Nodes
---
Once you've started roscore, you can run programs that use ROS. A running instance of a ROS program is called a **node**.
> *If we execute multiple copies of the same program at the same time, each of those copies is treated as a separate node.*

####**_Starting nodes_**
```
rosrun package-name executable-name
```
The first parameter is a package name. The second parameter is simply the name of an executable file within that package.
####**_Listing nodes_**
To get a list of running nodes, try this command:
```
rosnode list
```
####**_Inspecting a node_**
You can get some information about a particular node using this command:
```
rosnode info node-name
```
####**_Killing a node_**
To kill a node you can use this command:
```
rosnode kill node-name
```
> *You can also kill a node using the usual Ctrl-C technique. However, that method may not give the node a chance to unregister itself from the master. To remove dead nodes from _rosnode list_, you can use```rosnode cleanup```*

Killing and restarting a node usually does not have a major impact on other nodes.
##2.7 Topics and messages
---
The primary mechanism that ROS nodes use to communicate is to send **messages**. Messages in ROS are organized into named **topics**. The idea is that a node that wants to share information will **publish** message on the appropriate topic or topics; a node that wants to receive information wil **subscribe** to the topics that it's interested in. The ROS master takes care of ensuring that publishers and subscribers can find each other; the messages themselves are sent directly from publisher to subscriber.
####**_Viewing the graph_**
The easiest way to visualize the publish-subscribe relationships between ROS nodes is to use this command:
```
rqt_graph
```
the _r_ is for ROS, and the _qt_ refers to the _Qt GUI_.
####**_Messages and message type_**

_Listing topics_

To get a list of active topics, use this command:
```
rostopic list
```
_Echoing massages_

You can see the actual messages that are being published on a single topic using the rostopic command:
```
rostopic echo topic-name
```
_Measuring publication rates_

There are also two commands for measuring the speed at which messages are published and the bandwidth consumed by those messages:
```
rostopic hz topic-name

rostopic bw topic-name
```
_Inspecting a topic_

You can learn more about a topic using the rostopic info command:
```
rostopic info topic-name
```
The most important part of this output is the very first line, which shows the **message type** of the topic.

_Inspecting a message type_

To see details about a message type, use a command like this:
```
rosmsg show message-type-name
```
_Publishing messages from the command line_

You may find it useful at times to publish messages by hand.
```
rostopic pub -r rate-in-hz topic-name message-type message-content
```

There are a few additional options to rostopic pub:

1. The form shown here uses -r to select the **rate mode** of rostopic pub, which publishes messages at regular intervals.

2. It is also possible to read messages from a file (using -f) or from standard input (by omitting both -f and the message content from the command).

_Understanding message type names_

Every message type belongs to a specific package. Message type names always contain a slash, and the part before the slash is the name of the containing package:
> _package-name/type-name_


##2.8 A larger example
---
####**_Communication via topics is many-to-many_**

Topics and message are used for **many-to-many** communication. Many publishers ans many subscribers can share a single topic.

####**_Nodes are loosely coupled_**

1. Software (like turtle_teleop_key) that produces information can publish that information, without worrying about how that information is consumed.
2. Software (like turtlesim_node) that consumes information can subscribe to the topic or topics containing the data it needs, without worrying about how those data are produced.

ROS does procide a mechanism, called **services**, for slightly more direct, one-to-one communication.
##2.9 Checking for problems
---
When ROS is not behaving the way you expect, using this command with no arguments:
```
roswtf
```
**_ps: hah, good name 'wtf'_**

---