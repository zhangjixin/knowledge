#Chapter 6 --Launch files
> In which we configure and run many nodes at once using launch files

## 6.1 Using launch files
---
a small example launch file:
```
<launch>
<node 
    pkg="turtlesim"
    type="turtlesim_node"
    name="turtlesim"
    respawn="true"
/>
<node 
    pkg="turtlesim"
    type="turtle_teleop_key"
    name="teleop_key"
    required="true"
    launch-prefix="xterm -e"
/>
<node
    pkg="agitr"
    type="subpose"
    name="pose_subscriber"
    output="screen"
/>
</launch>
```
###**_Executing launch files_**

To execute a launch file, use the roslaunch command:
```
roslaunch package-name launch-file-name

roslaunch agitr example.launch
```

Before starting any nodes, roslaunch will determine whether resocore is already running and, if not, start it automatically.

###**_Requesting verbosity_**

Like many command line tools, roslaunch has an option to request verbose output:
```
roslaunch -v package-name launch-file-name
```
###**_Ending a launched session_**

To terminate an active roslaunch, use Ctrl-C. This signal will attempt to gracefully shut down each active node from the launch, and will forcefully kill any nodes that do not exit within a short time after that.

##6.2 Creating launch files
---

###**_Where to place launch files_**

each launch file should be associated with a particular package. The usual naming scheme is to give launch files names ending with `.launch`. The simplest place to store launch files is directly in the package directory. When looking for launch files, roslaunch will also search subdirectories of each package directory. Some packages utilize this feature by organizing launch files into a subdirectory of their own, usually called `launch`.

###**_Basic ingredients_**

The simplest launch files consists of a root element containing several node elements.

####**_Inserting the root element_**

Launch files must have exactly one **root element**. For ROS launch files, the root element is defined by a pair of launch tags:
```
<launch>
 ...
</launch>
```
####**_Launching nodes_**

A node element looks like this:
```
<node 
    pkg="package-name"
    type="executable-name"
    name="node-name"
/>
```
You can also write the closing tag explicitly:
```
<node pkg="..." type="..." name="...">...</node>
```
A node element has three required attributes:

The *pkg* and *type* attributes identify which program ROS should run to start this node.

The *name* attribute assigns a name to the node. This overrides any name that the node would normally assign to itself in its call to ros::init.

####**_Finding node log files_**

By default, standard output from launched nodes is redirected to a log file:
```
~/.ros/log/run_id/node_name-number-stdout.log
```
####**_Directing output to the console_**

To override this behavior for a single node, use the output attribute in its node element:
```
output="screen"
```
    Nodes launched with this attribute will display their standard output on screen instead of in the log files.
    We can also force roslaunch to display output from all of its nodes, using the --screen command-line option:
```
roslaunch --screen package-name launch-file-name
```
####**_Requesting respawning_**

we can ask roslaunch to restart a node when it terminats, by using a respawn attribute:`respawn="true"`

####**_Requiring nodes_**

An alternative to respawn is to declare that a node is required:`required="true"`.
when a required node terminates, roslaunch responds by terminating all of the other active nodes and exiting itself.
####**_Launching nodes in their own windows_**

One potential drawback to using roslaunch is that all of the nodes share the same terminal. If you want to launch nodes in their own windows, using the launch-prefix attribute of a node element:
```
launch-prefix="command-prefix"
launch-prefix="xterm -e"
```
##6.3 Launching nodes inside a namespace
---
assign the ns attribute in its node element:
```
ns="namespace"
```
##6.4 Remapping names
---
In addition to resolving relative names and private names, ROS nodes also support **remappings**, which provide a finer level of control for modifying the names used by our nodes.

###**_Creating remappings_**

There are two ways create remappings when starting a node.

1. give the original name and the new name, separated by a *:=*, `original-name:=new-name`
2. use a remap element: `<remap from="original" to="new-name"/>`. e.m. 
```
<node node-attributes>
<remap from="" to="">
</node>
```
##6.5 Other launch file elements
---
###**_Including other files_**

```
<include file="path-to-launch-file"/>
```
The include element also supports the ns attribute for pushing its contents into a namespace:
```
<include file="..." ns="namespace" />
```
###**_Launch arguments_**
To help make launch files configurable, roslaunch supports **launch arguments**, also called **arguments** or even **args**.

####**_Decalring arguments_**
```
<arg name="arg-name" />
```
####**_Assigning argument values_**
```
roslaunch package-name launch-file-name arg-name:=arg-value
```
Alternatively, you can provide a value as part of the arg declaration, using one of these two syntaxes:
```
<arg name="arg-name" default="arg-value" />
<arg name="arg-name" value="arg-value" />
```
####**_Accessing argument values_**
Once an argument is declared and a value assigned to it, you can use its value using an arg substitution, like this:
```
$(arg arg-name)
```
####**_Sending argument values to included launch files_**
```
<include file="...">
    <arg name="arg-name" value="arg-value"/>
    ...
</include>
```
###**_Creating groups_**
The group element can serve two purposes:
> Groups can push several nodes into the same namespace.```<group ns="namespace" /> ... </group>```
> groups can conditionally enable or disable nodes. ```<group if="0-or-1" /> ... </group>```


