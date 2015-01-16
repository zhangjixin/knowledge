# Chapter 7 -- Parameters
> In which we configure nodes using parameters.

##7.1 Accessing parameters from the command line
---
####**_Listing parameters_**
```
rosparam list
```
####**_Querying parameters_**
```
rosparam get parameter_name
```
It is also possible to retrieve the values of every parameter in a namespace:
```
rosparam get namespace
```
####**_Creating and loading parameter files_**
To store all of the parameters from a namespace, use rosparam dump:
```
rosparam dump filename namespace
```
The opposite of dump is load, which reads parameters from a file and adds them to the parameter sever:
```
rosparam load filename namespace
```
###7.3 Accessing parameters from C++
---
The C++ interface to ROS parameters is quite staightforward:
```
void ros::param::set(parameter_name, input_value);
bool ros::param::get(parameter_name, output_value);
```
###7.4 Setting parameters in launch files
---

####**_Setting parameters_**
```
<param name="param-name" value="param-value" />
e.m.
<group ns="duck_colors">
    <param name="huey" value="red" />
</group>
```
####**_Setting private parameters_**
```
<node ...>
    <param name="param-name" value="param-value" />
</node>
```
####**_Reading parameters from a file_**
```
<rosparam commadn="load" file="path-to-param-file" />
```

---


