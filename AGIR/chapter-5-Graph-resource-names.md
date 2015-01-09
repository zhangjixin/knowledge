#Chapter 5 -- Graph resource names
>> In which we learn how ROS resolves the names of nodes, topics, paremeters, and services.

##5.1 Global names
---
Nodes, topics, services, and parameters are collectively referred to as **graph resources**. Every graph resource is identified by a short string called a **graph resource name**.

Names, which make sense anywhere they're used, called **global names**.

There are several parts to a global names:

1. A leading slash `/`, which identifies the name as a global name.
2. A sequence of zero or more **namespaces**, separated by slashes. Global names that don't explicitly mention any namespace(`/turtlesim`) are said to be in the **global namespace**.
3. A **base name** that describes the resource itself.

##5.2 Relative names
---
A name, allow ROS to supply a default namespace, is called a **relative graph resource name**, or simply a **relative name**. The characteristic feature of a relative name is that it lacks a leading slash(/).
```
e.m.
teleop_trutle
turtlesim
```
###**_Resolving relative names_**

To resolve a relative name to a global name, ROS attaches the name of the current default namespace to the front of the relative name.
```
default namespace         relative name           globalname
    /turtle         +       cmd_vel        ->       /turtle/cmd_vel
    /a/b/c          +       d/e/f          ->       /a/b/c/d/e/f
```
###**_Setting the default namespace_**

This default namesspace is tracked individually for each node, rather than being a system-wide setting. The default global namespace is `/`. There are a couple of mechanisms that for setting the default namespace:

1. ros::init, accept a command line paremeter called __ns, which specifies a default namespace for that program. `__ns:=default-namespace`.
2. You can also set the default namespace using an environment variable.`export ROS_NAMESPACE=default-namespace`. This is used only when no other default namespace is specified by the __ns paremeter.

###**_Understanding the purpose of relative names_**

Relative names do provide a shortcut to avoid typing the full global names every time, their real value is that they make it easier to build complicated systems by composing smaller parts.

When a node uses relative names, it is essentially giving its users the ability to easily push that node and the topics it uses down into a namespace that the node's original designers did not necessarily anticipate.

##5.3 Private names
---
**Private names**, which begin with a tilde(~) character, are the third and final class of graph resource names. Instead of using the current default namespace, private names use the name of their node as a namespace.

For instance, in a node whose global name is `/sim1/pubvel`, the private name `~max_vel` would be converted to a global name like `/sim1/pubvel/max_vel`.

Private names are private only in the sense that they are resolved into a namespace that is unlikely to be used by any other nodes.

##5.4 Anonymous names
---
**anonymous names** are specifically used to name nodes. The purpose of an anonymous name is to make it easier to obey the rule that each node must have a unique name.

To request an anonymous name, a node should pass *ros::init_options::AnonymousName* as a fourth parameter to ros::init:
```
ros::init(argc, argv, base_name, ros::init_options::AnonymousName);
```
for instance:
```
ros::init(argc, argv, "anon", ros::init_options::AnonymousName);

/anon_1376942789547655
```

---
