#Chapter 9 -- Recording and replaying messages
    In which we use bag files to record and replay messages.

With rosbag, we can record the messages published on one or more topics to a file, and then later replay those messages.
##9.1 Recording and replaying bag files
---
The term bag file refers to a specially formatted file that stores timestamped ROS messages. The rosbag command can be used both to record and to replay bag files.
####**_Recording bag files_**
To create a bag file, use the rosbag command:
```
rosbag record -O filename.bag topic-names
```
you can use ` rosbag record -a ` to record messages on every topic that is currently being published.

When you have finished recording, use Ctrl-C to stop rosbag.
####**_Replaying bag files_**
```
rosbag play filename.bag
```
####**_Inspecting bag files_**
```
rosbag info filename.bag
```
##9.3 Bags in launch files
---
a record node might look like this:
```
<node
    pkg="rosbag"
    name="record"
    type="record"
    args="-O filename.bag topic-names"
/>
```
Likewise, a play node might look like this:
```
<node
    pkg="rosbag"
    name="play"
    type="play"
    args="filename.bag"
/>
```
---
