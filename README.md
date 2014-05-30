# ROSLab


Simple integrated development environment, designed to use the benefits of the ROS framework.


## Description


ROSLab organizes ROS workspace in 'rosinstall packages' and ROSLab packages. At the moment pure packages are not listed. 'rosinstall packages' are listed from '.rosinstall' in current workspace. ROSLab packages contain their own project files and which are loaded at startup. The workspace is structured as a tree where every item has corresponding functionality. It is designed that the most ROS dependent code is auto-generated and only 'functions' can be edited by the user. So it is aimed for beginners which are starting with ROS and professionals which are often writing glue code or wrappers for existing libraries.

### ROSLab Packages


ROSLab packages are the main components in ROSLab IDE. Those are aimed to contain several libraries or wrappers and nodes to start them. All the common ROS files (package.xml, CMakeLists.txt, etc.) are also auto-generated.

#### Dependency

A dependency is a ROS package your package depends on.

#### Library

A library is a object class which encapsulates the ROS components. In simple cases you just have to add calls to your library in the correct function or callback. A library can contain imports, functions and the main ROS communication types like publishers, subscribers, service clients and service servers. Additionally TF broadcasters and listeners are supported.

#### Node

A node is just an executable (script) which creates an instance of the selected library and calls its 'run' function.


## First start


At first start ROSLab IDE will scan your current ROS distribution for any packages containing messages or services, this can take some time, please be patient, its only done once. After this you should open the 'Windows menu -> Settings' and enter your personal info. This is helpful because those settings are used for default auto-generation parameters.


## Tutorial / Example


This tutorial is a pendant to the common ROS publisher / subscriber tutorials.

1. Add new package. Right-click on workspace item -> Add -> package. Name it 'roslab_tutorial'.
2. Add new library. Right-click on new package item -> Add -> library. Name it 'Talker'.
3. Add publisher to library. Right-click on new library item -> Add -> Basic Communication. Select it 'Publisher', package: std_msgs, type: std_msgs/String and name the topic '/chatter'.
4. Add function to library. Right-click on library item -> Add -> Function. Name it 'run'. Publish a continously a string from this function.
```python
r = rospy.Rate(1.0)
self._chatter_msg.data = 'Hello World!'
while not rospy.is_shutdown():
    self._chatter_pub.publish(self._chatter)
    print 'I said:' self._chatter_msg.data
    r.sleep()
```
5. Add new library. Right-click on package item -> Add -> library. Name it 'Listener'.
6. Add subscriber to library. Right-click on new library item -> Add -> Basic Communication. Select it 'Subscriber', package: std_msgs, type: std_msgs/String and name the topic '/chatter'.
7. Print received message in subscriber callback.
```python
self._chatter_msg = _chatter_msg
print 'I heard:', self._chatter_msg.data
```
8. Add node for each library. Right-click on package item -> Add -> Node. Name them 'talker' for 'Talker' and 'listener' for 'Listener'. (Never end with '_node'! This will be added automatically!)
9. Build workspace. Right-click on workspace item -> Build. Output should indicate that everything is fine.
10. Start roscore. Menu ROS -> start roscore.
11. Start nodes. Right-click on each node -> start.
12. Output should indicate that everything is up and running.


This README / Documentation will be updated and/or improved as soon as possible!