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


# Tutorials

## Interactions with a ROSLab IDE

Most of the interactions are done by items context menus. When your workspace is loaded, there will be a tree showing it up. There you can select and expand the items. Just right-click on an item to open its context menu and to interact with it.

### Examples

Create new ROSLab package:
> Workspace Item Menu -> Add -> ROSLab package

- Name your package.

Create new library in ROSLab package:
> ROSLab package Item Menu -> Add -> Library

- Name your library.

## Creating a simple publisher / subscriber
This tutorial is a pendant to the common ROS publisher / subscriber tutorials.

### Creating the package
- Create a ROSLab package and name it 'roslab_tutorial'.

### Creating the publisher library
- Add new library to 'roslab_tutorial' and name it 'Talker'.

> ROSLab package item 'roslab_tutorial' -> Add -> Library

- Add a publisher to 'Talker'. Select 'Publisher', package: std\_msgs, type: std_msgs/String and name the topic '/chatter'.

> Library item 'Listener' -> Add -> Basic Communication

- Add a function to 'Talker' and name it 'run'. Here we will publish continously a string. If you need to understand the code, have a look at the original ROS tutorial (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29).

> Library item 'Listener' -> Add -> Function 

Code snippet:
```python
r = rospy.Rate(1.0)
self._chatter_msg.data = 'Hello World!'
while not rospy.is_shutdown():
    self._chatter_pub.publish(self._chatter_msg)
    print 'I said:' self._chatter_msg.data
    r.sleep()
```

### Creating the subscriber library
- Add new library to 'roslab_tutorial' and name it 'Listener'.

> ROSLab package item 'roslab_tutorial' -> Add -> Library

- Add a subscriber to 'Listener'. Select 'Subscriber', package: std\_msgs, type: std_msgs/String and name the topic '/chatter'.

> Library item 'Listener' -> Add -> Basic Communication

- Edit subscriber callback to print received messages.
Code snippet:
```python
self._chatter_msg = _chatter_msg
print 'I heard:', self._chatter_msg.data
```

### Creating nodes for our libraries
- Add node for each library.

> ROSLab package item 'roslab_tutorial' -> Add -> Node
Name them 'talker' for 'Talker' and 'listener' for 'Listener'. (Never end with '_node'! This will be added automatically!)

### Building your nodes
- Easiest way atm is to build the whole workspace.

> Workspace item -> Build
Output should indicate that everything is fine.

### Try it out!
- Start roscore. Menu ROS -> start roscore.

> Menu 'ROS' -> start roscore

- Start nodes. 

> Node item 'listener' -> start
> Node item 'talker' -> start
Output should indicate that everything is up and running.

## Creating a simple service / client
This tutorial is a pendant to the common ROS service / client tutorials.

### Creating the package
- Create a ROSLab package and name it 'roslab_tutorial'. If you already have such a package you can use it.

### Creating the service server library
- Add new library to 'roslab_tutorial' and name it 'AddTwoIntsServer'.

> ROSLab package item 'roslab_tutorial' -> Add -> Library

- Add a service server to 'AddTwoIntsServer'. Select 'Service Server', package: rospy_tutorials, type: rospy_tutorials/AddTwoInts and name the topic '/add\_two_ints'. In the service callback we just add two integers and print the result.

> Library item 'AddTwoIntsServer' -> Add -> Basic Communication

Code snippet:
```python
print 'Returning [{} + {} = {}]'.format(_add_two_ints_req.a, _add_two_ints_req.b, (_add_two_ints_req.a + _add_two_ints_req.b))
return AddTwoIntsResponse(_add_two_ints_req.a + _add_two_ints_req.b)
```

### Creating the service client library
- Add new library to 'roslab_tutorial' and name it 'AddTwoIntsClient'.

> ROSLab package item 'roslab_tutorial' -> Add -> Library

- Add import to 'roslab_tutorial'. Set module 'sys' and keep class empty.

> ROSLab package item 'roslab_tutorial' -> Add -> Import

- Add a service client to 'AddTwoIntsClient'. Select 'Service Client', package: rospy_tutorials, type: rospy_tutorials/AddTwoInts and name the topic '/add\_two_ints'.

> Library item 'AddTwoIntsClient' -> Add -> Basic Communication

- Add a function to 'AddTwoIntsClient' and name it 'usage'. Here we describe how to call the service node.

> Library item 'AddTwoIntsClient' -> Add -> Function

Code snippet:
```python
return '{} [x y]'.format(sys.argv[0])
```

- Add a function to 'AddTwoIntsClient' and name it 'run'. Here we will call the service server and print the response. If you need to understand the code, have a look at the original ROS tutorial (http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29).

> Library item 'AddTwoIntsClient' -> Add -> Function

Code snippet:
```python
if len(sys.argv) == 3:
    x = int(sys.argv[1])
    y = int(sys.argv[2])
else:
    print self.usage()
    sys.exit(1)
print 'Requesting {} + {}'.format(x, y)
print '{} + {} = {}'.format(x, y, self._add_two_ints_sc(x, y))
```

### Creating nodes for our libraries
- Add node for each library.

> ROSLab package item 'roslab_tutorial' -> Add -> Node

Name them 'ati\_server' for 'AddTwoIntsServer' and 'ati\_client' for 'AddTwoIntsClient'. (Never end with '_node'! This will be added automatically!)

### Building your nodes
- Easiest way atm is to build the whole workspace.

> Workspace item -> Build

Output should indicate that everything is fine.

### Try it out!
- Start roscore. Menu ROS -> start roscore.

> Menu 'ROS' -> start roscore

- Start server node.

> Node item 'ati\_server' -> start

- Start cleint node with '3 5' as args.

> Node item 'ati_client' -> start with args

Output should indicate that everything is up and running.

This README / Documentation will be updated and/or improved as soon as possible!