# `transforms` Library

## Overview

`transforms` is a C++ library that provides utilities for handling transformations in a ROS environment. It provides a TransformBroadcaster class which is responsible for broadcasting transformations that include both moving and static frames. The transformations being broadcast are taken from the current odometry message, and includes transformations from odometry to various frames such as "base_link", "base_link_orientation", "map", and "base_scan". The library utilizes ROS's tf library for the underlying transformations.

## Files

- `TransformNode.cpp`: Main node file. This file initializes a node, creates an odometry subscription handler, and runs the broadcaster.
- `TransformBroadcaster2.h`: Header file for the TransformBroadcaster class. This class contains functions for broadcasting transformations.
- `TransformBroadcaster2.cpp`: Implementation file for the TransformBroadcaster class.

- TransformNode.cpp:

    - This is the entry point of the program where the main function resides.
    - Here, the ROS node is initialized with the name "TransformNode" and a NodeHandle is created for interfacing with ROS.
    - An odometry subscription handler SubHandler is initialized to subscribe to the "/mavros/local_position/odom" topic, which publishes messages of the nav_msgs::Odometry type.
    - Then, a TransformBroadcaster object is created with the odometry subscription handler as an argument.
    - The TransformBroadcaster object's run function is then called, which starts broadcasting the transformations until the node terminates.
    - The main function ends by returning 0, indicating successful execution.

- TransformBroadcaster2.h:

    - The header file for the TransformBroadcaster class, defines the interface for the class.
    - The class contains private member variables representing the transform broadcaster, various transforms, the odometry subscription handler, and the broadcasting frequency.
    - There are also private helper functions for broadcasting each transform (broadcast_transform) and for broadcasting moving frames (broadcast_moving_frames) and static frames (broadcast_static_frames).
    - The public interface of the class consists of a constructor that accepts a SubHandler for `nav_msgs::Odometry` and an optional frequency, and a run function that starts the broadcasting of transforms.

- TransformBroadcaster2.cpp:

    - This file provides the implementation for the TransformBroadcaster class defined in TransformBroadcaster2.h.
    - The broadcast_transform function sets the origin and rotation of a transform and broadcasts the transform.
    - The broadcast_moving_frames function broadcasts moving frames from the odometry to the "base_link" and "base_link_orientation" frames.
    - The broadcast_static_frames function broadcasts static frames from the "map" frame to the "odom" frame and from the "base_link" frame to the "base_scan" frame.
    - The run function starts an infinite loop that keeps broadcasting moving and static frames at a rate defined by the frequency member variable until ROS is shutdown.

## Usage

First, you need to include the `TransformBroadcaster2.h` header in your code:

```cpp
#include "TransformBroadcaster2.h"
```
To use the `TransformBroadcaster` class, you need to construct it with a `SubHandler` object that handles `nav_msgs::Odometry` messages, and optionally, a frequency:

```cpp
SubHandler <nav_msgs::Odometry> odom_sh (&nh, ::odomTopic);
TransformBroadcaster bc (&odom_sh, 10);
```

The TransformBroadcaster class has a `run()` function that runs the broadcaster. This function takes in a boolean parameter that indicates whether the broadcaster should run in a loop or not. If the parameter is set to true, the broadcaster will run in a loop until the node is shutdown. If the parameter is set to false, the broadcaster will only run once. The default value for this parameter is true.

```cpp
bc.run();
```

