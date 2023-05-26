# ROS Handlers Module

The `handlers` module provides a set of classes to streamline the usage of subscribers and publishers in the Robot Operating System (ROS). It encapsulates common operations for subscribing to topics and publishing messages into straightforward, easy-to-use classes that maintain a consistent interface across different message types. 

This module includes two primary components, `SubHandler` and `PubHandler`, that serve as templates for ROS subscriber and publisher handlers respectively. These classes are designed to handle any type of ROS message, provided as a template parameter, which makes them versatile across various types of robotic applications.

## SubHandler

`SubHandler` is a template class that abstracts the operation of a ROS subscriber. It provides an interface to initialize a subscriber, automatically update messages when new ones are received, and access the most recent message.

### SubHandler.h:

This header file declares the `SubHandler` template class, designed to abstract the operation of a ROS (Robot Operating System) subscriber. This class is made to handle any type of ROS message, given by the template parameter `<T>`. 

The class consists of the following private members:

- `msg`: An object of the template type `T`, which holds the most recently retrieved ROS message.
- `initialized`: A boolean flag indicating whether a message has been received.
- `sub`: A ROS subscriber object.
- `topicName`: A string representing the name of the ROS topic the subscriber is listening to.
- `queueSize`: An integer specifying the size of the queue for the ROS subscriber.

The class also declares a callback function for the ROS subscriber, which is executed when a message is received. It also includes the following public members:

- A constructor to initialize the `SubHandler` object with a given node handle, topic name, and queue size.
- An `update_msg` method to ensure the message is updated when a new one is received.
- A `currMsg` getter method to access the currently stored message.

### SubHandler.cpp:

This is the implementation file for the `SubHandler` class. It defines the constructor, the callback function, and the `update_msg` method. It also contains explicit template instantiations for `nav_msgs::Odometry` and `sensor_msgs::LaserScan`, which informs the compiler about which template types it should generate code for.


## PubHandler

`PubHandler` is a template class that abstracts the operation of a ROS publisher. It provides an interface to initialize a publisher and publish messages to the ROS topic.


### PubHandler.h:

This header file declares the `PubHandler` template class, designed to abstract the operation of a ROS publisher. This class can handle any type of ROS message, given by the template parameter `<T>`. 

The class consists of the following private members:

- `pub`: A ROS publisher object.
- `topicName`: A string representing the name of the ROS topic the publisher is publishing to.
- `queueSize`: An integer specifying the size of the queue for the ROS publisher.

The class also includes the following public members:

- A constructor to initialize the `PubHandler` object with a given node handle, topic name, and queue size.
- A `publish` method to publish a given message to the ROS topic.

### PubHandler.cpp:

This is the implementation file for the `PubHandler` class. It defines the constructor and the `publish` method. Similar to `SubHandler.cpp`, it also contains explicit template instantiations for `geometry_msgs::TwistStamped`, which informs the compiler about which template types it should generate code for.



Overall, these classes serve to simplify the usage of ROS communication features, reducing the complexity of managing multiple publishers and subscribers and improving code readability and maintainability.