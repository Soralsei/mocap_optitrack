# Optitrack ROS driver
This is a fork of the official Optitrack ROS driver that adds the ability to publish unlabeled markers to a ROS topic.
The official driver read the "free marker" (markers that are not assigned to any rigidbody/skeleton/markerset) data in Motive's data frames but never did anything with that data.
This fork adds the ability to publish these unlabeled markers to a topic of your choice.

The configuration of the unlabeled marker publisher is similar to the other publishers in the driver.
To configure the unlabeled marker publisher, you can add the `markers` object to the yaml configuration file as follows:
```yaml
rigid_bodies:
  # RigidBodies configuration...
# Unlabeled marker configuration ##
markers:
  topic: "topic name"
  parent_frame_id: "TF frame ID in which the markers will be published"
###################################
optitrack_config:
  # Motive server config...
```

To check how to configure the other publishers, check out the [official wiki](https://wiki.ros.org/mocap_optitrack) or the source code at [src/mocap_config.cpp](https://github.com/Soralsei/mocap_optitrack/blob/master/src/mocap_config.cpp)
