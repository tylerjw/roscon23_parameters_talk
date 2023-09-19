# TODO

- [ ] Example C++ code for declaring, getting, validating, dynamic update
- [ ] What is the story? Why do people like generate_parameter_library?
- [ ] What is the call to action? What should people do use / contribute / upstream?
- [ ] How do we theme the presentation? Jokes? Capture / Hold attention?


## References

- [Understanding ROS 2 Parameters Tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Parameter Concept](https://docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html)
- [Using parameters in a class (C++)](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
- [Using parameters in a class (Python)](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [Migrating Launch Files](https://docs.ros.org/en/rolling/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html)
- [Migrating Parameters](https://docs.ros.org/en/rolling/How-To-Guides/Migrating-from-ROS1/Migrating-Parameters.html)
- [parameter_blackboard source](https://github.com/ros2/demos/blob/rolling/demo_nodes_cpp/src/parameters/parameter_blackboard.cpp)
- [Parameter API design in ROS](https://design.ros2.org/articles/ros_parameters.html)


## Details from Docs

### Features

- read-only (set through declaration) can only be modified (set) durring startup and not afterwards through a service call. `qos_overrides` parameters have this property.

### Types
- Key (string)
- Value
  - One of: bool, int64, float64, string, byte[], bool[], int64[], float64[], string[]
- Descriptor
 - Default to empty
 - description (string)
 - ranges (integer or floating point)
 - type
 - additional constraints

Parameters have a type. Attempts to change the type fail at runtime. `dynamic_typing` is a setting in the descriptor. I don't undestand how this works, nor do I support it in generate_parameter_library.

### Callbacks (functions to register them)
- `add_pre_set_parameters_callback` - takes mutable list of parameters and returns nothing. Can be used to automatically change some parameters based on others.
- `add_on_set_parameters_callback` - takes immutable list of parameters and returns `rcl_interfaces/msg/SetParametersResult`. This is used for validation. When an invalid value or set of values is attempted to be set you can fail with a helpful error message. This must **NOT** have side-effects, for this reason we only use free functions for validation.
- `add_post_set_parameters_callback` - used for reacting to a changed parameter. We should probably use this to update the timestamp and update paraemters. Currently we just read parameters when the user gets the struct.

### Services (prefixed with `/node_name/`)
- `describe_parameters` - list of parameter keys -> list of descriptors
- `get_parameter_types` - list of parameter keys -> list of types
- `get_parameters` - list of parameter keys -> list of values
- `list_parameters` - (optional) list of prefixes -> list of parameter keys
- `set_parameters` - list of parameter keys and values -> list of results
- `set_parameters_atomically` - list of parameter keys and values -> result

### Initial setting
- cli arguments - `--ros-args -p param_name:=param_value`
- yaml file - `--ros-args --params-file demo_params.yaml`

### Migrating from ROS 1

In ROS 2 there is no roscore with a global parameter blackboard. Instead parameters are per-node and managed by each processes. There is a `parameter_blackboard` node in the `demo-nodes-cpp` that behaves similar to to the ros1 parameter blackboard.

### ROS Parameters in each node

- qos_overrides./parameter_events.publisher.depth
- qos_overrides./parameter_events.publisher.durability
- qos_overrides./parameter_events.publisher.history
- qos_overrides./parameter_events.publisher.reliability
- use_sim_time

### Validation

Tully Foote in Parameter API Design:
> This implies that some entity has the authority to reject or accept a change based on arbitrary criteria. This would also include the ability to convey at least part of the criteria for the acceptance of a change to external actors. For example, communicating the range for an integer or a few choices for a string. This type of information would be used to generate generic user interfaces, but might not capture all criteria. Since the validation criteria can be arbitrary complex and can potentially not be communicated to a client the parameter server could offer to validate an atomic set of parameters and respond with a boolean flag if the values would be accepted (based on the current criteria). Obviously the result might be different when the values are set shortly after but it would allow to implement validators in e.g. a GUI generically.

> When updating a value it can be valuable to know if the parameter update would be accepted without actually requesting the change to happen.

### Observability / Logging

How do we log parameters that are set on nodes and capture that state for replay?

## Notes from the RCLCPP Source

### Declaring parameters

https://github.com/ros2/rclcpp/blob/ea31f94eb453acf3f34b37b85e38284abc514d57/rclcpp/include/rclcpp/node.hpp#L426
This declare method returns a ParameterValue object and is also the same as calling declare and then get on a parameter (and also setting in the case of a default).
If setting the parameter causes validation failure, an exception is thrown.

https://github.com/ros2/rclcpp/blob/ea31f94eb453acf3f34b37b85e38284abc514d57/rclcpp/include/rclcpp/node.hpp#L451
Declare and get that does not provide a default value. This enforced the parameter must be set in some other way?

https://github.com/ros2/rclcpp/blob/ea31f94eb453acf3f34b37b85e38284abc514d57/rclcpp/include/rclcpp/node.hpp#L481
Template variations vs the ones that take the ParameterValue type.

https://github.com/ros2/rclcpp/blob/ea31f94eb453acf3f34b37b85e38284abc514d57/rclcpp/include/rclcpp/node.hpp#L547
Declare a map of parameters of the same type

### Descriptor

https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/msg/ParameterDescriptor.msg

## Questions

- At what point is a parameter considered undeclared? Is this a race condition? How does `allow_undeclared_parameters` work?
- Can `ros2 param dump` be called without a node name or a wildcard to get all the parameters?
- Unsetting parameters? Notifications of this event?
- Incremental query of parameters for tree structure in UI?
- How do we log parameters that are set on nodes and capture that state for replay? Parameter Event topic?
- What is `ignore_override`?
