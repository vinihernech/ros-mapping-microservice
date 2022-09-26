# ros-mapping-microservice

# About 

Microservice for automatic mapping of environments using mobile robots with differential traction. This microservice uses an interface between ROS and a Programmable Intelligent Space based on computer vision. Mapping is done using a SLAM algorithm available in ROS [HectorSLAM](http://wiki.ros.org/hector_slam).

## Programmable Intelligent Space (PIS)

The mapping is performed within an intelligent space, so the exploration points that the robot will navigate are in the PIS reference.
A python script stores the robot's initial pose and a transformation is done to convert the points in the PIS reference frame to the map's reference frame. The map's frame of reference is defined by the robot's initial pose.

## ROS
In the ROS part, a launch file initializes HectorSLAM with the settings chosen by the user. These settings consist of map size, resolution and something like that, and these are available in the Deployment file. 

# Dependences 

[Is-reconstruction](https://github.com/matheusdutra0207/is-reconstruction): Estimate the 3D poses of the ArUco Markers.

[ROS-master](http://wiki.ros.org/Master): Provides naming and registration services to the rest of the nodes in the ROS system.

# Usage 

## Streams:
| Name | ⇒ Input | Output  ⇒ | Description |
| ---- | ------- | --------- | ----------- |
| IsRosMapping | :incoming_envelope: **topic:** `IsRosMapping.{robot_id}.MapRequest` <br> :gem: **schema:** MapRequest| :incoming_envelope: **topic:**  `move_base/goal` <br> :gem: **schema:** [MoveBaseGoal](http://docs.ros.org/en/groovy/api/move_base_msgs/html/msg/MoveBaseGoal.html) | It receives exploration points in the PIS referential and sends them to the [ROS navigation stack](http://wiki.ros.org/move_base).|

## Configuration :gear:

The behavior of the microservice can be customized by passing a YAML configuration file.

An example configuration file can be found in [`etc/config/config.yaml`](https://github.com/vinihernech/ros-mapping-microservice/blob/main/etc/config/config.yaml) and [`etc/config/hector_config/hector_params.yaml`](https://github.com/vinihernech/ros-mapping-microservice/blob/main/etc/config/hector_config/hector_params.yaml).


## Kubernetes <img alt="k8s" width="26px" src="https://raw.githubusercontent.com/github/explore/80688e429a7d4ef2fca1e82350fe8e3517d3494d/topics/kubernetes/kubernetes.png" />

Make sure you have [kubectl](https://kubernetes.io/docs/tasks/tools/install-kubectl/) installed and the right `~/.kube/config` file to be able to interact with the cluster.

Deploy the stream application:

```bash
kubectl apply -f etc/k8s/deployment.yaml
```

The `.yaml` file describes two things:
* a deployment;
* a configmap;

A deployment is a way to run our application and guarantee that an N number of replicas will be running. The configmap allows load the options you desires when deploying into the kubernetes platform. See more about [deployment](https://kubernetes.io/docs/concepts/workloads/controllers/deployment/) and [confimap](https://kubernetes.io/docs/concepts/configuration/configmap/).

# Example
An example of use is shown in [examples/client.py](https://github.com/vinihernech/ros-mapping-microservice/blob/main/examples/client.py).
