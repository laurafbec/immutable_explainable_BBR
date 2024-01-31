# immutable_explainable_BBR
This repository includes a ROS 2 accountability and explainability solution based on the use of Rosbag2, Blockchain and Large Language Models. This approach provide an immutable and explainable Black Box Recorder, indentifying the causes that have triggered a set of specific events and providing natural and meaningful explanations to non-expert users.

# Software artifacts
ROS 2 Humble

RB1 simulator for ROS 2

[AWS RoboMaker Hospital](https://github.com/aws-robotics/aws-robomaker-hospital-world) World ROS package available [here](https://github.com/jmguerreroh/aws-robomaker-hospital-world/tree/ros2).

The following images show the floor plan of the scenario used in the development of this work, including in the second one an obstacle in the route initially calculated to reach the goal destination. This obstacle will force the change of the pre-computed path.

![imagen](https://user-images.githubusercontent.com/13176052/227868761-7df42f3d-9043-4b07-af27-2b843806be0e.png)

![imagen](https://user-images.githubusercontent.com/13176052/227868841-21b6f0e0-1017-4136-94aa-396ba1205a6b.png)

Nodejs, Ganache, etc.

[LocalGPT](https://github.com/PromtEngineer/localGPT)

# Software artifacts
## Accountability approach
### Recording service
After starting the simulation, immutable black-box recording information service can be started with the command
```
ros2 launch tamper_proof_bag_recorder tp_bag_recorder.launch.py
```
The previous recording service can be started and stopped by running
```
ros2 service call /tp_bag_recorder_srv std_srvs/srv/SetBool data:\ true
ros2 service call /tp_bag_recorder_srv std_srvs/srv/SetBool data:\ false
```
Output example
```
[INFO] [tp_bag_recorder_srv-1]: process started with pid [15535]
[tp_bag_recorder_srv-1] [INFO] [1700671326.257738212] [tp_bag_recorder_srv]: Initial nonce 92fcd02be0566fb23f8aff37ec7fd4900fcd96b7973623ab80cf2f23742e9e4e
[tp_bag_recorder_srv-1] [INFO] [1700671365.555115050] [tp_bag_recorder_srv]: Sending transaction...
```
