# Immutable & Explainable Black Box Recorder
This repository includes a ROS 2 accountability and explainability solution based on the use of Rosbag2, Blockchain and Large Language Models. This approach provide an immutable and explainable Black Box Recorder, indentifying the causes that have triggered a set of specific events and providing natural and meaningful explanations to non-expert users.

# Software artifacts
ROS 2 Humble

RB1 simulator for ROS 2

[AWS RoboMaker Hospital](https://github.com/aws-robotics/aws-robomaker-hospital-world) World ROS package available [here](https://github.com/jmguerreroh/aws-robomaker-hospital-world/tree/ros2).

The following images show the floor plan of the scenario used in the development of this work, including in the second one an obstacle in the route initially calculated to reach the goal destination. This obstacle will force the change of the pre-computed path.

![imagen](https://user-images.githubusercontent.com/13176052/227868761-7df42f3d-9043-4b07-af27-2b843806be0e.png)

![imagen](https://user-images.githubusercontent.com/13176052/227868841-21b6f0e0-1017-4136-94aa-396ba1205a6b.png)

Nodejs, Ganache, Solidity etc.

[LocalGPT](https://github.com/PromtEngineer/localGPT)

# Usage
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
### Solidity contract deployment
The deployment of the Solidity contract containing the blockchain logic can be done through the command
```
ros2 run tamper_proof_bag_recorder contract_deployment
```
After every deployment, the contract address muy be updated [here](https://github.com/laurafbec/immutable_explainable_BBR/blob/main/tamper_proof_bag_recorder/config/blockchain_config.yaml).

Output example
```
Contract Address: 0x7223a08911947C7351B75EA41266Cb695c72124e
```
### Hash chain generation
Hash chain can be generated by using the command
```
ros2 launch tamper_proof_bag_recorder bag_reader_chained_proofs.launch.py
```
Output example
```
[INFO] [bag_reader_chained_proofs-1]: process started with pid [33763]
[bag_reader_chained_proofs-1] [INFO] [1704890660.493952991] [bag_reader_chained_proofs]: URI: /home/laura/ros2_ws/rosbag_output/bag_20240109-1958675830/bag_20240109-1958675830_0.mcap
[bag_reader_chained_proofs-1] [INFO] [1704890660.494336704] [bag_reader_chained_proofs]: Initial nonce: 71b31d8ca5ea07527f39dcb391e0852cdf0fc77f8475d92672fc9888924a40bc
[bag_reader_chained_proofs-1] [INFO] [1704890660.538509840] [bag_reader_chained_proofs]: Stored hash: c3066d23ed8ea92b7f40d6526f48b772aaa1e17e45b14bbf3fa61cee3c2d1970
[bag_reader_chained_proofs-1] [INFO] [1704890660.539432615] [bag_reader_chained_proofs]: Stored hash: a9be65ea92a8827fc7f38ab9936455a8d0542eedaf1d3e03f1418b207217d0bb
[bag_reader_chained_proofs-1] [INFO] [1704890660.622635528] [bag_reader_chained_proofs]: Stored hash: be7d1f5720e985ddf4dc54cfd06ac5cb4d1663a33be030727ce1e9499c6a55b4
[bag_reader_chained_proofs-1] [INFO] [1704890660.687805750] [bag_reader_chained_proofs]: Stored hash: b1ad4602af653f04726942bbe9eb26e223435cf8f06f7566860391592e2bf804
[bag_reader_chained_proofs-1] [INFO] [1704890660.803056400] [bag_reader_chained_proofs]: Stored hash: d69fdb8348b87353013b89970f105b4a0fbc333816f215df4e1dd9ed06dba47c
[bag_reader_chained_proofs-1] [INFO] [1704890660.805142535] [bag_reader_chained_proofs]: Stored hash: 5171ba85ce3f3ab6fece2c855bfbb62e4403747175ad69a3407d438ee03520f5
[bag_reader_chained_proofs-1] [INFO] [1704890660.813931821] [bag_reader_chained_proofs]: Stored hash: f704fd34db8c52d8df98222ba57d757f0c1bc746c54e4df866f087c6d536ca30
[bag_reader_chained_proofs-1] [INFO] [1704890660.815299259] [bag_reader_chained_proofs]: Stored hash: 3b0f6e4996dbad2c36fb3f6920408cffb65feac574dbdc87141f3a7376b44375
[bag_reader_chained_proofs-1] [INFO] [1704890660.901192472] [bag_reader_chained_proofs]: Stored hash: c3066d23ed8ea92b7f40d6526f48b772aaa1e17e45b14bbf3fa61cee3c2d1970
....
```
### Hash verification
The storagement of a hash value in blockchain ca be checked by running
```
ros2 launch tamper_proof_bag_recorder proof_checker.launch.py
```
```
ros2 service call /proof_checker custom_interfaces/srv/ProofCheckerServiceMessage "proof: 'c3066d23ed8ea92b7f40d6526f48b772aaa1e17e45b14bbf3fa61cee3c2d1970'"
requester: making request: custom_interfaces.srv.ProofCheckerServiceMessage_Request(proof='c3066d23ed8ea92b7f40d6526f48b772aaa1e17e45b14bbf3fa61cee3c2d1970')

response:
custom_interfaces.srv.ProofCheckerServiceMessage_Response(answer='The hash value is stored in the block number 3946 on the Blockchain network')
```
## Explainability approach
