from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node


from rclpy.serialization import serialize_message

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.msg import Log

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import rosbag2_py
from rclpy.time import Time

import os
import datetime
from .blockchain import Blockchain

from ament_index_python.packages import get_package_share_directory
import yaml

import secrets
import hashlib



class TamperProofBagRecorder(Node):

    N_HASHES_BLOCK = 225

    def __init__(self):
        super().__init__('tp_bag_recorder_srv')
      
        # QoS profiles subscription definition
        self.map_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_ALL,
          depth=5)

        self.scan_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=500)

        self.odom_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=100)

        self.cmd_vel_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=100)

        self.tf_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=200)

        self.camera_image_raw_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=50)

        self.amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.tf_static_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_ALL
          )
        
        self.robot_description_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.global_costmap_costmap_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.local_costmap_costmap_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.navigate_to_pose_action = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.rosout_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1000)

        param_descriptors = [
            ('initialpose', rclpy.Parameter.Type.INTEGER),
            ('scan', rclpy.Parameter.Type.INTEGER),
            ('map', rclpy.Parameter.Type.INTEGER),
            ('odom', rclpy.Parameter.Type.INTEGER),
            ('cmd_vel', rclpy.Parameter.Type.INTEGER),
            ('amcl_pose', rclpy.Parameter.Type.INTEGER),
            ('tf', rclpy.Parameter.Type.INTEGER),
            ('tf_static', rclpy.Parameter.Type.INTEGER),
            ('robot_description', rclpy.Parameter.Type.INTEGER),
            ('rosout', rclpy.Parameter.Type.INTEGER),
            ('global_costmap_costmap', rclpy.Parameter.Type.INTEGER),
            ('local_costmap_costmap', rclpy.Parameter.Type.INTEGER),
            ('plan', rclpy.Parameter.Type.INTEGER),
            ('navigate_to_pose_action', rclpy.Parameter.Type.INTEGER),
            ('camera_image_raw', rclpy.Parameter.Type.INTEGER),
            ('behavior_tree_log', rclpy.Parameter.Type.INTEGER),
        ]
    
        # Get parameters values
        self.declare_parameters(
            namespace='',
            parameters=param_descriptors
        )

        # Get parameters values
        param_names = [param[0] for param in param_descriptors]

        # Initialize an array to store parameter values
        parameter_values = {}

        # Iterate over parameter names to get and store parameter values
        for param_name in param_names:
            param_value = self.get_parameter(param_name).get_parameter_value().integer_value
            parameter_values[param_name] = param_value

        # Topics and QoS profiles definition
        self.topics_qos_profiles = {
            'initialpose': (PoseWithCovarianceStamped, 'geometry_msgs/msg/PoseWithCovarianceStamped', None, parameter_values.get('initialpose', 1)),
            'scan': (LaserScan, 'sensor_msgs/msg/LaserScan', self.scan_qos, parameter_values.get('scan', 50)),
            'map': (OccupancyGrid, 'nav_msgs/msg/OccupancyGrid', self.map_qos, parameter_values.get('map', 1)),
            'odom': (Odometry, 'nav_msgs/msg/Odometry', self.odom_qos, parameter_values.get('odom', 100)),
            'cmd_vel': (Twist, 'geometry_msgs/msg/Twist', self.cmd_vel_qos, parameter_values.get('cmd_vel', 15)),
            'amcl_pose': (PoseWithCovarianceStamped, 'geometry_msgs/msg/PoseWithCovarianceStamped', self.amcl_pose_qos, parameter_values.get('amcl_pose', 10)),
            'tf': (TFMessage, 'tf2_msgs/msg/TFMessage', self.tf_qos, parameter_values.get('tf', 1000)),
            'tf_static': (TFMessage, 'tf2_msgs/msg/TFMessage', self.tf_static_qos, parameter_values.get('tf_static', 1)),
            'robot_description': (String, 'std_msgs/msg/String', self.robot_description_qos, parameter_values.get('robot_description', 1)),
            'rosout': (Log, 'rcl_interfaces/msg/Log', self.rosout_qos, parameter_values.get('rosout', 5)),
            'global_costmap/costmap': (OccupancyGrid, 'nav_msgs/msg/OccupancyGrid', self.global_costmap_costmap_qos, parameter_values.get('global_costmap_costmap', 1)),
            'local_costmap/costmap': (OccupancyGrid, 'nav_msgs/msg/OccupancyGrid', self.local_costmap_costmap_qos, parameter_values.get('local_costmap_costmap', 10)),
            'plan': (Path, 'nav_msgs/msg/Path', None, parameter_values.get('plan', 5)),
            'navigate_to_pose/_action/status': (GoalStatusArray, 'action_msgs/msg/GoalStatusArray', self.navigate_to_pose_action, parameter_values.get('navigate_to_pose_action', 1)),
            'camera/image_raw': (Image, 'sensor_msgs/msg/Image', self.camera_image_raw_qos, parameter_values.get('camera_image_raw', 15)),
            'behavior_tree_log': (BehaviorTreeLog, 'nav2_msgs/msg/BehaviorTreeLog', None, parameter_values.get('behavior_tree_log', 15))
        }

        # Record var init
        self.record = False

         # Writer object
        self.writer = rosbag2_py.SequentialWriter()

        # Service definition
        self.srv = self.create_service(SetBool, 'tp_bag_recorder_srv', self.BagRecorder_callback)

         # Set the blockchain
        self.blockchain = Blockchain()
        self.blockchain.load_blockchain_config()

        # Get the absolute file path of the .env file
        self.blockchain_config_file = os.path.join(get_package_share_directory('tamper_proof_bag_recorder'), 'config', 'params.yaml')

        self.previous_hash = self.generate_random_hash()
        self.get_logger().info("Initial nonce %s" % (self.previous_hash))

        self.hash_value_list = []

        self.topic_messages_counters = {}
        self.building_block = False

        # Transaction block gas estimation
        #self.block_gas_limit = self.blockchain.get_block_gas_limit()
        


    def BagRecorder_callback(self, request, response):

        # URI of the bag to create and the format
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%f")
        path = os.path.join(os.getcwd(), "rosbag_output")
        if not os.path.exists(path):
            os.makedirs(path)
        uri = os.path.join(path, "bag_" + timestamp)
        storage_options = rosbag2_py._storage.StorageOptions(uri=uri, storage_id='mcap')

        # Default conversion options to store the message in the serialization format the are received in
        converter_options = rosbag2_py._storage.ConverterOptions('', '')

        default_qos = 10
        
        if request.data == True:
            self.record = True
            try:
                self.writer.open(storage_options, converter_options)
            except RuntimeError:
                self.get_logger().error('Failed to create database directory {}'.format(uri))
          
            # Topics' specification and writer's registering process
            for topic, (msg_type, type_string, qos_profile, bc_rate) in self.topics_qos_profiles.items():
                if qos_profile:
                    offered_qos_profiles = self.qos_profile_to_yaml(qos_profile)
                else:
                    offered_qos_profiles = ''
                topic_info = rosbag2_py._storage.TopicMetadata(name=topic, type=type_string, serialization_format='cdr', offered_qos_profiles=offered_qos_profiles)
                self.writer.create_topic(topic_info)

            # Topic's subscription
            for topic, (msg_type, type_string, qos_profile, bc_rate) in self.topics_qos_profiles.items():
                if qos_profile is None:
                    qos_profile = default_qos
                topic_name = topic
                self.subscription = self.create_subscription(msg_type, topic, lambda msg, topic_name=topic, bc_rate=bc_rate: self.topic_callback(msg, topic_name, bc_rate), qos_profile)
        
            
            response.success = True
            response.message = 'Recording...'

        if request.data == False:
            self.record = False            
            if hasattr(self, 'subscription'):
               self.destroy_subscription(self.subscription)

            self.writer = None
            try:
              del self.writer
              self.writer = rosbag2_py.SequentialWriter()
            except NameError as e:
              print(f"An object exists: {e}")

            response.success = False
            response.message = 'Not recording...'

            # Send last messages to the Blockchain
            self.message_counter = 0

            # Get last message value
            message_str = str(self.last_msg)
            hash_value = self.blockchain.get_hash_value(message_str, self.previous_hash)
            self.previous_hash = hash_value

            # Store hash in the Blockchain network
            if len(self.hash_value_list) < TamperProofBagRecorder.N_HASHES_BLOCK:
                self.hash_value_list.append(hash_value)
                self.blockchain.store_hash_in_blockchain(self.hash_value_list)
                self.hash_value_list.clear()
            else:
                self.blockchain.store_hash_in_blockchain(self.hash_value_list)
                self.hash_value_list.clear()
                # Sending the last messages hash if the list's size exceeds gas limit per block
                self.hash_value_list.append(hash_value)
                self.blockchain.store_hash_in_blockchain(self.hash_value_list)
        
        return response

    # Create QoS object from dictionary
    def create_qos_profile(properties):
        qos = QoSProfile()
        for key, value in properties.items():
            setattr(qos, key, value)
        return qos
  
    # Callbacks functions to pass the serialized message to the writer
    def topic_callback(self, msg, topic_name, bc_rate):
        if self.record:
            try:
                if topic_name not in self.topic_messages_counters:
                  self.topic_messages_counters[topic_name] = 0    

                self.writer.write(topic_name, serialize_message(msg), self.get_clock().now().nanoseconds)

                self.topic_messages_counters[topic_name] += 1
                self.last_msg = msg

                if self.topic_messages_counters[topic_name] % bc_rate == 0:
                  message_str = str(msg)
                  hash_value = self.blockchain.get_hash_value(message_str, self.previous_hash)
                  self.get_logger().info("Hash value: %s" % (hash_value))

                  # Hashes' list gas estimation
                  #hash_estimated_gas = self.blockchain.get_hashes_list_gas_price(self.hash_value_list)
                  #self.get_logger().info("Estimated hash list gas: %s" % (hash_estimated_gas))
                
                  self.previous_hash = hash_value
                  self.hash_value_list.append(hash_value)

                  # Store hash in the Blockchain network
                if len(self.hash_value_list) >= TamperProofBagRecorder.N_HASHES_BLOCK:
                    self.get_logger().info("Sending transaction...")
                    self.blockchain.store_hash_in_blockchain(self.hash_value_list)
                    self.hash_value_list.clear()

            except RuntimeError:
                self.get_logger().error('{} topic has not been created yet! Call create_topic first.'.format(topic_name))
        

    # Return nanoseconds from a duration
    def duration_to_node(self, duration):
        t = Time(nanoseconds=duration.nanoseconds)
        node = {}
        (node['sec'], node['nsec']) = t.seconds_nanoseconds()
        return node

    # Return yaml output from QoS profile
    def qos_profile_to_yaml(self, qos_profile):
        profile_list = []
        qos = {}
        qos['history'] = int(qos_profile.history)
        qos['depth'] = int(qos_profile.depth)
        qos['reliability'] = int(qos_profile.reliability)
        qos['durability'] = int(qos_profile.durability)
        qos['lifespan'] = self.duration_to_node(qos_profile.lifespan)
        qos['deadline'] = self.duration_to_node(qos_profile.deadline)
        qos['liveliness'] = int(qos_profile.liveliness)
        qos['liveliness_lease_duration'] = self.duration_to_node(qos_profile.liveliness_lease_duration)
        qos['avoid_ros_namespace_conventions'] = qos_profile.avoid_ros_namespace_conventions
        profile_list.append(qos)

        return yaml.dump(profile_list, sort_keys=False)

    def generate_random_hash(self):
      random_bytes = secrets.token_bytes(32)  # Generate 32 random bytes (256 bits)
      hash_value = hashlib.sha256(random_bytes).hexdigest()
      return hash_value

    
    
def main(args=None):
    rclpy.init(args=args)
    sbr = TamperProofBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
