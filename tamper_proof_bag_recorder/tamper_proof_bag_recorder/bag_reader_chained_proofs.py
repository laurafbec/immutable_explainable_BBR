import rclpy
from rclpy.node import Node

import os
from pathlib import Path


from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from rclpy.serialization import serialize_message

from .blockchain import Blockchain


class BagReaderChainedProofs(Node):

    def __init__(self):

        # Initialize the node
        super().__init__('bag_reader_chained_proofs')

        param_descriptors = [
            ('rosbag_uri', rclpy.Parameter.Type.STRING),
            ('initial_nonce', rclpy.Parameter.Type.STRING),
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
            ('global_costmap/costmap', rclpy.Parameter.Type.INTEGER),
            ('local_costmap/costmap', rclpy.Parameter.Type.INTEGER),
            ('plan', rclpy.Parameter.Type.INTEGER),
            ('navigate_to_pose/_action/status', rclpy.Parameter.Type.INTEGER),
            ('camera/image_raw', rclpy.Parameter.Type.INTEGER),
            ('behavior_tree_log', rclpy.Parameter.Type.INTEGER)
        ]
    
        # Get parameters values
        self.declare_parameters(
            namespace='',
            parameters=param_descriptors
        )

        # Get parameters values
        param_names = [param[0] for param in param_descriptors]

        # Initialize an array to store parameter values
        self.parameter_values = {}

        # Iterate over parameter names to get and store parameter values
        for param_name, param_type in param_descriptors:
            if param_type == rclpy.Parameter.Type.STRING:
                param_value = self.get_parameter(param_name).get_parameter_value().string_value
            elif param_type == rclpy.Parameter.Type.INTEGER:
                param_value = self.get_parameter(param_name).get_parameter_value().integer_value
            else:
                param_value = None
            self.parameter_values[param_name] = param_value

        #path = os.path.join(os.getcwd(), "rosbag_output")
        self.uri = self.parameter_values.get('rosbag_uri')
        self.previous_hash = self.parameter_values.get('initial_nonce')
        self.get_logger().info("URI: %s" % (self.uri))
        self.get_logger().info("Initial nonce: %s" % (self.previous_hash))

        self.topic_messages_counters = {}

    
    def read_rosbag(self):
        storage_options, converter_options = self.get_rosbag_options(self.uri, storage_id='mcap')

        # Reader object
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        # Set the blockchain
        blockchain = Blockchain()
       
        topic_types = reader.get_all_topics_and_types()

        # Create a map for quicker lookup
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        message_counter = 0
      
        # Check behavior_tree_log messages
        while reader.has_next():
            (topic, data, t) = reader.read_next()

            if topic not in self.topic_messages_counters:
                  self.topic_messages_counters[topic] = 0

            self.topic_messages_counters[topic] += 1

            if self.topic_messages_counters[topic] % self.parameter_values.get(topic) == 0:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                #self.get_logger().info("Message tye: %s, Timestamp: %s" % (msg_type, t))

                message_str = str(msg)

                hash_value = blockchain.get_hash_value(message_str, self.previous_hash)
                self.get_logger().info("Stored hash: %s" % (hash_value))
                self.previous_hash = hash_value

    def get_rosbag_options(self, path, storage_id, serialization_format='cdr'):
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format=serialization_format, output_serialization_format=serialization_format)
        return storage_options, converter_options
   
def main(args=None):
    rclpy.init(args=args)
    sbr = BagReaderChainedProofs()
    sbr.read_rosbag()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()