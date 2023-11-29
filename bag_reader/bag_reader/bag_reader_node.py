import rclpy
from rclpy.node import Node

import os
from pathlib import Path

import math
import rosbag2_py
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message



class BagReaderNode(Node):

    
    def __init__(self):

        # Initialize the node
        super().__init__('bag_interpreter_node')

        # Get parameters values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rosbag_dir', rclpy.Parameter.Type.STRING),
                ('rosbag_file_path', rclpy.Parameter.Type.STRING),
                ('log_output_dir', rclpy.Parameter.Type.STRING),
                ('log_output_file_path', rclpy.Parameter.Type.STRING)

        ])

        rosbag_dir = self.get_parameter('rosbag_dir').get_parameter_value().string_value
        rosbag_file_path = self.get_parameter('rosbag_file_path').get_parameter_value().string_value
        log_output_dir = self.get_parameter('log_output_dir').get_parameter_value().string_value
        log_output_file_path = self.get_parameter('log_output_file_path').get_parameter_value().string_value

        # Check if parameters are None, if so, use default values
        if rosbag_dir is None:
            rosbag_dir = os.path.join(os.getcwd(), "rosbag_output")

        if rosbag_file_path is None:
            self.get_logger().info("Rosbag file path is missing.")

        if log_output_dir is None:
            log_output_dir = os.path.join(os.getcwd(), "log_output")

        if log_output_file_path is None:
            self.log_output_file_path = "raw_data.txt"
        else:
            self.log_output_file_path = log_output_file_path
        
        # Get Rosbag path URI
        self.uri = os.path.join(rosbag_dir, rosbag_file_path)       

        # Create a directory to store the log file
        self.log_dir = log_output_dir
        Path(self.log_dir).mkdir(parents=True, exist_ok=True)
   
        

    # Read Rosbag file
    def read_rosbag(self):
        storage_options, converter_options = self.get_rosbag_options(self.uri, storage_id='sqlite3')

        # Reader object
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
       
        topic_types = reader.get_all_topics_and_types()

        # Create a map for quicker lookup
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        # Set filter for topic of string type
        storage_filter = rosbag2_py.StorageFilter(topics=['rosout','amcl_pose','navigate_to_pose/_action/status','plan','scan','behavior_tree_log','cmd_vel'])
        reader.set_filter(storage_filter)
      
        # Open the txt file for writing messages
        log_file = os.path.join(self.log_dir, self.log_output_file_path)

        with open(log_file, 'w') as logfile:
            while reader.has_next():
                (topic, data, t) = reader.read_next()
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                logfile.write(str(msg) + "\n") 


    # Get Rosbag options
    def get_rosbag_options(self, path, storage_id, serialization_format='cdr'):
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format=serialization_format, output_serialization_format=serialization_format)
        return storage_options, converter_options
   
def main(args=None):
    rclpy.init(args=args)
    sbr = BagReaderNode()
    sbr.read_rosbag()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()