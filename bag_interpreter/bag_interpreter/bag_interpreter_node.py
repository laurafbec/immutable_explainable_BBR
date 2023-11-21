import rclpy
from rclpy.node import Node

import os
from pathlib import Path

import math
import rosbag2_py
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# Mapping dictionary to convert numeric log levels to string representations
LOG_LEVEL_MAP = {
    10: "DEBUG",
    20: "INFO",
    30: "WARNING",
    40: "ERROR",
    50: "FATAL",
}

# Mapping Nav2 BT nodes' names into a description
NAV2_BT_MAP = {
    "ComputePathToPose": "determines a viable path from a starting point to a specified target pose or location",
    "FollowPath": "tracks a specified path or trajectory",
    "RateController": "throttles the tick rate for its child",
    "NavigateWithReplanning": "performs the path planning and adjustment to adapt to changing environmental conditions in real-time",
    "NavigateRecovery": "recovers from unexpected situations"
}

# Mapping Nav2 BT nodes' status into a description
NAV2_BT_STATUS_MAP = {
    "IDLE": "is waiting to be executed.",
    "RUNNING": "is running.",
    "SUCCESS": "has succeeded.",
    "FAILURE": "has failed."
}

class BagInterpreterNode(Node):

    DISTANCE_THRESHOLD = 1.2 #(20%)
    OBSTACLE_DISTANCE_THRESHOLD = 1.4 #1.4 m

    def __init__(self):

        # Initialize the node
        super().__init__('bag_interpreter_node')

        # Initialize variables
        self.previous_goal_id = None
        self.finished_goals = []
        self.n_goal = 0
        self.nav_status_log_message = "No navigation is running. "

        self.last_scan = ""
        self.previous_distance = float('inf')
        self.changed_route_log_message = "Planned path has not changed. "
        self.new_plan = 0

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
            self.log_output_file_path = "interpreter_data.txt"
        else:
            self.log_output_file_path = log_output_file_path
        
        # Get Rosbag path URI
        self.uri = os.path.join(rosbag_dir, rosbag_file_path)       

        # Create a directory to store the log file
        self.log_dir = log_output_dir
        Path(self.log_dir).mkdir(parents=True, exist_ok=True)

    # Process messages from /rosout topic
    def extract_rosout_info(self, t, msg, logfile):
        timestamp = int(t / 10**9)
        log_level = LOG_LEVEL_MAP.get(msg.level, "UNKNOWN")
        message = msg.msg
        file_location = msg.file
        function_name = msg.function
        line_number = msg.line
        log_message = f"{timestamp} Log level: {log_level} Message: {message} File location: {file_location} Function name: {function_name} Line number: {line_number}"
                   
        self.get_logger().info(log_message)
        logfile.write(log_message + "\n")

    # Process messages from /behavior_tree_log topic
    def extract_behavior_tree_info(self, t, msg, logfile):
        timestamp = int(t / 10**9)
        additional_info = []
        for event in msg.event_log:
            event_ts_s = event.timestamp.sec
            event_ts_ns = event.timestamp.nanosec
            node_name = event.node_name
            previous_status = event.previous_status
            current_status = event.current_status
            node_description = NAV2_BT_MAP.get(node_name, node_name)
            current_status_description = NAV2_BT_STATUS_MAP.get(current_status, current_status)
            additional_info.append(f"Nav2 Behavior Tree node {node_name} that {node_description}, {current_status_description}")

        if additional_info:
            log_message = f"{timestamp} {' '.join(additional_info)}"
            self.get_logger().info(log_message)
            logfile.write(log_message + "\n")

    # Process messages from /amcl_pose topic
    def extract_amcl_pose_info(self, t, msg, logfile):
        timestamp = int(t / 10**9)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        log_message = f"{timestamp} Position: {x}, {y}. Orientation: {z},{w}"
        self.get_logger().info(log_message)
        logfile.write(log_message + "\n")

    # Process messages from NavigateToPose action status
    def extract_navigateToPose_action(self, t, msg, logfile):
        self.nav_status_log_message = ""
        timestamp = int(t / 10**9)
        for status in msg.status_list:
            current_goal_id = status.goal_info.goal_id
            if current_goal_id != self.previous_goal_id and current_goal_id not in self.finished_goals:
                self.previous_goal_id = current_goal_id
                self.n_goal += 1
                self.nav_status_log_message = f"{timestamp} Navigation to the goal number {self.n_goal} has started."
                logfile.write(self.nav_status_log_message + "\n")
            if status.status in [2, 4, 5, 6] and current_goal_id not in self.finished_goals:
                status_dict = {
                    2: "is in progress",
                    4: "has succeeded",
                    5: "was cancelled",
                    6: "has aborted"
                }
                self.nav_status_log_message = "{} Navigation to the goal number {} {}. ".format(timestamp, self.n_goal, status_dict[status.status])
                if status.status in [4, 5, 6]:
                    self.finished_goals.append(current_goal_id)
        if not self.nav_status_log_message:
            self.nav_status_log_message = f"{timestamp} No navigation is running. "
        
        if self.nav_status_log_message:
            self.get_logger().info(self.nav_status_log_message)
            logfile.write(self.nav_status_log_message + "\n")
        
    # Get last laser scan message
    def extract_scan_action_info(self, t, msg, logfile):
        self.last_scan = msg

    # Process messages from /plan topic. Used to detect a change in a planned trajectory
    def extract_plan_info(self, t, msg, logfile):
        total_distance = 0
        self.changed_route_log_message = ""
        timestamp = int(t / 10**9)

        for i in range(len(msg.poses) -1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i+1].pose.position
            distance = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
            total_distance += distance
        
        # Euclidean distance checkings to detect a change in the planned trajectory
        if total_distance > self.previous_distance * BagInterpreterNode.DISTANCE_THRESHOLD:
            self.changed_route_log_message = f"{timestamp} Planned path has changed when trying to achieve goal pose number {self.n_goal}"
            if min(self.last_scan.ranges) < BagInterpreterNode.OBSTACLE_DISTANCE_THRESHOLD:
                self.changed_route_log_message += " because there was an obstacle."
            else:
                self.changed_route_log_message += " due to an unknown reason."
            self.new_plan = 1
        elif self.new_plan == 1:
            self.changed_route_log_message += f"{timestamp} The trajectory has been replanned in order to achieve the goal pose."
            self.new_plan = 0

        self.previous_distance = total_distance

        if self.changed_route_log_message:
            self.get_logger().info(self.changed_route_log_message )
            logfile.write(self.changed_route_log_message + "\n")

    # Process messages from /cmd_vel topic
    def extract_cmd_vel_info(self, t, msg, logfile):
        timestamp = int(t / 10**9)
        log_message = f"{msg}"
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z
        log_message = f"{timestamp} Linear velocity: {x}, {y}. Angular velocity: {z}"
        self.get_logger().info(log_message)
        logfile.write(log_message + "\n")      
        

    # Read Rosbag file
    def read_rosbag(self):
        storage_options, converter_options = self.get_rosbag_options(self.uri, storage_id='mcap')

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

        extraction_functions = {
            "rosout": self.extract_rosout_info,
            "behavior_tree_log": self.extract_behavior_tree_info,
            "amcl_pose": self.extract_amcl_pose_info,
            "navigate_to_pose/_action/status": self.extract_navigateToPose_action,
            'scan': self.extract_scan_action_info,
            'plan': self.extract_plan_info,
            'behavior_tree_log': self.extract_behavior_tree_info,
            'cmd_vel': self.extract_cmd_vel_info            
        }

        with open(log_file, 'w') as logfile:
            while reader.has_next():
                (topic, data, t) = reader.read_next()
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)

                extraction_function = extraction_functions.get(topic)
                if extraction_function:
                    extraction_function(t, msg, logfile) 

    # Get Rosbag options
    def get_rosbag_options(self, path, storage_id, serialization_format='cdr'):
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format=serialization_format, output_serialization_format=serialization_format)
        return storage_options, converter_options
   
def main(args=None):
    rclpy.init(args=args)
    sbr = BagInterpreterNode()
    sbr.read_rosbag()
    rclpy.shutdown()


if __name__ == '__main__':
    main()