import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   realsense_camera = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense2_camera'), 'launch'),
         '/rs_get_params_from_yaml_launch.py']),
      launch_arguments={'camera_name': 'D435',
      			'camera_namespace': 'robot', 
                        'config_file': '/root/ros2_ws/src/realsense-ros/realsense2_camera/config/config.yaml'}.items(),
      )
   
   img_process = Node(
	    package='image_processing',
	    namespace='image_processing',
	    executable='image_processing',
	    name='image_processing',
	)
   
   udp_cm = Node(
	    package='udp_cm',
	    namespace='udp_cm',
	    executable='udp_cm',
	    name='udp_cm',
	)
	
   dxl = Node(
	    package='drive_dynamixel',
	    namespace='dynamixel_controller',
	    executable='dynamixel_controller',
	    name='dynamixel_controller_node'
	)
	
   serial = Node(
	    package='serial_cm',
	    namespace='serial_cm',
	    executable='serial_cm',
	    name='serial_cm_node'
	)
   
   qt6_gui = Node(
	    package='qt6_gui',
	    namespace='qt6_gui',
	    executable='qt6_gui',
	    name='qt6_gui_node',
	    # arguments=['--ros-args', '--log-level', 'debug']
	)
   
   return LaunchDescription([
      realsense_camera,
      img_process,
      udp_cm,
      dxl,
      serial,
      qt6_gui
   ])
