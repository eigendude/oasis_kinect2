import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='kinect2_bridge', 
            executable='kinect2_bridge',
            output='screen',
            parameters=[
                    
                        {'base_name':'kinect2'},
                        {'sensor':''},
                        {'publish_tf':'true'},
                        {'base_name_tf':'odom'},
                        {'fps_limit':'-1.0'},
                        #{'calib_path':'$(arg calib_path)'},
                        {'use_png':'false'},
                        {'jpeg_quality':'90'},
                        {'png_level':'1'},
                        {'depth_method':'default'},
                        {'depth_device':'-1'},
                        {'reg_method':'default'},
                        {'reg_device':'-1'},
                        {'max_depth':'12.0'},
                        {'min_depth':'0.1'},
                        {'queue_size':'5'},
                        {'bilateral_filter':'true'},
                        {'edge_aware_filter':'true'},
                        {'worker_threads':'4'},
                    
                ]
            ),

        # launch plugin through rclcpp_components container
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('rgb/camera_info', '/kinect2/sd/camera_info'),
                                ('rgb/image_rect_color', '/kinect2/sd/image_color_rect'),
                                ('depth_registered/image_rect',
                                 '/kinect2/sd/image_depth_rect'),
                                ('points', '/kinect2/sd/points')]
                ),
            ],
            output='screen',
        ),

    ])
