import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory


# use_cases = {
# 0: "MODE_9_5FPS_2000  8+1, 5  FPS, max exposure time 2000 micro seconds",
# 1: "MODE_9_10FPS_1000 8+1, 10 FPS, max exposure time 1000 micro seconds",
# 2: "MODE_9_15FPS_700 8+1, 15 FPS, max exposure time  700 micro seconds",
# 3: "MODE_9_25FPS_450 8+1, 25 FPS, max exposure time  450 micro seconds",
# 4: "MODE_5_35FPS_600 4+1, 35 FPS, max exposure time  600 micro seconds",
# 5: "MODE_5_45FPS_500 4+1, 45 FPS, max exposure time  500 micro seconds",
# 6: "MODE_MIXED_30_5 Mixed mode: 30/5 FPS, max exposure time  300/1300 micro seconds",
# 7: "MODE_MIXED_50_5 Mixed mode: 50/5 FPS, max exposure time  250/1000 micro seconds",
# 8: "Low_Noise_Extended 5 FPS, Low_Noise_Extended",
# 9: "Fast_Acquisition 45 FPS, custom filter level",
# }

# exposure_modes = {
# 0: "MANUAL; Manual exposure mode",
# 1: "AUTOMATIC; Automatic exposure mode",
# }

# filter_levels = {
# 0: "Off, Turn off all filtering of the data (validation will still be enabled)",
# 200: "Legacy, Standard settings for older cameras",
# 255: "Full, Enable all filters that are available for this camera",
# 256: "Custom, Value returned by getFilterLevel if the processing parameters differ from all of the presets"
# }

def generate_launch_description():
    # rviz_config = os.path.join(get_package_share_directory('pico_flexx_driver_ros2'), 'rviz2', 'pico_flexx.rviz')
    # parameters_file = os.path.join(get_package_share_directory('pico_flexx_driver_ros2'), 'config', 'ros2_params.yaml')
    return LaunchDescription([
        Node(package='tf2_ros', 
             executable='static_transform_publisher',
             arguments=['0', '0', '0', '1.57', '0', '-1.57', 'pico_flexx_camera_link', 'pico_flexx_camera_optical_frame']),
        Node(package='pico_flexx_driver_ros2', 
             executable='pico_flexx_driver_ros2',
             output='screen',
             parameters=[
                {'base_name': "pico_flexx"},
                {'sensor': ""}, # ID of the use case. A list of supported use cases is listed on startup. 
                {'use_case': 7}, #  
                {'automatic_exposure':True}, # Enable or disable automatic exposure.
                {'exposure_time':1000}, # Exposure time. Only for manual exposure.
                {'exposure_mode':1}, # Exposure mode (0 = manual, 1 = automatic)
                {'automatic_exposure2':True}, # Enable or disable automatic exposure.
                {'exposure_time_stream2':1000}, # Stream 2 Exposure time. Only for manual exposure.
                {'exposure_mode_stream2':1}, # Stream 2 Exposure mode (0 = manual, 1 = automatic) 
                {'max_noise':0.07}, # Maximum allowed noise. Data with higher noise will be filtered out. 
                {'filter_level':200}, # Filter level (0 = Off, 200 = Legacy, 255 = Full) 
                {'range_factor':2.0}, # Range of the 16-Bit mono image which should be mapped to the 0-255 range of the 8-Bit mono image. The resulting range is `range_factor` times the standard deviation arround mean. 
                {'queue_size':5}, # Queue size for publisher. 
            ]),
        # Node(name='rviz2',
        #      package='rviz2',
        #      executable='rviz2',
        #      output='screen',
        #      arguments=[rviz_config]),

    ])

