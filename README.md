# PMD CamBoard pico flexx ROS2 Driver

This package was ported to ROS2 from the ROS1 package [here](https://github.com/code-iai/pico_flexx_driver).

## Dependencies

- Ubuntu 20.04
- ROS2 Foxy
- [Royale SDK](http://www.pmdtec.com/picoflexx/) (recommended: latest version, at least 3.23)

## Status

The driver has been tested on:
 - Ubuntu 20.04 
 - ROS2 Foxy
 - libroyale3.23.0

## Install

1. Install ROS2: [Instructions for Ubuntu 20.04](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/)
2. [ROS2 tutorials](https://index.ros.org/doc/ros2/Tutorials/)
3. Download the royale SDK from http://www.pmdtec.com/picoflexx/ and extract it.

4. Extract the archive that matches your kernel architecture from the extracted SDK. You can find out what your kernel architecture is by running `uname -m`.

5. Install the udev rules provided by the SDK

   ```
   sudo cp libroyale-*/driver/udev/10-royale-ubuntu.rules /etc/udev/rules.d/
   ```
6. Copy folders `bin`, `include` and `share` from the extracted folder and keep it inside `<catkin_ws>/src/pico_flexx_driver/royale`

7. Make sure that your user is in the `plugdev` group (you can use the command `groups` to check). If not, you need to add your user to the group:

   ```
   sudo usermod -a -G plugdev <your-user-name-here>
   ```

   Afterwards, you have to log out and log back in for the changes to take effect.


8. Run `./setup.sh` to create symlinks.

9. Run `colcon build --symlink-install` in the workspace folder.

## Usage

To start the pico flexx driver, please use the provided launch file:

`ros2 launch pico_flexx_driver_ros2 pico_flexx_driver.launch.py`

#### Parameters

The launch file has the following parameters:

- `base_name` (default="pico_flexx"):

  Name of the node. All topics will be advertised under this name.

- `sensor` (default=""):

  ID of the sensor that should be used. IDs of all connected devices are listed on startup.

- `use_case` (default="0"):

  ID of the use case. A list of supported use cases is listed on startup.

- `automatic_exposure` (default="true"):

  Enable or disable automatic exposure.

- `automatic_exposure_stream2` (default="true"):

  Enable or disable automatic exposure for stream 2.

- `exposure_time` (default="1000"):

  Exposure time. Only for manual exposure.

- `exposure_time_stream2` (default="1000"):

  Exposure time for stream 2. Only for manual exposure.

- `max_noise` (default="0.07"):

  Maximum allowed noise. Data with higher noise will be filtered out.

- `range_factor` (default="2.0"):

  Range of the 16-Bit mono image which should be mapped to the 0-255 range of the 8-Bit mono image. The resulting range is `range_factor` times the standard deviation around mean.

- `queue_size` (default="5"):

  Queue size for publisher.

#### Topics

1. /pico_flexx_camera_info_stream1
2. /pico_flexx_camera_info_stream2
3. /pico_flexx_depth_image_stream1
4. /pico_flexx_depth_image_stream2
5. /pico_flexx_gray_image_stream1
6. /pico_flexx_gray_image_stream2
7. /pico_flexx_point_cloud_stream1
8. /pico_flexx_point_cloud_stream2
9. /pico_flexx_update_fps_stream1
10. /pico_flexx_update_fps_stream2

<!-- When a mixed mode use case is selected, the second stream for all topics below
is published under the `stream2` namespace (e.g.,
`/pico_flexx/stream2/points`). In mixed mode, both a low-range, high-noise,
high-frequency point cloud and a high-range, low-noise, low-frequency (5 Hz)
point cloud are published. The 5 Hz point cloud in mixed mode only allows a
maximum exposure time of 1300 microseconds, so it has slightly higher noise
than the 5 Hz point cloud in single mode at 2000 microseconds. -->
<!-- 
##### `/pico_flexx/camera_info`
Bandwidth: 0.37 KB per message (@5 Hz: ~2 KB/s, @45 Hz: ~ 17 KB/s)

This topic publishes the camera intrinsic parameters.

##### `/pico_flexx/image_depth`
Bandwidth: 153.28 KB per message (@5 Hz: ~766 KB/s, @45 Hz: ~ 6897 KB/s)

This is the distorted depth image. It is a 32-Bit float image where each pixel is a distance measured in meters along the optical axis. -->

<!-- ##### `/pico_flexx/image_mono16`
Bandwidth: 76.67 KB per message (@5 Hz: ~383 KB/s, @45 Hz: ~ 3450 KB/s)
 -->
<!-- This is the distorted IR image. It is a 16-Bit image where each pixel is an intensity measurement. -->

<!-- ##### `/pico_flexx/image_mono8` -->
<!-- Bandwidth: 38.37 KB per message (@5 Hz: ~192 KB/s, @45 Hz: ~ 1727 KB/s) -->

<!-- This is the distorted IR image. It is a 8-Bit image where each pixel is an intensity measurement. -->

<!-- ##### `/pico_flexx/image_noise`
Bandwidth: 153.28 KB per message (@5 Hz: ~766 KB/s, @45 Hz: ~ 6897 KB/s)

This is the distorted noise image. It is a 32-Bit float image where each pixel is a noise value of the corresponding depth pixel (standard deviation, measured in meters).

##### `/pico_flexx/points`
Bandwidth: 720 KB per message (@5 Hz: ~3600 KB/s, @45 Hz: ~ 32400 KB/s)

This is the point cloud created by the sensor. It contains 6 fields in the following order: X, Y, Z, Noise (float), Intensity (16-Bit), Gray (8-Bit).
The 3D points themselves are undistorted, while the 2D coordinates of the points are distorted. The point cloud is organized, so that the each point belongs to the pixel with the same index in one of the other images. -->

