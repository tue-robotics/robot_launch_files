# ROS 2 migration

The launch files in this package were converted from ROS 1 XML (`*.launch`) to ROS 2 Python
(`*.launch.py`), and the package build type changed from `catkin` to `ament_cmake`.

The launch files themselves are complete, but a launch file is only half of the contract. This
document lists what still has to change **outside this repository** before the launched system runs.

## 1. Parameter files in the robot bringup repositories

ROS 1 `<rosparam command="load" file="..."/>` loaded a flat YAML file into a node's private
namespace. ROS 2 loads parameter files per node, and every file needs a node-name key and a
`ros__parameters` key:

```yaml
# before (ROS 1)
some_parameter: 42

# after (ROS 2)
/**:                      # or the fully qualified node name
  ros__parameters:
    some_parameter: 42
```

`/**:` matches any node, which is the closest equivalent to the ROS 1 behaviour. Use the fully
qualified node name instead when one file is shared by several nodes that need different values.

Every file below is referenced from a launch file in this package and needs that change.
`<name>` is the value of the launch file's `name` argument, so those entries exist once per
camera, laser or arm.

| Parameter file (relative to `$ROBOT_BRINGUP_PATH/parameters`) | Change |
| --- | --- |
| `diagnostics/aggegrator.yaml` | wrap only |
| `diagnostics/node_alive_neglect_nodes.yaml` | wrap only |
| `essentials/robot_state_publisher.yaml` | wrap only |
| `hardware/actuators/dynamixel_joint_controllers.yaml` | wrap only |
| `hardware/managers/battery_parameters.yaml` | wrap only |
| `hardware/sensors/<name>.yaml` | wrap only |
| `hardware/sensors/<name>_filters_parameters.yaml` | wrap only |
| `hardware/sensors/<name>_parameters.yaml` | wrap only |
| `hardware/sensors/head_camera_laser_projection.yaml` | wrap only |
| `interaction/mobile_ui_server.yaml` | wrap only |
| `interaction/speech_client.yaml` | wrap only |
| `interaction/text_to_speech.yaml` | wrap only |
| `localization/amcl.yaml` | wrap only |
| `localization/gmapping.yaml` | wrap only |
| `manipulation/<name>/grasp_precompute.yaml` | wrap only |
| `manipulation/head_cmd_vel_client.yaml` | wrap only |
| `manipulation/head_server.yaml` | wrap only |
| `multirobot/trigger_client.yaml` | wrap only |
| `multirobot/trigger_server.yaml` | wrap only |
| `multirobot/world_model_client_bridge.yaml` | wrap only |
| `multirobot/world_model_server_bridge.yaml` | wrap only |
| `navigation/global_costmap.yaml` | wrap **and** nest under `global_costmap:` |
| `navigation/global_costmap_gmapping.yaml` | wrap **and** nest under `global_costmap:` |
| `navigation/global_planner.yaml` | wrap only |
| `navigation/local_costmap.yaml` | wrap **and** nest under `local_costmap:` |
| `navigation/local_costmap_gmapping.yaml` | wrap **and** nest under `local_costmap:` |
| `navigation/local_planner.yaml` | wrap only |
| `navigation/navscan.yaml` | wrap only |
| `world_modeling/face_recognition.yaml` | wrap only |

### The costmap files need more than wrapping

ROS 1 could load a file into a sub-namespace of a node:

```xml
<rosparam command="load" file=".../local_costmap.yaml" ns="local_costmap" />
```

ROS 2 has no per-node parameter namespaces, so the `ns` attribute cannot be expressed in the
launch file. The nesting has to move into the YAML itself:

```yaml
/**:
  ros__parameters:
    local_costmap:            # <- the former ns attribute
      recent_obstacles:
        observation_sources: base_laser
```

`navigation.launch.py` and `navigation_gmapping.launch.py` set
`local_costmap.recent_obstacles.observation_sources` in simulation, which only takes effect once
the costmap files are nested this way.

## 2. Credential files

`picovoice.launch.py`, `slack.launch.py` and `telegram.launch.py` load their credentials as ROS 2
parameter files, so these need the same wrapper:

- `~/MEGA/credentials/picovoice/access_key_<robot_name>.yaml` and `access_key_test.yaml`
- `~/MEGA/credentials/slack/token_<robot_name>.yaml` and `token_test.yaml`
- `~/MEGA/credentials/telegram/token_<robot_name>.yaml` and `token_test.yaml`

```yaml
/**:
  ros__parameters:
    token: "API_TOKEN"
```

## 3. Parameter files owned by other packages

- `people_recognition_3d`'s `config/config.yaml`, loaded by `people_recognition.launch.py`.
- Whatever file is passed as `param_file` to `vizbox.launch.py`.

## 4. Launch arguments that changed

Callers of these launch files have to be updated.

| Launch file | Change |
| --- | --- |
| all | `machine` removed; ROS 2 launch has no remote machine support, so the `.machine` includes are gone |
| `localization/localization.launch.py` | `map` added and required; it was never forwarded to `amcl.launch.py`, so both localization types failed to start in ROS 1 as well |
| `localization/gmapping.launch.py` | `sensor` renamed to `laser`, so both localization types take the same arguments |
| `hardware/sensors/depth_to_laser.launch.py` | `camera_laser_transform` split into `camera_laser_x`, `_y`, `_z`, `_roll`, `_pitch`, `_yaw`, because ROS 2 `static_transform_publisher` takes named arguments |
| `simulator/simulator.launch.py` | `gdb` replaced by `launch_prefix`, which is passed straight to `Node(prefix=...)` |
| `hardware/sensors/kinect.launch.py` | `gdb` removed; it was declared but never used |
| `interaction/text_to_speech.launch.py` | `machine-tts` and `machine-play` removed |
| `interaction/speech_recognition.launch.py` | `qr_decoder_image_topic` default changed from `~image` to `~/image`, the ROS 2 private-name syntax |

## 5. Changes needed inside launched packages

- **`philips_text_to_speech`** must read its key file itself. ROS 1 `<param textfile="..."/>` inlined
  the file contents into a parameter; ROS 2 has no equivalent, so `text_to_speech.launch.py` passes
  the path in a `key_file` parameter instead.
- **`robot_state_publisher`** reads `robot_description` as a node parameter. ROS 1 relied on the
  global parameter server, which ROS 2 does not have, so
  `essentials/robot_state_publisher.yaml` has to provide it.
- **`map_server`** took the map as a positional argument. Its ROS 2 successor `nav2_map_server`
  takes a `yaml_filename` parameter, so `amcl.launch.py` needs updating together with that port.

## 6. Packages referenced but not built for ROS 2

These are launched by name and will fail with `package '<name>' not found` until they are ported.
The launch files keep the ROS 1 package and executable names, with a `TODO(ros2)` comment naming
the replacement where a ROS 2 native one exists.

- `amcl`
- `battery_manager`
- `depthimage_to_laserscan`
- `depthimage_to_navscan_rgbd`
- `diagnostic_aggregator`
- `dragonfly_speech_recognition`
- `dynamixel_controllers`
- `emergency_speakup`
- `fast_simulator`
- `fast_simulator_data`
- `gmapping`
- `handle_locator`
- `head_ref`
- `hmi`
- `image_recognition_age_gender`
- `image_recognition_color_extractor`
- `image_recognition_face_recognition`
- `image_recognition_tensorflow`
- `kinect_driver`
- `laser_filters`
- `map_server`
- `multirobot_communication`
- `node_alive`
- `people_recognition_2d`
- `people_recognition_3d`
- `picaso_4d_systems`
- `picovoice_driver`
- `rgb_lights_manager`
- `rtt_control_components`
- `rtt_ros`
- `slack_ros`
- `speech_recognition`
- `telegram_ros`
- `test_tools`
- `text_to_speech`
- `tf_server`
- `tue_manipulation`
- `tue_mobile_ui`
- `urg_node`
- `vizbox`

Known ROS 2 native replacements:

| ROS 1 | ROS 2 |
| --- | --- |
| `map_server` | `nav2_map_server` |
| `amcl` | `nav2_amcl` |
| `gmapping` | `slam_toolbox` |
| `rtt_ros`, `rtt_control_components` | none; Orocos RTT is ROS 1 only |

## 7. Scripts

`script/` still contains `rospy` based nodes. They are installed into `lib/robot_launch_files`, so
`Node(executable=...)` resolves them, but they need a separate port to `rclpy` before they run.
This affects `joke.py`, `scan_gmapping.py`, `ssl_dummy` and `rqt.bash`, which are referenced from
launch files, plus `bodypart_resetter.py`, `emoticon_handler` and `topic_monitor`, which are not.

`rqt.bash` also still copies the rviz config to `~/.rviz`, which ROS 2 rviz does not read; it
should be `~/.rviz2`.

## 8. tue-env target

CI and `tue-get` resolve this package through the `ros-robot_launch_files` target in
[tue-env-targets](https://github.com/tue-robotics/tue-env-targets), which had the flat
pre-migration form:

```yaml
- type: ros
  source:
    type: git
    url: https://github.com/tue-robotics/robot_launch_files.git
```

Every migrated package splits the target so that ROS 2 distros track `master` while Noetic keeps
tracking the ROS 1 code on a `ros1` branch, as `ros-ed` and `ros-upower_ros` do:

```yaml
- type: ros
  default:
    source:
      type: git
      url: https://github.com/tue-robotics/robot_launch_files.git
  noetic:
    source:
      type: git
      url: https://github.com/tue-robotics/robot_launch_files.git
      version: ros1
```

Without that split, a Noetic workspace picks up the ROS 2 launch files from `master`.

Both halves are in place: the `ros1` branch of this repository points at `cf7b017`, the last commit
before the migration, and tue-env-targets
[#558](https://github.com/tue-robotics/tue-env-targets/pull/558) applies the split. Keep the `ros1`
branch as long as Noetic is supported.

CI itself runs on the `jazzy` and `rolling-u24` images, matching `ed` and `rgbd`. Humble is left
out on purpose: `ed` and `rgbd` are launched by this package and are not built for Humble, so a
Humble job would test a distro on which these launch files cannot run. All launch substitutions
used here do exist on Humble, so adding `humble` back to the matrix is a one-line change.

## Note for linting this package

Running `ruff` from the repository root makes its isort rule treat `launch` as a first-party
module, because this repository has a `launch/` directory. Set `known-third-party = ["launch"]`
when wiring up a ruff configuration here. The `INP001` rule also does not apply: ROS 2 launch
directories intentionally have no `__init__.py`.
