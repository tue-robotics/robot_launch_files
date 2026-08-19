# robot_launch_files
Generic TU/e Launch files

The launch files are ROS 2 Python launch files (`*.launch.py`), grouped by subsystem under
`launch/`. Launch them with `ros2 launch robot_launch_files <file>.launch.py`, for example:

```bash
ros2 launch robot_launch_files localization.launch.py laser:=base_laser map:=/path/to/map.yaml
```

Use `--show-args` to list the arguments a launch file accepts.

Most launch files read their parameters from `$ROBOT_BRINGUP_PATH/parameters/...`, so
`ROBOT_BRINGUP_PATH` has to point at the robot's bringup repository.

These files were migrated from ROS 1 XML launch files. The parameter files in the bringup
repositories, and several of the launched packages, still need changes to match; see
[docs/ros2_migration.md](docs/ros2_migration.md) for the full list.
