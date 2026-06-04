aist_routines
==================================================

## Base task
You can launch most basic servers commonly required for any tasks with the following command,
```bash
ros2 launch aist_routines base.launch.py [config:=<config>] [settings_file:=<setting_file>] [sim:=<sim>] [vis:=<vis>]
```
where
 - **config** -- Name of the hardware configuration (default: `aist`)
 - **setting_file** -- Name of the file which defines settings for various servers including `collision_object_manager` (default: [default.yaml](./config/default.yaml))
 - **sim** -- Launch camera drivers, if `false` (default: `false`)
 - **vis** -- Launch `rviz2`, if `true` (default: not of `sim` value)
