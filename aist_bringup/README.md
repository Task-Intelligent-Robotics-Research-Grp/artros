aist_bringup
==================================================

## Configuration and scene

## Configuration files
### Listing up hardware devices consisting the system

## Bring up hardware devices
```bash
ros2 launch aist_bringup launch.py [config:=<config>] [scene:=<scene>] [sim:=<sim>] [vis:=<vis>]
```
where
 - **config** -- Name of the hardware configuration (default: `aist`)
 - **scene** -- Name of the scene (default: '', i.e. empty string)
 - **sim** -- Launch `gz`, if `true` (default: `false`)
 - **vis** -- Launch `rviz2`, if `true` (default: `sim` value)
