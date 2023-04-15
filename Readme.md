# Navigate to anywhere

Stupid robot simulated driven through `/navigate_to_pose`

## Linting and style checking
* `ament_cpplint`
* `ament_uncrustify`


### msgs

Feeback
```yaml
current_pose:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  pose:
    position:
      x: 100.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
navigation_time:
  sec: 106
  nanosec: 914365716
estimated_time_remaining:
  sec: 0
  nanosec: 0
number_of_recoveries: 0
distance_remaining: 0.0
```

Command line to send goal
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: -2, y: -2}}}}" --feedback
```
