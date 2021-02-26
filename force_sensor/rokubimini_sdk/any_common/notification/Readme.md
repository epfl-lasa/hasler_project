## Notification

### Notifications for GUI Visualization
- Set always the `latch` option to TRUE for the `NotificationPublisher`
- Send a new notification only if the description/content is new and/or the level changed
- Remember to send an INFO notification if the WARN/ERROR/FATAL issue is resolved, otherwise the visualization won't turn green again

### Output Device Naming
Use a prefix `onboard_`, `joystick_` or `operator_` and add e.g. `screen` or `audio`. 
For example, to address a notification to the touchscreen on the robot use `onboard_screen`.

### Defined Output Devices
| Output device         | Description           |
| --------------------- | --------------------- |
| `operator_screen`     | Operator PC, used in `rqt_quadrupedstate_visualizer` |
| ...                   | ... |
