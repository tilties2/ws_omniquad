# ws_omniquad

The repo contains a series of nodes to interface with "Omniquad" drone presented in the paper "Design and Control of an Omnidirectional Aerial Robot with a Miniaturized Haptic Joystick for Physical Interaction".

More info on the mechanical design of the platform can be found here [Haptic-Omniquad](https://github.com/tilties2/Haptic-OmniQuad.git)
More info on the autopilot of Omniquad can be found here [PX4-Omniquad](https://github.com/tilties2/PX4-OmniQuad.git)

## Nodes

- `haptic_rc_ros2.py` interface joystick with ROS2
- `wrench_estimator.py` implementation of wrench estimator algorithm