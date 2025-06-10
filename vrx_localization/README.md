## Localização:

### Passos:
```
cd ~/vrx_ws
colcon build --merge-install
source /opt/ros/humble/setup.bash
source install/setup.bash 
. install/setup.bash
```

Para executar o **localization_node**:
```
ros2 run vrx_localization localization_node
```
