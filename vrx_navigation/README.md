## Navegação:
* Navegação via WASD;
* Ativação dos motores quando o nodo de localização está ativo;
* Navegação com GPS via waypoints definidos por coordenadas.

### Passos:
```
cd ~/vrx_ws
colcon build --merge-install
source /opt/ros/humble/setup.bash
source install/setup.bash 
. install/setup.bash
```

Para executar o **timed_thrust_node**:
```
ros2 run vrx_navigation timed
```

Para executar o **navigation_node_keyboard**:
```
ros2 run vrx_navigation waypoint_navigator
```

Para executar o **waypoint_navigator**:
```
ros2 launch vrx_navigation navigation.launch.py
```
