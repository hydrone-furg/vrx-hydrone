# vrx-hydrone

## Setup inicial:
### Passos:
Siga esses [passos](https://github.com/osrf/vrx/wiki/preparing_system_tutorial/be4182c59bdf4bc26040c99ae722b59e9bd20829) para instalação correta.

```
cd ~/vrx_ws/src
source /opt/ros/humble/setup.bash
cd ~/vrx_ws
colcon build --merge-install
. install/setup.bash
```

Para executar o **mundo: sydney_regatta**:
```
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```
