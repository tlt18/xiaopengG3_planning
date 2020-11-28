# zzz代码解读planning/decision

## continuous_models

### nodes

decision_node

头文件

```python
from zzz_common.params import parse_private_args
from zzz_cognition_msgs.msg import MapState
from zzz_planning_msgs.msg import DecisionTrajectory
from nav_msgs.msg import Path 
from visualization_msgs.msg import Marker, MarkerArray
from zzz_planning_decision_continuous_models import MainDecision
# 采样选择路线
from zzz_planning_decision_continuous_models.Werling.Werling_planner import Werling
from zzz_planning_decision_continuous_models.follow import Follow_Ref_Path
```

Werling作为MainDecision的一个参数

```python
model_trajectory = Werling()
self._decision_instance = MainDecision(trajectory_planner=model_trajectory)
```

接收到params.dynamic_map_topic就回调MainDecision.receive_dynamic_map()

这个params.dynamic_map_topic就是"/zzz/cognition/local_dynamic_map/map_with_ref"

```python
self._dynamic_map_subscriber = rospy.Subscriber(params.dynamic_map_topic, MapState, self._decision_instance.receive_dynamic_map)
```

在循环中调用MainDecision.update_trajectory()

```python
def loop(self):
    publish_msg = self._decision_instance.update_trajectory()
```

其中MainDecision在zzz_planning_decision_continuous_models的main.py中，

Werling()在zzz_planning_decision_continuous_models/Werling/Werling_planner.py中。



>接收receive_dynamic_map和发送DecisionTrajectory的时序问题



### main.py

receive_dynamic_map接收dynamic_map，topic来自于params.dynamic_map_topic，进入之后只是更新self._dynamic_map_buffer。

```python
def receive_dynamic_map(self, dynamic_map):
    self._dynamic_map_buffer = dynamic_map
```

update_trajectory调用self.\_trajectory_planner.trajectory_update，self._trajectory_planner实际上就是Werling()，还是看Werling()。

```python
def update_trajectory(self, close_to_junction=40):
        if self._dynamic_map_buffer is None:
            return None
        dynamic_map = self._dynamic_map_buffer
        # 多车道模型并且距离太远，清除
        if dynamic_map.model == dynamic_map.MODEL_MULTILANE_MAP and           dynamic_map.mmap.distance_to_junction > close_to_junction:
            self._trajectory_planner.clear_buff(dynamic_map)
            return None
        # 多车道模型，建立frenet_path，也不进入主体部分
        elif dynamic_map.model == dynamic_map.MODEL_MULTILANE_MAP:
            msg = self._trajectory_planner.build_frenet_path(dynamic_map)
            return None
        # trajectory_update输入参数dynamic_map
        else:
            return self._trajectory_planner.trajectory_update(dynamic_map)
```



### Werling

#### 相关博客

[Frenet坐标系局部路径规划器](https://blog.csdn.net/u010918541/article/details/105054491/)

[机器人局部避障的动态窗口法](https://blog.csdn.net/heyijia0327/article/details/44983551)

[基于Frenet优化轨迹的无人车动作规划方法](https://blog.csdn.net/adamshan/article/details/80779615)

#### Werling_planner.py

传入的是"/zzz/cognition/local_dynamic_map/map_with_ref"的MapState。

##### class Werling():

包含

\__init__：初始化相关变量，没有进行实质操作。

clear_buff：csp非0的情况下清除规划的路径。

build_frenet_path

trajectory_update

initialize

ref_tail_speed

calculate_start_state

frenet_optimal_planning

generate_target_course

calc_frenet_paths

calc_global_paths



clear_buff()重新初始化

```python
def clear_buff(self, dynamic_map):
```

build_frenet_path()调用generate_target_course生成参考线，generate_target_course用的是二维立方样条插值。

```python
def build_frenet_path(self, dynamic_map,clean_current_csp = False):
    if self.csp is None or clean_current_csp:
       self.reference_path = dynamic_map.jmap.reference_path.map_lane.central_path_points
       ref_path_ori = convert_path_to_ndarray(self.reference_path)
       self.ref_path = dense_polyline2d(ref_path_ori, 2)
       self.ref_path_tangets = np.zeros(len(self.ref_path))
       self.ref_path_rviz = convert_ndarray_to_pathmsg(self.ref_path)

       Frenetrefx = self.ref_path[:,0]
       Frenetrefy = self.ref_path[:,1]
       tx, ty, tyaw, tc, self.csp = self.generate_target_course(Frenetrefx,Frenetrefy)
```

外界调用trajectory_update更新参考路径。

trajectory_update()

1. initialize(dynamic_map)
   - 计算到ref路径末端的dist放在self.dist_to_end中
   - 估计frenet frame，调用build_frenet_path(dynamic_map)
   - 初始化预测模块
2. self.calculate_start_state(dynamic_map)
   - 返回当前车辆的状态
   - len(self.last_trajectory_array_rule) > 5:
     - last_trajectory_rule中最近的点作为start_state
   - 否则:
     - 用get_frenet_state生成一个轨迹
3. self.frenet_optimal_planning(self.csp, self.c_speed, start_state)
   - fplist = self.calc_frenet_paths(c_speed, start_state)存frenet_paths，包括路径导数和代价。
   - fplist = self.calc_global_paths(fplist, csp)添加全局路径参数.x .y .yaw .ds
   - 检查路径
   - sorted对所有路径排序
   - 检查第一个没有遇到障碍物的路径，返回
   - 对规划路径分割，还对预期速度进行控制



## TODO

predict

采样

读文章

需要改的部分：

1. Werling_planning.py/calc_frenet_paths

2. predict.py/check_collision