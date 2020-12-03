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

1. \__init__

   初始化相关变量，没有进行实质操作。

2. clear_buff：

   csp非0的情况下清除规划的路径.

   在内部不调用。

3. build_frenet_path：

   通过中心路径生成csp变换。

   csp是空的时候，读入地图中的中心路径，转换成序列，插值，生成一个csp变换。

   在trajectory_update里面csp为空时调用被调用。

4. trajectory_update：

   调用initialize初始化。

   调用calculate_start_state计算起始状态：s位置，d位置，d速度，d加速度。

   调用frenet_optimal_planning，输入坐标变换，当前速度，轨迹初始状态。

   轨迹不空：

   - 提取xy赋给last_trajectory_array_rule
   - 提取本身赋给last_trajectory_rule，这个是为下一次的否则提供参考
   - 期望速度设置为s上的速度。

   否则如果上一次的last_trajectory_rule大于5，速度大于1：

   - trajectory_array为上一次的轨迹xy
   - generated_trajectory为上一次轨迹
   - 期望速度为0

   否则：

   - trajectory_array采用参考路径
   - 期望速度采用0

   轨迹(xy)和期望速度插值，存到msg中，返回msg。

   外部主要调用的函数。

5. initialize

   当前车位置到中心参考路径末端的距离。

   csp为空时调用build_frenet_path生成。

   调用predict，传入地图、障碍、自车信息，返回障碍物信息储存在self.obs_prediction中。

6. ref_tail_speed

7. calculate_start_state

   如果上一次last_trajectory_array_rule的len大于5，start_state的纵向位置、速度、加速度取自和当前位置最近的上次规划序列的点。

   否则，调用get_frenet_state(dynamic_map.ego_state, self.ref_path, self.ref_path_tangets)生成start_state。

   返回start_state。

   这里分类的目的是为了用多帧信息，生成的轨迹也更加光滑。

8. frenet_optimal_planning

   调用calc_frenet_paths，输入当前速度和轨迹起始位置，返回一系列轨迹。

   调用calc_global_paths，输入一系列frenet坐标下的轨迹，返回全局坐标下的轨迹。

   调用check_paths，输入全局坐标下的轨迹，输出速度、加速度、曲率有没有越界的轨迹。

   对剩下的轨迹代价值进行排序，用obs_prediction进行碰撞检测，返回第一个满足碰撞检测的轨迹。

   在trajectory_update中调用。

9. generate_target_course：读入插值后的路径中心点，生成csp变换，在build_frenet_path中调用。

10. calc_frenet_paths

    三重循环：

    - 目标处横向距离
    - 考虑时间长度，时间间隔是给定的，横向目标只有横向距离
    - 纵向的最终速度

    特别注意两个方向解方程的约束。

    从而生成一系列的轨迹(带有sd方向位置速度加速度时间)。

    在frenet_optimal_planning中调用。

> 第二重循环的目的是什么？为什么要调整预测总时长？

11. calc_global_paths

    把frenet坐标转换为世界坐标。



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





减小延时方面，上次新宇学长说check_collision的平方开根号可能算得比较慢，我们这里能把2-范数改成1-范数考虑吗？

延时补偿方面，我还没有很明确的想法。感觉系统每次延时不一样，可能每次采样生成道路的时间差不多，但是每次check_collision的执行次数可能有区别。如果我们假设每次决策的延时是一样的比如是$\Delta t$，那我能不能在物体预测和自车位置上都用运动学模型向前走$\Delta t$再进行决策？即便我们每次决策延时不一样，那我们通过记录时间戳也好，计算check_collision的次数也好，把这个延时一起交给决策有意义吗...

