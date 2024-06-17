
#### 1.该包的功能：

+ 姿态变换节点的通信方式为基于服务的通信

  服务端`pose_transformation_server.py`接收请求（相机坐标系下物体位姿，匹配的模型名称），向客户端返回抓取位姿
+ 姿态变换客户节点`pose_transformation_client.py`用来测试服务通信功能

  客户端向服务端发送请求（相机坐标系下物体位姿，匹配的模型名称），并输出服务端处理后的抓取位姿
+ 服务的消息采用如下格式：

  ```
  float32[16] final_pose
  string matched_matched_model
  ---
  float32[9] rotation_matrix
  float32[6] grasp_pose
  float32[6] up_pose
  ```
#### 2.必要参数文件说明：
+ 手眼变换矩阵位于config文件夹下，根据实际相机安装位置进行替换
  ```
  0	-1	0	300
  -1	0	0	-800
  0	0	-1	1000
  0	0	0	1
  ```
+ 抓取配置参数文件位于config文件夹下，可根据需求按照样例添加，参数加载位置可通过修改launch文件实现，当前设置路径为`grasp_icp/config`，若对该包进行单独测试，则将launch文件路径修改为`file="$(find  pose_transformation)/config/model_list.yaml"`
  ```yaml
  pose_parameters:
    t_pipe_u: {t_vec: [[14], [20], [-165]], t_vec_up: [[14], [20], [-280]], r_vec: [[0], [0], [0]]}
    pipe_u: {t_vec: [[20], [27.5], [-165]], t_vec_up: [[20], [27.5], [-280]], r_vec: [[0], [0], [1.5708]]}
    L_pipe_u: {t_vec: [[60], [40], [-165]], t_vec_up: [[60], [40], [-280]], r_vec: [[0], [0], [0]]}
  ```
  + t_vec:为相对于模型原点的抓取位置
  + t_vec_up:为抓取物体后提升的位置，防止机器人移动物体时发生碰撞
  + r_vec:为相对于模型原点的抓取姿态


#### 3.该包功能单独验证方法：
+ clone this repository to catkin_ws/src.
+ 启动测试环境
  ```bash
  cd pose_transformation/Docker
  bash noetic.bash 
  ```
+ 编译功能包
  ```bash
  cd catkin_ws/
  catkin_make
  source devel/setup.bash 
  ```
+ 启动位姿变换服务端并加载抓取配置参数
  打开pose_transform.launch，并将参数路径更改为`file="$(find  pose_transformation)/config/model_list.yaml"`
  ```bash
  # in the container
  roslaunch pose_transformation pose_transform.launch
  ```
+ 启动位姿变换客户端
  ```bash
  # open another terminal
  rosrun pose_transformation pose_transformation_client.py
  ```
