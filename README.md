### 已完成

#### 1.该包的功能：

+ 将姿态变换节点的通信方式改写为基于服务的通信

  服务端接收请求（相机坐标系下物体位姿，匹配的模型名称），向客户端返回抓取位姿
+ 添加姿态变换客户节点用来测试服务通信功能

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
+ 功能包内包含原姿态变换文件作为模块供服务端调用
+ 抓取配置参数文件位于config文件夹下，可根据需求按照例子添加，参数加载位置可通过修改launch文件实现，当前设置路径为grasp_icp/config

#### 2.该包功能单独验证方法：
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
+ 启动服务端和客户端并加载配置参数
  打开pose_transform.launch，将cilent节点解注释，并将参数路径更改为file="$(find  pose_transformation)/config/model_list.yaml"

```
roslaunch pose_transformation pose_transform.launch
```
