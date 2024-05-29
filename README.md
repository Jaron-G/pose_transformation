### 已完成

#### 1.该包的功能：

+ 将姿态变换节点的通信方式改写为基于服务的通信

  服务端接收请求（相机坐标系下物体位姿，匹配的模型名称），向客户端返回抓取位姿
+ 添加姿态变换客户节点用来测试服务通信功能

  客户端向服务端发送请求（相机坐标系下物体位姿，匹配的模型名称），并打印出服务端处理后的抓取位姿
+ 服务的消息采用如下格式：

  ```
  float32[16] final_pose
  string matched_matched_model
  ---
  float32[6] grasp_pose
  ```
+ 功能包内包含原姿态变换文件作为模块供服务端调用
+ 抓取配置参数文件位于config文件夹下，可根据需求按照例子添加

#### 2.验证方法：

```
cd catkin_ws/
catkin_make
source devel/setup.bash 
```

+ 启动服务端和客户端并加载配置参数

```
roslaunch pose_transformation pose_transform.launch
```
