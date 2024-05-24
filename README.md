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

+ 功能包内仍包含原姿态变换文件，用来核对基于服务通信的抓取姿态

#### 2.验证方法：
```
cd catkin_ws/
catkin_make
source devel/setup.bash 
```
启动ROS

```
roscore
```

另开两个终端启动服务端和客户端：

```
rosrun pose_transformation_pkg pose_transformation_server.py
```

```
rosrun pose_transformation_pkg pose_transformation_cilent.py
```

直接运行姿态变换：

```
cd catkin_ws/src/pose_transformation_pkg/src
python3 PoseTransformation.py
```

客户端输出的抓取位姿应与直接运行的抓取位姿相同。