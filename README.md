MVSROS_project
----

主要文件介绍
---

***MVS_ROS2/hik_camera*** 

  **config**
  
  存放的是相机参数的配置文件camera_params.yaml, 可以通过修改该文件来改变搜索相机ip号或sn号，以及相机读取图片时的参数设置（包括exposure_time， gain，frame_rate， pixel_format）。
  
  
  **include/hik_camera**
  
  这里的hik_camera_driver.hpp是存放声明参数与函数的头文件。
  
  
  **launch**
  
  构建pkg，启动节点。
  
  
  **src**
  
  1.hik_camera_driver.cpp:定义初始化设备与收集图像相关功能的cpp文件。
  
  2.hik_camera_node.cpp：主节点。
  
  
  **CMakeLists.txt**
  
  指导节点编译。
  
  
  **packege.xml**
  
  包含对pkg描述，作者信息以及所需依赖。


***MVS_SDK/SDK***

  这里是海康威视MVS_SDK的C++原文件，本项目就是基于该SDK的二次开发。


编译配置节点
---
最开始你要打开config/camera_params.yaml,并修改你实际的参数，如果不知道参数可以通过命令行来查找。（这里不多赘述）
（这里所使用的为zsh。）

首先，先进入到MVSROS_project的目录：

```
cd ~/zbx/MVSROS_project/MV
```

然后在这里source你的ros2，以我为例:

```
source /opt/ros/humble/setup.zsh   
```

接着source install文件里面的setup.py:

```
source install/setup.zsh 
```

然后就可以启动构建pkg了:

```
colcon build --packages-select hik_camera
```
(这里会弹出一些警告，忽视即可)

最后运行节点：

```
ros2 run hik_camera hik_camera_node
```

之后节点就会开始运行。

接收和查看节点
---
新建一个终端，然后先source你的ros2：

```
source /opt/ros/humble/setup.zsh
```

首先可以打开node list查看是否节点正在运行：

```
ros2 node list
```

然后查看是否有名为hik_camera的node。

之后查看话题：

```
ros2 topic list
```

然后查看是否有名为image_raw的话题。

这是如果你查看image_raw的话题的话：

```
ros2 topic echo /image_raw
```

会看到一串数字编码，说明相机正在向你的电脑传输图片数据。

之后在终端输入：

```
ros2 run rqt_image_view rqt_image_view
```

便会弹出一个GUI窗口，只需要在窗口下拉的菜单中找到image_raw的话题然后点击，图像就会实时显示在窗口了。



