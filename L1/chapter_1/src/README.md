#### Dependencies
- pip (make sure the version of python2), refer to [blog](https://blog.csdn.net/weixin_41718085/article/details/83882445?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param) for how to switch the version
- numpy `sudo pip install -U numpy`
- kitti2bag `sudo pip install kitti2bag`

#### Run guide
1. Q1:
```asm
roslaunch lidar_localization test_frame.launch
cd [bag_path]
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```

2. Q2:

modify the `registration_method` to `ICP` in `config.yaml`
```asm
roslaunch lidar_localization front_end.launch
cd [bag_path]
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```

3. Q3:

modify the `registration_method` to `NDTOMP` in `config.yaml`
```asm
roslaunch lidar_localization front_end.launch
cd [bag_path]
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```




===========================================================================================
#### 1.这个项目是干嘛的
我正在知乎上写一个从零开始做自动驾驶定位的系列博客，这个工程就是博客的配套源代码。  
博客专栏地址：https://zhuanlan.zhihu.com/c_1114864226103037952

#### 2.这些代码咋用啊
每篇博客更新后，如果配套有代码更新，我会在关键节点版本上添加tag，同时会在博客里注明当篇博客对应的的版本tag，这样博客的阅读进度和代码的进度就能够对应上

#### 3.调试环境
- ubuntu 16.04
- ros kinectic
- pcl 1.7
- glog

这是我个人的调试环境，也没在别的环境上测试过，所以前期可能会暴露不少问题，如果遇到，各位可以在本工程里提issue，也可以在对应的博客下面的评论区留言。

#### 4.测试数据
开源程序最好使用开放数据集，所以我们选择了KITTI，并且把RawData里的"2011_10_03_drive_0027_sync"做成了bag文件，后面所有程序的测试都是在这个bag基础上做的。
数据文件我放在了百度网盘里，
