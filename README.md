# CSM-LIO

Update:

2023-01-28：请注意此仓库当前代码仅为过程版本，功能开发和环境依赖都远未完成，勿测。   
**2023-02-12：算法的完整功能已实现，但尚未测试环境配置，您可以测试和提issue。**

## 1. 简介（Introduction）

**CSM-LIO** 是以相关性扫描匹配算法（Correlative Scan Match, CSM）为基础构建的松耦合激光惯性里程计，该项目在 Cartographer(3D) 的基础上构建，但对代码结构做了极大精简，表现在：(1) 解除了类的层层嵌套、直达算法实现，(2) 所有参数从yaml文件中加载、摒弃了通过lua脚本语言解析参数的复杂做法，(3) 整个工程完全剥离了对protobuf的依赖、更加清爽，(4) 支持在rviz上可视化 3D submap。同时，CSM-LIO 保持了 Google 的代码风格和合理的代码结构。

CSM-LIO 具有以下特性：   
- 计算效率高，32线机械式激光雷达单帧处理时间约为40ms；    
- 鲁棒性好，得益于算法原理和 Google 的良好实现，不容易跑飞，退化情况相对较少；    
- 兼容性极佳，天然能够 work with 任何模态的 LiDAR，包括机械旋转式、非重复扫描式、其它类固态/固态式等。

CSM 的算法原理是实现这些特性的关键，作为一种 scan to probability-grid 的配准算法，CSM 无需对 scan 抽取任何特征，一方面节约了时间，另一方面对点云的分布形态无要求，因此能够兼容不同扫描方式（模态）的 LiDAR。

CSM-LIO 目前已经测试了机械旋转式16线 & 32线、Livox（览沃）Mid360 & Horizon & XX、...等。下方是机械旋转式 LiDAR 的测试截图。      

<img src=docs/homemadedataset_06.png width=80% />

[TODO:放一个视频在这]()


## 2. 工程架构（Architecture） //TODO
CSM-LIO 基本保留了 Cartographer 的前端算法流程（LocalTrajectoryBuilder部分），但代码结构方面稍有不同，因为 CSM-LIO 是作为我个人的SLAM工程（Infinity-SLAM）的一个模块出现的，因此其结构长这个样子：

[TODO:代码结构框图]()

*补充说明：本工程中涉及 Cartographer 的部分，著作权和许可归原作者所有。*


## 3. 开源数据集测试（Tests on Open-Source Dataset） //TODO
### 数据集1
### 数据集2
### 数据集3


## 4. 编译与运行（Compile and Run） //TODO
配置第三方依赖：   
> `XXX`   
> yaml-cpp（ubuntu默认自带，若无请点[这里](https://github.com/jbeder/yaml-cpp)手动编译安装）   

编译：   
> mkdir -p XXX   
> git clone XXX   
> cd XX  
> catkin_make -DXX  
> source devel/setup.bash

运行：   
> roslaunch XX XX  


## 5. 未来工作（TODOs） //TODO
- 测试 XX LiDAR 数据集
- 耗时分析

## 6. 其它（Others） //TODO
- Infinity-SLAM 项目
- ... 







