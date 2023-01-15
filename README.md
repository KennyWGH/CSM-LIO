# CSM-LIO

## 1. 简介（Introduction）

具有以下特性：   
以 Correlative Scan Match (CSM) 算法为核心搭建的松耦合LIO算法；
计算速度快，32线旋转式激光雷达单帧处理时间Xms；  
鲁棒性好，得益于CSM； 
兼容不同形态的激光雷达，就本算法而言，机械旋转式和Livox，XXX，都已经经过了测试；   

## 2. 工程架构（Architecture）
本工程中的核心代码直接裁剪自Google Cartographer()，包括多sensor间数据对齐、ImuTracker、PoseExtrapolator、Submap3D/ActivaeSubmap3D/etc、、，等部分。著作权和许可归XX所有。  
本工程的代码架构接近，如下图所示，无改动，因此不再赘述。

**ros_app 职责范围**
- 从lua文件加载所有必要参数；
- 管理所有话题订阅；
- 管理消息格式转换和数据投喂；
- 从算法层获取必要的信息、转换格式、并发送到外界，包括rviz；
> 不再区分traj_id的概念，仅区分sensor_id即可；  
> 不再考虑续扫，保留原有接口但函数体为空；

**cartographer_ros 的参数文件解析系统（.lua）**
- 整个文件解析系统都实现在 *`lua_parameter_dictionary.h/cc`、`configuration_file_resolver.h/cc`* 两个文件中，这两个文件属于 `cartographer工程`，它们定义的类/函数/对象/等都位于 `cartograph-er::common` 命名空间；
- 第一个类名字叫做 **ConfigurationFileResolver**，专门负责从特定路径下搜索各个lua文件，确认lua文件存在；
- 搜索路径有两个来源，一个是main函数中给定的路径（来自launch文件，通过构造函数传参给类对象），另一个是编译过程中自动记录下的程序安装路径（路径变量位于cofig.h），这两者正常情况下是相同的；
- 类成员函数 ConfigurationFileResolver::**GetFullPathOrDie()** 负责确认给定名字的lua文件存在，并返回lua文件的绝对路径，如`/home/.../install_isolated/.../wgh_backpack_3d.lua`；
- 类成员函数 ConfigurationFileResolver::**GetFileContentOrDie()** 负责通过`std::ifstream`把lua文件中的文本内容全部加载到一个`std::string对象`中，并返回，后续会有lua脚本语言专门从字符串中解析出所有参数！至此，ConfigurationFileResolver类的使命就完成了。
- 接下来，轮到另一个类登场了 —— **class LuaParameterDictionary**，专门负责（通过lua脚本语言？）从`std::string对象`内解析出所有参数；由于在构造时引入了 ConfigurationFileResolver对象，该类还支持解析嵌套的lua文件 —— 也即被当前lua文件通过`include "wgh_map_builder.lua"`的方式包含的其它lua文件；具体的实现较为复杂，应该是跟lua脚本语言语法有关，这里不再深入研究，知道如何使用即可。
- 解析出来的所有必要参数，最终是以 **struct Node-Options**、**struct Trajectory-Options** 结构体的方式给到下游代码去使用的，这又涉及到了另外两个源文件 —— *`node_options.h/cc`、`trajectory_options.h/cc`*，这两个文件都属于 `cartographer_ros工程`，但是，结构体内部嵌套（以及多重嵌套）的结构体 —— 如 *class ::cartograph-er::mapping::proto::**MapBuilder-Options***、*class ::cartograph-er::mapping::proto::**TrajectoryBuilder-Options*** 等，则对应性地属于`cartographer工程`，它们都是由 *`.proto`* 脚本文件导出的class。
> 所有 *`.proto`* 脚本文件导出的部分，都用C++普通结构体替代。

**CSM-LIO架构设计**
- 整个核心库（core文件夹）在编译时仍作为library存在，与google原版工程类似；
- 改变整个核心库的顶层命名空间，删掉所有和CSM-LIO无关的代码，追求瘦身；
- 新增CSMLidarInertialOdometry作为顶层类，相当于取代MapBuilder或者CollatedTrajectoryBuilder的作用；
- 不改变已有类的属性和派生层次，但是顶层类直接引用需要的派生类作为成员，而非基类 —— 我们没有多态的需求；这样做的另一个好处是，可以随意在衍生类中新增接口，开发上更加灵活；

**TODOs**
- 开发 CSMLidarInertialOdometry 顶层类，该类直达必要组件，目标是取代 Collated & Global & LocalTrajectoryBuilder3D。详细来说，原生版本中，外部直接桥接的接口类是 Collated，然后依次调用 Global 和 Local，我们把三者的接口和功能合并到一处；【已完成】
- 在现有core框架下，把core/app层写好，编译运行起来，实现cartographer的表现；（大工程）【已完成】
- core留接口向外输出配准位姿，以便外层用“配准位姿+IMU”做点云去畸变，方便mapping模块直接使用去畸变后的点云；
- 精简core层的代码，删除一切不必要的，极致精简瘦身；
- 不要在核心层依赖GFLAG/GLOG的非日志功能，方便将来可能为了极致轻量化剥离GFLAG/GLOG。
- 重写参数系统，重写或取代 *`.pb.h/cc`* 文件，改用简单结构体 —— 或者直接一个大参数结构体！（用yaml或lua加载）就这么干。我们的目的是让参数的增/删/加载/访问更加方便，最好摆脱lua依赖。
- 重写LIO中的性能监控，每隔一段时间日志打印一次。
- 用深拷贝+独立线程来刷新global map。（待定）
- 用服务响应的方式、用非线性优化的方法、做“lidar2imu”角度外参矫正。（待定）
- online dynamic removal and mapping, visualize incremental grid map and online dymamic objects removal.


## 3. 算法特性（Features）
内容。

### 3.1 XXX
内容。

### 3.2 XXX 
内容。

### 3.N 在线建图（Online Mapping）
在线生产点云地图。具有以下两个功能。  

- 在线过滤动态物体（Online Dynamic Removal）：正文内容介绍。

- 地面分割？（Efficient Online Segmentation / or RANSAC / or others like lio-mapping by livox）：正文内容介绍。

## 4. 编译与运行（Compile and Run）
依赖，依赖`XXX`。  

编译。  
> cd XX  
> cmake .. -DXX  
> catkin_make -DXX  

运行。  
> roslaunch XX XX  


## 5. 开源数据集示例（Open-Source Dataset Examples）


## 6. 其它（Others）