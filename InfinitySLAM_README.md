# Infinity-SLAM
无限拓展的个人SLAM框架，目前已集成XXX；开箱即用，欢迎clone和测试；持续迭代中 ....

namesapce infinity::
> |  表头 | 表头 | 表头 |
> | :---- | ----: | :----: |
> | 单元格1 | 单元格3 | 单元格5 |
> | 单元格2 | 单元格4 | 单元格6 |
> 

next ##############3

| 模块         | 描述          |
| ------------ | ------------ |
| common       | ：公共数据类型、数据结构 |
| sensor       | ：与传感器相关的数据结构 |
| transform    | ：几何变换 |
| utils        | ：抽取出来的公共操作、多为模板函数 |
| io           | ：系统输入输出 |
| ieskf        | ：滤波算法实现 |
| optimization | ：优化算法实现 |
| csmlio       | ：CSM-LIO实现（独立仓库点[这里](https://github.com/KennyWGH/CSM-LIO)） |
| liosam       | ：LIO-SAM实现 [link]() |
| legoloam     | ：LeGO-LOAM实现 [link]() |
| loop         | ：回环检测 |
| mapping      | ：建图、动态过滤 |
| frees_sapce  | ：可行驶区域生成/规划 |
| slam         | ：Infinity-SLAM 顶层模块 |

next #########

> - xxx
> - ...

XXX


## Features
优雅的工程级C++实现，风格 Google Style，遵循 C++ 14 标准


## 工程风格（code style）
充分利用函数对象来设计接口，减少甚至避免各个模块间的耦合；