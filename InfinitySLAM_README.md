# Infinity-SLAM
无限拓展的个人SLAM框架，目前已集成XXX；开箱即用，欢迎clone和测试；持续迭代中 ....

namesapce infinity::
    common/    # include sensor?
    utils/     # include transform?
    sensor/
    transform/
    io/
    optimization/
    filter/
    liosam/
    csmlio/
    legoloam/
    loop/
    dync_remov/
    mapping/
    ...

XXX


## Features
优雅的工程级C++实现，风格 Google Style，遵循 C++ 14 标准


## 工程风格（code style）
充分利用函数对象来设计接口，减少甚至避免各个模块间的耦合；