# 本文件定义了一些CMake语法下的函数和宏，我们可以像调用CMake的内置函数和宏一样来调用
# 这些自定义的函数和宏。

# 在Cmake3.5版本之前，cmake_parse_arguments()命令是由CMakeParseArguments模块
# 提供的，我们需要include它。
include(CMakeParseArguments)

# 许多内置函数或宏支持任意数量和任意顺序的关键字、每个关键字后可以支持任意数量的参数，
# 比如常见的target_link_libraries()命令；这种灵活性在自定义的函数或宏中也能够实现，
# 方法正是使用cmake_parse_arguments()命令 —— 这是一个CMake内置命令，我们可以在如
# 下代码中看到使用方法。
# 官方解释：cmake_parse_arguments is intended to be used in macros or 
# functions for parsing the arguments given to that macro or function. 
# It processes the arguments and defines a set of variables which hold 
# the values of the respective options.
# 我们可以在官方网站或者网络博客中看到对cmake_parse_arguments()命令的具体解释。
macro(parse_arguments_as_ARG_X ARGS)
  set(prefix ARG)
  set(OPTIONS)
  set(ONE_VALUE_ARG)
  set(MULTI_VALUE_ARGS SRCS)
  cmake_parse_arguments(
    "${prefix}"
    "${OPTIONS}" 
    "${ONE_VALUE_ARG}" 
    "${MULTI_VALUE_ARGS}" 
    ${ARGS})
  # 本宏效果：
  # 不支持解析【无参数的关键字】；
  # 不支持解析【一个参数的关键字】；
  # 支持解析【多个参数的关键字】SRCS；
  # 在本宏的外部引用解析出的变量时，需要加前缀，比如 ${ARG_SRCS} ，而非 ${SRCS} 。
endmacro(parse_arguments_as_ARG_X)

function(add_flag_to_variable VAR_NAME FLAG)
  if (${VAR_NAME}) 
    set(${VAR_NAME} "${${VAR_NAME}} ${FLAG}" PARENT_SCOPE)
  else()
    set(${VAR_NAME} "${FLAG}" PARENT_SCOPE)
  endif()
endfunction(add_flag_to_variable)

# 本文件允许外部调用的接口仅包括下方的函数和宏。
macro(initialize_this_project)

  set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

  # 编译选项设置：
  # 应该启用所有可用、合理的告警选项，有些告警选项只在启用了优化的情况下才有效，或者优
  # 化级别越高，效果越好；
  # 使用的选项应该支持尽可能多的编译器，每个编译器对标准的实现略有不同，支持多个编译器
  # 将有助于确保实现最可移植、最可靠的代码。
  set(CXX_COMPILE_FLAGS "-pthread -fPIC ${CXX_COMPILE_FLAGS}")
  if (CMAKE_CXX_COMPILER_ID MATCHES "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.1)
    message(STATUS "This project is going to use GNU compiler.")
    add_flag_to_variable(CXX_COMPILE_FLAGS "-std=c++11")
  endif()
  add_flag_to_variable(CXX_COMPILE_FLAGS "-Wall") #和"-W"一样，开启所有警告。
  add_flag_to_variable(CXX_COMPILE_FLAGS "-Wpedantic") #发布严格的ISO C和ISO C ++所要求的所有警告。
  # 由于我们可能使用很高的编译优化级别，所以对代码要求很高，不允许出现以下警告（会被强制当作errors处理）。
  add_flag_to_variable(CXX_COMPILE_FLAGS "-Werror=format-security") #不允许类型错误，如printf函数。
  add_flag_to_variable(CXX_COMPILE_FLAGS "-Werror=missing-braces")  #聚合初始化两边缺少大括号。
  add_flag_to_variable(CXX_COMPILE_FLAGS "-Werror=reorder")         #初始化顺序必须和声明顺序一致。
  add_flag_to_variable(CXX_COMPILE_FLAGS "-Werror=return-type")     #返回类型错误警告。
  add_flag_to_variable(CXX_COMPILE_FLAGS "-Werror=switch")          #switch语句参数漏参或越界等。
  add_flag_to_variable(CXX_COMPILE_FLAGS "-Werror=uninitialized")   #在初始化之前使用变量。

  # 默认以Release级别编译代码
  if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
  endif()

  # 根据编译优化级别添加相应的编译选项
  if (CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Building in Release mode.")
    add_flag_to_variable(CXX_COMPILE_FLAGS "-O3 -DNDEBUG") 
    #选项 O3 是较高的编译优化级别，在此之前还有O1、O2等级别；O3级别在常规优化选项之外
    #还会采取很多向量化算法提高代码的并行程度，也会利用现代CPU的特性做加速，总之能相当
    #大程度地加快代码的执行速度。
    #相比于另一个高级别的优化选项 Os，O3的目标是宁愿增加目标代码的大小，也要拼命的提高
    #运行速度；而Os是在O2的基础之上，尽量的降低目标代码的大小，这对于存储容量很小的设备
    #来说非常重要。
    #选项 NDEBUG 是被定义为根据C标准强制关闭断言的宏。
    #NDEBUG宏 是Standard C中定义的宏，专门用来控制assert()的行为。如果定义了这个宏，
    #则assert不会起作用；在vc/vs编译器中，对生成的release版本项目，默认会定义这个宏，
    #而gcc不会这样做，所以得用 -DNDEBUG 参数来显示地定义。
    #我们可以这样理解该选项的功能：开发者在debug版本中用assert来方便调试，但在Release
    #版本中，我们不需要调试了，所以手动关闭掉assert。
  elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(WARNING "Building in Debug mode, expect very slow performance.")
    add_flag_to_variable(CXX_COMPILE_FLAGS "-g") #支持gdb调试？
  endif()
  message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

  # Add a hook that reruns CMake when source files are added or removed.
  # 添加一个挂钩，在添加或删除源文件时重新运行 CMake。  20221228：暂时不考虑这个功能。

endmacro()

function(add_and_install_binary NAME)
  message(STATUS "")

  #解析源文件参数，添加构建目标
  parse_arguments_as_ARG_X("${ARGN}")
  add_executable(${NAME}, ${ARG_SRCS})

  #设置构建参数
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${CXX_COMPILE_FLAGS}")
  set_target_properties(${NAME} 
    PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
  target_include_directories(${NAME} PUBLIC ${PROJECT_NAME})
  target_link_libraries(${NAME} PUBLIC ${PROJECT_NAME})

  #设置安装参数
  install(TARGETS "${NAME}" RUNTIME DESTINATION bin)
endfunction()

