#1.cmake verson，指定cmake版本
cmake_minimum_required(VERSION 2.8)

#2.project name，指定项目的名称，一般和项目的文件夹名称对应
PROJECT(RaspberryMoveDetect)

#查找依赖的opencv
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "OpenCV library status:")
  message(STATUS "    version: ${OpenCV_VERSION}")
  message(STATUS "    libraries: ${OpenCV_LIBS}")
  message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
endif()

set(BGSLIBRARY_INCLUDE_FILE "/home/pi/Downloads/bgslibrary/package_bgs")

#3.head file path，头文件目录
include_directories(
include
${OpenCV_INCLUDE_DIRS}
${BGSLIBRARY_INCLUDE_FILE}
)

#4.source directory，源文件目录
AUX_SOURCE_DIRECTORY(src DIR_SRCS)

#5.set environment variable，设置环境变量，编译用到的源文件全部都要放到这里，否则编译能够通过，但是执行的时候会出现各种问题，比如"symbol lookup error xxxxx , undefined symbol"
set(OTHER_FILE ${DIR_SRCS})

#6.add executable file，添加要编译的可执行文件
add_executable(${PROJECT_NAME} ${OTHER_FILE})

#7.add link library，添加可执行文件所需要的库，比如我们用到了libm.so（命名规则：lib+name+.so），就添加该库的名称
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} bgslibrary_core)