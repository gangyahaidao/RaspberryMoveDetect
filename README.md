# RaspberryMoveDetect

## 本工程主要有两个功能：
* 不同功能切换是在配置文件中配置
* 功能1：基于普通USB摄像头在树莓派上实现运动物体检测，支持框选检测特定区域，框选区域之后会将该区域进行栅格的划分，划分精度在config目录下的VideoCapture_config.xml配置文件中配置，被触发的栅格会被标记
* 功能2：检测某块框选的区域图像是否改变，如检测是否有人经过或者框选的区域是否被占用等
* 两个功能最终都会触发树莓派的Pin4引脚连接的继电器，引脚控制使用了WiringPi库，具体安装教程可参考：http://bbs.elecfans.com/jishu_704532_1_2.html

## 使用方法
* 步骤1：安装bgslibrary库，地址为：https://github.com/andrewssobral/bgslibrary
    * 这个库是专门做背景剔除运动检测的库，里面包含各种不同效率和效果的算法，相比于opencv原生的运动检测算法，bgslibrary没有鬼影以及残留等问题，针对树莓派选择了使用DPMean算法，本库只是在功能1中使用，功能2依赖于opencv不需要安装，树莓派上具体安装如下：
        * mkdir build
        * cmake ..
        * make
        * sudo make install
* 步骤2：创建build目录，运行
    * cmake ..
    * make

## 运行截图
* 功能1截图
![](https://github.com/gangyahaidao/RaspberryMoveDetect/blob/master/images/2018-12-13-092613_1024x600_scrot.png)

* 功能2截图
![](https://github.com/gangyahaidao/RaspberryMoveDetect/blob/master/images/2018-12-12-154513_1024x600_scrot.png)