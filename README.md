# BLE Pickleball

#### 介绍
​         BLE智能匹克球拍项目立足于当前兴起的“匹克球”运动，基于嵌入式和物联网技术对匹克球智能球拍进行设计，该智能球拍能够实现准确记录击球次数、数据存储和上传、手机移动端实时监测等功能。产品设计要求具有精准、便携、交互性良好等特点。

#### 软件架构
软件架构说明

#### 设计记录

TI官方cc2640推出的SDK在IAR环境下不支持中文路径，环境搭建注意事项如下：

1. IAR按默认路径安装（人家SDK里固化了），否则编译不过；

2. SDK路径不应过长，建议直接放根目录；

3. SDK路径不能有中文及特殊字符，可用“_”，不能用“-”；

4. 编译时需先编译Stack，然后编译App项目。

   ![image-20211116165815507](C:\ti\simplelink_cc2640r2_sdk_1_40_00_45\examples\rtos\CC2640R2_BLERACKET\ble5stack\BLE_RACKET_RSM_V1.1\README.assets\image-20211116165815507.png)

### **原子操作**

原子(atomic)本意是"不能被进一步分割的最小粒子"，而原子操作(atomic operation)意为"```不可中断的一个或一系列操作```"。其实用大白话说出来就是让多个线程对同一块内存的操作是串行的，不会因为并发操作把内存写的不符合预期。

### 墨水屏库文件替换

HNK-E0154A07-A1型号墨水屏对应的库文件为lcd_epd_libA07_r2.a![image-20220117173955792](C:\ti\simplelink_cc2640r2_sdk_1_40_00_45\examples\rtos\CC2640R2_BLERACKET\ble5stack\BLE_RACKET_RSM_V1.1\README.assets\image-20220117173955792.png)

修改全局环境变量，避免路径冲突（否则编译不过）![image-20220122103200518](C:\ti\simplelink_cc2640r2_sdk_1_40_00_45\examples\rtos\CC2640R2_BLERACKET\ble5stack\BLE_RACKET_RSM_V1.1\README.assets\image-20220122103200518.png)

### cJTAG烧录接口

需连接XDS110仿真器TMS、TCK、GND、RST（最好连接）


#### 安装教程

1.  xxxx
2.  xxxx
3.  xxxx

#### 使用说明

1.  xxxx
2.  xxxx
3.  xxxx

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
