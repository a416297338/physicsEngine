# 这是一个简单的 physicsEngine 


功能如下:

![1](https://github.com/user-attachments/assets/805634c1-e5cc-4211-a279-f2f6a9bb2e5e)

鼠标左键创造⭕，键盘上点3-9的数字后，再点击右键创造3-9边形。
参考与借鉴了别人框架，重新实现了物理塑性碰撞中所有应该有的部分（ 弹性碰撞与软性约束没写，这里指的是弹簧一类的）。
算法部分写的还行,整体物理框架结构写完后感觉还有欠缺。
详细介绍文章链接：
https://zhuanlan.zhihu.com/p/25402896393
运行方案如下：
1. Get a C++20 compiler
2. Build with CMake :

```bash
cmake -S . -B .\build\
cmake --build .\build\
