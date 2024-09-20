# warp

这是一个生成letterbox的cuda实现，主要使用的函数warp_affine_padding在affine.cu中定义。

这里用的是bilinar的插值方法，核函数中每个线程负责计算一个二维坐标上的三通道像素值

## 1. 目录结构

```bash
.
├── CMakeLists.txt
├── docs              # 一些笔记
│   └── note.md
├── include
│   ├── affine.h      # 函数声明
│   └── utils.h       # cuda错误检查方法
├── output.png        # 结果示意图
├── pics              # 输入图片
│   ├── 1.jpg
│   └── water.jpg
├── readme.md
└── src
    ├── affine.cu     # 函数定义
    └── main.cpp      # 调用方式
         
```

## 2. 使用方式
编译后执行即可
```bash
mkdir build
cd build
cmake ..
make 
cd ..
./warp_affine_demo
```

## 3. 其他
bilinar插值原理可参考[link](https://zhuanlan.zhihu.com/p/110754637)，在affine中也有相关注释