一些笔记

# 常量指针和指针常量

### 1. 常量指针
指向常量的指针，即无法通过指针修改所指内容
```c
int a,b;
const int* p = &a;
*p = 9; // error
p = &b  // ok
```

### 2. 指针常量
指针本身是常量，指向固定位置，但可修改该位置的内容
```c
int a,b;
int * const p = &a;
*p = 9; // ok
p = &b  // error
```

# 仿射变换矩阵
根据原图三个点和目标三个对应点可以求出仿射变换矩阵M

所有点的映射关系是[x',y',1]^T = M*[x,y,1]^T

使用opencv的getAffineTransform默认得到mat是CV_64F，如果后续要float的话需要转换一下，因为存储方式不同不能直接按float转换。   
在gdb时无法直接打出mat的data内容（实际上是不知道mat的数据类型），可以通过以下方式查看：

```c++
  auto res = getAffineMatrix(cv::Size(640, 426), cv::Size(640, 640));
  int type = res.type();
  int channels = CV_MAT_CN(type);  // 获取通道数
  int depth = CV_MAT_DEPTH(type);   // 获取深度类型

  switch (depth) {
    case CV_8U: std::cout << "Depth: CV_8U (8-bit unsigned)" << std::endl; break;
    case CV_8S: std::cout << "Depth: CV_8S (8-bit signed)" << std::endl; break;
    case CV_16U: std::cout << "Depth: CV_16U (16-bit unsigned)" << std::endl; break;
    case CV_16S: std::cout << "Depth: CV_16S (16-bit signed)" << std::endl; break;
    case CV_32S: std::cout << "Depth: CV_32S (32-bit signed)" << std::endl; break;
    case CV_32F: std::cout << "Depth: CV_32F (32-bit float)" << std::endl; break;
    case CV_64F: std::cout << "Depth: CV_64F (64-bit float)" << std::endl; break;
    default: std::cout << "Unknown depth" << std::endl; break;
  }

  for (int i = 0; i < res.rows; ++i) {
    for (int j = 0; j < res.cols; ++j) {
        std::cout << res.at<double>(i, j) << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "Matrix type: " << type << std::endl;
  std::cout << "Channels: " << channels << std::endl;
```
注：
1. mat.type()得到的是通道数和深度的完整类型，如CV_8UC3   
2. CV_MAT_CN(type)得到的是通道数
3. CV_MAT_DEPTH(type)得到数据的深度（类型），如CV_32F

# cmake

读取目录下所有相关文件的方法是使用file(GLOB ...)或file(GLOB_RECURSE ...)，区别是GLOB_RECURSE会递归的获取所有目录下的相关文件，在这个项目中没区别
```bash
file(GLOB SOURCES "src/*.cpp" "src/*.cu")
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.cu")
```