# 点云交互式提取边缘线
本仓库包含三个 Python 脚本(**detect-lines.py**, **Curvature_calculation.py**, 和 **intersection.py**)，用于使用 Open3D 库处理和分析点云数据。以下是每个脚本的简要概述以及如何使用它们的说明。

## detect-lines.py

**描述：**

detect-lines.py 脚本提供了从点云中交互式选择点、提取指定搜索半径内的局部点、对局部点云进行平面拟合以及迭代细化过程的功能。它还利用曲率计算和交集操作进行进一步的分析。

**依赖项：**
```
Open3D

NumPy

Matplotlib

scikit-learn
```
**使用方式：**

确保已安装所需的依赖项。
使用您的点云文件的路径更新 point_cloud_path 变量。
使用 Python 解释器运行脚本。

## Curvature_calculation.py

**描述：**

Curvature_calculation.py 脚本提供了计算给定点云表面曲率的函数。它利用奇异值分解 (SVD) 计算特征值，然后使用这些特征值来确定每个点的曲率。

**依赖项：**
```
Open3D

NumPy
```
**使用方式：**

将 Curvature_calculation.py 脚本导入到您的项目中。
使用 caculate_surface_curvature 函数，提供点云和可选的半径参数。

## intersection.py

**描述：**

intersection.py 脚本定义了用于找到两个点云的交集以及在点云中识别最远点的函数。

**依赖项：**
```
Open3D

NumPy

SciPy
```
**使用方式：**

将 **intersection.py** 脚本导入到您的项目中。
利用 intersection 函数找到两个点云的交集。
使用 find_farthest_points 函数在点云中识别最远的点。
注意：在使用脚本之前，请确保已安装所需的库。您可以使用以下命令安装它们：

### bash
#### Copy code

```sh
pip install open3d numpy matplotlib scikit-learn scipy
```
随时根据需要自定义和集成这些脚本到您的工作流中。如果遇到任何问题或有疑问，请参考各库的文档或寻求帮助。