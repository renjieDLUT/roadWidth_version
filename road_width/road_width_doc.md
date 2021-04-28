@[TOC](通车道宽度计算)
# 1.要求
利用纵目发出的**gridmap**计算通车道宽度；通车道宽度用于泊车路径规划。
## gridmap
![在这里插入图片描述](https://img-blog.csdnimg.cn/20201014154727860.png#pic_center)
# 2.场景
根据泊车状态（确任泊车位，泊车中），指定不同的通车道计算算法。
## 泊车状态1（确认泊车位）
在该状态，还没确定泊哪个车位，计算出来的通车道宽度，为所有检测出的车位提供统一的通车道宽度。（或者每个车位都计算一个通车道宽度）
## 泊车状态2（泊车中）
在该状态下，用户已经选好了泊车位，SF已建立了**泊车坐标系**，在该坐标系下计算通车道
# 3.算法策略
## 原理
根据图像中能截取最大的free矩形区域，并且车辆的后轴中心在该区域内，矩形的长度为标定量（==前后5m==），宽度即为所求的通车道宽度。（若需要为每个车位求取一个通车道，即车辆不必为该区域内，根据车位的P0与P1点，来确定矩形的长度与搜索边界）
## 步骤
- **a**.将gridmap转成Mat图像格式；
- **b**.计算车辆位置、P0、P1对应的像素点位置
- **c**.确定图像的旋转基准angle（车辆的航向方向，或车位P0P1的方向；状态为泊车中时，为泊车坐标系的X方向）；
- **d**.计算仿射矩阵，进行仿射变换，计算变化后的图像，以及变换后的车辆位置、P0、P1对应的像素点位置
- **e**.选择参考点
  - 若选择参考车辆的航向方向，参考点为车辆所对应的像素点为a(ua,va)；
  - 若选择参考车位P0P1的方向，参考点为P0与P1的中点靠外侧2m处，所对应的像素点为b(ub,vb)；
- **f**.计算在该变换下的通车道宽度，记==5m==所对应的像素值m；
   - 矩阵块的左上角为(ua-m,x)，矩阵的右下角为(ua+m,y);
   - 遍历x,y的值（二分法），截取矩阵块，判断矩阵块是否为free;
   - 取free矩阵块中最大的y-x;即为通车道宽度
- **g**.若状态为确认泊车位以旋转基准angle为基准，遍历【angle-10°，angle+10°】，重复步骤(d-f);若状态为泊车中，不重复步骤。
- **h**.最大值即为通车道宽度。
# 4. 接口
|通车道宽度CRoadWidth|
|--------|
|<kbd>-</kbd> **int** `_state`，车辆状态|
|<kbd>-</kbd>   **Vector3d**   `_pose`，车辆的位姿|
|<kbd>-</kbd>   **CvPoint** `_pixVeh`，车辆位姿的像素点| 
|<kbd>-</kbd>   **vector< Slot >**  ` _slots`，车位信息|
|<kbd>-</kbd>   **vector< vector< CvPoint > >**  `_pixSlots`，车位的像素点| 
|<kbd>-</kbd>   **gridmap** `_gridmap`，订阅的gridmap|
|<kbd>-</kbd>   **Mat** `_map`，gridmap转成的图像|
|<kbd>-</kbd>   **double**  `_baseAngle`，旋转基准角度|
|<kbd>-</kbd>   **double**  `_rotateAngle`，图片旋转角度|
|<kbd>-</kbd> **Vector3d** `_parkCoord`，泊车坐标系|
|<kbd>-</kbd> **vector< Vector2d >** `_referencePoint`，参考点的位置 |
|<kbd>-</kbd> **vector< CvPoint >** `_referencePixel`，参考点的像素位置 |
|<kbd>-</kbd> **double** `_scale` ，分辨率，cm/pixel，默认值为10|
|<kbd>-</kbd> **double** `_mFront` ，计算通车道时，前向距离，默认值为5m,500|
|<kbd>-</kbd> **double** `_mRear` ，计算通车道时，后向距离，默认值为5m,500|
|<kbd>-</kbd> **double** `_thresholdAngle`，遍历旋转角度的左右阈值，默认值为10° |
|<kbd>-</kbd> **double** `_maxRoadWidth`，最大通车道宽度，默认值为7m|
|<kbd>-</kbd> **double** `_roadWidth`，通车道宽度，默认值为2m|
|------------------------------------------------------------------------------------------------------------|
|<kbd>+</kbd> `CRoadWidth`( **gridmap**  grid, **Vector3d** pose, **vector< Slot >**    `slots`, **float** parkCoord，**double** scale)|
|<kbd>+</kbd> **void** `setGridmap`(**gridmap**  grid)|
|<kbd>+</kbd> **void** `setPose`( **Vector3d** pose)|
|<kbd>+</kbd> **void** `setSlots`( **vector< Slot >**    slots)|
|<kbd>+</kbd> **void** `setParkCoord`( **Vector3d** parkCoord)|
|<kbd>+</kbd> **void** `toMat( )`，将gridmap转换为mat|
|<kbd>+</kbd> **void** `point2D2pixel()`，计算坐标系下的点所对应的像素点 
|<kbd>+</kbd> **void** `findRefence( )`，找参考点的位置| 
|<kbd>+</kbd> **double** `roadWidthNoTraverse()`，旋转一次后计算出来的通车道宽度|
|<kbd>+</kbd> **double** `getRoadWidth()`，获取通车道宽度|
