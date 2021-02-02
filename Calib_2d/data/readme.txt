该文档对./data文件夹下存放一些数据文件进行了说明：

1. data_x.txt:
    存放对应的../img/img_camx.png中检测到的各个corner的坐标。对于同一个marker的4个corner，存放顺序为从左上角开始顺时针;

2. intrinsic.yml:
    存放的是相机的内参矩阵和畸变系数向量（用calibrateCamera()算出来的）；

3. extrinsic.yml:
    存放的是各个视图的外参，旋转矩阵同时也把其旋转向量的形式存储了，还有位移向量。注意，如：RotatMat1表示的是第1个视图
的旋转矩阵，TransVec3表示的是第3个视图位移向量（用calibrateCamera()算出来的）；

4. extrinsic_pnp.yml:
    存储的也是各个视图的外参，格式与extrinsic.yml完全一致。不同之处在于，这里所存储的外参数据是以extrinsic.yml中
的为初值，使用solvePnP()计算出来的；

5. tst1_image_points_norm.txt:
    （产生于 Drareni_computeHomography()函数的 test1 部分）
    当前所选择（所测试）的图片中，图像平面坐标系 下各个角点的(Normalized)坐标。用来作为参照；

6. tst1_worldTransToImage_points_norm.txt:
    （产生于 Drareni_computeHomography()函数的 test1 部分）
    当前所选择（所测试）的图片中，世界平面坐标系 下各个角点的(Normalized)坐标 用Drareni方法计算出的矩阵映射回 图像平面坐标系 下的
    (Normalized)的坐标。用来与上一条的参照坐标进行对比，看映射得准不准；

7. tst2_world_points_norm.txt:
    （产生于 Drareni_computeHomography()函数的 test2 部分）
    当前所选择（所测试）的图片中，世界平面坐标系 下各个角点的(Normalized)坐标。用来作为参照；

8. tst2_imageTransToWorld_points_norm.txt:
    （产生于 Drareni_computeHomography()函数的 test2 部分）
    当前所选择（所测试）的图片中，图像平面坐标系 下各个角点的(Normalized)坐标 用Drareni方法计算出的矩阵映射到 世界平面坐标系 下的
    (Normalized)的坐标。用来与上一条的参照坐标进行对比，看映射得准不准；

