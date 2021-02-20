该文档对./data文件夹下存放一些数据文件进行了说明：

1. data_x.txt:
    存放对应的../img/img_camx.png中检测到的各个corner的坐标。对于同一个marker的4个corner，存放顺序为从左上角开始顺时针;

2. /calibrationData/intrinsic.yml:
    存放的是相机的内参矩阵和畸变系数向量（用calibrateCamera()算出来的）；

3. /calibrationData/extrinsic.yml:
    存放的是各个视图的外参，旋转矩阵同时也把其旋转向量的形式存储了，还有位移向量。注意，如：RotatMat1表示的是第1个视图
的旋转矩阵，TransVec3表示的是第3个视图位移向量（用calibrateCamera()算出来的）；

4. /calibrationData/extrinsic_pnp.yml:
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

9. tsk4_testPoints_Robot(x).txt:
    用于task4，它记录了走的点在机器人坐标系下的坐标。每一行代表走的一个验证点，"；"前的两个数字分别代表这个角点在标定板上的行号和列号
    （均从0开始，第一个角点为(0,0)，以此类推）。"；"后的数字代表该角点在机器人坐标系下的坐标。

10. /calibrationData/imageToWorld.yml:
    （产生于test3的3.2.2部分）
    该文件中存储了从 图像平面坐标系 到 世界平面坐标系 转换时要用到的三个矩阵：
        （1）T_img:图像平面坐标系的标准化(Normalization)矩阵；
        （2）T_wrld:世界平面坐标系的标准化(Normalization)矩阵；
        （3）H:从 升维世界坐标系(normalized坐标) 到 图像坐标系(normalizaed坐标) 的转化矩阵（3X6的）。

11. /calibrationData/worldToRobot.yml:
    (产生于task4的4.1.3部分)
    该文件中存储了 H_v(v表示verify)：从 世界坐标系 到 机器人坐标系 的坐标转换矩阵。

12. tsk5_objectToBeGrasped(x).txt:
    (用于于task5的5.2部分)
    该文件中存储了 待抓取物体 在图像平面中的坐标（在图片中自己用ps量）和它们在 机器人坐标系（平面）的坐标（走点量出来的）。
    格式为：