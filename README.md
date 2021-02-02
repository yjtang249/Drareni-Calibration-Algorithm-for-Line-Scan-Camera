# Drareni-Calibration-Algorithm-for-Line-Scan-Camera

This project is aimed to deal with calibration of line scan camera. The final goal is to find the mapping relationship between image plane taken by line scan camera and world plane in which the objects to be gripped are set, so that once the target object is recogonized in image plane gotten from camera, the gripper of robot can accurately locate the target in real world and grasp it.

There's something different between the original Drareni Algorithm and my implementation, the chief reason is that there's no need for us to dcompose the mapping matrix *H* once it is computed, because we do not care about the exact intrisinc parameters and external parameters. In other words, only the mapping matrix itself is what we want, the further step(Decomposition of *H*) is unecessary. This kind of "calibration" may not accord with the definition of so-called camera calibration, but anyway, this is just the application scenarios of this project.

Beside, I also implement the basic mapping method for our "calibration" task, that is, to find homography between image plane and world plane(2D to 2D) using the plain DLT and RANSAC algorithm.

The detail of each function can be found in source code files together with their implementations, and its brief introductions are in the coorespoding heard files along with their declarations. Also, the original paper and an introduction PPT are provided.


这个项目主要是用于线阵相机的标定。最终的目标是找到图像平面坐标系与世界平面坐标的映射关系，从而使得机器人在工作时可以通过在线阵相机拍得得图像中识别出待抓取物体，进而指挥机械臂末端在世界平面中定位到该物体完成抓取。

我在实现这个算法的时候和原始论文中的版本有所不同。主要的原因是因为，我们这里所做的的“标定”不是真正地要完全得到内参、外参的各个参数，而是只要求出图像平面和世界坐标平面的映射矩阵*H*就可以了。所以，当我们求出*H*之后，就不需要进行进一步的分解了。

此外，我还在该代码中实现了一般的基于DLT和RANSAC算法的2D平面之间的单应估计算法。

各个函数的详细说明在源代码文件中，简介在头文件中。同时还附上了Drareni的原论文和介绍PPT。