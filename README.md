# Drareni-Calibration-Algorithm-for-Line-Scan-Camera

This project is aimed to deal with calibration of line scan camera. The final goal is to find the mapping relationship between image plane taken by line scan camera and world plane in which the objects to be gripped are set, so that once the target object is recogonized in image plane gotten from camera, the gripper of robot can accurately locate target in real world and grasp it.

There's something different between the original Drareni Algorithm and my implementation, the chief reason is that there's no need for us to dcompose the mapping matrix *H* once it is computed, because we do not care about the exact intrisinc parameters and external parameters. In other words, only the mapping matrix itself is what we want, the further step(Decomposition of *H*) is unecessary. This kind of "calibration" may not accord with the definition of so-called camera calibration, but anyway, this is just the application scenarios of this project.

Beside, I also implement the basic mapping method for our "calibration" task, that is, to find homography between image plane and world plane(2D to 2D) using the plain DLT and RANSAC algorithm.

The detail of each function can be found in source code files together with their implementations, and its brief introductions are in the coorespoding heard files along with their declarations. Also, the original paper is provided.