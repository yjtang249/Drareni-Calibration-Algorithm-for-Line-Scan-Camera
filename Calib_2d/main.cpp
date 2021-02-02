#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp> //用opencv自带的aruco模块
#include <opencv2/aruco/dictionary.hpp>
#include <string>
#include <vector>
#include <regex>
#include "calibration.h"


using namespace std;
using namespace cv;


int main()
{
    string imgFolder = "./img/";  //图片存放文件夹的路径
    int view_num = 4;  //关于同一标定板拍了多少张不同角度的照片(view)
    Size corner_point_size = Size(9,6);  //标定板上每行和每列各包含多少个角点
    Size actual_grid_size = Size(28, 28);  //在3D真实世界中，每个棋盘格子的实际(宽x高)，单位：mm

    //********************task 1. 探测角点 + 相机标定（求内参）+ 位姿估计********************
    cout<<"*****************Task 1: 角点检测+相机标定+位姿估计*****************"<<endl;
    //1.手动添加view
    vector<string> imgList;  //所有待求位姿的view（手动添加）
//    imgList.push_back(imgFolder + "chessboard_org2.png");
//    for(int i=0; i< view_num ; i++)
//    {
//        string imgname = imgFolder+"chessboard"+to_string(i+1)+".png";
//        imgList.push_back(imgname);
//    }
    string testImgFile = "chessboard_test1";  //test3.3要用的测试图像的名字（**********************）

    imgList.push_back(imgFolder+testImgFile+".png");
    imgList.push_back(imgFolder+"chessboard9.png");
    //手动添加view完毕

    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));


    //2.获取(其实就是自己指定)各个角点在3d世界坐标系中的位置（以第一个角点为原点，整个标定板为xoy平面建系）
    vector<vector<Point3f>> objectPoints_seq;  //用于保存各个角点在3D世界坐标系中的坐标
    int viewNum = imgList.size();
    for(int i=0; i<viewNum; i++)  //每轮循环代表一个view
    {
        vector<Point3f> object_points;
        for(int j=0; j<corner_point_size.height; j++)
        {
            for(int k=0; k<corner_point_size.width; k++)
            {
                Point3f obj_point;
                obj_point.x = (float)k*actual_grid_size.height;
                obj_point.y = (float)j*actual_grid_size.width;
                obj_point.z = 0.0;
                object_points.push_back(obj_point);
            }
        }
        objectPoints_seq.push_back(object_points);
    }


    //3.执行相机标定（会先进行角点检测），求取并保存相机内参、畸变系数向量、外参到.yml文件
    vector<vector<Point2f>> cornerPoints_seq;  //用于接收检测到的每个view上的各个角点的坐标
    string intr_file = "./data/intrinsic.yml";  //相机内参的存储文件
    string extr_file = "./data/extrinsic.yml";  //相机外参的存储文件
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    cameraCalibration(imgList, corner_point_size, actual_grid_size, cornerPoints_seq, objectPoints_seq,
                      cameraMatrix, distCoeffs, rvecs, tvecs, intr_file, extr_file);  //***

    //4.用PnP方法求解相机外参（位姿），并保存到.yml文件
    string extr_file_pnp = "./data/extrinsic_pnp.yml";

    //4.1位姿估计，注：在etPose_pnp()会用cameraCalibration()得到的rvecs和tvecs的值作为初值，利用Levenberg-Marquardt优化算法进行更新
    getPose_pnp(objectPoints_seq, cornerPoints_seq, intr_file, rvecs, tvecs, extr_file_pnp);  //***
    //******************** End of task 1 ********************


    //********************task 2.计算两幅图像之间的单应矩阵，并用重投影误差评估测试结果********************
    cout<<endl;
    cout<<"*****************Task 2: 计算两幅图像之间的单应矩阵，并用重投影误差评估测试结果*****************"<<endl;

    Mat mask;  //用于储存哪几组点对应是inlier（值不为0），哪几组点对应是outlier（值为0）.
    string source_img_name = imgFolder+"chessboard_org2.png";  //源图片
    string target_img_name = imgFolder+"chessboard4.png";  //目标图片

    //2.1 计算单应矩阵，并返回源图像和目标图像中inliers的坐标
    vector<Point2f> src_points_inliers;  //用于存放源图像中属于inlier的场景点
    vector<Point2f> trgt_points_inliers;  //用于存放目标图像中属于inlier的场景点
    Mat homography = computeHomography_2d(source_img_name, target_img_name, corner_point_size, mask,
                                          src_points_inliers, trgt_points_inliers);  //***
    //至此，得到了src_points_inliers、trgt_points_inliers中的值

    //2.2 计算对称转移误差(Symmetric Transfer Error, STE)
    vector<Point2f> src_points_proj;  //用于存放源图像上的点投影（到目标图像上）之后的点的坐标
    vector<Point2f> trgt_points_proj;  //用于存放目标图像上的点投影（到源图像上）之后的点的坐标
    symmetricTransfer_error(src_points_inliers, trgt_points_inliers, homography,
                            src_points_proj, trgt_points_proj );  //***

    //2.3 （在转移后的图片上）绘制转移点
    //画圈圈（绘制转移过来的inlier点）
    cout<< "********开始绘制带转移点的 源图像 和 目标图像******"<<endl;
    cout << "绘制源图片（带目标图片上转移过来的点）:" << endl;
    drawCircleOnImage(source_img_name, trgt_points_proj, Scalar(0,0,255), "Source Image with transfered points");

    cout << "绘制目标图片（带源图片上转移过来的点）:" << endl;
    drawCircleOnImage(target_img_name, src_points_proj, Scalar(255,0,0), "Target Image with transfered points");
    //******************** End of task 2 ********************


    //********************task 3.使用Drareni算法计算单应矩阵H(3X6)，并评估实验结果********************
    cout<<endl;
    cout<<"*****************Task 3: 使用Drareni算法计算单应矩阵H(3X6)，并评估实验结果*****************"<<endl;
    //（20210129）目前还没有拿到线阵相机所拍的图像，用的是面阵相机所拍的图像测的

    //3.1 选定一张测试用的图像，同时把测试要用到的角点坐标转换成指定形式
    //指定用来测试的图像： view[0]
    vector<Point2f> imagePoints = cornerPoints_seq[0];  //view[0]中的所有点
    vector<Point3f> worldPoints_3d = objectPoints_seq[0];
    vector<Point2f> worldPoints;
    for(int i=0; i<imagePoints.size(); i++)
    {
        worldPoints.emplace_back(worldPoints_3d[i].x, worldPoints_3d[i].y );
    }

    //在测试图像中选定多少个角点参与运算
    int wrldPts_num = 54;
    vector<Point2f> imgPoints_atd;
    vector<Point2f> wrldPoints_atd;
    for(int i=0; i<wrldPts_num; i++)
    {
        imgPoints_atd.push_back(imagePoints[ i ]);
        wrldPoints_atd.push_back(worldPoints[ i ]);
    }

    //3.2 计算Drareni算法的转换矩阵H(3X6) (normalized World coord -> normalized Image coord)
    Mat T_img, T_wrld;
    Mat H = Drareni_computeHomography(wrldPoints_atd, imgPoints_atd, T_img, T_wrld);  //***
    cout<<"H( normalized World coord -> normalized Image coord): "<<endl<<H<<endl;
    cout<<"T_img( Image Plane coord -> normalized Image Plane coord ): "<<endl<<T_img<<endl;
    cout<<"T_wrld( World Plane coord -> normalized Iorld Plane coord ): "<<endl<<T_wrld<<endl;


    //3.3 在图像平面上选择一个点，看看用转换矩阵把它转换到 世界平面坐标系 中得到的坐标与其 在世界平面坐标系中的真是坐标 差多少
    // 3.3.1 直接输出计算后的坐标与真实世界坐标对比
//    int test_point_ptId = 21;  //被选中的测试点的id：我看看把这个图像平面上的点坐标用Drareni方法求出的 H矩阵 反映射到世界平面，得到的坐标是多少
//    Point2f p_test = imagePoints[test_point_ptId];
    Point2f p_test = Point2f(483.0,355.0);  //测试点（1号黑点） 在图像平面的真实坐标（单位：pixel）

    Point2f p_computed = Drareni_computWorldCoord(p_test.x, p_test.y, H, T_img, T_wrld);  //***
    cout<<"computed coordinate of the Test Point in World Plane (denormalized): "<<p_computed<<endl;

    Point2f p_test_actual = Point2f(34.0,21.5);  //测试点 在世界平面上的真实坐标（单位：mm）
//    cout<<"actual coordinate in World Plane (denormalized): "<<worldPo ints[test_point_ptId]<<endl;
    cout<<"actual coordinate of the Test Point in World Plane (denormalized): ("<<p_test_actual.x<<", "<<p_test_actual.y<<")"<<endl;

    //3.3.2 测试结果可视化
    // 可视化说明: 在 标准的chessboard图像 上绘制点来模拟在 世界平面 上识别出点（注意：第一个角点是3D世界坐标系的原点）；
    // 红色圆圈 表示测试点在世界平面上的 真实坐标，
    // 蓝色圆圈 表示输入图像平面坐标，利用Drareni方法转换矩阵 求出的测试点在世界平面中的坐标。
    cout<<endl<<"###################### 可视化Test ######################"<<endl;

    //3.3.2 (1): 从txt文件中读出测试点的 图像平面坐标 和 世界平面坐标
    vector<Point2f> img_coords;
    vector<Point2f> wrld_coords;
    ifstream inFile;
    inFile.open("./data/"+testImgFile+".txt", ios::in);  //存储当前这个测试图像上测试点坐标的txt文件
    string str, temp;  //str:用于存储从文件中读取的每一行的内容
    smatch result;
    regex pattern("([0-9]+[.][0-9]+,[0-9]+[.][0-9]+)");	//匹配括号内的内容
    int count = 0;

    //迭代器声明
    string::const_iterator iterStart;
    string::const_iterator iterEnd;

    while(getline(inFile, str))
    {
        iterStart = str.begin();
        iterEnd = str.end();

        while (regex_search(iterStart, iterEnd, result, pattern))
        {
            count++;  //每匹配到一次就计数
            temp = result[0];
            vector<string> substrings = string_split(temp,",");
            Point2f p = Point2f(atof(substrings[0].c_str()), atof(substrings[1].c_str()) );
            if(count%2==1)
            {
                img_coords.push_back(p);
            }
            else
            {
                wrld_coords.push_back(p);
            }
            iterStart = result[0].second;    //更新搜索起始位置,搜索剩下的字符串
        }
    }
    //end 3.4.1（至此，testPoints.txt中所有测试点的图像坐标和在3D世界中的坐标已被读取到vector<Point2f>中）
    int testPoint_num = img_coords.size();

    //3.3.2 (2): 正式进行可视化绘制
    string chsbd_std = imgFolder + "chessboard_standard.png";  //标准棋盘格图像（用于作为参照）
    Mat imgTest = imread(chsbd_std);
    Size img_grid_size = Size(89, 90);  //标准棋盘格图像平面上一个格子的尺寸：宽x高（单位：pixel）

    for(int i=0; i<testPoint_num; i++)
    {
        Point2f p_test_actual_i = wrld_coords[i];
        Point2f p_computed_i = Drareni_computWorldCoord(img_coords[i].x, img_coords[i].y, H, T_img, T_wrld);
        cout<<endl<<"测试点"+to_string(i+1)+":  ";
        cout<<"它在世界平面的真实坐标为("+to_string(p_test_actual_i.x)+","+to_string(p_test_actual_i.y)+"); ";
        cout<<"它由图像平面坐标计算出的在世界平面的坐标为("+to_string(p_computed_i.x)+","+to_string(p_computed_i.y)+")。 "<<endl;

        Point2f p_test_actual_draw = worldCoord_to_stdImageCoord(p_test_actual_i, actual_grid_size, img_grid_size);
        Point2f p_computed_draw = worldCoord_to_stdImageCoord(p_computed_i, actual_grid_size, img_grid_size);;

        circle(imgTest, p_test_actual_draw, 2, Scalar(0, 0, 255), 2);  //绘制真实坐标(Red)
        circle(imgTest, p_computed_draw, 2, Scalar(255, 0, 0), 2);  //绘制计算出的坐标(Blue)
    }
    imshow("Actual coord and computed coord on the Test Image", imgTest);
    while( (char)waitKey(0)!=27 )
        ;


    //******************** End of task 3 ********************

    return 0;
}