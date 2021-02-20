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
    Size corner_point_size = Size(11,8);  //标定板上每行和每列各包含多少个角点
    Size actual_grid_size = Size(60, 60);  //在3D真实世界中，每个棋盘格子的实际(宽x高)，单位：mm

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
    string testImgFile = "lineScanImage2";  //test3.3要用的测试图像的名字（**********************）

    imgList.push_back(imgFolder+testImgFile+".jpg");
    //手动添加view完毕

    Mat cameraMatrix = Mat(3, 3, CV_64FC1, Scalar::all(0));
    Mat distCoeffs = Mat(1, 5, CV_64FC1, Scalar::all(0));


    //************task 2.获取(其实就是自己指定)各个角点在3d世界坐标系中的位置（以第一个角点为原点，宽为x轴正向，整个标定板为xoy平面建系）
    vector<vector<Point3f>> objectPoints_seq;  //用于保存 各个view中 各个角点 在3D世界坐标系中的坐标
    int viewNum = imgList.size();
    for(int i=0; i<viewNum; i++)  //每轮循环代表一个view
    {
        vector<Point3f> object_points;
        for(int j=0; j<corner_point_size.height; j++)
        {
            for(int k=0; k<corner_point_size.width; k++)
            {
                Point3f obj_point;
                obj_point.x = (double)k*actual_grid_size.width;
                obj_point.y = (double)j*actual_grid_size.height;
                obj_point.z = 0.0;
                object_points.push_back(obj_point);
            }
        }
        objectPoints_seq.push_back(object_points);
    }


    //3.执行相机标定（会先进行角点检测），求取并保存相机内参、畸变系数向量、外参到.yml文件
    vector<vector<Point2f>> cornerPoints_seq;  //用于接收检测到的每个view上的各个角点的坐标
    string intr_file = "./data/calibrationData/intrinsic.yml";  //相机内参的存储文件
    string extr_file = "./data/calibrationData/extrinsic.yml";  //相机外参的存储文件
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    cameraCalibration(imgList, corner_point_size, actual_grid_size, cornerPoints_seq, objectPoints_seq,
                      cameraMatrix, distCoeffs, rvecs, tvecs, intr_file, extr_file);  //***

    //4.用PnP方法求解相机外参（位姿），并保存到.yml文件
    string extr_file_pnp = "./data/calibrationData/extrinsic_pnp.yml";

    //4.1位姿估计，注：在etPose_pnp()会用cameraCalibration()得到的rvecs和tvecs的值作为初值，利用Levenberg-Marquardt优化算法进行更新
    getPose_pnp(objectPoints_seq, cornerPoints_seq, intr_file, rvecs, tvecs, extr_file_pnp);  //***
    //******************** End of task 1 ********************



//    //********************task 2.计算两幅图像之间的单应矩阵，并用重投影误差评估测试结果********************
//    cout<<endl;
//    cout<<"*****************Task 2: 计算两幅图像之间的单应矩阵，并用重投影误差评估测试结果*****************"<<endl;
//
//    Mat mask;  //用于储存哪几组点对应是inlier（值不为0），哪几组点对应是outlier（值为0）.
//    string source_img_name = imgFolder+"chessboard_org.png";  //源图片
//    string target_img_name = imgFolder+"chessboard4.png";  //目标图片
//
//    //2.1 计算单应矩阵，并返回源图像和目标图像中inliers的坐标
//    vector<Point2f> src_points_inliers;  //用于存放源图像中属于inlier的场景点
//    vector<Point2f> trgt_points_inliers;  //用于存放目标图像中属于inlier的场景点
//    Mat homography = computeHomography_2d(source_img_name, target_img_name, corner_point_size, mask,
//                                          src_points_inliers, trgt_points_inliers);  //***
//    //至此，得到了src_points_inliers、trgt_points_inliers中的值
//
//    //2.2 计算对称转移误差(Symmetric Transfer Error, STE)
//    vector<Point2f> src_points_proj;  //用于存放源图像上的点投影（到目标图像上）之后的点的坐标
//    vector<Point2f> trgt_points_proj;  //用于存放目标图像上的点投影（到源图像上）之后的点的坐标
//    symmetricTransfer_error(src_points_inliers, trgt_points_inliers, homography,
//                            src_points_proj, trgt_points_proj );  //***
//
//    //2.3 （在转移后的图片上）绘制转移点
//    //画圈圈（绘制转移过来的inlier点）
//    cout<< "********开始绘制带转移点的 源图像 和 目标图像******"<<endl;
//    cout << "绘制源图片（带目标图片上转移过来的点）:" << endl;
//    drawCircleOnImage(source_img_name, trgt_points_proj, Scalar(0,0,255), "Source Image with transfered points");
//
//    cout << "绘制目标图片（带源图片上转移过来的点）:" << endl;
//    drawCircleOnImage(target_img_name, src_points_proj, Scalar(255,0,0), "Target Image with transfered points");
//    //******************** End of task 2 ********************



    //********************task 3.使用Drareni算法计算单应矩阵H(3X6)、标准化矩阵T_img、T_wrld，并评估实验结果********************
    cout<<endl;
    cout<<"*****************Task 3: 使用Drareni算法计算单应矩阵H(3X6)、标准化矩阵T_img、T_wrld，并评估实验结果*****************"<<endl;
    //（20210129记录）目前还没有拿到线阵相机所拍的图像，用的是面阵相机所拍的图像测的
    // (20210205记录) 线扫相机成像已经ok，现在用的是线扫相机+厚的12x9的标定板成的图像

    //3.1 选定一张测试用的图像，同时把测试要用到的角点坐标转换成指定形式
    //指定用来测试的图像： view[0]，即在line30左右添加到imgList中的第一张图
    vector<Point2f> imagePoints = cornerPoints_seq[0];  //view[0]中的所有角点的图像坐标
    vector<Point3f> worldPoints_3d = objectPoints_seq[0];
    vector<Point2f> worldPoints;  //所有角点在 世界平面坐标系 中的坐标（手动赋值的）
    for(int i=0; i<imagePoints.size(); i++)
    {
        worldPoints.emplace_back(worldPoints_3d[i].x, worldPoints_3d[i].y );
    }

    //在测试图像中选定多少个角点参与运算
    int wrldPts_num = corner_point_size.width*corner_point_size.height;
    vector<Point2f> imgPoints_atd;
    vector<Point2f> wrldPoints_atd;
    for(int i=0; i<wrldPts_num; i++)
    {
        imgPoints_atd.push_back(imagePoints[ i ]);
        wrldPoints_atd.push_back(worldPoints[ i ]);
    }

    //3.2 计算Drareni算法的转换矩阵H(3X6) (normalized World coord -> normalized Image coord)、
    // 图像平面的标准化矩阵T_img(3X3)、世界平面的标准化矩阵T_wrld(3X3)
    //3.2.1 计算H、T_img、T_wrld
    Mat T_img, T_wrld;
    Mat H = Drareni_computeHomography(wrldPoints_atd, imgPoints_atd, T_img, T_wrld);  //***标定
    cout<<"H( normalized World coord -> normalized Image coord): "<<endl<<H<<endl;
    cout<<"T_img( Image Plane coord -> normalized Image Plane coord ): "<<endl<<T_img<<endl;
    cout<<"T_wrld( World Plane coord -> normalized Iorld Plane coord ): "<<endl<<T_wrld<<endl;

    //3.2.2 将H、T_img、T_wrld存储到.yml文件中
    cv::FileStorage imageToWorld("./data/calibrationData/imageToWorld.yml", FileStorage::WRITE);
    imageToWorld << "H" <<H;
    imageToWorld << "T_img" << T_img;
    imageToWorld << "T_wrld" << T_wrld;
    imageToWorld.release();
    cout<<"Drareni算法算出的结果矩阵：H(3X6)、T_img(3X3)、T_wrld(3X3)已经存储完毕。"<<endl;


//    //3.3 在图像平面上选择一个点，看看用转换矩阵把它转换到 世界平面坐标系 中得到的坐标与其 在世界平面坐标系中的真是坐标 差多少
//    // 3.3.1 直接输出计算后的坐标与真实世界坐标对比
////    int test_point_ptId = 21;  //被选中的测试点的id：我看看把这个图像平面上的点坐标用Drareni方法求出的 H矩阵 反映射到世界平面，得到的坐标是多少
////    Point2f p_test = imagePoints[test_point_ptId];
//    Point2f p_test = Point2f(226.0,196.0);  //测试点（1号黑点） 在图像平面的真实坐标（单位：pixel）
//
//    Point2f p_computed = Drareni_computWorldCoord(p_test.x, p_test.y, H, T_img, T_wrld);  //***
//    cout<<"computed coordinate of the Test Point in World Plane (denormalized): "<<p_computed<<endl;
//
//    Point2f p_test_actual = Point2f(47.0,31.5);  //测试点 在世界平面上的真实坐标（单位：mm）
////    cout<<"actual coordinate in World Plane (denormalized): "<<worldPo ints[test_point_ptId]<<endl;
//    cout<<"actual coordinate of the Test Point in World Plane (denormalized): ("<<p_test_actual.x<<", "<<p_test_actual.y<<")"<<endl;
//
//    //3.3.2 测试结果可视化
//    // 可视化说明: 在 标准的chessboard图像 上绘制点来模拟在 世界平面 上识别出点（注意：第一个角点是3D世界坐标系的原点）；
//    // 红色圆圈 表示测试点在世界平面上的 真实坐标，
//    // 蓝色圆圈 表示输入图像平面坐标，利用Drareni方法转换矩阵 求出的测试点在世界平面中的坐标。
//    cout<<endl<<"###################### 可视化Test ######################"<<endl;
//
//    //3.3.2 (1): 从txt文件中读出测试点的 图像平面坐标 和 世界平面坐标
//    vector<Point2f> img_coords;
//    vector<Point2f> wrld_coords;
//    ifstream inFile;
//    inFile.open("./data/"+testImgFile+".txt", ios::in);  //存储当前这个测试图像上测试点坐标的txt文件
//    string str, temp;  //str:用于存储从文件中读取的每一行的内容
//    smatch result;
//    regex pattern("([-]?[0-9]+[.][0-9]+,[-]?[0-9]+[.][0-9]+)");	//匹配括号内的内容
//    int count = 0;
//
//    //迭代器声明
//    string::const_iterator iterStart;
//    string::const_iterator iterEnd;
//
//    while(getline(inFile, str))
//    {
//        iterStart = str.begin();
//        iterEnd = str.end();
//
//        while (regex_search(iterStart, iterEnd, result, pattern))
//        {
//            count++;  //每匹配到一次就计数
//            temp = result[0];
//            vector<string> substrings = string_split(temp,",");
//            Point2f p = Point2f(atof(substrings[0].c_str()), atof(substrings[1].c_str()) );
//            if(count%2==1)
//            {
//                img_coords.push_back(p);
//            }
//            else
//            {
//                wrld_coords.push_back(p);
//            }
//            iterStart = result[0].second;    //更新搜索起始位置,搜索剩下的字符串
//        }
//    }
//    //end 3.3.2 (1)（至此，testPoints.txt中所有测试点的图像坐标和在3D世界中的坐标已被读取到vector<Point2f>中）
//    int testPoint_num = img_coords.size();
//
//    //3.3.2 (2): 正式进行可视化绘制
//    string chsbd_std = imgFolder + "chessboard_standard.png";  //标准棋盘格图像（用于作为参照）
//    Mat imgTest = imread(chsbd_std);
//    Size img_grid_size = Size(89, 90);  //标准棋盘格图像平面上一个格子的尺寸：宽x高（单位：pixel）
//
//    for(int i=0; i<testPoint_num; i++)
//    {
//        Point2f p_test_actual_i = wrld_coords[i];
//        Point2f p_computed_i = Drareni_computWorldCoord(img_coords[i].x, img_coords[i].y, H, T_img, T_wrld);
//        cout<<endl<<"测试点"+to_string(i+1)+":  ";
//        cout<<"它在世界平面的真实坐标为("+to_string(p_test_actual_i.x)+","+to_string(p_test_actual_i.y)+"); ";
//        cout<<"它由图像平面坐标计算出的在世界平面的坐标为("+to_string(p_computed_i.x)+","+to_string(p_computed_i.y)+")。 "<<endl;
//
//        Point2f p_test_actual_draw = worldCoord_to_stdImageCoord(p_test_actual_i, actual_grid_size, img_grid_size);
//        Point2f p_computed_draw = worldCoord_to_stdImageCoord(p_computed_i, actual_grid_size, img_grid_size);;
//
//        circle(imgTest, p_test_actual_draw, 2, Scalar(0, 0, 255), 2);  //绘制真实坐标(Red)
//        circle(imgTest, p_computed_draw, 2, Scalar(255, 0, 0), 2);  //绘制计算出的坐标(Blue)
//    }
//    imshow("Actual coord and computed coord on the Test Image", imgTest);
//    imwrite("./img/result/testPointsCompar_"+testImgFile+".png", imgTest);  //将测试点的 计算出的世界平面坐标 和 真实世界平面坐标 的可视化对比图保存
//    while( (char)waitKey(0)!=27 )
//        ;
//    //end 3.3

    //******************** End of task 3 ********************



    //********************task 4.计算 世界平面坐标系 到 机器人平面坐标系 的转换关系（一劳永逸）********************
    cout<< "********************task 4.加上世界平面坐标系到机器人坐标系的转换关系（一劳永逸）********************" <<endl;

    //4.1 建立世界坐标系和机器人坐标系的关系（计算出转换矩阵H_v，并保存到.yml文件中）
    //4.1.1 testPoints_Robot.txt中走的点在机器人坐标系下的坐标
    vector<Point2i> testPoints_robot_id;  //用来存储这些走的角点的行列号
    vector<Point2f> testPoints_robot_coord;  //用来存储这些走的角点在机器人坐标系下的坐标

    ifstream inFile2;
    inFile2.open("./data/tsk4_testPoints_Robot2.txt", ios::in);  //该文件存储了走的点在 机器人平面坐标系 中的坐标
    string str2;  //str2:用于存储从文件中读取的每一行的内容
    vector<string> splitted_substring;
    vector<string> splitted_substring_num;
    vector<string> splitted_substring_coord;
    //迭代器声明

    while(getline(inFile2, str2))
    {
        if(str2.empty())
            continue;
        splitted_substring = string_split(str2,";");
        splitted_substring_num = string_split(splitted_substring[0], ",");
        splitted_substring_coord = string_split(splitted_substring[1], ",");
        Point2i rowAndCol_num = Point2i( stoi(splitted_substring_num[0]), stoi(splitted_substring_num[1]) );
        Point2f pt_coord = Point2f( stof(splitted_substring_coord[0]), stof(splitted_substring_coord[1]) );

        testPoints_robot_id.push_back(rowAndCol_num);
        testPoints_robot_coord.push_back(pt_coord);

        //每轮循环结束记得清空那三个临时的vector<string>
        splitted_substring.clear();
        splitted_substring_num.clear();
        splitted_substring_coord.clear();
    }

    //4.1.2 将对应点的世界坐标提取到一个vetor<Point2f>中
    vector<Point2f> testPoints_world_coord;  //用来存储这些走的点的世界坐标（这些坐标就是 行号、列号 乘上 格子的宽、高 得来的）
    int verifyingPoint_num = testPoints_robot_id.size();  //一共走了多少个点
    for(int i=0; i<verifyingPoint_num; i++)
    {
        Point2f verifyingPoint = Point2f( actual_grid_size.width*testPoints_robot_id[i].y,
                                          actual_grid_size.height*testPoints_robot_id[i].x  );
//        cout<<"第"+to_string(i+1)+"个走的点在 世界平面 的坐标："<<verifyingPoint<<endl;
        testPoints_world_coord.push_back(verifyingPoint);
    }
    //至此，testPoints_world_coord填充完毕

    //4.1.3 由于世界坐标系和机器人坐标系的手性是不同的，所以这里我们要把 世界坐标系 先转换到 过渡世界坐标系，
    //过渡世界坐标系 其实就是把原来的 世界坐标系 的x轴和y轴互换了一下
    vector<Point2f> testPoints_interimWorld_coord = chirality_transformation(testPoints_world_coord);
    cout<<"世界平面坐标系 到 过渡世界平面坐标系 的转换已完成！"<<endl;

    //4.1.4 计算两组2D点之间的单应：H_v（src:过渡世界坐标系，trgt:机器人坐标系）
    Mat mask_verifing;
    vector<Point2f> verifying_inliers_src;
    vector<Point2f> verifying_inliers_trgt;
    Mat H_v = computeHomography2_2d(testPoints_interimWorld_coord, testPoints_robot_coord, corner_point_size,
                                            mask_verifing, verifying_inliers_src, verifying_inliers_trgt );
    // H_v(v表示verify)：从 过渡世界坐标系 到 机器人坐标系 的坐标转换矩阵

    //4.1.5 保存H_v到.yml文件
    cv::FileStorage wrldToRbt("./data/calibrationData/worldToRobot.yml", FileStorage::WRITE);
    wrldToRbt << "H_v" <<H_v;
    wrldToRbt.release();
    cout<<"过渡世界平面坐标系 到 机器人(平面)坐标系 的单应矩阵H_v已经存储完毕。"<<endl;

    //******************** End of task 4 ********************



    //********************task 5.(Test) 最终测试********************
    //给定一些（待抓取）物体（可能很大）在 图像平面 上的坐标（去图像中用PS量），利用H、T_img、T_wrld、H_v计算出它们在 机器人坐标系（平面）
    //中的坐标，并将计算出的 机器人坐标（简称） 与它们真实的 机器人坐标（用走点的方法测量出来的）对比，看差多少。
    cout<<endl<<"******************** task 5.(Test) 最终测试********************" <<endl;

    //5.1 从.yml文件中读取需要用到的矩阵
    //这些变量分别用来存储从xxx.yml文件中读出来的矩阵
    Mat H1(3,6,CV_64FC1);
    Mat T_img1(3,3,CV_64FC1);
    Mat T_wrld1(3,3,CV_64FC1);
    Mat H_v1(3,3,CV_64FC1);

    string file_imageToWorld = "./data/calibrationData/imageToWorld.yml";
    string file_worldToRobot = "./data/calibrationData/worldToRobot.yml";
    FileStorage fs_imgToWorld(file_imageToWorld, FileStorage::READ);
    FileStorage fs_worldToRobot(file_worldToRobot, FileStorage::READ);

    fs_imgToWorld["H"]>>H1;
    fs_imgToWorld["T_img"]>>T_img1;
    fs_imgToWorld["T_wrld"]>>T_wrld1;
    fs_worldToRobot["H_v"]>>H_v1;

    cout<<endl<<"test task5"<<endl;
    cout<<"H"<<endl<<H1<<endl;
    cout<<"T_img"<<endl<<T_img1<<endl;
    cout<<"T_wrld"<<endl<<T_wrld1<<endl;
    cout<<"H_v"<<endl<<H_v1<<endl;

    //test:测试一下H_v求得怎么样
    cout<<endl<<"插入Test: 测试一下求得的从 过渡世界平面坐标系 到 机器人坐标系 的转换矩阵H_v准不准："<<endl;
    vector<Point2f> src_points_proj;  //源平面（过渡世界平面）上的点投影到目标平面（机器人平面）的点坐标（待填充）
    vector<Point2f> trgt_points_proj;  //目标平面（机器人平面）上的点投影回源平面（过渡世界平面）的点坐标（待填充）
    double symError = symmetricTransfer_error(testPoints_interimWorld_coord, testPoints_robot_coord, H_v, src_points_proj, trgt_points_proj);
    cout<<"接下来输出走的点在 源平面（过渡世界平面）上的坐标，和从目标平面（机器人平面）投影到源平面 的坐标"<<endl;
    for(int i=0; i<verifyingPoint_num; i++)
    {
        cout<<"srcPoint "+to_string(i+1)+":"<<testPoints_interimWorld_coord[i]<<"  ;  "<<trgt_points_proj[i]<<endl;
    }
    cout<<"接下来输出走的点在 目标平面（机器人平面）上的坐标，和从源平面（过渡世界平面）投影到目标平面 的坐标"<<endl;
    for(int i=0; i<verifyingPoint_num; i++)
    {
        cout<<"targetPoint "+to_string(i+1)+":"<<testPoints_robot_coord[i]<<"  ;  "<<src_points_proj[i]<<endl;
    }
    cout<<"插入Test结束！"<<endl;
    // end test:测试一下H_v求得怎么样

    //5.2 从.txt文件中读出 待抓物体的 图像平面坐标系的坐标 和 机器人坐标系的坐标
    string fileToBeLoaded = "./data/tsk5_objectToBeGrasped2.txt";  //待加载文件目录（包含待抓物体的 图像平面坐标系的坐标 和 机器人坐标系的坐标）
    vector<Point2f> obj_img_coords;
    vector<Point2f> obj_robot_coords;
    ifstream imgAndRobot_coord;
    imgAndRobot_coord.open(fileToBeLoaded, ios::in);
    string str5, temp5;  //str:用于存储从文件中读取的每一行的内容
    smatch result5;
    regex pattern5("([-]?[0-9]+[.][0-9]+,[-]?[0-9]+[.][0-9]+)");	//匹配括号内的内容
    int count5 = 0;

    //迭代器声明
    string::const_iterator iterStart5, iterEnd5;

    while(getline(imgAndRobot_coord, str5))
    {
        iterStart5 = str5.begin();
        iterEnd5 = str5.end();
        while (regex_search(iterStart5, iterEnd5, result5, pattern5))
        {
            count5++;  //每匹配到一次就计数
            temp5 = result5[0];
            vector<string> substrings = string_split(temp5,",");
            Point2f p = Point2f(atof(substrings[0].c_str()), atof(substrings[1].c_str()) );
            if(count5%2==1)
            {
                obj_img_coords.push_back(p);
            }
            else
            {
                obj_robot_coords.push_back(p);
            }
            iterStart5 = result5[0].second;    //更新搜索起始位置,搜索剩下的字符串
        }
    }
    //end 5.2 读取正常，obj_img_coords和obj_robot_coords填充完毕

    //5.3(版本一) 用待抓取的那几个物体的图像坐标来测
    //5.3.1 计算。
    // 用（5.1）从.yml文件中读取出来的矩阵 和 (5.2)从.txt文件中读取出来的 待抓物体在图像平面坐标系的坐标 计算待抓物体在 机器人坐标系中的坐标
    vector<Point2f> worldCoords;
    vector<Point2f> to_robot = imageToRobot(obj_img_coords, H1, T_img1, T_wrld1, H_v1, worldCoords);

//    //这是另一种写法，用symmetricTransfer_error()函数来得到映射后的坐标
//    vector<Point2f> to_interimWorld, to_robot;
//    double err2 = symmetricTransfer_error(obj_interimWorld_coords, obj_robot_coords, H_v1, to_robot, to_interimWorld);

    cout<<endl<<"接下来输出这些物体在 机器人坐标系的真实坐标 和 计算出的机器人坐标系坐标："<<endl;
    int objNum = to_robot.size();
    for(int i=0; i<objNum; i++)
    {
        cout<<"Object "+to_string(i+1)+": "<<obj_robot_coords[i]<<" ; "<<to_robot[i]<<endl;
    }

    //5.3.2 可视化。根据待抓取物体在 世界平面坐标系 的坐标，在lineScan_cut.png上绘制出位置，看看是不是在物体的范围内
    vector<Point2f> stdImg_coords;
    stdImg_coords.reserve(objNum);
    for(int i=0; i<objNum; i++)
    {
        if(worldCoords[i].x<-60||worldCoords[i].y<-60)
            continue;  //这里我们只绘制x和y坐标都大于-60的物体
        Point2f stdImg_coord = worldCoord_to_stdImageCoord(worldCoords[i], actual_grid_size, Size(140,142) );
        stdImg_coords.emplace_back(stdImg_coord);
    }
    drawCircleOnImage("./img/lineScanImage2_cut.png", stdImg_coords, Scalar(0,0,255), "Object to be grasped", 40, 20);

    //end 5.3(版本一)

//    //5.3(版本二) 用走点时的那几个点 的图像坐标来测
//    vector<Point2f> obj_img_coords2;  //走点的点 在图片坐标系中的坐标
//    obj_img_coords2.reserve(9);
//    obj_img_coords2.emplace_back( Point2f(1256.0,6095.0) );
//    obj_img_coords2.emplace_back( Point2f(1256.0,5551.0) );
//    obj_img_coords2.emplace_back( Point2f(1256.0,5013.0) );
//    obj_img_coords2.emplace_back( Point2f(1684.0,6105.0) );
//    obj_img_coords2.emplace_back( Point2f(1686.0,5552.0) );
//    obj_img_coords2.emplace_back( Point2f(1685.0,5008.0) );
//    obj_img_coords2.emplace_back( Point2f(2112.0,6107.0) );
//    obj_img_coords2.emplace_back( Point2f(2106.0,5559.0) );
//    obj_img_coords2.emplace_back( Point2f(2102.0,5019.0) );
//    vector<Point2f> obj_robot2=testPoints_robot_coord;  //走点的点
//
//    vector<Point2f> worldCoords;
//    vector<Point2f> obj_interimWorld_coords = imageToRobot(obj_img_coords2, H1, T_img1, T_wrld1, H_v1, worldCoords);
//    vector<Point2f> to_interimWorld, to_robot;
//    double err2 = symmetricTransfer_error(obj_interimWorld_coords, obj_robot2, H_v1, to_robot, to_interimWorld);
//    cout<<endl<<"我他妈倒要看看是哪里不对："<<endl;
//    cout<<"在机器人坐标系下的真实的坐标， 转移到机器人坐标系下的坐标"<<endl;
//    for(int i=0; i<to_robot.size(); i++)
//    {
//        cout<<"Obj "+to_string(i+1)+": "<<obj_robot2[i]<<"  ;  "<<to_robot[i]<<endl;
//    }
//    //end 5.3(版本二)：测出来效果还可以，只差了几个像素

    return 0;
}