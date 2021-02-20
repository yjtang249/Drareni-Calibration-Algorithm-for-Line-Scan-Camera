#ifndef CALIBRATION_CALIBRATION_H
#define CALIBRATION_CALIBRATION_H

#endif //CALIBRATION_CALIBRATION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <vector>
#include <string>
using namespace std;
using namespace cv;

/**
 * ./calibration.h:
 *      该文件是一些标定时使用的函数的声明，这些函数的实现在 ./calibration.cpp 中。
 */

//declaration of funcs

//(Tool function)分割字符串
vector<string> string_split(const string& str, const string& delim);

//(Tool function)传入一个文件的路径（绝对路径或相对路径），提取出该文件的文件名（即路径中的最后一部分）
string extract_fileName_from_path(const string& filePath);

//(Tool function) 给定若干个2D点，和一个3X3的单转换矩阵，返回这组2D点转换后的点坐标
vector<Point2f> transform_homography(const vector<Point2f>& srcPts, const Mat& H);

//角点检测(包括亚像素精确化)
void cornerPointsDetect(vector<string>& imgNameList, const Size& board_size, vector<vector<Point2f>>& cornerPoints_seq, int draw, int save, int imgSave);

//计算3D场景点到每个view的（平均）重投影误差（用于评价标定结果）
double cal_reproj_err(const vector<vector<Point3f>>& objPoints_seq, const vector<vector<Point2f>>& cornerPoints_seq,
                      const Mat& cameraMat, const Mat& distCoeff, const vector<Mat>& rvecs, const vector<Mat>& tvecs);

//（相机标定（张氏标定法））输入图像和数据，进行角点检测和相机标定，并把标定结果（内参、外参）存储到.yml文件中（返回内参、外参，及检测到的每个view中的各个角点的坐标）
void cameraCalibration( vector<string>& imgNameList, const Size& board_size, const Size& square_size,
                        vector<vector<Point2f>>& cornerPoints_seq, const vector<vector<Point3f>>& objectPoints_seq,
                        Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs,
                        const string& intr_file, const string& extr_file);

//从指定的.yml文件中读取某个矩阵形式的参数（如内参矩阵、旋转向量等），并以矩阵形式返回
Mat getMat_from_yml(const string& fileName, const string& key);

//（位姿估计）用PnP方法求解每一个view对应的相机的位姿,每个位置的旋转向量和转移向量作为输出参数返回
void getPose_pnp( const vector<vector<Point3f>>& objectPoints_seq, const vector<vector<Point2f>>& cornerPoints_seq,
                  const string& intr_param, vector<Mat>& rvecs, vector<Mat>& tvecs, const string& extr_file);

//（求H: 2D->2D）给定 源图像路径 和 目标图像路径，对它们进行角点检测，并求得从 源图像坐标 转换到 目标图像坐标 的单应矩阵(2D->2D)。
//同时打印出这些点对应中一共有多少组inliers,多少组outliers，并把源图像和目标图像中inlier的坐标分别作为输出参数返回；
Mat computeHomography_2d(const string& img_src, const string& img_trgt, const Size& board_size, Mat& mask,
                         vector<Point2f>& src_points_inliers, vector<Point2f>& trgt_points_inliers);

//（求H: 2D->2D）给定 源图像中角点坐标 和 目标图像中对应点坐标，求得从 源图像坐标 转换到 目标图像坐标 的单应矩阵(2D->2D)。
//同时打印出这些点对应中一共有多少组inliers,多少组outliers，并把源图像和目标图像中inlier的坐标分别返回；
Mat computeHomography2_2d(const vector<Point2f>& src_points, const vector<Point2f>& trgt_points,
                          const Size& board_size, Mat& mask, vector<Point2f>& src_points_inliers, vector<Point2f>& trgt_points_inliers );

//给定两幅图像中的若干组点对应，以及从第一幅图像到第二幅图像的单应矩阵，计算出这些点对应的对称转移误差（用于评价单应矩阵求得怎么样）。
//源图像上的点投影后的坐标、目标图像上的点投影之后的坐标 作为输出参数返回。
double symmetricTransfer_error(vector<Point2f>& src_points, vector<Point2f>& trgt_points, Mat& homography,
                               vector<Point2f>& src_points_proj, vector<Point2f>& trgt_points_proj);

//在图像上绘制目标点：给定一张图片和一组点坐标，在该图片上用圆圈绘制这些点，并显示(按esc键退出)
void drawCircleOnImage(const string& imgName, const vector<Point2f>& points, const Scalar& color, const string& windowName,
                       int radius, int thickness);

//给定两个点(2个坐标)，求出它们之间的欧式距离
double euclid_distanc(const Point2f& p1, const Point2f& p2);

//给定一堆2维坐标的输入点，对它们进行标准化
void normalize_for_DLT(const vector<Point2f>& src_pts, vector<Point2f>& trgt_pts, Mat& T);

//（ Drareni ）用Drarenei标定方法（一种线阵相机动态扫描成像标定方法），获得从 升维世界坐标系 到图像坐标系 的转化矩阵(3X6)
Mat Drareni_computeHomography(const vector<Point2f>& worldPoints, const vector<Point2f>& imagePoints, Mat& T1, Mat& T2 );

//（ Drareni ）用Drarenei标定方法，给出 某点在图像平面上的坐标 和 求出的转换H矩阵(3X6)，返回该点在世界平面(世界坐标系中z=0的平面)上的坐标
Point2f Drareni_computWorldCoord(double u, double v, const Mat& H, const Mat& T1, const Mat& T2);

//这个函数用于test3.4，作用是将 标准棋盘格图像平面坐标系 上的坐标转换为在 世界平面坐标系 上的坐标。
Point2f worldCoord_to_stdImageCoord(const Point2f& wrldCoord, const Size& actual_grid_size, const Size& img_grid_size);

//(Tool function)坐标系(2D)手性转换。说得通俗点就是把原来的点x和y坐标互换
vector<Point2f> chirality_transformation(const vector<Point2f>& srcPts);

//（图像平面坐标系->机器人(平面)坐标系）给出一组点在 图像平面坐标系 的坐标，和4个需要用到的转换矩阵，得到这一组点在 机器人(平面)坐标系 的坐标；
vector<Point2f> imageToRobot(const vector<Point2f>& imgCoords, const Mat& H, const Mat& T_img, const Mat& T_wrld, const Mat& H_v,
                                   vector<Point2f>& worldCoords);
