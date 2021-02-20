#include "calibration.h"
#include <math.h>

using namespace std;
using namespace cv;

/**
 * ./calibration.cpp:
 *      该文件是一些标定时使用的工具函数的实现，以及相关的说明。
 */


/*
 * func: 分割字符串
 * args:
 *     str: 待分割的字符串；
 *     delim: 分割符；
 * return:
 *     分割得到的子串组列表。
 * */
vector<string> string_split(const string& str, const string& delim)
{
    vector<string> res;
    if ("" == str) return res;
    //先将要切割的字符串从string类型转换为char*类型
    char *strs = new char[str.length() + 1]; //不要忘了
    strcpy(strs, str.c_str());

    char *d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while (p) {
        string s = p; //分割得到的字符串转换为string类型
        res.push_back(s); //存入结果数组
        p = strtok(NULL, d);
    }

    return res;
}


/*
 * func: 传入一个文件的路径（绝对路径或相对路径），提取出该文件的文件名（即路径中的最后一部分）
 * args:
 *      filePath: 传入文件的路径；
 * return:
 *      该文件的文件名。
 * */
string extract_fileName_from_path(const string& filePath)
{
    vector<string> splitted_str = string_split(filePath, "/");
    int substring_num = splitted_str.size();
    return splitted_str[substring_num-1];  //提取出用"/"分割 文件路径字符串 的最后一个子串
}


/*
 * func: 给定若干个2D点，和一个3X3的单转换矩阵，返回这组2D点转换后的点坐标
 * args:
 *     srcPts: （源平面上）待转换的若干个2D点的坐标；
 * return:
 *      转换后的目标平面上的2D点的坐标。
 * */
vector<Point2f> transform_homography(const vector<Point2f>& srcPts, const Mat& H)
{
    int ptsNum = srcPts.size();
    vector<Point2f> srcPts_proj;
    srcPts_proj.reserve(ptsNum);  //因为容器的大小是固定的，因此在往里添加元素之前，可以先预留好空间（避免不必要的重新分配）
    Mat X = Mat(3, ptsNum, CV_64FC1, Scalar::all(1) );

    for(int i=0; i<ptsNum; i++)
    {
        X.at<double>(0,i) = srcPts[i].x;
        X.at<double>(1,i) = srcPts[i].y;
    }

    Mat X_proj = H*X;
    for(int i=0; i<ptsNum; i++)
    {
        srcPts_proj.emplace_back( Point2f( X_proj.at<double>(0,i)/X_proj.at<double>(2,i), X_proj.at<double>(1,i)/X_proj.at<double>(2,i) ) );
    }

    return srcPts_proj;
}


/**
 * func:角点检测(包括亚像素精确化)
 * args:
 *      imgNameList: 各个待提取的图像的路径；
 *      board_size: chessboard上每行和每列内叉点的个数，格式为: cv::Size(points_per_row, points_per_col) ;
 *      cornerPoints_seq: 检测到的角点；
 *      draw: 表示检测完之后是否绘制带角点的图像；
 *      save: 表示检测完之后是否把识别出的角点坐标保存到文件；
 *      imgSave: 表示是否将绘制的带角点的图像输出，只有当draw为1且该参数为1时才有效（输出到./img/result/路径下，输出图像的名字为 withCorners_xxx，其中xxx为原图像名）
 * return:
 *      none.
 */
void cornerPointsDetect(vector<string>& imgNameList, const Size& board_size, vector<vector<Point2f>>& cornerPoints_seq, int draw=1, int save=1, int imgSave=1)
{
    int imgNum = imgNameList.size();  //待提取图像的数量
    vector<Point2f> cornerPoints;  //缓存每幅图像上检测到的角点
    for(int i =0; i<imgNum; i++)
    {
        Mat imageInput = imread(imgNameList[i]);  //当前处理的图像
        Mat imageGray;  //如果相机本来成的就是灰度图，那这个就不需要了，不用再额外进行RGB转灰度图的步骤
        cout << "imageInput.channels()=" << imageInput.channels() <<endl;

        cvtColor(imageInput, imageGray, COLOR_RGB2GRAY);  //把原图转换成灰度图（方便进行亚像素精确化）
        int fineded = findChessboardCorners(imageGray, board_size, cornerPoints, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FILTER_QUADS );
        if(fineded == 0 )
        {
            cout<<"第"<<i+1<<"张图像角点提取失败！"<<endl;
            imshow("Fail to extract all corner Points", imageInput);
            waitKey(0);
        }
        else
        {
      //说明：由于cornerSubPix()函数必须传入Point2f的vector，而我为了后面计算的精度决定用Point2f，所以老子不做亚像素精确化了！！！
            //1.2 亚像素精确化: cornerSubPix()，该函数的作用是更加精确地修正角点的位置，是基于位置的修正，所以输入图像应为单通道
            // refinijng之后的角点坐标还是储存在第二个参数( vector<Point2f> ) 中
            cv::cornerSubPix(imageGray, cornerPoints, Size(5,5),
                             Size(-1,-1),
                             cv::TermCriteria(TermCriteria::EPS+TermCriteria::COUNT,20, 0.01 ));

            cornerPoints_seq.push_back(cornerPoints);  //保存当前图片存的精确角点

            //1.3 在图像上绘制一下角点，按ese退出（继续）
            if(draw != 0)
            {
                cout << "绘制第" << to_string(i + 1) << "张图片（带角点）:" << endl;
                cv::drawChessboardCorners(imageInput, board_size, cornerPoints, true);
                imshow("image " + to_string(i) + " with detected corner points", imageInput);
                if(imgSave != 0)
                {
                    string origin_imgName = extract_fileName_from_path(imgNameList[i]);
                    imwrite("./img/result/withCorners_"+origin_imgName, imageInput);
                }
                while ((char) waitKey(0) != 27);
            }

        }
    }
    //1.4 保存角点坐标到.txt文件
    if(save != 0)
    {
        ofstream out;
        for(int i=0; i<imgNum; i++)
        {
            string full_imgName = imgNameList[i];  //当前图像的文件名（完整路径）
            string imgName_withSuffix = string_split(full_imgName, "/").back();  //当前图像的文件名（仅仅是这个图像文件的名字，带后缀，如.png）
            string imgName_noSuffix = string_split(imgName_withSuffix, ".")[0];  //当前图像的文件名（仅仅是这个图像文件的名字，不带后缀）
            string corner_file = "./data/cornerPoints_"+imgName_noSuffix+".txt";  //保存角点的.txt文件的文件名

            int cornerPoint_num = cornerPoints_seq[i].size();  //该图像共有多少个角点
            out.open(corner_file, ios::out | ios::trunc);

            for (int j = 0; j < cornerPoint_num; j++)
                out<< cornerPoints_seq[i][j].x <<","<< cornerPoints_seq[i][j].y<<endl;

            out.close();
        }
    }
    cout<<"角点检测完成！"<<endl;
}//end func


/**
 * func:计算3D场景点到每个view的（平均）重投影误差（用于评价标定结果）
 * args:
 *     objPoints_seq: 所有view上各个点的3d世界坐标；
 *     cornerPoints_seq: 所有view上各个点在图像平面上的坐标；
 *     cameraMat, distCoeff: 摄像机矩阵和畸变系数；
 *     rvecs, tvecs:  每个view的旋转向量和位移向量；
 * return:
 *     所有view上所有点的平均重投影误差。
 * */
double cal_reproj_err(const vector<vector<Point3f>>& objPoints_seq, const vector<vector<Point2f>>& cornerPoints_seq,
        const Mat& cameraMat, const Mat& distCoeff, const vector<Mat>& rvecs, const vector<Mat>& tvecs)
{
    double per_err = 0.0;  //当前这幅图像上的所有点重投影误差之和
    double total_err = 0.0;  //当前已经计算了的图像上的所有点重投影误差之和
    int total_points_num = 0;  //所有view上的点的总个数
    int imgNum = objPoints_seq.size();  //view的数量
    vector<Point2f> reproj_points;  //（暂时）保存重新投影的点

    for(int i=0; i<imgNum; i++)
    {
        projectPoints(objPoints_seq[i], rvecs[i], tvecs[i], cameraMat, distCoeff, reproj_points);
        per_err = norm(reproj_points, cornerPoints_seq[i], NORM_L2);
        total_points_num += objPoints_seq[i].size();
        total_err += per_err;
        cout<<"第"<<i+1<<"幅图像上的平均(每点)的误差为:"<<per_err/objPoints_seq[i].size()<<endl;
    }
    double avg_err = total_err/total_points_num;

    return avg_err;
}


/**
 * func:输入图像和数据，进行角点检测和相机标定，并把标定结果（内参、外参）存储到.yml文件中
 * args:
 *      imgNameList: 各个chesssboard图像的路径；
 *      board_size: chessboard上每行和每列内叉点的个数，格式为: cv::Size(points_per_row, points_per_col) ;
 *      square_size: 在真实的3d世界坐标系中，每一个棋盘格的大小，格式为为：cv::Size(square_width, square_height) ;
 *      cornerPoints_seq 检测到的每个view中的各个角点的坐标（输出参数）；
 *      objectPoints_seq: 各个角点在3D世界坐标系中的坐标（输入参数）；
 *      cameraMatrix, distCoeffs, rvecs, tvecs: 标定得到的结果（输出参数）；
 *      intri_file: 标定得到的内参要保存到的保存文件名（输入参数），默认值为"noSaving"，表示不把结果保存到文件；
 *      extri_file: 标定得到的外参要保存到的保存文件名（输入参数），默认值为"noSaving"，表示不把结果保存到文件；
 * return:
 *      none.
 * */
void cameraCalibration( vector<string>& imgNameList, const Size& board_size, const Size& square_size,
        vector<vector<Point2f>>& cornerPoints_seq, const vector<vector<Point3f>>& objectPoints_seq,
        Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs,
        const string& intr_file="noSaving", const string& extr_file="noSaving")
{
    //step 1.角点检测
    cout<< "*******开始角点检测。*********"<<endl;
    int imgNum = imgNameList.size();  //图像的张数

    //输出一下图像的尺寸
    Size imgSize;    //图像的尺寸，自己设置的（若各不相同则以第一张的尺寸为准）
    Mat imageInput = imread(imgNameList[0]);
    imgSize.height = imageInput.rows;
    imgSize.width = imageInput.cols;
    cout<< "imgSize.width = "<<imgSize.width<<", imgSize.height = "<<imgSize.height<<endl;  //把宽高输出一下

    //倒数第3个参数为1，表示在图像上绘制检测到的角点；倒数第2个参数为1，表示将每幅图像上检测到的角点坐标保存到.txt文件中
    cornerPointsDetect(imgNameList, board_size, cornerPoints_seq, 1, 1, 1);


    //step 2. 求相机内参（外参也求出来了）
    //2.1 运行标定过程，获得 相机内参矩阵、畸变系数向量 + 各个view的 旋转矩阵（向量形式）、位移向量
    cout<< "************开始进行相机标定：***************"<<endl;
    double reproj_error = cv::calibrateCamera(objectPoints_seq, cornerPoints_seq, imgSize,
                            cameraMatrix, distCoeffs, rvecs, tvecs, CALIB_FIX_K3);
    cout<< "标定完成！" <<endl;

    //2.2 评价标定结果（计算重投影误差）
    cout<< "************评价标定结果：***************"<<endl;
    double my_reproj_error = cal_reproj_err(objectPoints_seq, cornerPoints_seq, cameraMatrix, distCoeffs, rvecs, tvecs);
    cout << "平均重投影误差为:"<<my_reproj_error<<endl;

    //2.3 保存结果：保存到.yml文件中
    bool saveIntr = !(intr_file=="noSaving");  //是否要保存内参到文件
    bool saveExtr = !(extr_file=="noSaving");  //是否要保存内参到文件

    // 2.3.1(若函数参数给出了要保存到的内参文件名)保存内参
    if(saveIntr)
    {
        cv::FileStorage fs1(intr_file, FileStorage::WRITE);
        fs1 << "intrinsic" << cameraMatrix;
        fs1 << "distCoeff" << distCoeffs;
        fs1.release();  //close fs1
    }

    // 2.3.2(若函数参数给出了要保存到的外参文件名)保存每个view的外参
    if(saveExtr)
    {
        cv::FileStorage fs2(extr_file, FileStorage::WRITE);
        Mat rmat;
        for (int i = 0; i < imgNum; i++) {
            Rodrigues(rvecs[i], rmat);
            fs2 << "RotatVect" + to_string(i + 1) << rvecs[i];
            fs2 << "RotatMat" + to_string(i + 1) << rmat;
            fs2 << "TransVec" + to_string(i + 1) << tvecs[i];
        }
        fs2.release();
    }
}// end func


/**
 * func:从指定的.yml文件中读取某个矩阵形式的参数（如内参矩阵、旋转向量等），并以矩阵形式返回；
 * args:
 *      fileName:要读取的.yml文件名；
 *      key:要读取的参数（矩阵形式）的key名；
 * return:
 *      读取到的参数(矩阵形式)；
 * */
Mat getMat_from_yml(const string& fileName, const string& key)
{
    FileStorage fs(fileName, FileStorage::READ);
    Mat matReaded;
    fs[key] >> matReaded;
    cout<< "参数"<<key<<"读取成功"<<endl;
    return matReaded;
}//end func


/**
 * func:用PnP方法求解每一个view对应的相机的位姿
 * args:
 *      objectPoints: 各个view中的各个点在3d世界坐标系中的坐标；
 *      cornerPoints: 各个view中的各个点在对应的图像平面中的坐标；
 *      intri_param: 要读取的存储内参的文件的文件名:
 *      rvecs, tvecs: 各个view的位姿估计结果（输出参数）；
 *      extri_file: 标定得到的外参保存地址（输入参数），默认值为"noSaving"，表示不把结果写入文件；
 * return:
 *      none.
 */
void getPose_pnp( const vector<vector<Point3f>>& objectPoints_seq, const vector<vector<Point2f>>& cornerPoints_seq,
        const string& intr_file, vector<Mat>& rvecs, vector<Mat>& tvecs, const string& extr_file="noSaving")
{
    Mat cameraMat = getMat_from_yml(intr_file, "intrinsic");  //读取相机内参矩阵
    Mat distCoeff = getMat_from_yml(intr_file, "distCoeff");  //读取相机的畸变系数向量
    bool savePose = !(extr_file == "noSaving");  //表示求出来的位姿是否需要保存到文件
    FileStorage fs(extr_file, FileStorage::WRITE);

    int view_num = objectPoints_seq.size();  //有多少个view
    vector<Point3f> objectPoints;
    vector<Point2f> cornerPoints;
    Mat rmat;
    cout<< "************开始用PnP方法估计相机位姿：***************"<<endl;
    for(int i =0; i<view_num; i++)
    {
        Mat rvec = rvecs[i];
        Mat tvec = tvecs[i];
        objectPoints = objectPoints_seq[i];
        cornerPoints = cornerPoints_seq[i];
        solvePnP(objectPoints, cornerPoints, cameraMat, distCoeff, rvec, tvec, true);
        rvecs[i] = rvec;
        tvecs[i] = tvec;
//        rvecs.push_back(rvec);
//        tvecs.push_back(tvec);

        if(savePose)  //保存pose到文件
        {
            Rodrigues(rvec, rmat);
            fs << "RotatVect" + to_string(i + 1) << rvec;
            fs << "RotatMat" + to_string(i + 1) << rmat;
            fs << "TransVec" + to_string(i + 1) << tvec;
        }
    }

    fs.release();
    cout<<"************开始计算平均重投影误差：***************"<<endl;
    double avg_reproj_err = cal_reproj_err(objectPoints_seq, cornerPoints_seq, cameraMat, distCoeff, rvecs, tvecs);
    cout<< "用PnP位姿估计方法得到的外参计算出的平均重投影误差为:"<<avg_reproj_err<<endl;

    cout<<"PnP位姿估计完成!"<<endl;

}//end func


/**
 * func:给定源图像和目标图像路径，对它们进行角点检测，并求得从 源图像坐标 转换到 目标图像坐标 的单应矩阵(2D->2D)。
 *      同时打印出这些点对应中一共有多少组inliers,多少组outliers，并把源图像和目标图像中inlier的坐标分别返回；
 * args:
 *     img_src: 源图像路径；
 *     img_trgt: 目标图像路径；
 *     board_size: （源和目标）图像上每行和每列的角点个数
 *     mask: 一个(N x 1)的矩阵，表示每组点对应是inlier(值!=0)还是outlier(值==0)（输出参数）；
 *     src_points_inliers: 源图像中属于inlier的场景点（输出参数）；
 *     trgt_points_inliers: 目标图像中属于inlier的场景点（输出参数）；
 * return:
 *     从源图像坐标到目标图像坐标单应矩阵。
 * */
Mat computeHomography_2d(const string& img_src, const string& img_trgt, const Size& board_size, Mat& mask,
                         vector<Point2f>& src_points_inliers, vector<Point2f>& trgt_points_inliers)
{
    vector<string> img_List;
    img_List.push_back(img_src);
    img_List.push_back(img_trgt);

    //1.角点检测
    vector<vector<Point2f>> cornerpoints_seq;
    cout<< "*******开始角点检测。*********"<<endl;
    cornerPointsDetect(img_List, board_size, cornerpoints_seq, 0, 0, 0);
    vector<Point2f> src_points = cornerpoints_seq[0];
    vector<Point2f> trgt_points = cornerpoints_seq[1];

    cout<< "*******开始计算单应矩阵。*********"<<endl;
    int pts_num = src_points.size();
    cout<<"接下来用源图像和目标图像上的"+to_string(pts_num)+"组点对应计算单应矩阵:"<<endl;
    Mat homography = findHomography(src_points, trgt_points, RANSAC, 5,mask);

    //2.计算这些场景点中有多少个inlier，多少个outlier。
    // 并把 源图像中的inliers 和 目标图像中的inliers 坐标另存到输出参数中
    int inlier_num =0, outlier_num=0;
    for(int i=0; i<mask.rows; i++)
    {
        //注意，mask矩阵的type()是uchar，所以在用其元素进行判断时也要读取成ucahr类型
        if(mask.at<uchar>(i,0))  //inlier
        {
            inlier_num++;
            src_points_inliers.push_back(src_points[i]);
            trgt_points_inliers.push_back(trgt_points[i]);
//            cout<<"inlier: mask["+to_string(i)+"]"<<endl;  //输出inlier的序号
        }
        else  //outlier
        {
            outlier_num++;
//            cout<<"outlier: mask["+to_string(i)+"]"<<endl;  //输出outlier的序号
        }
    }
    cout<<"这"+to_string(pts_num)+"组点对应中，一共有"+to_string(inlier_num)+"组inliers, "+to_string(outlier_num)+"组outliers."<<endl;
    cout<< "单应矩阵计算完成。"<<endl;

    return homography;
}


/**
 * func:给定源图像和目标图像中的若干组点对应坐标，求得从 源图像坐标 转换到 目标图像坐标 的单应矩阵(2D->2D)。
 *      同时打印出这些点对应中一共有多少组inliers,多少组outliers，并把源图像和目标图像中inlier的坐标分别返回；
 * args:
 *     src_points: 源图像中场景点的坐标（2D点坐标）；
 *     trgt_points: 目标图像中场景点的坐标（2D点坐标）；
 *     board_size: （源和目标）图像上每行和每列的角点个数
 *     mask: 一个(N x 1)的矩阵，表示每组点对应是inlier(值!=0)还是outlier(值==0)（输出参数）；
 *     src_points_inliers: 源图像中属于inlier的场景点（输出参数）；
 *     trgt_points_inliers: 目标图像中属于inlier的场景点（输出参数）；
 * return:
 *     从源图像坐标到目标图像坐标单应矩阵(3x3)
 * */
Mat computeHomography2_2d(const vector<Point2f>& src_points, const vector<Point2f>& trgt_points,
        const Size& board_size, Mat& mask, vector<Point2f>& src_points_inliers, vector<Point2f>& trgt_points_inliers )
{
    cout<< "*******开始计算单应矩阵。*********"<<endl;
    int pts_num = src_points.size();
    cout<<"接下来用源图像和目标图像上的"+to_string(pts_num)+"组点对应计算单应矩阵:"<<endl;
    Mat homography = findHomography(src_points, trgt_points, RANSAC, 5,mask);

    //2.计算这些场景点中有多少个inlier，多少个outlier。
    // 并把 源图像中的inliers 和 目标图像中的inliers 坐标另存到输出参数中
    int inlier_num =0, outlier_num=0;
    for(int i=0; i<mask.rows; i++)
    {
        //注意，mask矩阵的type()是uchar，所以在用其元素进行判断时也要读取成ucahr类型
        if(mask.at<uchar>(i,0))  //inlier
        {
            inlier_num++;
            src_points_inliers.push_back(src_points[i]);
            trgt_points_inliers.push_back(trgt_points[i]);
//            cout<<"inlier: mask["+to_string(i)+"]"<<endl;  //输出inlier的序号
        }
        else  //outlier
        {
            outlier_num++;
//            cout<<"outlier: mask["+to_string(i)+"]"<<endl;  //输出outlier的序号
        }
    }
    cout<<"这"+to_string(pts_num)+"组点对应中，一共有"+to_string(inlier_num)+"组inliers, "+to_string(outlier_num)+"组outliers."<<endl;
    cout<< "单应矩阵计算完成。"<<endl;

    return homography;
}


/**
 * func: 给定两幅图像中的若干组点对应，以及从第一幅图像到第二幅图像的单应矩阵，计算出这些点对应的对称转移误差（用于评价单应矩阵求得怎么样）
 * args:
 *     src_points: 各个场景点在原图像中的坐标；
 *     trgt_points: 各个场景点在目标图像中的坐标；
 *     homography: 从 源图像坐标系 到 目标图像坐标系 的单应矩阵；
 *     src_points_proj: 用于存放源图像上的点投影（到目标图像上）之后的点的坐标（输出参数）；
 *     trgt_points_proj: 用于存放目标图像上的点投影（到源图像上）之后的点的坐标（输出参数）；
 * return:
 *     总的对称转移误差。
 * */
double symmetricTransfer_error(vector<Point2f>& src_points, vector<Point2f>& trgt_points, Mat& homography,
                               vector<Point2f>& src_points_proj, vector<Point2f>& trgt_points_proj)
{
    double ST_err = 0.0;  //记录对称转移误差
    Mat H = homography;  //单应矩阵H
    Mat H_inv;  //H的逆矩阵
    H_inv = H.inv();

    int pts_num = src_points.size();  //有多少组点对应（全部参与计算）

    //1.把每个点的坐标转换为齐次坐标形式，并存储到矩阵的某一列中。这样可以用矩阵乘法来表示齐次坐标的透视投影
    Mat X = Mat(3, pts_num, CV_64FC1, Scalar::all(1));  //用于存放源图像上的各个点（齐次坐标，矩阵形式）
    Mat Y = Mat(3, pts_num, CV_64FC1, Scalar::all(1)); //用于存放目标图像上的各个点（齐次坐标，矩阵形式）
    for(int i=0; i<pts_num; i++)
    {
        X.at<double>(0,i) = src_points[i].x;
        X.at<double>(1,i) = src_points[i].y;
        Y.at<double>(0,i) = trgt_points[i].x;
        Y.at<double>(1,i) = trgt_points[i].y;
    }
    //至此，X中每一列储存的是源图像上的点投影后的点坐标（齐次坐标），X_prm中每一列储存的是目标图像上的点投影后的点坐标（其齐次坐标）

    //2. 用给定的单应矩阵进行投影变换，把坐标转换后的点记录在对应的vector<Point2f>中
    Mat X_proj = H*X;
    Mat Y_proj = H_inv*Y;
    //把投影后的点坐标（以非齐次形式）储存到vector<Point2f>中
    for(int i=0; i<pts_num; i++)
    {
        src_points_proj.emplace_back( Point2f(X_proj.at<double>(0,i)/X_proj.at<double>(2,i),X_proj.at<double>(1,i)/X_proj.at<double>(2,i) ) );
        trgt_points_proj.emplace_back( Point2f(Y_proj.at<double>(0,i)/Y_proj.at<double>(2,i),Y_proj.at<double>(1,i)/Y_proj.at<double>(2,i) ) );
    }

    //3. 计算对称转移误差
    cout<< "*******开始计算对称转移误差 *********"<<endl;
    double ST1 = norm(trgt_points, src_points_proj, NORM_L2);
    double ST2 = norm(src_points, trgt_points_proj, NORM_L2);
    cout<<"源图片上的点转移到目标图片坐标系上，总的误差为:"+to_string(ST1)<<endl;
    cout<<"目标图片上的点转移到源图片坐标系上，总的误差为:"+to_string(ST2)<<endl;
    ST_err += ST1;
    ST_err += ST2;
    cout<<to_string(pts_num)+"组inliers点对应（总的）对称转移误差为:"+to_string(ST_err)<<endl;
    cout<<"（平均每组点对应）的对称转移误差为:"+to_string(ST_err/pts_num)<<endl;
    cout<< "对称转移误差计算完成。"<<endl;

    return ST_err;
}


/**
 * func: 给定一张图片和一组点坐标，在该图片上用圆圈绘制这些点，并显示(按esc键退出)
 * args:
 *     imgName: 图片名；
 *     points: 要绘制的点坐标（一组点）；
 *     color: 画的圈圈的颜色；
 *     windowName: 显示图片时窗口的名字；
 * return:
 *     none.
 */
void drawCircleOnImage(const string& imgName, const vector<Point2f>& points, const Scalar& color, const string& windowName,
                      int radius=10, int thickness=5)
{
    Mat img = imread(imgName);
    int pts_num = points.size();
    for(int i=0; i<pts_num; i++)
        circle(img, points[i], radius, color, thickness);
    imshow(windowName, img);
    while ((char) waitKey(0) != 27);
}


/*
 * func: 给定两个点(2个坐标)，求出它们之间的欧式距离
 * args:
 *      p1, p2: 待求距离的两点；
 * return:
 *      p1、p2之间的欧式距离。
 * */
double euclid_distanc(const Point2f& p1, const Point2f& p2)
{
    return pow( ( pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) ), 0.5);
}


/*
 * func: 给定一堆2维坐标的输入点，对它们进行标准化
 * args:
 *      src_pts: 标准化之前的点（输入参数）；
 *      trgt_pts: 标准化之后的点（输出参数）；
 *      T: 转化矩阵；
 * return:
 *      None。
 * */
void normalize_for_DLT(const vector<Point2f>& src_pts, vector<Point2f>& trgt_pts, Mat& T)
{
    int pts_num = src_pts.size();  //场景点的数量

    //1. 求中心点的x和y坐标
    double sum_x = 0.0;
    double sum_y = 0.0;
    for(int i=0; i<pts_num; i++)
    {
        sum_x += src_pts[i].x;
        sum_y += src_pts[i].y;
    }
    auto cx = double( sum_x/pts_num );
    auto cy = double( sum_y/pts_num );
    Point2f centroid = Point2f (cx, cy);

    //2.求得每个点到中心点的平均距离
    double d_sum = 0.0;
    for(int i=0; i<pts_num; i++)
    {
        d_sum += euclid_distanc(centroid, src_pts[i]);
    }
    double d_mean = d_sum/pts_num;  //平均欧式距离
    double s = pow(2,0.5)/d_mean;

    //3.构建Transform矩阵T
    T = (Mat_<double>(3,3) << s, 0.0, (-1)*s*cx, 0.0 ,s, (-1)*s*cy, 0.0, 0.0, 1.0);
//    cout<<"In function normalize_for_DLT(): T="<<endl<<T<<endl;  //测试输出用

    //4.用Transform对source点进行转换，得到转换之后的点坐标
    for(int i =0; i<pts_num; i++)
    {
        Point2f srcPoint = src_pts[i];
        Mat x_head = T*( Mat_<double>(3,1)<< srcPoint.x, srcPoint.y, 1.0);  //把source点变为齐次坐标，再进行转换，shape:3X1
        Point2f pt_x_head = Point2f( x_head.at<double>(0,0)/x_head.at<double>(2,0), x_head.at<double>(1,0)/x_head.at<double>(2,0) );
        trgt_pts.push_back(pt_x_head);
    }

}


/**
 * func: 运用Drarenei标定方法（一种线阵相机动态扫描成像标定方法）获得从 升维世界坐标系(normalized坐标) 到图像坐标系(normalizaed坐标) 的转化矩阵(3X6)
 * args:
 *      worldPoints: 场景点在世界平面上的坐标（只有x和y坐标，因为假设传送带所在的平面为z=0）；
 *      imagePoints: 场景点在图像坐标系中的坐标；
 *      (注：该算法要求至少要传入6组点对应！)
 *      T_img: 图像平面坐标系中的点的normalize矩阵：xi' = T1·xi（输出参数）；
 *      T_wrld: 世界平面坐标系中的点的normalize矩阵：ai' = T2·ai（输出参数）；
 * return:
 *      H: 从 升维世界坐标系(normalized坐标) 到 图像坐标系(normalizaed坐标) 的转化矩阵（3X6的）。
// */
Mat Drareni_computeHomography(const vector<Point2f>& worldPoints, const vector<Point2f>& imagePoints, Mat& T_img, Mat& T_wrld )
{
    Mat H;
    if(worldPoints.size() != imagePoints.size())
    {
        cout<<"传入的点对应数不一致！"<<endl;
        return H;
    }
    if(worldPoints.size() < 6)
    {
        cout<<"请传入至少6组点对应！"<<endl;
        return H;
    }
    int pointNum = worldPoints.size();  //传入的点对应数量
    cout<<"用Drareni方法计算单应矩阵H,点对应的数量为:"+to_string(pointNum)<<endl;

    //step 1: Normalization
    vector<Point2f> worldPoints_norm, imagePoints_norm;
    normalize_for_DLT(worldPoints, worldPoints_norm, T_wrld);  //对 世界平面坐标系 中的点做normalization
    normalize_for_DLT(imagePoints, imagePoints_norm, T_img);  //对 图像平面坐标系 中的点做normalization

    //step 2: 用DLT方法，求解向量h
    //2.1: 构建线性方程组（每一组点对应可提供2个线性无关的方程）
    Mat A;  //Ah=0（超定齐次线性方程组）中的A
    for(int i=0; i<pointNum; i++)
    {
        Point2f p_wrld = worldPoints_norm[i];
        double a = p_wrld.x;
        double b = p_wrld.y;

        Point2f p_img = imagePoints_norm[i];
        double u = p_img.x;
        double v = p_img.y;

        //每一组点对应提供2个线性无关的方程
        Mat row1=(Mat_<double>(1,12)<<0.0, 0.0, 0.0, a, b, 1.0, a*a, b*b, a*b, (-1.0)*a*v, (-1.0)*b*v, (-1.0)*v );
        Mat row2=(Mat_<double>(1,12)<<a, b, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (-1.0)*a*u, (-1.0)*b*u, (-1.0)*u );
        A.push_back(row1);
        A.push_back(row2);
    }

    //2.2: （求解h向量，这里求的是最小二乘解, s.t. ||h||=1 ）对矩阵A进行奇异值分解
    //法1：奇异值分解（h的 模为1的最小二乘解 就是 最小奇异值对应的右奇异向量）
    Mat U, D, V_T;
    SVD::compute(A, D, U, V_T);  //待求的h的最优解就是V_T的最后一行
    auto* h_ptr = V_T.ptr<double>(V_T.rows-1); //指向V_T最后一行的行指针(shape: 1X12)
    H = (Mat_<double>(3,6)<<h_ptr[0], h_ptr[1], h_ptr[2], 0.0, 0.0, 0.0,
                                        h_ptr[3], h_ptr[4], h_ptr[5], h_ptr[6], h_ptr[7], h_ptr[8],
                                        h_ptr[9], h_ptr[10],h_ptr[11], 0.0, 0.0, 0.0);  //把h从向量形式转换成矩阵形式

    //test 1. 正向投影，看看求出的H怎么样（世界坐标系中的点 -> 图像坐标系）
    string test_imgPts_norm = "./data/tst1_image_points_norm.txt";  //该文件内容：见./data/readme.txt, 5
    string test_trsfdwrldPts_norm = "./data/tst1_worldTransToImage_points_norm.txt";  //该文件内容：见./data/readme.txt, 6
    ofstream out1;
    ofstream out2;
    out1.open(test_imgPts_norm, ios::trunc|ios::out);
    out2.open(test_trsfdwrldPts_norm, ios::trunc|ios::out);

    for(int i=0; i<pointNum; i++)
    {
        out1<<imagePoints_norm[i].x<<", "<<imagePoints_norm[i].y<<endl;

        double a_tst = worldPoints_norm[i].x;
        double b_tst = worldPoints_norm[i].y;
        Mat X = ( Mat_<double>(6,1)<<a_tst, b_tst, 1.0, a_tst*a_tst, b_tst*b_tst, a_tst*b_tst );
        Mat X_prm = H*X;
        out2<< X_prm.at<double>(0,0)/X_prm.at<double>(2,0)<<", "<<X_prm.at<double>(1,0)/X_prm.at<double>(2,0)<<endl;
    }
    out1.close();
    out2.close();
    //end test1（没问题）

    //test 2.反向投影 (图像坐标系中的点 -> 世界坐标系中)
    string test_wrldPts_norm = "./data/tst2_world_points_norm.txt";  //该文件内容：见./data/readme.txt, 7
    string test_trsfdImgPts_norm = "./data/tst2_imageTransToWorld_points_norm.txt";  //该文件内容：见./data/readme.txt, 8
    ofstream out3;
    ofstream out4;
    out3.open(test_wrldPts_norm, ios::trunc|ios::out);
    out4.open(test_trsfdImgPts_norm, ios::trunc|ios::out);

    for(int i=0; i<pointNum; i++)
    {
        out3<< worldPoints_norm[i].x<<", "<<worldPoints_norm[i].y<<endl;

        double u_tst = imagePoints_norm[i].x;
        double v_tst = imagePoints_norm[i].y;  //注：这里的u_tst和v_tst都是normalized形式
        double h11=H.at<double>(0,0), h12=H.at<double>(0,1), h13=H.at<double>(0,2),
                h21=H.at<double>(1,0), h22=H.at<double>(1,1), h23=H.at<double>(1,2),
                h24=H.at<double>(1,3), h25=H.at<double>(1,4), h26=H.at<double>(1,5),
                h31=H.at<double>(2,0), h32=H.at<double>(2,1), h33=H.at<double>(2,2);

        double m1 = h21 - v_tst*h31, m2 = h22 - v_tst*h32, m3 = h24, m4 = h25, m5 = h26, m6 = h23 - v_tst*h33,
                m7 = h11 - u_tst*h31, m8 = h12 - u_tst*h32, m9 = h13 - u_tst*h33;

        double k1 = m3*m8*m8 + m4*m7*m7 - m5*m7*m8,
                k2 = m1*m8*m8 - m2*m7*m8 + 2*m4*m7*m9 - m5*m8*m9,
                k3 = m4*m9*m9 + m6*m8*m8 - m2*m8*m9;

        double a_norm = ( (-1)*k2 - pow(k2*k2-4*k1*k3,0.5) )/(2*k1);
        double b_norm = (-1)*(m9+m7*a_norm)/m8;
        out4<< a_norm<<", "<<b_norm<<endl;
    }
    out3.close();
    out4.close();
    //end test2

    //step 3: Denormalization（不在该函数中完成）

    return H;
}


/**
 * func: 给出 某点在图像平面上的坐标 和用 Drarenei标定方法 求出的H矩阵(3X6)，返回该点在世界平面(世界坐标系中z=0的平面)上的坐标
 * args:
 *      u: 待求点在图像平面上的x坐标；
 *      v: 待求点在图像平面上的y坐标；
 *      H: 从世界平面到图像平面的转化矩阵（3X6）（输入参数）；
 *      T_img: 图像平面坐标系中的点的normalize矩阵：xi' = T1·xi（输入参数）;
 *      T_wrld: 世界平面坐标系中的点的normalize矩阵：ai' = T2·ai（输入参数）；
 * return:
 *     该点在世界平面(世界坐标系中z=0的平面)上的坐标。
 * */
Point2f Drareni_computWorldCoord(double u, double v, const Mat& H, const Mat& T_img, const Mat& T_wrld)
{
    //1. 求解( a_norm, b_norm )
    Mat src_pt_norm = T_img*( Mat_<double>(3,1)<< u, v, 1.0);  //对图像坐标进行normalization

    //1.1 用T1矩阵对(u,v)做normalizetion
    double u_norm = src_pt_norm.at<double>(0,0)/src_pt_norm.at<double>(2,0) ;
    double v_norm = src_pt_norm.at<double>(1,0)/src_pt_norm.at<double>(2,0) ;
    double h11=H.at<double>(0,0), h12=H.at<double>(0,1), h13=H.at<double>(0,2),
          h21=H.at<double>(1,0), h22=H.at<double>(1,1), h23=H.at<double>(1,2),
          h24=H.at<double>(1,3), h25=H.at<double>(1,4), h26=H.at<double>(1,5),
          h31=H.at<double>(2,0), h32=H.at<double>(2,1), h33=H.at<double>(2,2);


    //1.2 （法一：代入公式求解）构建一元二次方程组并解
    double m1 = h21 - v_norm*h31, m2 = h22 - v_norm*h32, m3 = h24, m4 = h25, m5 = h26, m6 = h23 - v_norm*h33,
          m7 = h11 - u_norm*h31, m8 = h12 - u_norm*h32, m9 = h13 - u_norm*h33;
    double k1 = m3*m8*m8 + m4*m7*m7 - m5*m7*m8,
          k2 = m1*m8*m8 - m2*m7*m8 + 2*m4*m7*m9 - m5*m8*m9,
          k3 = m4*m9*m9 + m6*m8*m8 - m2*m8*m9;
    // 接下来解：k1*a^2 + k2*a + k3 = 0
    double a_norm = ( (-1)*k2 - pow(k2*k2-4*k1*k3,0.5) )/(2*k1);
    double b_norm = (-1)*(m9+m7*a_norm)/m8;

//     1.2 （法二：求解缺定齐次线性方程。。。我觉得这么做有问题。。。于是就给它注释掉了。。。）
//     //即M·H·x=0，其中，M、H已知，求x。其中x的形式是x=(a, b, 1, a^2, b^2, a*b).T
//     Mat M = ( (Mat_<double>(3,3) << 0.0, -1.0, v, 1.0, 0.0, (-1)*u, (-1)*v, u, 0.0 ) );
//     Mat MH = M*H;  //shape: 3X6
//     Mat x;
//     SVD::solveZ(MH, x);  //求出under-determined（缺定？）方程组的最优解(shape: 6X1)
//
//    //对x3进行归一化（放缩x令x3=1）
//     cout<<"x (对x3归一化之前):"<<endl<<x<<endl;
//     cout<<"MH·x(对x3归一化之前)"<<endl<<MH*x<<endl;
//     x = (1.0/x.at<double>(2,0))*x;
//     cout<<"x (对x3归一化之后):"<<endl<<x<<endl;
//     cout<<"MH·x(对x3归一化之后)"<<endl<<MH*x<<endl;
//
//     double a = ( x.at<double>(0,0) + ( x.at<double>(3,0)/x.at<double>(0,0) )
//            + ( pow(x.at<double>(3,0),0.5) ) + ( x.at<double>(5,0)/x.at<double>(1,0) ) )/4;
//
//     double b = ( x.at<double>(1,0) + ( x.at<double>(4,0)/x.at<double>(1,0) )
//                + ( pow(x.at<double>(4,0),0.5) ) + ( x.at<double>(5,0)/x.at<double>(0,0) ) )/4;

    //2. 用T2对求得的 (a_norm, b_norm) 做Denormalization
    Mat trgt_pt_norm = ( Mat_<double>(3,1)<<a_norm, b_norm, 1.0 );
    Mat trgt_pt = T_wrld.inv()*trgt_pt_norm;
    double a = ( trgt_pt.at<double>(0,0)/trgt_pt.at<double>(0,2) );
    double b = ( trgt_pt.at<double>(1,0)/trgt_pt.at<double>(0,2) );

    Point2f p = Point2f(a, b);
    return p;
}


/*
 * func: 这个函数用于test3.4，即在 标准棋盘格图像 上绘制圆圈来模拟在 世界平面 上绘制圆圈。世界平面坐标系是以第一个角点为原点建立的，
 *       而标准棋盘格图像平面则是以左上角为原点建立坐标系的。该函数将 世界平面坐标系的坐标 转化为 标准棋盘格图像平面坐标系 的坐标。
 *       转换之后即可在 标准棋盘格图像平面 上绘制计算出测试点的世界坐标，以及测试点的标准世界坐标 进行对比；
 * args:
 *       wrldCoord: 待转换的 世界平面坐标系 的坐标；
 *       actual_grid_size: 一个棋盘格在世界平面上的真实尺寸：宽x高（单位：mm）；
 *       img_grid_size: 一个棋盘格在标准棋盘格图像平面上的尺寸：宽x高（单位：pixel）;
 * return:
 *       转换后的在标准棋盘格图像平面上的坐标。
 * */
Point2f worldCoord_to_stdImageCoord(const Point2f& wrldCoord, const Size& actual_grid_size, const Size& img_grid_size)
{
    double x = (wrldCoord.x/actual_grid_size.width)*img_grid_size.width + img_grid_size.width;
    double y = (wrldCoord.y/actual_grid_size.height)*img_grid_size.height + img_grid_size.height;
    Point2f p = Point2f (x, y);
    return p;
}


/*
 * func: (改变手性，即xOy->yOx)给定若干个点的坐标(2D空间)，将它们映射到新坐标系（新坐标系就是原坐标系x轴和y轴对调一下）
 * args:
 *     srcPts: 这些点在手性转换之前的原坐标系下的坐标；
 * return:
 *     这些点在手性转换之后的新坐标系下的坐标；
 * */
vector<Point2f> chirality_transformation(const vector<Point2f>& srcPts)
{
    int ptsNum = srcPts.size();
    vector<Point2f> trgtPts;
    trgtPts.reserve(ptsNum);

    for(int i=0; i<ptsNum; i++)
    {
        Point2f pt = Point2f(srcPts[i].y, srcPts[i].x);
        trgtPts.emplace_back(pt);
    }

    return trgtPts;
}


/*
 * func:( 图像平面坐标系->机器人（平面）坐标系 )给出一组点在 图像平面坐标系 的坐标，和4个需要用到的转换矩阵，
 *      得到这一组点在 机器人(平面)坐标系 的坐标；
 * args:
 *      imgCoords：待转换的这一组点在 图像平面坐标系 的坐标；
 *      H：从 升维世界坐标系(normalized坐标) 到 图像坐标系(normalizaed坐标) 的转化矩阵（3X6）；
 *      T_img：图像平面坐标系中的点的normalize矩阵：xi' = T1·xi（3X3）;
 *      T_wrld：世界平面坐标系中的点的normalize矩阵：ai' = T2·ai（3X3）;
 *      H_v：从 世界(平面)坐标系 到 机器人(平面)坐标系 的转化矩阵（3X3）;
 *      worldCoords：用来存储这一组点在 世界平面坐标系 的坐标（输出参数）;
 * return:
 *      这一组点转换到 机器人（平面）坐标系 中的坐标。只有x和y坐标，而且其x和y坐标和4轴机器人的X、Y方向保持一致。
 * */
vector<Point2f> imageToRobot(const vector<Point2f>& imgCoords, const Mat& H, const Mat& T_img, const Mat& T_wrld, const Mat& H_v,
                            vector<Point2f>& worldCoords)
{
    int ptsNum = imgCoords.size();  //待转换点的个数
//    vector<Point2f> worldCoords;  //用来储存 世界平面坐标系 的坐标
    worldCoords.reserve(ptsNum);

    //step1.先得将这些点在 图像平面坐标系 中的坐标转换成在 世界平面坐标系 中的坐标。调用函数Drareni_computWorldCoord()
    cout<<endl<<"待抓的几个物体在 世界平面坐标系 中的坐标："<<endl;
    for(int i=0; i<ptsNum; i++)
    {
        Point2f pt_img = imgCoords[i];
        Point2f pt_wrld = Drareni_computWorldCoord(pt_img.x, pt_img.y, H, T_img, T_wrld);  //先转换到世界平面坐标系中
        cout<<"object "+to_string(i+1)+"(计算出的)在 世界平面坐标系 中的坐标："<<pt_wrld<<endl;
        worldCoords.emplace_back(pt_wrld);
    }
    //worldCoords填充完毕

    //step2.将 世界坐标系 转换到 过渡世界坐标系，调用函数chirality_transformation()
    vector<Point2f> interimWorldCoords;  ////用来储存 过渡世界平面坐标系 的坐标
    interimWorldCoords = chirality_transformation(worldCoords);
    cout<<"待抓的几个物体在 过渡世界平面坐标系 中的坐标："<<endl;
    for(int i=0; i<ptsNum; i++)
        cout<<"object "+to_string(i+1)+"(计算出的)在 过渡世界平面坐标系 中的坐标："<<interimWorldCoords[i]<<endl;

    //step3.将这些点在 过渡世界平面坐标系 中的坐标转换成在 机器人(平面)坐标系 中的坐标。调用函数transform_homography()
    vector<Point2f> robotCoords = transform_homography(interimWorldCoords, H_v);

    //暂时改为返回 过渡世界坐标系 的坐标
    return robotCoords;

}
