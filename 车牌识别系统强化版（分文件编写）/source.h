#include <iostream>
#include <opencv2/opencv.hpp>
#include<math.h>
using namespace std;
using namespace cv;
Mat Image_Preprocessing(Mat temp);//图像预处理
Mat Morphological_Processing(Mat temp);//形态学处理
Mat Locate_License_Plate(Mat temp, Mat src, Mat gray_src);//车牌定位
Mat Affine_Transform(Mat temp);//仿射变换（用于将提取出来车牌转化为同一尺寸）
Mat Remove_Vertial_Border(Mat temp);//移除车牌垂直边框
Mat Remove_Horizon_Border(Mat temp);//移除车牌水平边框
Mat Horizon_Cut(Mat temp);//水平裁剪车牌（除去车牌号码以外的冗余部分）
void Locate_String(int *x_begin, int *x_end, Mat temp);//车牌字符定位
void Draw_Result(int *x_begin, int *x_end, Mat temp);//绘制车牌字符框选结果
void Recognize_Lisence(int *x_begin, int *x_end, Mat temp);//车牌号码识别