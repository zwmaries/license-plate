#include <iostream>
#include <opencv2/opencv.hpp>
#include<math.h>
using namespace std;
using namespace cv;
Mat Image_Preprocessing(Mat temp);//ͼ��Ԥ����
Mat Morphological_Processing(Mat temp);//��̬ѧ����
Mat Locate_License_Plate(Mat temp, Mat src, Mat gray_src);//���ƶ�λ
Mat Affine_Transform(Mat temp);//����任�����ڽ���ȡ��������ת��Ϊͬһ�ߴ磩
Mat Remove_Vertial_Border(Mat temp);//�Ƴ����ƴ�ֱ�߿�
Mat Remove_Horizon_Border(Mat temp);//�Ƴ�����ˮƽ�߿�
Mat Horizon_Cut(Mat temp);//ˮƽ�ü����ƣ���ȥ���ƺ�����������ಿ�֣�
void Locate_String(int *x_begin, int *x_end, Mat temp);//�����ַ���λ
void Draw_Result(int *x_begin, int *x_end, Mat temp);//���Ƴ����ַ���ѡ���
void Recognize_Lisence(int *x_begin, int *x_end, Mat temp);//���ƺ���ʶ��