#include"source.h"
int main()
{
	//����ͼ��
	Mat src, gray_src;
	src = imread("3.jpg");
	cvtColor(src, gray_src, COLOR_BGR2GRAY);
	Mat gray_blur_Image;
	GaussianBlur(gray_src, gray_blur_Image, Size(3, 3), 0, 0);
	Mat Canny_Image = Image_Preprocessing(gray_blur_Image);
	//��̬ѧ����
	Mat median_Image = Morphological_Processing(Canny_Image);
	//��������������ɸѡ��
	Mat contour_Image = median_Image.clone();
	Mat Roi = Locate_License_Plate(contour_Image, src, gray_src);
	//����任
	Mat warp_dstImage = Affine_Transform(Roi);
	Mat bin_warp_dstImage;
	threshold(warp_dstImage, bin_warp_dstImage, 0, 255, THRESH_BINARY | THRESH_OTSU);
	//threshold(warp_dstImage, bin_warp_dstImage, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);//�����еĻ�ɫ���Ƹ������д���
	imshow("ͬһ�ߴ�Ķ�ֵͼ��", bin_warp_dstImage);

	//����ʶ��
	//�г����Ƶ�ˮƽ�봹ֱ�߿�
	bin_warp_dstImage = Remove_Vertial_Border(bin_warp_dstImage);
	bin_warp_dstImage = Remove_Horizon_Border(bin_warp_dstImage);
	//��ȥ���ƺ�����������ಿ��
	Mat license = Horizon_Cut(bin_warp_dstImage);
	//��������x����ͶӰ
	int *x_begin = new int[8];
	int *x_end = new int[8];
	for (int i = 0; i < 8; i++)
	{
		x_begin[i] = 0;
		x_end[i] = 0;
	}
	Locate_String(x_begin, x_end, license);
	Draw_Result(x_begin, x_end, license);
	cout << "���ƺ�ʶ������" << endl;
	Recognize_Lisence(x_begin, x_end, license);
	delete[] x_begin;
	delete[] x_end;
	waitKey(0);
	return 0;
}