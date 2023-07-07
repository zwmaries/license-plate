#include"source.h"
int main()
{
	//加载图像
	Mat src, gray_src;
	src = imread("3.jpg");
	cvtColor(src, gray_src, COLOR_BGR2GRAY);
	Mat gray_blur_Image;
	GaussianBlur(gray_src, gray_blur_Image, Size(3, 3), 0, 0);
	Mat Canny_Image = Image_Preprocessing(gray_blur_Image);
	//形态学处理
	Mat median_Image = Morphological_Processing(Canny_Image);
	//矩形轮廓查找与筛选：
	Mat contour_Image = median_Image.clone();
	Mat Roi = Locate_License_Plate(contour_Image, src, gray_src);
	//仿射变换
	Mat warp_dstImage = Affine_Transform(Roi);
	Mat bin_warp_dstImage;
	threshold(warp_dstImage, bin_warp_dstImage, 0, 255, THRESH_BINARY | THRESH_OTSU);
	//threshold(warp_dstImage, bin_warp_dstImage, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);//本例中的黄色车牌改用这行代码
	imshow("同一尺寸的二值图像", bin_warp_dstImage);

	//车牌识别
	//切除车牌的水平与垂直边框
	bin_warp_dstImage = Remove_Vertial_Border(bin_warp_dstImage);
	bin_warp_dstImage = Remove_Horizon_Border(bin_warp_dstImage);
	//除去车牌号码以外的冗余部分
	Mat license = Horizon_Cut(bin_warp_dstImage);
	//将车牌向x轴做投影
	int *x_begin = new int[8];
	int *x_end = new int[8];
	for (int i = 0; i < 8; i++)
	{
		x_begin[i] = 0;
		x_end[i] = 0;
	}
	Locate_String(x_begin, x_end, license);
	Draw_Result(x_begin, x_end, license);
	cout << "车牌号识别结果：" << endl;
	Recognize_Lisence(x_begin, x_end, license);
	delete[] x_begin;
	delete[] x_end;
	waitKey(0);
	return 0;
}