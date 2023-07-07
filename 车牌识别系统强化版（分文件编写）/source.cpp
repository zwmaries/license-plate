#include"source.h"
Mat Image_Preprocessing(Mat temp)//图像预处理
{
	//创建一个结构元素kernel 结构元素是一个大小为25x25的矩形矩阵，锚点位于(-1, -1)处
	Mat kernel = getStructuringElement(MORPH_RECT, Size(25, 25), Point(-1, -1));
	//用于存储形态学开操作的结果
	Mat open_gray_blur_Image;
	//将形态学开操作应用于输入图像temp。形态学开操作使用结构元素kernel执行，结果存储在open_gray_blur_Image中。MORPH_OPEN参数指定了开操作
	morphologyEx(temp, open_gray_blur_Image, MORPH_OPEN, kernel);
	Mat rst;
	//将形态学开操作的结果open_gray_blur_Image从原始图像temp中减去，并将结果存储在rst中
	subtract(temp, open_gray_blur_Image, rst, Mat());
	imshow("rst", rst);
	//用于存储Canny边缘检测的结果
	Mat Canny_Image;
	//将Canny边缘检测算法应用于rst图像。阈值设置为400和200，最后一个参数3指定了Sobel算子的孔径大小
	Canny(rst, Canny_Image, 200, 400, 3);
	return Canny_Image;
}

Mat Morphological_Processing(Mat temp)//形态学处理
{
	//图片膨胀处理
	//用于存储膨胀和腐蚀操作的结果
	Mat dilate_image, erode_image;
	//创建了一个自定义的结构元素elementX。该结构元素是一个大小为25x1的矩形矩阵，用于在x方向上进行膨胀和腐蚀操作
	Mat elementX = getStructuringElement(MORPH_RECT, Size(25, 1));
	//用于在y方向上进行膨胀和腐蚀操作
	Mat elementY = getStructuringElement(MORPH_RECT, Size(1, 19));
	//该对象将在膨胀和腐蚀操作中用作锚点
	Point point(-1, -1);

	//将输入图像temp使用结构元素elementX进行膨胀操作，并将结果存储在dilate_image中。膨胀的迭代次数为2
	dilate(temp, dilate_image, elementX, point, 2);
	//将上一步膨胀得到的图像dilate_image使用结构元素elementX进行腐蚀操作，并将结果存储在erode_image中。腐蚀的迭代次数为4
	erode(dilate_image, erode_image, elementX, point, 4);
	//再次对上一步腐蚀得到的图像erode_image使用结构元素elementX进行膨胀操作，并将结果存储在dilate_image中。膨胀的迭代次数为2
	dilate(erode_image, dilate_image, elementX, point, 2);

	//通过在X方向上进行两次膨胀操作，可以进一步增强图像中水平方向上的连通性

	//自定义核：进行 Y 方向的膨胀腐蚀
	//对上一步膨胀得到的图像dilate_image使用结构元素elementY进行腐蚀操作，并将结果存储在erode_image中。腐蚀的迭代次数为1
	erode(dilate_image, erode_image, elementY, point, 1);
	//再次对上一步腐蚀得到的图像erode_image使用结构元素elementY进行膨胀操作，并将结果存储在dilate_image中。膨胀的迭代次数为2
	dilate(erode_image, dilate_image, elementY, point, 2);

	//反复对图像进行膨胀腐蚀可以去除图像的噪点，填充空洞
	
	//平滑处理 中值滤波
	Mat median_Image;
	//对上一步膨胀得到的图像dilate_image进行中值滤波操作，并将结果存储在median_Image中。滤波器的大小为15x15
	medianBlur(dilate_image, median_Image, 15);
	//再次对上一步中值滤波得到的图像median_Image进行中值滤波操作，并将结果仍然存储在median_Image中。滤波器的大小为15x15
	medianBlur(median_Image, median_Image, 15);

	//通过进行两次中值滤波操作，可以进一步减少噪声的影响，
	// 第一次中值滤波可以去除较大的噪声，而第二次中值滤波则可以进一步平滑图像
	//imshow("中值滤波", median_Image);
	return median_Image;
}

Mat Locate_License_Plate(Mat temp, Mat src, Mat gray_src)//车牌定位
{
	vector<vector<Point>> contours;
	findContours(temp, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//画出轮廓
	//findContours() 函数遍历图像中的每个像素，找到连续的边缘像素，
	//并将它们组成一个或多个轮廓。它根据像素之间的连接关系和空间位置，将相邻的边缘像素归为同一个轮廓

	drawContours(temp, contours, -1, Scalar(255), 1);
	//轮廓表示为一个矩形
	Mat Roi;
	for (int i = 0; i < contours.size(); i++)
	{
		RotatedRect rect = minAreaRect(Mat(contours[i]));
		Point2f p[4];
		rect.points(p);
		double axisLongTemp = 0.0, axisShortTemp = 0.0;//矩形的长边和短边
		axisLongTemp = sqrt(pow(p[1].x - p[0].x, 2) + pow(p[1].y - p[0].y, 2));  //计算长轴（勾股定理）
		axisShortTemp = sqrt(pow(p[2].x - p[1].x, 2) + pow(p[2].y - p[1].y, 2)); //计算短轴（勾股定理）
		double LengthTemp;     //中间变量
		if (axisShortTemp > axisLongTemp)   //短轴大于长轴，交换数据
		{
			LengthTemp = axisLongTemp;
			axisLongTemp = axisShortTemp;
			axisShortTemp = LengthTemp;
		}
		double rectArea = axisLongTemp * axisShortTemp;//计算矩形面积
		double Area = contourArea(Mat(contours[i]));//轮廓面积
		double rectDegree = Area / rectArea;//计算矩形度
		//这部分的条件判断可视实际情况做调整
		if (axisLongTemp / axisShortTemp >= 2.2 && axisLongTemp / axisShortTemp <= 5.1 && rectDegree > 0.63 && rectDegree < 1.37 && rectArea>2000 && rectArea < 50000)
		{
			for (int i = 0; i < 4; i++)       //划线框出车牌区域
				line(src, p[i], p[((i + 1) % 4) ? (i + 1) : 0], Scalar(0, 0, 255), 2, 8, 0);

			float width_height = (float)rect.size.width / (float)rect.size.height;
			//在最新的opencv-python 4.5.1.48版本中，这个角度的范围不再是负的了，变成了(0,90]。大小是θ
			//的余角，相当于y轴顺时针旋转与最小外接矩形第一个重合的边之间的角度
			float angle = rect.angle;
			if (width_height < 1)
			{
				angle = angle - 90;
			}
			Mat rotMat = getRotationMatrix2D(rect.center, angle, 1);//获得矩形的旋转矩阵
			Mat warpImg;
			warpAffine(gray_src, warpImg, rotMat, src.size(), INTER_CUBIC, BORDER_CONSTANT);
			imshow("仿射变换", warpImg);
			//图像切割
			Size minRectSize = rect.size;
			if (width_height < 1)
				swap(minRectSize.width, minRectSize.height);
			getRectSubPix(warpImg, minRectSize, rect.center, Roi);
		}
	}

	imshow("test", src);
	imshow("车牌提取结果", Roi);
	return Roi;
}

//仿射变换
Mat Affine_Transform(Mat temp)
{
	Mat warp_dstImage = Mat::zeros(100, 500, temp.type());
	Point2f srcTri[3];
	Point2f dstTri[3];
	//设置三个点来计算仿射变换
	srcTri[0] = Point2f(0, 0);
	srcTri[1] = Point2f(temp.cols, 0);
	srcTri[2] = Point2f(0, temp.rows);

	dstTri[0] = Point2f(0, 0);
	dstTri[1] = Point2f(500, 0);
	dstTri[2] = Point2f(0, 100);
	//计算仿射变换矩阵
	Mat warp_mat(2, 3, CV_32FC1);
	warp_mat = getAffineTransform(srcTri, dstTri);
	//对加载图形进行仿射变换操作
	warpAffine(temp, warp_dstImage, warp_mat, warp_dstImage.size());
	return warp_dstImage;
}

//移除车牌垂直边框
Mat Remove_Vertial_Border(Mat temp)
{
	//一个矩形矩阵，行数为1，列数与temp图像相同。它用于定义接下来的形态学操作的形状和大小
	Mat vline = getStructuringElement(MORPH_RECT, Size(1, temp.cols), Point(-1, -1));
	Mat dst1, temp1;
	//进行腐蚀操作,腐蚀操作可以帮助细化或移除某些结构
	erode(temp, temp1, vline);

	//进行膨胀操作,膨胀操作可以帮助加粗或扩展某些结构
	dilate(temp1, dst1, vline);
	/*namedWindow("提取垂直线", WINDOW_AUTOSIZE);*/
	/*imshow("提取垂直线", dst1);*/

	//在temp中移除dst1，即从原始图像中移除垂直线
	subtract(temp, dst1, temp, Mat());
	/*imshow("切割车牌垂直边框结果", temp);*/
	return temp;
}

Mat Remove_Horizon_Border(Mat temp)
{
	Mat hline = getStructuringElement(MORPH_RECT, Size(60, 1), Point(-1, -1));//矩形形状为：1*src.cols
	Mat dst1, temp1;
	erode(temp, temp1, hline);
	dilate(temp1, dst1, hline);
	/*namedWindow("提取水平线", WINDOW_AUTOSIZE);*/
	/*imshow("提取水平线", dst1);*/
	subtract(temp, dst1, temp, Mat());
	/*imshow("切割车牌水平边框结果", temp);*/
	return temp;
}

//水平裁剪车牌
Mat Horizon_Cut(Mat temp)
{
	int *counter_y = new int[temp.rows];
	for (int i = 0; i < temp.rows; i++)
		counter_y[i] = 0;
	for (int row = 0; row < temp.rows; row++)
	{
		int count = 0;
		for (int col = 0; col < temp.cols; col++)
		{
			if (temp.at<uchar>(row, col) == 255)
			{
				count++;
			}
		}
		if (count > 50)
		{
			counter_y[row] = 1;
		}
	}
	int count_temp = 0;
	int *record = new int[temp.rows];
	for (int i = 0; i < temp.rows; i++)
		record[i] = 0;
	for (int i = 0; i < temp.rows; i++)
	{
		if (counter_y[i] == 1)
		{
			count_temp++;
			record[i] = count_temp;
		}
		else
			count_temp = 0;
	}
	int max = record[0];
	int index = 0;
	for (int i = 1; i < temp.rows; i++)
	{
		if (max < record[i])
		{
			max = record[i];
			index = i;
		}
	}
	int index_row_begin = index - max + 1;
	int index_row_end = index;
	int height = index_row_end - index_row_begin;
	Mat image_preprocess = Mat::zeros(height, temp.cols, CV_8UC1);
	for (int row = 0; row < image_preprocess.rows; row++)
	{
		for (int col = 0; col < image_preprocess.cols; col++)
		{
			image_preprocess.at<uchar>(row, col) = temp.at<uchar>(row + index_row_begin, col);
		}
	}
	imshow("image_preprocess", image_preprocess);
	return image_preprocess;
}

//车牌字符定位
void Locate_String(int *x_begin, int *x_end, Mat temp)
{
	int *counter_x = new int[temp.cols];//记录每一列的白像素个数
	for (int i = 0; i < temp.cols; i++)
		counter_x[i] = 0;
	for (int col = 0; col < temp.cols; col++)
	{
		int count = 0;
		for (int row = 0; row < temp.rows; row++)
		{
			if (temp.at<uchar>(row, col) == 255)//如果该像素值表示白色
			{
				count++;
			}
		}
		counter_x[col] = count;
	}
	int index_col = 0;
	int number_width = 0;//记录字符宽度
	for (int i = 0; i < temp.cols - 1; i++)
	{
		if (counter_x[i] >= 3)//当每一列的白像素个数大于某一阈值，字符宽度开始累加
		{
			number_width++;
			if (number_width > 8)//10 当白像素个数大于某一阈值的连续列的个数大于10，开始更新字符位置信息
			{
				x_end[index_col] = i;
				x_begin[index_col] = i - number_width + 1;
				if (counter_x[i + 1] < 4)
				{
					number_width = 0;
					index_col++;
				}
			}
		}
		else
		{
			number_width = 0;
		}
		if (index_col >= 8)
			break;
	}
}

//字符分割
void Draw_Result(int *x_begin, int *x_end, Mat temp)
{
	int x, y;
	int width;
	int length;
	Mat Result = temp.clone();
	for (int i = 0; i < 8; i++)
	{
		x = x_begin[i];
		y = 0;
		width = x_end[i] - x_begin[i];
		length = temp.cols;
		Rect rect(x, y, width, length);
		Scalar color(255, 255, 255);
		rectangle(Result, rect, color, 2, LINE_AA);
	}
	imshow("车牌号码分割结果", Result);
}

void Recognize_Lisence(int *x_begin, int *x_end, Mat temp)
{
	int cycle_index = 0;
	for (int i = 0; i < 8; i++)
	{
		if (x_end[i] > 0)
			cycle_index++;
	}
	for (int i = 0; i < cycle_index; i++)
	{
		float error[28] = { 0 };
		//	//picture1是二值图像
		Mat picture1 = Mat::zeros(temp.rows, x_end[i] - x_begin[i], temp.type());
		for (int row = 0; row < picture1.rows; row++)
		{
			for (int col = 0; col < picture1.cols; col++)
			{
				picture1.at<uchar>(row, col) = temp.at<uchar>(row, col + x_begin[i]);
			}
		}
		Mat NUM[28];//字符匹配模板
		for (int i = 0; i < 28; i++)
		{
			stringstream stream;
			stream << "pictures/num_";
			stream << i;
			stream << ".bmp";
			String name = stream.str();
			NUM[i] = imread(name);
			if (NUM[i].empty())
			{
				cout << "未能读取" << name << endl;
			}
			cvtColor(NUM[i], NUM[i], COLOR_BGR2GRAY);
			threshold(NUM[i], NUM[i], 0, 255, THRESH_BINARY);

			Point2f srcTri[3];
			Point2f dstTri[3];
			Mat warp_mat(2, 3, CV_32FC1);
			//创建仿射变换目标图像与原图像尺寸类型相同
			Mat result = Mat::zeros(picture1.rows, picture1.cols, picture1.type());
			//设置三个点来计算仿射变换
			srcTri[0] = Point2f(0, 0);
			srcTri[1] = Point2f(NUM[i].cols, 0);
			srcTri[2] = Point2f(0, NUM[i].rows);
			dstTri[0] = Point2f(0, 0);
			dstTri[1] = Point2f(picture1.cols, 0);
			dstTri[2] = Point2f(0, picture1.rows);
			//计算仿射变换矩阵
			warp_mat = getAffineTransform(srcTri, dstTri);
			//对加载图形进行仿射变换操作
			warpAffine(NUM[i], result, warp_mat, picture1.size());
			threshold(result, result, 0, 255, THRESH_BINARY_INV);
			float error_sum = 0;
			float error_temp = 0;
			for (int row = 0; row < result.rows; row++)
			{
				for (int col = 0; col < result.cols; col++)
				{
					error_temp = picture1.at<uchar>(row, col) - result.at<uchar>(row, col);
					error_sum = error_sum + pow(error_temp, 2);
				}
			}
			error[i] = error_sum / (picture1.rows*picture1.cols * 255);
		}
		float min_error = error[0];
		int Index = 0;
		for (int i = 1; i < 28; i++)
		{
			if (min_error > error[i])
			{
				min_error = error[i];
				Index = i;
			}
		}
		if (Index == 10)
			cout << "E" << '\t';
		else if (Index == 11)
			cout << "V" << '\t';
		else if (Index == 12)
			cout << "苏" << '\t';
		else if (Index == 13)
			cout << "沪" << '\t';
		else if (Index == 14)
			cout << "B" << '\t';
		else if (Index == 15)
			cout << "S" << '\t';
		else if (Index == 16)
			cout << "京" << '\t';
		else if (Index == 17)
			cout << "N" << '\t';
		else if (Index == 18)
			cout << "J" << '\t';
		else if (Index == 19)
			cout << "P" << '\t';
		else if (Index == 20)
			cout << "A" << '\t';
		else if (Index == 21)
			cout << "浙" << '\t';
		else if (Index == 22)
			cout << "G" << '\t';
		else if (Index == 23)
			cout << "U" << '\t';
		else if (Index == 24)
			cout << "豫" << '\t';
		else if (Index == 25)
			cout << "K" << '\t';
		else if (Index == 26)
			cout << "陕" << '\t';
		else if (Index == 27)
			cout << "·" << '\t';

		else if (Index >= 0 && Index <= 9)
			cout << Index << '\t';
	}
	cout << endl;
}