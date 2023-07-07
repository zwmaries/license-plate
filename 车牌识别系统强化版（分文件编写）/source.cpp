#include"source.h"
Mat Image_Preprocessing(Mat temp)//ͼ��Ԥ����
{
	//����һ���ṹԪ��kernel �ṹԪ����һ����СΪ25x25�ľ��ξ���ê��λ��(-1, -1)��
	Mat kernel = getStructuringElement(MORPH_RECT, Size(25, 25), Point(-1, -1));
	//���ڴ洢��̬ѧ�������Ľ��
	Mat open_gray_blur_Image;
	//����̬ѧ������Ӧ��������ͼ��temp����̬ѧ������ʹ�ýṹԪ��kernelִ�У�����洢��open_gray_blur_Image�С�MORPH_OPEN����ָ���˿�����
	morphologyEx(temp, open_gray_blur_Image, MORPH_OPEN, kernel);
	Mat rst;
	//����̬ѧ�������Ľ��open_gray_blur_Image��ԭʼͼ��temp�м�ȥ����������洢��rst��
	subtract(temp, open_gray_blur_Image, rst, Mat());
	imshow("rst", rst);
	//���ڴ洢Canny��Ե���Ľ��
	Mat Canny_Image;
	//��Canny��Ե����㷨Ӧ����rstͼ����ֵ����Ϊ400��200�����һ������3ָ����Sobel���ӵĿ׾���С
	Canny(rst, Canny_Image, 200, 400, 3);
	return Canny_Image;
}

Mat Morphological_Processing(Mat temp)//��̬ѧ����
{
	//ͼƬ���ʹ���
	//���ڴ洢���ͺ͸�ʴ�����Ľ��
	Mat dilate_image, erode_image;
	//������һ���Զ���ĽṹԪ��elementX���ýṹԪ����һ����СΪ25x1�ľ��ξ���������x�����Ͻ������ͺ͸�ʴ����
	Mat elementX = getStructuringElement(MORPH_RECT, Size(25, 1));
	//������y�����Ͻ������ͺ͸�ʴ����
	Mat elementY = getStructuringElement(MORPH_RECT, Size(1, 19));
	//�ö��������ͺ͸�ʴ����������ê��
	Point point(-1, -1);

	//������ͼ��tempʹ�ýṹԪ��elementX�������Ͳ�������������洢��dilate_image�С����͵ĵ�������Ϊ2
	dilate(temp, dilate_image, elementX, point, 2);
	//����һ�����͵õ���ͼ��dilate_imageʹ�ýṹԪ��elementX���и�ʴ��������������洢��erode_image�С���ʴ�ĵ�������Ϊ4
	erode(dilate_image, erode_image, elementX, point, 4);
	//�ٴζ���һ����ʴ�õ���ͼ��erode_imageʹ�ýṹԪ��elementX�������Ͳ�������������洢��dilate_image�С����͵ĵ�������Ϊ2
	dilate(erode_image, dilate_image, elementX, point, 2);

	//ͨ����X�����Ͻ����������Ͳ��������Խ�һ����ǿͼ����ˮƽ�����ϵ���ͨ��

	//�Զ���ˣ����� Y ��������͸�ʴ
	//����һ�����͵õ���ͼ��dilate_imageʹ�ýṹԪ��elementY���и�ʴ��������������洢��erode_image�С���ʴ�ĵ�������Ϊ1
	erode(dilate_image, erode_image, elementY, point, 1);
	//�ٴζ���һ����ʴ�õ���ͼ��erode_imageʹ�ýṹԪ��elementY�������Ͳ�������������洢��dilate_image�С����͵ĵ�������Ϊ2
	dilate(erode_image, dilate_image, elementY, point, 2);

	//������ͼ��������͸�ʴ����ȥ��ͼ�����㣬���ն�
	
	//ƽ������ ��ֵ�˲�
	Mat median_Image;
	//����һ�����͵õ���ͼ��dilate_image������ֵ�˲���������������洢��median_Image�С��˲����Ĵ�СΪ15x15
	medianBlur(dilate_image, median_Image, 15);
	//�ٴζ���һ����ֵ�˲��õ���ͼ��median_Image������ֵ�˲����������������Ȼ�洢��median_Image�С��˲����Ĵ�СΪ15x15
	medianBlur(median_Image, median_Image, 15);

	//ͨ������������ֵ�˲����������Խ�һ������������Ӱ�죬
	// ��һ����ֵ�˲�����ȥ���ϴ�����������ڶ�����ֵ�˲�����Խ�һ��ƽ��ͼ��
	//imshow("��ֵ�˲�", median_Image);
	return median_Image;
}

Mat Locate_License_Plate(Mat temp, Mat src, Mat gray_src)//���ƶ�λ
{
	vector<vector<Point>> contours;
	findContours(temp, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//��������
	//findContours() ��������ͼ���е�ÿ�����أ��ҵ������ı�Ե���أ�
	//�����������һ����������������������֮������ӹ�ϵ�Ϳռ�λ�ã������ڵı�Ե���ع�Ϊͬһ������

	drawContours(temp, contours, -1, Scalar(255), 1);
	//������ʾΪһ������
	Mat Roi;
	for (int i = 0; i < contours.size(); i++)
	{
		RotatedRect rect = minAreaRect(Mat(contours[i]));
		Point2f p[4];
		rect.points(p);
		double axisLongTemp = 0.0, axisShortTemp = 0.0;//���εĳ��ߺͶ̱�
		axisLongTemp = sqrt(pow(p[1].x - p[0].x, 2) + pow(p[1].y - p[0].y, 2));  //���㳤�ᣨ���ɶ���
		axisShortTemp = sqrt(pow(p[2].x - p[1].x, 2) + pow(p[2].y - p[1].y, 2)); //������ᣨ���ɶ���
		double LengthTemp;     //�м����
		if (axisShortTemp > axisLongTemp)   //������ڳ��ᣬ��������
		{
			LengthTemp = axisLongTemp;
			axisLongTemp = axisShortTemp;
			axisShortTemp = LengthTemp;
		}
		double rectArea = axisLongTemp * axisShortTemp;//����������
		double Area = contourArea(Mat(contours[i]));//�������
		double rectDegree = Area / rectArea;//������ζ�
		//�ⲿ�ֵ������жϿ���ʵ�����������
		if (axisLongTemp / axisShortTemp >= 2.2 && axisLongTemp / axisShortTemp <= 5.1 && rectDegree > 0.63 && rectDegree < 1.37 && rectArea>2000 && rectArea < 50000)
		{
			for (int i = 0; i < 4; i++)       //���߿����������
				line(src, p[i], p[((i + 1) % 4) ? (i + 1) : 0], Scalar(0, 0, 255), 2, 8, 0);

			float width_height = (float)rect.size.width / (float)rect.size.height;
			//�����µ�opencv-python 4.5.1.48�汾�У�����Ƕȵķ�Χ�����Ǹ����ˣ������(0,90]����С�Ǧ�
			//����ǣ��൱��y��˳ʱ����ת����С��Ӿ��ε�һ���غϵı�֮��ĽǶ�
			float angle = rect.angle;
			if (width_height < 1)
			{
				angle = angle - 90;
			}
			Mat rotMat = getRotationMatrix2D(rect.center, angle, 1);//��þ��ε���ת����
			Mat warpImg;
			warpAffine(gray_src, warpImg, rotMat, src.size(), INTER_CUBIC, BORDER_CONSTANT);
			imshow("����任", warpImg);
			//ͼ���и�
			Size minRectSize = rect.size;
			if (width_height < 1)
				swap(minRectSize.width, minRectSize.height);
			getRectSubPix(warpImg, minRectSize, rect.center, Roi);
		}
	}

	imshow("test", src);
	imshow("������ȡ���", Roi);
	return Roi;
}

//����任
Mat Affine_Transform(Mat temp)
{
	Mat warp_dstImage = Mat::zeros(100, 500, temp.type());
	Point2f srcTri[3];
	Point2f dstTri[3];
	//�������������������任
	srcTri[0] = Point2f(0, 0);
	srcTri[1] = Point2f(temp.cols, 0);
	srcTri[2] = Point2f(0, temp.rows);

	dstTri[0] = Point2f(0, 0);
	dstTri[1] = Point2f(500, 0);
	dstTri[2] = Point2f(0, 100);
	//�������任����
	Mat warp_mat(2, 3, CV_32FC1);
	warp_mat = getAffineTransform(srcTri, dstTri);
	//�Լ���ͼ�ν��з���任����
	warpAffine(temp, warp_dstImage, warp_mat, warp_dstImage.size());
	return warp_dstImage;
}

//�Ƴ����ƴ�ֱ�߿�
Mat Remove_Vertial_Border(Mat temp)
{
	//һ�����ξ�������Ϊ1��������tempͼ����ͬ�������ڶ������������̬ѧ��������״�ʹ�С
	Mat vline = getStructuringElement(MORPH_RECT, Size(1, temp.cols), Point(-1, -1));
	Mat dst1, temp1;
	//���и�ʴ����,��ʴ�������԰���ϸ�����Ƴ�ĳЩ�ṹ
	erode(temp, temp1, vline);

	//�������Ͳ���,���Ͳ������԰����Ӵֻ���չĳЩ�ṹ
	dilate(temp1, dst1, vline);
	/*namedWindow("��ȡ��ֱ��", WINDOW_AUTOSIZE);*/
	/*imshow("��ȡ��ֱ��", dst1);*/

	//��temp���Ƴ�dst1������ԭʼͼ�����Ƴ���ֱ��
	subtract(temp, dst1, temp, Mat());
	/*imshow("�и�ƴ�ֱ�߿���", temp);*/
	return temp;
}

Mat Remove_Horizon_Border(Mat temp)
{
	Mat hline = getStructuringElement(MORPH_RECT, Size(60, 1), Point(-1, -1));//������״Ϊ��1*src.cols
	Mat dst1, temp1;
	erode(temp, temp1, hline);
	dilate(temp1, dst1, hline);
	/*namedWindow("��ȡˮƽ��", WINDOW_AUTOSIZE);*/
	/*imshow("��ȡˮƽ��", dst1);*/
	subtract(temp, dst1, temp, Mat());
	/*imshow("�и��ˮƽ�߿���", temp);*/
	return temp;
}

//ˮƽ�ü�����
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

//�����ַ���λ
void Locate_String(int *x_begin, int *x_end, Mat temp)
{
	int *counter_x = new int[temp.cols];//��¼ÿһ�еİ����ظ���
	for (int i = 0; i < temp.cols; i++)
		counter_x[i] = 0;
	for (int col = 0; col < temp.cols; col++)
	{
		int count = 0;
		for (int row = 0; row < temp.rows; row++)
		{
			if (temp.at<uchar>(row, col) == 255)//���������ֵ��ʾ��ɫ
			{
				count++;
			}
		}
		counter_x[col] = count;
	}
	int index_col = 0;
	int number_width = 0;//��¼�ַ����
	for (int i = 0; i < temp.cols - 1; i++)
	{
		if (counter_x[i] >= 3)//��ÿһ�еİ����ظ�������ĳһ��ֵ���ַ���ȿ�ʼ�ۼ�
		{
			number_width++;
			if (number_width > 8)//10 �������ظ�������ĳһ��ֵ�������еĸ�������10����ʼ�����ַ�λ����Ϣ
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

//�ַ��ָ�
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
	imshow("���ƺ���ָ���", Result);
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
		//	//picture1�Ƕ�ֵͼ��
		Mat picture1 = Mat::zeros(temp.rows, x_end[i] - x_begin[i], temp.type());
		for (int row = 0; row < picture1.rows; row++)
		{
			for (int col = 0; col < picture1.cols; col++)
			{
				picture1.at<uchar>(row, col) = temp.at<uchar>(row, col + x_begin[i]);
			}
		}
		Mat NUM[28];//�ַ�ƥ��ģ��
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
				cout << "δ�ܶ�ȡ" << name << endl;
			}
			cvtColor(NUM[i], NUM[i], COLOR_BGR2GRAY);
			threshold(NUM[i], NUM[i], 0, 255, THRESH_BINARY);

			Point2f srcTri[3];
			Point2f dstTri[3];
			Mat warp_mat(2, 3, CV_32FC1);
			//��������任Ŀ��ͼ����ԭͼ��ߴ�������ͬ
			Mat result = Mat::zeros(picture1.rows, picture1.cols, picture1.type());
			//�������������������任
			srcTri[0] = Point2f(0, 0);
			srcTri[1] = Point2f(NUM[i].cols, 0);
			srcTri[2] = Point2f(0, NUM[i].rows);
			dstTri[0] = Point2f(0, 0);
			dstTri[1] = Point2f(picture1.cols, 0);
			dstTri[2] = Point2f(0, picture1.rows);
			//�������任����
			warp_mat = getAffineTransform(srcTri, dstTri);
			//�Լ���ͼ�ν��з���任����
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
			cout << "��" << '\t';
		else if (Index == 13)
			cout << "��" << '\t';
		else if (Index == 14)
			cout << "B" << '\t';
		else if (Index == 15)
			cout << "S" << '\t';
		else if (Index == 16)
			cout << "��" << '\t';
		else if (Index == 17)
			cout << "N" << '\t';
		else if (Index == 18)
			cout << "J" << '\t';
		else if (Index == 19)
			cout << "P" << '\t';
		else if (Index == 20)
			cout << "A" << '\t';
		else if (Index == 21)
			cout << "��" << '\t';
		else if (Index == 22)
			cout << "G" << '\t';
		else if (Index == 23)
			cout << "U" << '\t';
		else if (Index == 24)
			cout << "ԥ" << '\t';
		else if (Index == 25)
			cout << "K" << '\t';
		else if (Index == 26)
			cout << "��" << '\t';
		else if (Index == 27)
			cout << "��" << '\t';

		else if (Index >= 0 && Index <= 9)
			cout << Index << '\t';
	}
	cout << endl;
}