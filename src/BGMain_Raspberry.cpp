#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "package_bgs/bgslibrary.h"
#include <stdlib.h>

using namespace std;
using namespace cv;

bool isLeftSideDetected = false; // 摄像头视角的左边三分之一区域是否被触发
bool isMiddleSideDetected = false;
bool isRightSideDetected = false;

bool hasInitedSrcWindow = false;

#define WINDOW_SRC "SRC"
#define WINDOW_DEST "DEST"
int LEFT_TOP_X = 181;
int LEFT_TOP_Y = 187;
int RIGHT_BOTTOM_X = 589;
int RIGHT_BOTTOM_Y = 400;

const int KEY_ESC = 27;
int key_press = 0;

int CAMERA_NUM = 0;

//全局变量
Mat g_srcImage, g_srcImage_fliped;
Mat g_filterImage;
Mat g_grayImage;
int g_medianBlurThresh = 7;
int g_medianBlurGridThresh = 1;
int g_medianBlurMaxThresh = 30;
RNG g_rng(123456);
Rect g_rectangle;
bool g_bDrawingBox = false;//是否进行绘制
bool showOutput = true;
int modeGrid2 = false;//格子使用动点检测方法
bool modeScan = false;//控制人体扫描互动
int camera_BRIGHTNESS = 60; // 相机的亮度设置
int camera_GAIN = 70; // 相机的增益设置

string config_xml = "./config/VideoCapture_config.xml";

Mat hsvImage_base_template_pre, hsvImage_base_template;
Mat hsvImage_base;//直方图对比变量
Mat hsvImage_halfDown;
int histMinValue = 5;

int row_resolution = 1;//定义格子的横向百分比
int col_resolution = 1;//定义格子的竖向百分比
int row_count = 0;
int col_count = 0;
int grid_width = 0;//格子的宽高
int grid_height = 0;

int image_fliped_direc = -2;//定义原始图像翻转的形式，默认不进行翻转，如果检测到位-2则不执行翻转处理，因为小于0都会进行翻转

void DrawRectangle(cv::Mat& img, cv::Rect box);
void on_MouseHandle(int event, int x, int y, int flags, void* param);
void setGridStatus(int x, int y, int grid_array[][100]);
void saveConfig();
void on_ContoursChange(int, void*)
{	
	if (g_medianBlurThresh % 2 == 0)
		g_medianBlurThresh += 1;
	if (g_medianBlurGridThresh % 2 == 0) {
		g_medianBlurGridThresh += 1;
	}	
}
void on_ContoursChangeHalf(int, void*)
{
	cout << "--histMinValue = " << histMinValue << endl;
	saveConfig();
}

double preStartTime = getTickCount();
double preStopTime = getTickCount();
bool hasMotorStopped = false;
void startMotor(int pinNum) {
	if ((getTickCount() - preStartTime) / getTickFrequency() * 1000 >= 1000 || hasMotorStopped) {
		cout << "--启动电机" << endl;
		hasMotorStopped = false;
		char str[20] = { 0 };
		sprintf(str, "gpio -g write %d 1", pinNum);
		system(str);
		preStartTime = getTickCount();
	}	
	preStopTime = getTickCount();
}

void stopMotor(int pinNum) {
	if ((getTickCount() - preStopTime) / getTickFrequency() * 1000 >= 1000) {
		cout << "--停止电机" << endl;
		hasMotorStopped = true;
		char str[20] = { 0 };
		sprintf(str, "gpio -g write %d 0", pinNum);
		system(str);
		preStopTime = getTickCount();
	}	
}

void saveConfig()
{
	CvFileStorage* fs = cvOpenFileStorage(config_xml.c_str(), 0, CV_STORAGE_WRITE);
	cvWriteString(fs, "commit_camera", "启动哪个摄像头，默认0");
	cvWriteInt(fs, "CAMERA_NUM", CAMERA_NUM);
	cvWriteString(fs, "commit_X", "左上角x坐标");
	cvWriteInt(fs, "LEFT_TOP_X", LEFT_TOP_X);
	cvWriteString(fs, "commit_Y", "左上角y坐标");
	cvWriteInt(fs, "LEFT_TOP_Y", LEFT_TOP_Y);
	cvWriteString(fs, "commit_BX", "右下角x坐标");
	cvWriteInt(fs, "RIGHT_BOTTOM_X", RIGHT_BOTTOM_X);
	cvWriteString(fs, "commit_BY", "右下角y坐标");
	cvWriteInt(fs, "RIGHT_BOTTOM_Y", RIGHT_BOTTOM_Y);
	cvWriteString(fs, "commit_show", "是否显示图像以及打印终端信息");
	cvWriteInt(fs, "showOutput", showOutput);
	
	cvWriteString(fs, "commit_mode2", "是否设置为人体扫描模式");
	cvWriteInt(fs, "modeScan", modeScan);
	cvWriteString(fs, "commit_mode4", "格子检测的另一种方法, 1使用普通USB摄像头，2表示需要进行额外初始化的摄像头");
	cvWriteInt(fs, "modeGrid2", modeGrid2);
	cvWriteString(fs, "commit_brightness", "洛日摄像头的亮度设置0-64");
	cvWriteInt(fs, "camera_BRIGHTNESS", camera_BRIGHTNESS);
	cvWriteString(fs, "commit_gain", "洛日摄像头的增益设置0-100");
	cvWriteInt(fs, "camera_GAIN", camera_GAIN);
	
	cvWriteString(fs, "row_resolution_commit", "定义格子的横向精度，是一个百分比值1-100，同时需要将modeScan设置为1，建议取可以被100整除的值");
	cvWriteInt(fs, "row_resolution", row_resolution);
	cvWriteString(fs, "col_resolution_commit", "定义格子的竖向精度，是一个百分比值1-100");
	cvWriteInt(fs, "col_resolution", col_resolution);
	cvWriteString(fs, "image_fliped_direc_commit", "定义原始图像翻转的形式，0: 沿横向x轴翻转, 1: 沿竖向y轴翻转, -1: x、y轴同时翻转, -2不进行翻转");
	cvWriteInt(fs, "image_fliped_direc", image_fliped_direc);
	cvWriteString(fs, "g_medianBlurGridThresh_commit", "图像滤波因子，数值越大则过滤越好");
	cvWriteInt(fs, "g_medianBlurGridThresh", g_medianBlurGridThresh);
	cvWriteInt(fs, "histMinValue", histMinValue);

	cvReleaseFileStorage(&fs);
}

void loadConfig()
{
	CvFileStorage* fs = cvOpenFileStorage(config_xml.c_str(), nullptr, CV_STORAGE_READ);
	if (fs)
	{
		CAMERA_NUM = cvReadIntByName(fs, nullptr, "CAMERA_NUM", 0);
		LEFT_TOP_X = cvReadIntByName(fs, nullptr, "LEFT_TOP_X", 0);
		LEFT_TOP_Y = cvReadIntByName(fs, nullptr, "LEFT_TOP_Y", 0);
		RIGHT_BOTTOM_X = cvReadIntByName(fs, nullptr, "RIGHT_BOTTOM_X", 0);
		RIGHT_BOTTOM_Y = cvReadIntByName(fs, nullptr, "RIGHT_BOTTOM_Y", 0);
		if (RIGHT_BOTTOM_X == LEFT_TOP_X) {
			RIGHT_BOTTOM_X = LEFT_TOP_X + 100;
		}
		if (RIGHT_BOTTOM_Y == LEFT_TOP_Y) {
			RIGHT_BOTTOM_Y = LEFT_TOP_Y + 100;
		}
		showOutput = cvReadIntByName(fs, nullptr, "showOutput", false);
		modeScan = cvReadIntByName(fs, nullptr, "modeScan", false);
		modeGrid2 = cvReadIntByName(fs, nullptr, "modeGrid2", 0);
		camera_BRIGHTNESS = cvReadIntByName(fs, nullptr, "camera_BRIGHTNESS", 50);
		camera_GAIN = cvReadIntByName(fs, nullptr, "camera_GAIN", 50);
		row_resolution = cvReadIntByName(fs, nullptr, "row_resolution", 1);//读取模板匹配格子精度
		col_resolution = cvReadIntByName(fs, nullptr, "col_resolution", 1);
		row_count = (int)100 / row_resolution;//计算行数
		col_count = (int)100 / col_resolution;
		grid_width = (RIGHT_BOTTOM_X - LEFT_TOP_X) / col_count;//启动计算格子的高宽
		grid_height = (RIGHT_BOTTOM_Y - LEFT_TOP_Y) / row_count;
		image_fliped_direc = cvReadIntByName(fs, nullptr, "image_fliped_direc", -2);//加载图像翻转的形式
		g_medianBlurGridThresh = cvReadIntByName(fs, nullptr, "g_medianBlurGridThresh", 1);
		if (g_medianBlurGridThresh % 2 == 0) {
			g_medianBlurGridThresh += 1;
		}
		histMinValue = cvReadIntByName(fs, nullptr, "histMinValue", 5);

		cvReleaseFileStorage(&fs);
	}
	else {
		std::cout << "--first run config file not exist, will create one" << endl;
		saveConfig();
	}
}

int main(int argc, char **argv)
{
	cout << "Running:" << endl;
	cout << "\t1.配置文件路径: config/VideoCapture_config.xml" << endl;

	VideoCapture capture(CAMERA_NUM);
	if (!capture.isOpened())
	{
		cerr << "--打开摄像头错误，请检查，重试" << endl;
		// sleep(3);
	}
	//load config
	loadConfig();
	system("gpio -g mode 4 out");

	if (modeGrid2) {//进入格子检测的采用动点检测的方法
		cout << "--进入格子另一种检测互动模式" << endl;
		IBGS *bgs = new DPMean;//40 DPMean
		cout << "--modeGrid2 = " << modeGrid2 << endl;
		if (modeGrid2 == 2) { // 使用洛日红外摄像头，需要进行额外的设置
			capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
			capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
			capture.set(CV_CAP_PROP_BRIGHTNESS, camera_BRIGHTNESS);
			capture.set(CV_CAP_PROP_GAIN, camera_GAIN);
		}
		while (1) {
			double t = (double)getTickCount();
			capture >> g_srcImage;
			if (g_srcImage.empty())
			{
				cout << "frame empty, exit" << endl;
				break;
			}
			if (image_fliped_direc != -2) {//如果原始图像需要进行翻转
				flip(g_srcImage, g_srcImage_fliped, image_fliped_direc);
				g_srcImage = g_srcImage_fliped;//将翻转之后的图像替换原始图像
			}
			if (showOutput)
			{
				if (!hasInitedSrcWindow)
				{//如果还没有初始化显示窗口
					namedWindow(WINDOW_SRC, WINDOW_AUTOSIZE);
					setMouseCallback(WINDOW_SRC, on_MouseHandle, (void*)&g_srcImage);//设置鼠标操作回调函数
					createTrackbar("contous", WINDOW_SRC, &g_medianBlurGridThresh, g_medianBlurMaxThresh, on_ContoursChange);
					on_ContoursChange(0, 0);
					hasInitedSrcWindow = true;
				}
				if (g_bDrawingBox) {
					DrawRectangle(g_srcImage, g_rectangle);//当进行绘制的标识符为真，则进行绘制
				}
				// 绘制框选区域方格	
				for (int i = 0; i < row_count + 1; i++) {
					int x1 = LEFT_TOP_X;
					int y1 = LEFT_TOP_Y + i*grid_height + 1;
					if ((y1 > RIGHT_BOTTOM_Y) || (i == row_count && y1 < RIGHT_BOTTOM_Y)) { // 避免边框的线出现不齐的情况
						y1 = RIGHT_BOTTOM_Y;
					}
					int x2 = RIGHT_BOTTOM_X;
					int y2 = y1;
					line(g_srcImage, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255), 1);
				}
				for (int j = 0; j < col_count + 1; j++) {
					int x1 = LEFT_TOP_X + j*grid_width + 1;
					if ((x1 > RIGHT_BOTTOM_X) || (j == col_count && x1 < RIGHT_BOTTOM_X)) {
						x1 = RIGHT_BOTTOM_X;
					}
					int y1 = LEFT_TOP_Y;
					int x2 = x1;
					int y2 = RIGHT_BOTTOM_Y;
					line(g_srcImage, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 255), 1);
				}				
			}

			Mat img_mask;//前景移动物体图像
			Mat img_bkgmodel;//祛除移动物体的背景图像
			bgs->process(g_srcImage, img_mask, img_bkgmodel); // by default, it shows automatically the foreground mask image

			//1.对前景移动物体进行滤波,中值滤波
			medianBlur(img_mask, g_filterImage, g_medianBlurGridThresh);

			// 找出运动物体轮廓
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			findContours(g_filterImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

			// 多边形逼近轮廓+获取矩形和圆形边界框
			vector<vector<Point> > contours_poly(contours.size());
			vector<Rect> boundRect(contours.size());
			vector<Point2f>center(contours.size());
			vector<float>radius(contours.size());
			
			int grid_array[100][100] = { 0 };//定义存储格子值的数组，最大100格子精度
			// 循环遍历所有的部分
			for (unsigned int i = 0; i < contours.size(); i++)
			{
				approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);//用指定精度逼近多边形曲线
				for (vector<Point>::const_iterator itp = contours_poly[i].begin(); itp != contours_poly[i].end(); itp++) {
					setGridStatus(itp->x, itp->y, grid_array);
				}
			}			

			if (showOutput) {
				if (isLeftSideDetected) {
					cout << "Left" << endl;					
				}
				if (isMiddleSideDetected) {
					cout << "Middle" << endl;					
				}
				if (isRightSideDetected) {
					cout << "Right" << endl;					
				}				
				for (int i = 0; i < row_count; i++) { // 在彩色图像方格上显示触发点
					for (int j = 0; j < col_count; j++) {
						if (grid_array[i][j] == 1) {
							int x = LEFT_TOP_X + j*grid_width; // 每个子区域的左上角x,y坐标
							int y = LEFT_TOP_Y + i*grid_height;
							Point p0;
							p0.x = x + grid_width / 2;
							p0.y = y + grid_height / 2;
							circle(g_srcImage, p0, 5, Scalar(0, 0, 255), -1);
						}

					}
				}
			}		

			if (isLeftSideDetected || isRightSideDetected || isMiddleSideDetected) {				
				startMotor(4);
				isLeftSideDetected = false;
				isMiddleSideDetected = false;
				isRightSideDetected = false;
			}
			else {				
				stopMotor(4);
			}

			//// 绘制多边形轮廓+包围的矩形框+圆形框			
			//Mat drawing = Mat::zeros(g_filterImage.size(), CV_8UC3);
			//for (int unsigned i = 0; i<contours.size(); i++)
			//{
			//	Scalar color = Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255));//设置随机颜色
			//	drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());//绘制轮廓
			//}

			if (showOutput) {
				imshow(WINDOW_SRC, g_srcImage); // 显示经过处理之后的彩色图像
				// namedWindow(WINDOW_DEST, WINDOW_AUTOSIZE);
				// imshow(WINDOW_DEST, drawing);
			}

			t = ((double)getTickCount() - t) / getTickFrequency();
			if (showOutput) {
				cout << "--time= " << t * 1000 << " ms" << endl;
			}
			key_press = cvWaitKey((int)(10));
			if (key_press == KEY_ESC) {//退出整个程序
				cout << "--退出程序" << endl;
				break;
			}
		}
		delete bgs;
	}
	else if (modeScan) {
		cout << "--进入人体扫描互动模式" << endl;
		if (modeScan == 2) {
			capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
			capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
			capture.set(CV_CAP_PROP_BRIGHTNESS, camera_BRIGHTNESS);
			capture.set(CV_CAP_PROP_GAIN, camera_GAIN);
		}
		//【1】初始化计算直方图需要的实参
		// 对hue通道使用30个bin,对saturatoin通道使用32个bin
		int h_bins = 50; int s_bins = 60;
		int histSize[] = { h_bins, s_bins };
		// hue的取值范围从0到256, saturation取值范围从0到180
		float h_ranges[] = { 0, 256 };
		float s_ranges[] = { 0, 180 };
		const float* ranges[] = { h_ranges, s_ranges };
		// 使用第0和第1通道
		int channels[] = { 0, 1 };

		//【2】 创建储存直方图的 MatND 类的实例:
		MatND baseHist;
		MatND halfDownHist;
		int readPicCount = 0;

		while (1) {
			capture >> g_srcImage;
			if (g_srcImage.empty())
			{
				cout << "frame empty, exit" << endl;
				break;
			}
			if (image_fliped_direc != -2) {//如果原始图像需要进行翻转
				flip(g_srcImage, g_srcImage_fliped, image_fliped_direc);
				g_srcImage = g_srcImage_fliped;//将翻转之后的图像替换原始图像
			}
			if (showOutput) {
				if (!hasInitedSrcWindow)
				{//如果还没有初始化显示窗口
					namedWindow(WINDOW_SRC, WINDOW_AUTOSIZE);
					setMouseCallback(WINDOW_SRC, on_MouseHandle, (void*)&g_srcImage);//设置鼠标操作回调函数
					createTrackbar("contous", WINDOW_SRC, &histMinValue, g_medianBlurMaxThresh, on_ContoursChangeHalf);
					on_ContoursChangeHalf(0, 0);
					hasInitedSrcWindow = true;
				}
				if (g_bDrawingBox) DrawRectangle(g_srcImage, g_rectangle);//当进行绘制的标识符为真，则进行绘制
				imshow(WINDOW_SRC, g_srcImage);
			}
			if (readPicCount <= 99) {
				readPicCount++;
				cout << "--readPicCount = " << readPicCount << endl;
				continue;
			}
			else if (readPicCount == 100) {
				readPicCount++;				
				cvtColor(g_srcImage, hsvImage_base, COLOR_BGR2HSV);//【3】 将图像由BGR色彩空间转换到 HSV色彩空间
				hsvImage_halfDown = hsvImage_base(Rect(LEFT_TOP_X, LEFT_TOP_Y, RIGHT_BOTTOM_X - LEFT_TOP_X, RIGHT_BOTTOM_Y - LEFT_TOP_Y));
				if (showOutput) {//显示部分框选画面
					imshow("ROI", hsvImage_halfDown);
				}
				cout << "--复制初始图像到模板图像中" << endl;
				hsvImage_base_template = hsvImage_halfDown.clone();
			}
			else {
				//【3】 将图像由BGR色彩空间转换到 HSV色彩空间
				cvtColor(g_srcImage, hsvImage_base, COLOR_BGR2HSV);

				//【4】创建包含基准图像下半部的半身图像(HSV格式)  从原始图像中提取感兴趣的区域，每帧图像与模板区域进行比较
				hsvImage_halfDown = hsvImage_base(Rect(LEFT_TOP_X, LEFT_TOP_Y, RIGHT_BOTTOM_X - LEFT_TOP_X, RIGHT_BOTTOM_Y - LEFT_TOP_Y));
				if (showOutput) {//显示部分框选画面
					imshow("ROI", hsvImage_halfDown);
				}
				// 【5】分别计算基准图像，半身基准图像的HSV直方图:
				calcHist(&hsvImage_base_template, 1, channels, Mat(), baseHist, 2, histSize, ranges, true, false);//计算模板图像的直方图
				normalize(baseHist, baseHist, 0, 1, NORM_MINMAX, -1, Mat());

				calcHist(&hsvImage_halfDown, 1, channels, Mat(), halfDownHist, 2, histSize, ranges, true, false);
				normalize(halfDownHist, halfDownHist, 0, 1, NORM_MINMAX, -1, Mat());

				//【6】按顺序使用第一种对比标准将基准图像的直方图与其余各直方图进行对比:
				//进行图像直方图的对比
				// double base_base = compareHist(baseHist, baseHist, 0);
				double base_half = compareHist(baseHist, halfDownHist, 0);
				//输出结果
				if (showOutput) {
					printf("匹配结果如下:histMinValue = %.2f, 【比半身图】：%.2f; \n\n", histMinValue*0.1, base_half);
				}
				if (base_half <= histMinValue*0.1) {
					startMotor(4);
				}
				else {
					stopMotor(4);
				}
			}			
			
			key_press = cvWaitKey(30);
			if (key_press == KEY_ESC) {//退出整个程序
				cout << "--退出程序" << endl;
				break;
			}
		}
	}
	else {
		cout << "--不支持的互动模式，请在配置文件中选择一种模式" << endl;
		// sleep(3);
	}

	capture.release();
	cvDestroyAllWindows();

	getchar();//用于保持窗口显示
	return 0;
}

/**
判断传递的(x,y)点是否在矩形内，并设置格子状态点
*/
void setGridStatus(int x, int y, int grid_array[][100]) {
	int width_one_third = (RIGHT_BOTTOM_X - LEFT_TOP_X) / 3;
	int height_one_third = (RIGHT_BOTTOM_Y - LEFT_TOP_Y) / 3;
	if ((x >= LEFT_TOP_X && x <= RIGHT_BOTTOM_X) && (y >= LEFT_TOP_Y && y <= RIGHT_BOTTOM_Y)) {
		x = x - LEFT_TOP_X;//计算在所画框框中的相对位置
		y = y - LEFT_TOP_Y;
		grid_array[y/grid_height][x/grid_width] = 1;//将矩形所对应的格子状态置为1
		// cout << "grid_x=" << x / grid_width << ",grid_y=" << y / grid_height << endl;
		if (x <= (LEFT_TOP_X + width_one_third)) {
			isLeftSideDetected = true;
		}
		else if (x > (LEFT_TOP_X + width_one_third) && x <= (LEFT_TOP_X + width_one_third * 2)) {
			isMiddleSideDetected = true;
		}
		else if (x > (LEFT_TOP_X + width_one_third * 2)) {
			isRightSideDetected = true;
		}
	}
}

//-----------------------------------【DrawRectangle( )函数】------------------------------
//		描述：自定义的矩形绘制函数
//-----------------------------------------------------------------------------------------------
void DrawRectangle(cv::Mat& img, cv::Rect box)
{
	cv::rectangle(img, box.tl(), box.br(), cv::Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)));//随机颜色
}
void on_MouseHandle(int event, int x, int y, int flags, void* param)
{

	Mat& image = *(cv::Mat*) param;//参数为原始一帧图像
	Mat originalImage = image.clone();//在原始图像上进行画框之前将图像复制一份

	switch (event)
	{
		//鼠标移动消息
	case EVENT_MOUSEMOVE:
	{
		if (g_bDrawingBox)//如果是否进行绘制的标识符为真，则记录下长和宽到RECT型变量中
		{
			g_rectangle.width = x - g_rectangle.x;
			g_rectangle.height = y - g_rectangle.y;
		}
	}
	break;

	//左键按下消息
	case EVENT_LBUTTONDOWN:
	{
		g_bDrawingBox = true;
		g_rectangle = Rect(x, y, 0, 0);//记录起始点
	}
	break;

	//左键抬起消息
	case EVENT_LBUTTONUP:
	{
		g_bDrawingBox = false;//置标识符为false
							  //对宽和高小于0的处理
		if (g_rectangle.width < 0)
		{
			g_rectangle.x += g_rectangle.width;
			g_rectangle.width *= -1;
		}

		if (g_rectangle.height < 0)
		{
			g_rectangle.y += g_rectangle.height;
			g_rectangle.height *= -1;
		}
		//调用函数进行绘制
		DrawRectangle(image, g_rectangle);
		LEFT_TOP_X = g_rectangle.x;
		LEFT_TOP_Y = g_rectangle.y;
		if (g_rectangle.width > 0) {//如果只是鼠标点击了一下，检测区域不变
			RIGHT_BOTTOM_X = g_rectangle.x + g_rectangle.width;
		}
		else if (g_rectangle.width == 0) {
			RIGHT_BOTTOM_X = LEFT_TOP_X + 1;
		}
		if (g_rectangle.height > 0) {
			RIGHT_BOTTOM_Y = g_rectangle.y + g_rectangle.height;
		}
		else if (g_rectangle.height == 0) {
			RIGHT_BOTTOM_Y = LEFT_TOP_Y + 1;
		}
		grid_width = (RIGHT_BOTTOM_X - LEFT_TOP_X) / col_count;//重新计算模板区域格子的高宽
		grid_height = (RIGHT_BOTTOM_Y - LEFT_TOP_Y) / row_count;
		saveConfig();//写入文件
		if (showOutput) {
			cout << "--起点x = " << g_rectangle.x << ", y = " << g_rectangle.y << "， 区域宽w = " << g_rectangle.width << ", h = " << g_rectangle.height << endl;
		}
	}
	break;

	}
}