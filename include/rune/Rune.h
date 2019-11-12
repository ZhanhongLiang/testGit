/*
 * @Description: 
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-09-05 20:23:52
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-10-28 20:04:05
 */

#ifndef BUFF_DETECTOR_H_
#define BUFF_DETECTOR_H_
#define WINDOWS_DEBUG

#include "../../include/header/Header.h"
//#include "./AngleSolver.hpp"
//#include "driver/Camera.h"
#define NO_TARGET -1
#define MAX_NUM 921600//???
#define FRAME_NUM 20

class Buff_Detector
{
	//红蓝顺逆时针选择
	enum TeamSelect
	{
		BLUE_CLOCK = 1,
		BLUE_ANCLOCK = 2,
		RED_CLOCK = 3,
		RED_ANCLOCK = 4,
		BLUE_STATIC = 5,
		RED_STATIC = 6
	};

	//自适应调节模式
	enum BinaryMode
	{
		BGR = 1,
		HSV = 2,
		GRAY = 3,
		OTSU = 4,
	};
	//一般选择的是切线模式
	enum PredictMode
	{
		FIT_CIRCLE = 1,
		PUSH_CIRCLE = 2,
		TANGENT = 3
	};

	//选择模式
	struct SwitchParam
	{
		bool debug;
		bool fire;
		//初始化
		SwitchParam()
		{
			debug = 1;
			fire = 2;
		}
	};

	//装甲板数据初始化
	struct ArmorData
	{
		Point2f armorCenter;//装甲板中心点
		Point2f RadiusCenter;//半径中心点
		float angle;//角度
		int quadrant;//象限
		bool isFind;//是否找到
		ArmorData()
		{
			armorCenter = Point2f(0, 0);
			RadiusCenter = Point2f(0, 0);
			angle = 0;
			quadrant = 0;
			isFind = 0;
		}
	};

	//识别参数,用来调试参数
	struct DectParam
	{
		int bMode;//灰度图的模式
		int pMode;//预测画圆的模式
		Mat element;
		int radius;//圆的直径
		float noise_point_area;//噪点的面积
		int flabellum_area_min; //扇子的面积;
		int flabellum_area_max;
		float flabellum_whrio_min;//扇子的宽高比例最小值
		float flabellum_whrio_max;//扇子的宽高比例最大值
		float armor_whrio_min;//装甲板的宽高比例最小值
		float armor_whrio_max;//装甲板宽高比例最大值
		float armor_rev_thres;
		int armor_area_min;

		int cutLimitedTime;//时间限制？？
		float preAngle;//预测角度
		DectParam()
		{
			//--------半径，这个修改可以调整画圆的大小---------
			radius = 268;
			//模式
			bMode = BGR;//二值化方法；
			pMode = TANGENT;
			//getArmorCenter
			element = getStructuringElement(MORPH_RECT, Size(7, 7));
			noise_point_area = 200;//噪点面积
			flabellum_area_min = 3500;// standard:7000
			flabellum_area_max = 7000;// 扇形最大面积
			flabellum_whrio_min = 1.5;
			flabellum_whrio_max = 2.7;//standard:2
			armor_whrio_min = 1.5;
			armor_whrio_max = 2.7;// standard:2
			armor_rev_thres = 0.3;// standard: 0.0x，轮廓相似度
			armor_area_min = 300;
			// cutLimit
			cutLimitedTime = 40;// 400ms
								// predict
			preAngle = CV_PI / 7.8;//预测角度的大小
		}
	};
	
private:
	//param
	SwitchParam sParam;
	DectParam param;

	//init
	int mode;
	int tSelect;//红蓝模式选择
	cv::Mat debug_src_img;//调节图像
	bool dirFlag;
	uint frame_cnt = 0;//是否切换的帧数统计
						  //int teamSelect = RED;
	cv::Mat src_img;
	ArmorData lastData;//上一次的数据
	ArmorData lostData;//丢帧后的数据
	//AngleSolver *angleSolver;

public:

	//bool SetImage(const Mat src_img, Mat &, Point2f &offset);
	//初始化文件
	Buff_Detector();
	bool SetBinary(const Mat src_img, Mat &bin_img, int bMode);
	bool GetArmorCenter(const Mat src_img, const int bMode,ArmorData &data,Point2f offset);//, ArmorData &data, point2f offset);
	void Detect(const Mat src_img, int Mode, Point2f &pt, int &status);//,Point2f &pt,int status)
	float Distance(const Point2f pt1, const Point2f pt2);
	bool CircleFit(const vector<Point2f> &pt, Point2f &R_center);
	bool Predict(const ArmorData data, Point2f & preCenter, int pMode);
	bool MakeRectSafe(const Rect rect, const Size size);
	bool SetImage(const Mat src, Mat &dect_src, Point2f &offset);
	bool Change2Angle(const int quadrant, const float angle, float &tran_angle);
	void GetArmorRect(const RotatedRect & rect);
	void IsCut(const ArmorData new_data,int &status);

	bool GetFrameClock();
	//bool GetTargetFinalArmor();
	RotatedRect finalArmor2Angle;
	//void CountFrameNum();
	//bool InitCamera();
};
#endif
