#include "Armor_Detector.h"
#include "TestAngleSolver.h"
//#include "SerialPort.h"

#define BUFFER_SIZE 1
Mat test;
int teamSelect = RED;
double result;

volatile unsigned int prdIdx;
volatile unsigned int csmIdx;
struct ImageData {
	Mat img;
	unsigned int frame;
};

ImageData imgData[BUFFER_SIZE];

#ifdef Windows
	#define device 0
	VideoCapture capture_m(device);
#else
	#include "RMVideoCapture.hpp"
	RMVideoCapture capture_m("/dev/video0", 3);
#endif // Windows

Armor_Detector::Armor_Detector()  //�ڴ˴���ʼ��
{
	lastTargetarmor.size.height = 0;
	lastTargetarmor.size.width = 0;
	lastTargetarmor.center.x = 0;
	lastTargetarmor.center.y = 0;
	
	targetArmor.size.height = 0;
	targetArmor.size.width = 0;
	targetArmor.center.x = 0;
	targetArmor.center.y = 0;
}

bool Armor_Detector::InitCamera()
{
#ifdef Windows
	capture_m.open(device);
	if (!capture_m.isOpened())
	{
		printf("������ͷ��ʧ�ܣ�\n");
		return false;
	}
	else
	{
		//���ø�����ͷ�ֱ���
#ifdef Linux_EXPOSURE
		capture_m.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
		capture_m.set(CV_CAP_PROP_EXPOSURE, (0.01));//�ع� 50  -7
#else
		capture_m.set(CV_CAP_PROP_EXPOSURE, (-6));//�ع� 50  -7
#endif // Linux
		capture_m.set(CV_CAP_PROP_FRAME_WIDTH, 720);
		capture_m.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
		return true;
	}
#else
	capture_m.setVideoFormat(720, 480, 1);
	capture_m.setExposureTime(0, 64);//settings->exposure_time);
	capture_m.startStream();
	capture_m.info();
	return true;
#endif // Windows
}

bool armorlightPosdiff(RotatedRect light1, RotatedRect light2, RotatedRect armor)
{
	if ((light1.center.y + (light1.size.height * 0.5)) <= (armor.center.y + (armor.size.height * 0.5)))
		if ((light1.center.y - (light1.size.height * 0.5)) >= (armor.center.y - (armor.size.height * 0.5)))
			if ((light2.center.y + (light2.size.height * 0.5)) <= (armor.center.y + (armor.size.height * 0.5)))
				if ((light2.center.y - (light2.size.height * 0.5)) >= (armor.center.y - (armor.size.height * 0.5)))
					return true;
	return false;
}

void adjustRect(cv::RotatedRect &rect)
{
	if (rect.size.width > rect.size.height)
	{
		auto temp = rect.size.height;
		rect.size.height = rect.size.width;
		rect.size.width = temp;
		rect.angle += 90;
		if (rect.angle > 180)
			rect.angle -= 180;
	}

	if (rect.angle > 90)
		rect.angle -= 90;
	else if (rect.angle < -90)
		rect.angle += 90;   // ������Ƕ�Ϊ��, �ҵ����Ƕ�Ϊ��
}

bool Armor_Detector::DetectorPretreatment(Mat src, vector<cv::RotatedRect> &lights)
{
#ifdef DEBUG
	Mat show_lights_before_filter_ = cv::Mat::zeros(src.size(), CV_8UC3);
	Mat show_lights_after_filter_ = cv::Mat::zeros(src.size(), CV_8UC3);
	Mat show_armors_befor_filter_ = src.clone();
	Mat show_armors_after_filter_ = src.clone();
#endif // DEBUG

	cv::Mat color_light;
	std::vector<cv::Mat> bgr_channel;
	cv::split(src, bgr_channel);
	Mat gray_img;

	lights.clear();
	if (teamSelect == RED)
		cv::subtract(bgr_channel[2], bgr_channel[1], color_light);
	else
		cv::subtract(bgr_channel[0], bgr_channel[1], color_light);

	cv::Mat binary_brightness_img; // ���ȶ�ֵ��
	cv::Mat binary_color_img;      // ��ɫ��ֵ��
	cv::Mat binary_light_img;      // &
	Point2f pts[4];

	cv::cvtColor(src, gray_img, cv::ColorConversionCodes::COLOR_BGR2GRAY);
	// cv::adaptiveThreshold(gray_img, binary_brightness_img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,7,5);
	// ������������������֮һ
	cv::threshold(gray_img, binary_brightness_img, light_threshold_val, 255, CV_THRESH_BINARY);  //200

	float thresh;
	if (teamSelect == BLUE) // ����Կ����ƶ���Ȼ��Ӱ��
		thresh = blue_color_diff; //70
	else
		thresh = red_color_diff;  //50 40
	//  cv::adaptiveThreshold(color_light, binary_color_img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 5);
	cv::threshold(color_light, binary_color_img, thresh, 255, CV_THRESH_BINARY);
	// �������̬ѧ��Ҫ����һ��,������װ�װ�����ƶ�ʱ
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(binary_color_img, binary_color_img, element, cv::Point(-1, -1), 1);
	//cv::morphologyEx(binary_color_img,binary_color_img, MORPH_OPEN, element);
	binary_light_img = binary_color_img & binary_brightness_img;
	//cv::dilate(binary_light_img, binary_light_img, element, cv::Point(-1, -1), 1);
	//cv::dilate(binary_brightness_img, binary_brightness_img, element, cv::Point(-1, -1), 1);

	std::vector<std::vector<cv::Point>> contours_light;
	cv::findContours(binary_light_img, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<cv::Point>> contours_brightness;
	cv::findContours(binary_brightness_img, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
	if (contours_light.size() > 200 || contours_brightness.size() > 200) return false;  //װ�װ�����ࣨ�쳣������

	lights.reserve(contours_brightness.size());
	// TODO: To be optimized
	std::vector<int> is_processes(contours_brightness.size());
	cv::RotatedRect single_light, s_fitEllipse, s_minAreaRect;
	for (unsigned int i = 0; i < contours_light.size(); ++i) {
		for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
			if (!is_processes[j]) {
				if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) {
					single_light = cv::minAreaRect(contours_brightness[j]);
					lights.push_back(single_light);
					is_processes[j] = true;
					break;
				}
			}
		}
	}

#ifdef DEBUG
	cv::imshow("gray_img", gray_img);
	cv::imshow("binary_color_img", binary_color_img);
	cv::imshow("binary_light_img", binary_light_img);
	cv::imshow("binary_brightness_img", binary_brightness_img);
	//cv::imshow("lights_before_filter", show_lights_before_filter_);
	printf("%d\n", lights.size());
	imshow("res", test);
#endif // DEBUG
	return true;
}

void Armor_Detector::RIODetector(vector<cv::RotatedRect> &lights, vector<RotatedRect>& ArmorBuild)
{
	vector<RotatedRect> ss;
	Point2f pts[4];
	//ArmorBuild.clear();
	//cout << "0" << endl;
//#pragma omp parallel for
	for (int i = 0; i < lights.size(); i++)   //����ʶ��
	{
		adjustRect(lights[i]);
		//cout << lights[i].angle << endl;
		if (lights[i].size.area() > 50.0 && lights[i].size.area() < 5000) {    //����������� ���� �ϴ�  
			//cout << "1" << endl;
			if ((lights[i].size.width / lights[i].size.height < 0.65)) {
				//cout << "2" << endl;
				//if (lights[i].size.height > 1 && lights[i].size.width > 1) {
					//cout << "3" << endl;
				if (abs(lights[i].angle) < 35)
				{
#ifdef DEBUG1
						//cout << "NORMAL" << endl;
						lights[i].points(pts);
						for (int j = 0; j <= 3; j++)
							line(test, pts[j], pts[(j + 1) % 4], Scalar(0, 255, 0), 2);
#endif // DEBUG
					ss.push_back(lights[i]);
					continue;
				}
				//}
			}
		}

		if (lights[i].size.area() > 50.0 && lights[i].size.area() < 5000) {    //������״ ���� 
			//cout << "11" << endl;
			if ((lights[i].size.width / lights[i].size.height < 0.75)) {
				//cout << "22" << endl;
				//if (lights[i].size.height > 1 && lights[i].size.width > 1) {
					//cout << "33" << endl;
					//cout << lights[i].angle << endl;
				if (abs(lights[i].angle) < 40) {
					//cout << "44" << endl;
					//cout << lights[i].size.height << endl;
					if (lights[i].size.height > 10)
					{
#ifdef DEBUG1
							//cout << "FAST" << endl;
							lights[i].points(pts);
							for (int j = 0; j <= 3; j++)
								line(test, pts[j], pts[(j + 1) % 4], Scalar(0, 255, 0), 2);
#endif // DEBUG
						ss.push_back(lights[i]);
						continue;
					}
				}
				//}
			}
		}

		if (lights[i].size.area() > 10.0 && lights[i].size.area() < 5000) {     //���� ��С  ���ֵ��С�� 10000
			//cout << "111" << endl;
			//if (lights[i].size.height > 1 && lights[i].size.width > 1) {
				//cout << "222" << endl;
			if (abs(lights[i].angle) < 20)
			{
#ifdef DEBUG1
					//cout << "SMALL" << endl;
					lights[i].points(pts);
					for (int j = 0; j <= 3; j++)
						line(test, pts[j], pts[(j + 1) % 4], Scalar(0, 255, 0), 2);
#endif // DEBUG
				ss.push_back(lights[i]);
				continue;
			}
			//}
		}
	}

	//װ�װ�ʶ��
	RotatedRect ArmorRect;
	Point2f tmp[4];
	int width, height, maxHeight;
	float ArmorRatio, LightRatio1, LightRatio2, Y_diff, X_diff, ArmorArea, armor_light_angle_diff, lightYdiff, armor_lightCenterdiff;

	if (ss.size() >= 2)
	{
//#pragma omp parallel for
		for (int i = 0; i < ss.size(); i++) {
			for (int j = i + 1; j < ss.size(); j++) {
				LightRatio1 = ss[i].size.width / ss[i].size.height;
				LightRatio2 = ss[j].size.width / ss[j].size.height;
				Y_diff = abs(ss[i].size.height - ss[j].size.height) / ((ss[i].size.height > ss[j].size.height) ? ss[i].size.height : ss[j].size.height);
				lightYdiff = abs(ss[i].center.y - ss[j].center.y) / ((ss[i].size.height > ss[j].size.height) ? ss[i].size.height : ss[j].size.height);  //--------------
				X_diff = abs(ss[i].size.width - ss[j].size.width) / ((ss[i].size.width > ss[j].size.width) ? ss[i].size.width : ss[j].size.width);
				//cout << LightRatio1 << "   " << LightRatio2 << "    " << endl;
				if (LightRatio1 < 0.8 && LightRatio1 > 0.15) {          //�����ƶ�
					//cout << "1" << endl;
					if (LightRatio2 < 0.8 && LightRatio2 > 0.15) {
						//cout << "2" << endl;
						//cout << ss[i].angle << endl;
						if (abs(ss[i].angle) < 30) {   //----------------------------------------------------
							//cout << "3" << endl;
							if (abs(ss[i].angle - ss[j].angle) < 10) {
								//cout << "4" << endl;
								if (Y_diff < 0.40 && X_diff < 0.5) {  //0.3~0.5
									//cout << "5" << endl;
									ArmorRect.center.x = (ss[i].center.x + ss[j].center.x) / 2;
									ArmorRect.center.y = (ss[i].center.y + ss[j].center.y) / 2;
									ArmorRect.angle = (ss[i].angle + ss[j].angle) / 2;
									if (ArmorRect.angle > 100) ArmorRect.angle = 180 - ArmorRect.angle;
									width = (abs(ss[i].center.x - ss[j].center.x) + ss[i].size.width);  
									maxHeight = (ss[i].size.height > ss[j].size.height) ? ss[i].size.height : ss[j].size.height;
									height = (ss[i].size.height + ss[j].size.height) / 2/* + maxHeight*/;
									ArmorArea = height * width;
									if (width < height) {
										ArmorRect.size.width = height;
										ArmorRect.size.height = width;
									}
									if (width > height) {
										ArmorRect.size.width = width;
										ArmorRect.size.height = height;
									}
									ArmorRatio = width / (height * 1.0);
									if (/*armorlightPosdiff(ss[i], ss[j], ArmorRect)*/1) {
										//cout << "6" << endl;
										if (ArmorArea > 200) {
											//cout << "7" << endl;
											//cout << ArmorRatio << endl << endl;
											if (ArmorRatio <= 3) {              // �������2.0   
												//cout << "8" << endl;
												if (abs(ArmorRect.angle) < 20) {         // �������
													//cout << "OK1" << endl;
													ArmorBuild.push_back(ArmorRect);
#ifdef DEBUG1
													ArmorRect.points(tmp);
													for (int k = 0; k <= 3; k++)
														line(test, tmp[k], tmp[(k + 1) % 4], Scalar(0, 0, 255), 2);
#endif // DEBUG
													//continue;
												}
											}
											else if (ArmorRatio > 3 && ArmorRatio < 8) {               // �������2.0    
												//cout << "81" << endl;
												//cout << ArmorRect.angle << endl << endl << endl;
												if (abs(ArmorRect.angle) < 20) {         // �������
													//cout << "ok11" << endl;
													ArmorBuild.push_back(ArmorRect);
#ifdef DEBUG1
													ArmorRect.points(tmp);
													for (int k = 0; k <= 3; k++)
														line(test, tmp[k], tmp[(k + 1) % 4], Scalar(0, 0, 255), 2);
#endif // DEBUG
													//continue;
												}
											}
										}
									}
								}
							}
						}
					}
				}

				if (LightRatio1 < 0.8 && LightRatio1 > 0.15) {         //�����ƶ�
					//cout << "11" << endl;
					if (LightRatio2 < 0.8 && LightRatio2 > 0.15) {
						//cout << "22" << endl;
						if (abs(ss[i].angle) < 30) {  //-------------------------
							//cout << "33" << endl;
							if (abs(ss[i].angle - ss[j].angle) < 10) {
								//cout << "44" << endl;
								if (Y_diff < 0.40 && X_diff < 0.5) {
									//cout << "55" << endl;
									ArmorRect.center.x = (ss[i].center.x + ss[j].center.x) / 2;
									ArmorRect.center.y = (ss[i].center.y + ss[j].center.y) / 2;
									ArmorRect.angle = (ss[i].angle + ss[j].angle) / 2;
									if (ArmorRect.angle > 100) ArmorRect.angle = 180 - ArmorRect.angle;
									width = (abs(ss[i].center.x - ss[j].center.x) + ss[i].size.width);
									maxHeight = (ss[i].size.height > ss[j].size.height) ? ss[i].size.height : ss[j].size.height;
									height = (ss[i].size.height + ss[j].size.height) / 2/* + maxHeight*/;
									ArmorArea = height * width;
									armor_light_angle_diff = abs(ArmorRect.angle - ss[i].angle) + abs(ArmorRect.angle - ss[j].angle);
									if (width < height) {
										ArmorRect.size.width = height;
										ArmorRect.size.height = width;
									}
									if (width > height) {
										ArmorRect.size.width = width;
										ArmorRect.size.height = height;
									}
									ArmorRatio = width / (height * 1.0);
									if (/*armorlightPosdiff(ss[i], ss[j], ArmorRect)*/1) {
										//cout << "66" << endl;
										if (ArmorArea > 200) {
											//cout << "77" << endl;
											//cout << ArmorRatio << endl << endl;
											if (ArmorRatio <= 3 && ArmorRatio >= 1.0) {               // �������
												//cout << "88" << endl;
												if (armor_light_angle_diff < 20) {
													//cout << "99" << endl;
													if (abs(ArmorRect.angle) < 20) {         // �������
														//cout << "OKk" << endl;
														ArmorBuild.push_back(ArmorRect);
#ifdef DEBUG1
														ArmorRect.points(tmp);
														for (int k = 0; k <= 3; k++)
															line(test, tmp[k], tmp[(k + 1) % 4], Scalar(0, 0, 255), 2);
#endif // DEBUG
														//continue;
													}
												}
											}
											else if (ArmorRatio > 3 && ArmorRatio < 7) {
												//cout << "881" << endl;
												if (armor_light_angle_diff < 20) {
													//cout << "991" << endl;
													//cout << ArmorRect.angle << endl << endl << endl;
													if (abs(ArmorRect.angle) < 20) {         // �������
														//cout << "OKk1" << endl;
														ArmorBuild.push_back(ArmorRect);
#ifdef DEBUG1
														ArmorRect.points(tmp);
														for (int k = 0; k <= 3; k++)
															line(test, tmp[k], tmp[(k + 1) % 4], Scalar(0, 0, 255), 2);
#endif // DEBUG	
														//continue;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
#ifdef DEBUG1
	imshow("test", test);
	waitKey(1);
#endif // DEBUG
}

int pointSort(Point2f vertices[4], Point2f &min, Point2f &max, int *width, int *height)
{
	Point2f tMin = vertices[0];
	Point2f tMax = vertices[0];
	for (int i = 1; i < 4; i++) {
		if (vertices[i].x > max.x) tMax.x = vertices[i].x;
		if (vertices[i].y > max.y) tMax.y = vertices[i].y;
		if (vertices[i].x < min.x) tMin.x = vertices[i].x;
		if (vertices[i].y < min.y) tMin.y = vertices[i].y;
	}
	*width = max.x - min.x;
	*height = max.y - min.y;
	if (min.x < 0) min.x = 0;
	if (min.y < 0) min.y = 0;
	if (width <= 0 || height <= 0) {
		return 0;
	}
	max.x = tMax.x;
	max.y = tMax.y;
	min.x = tMin.x;
	min.x = tMin.x;

	return 1;
}

void Armor_Detector::targetFilter(RotatedRect targetArmor, RotatedRect lastTargetarmor)  //װ�װ壨�������˲�
{
	if (targetArmor.size.area() <= 100) {
		targetArmor = lastTargetarmor;
		targetGet = 0;
		return;
	}
	if (abs(lastTargetarmor.center.x - targetArmor.center.x) < (lastTargetarmor.size.width + targetArmor.size.width) / (2 * 5)) { //С��װ�װ���Ⱥ͸߶ȵ� 1/4
		if (abs(lastTargetarmor.center.y - targetArmor.center.y) < (lastTargetarmor.size.height + targetArmor.size.height) / (2 * 5)) {
			targetArmor = lastTargetarmor;
			//cout << "OK111" << endl;
		}
	}
	lastTargetarmor = targetArmor;
	targetGet = 1;
	//cout << "OK" << endl;
	return ;
}

void Armor_Detector::rioTactics(vector<RotatedRect> finalArmorBuild)
{
	Point2f tmp[4], tmpNext[4], min, max, minNext, maxNext;
	int width, height, widthNext, heightNext, flag = 0, count = 0;
	double maxArea = 0;
	if (finalArmorBuild.size() <= 0) {
		targetArmor = lastTargetarmor;
		targetGet = 0;
		//cout << "-1-1-1" << endl;
		return;
	}
	//cout << "000" << endl;
	if (finalArmorBuild.size() < 2 && finalArmorBuild.size() > 0) {
		//cout << "11111" << endl;
		targetArmor = finalArmorBuild[0];
		targetFilter(targetArmor, lastTargetarmor);
		return ;
	}
	for (int i = 0; i < finalArmorBuild.size(); i++) {
		if (minRatio < (finalArmorBuild[i].size.width / finalArmorBuild[i].size.height))
			count = i;
		if ((finalArmorBuild[i].size.width / finalArmorBuild[i].size.height) <= 3.0 && (finalArmorBuild[i].size.width / finalArmorBuild[i].size.height) > 0)
			flag += 1;                                                      //2.5
		if ((finalArmorBuild[i].size.width / finalArmorBuild[i].size.height) > 3.0) 
			flag += 2;
	}
	if ((flag / finalArmorBuild.size() == 1 || flag / finalArmorBuild.size() == 2) && flag % finalArmorBuild.size() == 0) {
		//cout << "22222" << endl;
		targetArmor = finalArmorBuild[count];
		targetFilter(targetArmor, lastTargetarmor);
		return ;
	}

	for (int i = 0; i < finalArmorBuild.size(); i++) {
		finalArmorBuild[i].points(tmp);
		pointSort(tmp, min, max, &width, &height);
		for (int j = i + 1; j < finalArmorBuild.size(); j++) {
			finalArmorBuild[j].points(tmpNext);
			pointSort(tmpNext, minNext, maxNext, &width, &height);
			if (max.x - minNext.x >= 0) {
				targetArmor = (finalArmorBuild[i].size.area() < finalArmorBuild[j].size.area()) ? finalArmorBuild[i] : finalArmorBuild[j];
				//cout << "33333" << endl;
				targetFilter(targetArmor, lastTargetarmor);
				return ;
			}
		}
	}
	targetGet = 0;
	return;
}

void Armor_Detector::finalImgshow(RotatedRect ArmorBuild)
{
	Point2f tmp[4];
	ArmorBuild.points(tmp);
	for (int k = 0; k <= 3; k++)
		line(src, tmp[k], tmp[(k + 1) % 4], Scalar(0, 0, 255), 2);
	imshow("finalImgshow", src);
	waitKey(1);
}

Point2f Armor_Detector::dis2Angle(RotatedRect targetArmor)
{
	int armorFlag;
	int armorType;

	int angleFlag;
	int objectype;
	AngleSolverParam param;
	AngleSolver solver;

	//��һ�������׳��쳣
	//solver.setResolution(Size(720, 480));

	param.readFile();
	solver.init(param);
	//solver.showCamTrix();
	Point2f targetAngle = { 0 };

	if (targetArmor.size.width / targetArmor.size.height > 2)
	{
		objectype = 2;
		//solver.setEnemyType(1);
		//solver.showEnermyType();

	}
	else if (targetArmor.size.width / targetArmor.size.height < 2)
	{
		objectype = 1;
		//solver.setEnemyType(0);
		//solver.showEnermyType();

	}
	else
	{
		objectype = 0;
		//cout << "other Armor!!" << endl;
	}

	if (objectype == 1 || objectype == 2)
	{
		solver.getTarget2dPoinstion(targetArmor, solver.point_of_armor, Point(0, 0));
		solver.setTarget1(solver.point_of_armor, objectype);
		//solver.showPoints2dOfArmor();
		angleFlag = solver.solve();

		if (angleFlag != AngleSolver::ANGLE_ERROR)
		{
			solver.compensateOffset();
			solver.angleLimit();
			targetAngle = solver.getAngle();
			//solver.showTvec();
			//solver.showEDistance();
			//yaw��ĽǶ�
			//cout << "yaw:" << targetAngle.x << endl;
			//pitch��Ƕ�
			//cout << "pitch:" << targetAngle.y << endl;
			return targetAngle;
		}
		else
		{
			cout << "no targetangle:" << endl;
		}
	}
	//solver.setTarget(targetArmor.center, 2);
	//solver.getTarget2dPoinstion(targetArmor, solver.point_of_armor, Point(0, 0));


	//solver.setTarget1(solver.point_of_armor, 2);

	//solver.showPoints2dOfArmor();


	//solver.setTarget(targetArmor.center, 2);
	//angleFlag = solver.solve();
}

bool complare(Point2f a, Point2f b)
{
	return a.y < b.y;
}

void Armor_Detector::angleFilter(Point2f angle)
{
	//cout << "11" << endl;
	if (angleFalse == 1) {
		angleFalse = 0;
		angleCount = 0;
		angleZero--;
		if (!angleZero) {
			for (int i = 0; i < 5; i++) {
				angles[i].x = 0;
				angles[i].y = 0;
			}
		}
		return;
	}

	angles[angleCount % 5].x = angle.x;
	angles[angleCount % 5].y = angle.y;
	angleCount++;
	//cout << angles[angleCount % 5].x << "    " << angles[angleCount % 5].y << endl;

	//�˲�
	if (angleCount < 1) return;
	//cout << "1" << endl;
	if (angleCount < 5) {
		sort(angles, angles + angleCount, complare);
		nowAngle.x = angles[angleCount / 2].x;
		nowAngle.y = angles[angleCount / 2].y;
	}
	else {
		sort(angles, angles + 4, complare);
		nowAngle.x = angles[(angleCount / 2) % 5].x;
		nowAngle.y = angles[(angleCount / 2) % 5].y;
	}
	//
	//cout << nowAngle.x << "    " << nowAngle.y << "    " << endl;
	//angleFalse = 0;
	
	return;
}

void Armor_Detector::set2Zero()
{
	ArmorBuild.clear();
	lights.clear();
	targetArmor.size.height = 0;
	targetArmor.size.width = 0;
}

Point2f getKalmanvalue(float x, float y)
{
	float size_x = 640;//cols of side
	float size_y = 480;//rows of side
	float anti_range = 0.5;//**Larger, anti-kalman more radical
	Mat image(size_y, size_x, CV_8UC3);

	Kalman_struct::KalmanFilter kf(x, y);//Differ from cv::KalmanFilter

	Point2f currentPoint(x, y);
	Point2f kalmanPoint(x, y);
	Point2f anti_kalmanPoint(x, y);


#ifdef DEBUG
	while (1)
	{
		double result;
		clock_t start = clock();
#endif // DEBUG


		//image = Scalar(0,0,0);//Clear points before
		currentPoint = Point2f(x, y);
		kalmanPoint = kf.run(x, y);

		if ((currentPoint.x + anti_range * (currentPoint.x - kalmanPoint.x)) <= size_x
			|| (currentPoint.x + anti_range * (currentPoint.x - kalmanPoint.x)) >= 0)//Prevent Anti-kal out of Mat
		{
			if (abs(currentPoint.x - kalmanPoint.x) > 3)//When points are closed, no Anti-kalman to reduce shaking
				anti_kalmanPoint.x = currentPoint.x + anti_range * (currentPoint.x - kalmanPoint.x);
			else
				anti_kalmanPoint.x = currentPoint.x;
		}
		else
		{
			anti_kalmanPoint.x = currentPoint.x;
		}


		if ((currentPoint.y + anti_range * (currentPoint.y - kalmanPoint.y)) <= size_y
			|| (currentPoint.y + anti_range * (currentPoint.y - kalmanPoint.y)) >= 0)//Prevent Anti-kal out of Mat
		{
			if (abs(currentPoint.y - kalmanPoint.y) > 3)//When points are closed, no Anti-kalman to reduce shaking
				anti_kalmanPoint.y = currentPoint.y + anti_range * (currentPoint.y - kalmanPoint.y);
			else
				anti_kalmanPoint.y = currentPoint.y;
		}
		else
		{
			anti_kalmanPoint.y = currentPoint.y;
		}

#ifdef DEBUG
		int color = 0;//gradually varied color
		circle(image, anti_kalmanPoint, 3, Scalar(0, 255, 0 + color), 2);//predicted point with green
		circle(image, currentPoint, 3, Scalar(255, 0, 0 + color), 2);//current position with red

		imshow("Anti-KalmanPoint", image);
		cout << "Current: " << currentPoint << " Kalman: " << kalmanPoint << " Anti-Kalman: " << anti_kalmanPoint << endl;

		x += 75;
		y += 10;
		color += 20;
		waitKey(1000);

		//if(TargetLost_times > 60)  //Initialize Kalman Filter when losing target in a long time
		//{
		//	KalmanFilter();
		//}

		if (color >= 255)
		{
			color = 255;
		}
		if ((x >= size_x) || (y >= size_y) || (x <= 0) || (y <= 0))
		{
			imwrite("Anti-KalmanPoint.jpg", image);
			break;
		}
		clock_t end = clock();
		result = (double)(end - start);
		printf("Use Time:%f\n", result / CLOCKS_PER_SEC);
	}
#endif // DEBUG

	return anti_kalmanPoint;
}

void Armor_Detector::ImageProducer()
{
	while (1)
	{
		while (prdIdx - csmIdx >= BUFFER_SIZE);
		capture_m >> imgData[prdIdx % BUFFER_SIZE].img;
		imgData[prdIdx % BUFFER_SIZE].frame++;
		++prdIdx;
	}
}

void Armor_Detector::ImageConsumer()
{
	Point2f angle;
	int frame_num = 0;
	while (1) 
	{
		set2Zero();  //????????????????????????????
		while (prdIdx - csmIdx == 0);
		imgData[csmIdx % BUFFER_SIZE].img.copyTo(src);
		frame_num = imgData[csmIdx % BUFFER_SIZE].frame;
		++csmIdx;
		clock_t start = clock();
		if (!DetectorPretreatment(src, lights)) continue;  //0.015s
		RIODetector(lights, ArmorBuild);   //0.005s
		
		rioTactics(ArmorBuild);
		if (targetGet)
		{
			//cout << "GET" << endl;
			angle = dis2Angle(targetArmor);
			angleFilter(angle);
			nextAngle = getKalmanvalue(nowAngle.x, nowAngle.y);
			//sleep(0.1);  //-------------------
			// if (sendPY(fd, nextAngle.y, nextAngle.x)){   //------
			// 	cout << "Yaw:" << nextAngle.x << endl;
			// 	cout << "Pitch:" << nextAngle.y << endl;
			// }
		}
		else
		{
			angleFalse = 1;
			angle.x = 0;
			angle.y = 0;
			nextAngle.x = 0;
			nextAngle.y = 0;
			nowAngle.x = 0;
			nowAngle.y = 0;
			//cout << "no armor!! " << endl;
		}
		
		
		
		
		
		clock_t end = clock();
		result = (double)(end - start);
		//printf("Use Time:%f\n", result / CLOCKS_PER_SEC);
		finalImgshow(targetArmor);
		//waitKey(1);
	}
}

#ifdef DEBUG
int main()
{
	Armor_Detector armor;
	Armor_Detector();

	Vec2f last_angle;
	armor.InitCamera();

	while (1)
	{
		RotatedRect rect;

		double angle_x = 0.0, angle_y = 0.0;
		double send_data[3] = { 0 };
		clock_t start = clock();
		armor.set2Zero();  //--------------------
		capture_m >> armor.src;
		//imshow("origin", armor.src);
		test = armor.src.clone();
		if (!armor.DetectorPretreatment(armor.src, armor.lights)) continue;  //0.015s
		armor.RIODetector(armor.lights, armor.ArmorBuild);   //0.005s
		//armor.rioTactics(armor.ArmorBuild);
		//if (armor.targetArmor.size.area() > 0)
		//{
		//	armor.dis2Angle(armor.targetArmor);
		//}
		//else
		//{
		//	//cout << "no armor!! " << endl;
		//}
		//clock_t end = clock();
		//result = (double)(end - start);
		//armor.finalImgshow(armor.targetArmor);
		////printf("Use Time:%f\n", result / CLOCKS_PER_SEC);
		waitKey(1);
	}
	return 0;
}
#endif // DEBUG
