/*
 * @Description: 
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-10-23 22:35:25
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-10-28 21:12:07
 */

#ifndef SETTINGS_H
#define SETTINGS_H
#define WRITEXML_H
#include "opencv2/core/core.hpp"
#include <string>
//#include "AngleSolver.hpp"
#include<iostream>
#include "../rune/Rune.h"
#include"../header/Header.h"

#define ARMOR_MODE 0
#define RUNE_MODE 1

/**
* @brief: file的xml文件
* @param const cv::RotatedRect & rect 装甲板像素点，预测点preCenter还是真正的armorCenter
* @param std::vector<cv::Point2f> & target2d
* @param const cv::Point2f & offset = cv::Point2f(0, 0)
* @return:
* @author: Chinwong_Leung
*/
class Settings
{
public:
	//文件路径初始化
	Settings(const std::string & filename)
	{
		FileStorage settingsReadFile(filename, FileStorage::READ);
		//FileStorage settingsWriteFile(filename, FileStorage::WRITE);
		FileRead(settingsReadFile);
		//FileWrite(settingsWriteFile);
		//settingsWriteFile.release();
		settingsReadFile.release();
	}

	//~Settings();
	void FileRead(const FileStorage & fs)
	{
		fs["rune_height"] >> runeParam.rune_height;
		fs["rune_width"] >> runeParam.rune_width;
		fs["intrinsic_file_480"] >> intrinsic_file_480;
	}

	void FileWrite( FileStorage& fs) const
	{
		/*fs << "rune_height" << runeParam.rune_height
		   << "rune_width" << runeParam.rune_width;*/
		cvWriteComment(*fs, "\nFor Debug Image", 0);
		//fs << "show_image" << show_image;
		//fs << "save_result" << save_result;

		// for rune system
		cvWriteComment(*fs, "\nParameter for Rune System", 0);
		//fs << "sudoku_cell_width" << runeParam.sudoku_cell_width
			//<< "sudoku_cell_height" << rune.sudoku_cell_height
			//<< "shoot_time_gap" << rune.shoot_time_gap
			//<< "shoot_filter_size" << rune.shoot_filter_size;
		fs << "rune_width" << runeParam.rune_width
			<< "rune_height" << runeParam.rune_height;

		// for armor system
		/*cvWriteComment(*fs, "\nParameter for Armor Detection System", 0);
		fs << "min_light_gray" << armor.min_light_gray
			<< "min_light_height" << armor.min_light_height
			<< "avg_contrast_threshold" << armor.avg_contrast_threshold
			<< "light_slope_offset" << armor.light_slope_offset
			<< "max_light_delta_h" << armor.max_light_delta_h
			<< "min_light_delta_h" << armor.min_light_delta_h
			<< "max_light_delta_v" << armor.max_light_delta_v
			<< "max_light_delta_angle" << armor.max_light_delta_angle
			<< "avg_board_gray_threshold" << armor.avg_board_gray_threshold
			<< "avg_board_grad_threshold" << armor.avg_board_grad_threshold
			<< "grad_threshold" << armor.grad_threshold
			<< "br_threshold" << armor.br_threshold;*/

		// for enemy color
		/*cvWriteComment(*fs, "\nParameter for Enemy Color, 0(default) means for red, otherwise blue", 0);
		fs << "enemy_color" << armor.enemy_color;

		cvWriteComment(*fs, "\nMinimum / Maximun distance (cm) of detection", 0);*/
		//fs << "min_detect_distance" << min_detect_distance;
		//fs << "max_detect_distance" << max_detect_distance;

		// for armor template
		cvWriteComment(*fs, "\nParameter for Template", 0);
		//fs << "template_image_file" << template_image_file;
		fs << "small_template_image_file" << std::string("small_template_image_file");

		// for camerar
		cvWriteComment(*fs, "\nParameter for Camera", 0);
		fs << "intrinsic_file_480" << intrinsic_file_480;
		fs << "intrinsic_file_720" << intrinsic_file_720;
		//fs << "exposure_time" << exposure_time;

		// for system mode
		cvWriteComment(*fs, "\nParameter for Vision System Mode, 0(default) means for armor detection mode, 1 means for rune system mode", 0);
		fs << "mode" << mode;

		cvWriteComment(*fs, "\nBullet speed (m/s)", 0);
		fs << "bullet_speed" << bullet_speed;

		cvWriteComment(*fs, "\nScale factor of Z", 0);
		fs << "scale_z" << scale_z;
	}
	struct RuneParam
	{
		float rune_height;
		float rune_width;

		RuneParam()
		{
			rune_height = 122;
			rune_width = 233;
		}
	};

public:
	RuneParam runeParam;
	int mode = 0;
	std::string intrinsic_file_480;
	std::string intrinsic_file_720;
	double bullet_speed = 122.0;
	double scale_z = 1.0;
	double scale_z_480 = 1.0;
	//std::string &filename;
};
#ifndef WRITEXML_H
int main()
{
	FileStorage settingsReadFile("D:/gitrepo/Robobigsymbol/src/rune/rune/Rune.xml", FileStorage::WRITE);
	settingsReadFile << "rune_height" << rune_param.rune_height
		<< "rune_width" << rune_param.rune_width;
	settingsReadFile.release();
	return 0;
}
#endif // !WRITEXML_H

#endif // !SETTINGS_H
