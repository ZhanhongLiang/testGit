/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-10-23 22:35:25
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-13 21:14:05
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_
#define WRITEXML_H_
#include <iostream>
#include <string>
#include "../header/Header.h"
#include "../rune/Rune.h"
#include "AngleSolver.hpp"
#include "opencv2/core/core.hpp"

#define ARMOR_MODE 0
#define RUNE_MODE 1

/**
 * @brief: file的xml文件
 * @param const cv::RotatedRect & rect
 * 装甲板像素点，预测点preCenter还是真正的armorCenter
 * @param std::vector<cv::Point2f> & target2d
 * @param const cv::Point2f & offset = cv::Point2f(0, 0)
 * @return:
 * @author: Chinwong_Leung
 */
class Settings {
 public:
  //文件路径初始化
  Settings(const std::string& filename) {
    FileStorage settingsReadFile(filename, FileStorage::READ);
    // FileStorage settingsWriteFile(filename, FileStorage::WRITE);
    FileRead(settingsReadFile);
    // FileWrite(settingsWriteFile);
    // settingsWriteFile.release();
    settingsReadFile.release();
  }

  //~Settings();
  void FileRead(
      const FileStorage& fs) {  // fs["rune_height"] >> runeParam.rune_height;
    // fs["rune_width"] >> runeParam.rune_width;

    fs["intrinsic_file_480"] >> intrinsic_file_480;
    fs["intrinsic_file_720"] >> intrinsic_file_720;

    fs["bullet_speed"] >> bullet_speed;
    fs["scale_z"] >> scale_z;
  }

  void FileWrite(FileStorage& fs) const {
    cvWriteComment(*fs, "\nParameter for Camera", 0);
    fs << "intrinsic_file_480" << intrinsic_file_480;
    fs << "intrinsic_file_720" << intrinsic_file_720;
    // fs << "exposure_time" << exposure_time;

    // for system mode
    cvWriteComment(*fs,
                   "\nParameter for Vision System Mode, 0(default) means for "
                   "armor detection mode, 1 means for rune system mode",
                   0);
    fs << "mode" << mode;

    cvWriteComment(*fs, "\nBullet speed (m/s)", 0);
    fs << "bullet_speed" << bullet_speed;

    cvWriteComment(*fs, "\nScale factor of Z", 0);
    fs << "scale_z" << scale_z;
  }

  struct RuneParam {
    float rune_height;
    float rune_width;

    RuneParam() {
      rune_height = 122;
      rune_width = 233;
    }
  };

  struct OtherParam {
    int angle_pitch;
    int angle_yaw;
    OtherParam() {
      angle_pitch = 0;
      angle_yaw = 0;
    };

   public:
    RuneParam runeParam;
    int mode = 0;
    std::string intrinsic_file_480;
    std::string intrinsic_file_720;
    double bullet_speed = 22.0;
    double scale_z = 1.0;
    // double scale_z_480 = 1.0;
    // std::string &filename;
  };
#ifndef WRITEXML_H_
  int main() {
    FileStorage settingsReadFile(
        "D:/gitrepo/Robobigsymbol/src/rune/rune/Rune.xml", FileStorage::WRITE);
    settingsReadFile << "rune_height" << rune_param.rune_height << "rune_width"
                     << rune_param.rune_width;
    settingsReadFile.release();
    return 0;
  }
#endif  // !WRITEXML_H

#endif  // !SETTINGS_H
