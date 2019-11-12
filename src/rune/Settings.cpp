/*
 * @Description:
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-11-12 18:05:32
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-11-12 19:35:21
 */
#include "include/rune/Settings.h"

Settings::Settings(const std::string& filename) {
  FileStorage settingsReadFile(filename, FileStorage::READ);
  // FileStorage settingsWriteFile(filename, FileStorage::WRITE);
  FileRead(settingsReadFile);
  // FileWrite(settingsWriteFile);
  // settingsWriteFile.release();
  settingsReadFile.release();
}

/**
 * @brief:
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */

void Settings::FileRead(const FileStorage& fs) {
  // fs["rune_height"] >> runeParam.rune_height;
  // fs["rune_width"] >> runeParam.rune_width;

  fs["intrinsic_file_480"] >> intrinsic_file_480;
  fs["intrinsic_file_720"] >> intrinsic_file_720;

  fs["bullet_speed"] >> bullet_speed;
  fs["scale_z"] >> scale_z;
}

/**
 * @brief:
 * @param {type}
 * @return:
 * @author: Chinwong_Leung
 */
void Settings::FileWrite(FileStorage& fs) const {
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
