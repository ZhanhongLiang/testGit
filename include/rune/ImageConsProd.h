/*
 * @Description: 
 * @Author: Chinwong_Leung
 * @Github: https://github.com/ZhanhongLiang
 * @Date: 2019-10-23 22:32:37
 * @LastEditors: Chinwong_Leung
 * @LastEditTime: 2019-10-27 22:11:45
 */
#pragma once
#include "../header/Header.h"
#include "./Rune.h"
#include <iostream>
#include "./Settings.hpp"
#include "./AngleSolver.hpp"

#define EXPOSURE_RUNE 2000
#define RUNE_WIDTH  888
#define RUNE_HEIGHT 1000

#define BUFFER_SIZE 1

class ImageConsProd
{
public:
	ImageConsProd(Settings * _settings, int fd_car);
	~ImageConsProd();

	//ImageData data[BUFFER_SIZE];
	void ImageProducer();
	void ImageConsumer();
	Settings * settings;
	int fd2car;
	void test();
};
