#pragma once

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <time.h>

#include "core/logging.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

#include <mutex>
#include <queue>
#include <thread>

#define CONTROL_TRIGGER_RECORD "rec"

#define CONTROL_KEY_RECORD "is_recording"
#define CONTROL_KEY_ISO "iso"
#define CONTROL_KEY_WB "awb"
#define CONTROL_KEY_COLORGAINS "cg_rb"
#define CONTROL_KEY_SHUTTER_ANGLE "shutter_a"
#define CONTROL_KEY_SHUTTER_SPEED "shutter_s"

#define CONTROL_KEY_FRAMERATE "fps"
#define CONTROL_KEY_WIDTH "width"
#define CONTROL_KEY_HEIGHT "height"
#define CONTROL_KEY_MODE "mode"
#define CONTROL_KEY_COMPRESSION "compress"
#define CONTROL_KEY_THUMBNAIL "thumbnail"
#define CONTROL_KEY_THUMBNAIL_SIZE "thumbnail_size"

#define CONTROL_KEY_RAW_CROP "raw_crop"

#define CONTROL_KEY_CAMERAINIT "cam_init"

class CinePIState
{
public:
	CinePIState() : is_recording_(false), clip_number_(0), still_number_(0) {};
	~CinePIState() {};

	void setRecording(bool state) { is_recording_ = state; }

	bool isRecording() { return is_recording_; }

	unsigned int getClipNumber()
	{
		auto console = spdlog::stdout_color_mt("get clip no");
		console->info("in clip no");
		return clip_number_;
	}

protected:
	float framerate_;
	bool is_recording_;
	unsigned int iso_;
	unsigned int awb_;
	float shutter_speed_;
	float shutter_angle_;
	unsigned int color_temp_;
	float cg_rb_[2];

	uint16_t width_;
	uint16_t height_;
	int mode_;
	int compression_;

	int thumbnail_;
	int thumbnail_size_;

	unsigned int clip_number_;
	unsigned int still_number_;
};
