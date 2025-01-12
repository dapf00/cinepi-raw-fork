#ifndef SHARED_CONTEXT_STRUCTS_HPP
#define SHARED_CONTEXT_STRUCTS_HPP

#include <semaphore.h>

struct CinePiMetadataStruct
{
	int ae_constraint_mode;
	bool ae_enable;
	int ae_exposure_mode;
	int ae_flicker_detected;
	int ae_flicker_mode;
	int ae_flicker_period;
	bool ae_locked;
	int ae_metering_mode;
	float analogue_gain;
	bool awb_enable;
	bool awb_locked;
	int awb_mode;
	float brightness;
	std::array<float, 2> color_gains;
	int color_temperature;
	float contrast;
	float digital_gain;
	int exposure_time;
	float exposure_value;
	int frame_duration;
	std::array<int, 2> frame_duration_limits;
	float gamma;
	float lens_position;
	int lux;
	float saturation;
	float sharpness;
};

struct CinePiInfoStruct
{
	bool is_recording;
	uint64_t ts;
	int fd_raw;
	int fd_isp;
	int fd_lores;
	size_t raw_length;
	size_t isp_length;
	size_t lores_length;
	int procid;
	uint64_t frame;
	unsigned int sequence;
	float framerate;
	int width;
	int height;
	int compression;
	int thumbnail;
	int thumbnail_size;
	std::array<int, 4> raw_crop;
};
struct CinePiCommandStruct
{
	/* Controls exposed by Libcamera */
	std::optional<int> ae_constraint_mode;
	std::optional<bool> ae_enable;
	std::optional<int> ae_exposure_mode;
	std::optional<int> ae_flicker_mode;
	std::optional<int> ae_flicker_period;
	std::optional<int> ae_metering_mode;
	std::optional<float> analogue_gain;
	std::optional<bool> awb_enable;
	std::optional<int> awb_mode;
	std::optional<float> brightness;
	std::optional<std::array<float, 2>> color_gains;
	std::optional<int> color_temperature;
	std::optional<float> contrast;
	std::optional<int> exposure_time;
	std::optional<float> exposure_value;
	std::optional<std::array<int64_t, 2>> frame_duration_limits;
	std::optional<float> gamma;
	std::optional<float> saturation;
	std::optional<float> sharpness;
	/* custom cinepi raw controls */
	std::optional<bool> set_recording;
	std::optional<bool> reinitialize;
	std::optional<int> width;
	std::optional<int> height;
	std::optional<int> compression;
	std::optional<int> thumbnail;
	std::optional<int> thumbnail_size;
	std::optional<std::array<int, 4>> raw_crop;
};
struct SharedMemoryBuffer
{
	sem_t sem;
	CinePiInfoStruct info;
	CinePiMetadataStruct metadata;
	StreamInfo raw;
	StreamInfo isp;
	StreamInfo lores;
	CinePiCommandStruct commands;
};
#endif
