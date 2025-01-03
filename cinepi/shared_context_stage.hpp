#pragma once
#include "cinepi_controller.hpp"
#include "cinepi_recorder.hpp"

#include <semaphore.h>
//#include "core/rpicam_app.hpp"
constexpr auto PROJECT_ID = "/CINE";

struct SharedMetadata
{
	float exposure_time;
	float analogue_gain;
	float digital_gain;
	unsigned int color_temp;
	int64_t sensor_timestamp;
	std::array<float, 2> color_gains;
	float focus;
	float fps;
	bool aelock;
	float lens_position;
	int af_state;
};

struct CommandBuffer
{
	std::optional<bool> recording;
	std::optional<bool> auto_white_balance;
	std::optional<bool> reinitialize;
	std::optional<int> width;
	std::optional<int> height;
	std::optional<int> compression;
	std::optional<int> thumbnail;
	std::optional<int> thumbnail_size;
	std::optional<std::array<int, 4>> raw_crop;
	std::optional<float> framerate;
	std::optional<float> iso;
	std::optional<float> shutter_speed;
	std::optional<std::array<float, 2>> color_gains;
};
struct SharedMemoryBuffer
{
	sem_t sem;
	int fd_raw;
	int fd_isp;
	int fd_lores;
	StreamInfo raw;
	StreamInfo isp;
	StreamInfo lores;
	size_t raw_length;
	size_t isp_length;
	size_t lores_length;
	int procid;
	uint64_t frame;
	uint64_t ts;
	SharedMetadata metadata;
	unsigned int sequence;
	float framerate;

	bool is_recording;
	bool auto_white_balance;
	int width;
	int height;
	int compression;
	int thumbnail;
	int thumbnail_size;
	std::array<int, 4> raw_crop;
	CommandBuffer commands;
	bool command_done;
};

using Stream = libcamera::Stream;

class SharedContext
{
public:
	SharedContext(CinePIRecorder *app);
	~SharedContext();

	//	char const *Name() const override;
	//void Read(boost::property_tree::ptree const &params) override;
	//void Configure() override;
	void process(CompletedRequestPtr &completed_request, CinePIController &controller);
	//void Teardown() override;

private:
	void readCommands(CinePIController &controller);
	void setInfo(CompletedRequestPtr &completed_request, CinePIController &controller);
	std::shared_ptr<spdlog::logger> console;

	void parseMetaData(libcamera::ControlList &ctrls);

	SharedMemoryBuffer *shared_data;
	key_t segment_key;

	bool running_ = true;

	uint64_t getTs();
	CinePIRecorder *_app;
	RawOptions *_options;
};
