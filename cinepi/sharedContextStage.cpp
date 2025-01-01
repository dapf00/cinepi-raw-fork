#include <mutex>
#include <semaphore.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <libcamera/stream.h>

#include "core/frame_info.hpp"
#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>

//#define PROJECT_ID 0x43494E45 // ASCII for "CINE"
constexpr auto PROJECT_ID = "/CINE";

struct SharedMetadata
{
	float exposure_time;
	float analogue_gain;
	float digital_gain;
	unsigned int colorTemp;
	int64_t ts;
	float colour_gains[2];
	float focus;
	float fps;
	bool aelock;
	float lens_position;
	int af_state;
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
	uint8_t stats[23200];
};

uint64_t getTs()
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return uint64_t(ts.tv_sec * 1000LL + ts.tv_nsec / 1000000);
}

using Stream = libcamera::Stream;

class sharedContextStage : public PostProcessingStage
{
public:
	sharedContextStage(RPiCamApp *app);
	~sharedContextStage();

	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Configure() override;
	bool Process(CompletedRequestPtr &completed_request) override;
	void Teardown() override;

private:
	std::shared_ptr<spdlog::logger> console;

	void parseMetaData(libcamera::ControlList &ctrls);

	int segment_id;
	SharedMemoryBuffer *shared_data;
	key_t segment_key;

	bool running_ = true;
};

#define NAME "sharedContext"

char const *sharedContextStage::Name() const
{
	return NAME;
}

void sharedContextStage::Read(boost::property_tree::ptree const &params)
{
}

sharedContextStage::sharedContextStage(RPiCamApp *app) : PostProcessingStage(app), shared_data(nullptr)
{
	// Constructor initialization if needed.
	console = spdlog::stdout_color_mt("sharedContextStage");

	const int size = sizeof(SharedMemoryBuffer);

	// Generate a unique key for the shared memory segment
	// segment_key = ftok("/tmp", PROJECT_ID);

	// Try to obtain an existing segment or create a new one
	//segment_id = shmget(segment_key, size, IPC_CREAT | S_IRUSR | S_IWUSR);
	int fd = shm_open(PROJECT_ID, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
	if (fd == -1)
	{
		spdlog::get("sharedContextStage")->error("shm_open failed: {}", errno);
	}
	if (ftruncate(fd, size) == -1)
	{
		spdlog::get("sharedContextStage")->error("fruncate failed: {}", errno);
	}

	// Attach the shared memory segment
	shared_data = static_cast<SharedMemoryBuffer *>(mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
	if (shared_data == (void *)-1)
	{
		spdlog::get("sharedContextStage")->error("mmap failed: {}", errno);
	}

	if (close(fd) == -1)
	{
		spdlog::get("sharedContextStage")->error("fclose failed: {}", errno);
	}
	shared_data->fd_raw = -1;
	shared_data->fd_isp = -1;
	shared_data->fd_lores = -1;
	shared_data->frame = 0;
	shared_data->procid = getpid();
	shared_data->ts = getTs();
	sem_init(&shared_data->sem, 1, 1);
	spdlog::get("sharedContextStage")->info("SharedContextStage created");
}

sharedContextStage::~sharedContextStage()
{
	sem_destroy(&shared_data->sem);
	munmap(shared_data, sizeof(SharedMemoryBuffer));
	shm_unlink(PROJECT_ID);
	// 	shmdt(shared_data);
	// 	shmctl(segment_id, IPC_RMID, NULL);
}

void sharedContextStage::Teardown()
{
	sem_destroy(&shared_data->sem);
	munmap(shared_data, sizeof(SharedMemoryBuffer));
	shm_unlink(PROJECT_ID);
	// shmdt(shared_data);
	// shmctl(segment_id, IPC_RMID, NULL);
}

void sharedContextStage::Configure()
{
	shared_data->raw = app_->GetStreamInfo(app_->RawStream());
	shared_data->isp = app_->GetStreamInfo(app_->GetMainStream());
	shared_data->lores = app_->GetStreamInfo(app_->LoresStream());
}

#include <chrono>

bool sharedContextStage::Process(CompletedRequestPtr &completed_request)
{
	if (sem_wait(&shared_data->sem) == -1)
	{
		spdlog::get("sharedContextStage")->error("sem_wait failed: {}", errno);
	}
	shared_data->ts = getTs();

	auto stats = completed_request->metadata.get(libcamera::controls::rpi::PispStatsOutput);
	if (stats.has_value())
	{
		libcamera::Span<const uint8_t> statsSpan = stats.value();
		std::memcpy(shared_data->stats, statsSpan.data(), statsSpan.size());
	};

	{
		shared_data->fd_raw = completed_request->buffers[app_->RawStream()]->planes()[0].fd.get();
		shared_data->fd_isp = completed_request->buffers[app_->GetMainStream()]->planes()[0].fd.get();
		shared_data->raw_length = completed_request->buffers[app_->RawStream()]->planes()[0].length;
		shared_data->isp_length = completed_request->buffers[app_->GetMainStream()]->planes()[0].length;
		shared_data->fd_lores = completed_request->buffers[app_->LoresStream()]->planes()[0].fd.get();
		shared_data->lores_length = completed_request->buffers[app_->LoresStream()]->planes()[0].length;
		shared_data->framerate = completed_request->framerate;
		shared_data->sequence = completed_request->sequence;
		parseMetaData(completed_request->metadata);
	}

	shared_data->frame++;

	if (sem_post(&shared_data->sem) == -1)
	{
		spdlog::get("sharedContextStage")->error("sem_post failed: {}", errno);
	}
	return false;
}

void sharedContextStage::parseMetaData(libcamera::ControlList &ctrls)
{
	auto colorT = ctrls.get(libcamera::controls::ColourTemperature);
	if (colorT)
		shared_data->metadata.colorTemp = *colorT;

	auto sts = ctrls.get(libcamera::controls::SensorTimestamp);
	if (sts)
	{
		shared_data->metadata.ts = (*sts);
	}

	auto exp = ctrls.get(libcamera::controls::ExposureTime);
	if (exp)
		shared_data->metadata.exposure_time = *exp;

	auto ag = ctrls.get(libcamera::controls::AnalogueGain);
	if (ag)
		shared_data->metadata.analogue_gain = *ag;

	auto dg = ctrls.get(libcamera::controls::DigitalGain);
	if (dg)
		shared_data->metadata.digital_gain = *dg;

	auto cg = ctrls.get(libcamera::controls::ColourGains);
	if (cg)
	{
		shared_data->metadata.colour_gains[0] = (*cg)[0], shared_data->metadata.colour_gains[1] = (*cg)[1];
	}

	auto fom = ctrls.get(libcamera::controls::FocusFoM);
	if (fom)
		shared_data->metadata.focus = *fom;

	auto ae = ctrls.get(libcamera::controls::AeLocked);
	if (ae)
		shared_data->metadata.aelock = *ae;

	auto lp = ctrls.get(libcamera::controls::LensPosition);
	if (lp)
		shared_data->metadata.lens_position = *lp;

	auto afs = ctrls.get(libcamera::controls::AfState);
	if (afs)
		shared_data->metadata.af_state = *afs;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new sharedContextStage(app);
}

static RegisterStage reg(NAME, &Create);
