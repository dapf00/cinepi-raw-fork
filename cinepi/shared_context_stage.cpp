#include "raw_options.hpp"
#include "shared_context_stage.hpp"

// #include <chrono>
// #include <mutex>
// #include <thread>
// #include <time.h>
// #include <unistd.h>
// #include <vector>

// #include <libcamera/stream.h>

// //#include "core/frame_info.hpp"
// #include "post_processing_stages/post_processing_stage.hpp"

// #include <spdlog/sinks/stdout_color_sinks.h>
// #include <spdlog/spdlog.h>

#include <fcntl.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <sys/ipc.h>
// #include <sys/mman.h>
// #include <sys/stat.h>

//#define PROJECT_ID 0x43494E45 // ASCII for "CINE"

uint64_t SharedContext::getTs()
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return uint64_t(ts.tv_sec * 1000LL + ts.tv_nsec / 1000000);
}
SharedContext::SharedContext(CinePIRecorder *app)
	: shared_data(nullptr), _app(app), console(spdlog::stdout_color_mt("SharedContext")), _options(_app->GetOptions())
{
	const int size = sizeof(SharedMemoryBuffer);

	int fd = shm_open(PROJECT_ID, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
	if (fd == -1)
	{
		spdlog::get("SharedContext")->error("shm_open failed: {}", errno);
	}
	if (ftruncate(fd, size) == -1)
	{
		spdlog::get("SharedContext")->error("fruncate failed: {}", errno);
	}

	// Attach the shared memory segment
	shared_data = static_cast<SharedMemoryBuffer *>(mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
	if (shared_data == (void *)-1)
	{
		spdlog::get("SharedContext")->error("mmap failed: {}", errno);
	}

	if (close(fd) == -1)
	{
		spdlog::get("SharedContext")->error("fclose failed: {}", errno);
	}
	shared_data->fd_raw = -1;
	shared_data->fd_isp = -1;
	shared_data->fd_lores = -1;
	shared_data->frame = 0;
	shared_data->procid = getpid();
	shared_data->ts = getTs();
	sem_init(&shared_data->sem, 1, 1);
	spdlog::get("SharedContext")->info("SharedContext created");
}

SharedContext::~SharedContext()
{
	sem_destroy(&shared_data->sem);
	munmap(shared_data, sizeof(SharedMemoryBuffer));
}

void SharedContext::process(CompletedRequestPtr &completed_request, CinePIController &controller)
{
	if (sem_wait(&shared_data->sem) == -1)
	{
		spdlog::get("SharedContext")->error("sem_wait failed: {}", errno);
		return;
	}
	readCommands(controller);
	setInfo(completed_request, controller);

	if (sem_post(&shared_data->sem) == -1)
	{
		spdlog::get("SharedContext")->error("sem_post failed: {}", errno);
		return;
	}

	return;
}

void SharedContext::setInfo(CompletedRequestPtr &completed_request, CinePIController &controller)
{
	shared_data->is_recording = controller.isRecording();
	shared_data->ts = getTs();
	shared_data->fd_raw = completed_request->buffers[_app->RawStream()]->planes()[0].fd.get();
	shared_data->fd_isp = completed_request->buffers[_app->GetMainStream()]->planes()[0].fd.get();
	shared_data->raw_length = completed_request->buffers[_app->RawStream()]->planes()[0].length;
	shared_data->isp_length = completed_request->buffers[_app->GetMainStream()]->planes()[0].length;
	shared_data->fd_lores = completed_request->buffers[_app->LoresStream()]->planes()[0].fd.get();
	shared_data->lores_length = completed_request->buffers[_app->LoresStream()]->planes()[0].length;
	shared_data->framerate = completed_request->framerate;
	shared_data->sequence = completed_request->sequence;
	shared_data->frame++;
	shared_data->width = _options->width;
	shared_data->height = _options->height;
	shared_data->compression = _options->compression;
	shared_data->thumbnail = _options->thumbnail;
	shared_data->thumbnail_size = _options->thumbnailSize;
	shared_data->raw = _app->GetStreamInfo(_app->RawStream());
	shared_data->isp = _app->GetStreamInfo(_app->GetMainStream());
	shared_data->lores = _app->GetStreamInfo(_app->LoresStream());
	shared_data->raw_crop =
		std::array<int, 4>({ _options->rawCrop[0], _options->rawCrop[1], _options->rawCrop[2], _options->rawCrop[3] });
	auto ctrls = completed_request->metadata;

	auto colorT = ctrls.get(libcamera::controls::ColourTemperature);
	if (colorT)
		shared_data->metadata.color_temp = *colorT;

	auto awb = ctrls.get(libcamera::controls::SensorTimestamp);
	if (awb)
		shared_data->auto_white_balance = *awb;

	auto sts = ctrls.get(libcamera::controls::SensorTimestamp);
	if (sts)
		shared_data->metadata.sensor_timestamp = *sts;

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
		shared_data->metadata.color_gains.at(0) = (*cg)[0], shared_data->metadata.color_gains.at(1) = (*cg)[1];

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

void SharedContext::readCommands(CinePIController &controller)
{
	libcamera::ControlList cl;
	if (shared_data->commands.recording)
	{
		bool rec = *(shared_data->commands.auto_white_balance);
		int trig = rec ? 1 : -1;
		controller.setTrigger(trig);
		controller.setRecording(rec);
		shared_data->commands.recording = std::nullopt;
	}
	if (shared_data->commands.auto_white_balance)
	{
		cl.set(libcamera::controls::AwbEnable, *(shared_data->commands.auto_white_balance));
		shared_data->commands.auto_white_balance = std::nullopt;
	}
	if (shared_data->commands.reinitialize)
	{
		controller.setInit();
		shared_data->commands.reinitialize = std::nullopt;
	}
	if (shared_data->commands.width)
	{
		_options->width = *(shared_data->commands.width);
		shared_data->commands.width = std::nullopt;
	}
	if (shared_data->commands.height)
	{
		_options->height = *(shared_data->commands.height);
		shared_data->commands.height = std::nullopt;
	}
	if (shared_data->commands.compression)
	{
		_options->compression = *(shared_data->commands.compression);
		shared_data->commands.compression = std::nullopt;
	}
	if (shared_data->commands.thumbnail)
	{
		_options->thumbnail = *(shared_data->commands.thumbnail);

		controller.setInit();

		shared_data->commands.thumbnail = std::nullopt;
	}
	if (shared_data->commands.thumbnail_size)
	{
		_options->thumbnailSize = *(shared_data->commands.thumbnail_size);
		controller.setInit();
		shared_data->commands.thumbnail_size = std::nullopt;
	}
	if (shared_data->commands.raw_crop)
	{
		_options->rawCrop[0] = shared_data->commands.raw_crop.value().at(0);
		_options->rawCrop[1] = shared_data->commands.raw_crop.value().at(1);
		_options->rawCrop[2] = shared_data->commands.raw_crop.value().at(2);
		_options->rawCrop[3] = shared_data->commands.raw_crop.value().at(3);
		shared_data->commands.auto_white_balance = std::nullopt;
	}
	if (shared_data->commands.framerate)
	{
		_options->framerate = *(shared_data->commands.framerate);
		//TODO: implement solution from cinepi_controller
		shared_data->commands.framerate = std::nullopt;
	};
	if (shared_data->commands.iso)
	{
		cl.set(libcamera::controls::AnalogueGain, *(shared_data->commands.iso));
		shared_data->commands.iso = std::nullopt;
	};
	if (shared_data->commands.shutter_speed)
	{
		cl.set(libcamera::controls::ExposureTime, *(shared_data->commands.shutter_speed));
		shared_data->commands.shutter_speed = std::nullopt;
	};
	if (shared_data->commands.color_gains)
	{
		cl.set(libcamera::controls::ColourGains, *(shared_data->commands.color_gains));
		shared_data->commands.color_gains = std::nullopt;
	};
	_app->SetControls(cl);
}
