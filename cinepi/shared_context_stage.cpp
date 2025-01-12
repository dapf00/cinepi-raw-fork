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
	shared_data->info.fd_raw = -1;
	shared_data->info.fd_isp = -1;
	shared_data->info.fd_lores = -1;
	shared_data->info.frame = 0;
	shared_data->info.procid = getpid();
	shared_data->info.ts = getTs();
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
	setMetadata(completed_request->metadata);

	if (sem_post(&shared_data->sem) == -1)
	{
		spdlog::get("SharedContext")->error("sem_post failed: {}", errno);
		return;
	}

	return;
}

void SharedContext::setInfo(CompletedRequestPtr &completed_request, CinePIController &controller)
{
	shared_data->info.is_recording = controller.isRecording();
	shared_data->info.ts = getTs();
	shared_data->info.fd_raw = completed_request->buffers[_app->RawStream()]->planes()[0].fd.get();
	shared_data->info.fd_isp = completed_request->buffers[_app->GetMainStream()]->planes()[0].fd.get();
	shared_data->info.fd_lores = completed_request->buffers[_app->LoresStream()]->planes()[0].fd.get();
	shared_data->info.raw_length = completed_request->buffers[_app->RawStream()]->planes()[0].length;
	shared_data->info.isp_length = completed_request->buffers[_app->GetMainStream()]->planes()[0].length;
	shared_data->info.lores_length = completed_request->buffers[_app->LoresStream()]->planes()[0].length;
	shared_data->info.framerate = completed_request->framerate;
	shared_data->info.sequence = completed_request->sequence;
	shared_data->info.frame++;
	shared_data->info.width = _options->width;
	shared_data->info.height = _options->height;
	shared_data->info.compression = _options->compression;
	shared_data->info.thumbnail = _options->thumbnail;
	shared_data->info.thumbnail_size = _options->thumbnailSize;
	shared_data->info.raw_crop =
		std::array<int, 4>({ _options->rawCrop[0], _options->rawCrop[1], _options->rawCrop[2], _options->rawCrop[3] });
	shared_data->raw = _app->GetStreamInfo(_app->RawStream());
	shared_data->isp = _app->GetStreamInfo(_app->GetMainStream());
	shared_data->lores = _app->GetStreamInfo(_app->LoresStream());
}

void SharedContext::setMetadata(libcamera::ControlList &cl)
{
	_setMetadataValue(cl, libcamera::controls::AeConstraintMode, shared_data->metadata.ae_constraint_mode);
	_setMetadataValue(cl, libcamera::controls::AeEnable, shared_data->metadata.ae_enable);
	_setMetadataValue(cl, libcamera::controls::AeExposureMode, shared_data->metadata.ae_exposure_mode);
	_setMetadataValue(cl, libcamera::controls::AeFlickerDetected, shared_data->metadata.ae_flicker_detected);
	_setMetadataValue(cl, libcamera::controls::AeFlickerMode, shared_data->metadata.ae_flicker_mode);
	_setMetadataValue(cl, libcamera::controls::AeFlickerPeriod, shared_data->metadata.ae_flicker_period);
	_setMetadataValue(cl, libcamera::controls::AeLocked, shared_data->metadata.ae_locked);
	_setMetadataValue(cl, libcamera::controls::AeMeteringMode, shared_data->metadata.ae_metering_mode);
	_setMetadataValue(cl, libcamera::controls::AnalogueGain, shared_data->metadata.analogue_gain);
	_setMetadataValue(cl, libcamera::controls::AwbEnable, shared_data->metadata.awb_enable);
	_setMetadataValue(cl, libcamera::controls::AwbLocked, shared_data->metadata.awb_locked);
	_setMetadataValue(cl, libcamera::controls::AwbMode, shared_data->metadata.awb_mode);
	_setMetadataValue(cl, libcamera::controls::Brightness, shared_data->metadata.brightness);
	// _setMetadataValue(cl, libcamera::controls::ColourGains, shared_data->metadata.color_gains); //
	auto color_gains_value = cl.get(libcamera::controls::ColourGains);
	if (color_gains_value)
	{
		std::array<float, 2> new_value = { (*color_gains_value)[0], (*color_gains_value)[1] };
		shared_data->metadata.color_gains = new_value;
	}
	_setMetadataValue(cl, libcamera::controls::ColourTemperature, shared_data->metadata.color_temperature);
	_setMetadataValue(cl, libcamera::controls::Contrast, shared_data->metadata.contrast);
	_setMetadataValue(cl, libcamera::controls::DigitalGain, shared_data->metadata.digital_gain);
	_setMetadataValue(cl, libcamera::controls::ExposureTime, shared_data->metadata.exposure_time);
	_setMetadataValue(cl, libcamera::controls::ExposureValue, shared_data->metadata.exposure_value);
	_setMetadataValue(cl, libcamera::controls::FrameDuration, shared_data->metadata.frame_duration);
	// _setMetadataValue(cl, libcamera::controls::FrameDurationLimits, shared_data->metadata.frame_duration_limits); //
	auto frame_duration_limits_value = cl.get(libcamera::controls::ColourGains);
	if (frame_duration_limits_value)
	{
		std::array<int, 2> new_value = { (*frame_duration_limits_value)[0], (*frame_duration_limits_value)[1] };
		shared_data->metadata.frame_duration_limits = new_value;
	}
	_setMetadataValue(cl, libcamera::controls::Gamma, shared_data->metadata.gamma);
	_setMetadataValue(cl, libcamera::controls::LensPosition, shared_data->metadata.lens_position);
	_setMetadataValue(cl, libcamera::controls::Lux, shared_data->metadata.lux); //
	_setMetadataValue(cl, libcamera::controls::Saturation, shared_data->metadata.saturation);
	_setMetadataValue(cl, libcamera::controls::Sharpness, shared_data->metadata.sharpness);
}

void SharedContext::readCommands(CinePIController &controller)
{
	libcamera::ControlList cl;
	_setCommand(cl, shared_data->commands.ae_constraint_mode, libcamera::controls::AeConstraintMode);
	_setCommand(cl, shared_data->commands.ae_enable, libcamera::controls::AeEnable);
	_setCommand(cl, shared_data->commands.ae_exposure_mode, libcamera::controls::AeExposureMode);
	_setCommand(cl, shared_data->commands.ae_flicker_mode, libcamera::controls::AeFlickerMode);
	_setCommand(cl, shared_data->commands.ae_flicker_period, libcamera::controls::AeFlickerPeriod);
	_setCommand(cl, shared_data->commands.ae_metering_mode, libcamera::controls::AeMeteringMode);
	_setCommand(cl, shared_data->commands.analogue_gain, libcamera::controls::AnalogueGain);
	_setCommand(cl, shared_data->commands.awb_enable, libcamera::controls::AwbEnable);
	_setCommand(cl, shared_data->commands.awb_mode, libcamera::controls::AwbMode);
	_setCommand(cl, shared_data->commands.brightness, libcamera::controls::Brightness);
	_setCommand(cl, shared_data->commands.color_gains, libcamera::controls::ColourGains);
	_setCommand(cl, shared_data->commands.color_temperature, libcamera::controls::ColourTemperature);
	_setCommand(cl, shared_data->commands.contrast, libcamera::controls::Contrast);
	_setCommand(cl, shared_data->commands.exposure_time, libcamera::controls::ExposureTime);
	_setCommand(cl, shared_data->commands.exposure_value, libcamera::controls::ExposureValue);
	_setCommand(cl, shared_data->commands.frame_duration_limits, libcamera::controls::FrameDurationLimits);
	_setCommand(cl, shared_data->commands.gamma, libcamera::controls::Gamma);
	_setCommand(cl, shared_data->commands.saturation, libcamera::controls::Saturation);
	_setCommand(cl, shared_data->commands.sharpness, libcamera::controls::Sharpness);
	if (shared_data->commands.set_recording)
	{
		bool rec = *(shared_data->commands.set_recording);
		int trig = rec ? 1 : -1;
		controller.setTrigger(trig);
		controller.setRecording(rec);
		shared_data->commands.set_recording = std::nullopt;
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
		shared_data->commands.thumbnail = std::nullopt;
	}
	if (shared_data->commands.thumbnail_size)
	{
		_options->thumbnailSize = *(shared_data->commands.thumbnail_size);
		shared_data->commands.thumbnail_size = std::nullopt;
	}
	if (shared_data->commands.raw_crop)
	{
		_options->rawCrop[0] = shared_data->commands.raw_crop.value().at(0);
		_options->rawCrop[1] = shared_data->commands.raw_crop.value().at(1);
		_options->rawCrop[2] = shared_data->commands.raw_crop.value().at(2);
		_options->rawCrop[3] = shared_data->commands.raw_crop.value().at(3);
		shared_data->commands.raw_crop = std::nullopt;
	}
	_app->SetControls(cl);
}
