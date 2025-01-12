#pragma once
#include "cinepi_controller.hpp"
#include "cinepi_recorder.hpp"
#include "shared_context_structs.hpp"

//#include "core/rpicam_app.hpp"
constexpr auto PROJECT_ID = "/CINE";

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
	void setMetadata(libcamera::ControlList &cl);
	std::shared_ptr<spdlog::logger> console;

	SharedMemoryBuffer *shared_data;
	key_t segment_key;

	bool running_ = true;

	uint64_t getTs();
	CinePIRecorder *_app;
	RawOptions *_options;

	template <typename T, typename U>
	void _setCommand(libcamera::ControlList &cl, std::optional<T> &command, const libcamera::Control<U> &control)
	{
		if (command)
		{
			cl.set(control, *command);
			command = std::nullopt;
		}
	}

	template <typename T, typename V>
	void _setMetadataValue(const libcamera::ControlList &cl, const libcamera::Control<T> &control, V &metadataField)
	{
		auto value = cl.get(control);
		if (value)
		{
			metadataField = *value;
		}
	}
};
