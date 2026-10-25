%{
#include "chrono/input_output/ChOutput.h"
%}

// Nested types, renamed to avoid generic names at module level
%rename(OutputSettings) chrono::ChOutput::Settings;
%rename(OutputChannel) chrono::ChOutput::Channel;

%include "../../../chrono/input_output/ChOutput.h"

// Lists of output channels (e.g., as returned by ChWheelTestRig::GetOutputChannels)
%template(vector_OutputChannel) std::vector<chrono::ChOutput::Channel>;
