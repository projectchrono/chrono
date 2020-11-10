#include "chrono_synchrono/cli/SynCLI.h"

namespace chrono {
namespace synchrono {

SynCLI::SynCLI(const std::string& program, const std::string& help_string) : m_options(program, help_string) {
    m_options.add_option("", cxxopts::Option("h,help", "Print usage"));
}

bool SynCLI::Parse(int argc, char* argv[], bool show_help, bool update_config) {
    try {
        m_result = std::make_shared<cxxopts::ParseResult>(m_options.parse(argc, argv));
    } catch (cxxopts::OptionException& e) {
        if (show_help) {
            GetLog() << "Error when parsing command line inputs."
                     << "\n"
                     << "what(): " << e.what() << "\n\n";
            Help();
        }
        return false;
    }

    if (m_result->count("help")) {
        if (show_help)
            Help();
        return false;
    }

    // if (update_config)
    //     ConfigFromCLI(this);

    return true;
}

void SynCLI::AddDefaultDemoOptions() {
    // General simulation options
    // AddOption<double>("Simulation", "step_size", "Step size", std::to_string(STEP_SIZE));
    // AddOption<double>("Simulation", "end_time", "End time", std::to_string(END_TIME));
    // AddOption<double>("Simulation", "heartbeat", "Heartbeat", std::to_string(HEARTBEAT));
    // AddOption<bool>("Simulation", "verbose", "Verbosity", std::to_string(VERBOSE));
    // AddOption<std::string>("Simulation", "contact_method", "Contact Method", ContactMethodToString(CONTACT_METHOD),
    //                        "NSC/SMC");

    // TODO: Can't find CHRONO_IRRLICHT for some reason
    // #ifdef CHRONO_IRRLICHT
    // Irrlicht options
    AddOption<std::vector<int>>("Irrlicht", "irr", "Ranks for irrlicht usage", "-1");
    AddOption<bool>("Irrlicht", "irr_save", "Toggle irrlicht saving ON", "false");
    AddOption<bool>("Irrlicht", "irr_vis", "Toggle irrlicht visualization ON", "false");
    // #endif

    // TODO: Can't find CHRONO_SENSOR for some reason
    // #ifdef CHRONO_SENSOR
    // Sensor options
    AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");
    // #endif
}

void SynCLI::AddOption(const std::string& group, cxxopts::Option option) {
    m_options.add_option(group, option);
}

const cxxopts::OptionValue& SynCLI::Get(const std::string& option) {
    return (*m_result)[option];
}

}  // namespace synchrono
}  // namespace chrono
