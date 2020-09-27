#ifndef SYN_CLI_H
#define SYN_CLI_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_thirdparty/cxxopts/cxxopts.hpp"

namespace chrono {
namespace synchrono {

class SYN_API SynCLI {
  public:
    /// Constructor
    SynCLI(const std::string& program, const std::string& help_string = " - command line options");

    /// Destructor
    ~SynCLI() {}

    /// Parse messages and display help message, if needed
    bool Parse(int argc, char* argv[], bool show_help = false, bool update_config = true);

    /// Print our the help menu
    void Help() { std::cout << m_options.help() << std::endl; }

    /// Add default options for synchrono demos
    void AddDefaultDemoOptions();

    /// Check for value in vector
    template <typename T>
    bool HasValueInVector(const std::string& option, T value) {
        const auto vector = Get(option).as<std::vector<T>>();
        return std::find(vector.begin(), vector.end(), value) != vector.end();
    }

    /// Add an option
    template <typename T = bool>
    void AddOption(const std::string& group,
                   const std::string& opts,
                   const std::string& desc,
                   const std::string& def = "",
                   const std::string& arg_help = "") {
        m_options.add_option(group, cxxopts::Option(opts, desc, cxxopts::value<T>()->default_value(def), arg_help));
    }
    void AddOption(const std::string& group, cxxopts::Option option);

    /// Get option
    const cxxopts::OptionValue& Get(const std::string& option);

    /// Get option as type
    /// Recommanded way of accessing
    template <typename T>
    const T GetAsType(const std::string& option) {
        try {
            return (*m_result)[option].as<T>();
        } catch (std::domain_error e) {
            std::cout << "SynCLI::GetAsType: Could not cast \"" << option << "\" as " << typeid(T).name() << std::endl;
            std::cout << "SynCLI::GetAsType: Exitting..." << std::endl;
            exit(-1);
        }
    }

  private:
    cxxopts::Options m_options;                      ///< Command line options
    std::shared_ptr<cxxopts::ParseResult> m_result;  ///< Parsing results
};

}  // namespace synchrono
}  // namespace chrono

#endif
