// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_CHANNEL_PLOTS_YAML_H
#define CH_CHANNEL_PLOTS_YAML_H

#include <string>
#include <vector>

#include "chrono_parsers/ChApiParsers.h"

#include "chrono/input_output/ChOutput.h"

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Plots of output channels, specified through the `plots` section of a YAML simulation specification file.
/// Channel values are recorded during the simulation (see Record) and plotted with Gnuplot at the end of the
/// simulation (see Generate). Each figure shows one or more channels as functions of time or of another channel.
/// If the recorded channels include a channel named `measuring`, plots can be restricted to the frames in which this
/// channel is positive.
class ChApiParsers ChChannelPlotsYAML {
  public:
    ChChannelPlotsYAML();

    /// Load the plot specifications from the given `plots` YAML node.
    void Load(const YAML::Node& a);

    /// Return true if plots were requested.
    bool Enabled() const { return !m_figures.empty(); }

    /// Return the number of requested figures.
    size_t GetNumFigures() const { return m_figures.size(); }

    /// Record the given channel values at the specified time.
    /// The set of channels must be the same at every call.
    void Record(double time, const std::vector<ChOutput::Channel>& channels);

    /// Generate the requested plots from the recorded data.
    /// For each figure, a Gnuplot script is written to the specified output directory and executed, producing an image
    /// file or an interactive window. Plotting requires the Chrono::Postprocess module and Gnuplot.
    void Generate(const std::string& out_dir) const;

  private:
    /// Plot output type.
    enum class PlotOutput { WINDOW, PNG, PDF };

    /// Figure specification: one or more channels as functions of time or of another channel.
    struct Figure {
        std::string x;               ///< abscissa channel (empty for time)
        std::vector<std::string> y;  ///< ordinate channels
        std::string title;           ///< figure title (optional)
        bool set_range;              ///< true if an ordinate range was specified
        double y_min;                ///< ordinate range (lower bound)
        double y_max;                ///< ordinate range (upper bound)
    };

    std::vector<Figure> m_figures;            ///< requested figures
    PlotOutput m_output;                      ///< plot output type
    bool m_measuring_only;                    ///< plot only frames with a positive `measuring` channel
    std::vector<std::string> m_channels;      ///< names of recorded channels
    std::vector<std::string> m_units;         ///< units of recorded channels
    std::vector<double> m_time;               ///< recorded times
    std::vector<std::vector<double>> m_data;  ///< recorded channel values (one vector per channel)
};

/// @} parsers_module

}  // namespace parsers
}  // namespace chrono

#endif
