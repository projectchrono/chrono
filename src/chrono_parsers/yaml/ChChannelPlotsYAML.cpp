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

#include <iostream>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_parsers/yaml/ChChannelPlotsYAML.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChChannelPlotsYAML::ChChannelPlotsYAML() : m_output(PlotOutput::PNG), m_measuring_only(true) {}

void ChChannelPlotsYAML::Load(const YAML::Node& a) {
    if (a["output"]) {
        auto output = ChToUpper(a["output"].as<std::string>());
        if (output == "WINDOW")
            m_output = PlotOutput::WINDOW;
        else if (output == "PNG")
            m_output = PlotOutput::PNG;
        else if (output == "PDF")
            m_output = PlotOutput::PDF;
        else {
            cerr << "Error: unknown plot output '" << a["output"].as<std::string>() << "'." << endl;
            throw std::runtime_error("Unknown plot output");
        }
    }
    if (a["measuring_only"])
        m_measuring_only = a["measuring_only"].as<bool>();

    if (!a["figures"])
        return;
    for (const auto& f : a["figures"]) {
        Figure figure;
        figure.set_range = false;
        if (f.IsScalar()) {
            // Shorthand: a single channel as a function of time
            figure.y.push_back(f.as<std::string>());
        } else {
            ChAssertAlways(f["y"]);
            if (f["y"].IsSequence()) {
                for (const auto& y : f["y"])
                    figure.y.push_back(y.as<std::string>());
            } else {
                figure.y.push_back(f["y"].as<std::string>());
            }
            if (f["x"])
                figure.x = f["x"].as<std::string>();
            if (f["title"])
                figure.title = f["title"].as<std::string>();
            if (f["y_range"]) {
                ChAssertAlways(f["y_range"].IsSequence() && f["y_range"].size() == 2);
                figure.set_range = true;
                figure.y_min = f["y_range"][0].as<double>();
                figure.y_max = f["y_range"][1].as<double>();
            }
        }
        m_figures.push_back(figure);
    }
}

void ChChannelPlotsYAML::Record(double time, const std::vector<ChOutput::Channel>& channels) {
    if (!Enabled())
        return;

    if (m_channels.empty()) {
        for (const auto& channel : channels) {
            m_channels.push_back(channel.name);
            m_units.push_back(channel.units);
        }
        m_data.resize(channels.size());
    }
    m_time.push_back(time);
    for (size_t i = 0; i < channels.size(); i++)
        m_data[i].push_back(channels[i].value);
}

void ChChannelPlotsYAML::Generate(const std::string& out_dir) const {
    if (!Enabled())
        return;

#ifdef CHRONO_POSTPROCESS
    if (m_time.empty()) {
        cerr << "Warning: no data recorded for plots." << endl;
        return;
    }

    // Index of a recorded channel (-1 if not found)
    auto find_channel = [this](const std::string& name) -> int {
        for (size_t i = 0; i < m_channels.size(); i++) {
            if (m_channels[i] == name)
                return (int)i;
        }
        return -1;
    };

    // Frames to plot (all, or only those with measurements enabled)
    std::vector<size_t> frames;
    int i_measuring = find_channel("measuring");
    for (size_t k = 0; k < m_time.size(); k++) {
        if (!m_measuring_only || i_measuring < 0 || m_data[i_measuring][k] > 0)
            frames.push_back(k);
    }
    if (frames.empty()) {
        cerr << "Warning: no frames to plot (measurements were never enabled)." << endl;
        return;
    }

    auto select = [&frames](const std::vector<double>& values) {
        std::vector<double> selected;
        selected.reserve(frames.size());
        for (auto k : frames)
            selected.push_back(values[k]);
        return selected;
    };
    auto label = [this](int i) { return m_channels[i] + " (" + m_units[i] + ")"; };

    for (size_t f = 0; f < m_figures.size(); f++) {
        const auto& figure = m_figures[f];

        // Abscissa data and label
        std::vector<double> x_data;
        std::string x_label;
        if (figure.x.empty()) {
            x_data = select(m_time);
            x_label = "time (s)";
        } else {
            int i = find_channel(figure.x);
            if (i < 0) {
                cerr << "Warning: unknown channel '" << figure.x << "' in plot " << f + 1 << ". Plot skipped." << endl;
                continue;
            }
            x_data = select(m_data[i]);
            x_label = label(i);
        }

        // Ordinate channels
        std::vector<int> y_channels;
        for (const auto& name : figure.y) {
            int i = find_channel(name);
            if (i < 0)
                cerr << "Warning: unknown channel '" << name << "' in plot " << f + 1 << ". Channel skipped." << endl;
            else
                y_channels.push_back(i);
        }
        if (y_channels.empty())
            continue;

        std::string stem = out_dir + "/plot_" + std::to_string(f + 1);
        postprocess::ChGnuPlot gplot(stem + ".gpl");
        switch (m_output) {
            case PlotOutput::PNG:
                gplot.SetPersist(false);
                gplot.OutputPNG(stem + ".png", 800, 600);
                break;
            case PlotOutput::PDF:
                gplot.SetPersist(false);
                gplot.OutputPDF(stem + ".pdf");
                break;
            case PlotOutput::WINDOW:
                break;
        }
        gplot.SetCommand("set termoption noenhanced");  // render channel names (with underscores) literally
        gplot.SetGrid();
        if (!figure.title.empty())
            gplot.SetTitle(figure.title);
        gplot.SetLabelX(x_label);
        gplot.SetLabelY(y_channels.size() == 1 ? label(y_channels[0]) : m_units[y_channels[0]]);
        if (figure.set_range)
            gplot.SetRangeY(figure.y_min, figure.y_max);
        for (auto i : y_channels)
            gplot.Plot(x_data, select(m_data[i]), m_channels[i], " with lines");

        // The plot is generated when the ChGnuPlot object goes out of scope
    }
#else
    cerr << "Warning: plots require the Chrono::Postprocess module. No plots generated." << endl;
#endif
}

}  // namespace parsers
}  // namespace chrono
