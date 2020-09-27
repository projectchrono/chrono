#include "chrono_synchrono/visualization/SynVisualizationManager.h"

namespace chrono {
namespace synchrono {

// SynVisualizationManager::SynVisualizationManager(const std::string& filename) {
//   Document d = ReadFileJSON(filename);
//   if (d.IsNull())
//     throw ChException("Visualization file not read properly in SynVisualizationManager.");
//
//   // Read top-level data
//   assert(d.HasMember("Name"));
//   assert(d.HasMember("Type"));
//
//   std::string name = d["Name"].GetString();
//   std::string type = d["Type"].GetString();
//   assert(type.compare("Visualization") == 0);
//
//   // ----------------------------
//   // Validations of the JSON file
//   // ----------------------------
// }

void SynVisualizationManager::Update(double step) {
    if (!m_vis_list.size())
        return;

    for (std::shared_ptr<SynVisualization> vis : m_vis_list)
        vis->Update(step);
}

void SynVisualizationManager::AddVisualization(std::shared_ptr<SynVisualization> vis) {
    vis->Initialize();
    m_vis_list.push_back(vis);
}

}  // namespace synchrono
}  // namespace chrono
