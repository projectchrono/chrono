#ifndef SYN_VISUALIZATION_MANAGER_H
#define SYN_VISUALIZATION_MANAGER_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/visualization/SynVisualization.h"

#include "chrono/physics/ChSystem.h"

using namespace chrono;

namespace chrono {
namespace synchrono {

class SYN_API SynVisualizationManager {
  public:
    /// Constructs a visualization manager
    SynVisualizationManager() {}
    // SynVisualizationManager(const std::string& filename);

    /// Destructor
    ~SynVisualizationManager() {}

    /// Initializes visualization manager
    void Update(double step);

    /// Attaches specified visualizer to manager
    void AddVisualization(std::shared_ptr<SynVisualization> vis);

    /// Get visualization list
    SynVisualizationList GetVisualizationList() { return m_vis_list; }

  protected:
    SynVisualizationList m_vis_list;  ///< vector of handles to visualizations
};

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_VIS_MANAGER_H
