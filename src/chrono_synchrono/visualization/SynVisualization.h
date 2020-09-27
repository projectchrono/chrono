#ifndef SYN_VISUALIZATION_H
#define SYN_VISUALIZATION_H

#include "chrono_synchrono/SynApi.h"

#include <vector>
#include <memory>

namespace chrono {
namespace synchrono {

class SYN_API SynVisualization {
  public:
    /// Visualization types
    enum VisualizationType {
        SENSOR,   ///> sensor visualization
        IRRLICHT  ///> irrlicht visualization
    };

    /// Constructs a visualization manager
    SynVisualization(VisualizationType type) : m_type(type), m_should_initialize(true) {}

    /// Destructor
    ~SynVisualization() {}

    /// Whether or not visualizers need initialization
    bool ShouldInitialize() { return m_should_initialize; }

    /// Advance the state of this visualizer.
    virtual void Update(double step) = 0;

    /// Initialize this visualizer.
    virtual void Initialize() = 0;

    /// Get visualizer type
    VisualizationType GetType() { return m_type; }

  protected:
    VisualizationType m_type;  ///< vis type

    bool m_should_initialize;  ///< does visualizers need initialization
};

typedef std::vector<std::shared_ptr<SynVisualization>> SynVisualizationList;

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_VIS_H
