#ifndef CH_VSG_SETTINGS_H
#define CH_VSG_SETTINGS_H

#include <iostream>
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {
typedef enum { Wireframe, Textured, Phong } DrawMode;

typedef enum {
    Emerald,
    Jade,
    Obsidian,
    Pearl,
    Ruby,
    Turquoise,
    Brass,
    Bronze,
    PolishedBronze,
    Chrome,
    Copper,
    PolishedCopper,
    Gold,
    PolishedGold,
    Pewter,
    Silver,
    PolishedSilver,
    BlackPlastic,
    CyanPlastic,
    GreenPlastic,
    RedPlastic,
    WhitePlastic,
    YellowPlastic,
    BluePlastic,
    BlackRubber,
    CyanRubber,
    GreenRubber,
    RedRubber,
    WhiteRubber,
    YellowRubber,
    BlueRubber
} PhongPresets;
}  // namespace vsg3d
}  // namespace chrono
#endif
