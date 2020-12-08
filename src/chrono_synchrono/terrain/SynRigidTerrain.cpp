#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#include "chrono/assets/ChVisualization.h"

#include "chrono_vehicle/ChVehicleModelData.h"

using namespace chrono::vehicle;
using namespace rapidjson;

namespace chrono {
namespace synchrono {

SynRigidTerrain::SynRigidTerrain(ChSystem* system, const std::string& filename) {
    auto d = SynTerrain::ParseTerrainFileJSON(filename);

    std::string subtype = d["Template"].GetString();
    assert(subtype.compare("RigidTerrain") == 0);

    std::string terrain_file;
    if (d.HasMember("Input File"))
        terrain_file = d["Input File"].GetString();
    else
        terrain_file = filename;

    m_rigid_terrain = chrono_types::make_shared<RigidTerrain>(system, vehicle::GetDataFile(terrain_file));

    if (d.HasMember("Visualization"))
        AddVisualizationAssetsJSON(d["Visualization"]);
}

void SynRigidTerrain::AddVisualizationAssetsJSON(const Value& a) {
    for (auto& patch : m_rigid_terrain->GetPatches()) {
        if (a.HasMember("Texture")) {
            std::string filename = a["Texture"]["Input File"].GetString();
            double sx = a["Texture"]["Scale"][0u].GetFloat();
            double sy = a["Texture"]["Scale"][1u].GetFloat();

            patch->SetTexture(GetChronoDataFile(filename), sx, sy);

#ifdef CHRONO_SENSOR
            auto patch_asset = patch->GetGroundBody()->GetAssets()[0];
            if (auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(patch_asset)) {
                std::shared_ptr<ChVisualMaterial> box_texture = chrono_types::make_shared<ChVisualMaterial>();
                box_texture->SetKdTexture(GetChronoDataFile(filename));

                if (a.HasMember("Fresnel")) {
                    box_texture->SetFresnelMin(a["Fresnel"]["Minimum"].GetFloat());
                    box_texture->SetFresnelMax(a["Fresnel"]["Minimum"].GetFloat());
                }

                if (a.HasMember("Specular Color")) {
                    auto r = a["Specular Color"][0u].GetFloat();
                    auto g = a["Specular Color"][1u].GetFloat();
                    auto b = a["Specular Color"][2u].GetFloat();

                    box_texture->SetSpecularColor({r, b, g});
                }

                visual_asset->material_list.push_back(box_texture);
            }
#endif
        }
    }
}

}  // namespace synchrono
}  // namespace chrono
