// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// FEA multiphysics
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChContinuumThermal.h"
#include "chrono/fea/ChContinuumElectrostatics.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"
#include "chrono/fea/ChDomainDeformation.h"
#include "chrono/fea/ChDomainThermal.h"
#include "chrono/fea/ChDomainThermoDeformation.h"
#include "chrono/fea/ChVisualDataExtractor.h"
#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChFieldElementHexahedron8Loaders.h"
#include "chrono/fea/ChFieldElementTetrahedron4.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkLockTrajectory.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "FEAvisualization.h"

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

using namespace chrono;
using namespace chrono::fea;

/// Heat flux, per unit of surface.
/// Use this for applying a heat flux load on the surface of finite elements, as a
/// per-volume scalar flux for thermal analysis.

class ChLoaderHeatFlux : public ChLoaderUVdistributed {
public:
    ChLoaderHeatFlux(std::shared_ptr<ChLoadableUV> mloadable)
        : ChLoaderUVdistributed(mloadable), m_heat_flux(0), num_integration_points(1) {}

    virtual void ComputeF(double U,              ///< parametric coordinate in surface
        double V,              ///< parametric coordinate in surface
        ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
    ) override {
        F(0) = m_heat_flux;
    }

    /// Set the heat flux applied to UV surface, as [W/m^2]
    void SetSurfaceHeatFlux(double heat_flux) { m_heat_flux = heat_flux; }
    
    /// Get the heat flux applied to UV surface, as [W/m^2]
    double GetSurfaceHeatFlux() { return m_heat_flux; }

    void SetIntegrationPoints(int val) { num_integration_points = val; }
    virtual int GetIntegrationPointsU() override { return num_integration_points; }
    virtual int GetIntegrationPointsV() override { return num_integration_points; }

private:
    double m_heat_flux;
    int num_integration_points;
};


/// Heat flux, per unit of volume. 
/// Use this for applying a heat flux load on the surface of finite elements, as a
/// per-volume scalar flux for thermal analysis.

class ChLoaderHeatVolumetric : public ChLoaderUVWdistributed {
public:
    ChLoaderHeatVolumetric(std::shared_ptr<ChLoadableUVW> mloadable)
        : ChLoaderUVWdistributed(mloadable), m_heat_flux(0), num_integration_points(1) {}

    virtual void ComputeF(double U,              ///< parametric coordinate in surface
        double V,              ///< parametric coordinate in surface
        double W,              ///< parametric coordinate in surface
        ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
    ) override {
        F(0) = m_heat_flux;
    }

    /// Set the heat flux applied to UVW volume, as [W/m^3]
    void SetVolumeHeatFlux(double heat_flux) { m_heat_flux = heat_flux; }

    /// Get the heat flux applied to UVW volume, as [W/m^3]
    double GetVolumeHeatFlux() { return m_heat_flux; }

    void SetIntegrationPoints(int val) { num_integration_points = val; }
    virtual int GetIntegrationPointsU() override { return num_integration_points; }
    virtual int GetIntegrationPointsV() override { return num_integration_points; }
    virtual int GetIntegrationPointsW() override { return num_integration_points; }

private:
    double m_heat_flux;
    int num_integration_points;
};


int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    if (false) 
    {
        auto mnode1 = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
        auto mnode2 = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
        auto mnode3 = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
        auto mnode4 = chrono_types::make_shared<ChNodeFEAfieldXYZ>();

        mnode1->Set(0, 0, 0);
        mnode2->Set(0, 0, 1);
        mnode3->Set(0, 1, 0);
        mnode4->Set(1, 0, 0);

        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();
        displacement_field->AddNode(mnode1);
        displacement_field->AddNode(mnode2);
        displacement_field->AddNode(mnode3);
        displacement_field->AddNode(mnode4);

        displacement_field->NodeData(mnode1).SetFixed(true);
        displacement_field->NodeData(mnode1).SetPos(*mnode1);
        displacement_field->NodeData(mnode2).SetPos(*mnode2);
        displacement_field->NodeData(mnode3).SetPos(*mnode3);
        displacement_field->NodeData(mnode4).SetPos(ChVector3d(1, 0, 0));
        displacement_field->NodeData(mnode4).SetLoad(ChVector3d(900, 100, 200));

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();
        temperature_field->AddNode(mnode1);
        temperature_field->AddNode(mnode2);
        temperature_field->AddNode(mnode3);
        temperature_field->AddNode(mnode4);

        temperature_field->NodeData(mnode1).SetFixed(true);
        temperature_field->NodeData(mnode4).State()[0] = 100;



        auto tetrahedron1 = chrono_types::make_shared <ChFieldElementTetrahedron4>();
        tetrahedron1->SetNodes(mnode1, mnode2, mnode3, mnode4);


        //--------------------------------------


        auto thermal_domain = chrono_types::make_shared <ChDomainThermal>(temperature_field);
        thermal_domain->AddElement(tetrahedron1);

        // Needed to setup all data and pointers
        thermal_domain->InitialSetup();

        std::cout << "thermal_domain->GetNumPerNodeCoordsPosLevel() \n" << thermal_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        ChVectorDynamic<> mstate_block;
        thermal_domain->GetStateBlock(tetrahedron1, mstate_block);
        std::cout << "thermal_domain->GetStateBlock(tetrahedron1,mstate_block) \n" << mstate_block << "\n";

        thermal_domain->material = chrono_types::make_shared<ChMaterial3DThermal>(); // not needed - already attached by default
        thermal_domain->material->SetDensity(1000);
        thermal_domain->material->SetSpecificHeatCapacity(20);
        thermal_domain->material->SetThermalConductivity(5);

        ChVectorDynamic<> Fi;
        thermal_domain->ElementComputeInternalLoads(tetrahedron1, thermal_domain->ElementData(tetrahedron1), Fi);
        std::cout << "thermal_domain->ElementComputeInternalLoads()  Fi = \n" << Fi << "\n";

        ChMatrixDynamic<> Ki(4, 4);
        thermal_domain->ElementComputeKRMmatrices(tetrahedron1, thermal_domain->ElementData(tetrahedron1), Ki, 1, 0, 0);
        std::cout << "thermal_domain->ElementComputeKRMmatrices()  Ki = \n" << Ki << "\n";

        //--------------------------------------


        auto elastic_domain = chrono_types::make_shared <ChDomainDeformation<ChMaterial3DStressStVenant>>(displacement_field);
        elastic_domain->AddElement(tetrahedron1);

        // Needed to setup all data and pointers
        elastic_domain->InitialSetup();

        std::cout << "elastic_domain->GetNumPerNodeCoordsPosLevel() \n" << elastic_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        elastic_domain->GetStateBlock(tetrahedron1, mstate_block);
        std::cout << "elastic_domain->GetStateBlock(tetrahedron1,mstate_block) \n" << mstate_block << "\n";

        auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        elastic_domain->material = elastic_material;
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(2e9);
        elastic_material->SetPoissonRatio(0.3);

        elastic_domain->ElementComputeInternalLoads(tetrahedron1, elastic_domain->ElementData(tetrahedron1), Fi);
        std::cout << "elastic_domain->ElementComputeInternalLoads()  Fi = \n" << Fi << "\n";

        //ChMatrixDynamic<> Ki;
        Ki.resize(tetrahedron1->GetNumNodes() * 3, tetrahedron1->GetNumNodes() * 3);
        elastic_domain->ElementComputeKRMmatrices(tetrahedron1, elastic_domain->ElementData(tetrahedron1), Ki, 1, 0, 0);
        std::cout << "elastic_domain->ElementComputeKRMmatrices()  Ki = \n" << Ki << "\n";

        // ---------------

        auto thermoelastic_domain = chrono_types::make_shared <ChDomainThermoDeformation>(temperature_field, displacement_field);
        thermoelastic_domain->AddElement(tetrahedron1);
        thermoelastic_domain->InitialSetup();

        std::cout << "thermoelastic_domain->GetNumPerNodeCoordsPosLevel() " << thermoelastic_domain->GetNumPerNodeCoordsPosLevel() << "\n";
        thermoelastic_domain->GetStateBlock(tetrahedron1, mstate_block);
        std::cout << "thermoelastic_domain->GetStateBlock(tetrahedron1,mstate_block) " << mstate_block << "\n";

    }

    //-----------------

    // Later one can access instanced per-element data as in these examples:
    //  thermal_domain->ElementData(tetrahedron1).element_data; // ...
    //  thermal_domain->ElementData(tetrahedron1).matpoints_data.size(); // ...
    //  thermal_domain->ElementData(tetrahedron1).nodes_data.size(); // ..  this would fail if no InitialSetup(


    if (false) {

        // Create a Chrono physical system
        ChSystemNSC sys;

        auto floor = chrono_types::make_shared<ChBody>();
        floor->SetFixed(true);
        sys.Add(floor);

        auto test_glyph = chrono_types::make_shared<ChGlyphs>();
        test_glyph->SetGlyphPoint(0, ChVector3d(0, 0.3, 0));
        test_glyph->SetGlyphsSize(0.1);
        floor->AddVisualShape(test_glyph);


        class Ch3DArrayOfNodes {
            size_t n, m, k;
        public:
            std::vector<std::shared_ptr<ChNodeFEAfieldXYZ>> data;
            Ch3DArrayOfNodes(size_t n_, size_t m_, size_t k_)
                : n(n_), m(m_), k(k_), data(n_* m_* k_) {}
            std::shared_ptr<ChNodeFEAfieldXYZ>& at(size_t i, size_t j, size_t l) {
                return data[i * m * k + j * k + l];
            }
        };
        class Ch3DArrayOfHexa {
            size_t n, m, k;
        public:
            std::vector<std::shared_ptr<ChFieldElementHexahedron8>> data;
            Ch3DArrayOfHexa(size_t n_, size_t m_, size_t k_)
                : n(n_), m(m_), k(k_), data(n_* m_* k_) {}
            std::shared_ptr<ChFieldElementHexahedron8>& at(size_t i, size_t j, size_t l) {
                return data[i * m * k + j * k + l];
            }
        };

        int nlayers_x = 5;
        int nlayers_y = 1;
        int nlayers_z = 1;
        double W_x = 3;
        double W_y = 0.5;
        double W_z = 0.5;

        Ch3DArrayOfNodes mnodes(nlayers_x + 1, nlayers_y + 1, nlayers_z + 1);
        Ch3DArrayOfHexa  melements(nlayers_x, nlayers_y, nlayers_z);

        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();

        for (int i_z = 0; i_z <= nlayers_z; ++i_z) {
            for (int i_y = 0; i_y <= nlayers_y; ++i_y) {
                for (int i_x = 0; i_x <= nlayers_x; ++i_x) {
                    ChVector3d mypos((W_x / nlayers_x) * i_x, (W_y / nlayers_y) * i_y, (W_z / nlayers_z) * i_z);
                    auto mnode = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
                    mnode->Set(mypos);
                    displacement_field->AddNode(mnode);
                    displacement_field->NodeData(mnode).SetPos(mypos); // initial position = ref position
                    mnodes.at(i_x, i_y, i_z) = mnode;
                    if (i_x > 0 && i_y > 0 && i_z > 0) {
                        auto hexa = chrono_types::make_shared <ChFieldElementHexahedron8>();
                        hexa->SetNodes({ mnodes.at(i_x - 1, i_y - 1, i_z - 1),
                                         mnodes.at(i_x  , i_y - 1, i_z - 1),
                                         mnodes.at(i_x  , i_y  , i_z - 1),
                                         mnodes.at(i_x - 1, i_y  , i_z - 1),
                                         mnodes.at(i_x - 1, i_y - 1, i_z),
                                         mnodes.at(i_x  , i_y - 1, i_z),
                                         mnodes.at(i_x  , i_y  , i_z),
                                         mnodes.at(i_x - 1, i_y  , i_z)
                            });
                        melements.at(i_x - 1, i_y - 1, i_z - 1) = hexa;
                    }
                }
            }
        }
        auto elastic_domain = chrono_types::make_shared <ChDomainDeformation<ChMaterial3DStressStVenant>>(displacement_field);
        for (auto& created_element : melements.data)
            elastic_domain->AddElement(created_element);

        auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        elastic_domain->material = elastic_material;
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(2e6);
        elastic_material->SetPoissonRatio(0.3);

        // Needed to setup all data and pointers
        elastic_domain->InitialSetup();

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_nodes->SetGlyphsSize(0.1);
        visual_nodes->AddPositionExtractor(chrono_types::make_shared<ChVisualDataExtractorPos>());
        visual_nodes->AddPropertyExtractor(chrono_types::make_shared<ChVisualDataExtractorPosDt>(), 0.0, 2.0, "Vel");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_nodes);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh->AddPositionExtractor(chrono_types::make_shared<ChVisualDataExtractorPos>());
        visual_mesh->AddPropertyExtractor(chrono_types::make_shared<ChVisualDataExtractorPosDt>(), 0.0, 2.0, "Vel");
        visual_mesh->SetColormap(ChColor(0, 1, 0));
        //visual_mesh->SetWireframe(true);
        floor->AddVisualShape(visual_mesh);

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test FEA");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 4, -6));
        vis->AddTypicalLights();
        
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);

        displacement_field->NodeData(mnodes.at(0, 0, 0)).SetFixed(true);
        displacement_field->NodeData(mnodes.at(0, 1, 0)).SetFixed(true);
        displacement_field->NodeData(mnodes.at(0, 0, 1)).SetFixed(true);
        displacement_field->NodeData(mnodes.at(0, 1, 1)).SetFixed(true);

        displacement_field->NodeData(mnodes.at(4, 0, 0)).SetLoad(ChVector3d(0,1000,0));
        displacement_field->NodeData(mnodes.at(1, 1, 0)).SetPos(displacement_field->NodeData(mnodes.at(1, 1, 0)).GetPos() + ChVector3d(0, 0.1, 0));
        displacement_field->NodeData(mnodes.at(1, 1, 1)).SetPos(displacement_field->NodeData(mnodes.at(1, 1, 1)).GetPos() + ChVector3d(0, 0, 0));

        //displacement_field->NodeData(mnodes.at(0, 0, 0)).SetPos(displacement_field->NodeData(mnodes.at(0, 0, 0)).GetPos() + ChVector3d(0.05, 0, 0));
        //displacement_field->NodeData(mnodes.at(0, 0, 1)).SetPos(displacement_field->NodeData(mnodes.at(0, 0, 1)).GetPos() + ChVector3d(0.05, 0, 0));

        sys.Add(elastic_domain);
        sys.Add(displacement_field);

        // Simulation loop
        double timestep = 0.01;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }

    }


    if (true) {

        // Create a Chrono physical system
        ChSystemNSC sys;

        auto floor = chrono_types::make_shared<ChBody>();
        floor->SetFixed(true);
        sys.Add(floor);

        class Ch3DArrayOfNodes {
            size_t n, m, k;
        public:
            std::vector<std::shared_ptr<ChNodeFEAfieldXYZ>> data;
            Ch3DArrayOfNodes(size_t n_, size_t m_, size_t k_)
                : n(n_), m(m_), k(k_), data(n_* m_* k_) {}
            std::shared_ptr<ChNodeFEAfieldXYZ>& at(size_t i, size_t j, size_t l) {
                return data[i * m * k + j * k + l];
            }
        };
        class Ch3DArrayOfHexa {
            size_t n, m, k;
        public:
            std::vector<std::shared_ptr<ChFieldElementHexahedron8>> data;
            Ch3DArrayOfHexa(size_t n_, size_t m_, size_t k_)
                : n(n_), m(m_), k(k_), data(n_* m_* k_) {}
            std::shared_ptr<ChFieldElementHexahedron8>& at(size_t i, size_t j, size_t l) {
                return data[i * m * k + j * k + l];
            }
        };

        int nlayers_x = 5;
        int nlayers_y = 1;
        int nlayers_z = 5;
        double W_x = 3;
        double W_y = 0.5;
        double W_z = 3;

        Ch3DArrayOfNodes mnodes(nlayers_x + 1, nlayers_y + 1, nlayers_z + 1);
        Ch3DArrayOfHexa  melements(nlayers_x, nlayers_y, nlayers_z);

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();

        for (int i_z = 0; i_z <= nlayers_z; ++i_z) {
            for (int i_y = 0; i_y <= nlayers_y; ++i_y) {
                for (int i_x = 0; i_x <= nlayers_x; ++i_x) {
                    ChVector3d mypos((W_x / nlayers_x) * i_x, (W_y / nlayers_y) * i_y, (W_z / nlayers_z) * i_z);
                    auto mnode = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
                    mnode->Set(mypos);
                    temperature_field->AddNode(mnode);
                    mnodes.at(i_x, i_y, i_z) = mnode;
                    if (i_x > 0 && i_y > 0 && i_z > 0) {
                        auto hexa = chrono_types::make_shared <ChFieldElementHexahedron8>();
                        hexa->SetNodes({ mnodes.at(i_x - 1, i_y - 1, i_z - 1),
                                         mnodes.at(i_x  , i_y - 1, i_z - 1),
                                         mnodes.at(i_x  , i_y  , i_z - 1),
                                         mnodes.at(i_x - 1, i_y  , i_z - 1),
                                         mnodes.at(i_x - 1, i_y - 1, i_z),
                                         mnodes.at(i_x  , i_y - 1, i_z),
                                         mnodes.at(i_x  , i_y  , i_z),
                                         mnodes.at(i_x - 1, i_y  , i_z)
                            });
                        melements.at(i_x - 1, i_y - 1, i_z - 1) = hexa;
                    }
                }
            }
        }
        auto thermal_domain = chrono_types::make_shared <ChDomainThermal>(temperature_field);
        for (auto& created_element : melements.data)
            thermal_domain->AddElement(created_element);

        auto thermal_material = chrono_types::make_shared<ChMaterial3DThermal>();
        thermal_domain->material = thermal_material;
        thermal_material->SetDensity(1000);
        thermal_material->SetSpecificHeatCapacity(1.11);
        thermal_material->SetThermalConductivity(0.16);

 

        // EXAMPLE INITIAL CONDITIONS (initial temperature of some nodes)

        temperature_field->NodeData(mnodes.at(0, 0, 4)).T() = 100;
        temperature_field->NodeData(mnodes.at(0, 1, 4)).T() = 100;

        // EXAMPLE DIRICHLET CONDITIONS (fixed temperature of some nodes)

        temperature_field->NodeData(mnodes.at(0, 0, 0)).SetFixed(true);
        temperature_field->NodeData(mnodes.at(0, 0, 0)).T() = 30;
        temperature_field->NodeData(mnodes.at(0, 1, 0)).SetFixed(true);
        temperature_field->NodeData(mnodes.at(0, 1, 0)).T() = 20;
        temperature_field->NodeData(mnodes.at(0, 0, 1)).SetFixed(true);
        temperature_field->NodeData(mnodes.at(0, 0, 1)).T() = 30;
        temperature_field->NodeData(mnodes.at(0, 1, 1)).SetFixed(true);
        temperature_field->NodeData(mnodes.at(0, 1, 1)).T() = 30;        
        
        // APPLY SOME LOADS

        // First: loads must be added to "load containers",
        // and load containers must be added to your system
        auto load_container = chrono_types::make_shared<ChLoadContainer>();
        sys.Add(load_container);

        // - IMPOSED HEAT FLUX ON SURFACE
        // Create a face wrapper, an auxiliary object that references a face of an
        // element as a ChLoadableUV so that can receive a surface load affecting a field 
        auto exa_face = chrono_types::make_shared<ChFieldHexahedronFace>(melements.at(2,0,3), temperature_field, 3); // 3rd face of hexa is y up

        auto heat_flux = chrono_types::make_shared<ChLoaderHeatFlux>(exa_face);
        heat_flux->SetSurfaceHeatFlux(0); // the surface flux: heat in W/m^2
        load_container->Add(heat_flux);

        // - IMPOSED HEAT SOURCE ON VOLUME
        // Create a volume wrapper, an auxiliary object that references a volume of an
        // element as a ChLoadableUVW so that it can receive a volume load affecting a field 
        auto exa_volume = chrono_types::make_shared<ChFieldHexahedronVolume>(melements.at(4, 0, 4), temperature_field);

        auto heat_source = chrono_types::make_shared<ChLoaderHeatVolumetric>(exa_volume);
        heat_source->SetVolumeHeatFlux(500); // the volumetric source flux: heat in W/m^3
        load_container->Add(heat_source);


        // Needed to setup all data and pointers
        thermal_domain->InitialSetup();


        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(thermal_domain);
        visual_nodes->SetGlyphsSize(0.05);
        visual_nodes->AddPropertyExtractor(chrono_types::make_shared<ChVisualDataExtractorTemperature>(), 0.0, 100, "Temp");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_nodes);

        auto visual_matpoints = chrono_types::make_shared<ChVisualDomainGlyphs>(thermal_domain);
        visual_matpoints->SetGlyphsSize(0.1);
        visual_matpoints->glyph_scalelenght = 0.01;
        visual_matpoints->AddPropertyExtractor(chrono_types::make_shared<ChDomainThermal::ChVisualDataExtractorHeatFlux>(), 0.0, 50, "q flux");
        visual_matpoints->SetColormap(ChColormap(ChColormap::Type::JET));
        floor->AddVisualShape(visual_matpoints);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(thermal_domain);
        visual_mesh->AddPropertyExtractor(chrono_types::make_shared<ChVisualDataExtractorTemperature>(), 0.0, 100, "Temp");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));
        visual_mesh->SetWireframe(true);
        //floor->AddVisualShape(visual_mesh);

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test FEA");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 4, -6));
        vis->AddTypicalLights();


        // SOLVER SETTINGS

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);


        sys.Add(thermal_domain);
        sys.Add(temperature_field);

        // Simulation loop
        double timestep = 50;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }

    }


    return 0;

}
