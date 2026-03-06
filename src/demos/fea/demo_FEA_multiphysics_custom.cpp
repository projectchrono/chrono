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
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChDomainDeformation.h"
#include "chrono/fea/ChDomainThermal.h"
#include "chrono/fea/ChDomainThermoDeformation.h"
#include "chrono/fea/ChMaterial3DThermalNonlinear.h"
#include "chrono/fea/ChDrawer.h"
#include "chrono/fea/ChDomainSurface.h"
#include "chrono/fea/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/ChFieldElementLoadableVolume.h"
#include "chrono/fea/ChFieldElementLoadableSurface.h"
#include "chrono/fea/ChLoaderHeatFlux.h"
#include "chrono/fea/ChLoaderHeatRadiation.h"
#include "chrono/fea/ChLoaderHeatVolumetricSource.h"
#include "chrono/fea/ChLoaderHeatConvection.h"
#include "chrono/fea/ChLinkFieldNode.h"
#include "chrono/fea/ChBuilderVolume.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "FEAvisualization.h"

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

using namespace chrono;
using namespace chrono::fea;





int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;


    // EXAMPLE 1
    // 
    // Example on how to develop your own multiphysics problem, that are not yet
    // available in the current Chrono API. 

    if (true) {
        
        // Suppose we are solving a problem where the chemical concentration of some substance changes
        // according to soma law in a chemical field, and it also affects mechanical properties of some
        // other field, ex. elasticity.
        

        // 1) Let's implement a new FIELD DATA for some field.
        //
        // We define data for a scalar field defining the concentration of some substance.
        // This is expressed as concentration c [mol/m^3], i.e. moles per volume.
        // Concentration c [mol/m^3] is related to moles per length n=[mol/m] as c=n/A, with A cross section.       
        // At each point in the discretized field, this is the data that we simulate.
        // Being a scalar, we inherit from ChFieldDataScalar. (btw. we could use directly 
        // ChFieldDataScalar in our ChDomain, but for readability we do the following, that
        // also add two helper functions Concentration() etc.)

        class ChFieldDataChemicalConcentration : public ChFieldDataScalar {
          public:
            double& Concentration() { return State()[0]; }
            double& Concentration_dt() { return StateDt()[0]; }
        };



        // 2) Let's implement a new FIELD class.
        //
        // The field is based on the ChFieldDataChemicalConcentration, i.e. per each node in the
        // discretization, we attach a ChFieldDataChemicalConcentration.

        class ChFieldChemicalConcentration : public ChField<ChFieldDataChemicalConcentration> {};



        // 3) Let's implement a new ELEMENT class.
        //
        // Suppose that the chemical field is like a network of capillary pipes, each 
        // edge of the network being an element. In the default Chrono API there's no such
        // element: let's do a minimal implementation.
        // Note that the element can be field-agnostic, and material-agnostic (although this 
        // may be customized in some special cases)
        
        class ChFieldElementChemicalEdge : public ChFieldElementLine {
          public:
            ChFieldElementChemicalEdge() { 
                this->quadrature_order = 1; }
            ChFieldElementChemicalEdge(std::shared_ptr<ChNodeFEAfieldXYZ> nodeA,
                                       std::shared_ptr<ChNodeFEAfieldXYZ> nodeB) {
                this->quadrature_order = 1;
                SetNodes(nodeA, nodeB);
            }
            virtual ~ChFieldElementChemicalEdge() {}

            /// Return the specified edge node (0 <= n <= 1).
            virtual std::shared_ptr<ChNodeFEAfieldXYZ> GetEdgeNode(unsigned int n) { return nodes[n]; }

            /// Set the nodes used by this edge.
            virtual void SetNodes(std::shared_ptr<ChNodeFEAfieldXYZ> nodeA, std::shared_ptr<ChNodeFEAfieldXYZ> nodeB) {
                nodes[0] = nodeA;
                nodes[1] = nodeB;
            }
            virtual std::shared_ptr<ChNodeFEAfieldXYZ> GetNodeA() { return nodes[0]; };
            virtual std::shared_ptr<ChNodeFEAfieldXYZ> GetNodeB() { return nodes[1]; };

            // Interfaces that must be implemented:

            /// Get the number of nodes used by this element.
            virtual unsigned int GetNumNodes() override { return 2; };

            /// Access the nth node.
            virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override { return nodes[n]; };

            // Return dimension of embedding space X
            virtual int GetSpatialDimensions() const override { return 3; };

            // Compute the shape function N at eta.x parametric coordinate.
            virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override { N.resize(GetNumNodes()); N(0) = 1. - eta.x(); N(1) = eta.x(); };

            // Compute shape function material derivatives dN/d\eta 
            virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override { dNde.setZero(GetManifoldDimensions(), GetNumNodes()); dNde(0,0) = -1; dNde(1,0) = 1;};

            // Tell how many material points are needed - NOTE: THIS EXAMPLE DEMONSTRATES ELEMENTS WITHOUT QUADRATURE
            virtual int GetNumQuadraturePointsForOrder(const int order) const { return 1; }

            // Get i-th material point weight and parametric coords  - NOTE: THIS EXAMPLE DEMONSTRATES ELEMENTS WITHOUT QUADRATURE
            virtual void GetMaterialPointWeight(const int order,
                                                  const int i,
                                                  double& weight,
                                                  ChVector3d& coords) const override { weight = 1.0; coords={0.5,0,0};};

            /// Update, called at least at each time step.
            /// If the element has to keep updated some auxiliary data, maybe implement this.
            virtual void Update() {}

          private:
            /// Initial setup (called once before start of simulation).
            /// This is used mostly to precompute matrices that do not change during the simulation, i.e. the local
            /// stiffness of each element, if any, the mass, etc.
            virtual void SetupInitial(ChSystem* system) {}

            std::array<std::shared_ptr<ChNodeFEAfieldXYZ>, 2> nodes;
        };

        /// Optional: Drawing function to draw a ChFieldElementChemicalEdge finite element, for
        /// the 3D visualization of the finite element in real time rendering, postprocessing, etc.
        class ChDrawerChemicalEdge : public ChDrawer {
          public:
            virtual void IncrementBufferSizes(ChFieldElement& melement,
                                              size_t& num_vertexes,
                                              size_t& num_triangles,
                                              size_t& num_normals) const {
                num_vertexes += 2;
                num_triangles += 1;
                num_normals += 1;
            };
            virtual void UpdateBuffers(ChFieldElement& melement,
                                       ChTriangleMeshConnected& mmesh,
                                       size_t& vert_offset,
                                       size_t& tri_offset,
                                       size_t& norm_offset) const {
                // Setup triangles: vertex indexes per each triangle. Here 1 triangle collapsed to a segment.
                ChVector3i ivert_offset((int)vert_offset, (int)vert_offset, (int)vert_offset);
                mmesh.GetIndicesVertexes()[tri_offset] = ChVector3i(0, 1, 1) + ivert_offset;
                ChVector3i inorm_offset = ChVector3i((int)norm_offset, (int)norm_offset, (int)norm_offset);
                mmesh.GetIndicesNormals()[tri_offset] = ChVector3i(0, 0, 0) + inorm_offset;
                vert_offset += 2;
                tri_offset += 1;
                norm_offset += 1;
            }
        };

        // 3) Let's implement a MATERIAL for our domain 
        // 
        // Here: properties, constitutive equations, optional per-sample data. Assumed at each "sample" of the model. 
        // In a ChDomainIntegrating, that perform Gauss quadrature over a continuum, samples are made at each Gauss point. 
        // In a ChDomainGeneric, samples are made where needed by elements.

        class ChMaterialChemicalDiffusion : public ChMaterial {
          public:
            ChMaterialChemicalDiffusion() : diffusion_coefficient(1.) {}
            virtual ~ChMaterialChemicalDiffusion() {}
            
            // a property: the Fick' diffusion coefficient
            double diffusion_coefficient; // The Fick' diffusion coefficient

            // a constitutive equation:  the Fick law  in 1D
            double ComputeMolarFlux1D(double dc_dx) { return -diffusion_coefficient * dc_dx; }

            /// Implement this if the material needs custom data per material point,
            /// returning a std::make_unique<ChFieldDataCustom>()  where ChFieldDataCustom is
            /// your custom class with aditional states/properties per-point.
            virtual std::unique_ptr<ChFieldData> CreateMaterialPointData() const override{ return nullptr; }
        };
        

        // 4) Let's implement a new DOMAIN class
        //
        // Domains reference one or more field, and computes the physical laws that connect their variables.
        // In this case, we'll define a domain that connects the chemical field with the mechanical field, by saying
        // that the chemical concentration of some substance affects the Young modulus of the elastic material.

        // Tip: an auxiliary data stored per each material point during the ChDomainChemicalNetwork
        // computation. For example, scratch pad data to be plotted in postprocessing, etc.
        // If you need to append additional data per each matpoint, do not modify this, just
        // define your class with custom data and use it in my_material_class::T_per_materialpoint

        class ChFieldDataAuxiliaryChemical : public ChFieldDataNONE {
          public:
            double molar_flux;  /// molar flux [mol/(m^2 s)]
        }; 

        // Domain for FEA chemical analysis over the web of edges. 
        // It is based on a scalar field of chemical moles 
     
        class ChDomainChemicalNetwork : public ChDomainGeneric<std::tuple<ChFieldChemicalConcentration>,
                                                               ChFieldDataAuxiliaryChemical, ChElementDataKRM> {
          public:
            // The following just to provide a shortcut in type naming.
            using Base = ChDomainGeneric<std::tuple<ChFieldChemicalConcentration>,
                                         ChFieldDataAuxiliaryChemical,
                                         ChElementDataKRM>;
            using DataPerElement = typename Base::DataPerElement;

            /// Construct the domain
            ChDomainChemicalNetwork(std::shared_ptr<ChFieldChemicalConcentration> mfield) : Base(mfield) {
                // attach a default material to simplify user side
                material = chrono_types::make_shared<ChMaterialChemicalDiffusion>();
            }

            /// The material assigned to this domain
            std::shared_ptr<ChMaterialChemicalDiffusion> material;

            /// For a given finite element, computes the internal loads Fi and set values in the Fi vector.
            /// Since in this example we are NOT using quadrature-based domains, we must override this with our
            /// custom implementation. See ChDomainIntegrating vs ChDomainGeneric.
            virtual void ElementComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
                                                     DataPerElement& data,
                                                     ChVectorDynamic<>& Fi) {
                int numelcoords = this->GetNumPerNodeCoordsVelLevel() * melement->GetNumNodes();
                Fi.setZero(numelcoords);
                if (auto edge = std::dynamic_pointer_cast<ChFieldElementChemicalEdge>(melement)) {
                    double length = (*edge->GetNodeA() - *edge->GetNodeB()).Length();
                    int i_field = 0;  // we have only one field, so index is 0
                    double c_1 = data.nodes_data[0][i_field]->State()[0];
                    double c_2 = data.nodes_data[1][i_field]->State()[0];
                    double dc_dx = (c_2 - c_1) / length;
                    double J = material->ComputeMolarFlux1D(dc_dx);
                    Fi(0) = -J;  // flux at node A
                    Fi(1) = J;   // flux at node B
                    data.matpoints_data_aux[0].molar_flux = J; // store scratch data it so that it can be plotted
                }
            }

            /// For a given finite element, computes matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled
            /// sum of the tangent matrices M,R,K,: H = Mfactor*M + Rfactor*R + Kfactor*K. 
            /// Since in this example we are NOT using quadrature-based domains, we must override this with our
            /// custom implementation. See ChDomainIntegrating vs ChDomainGeneric.
            virtual void ElementComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement,
                                                   DataPerElement& data,
                                                   ChMatrixRef H,
                                                   double Kfactor,
                                                   double Rfactor = 0,
                                                   double Mfactor = 0) {
                if (auto edge = std::dynamic_pointer_cast<ChFieldElementChemicalEdge>(melement)) {
                    double length = (*edge->GetNodeA() - *edge->GetNodeB()).Length();
                    // diffusion 
                    H(0, 0) = Kfactor * material->diffusion_coefficient / length;   // -dFi(0)/dc(0)
                    H(0, 1) = Kfactor * -material->diffusion_coefficient / length;  // -dFi(0)/dc(1)
                    H(1, 0) = Kfactor * -material->diffusion_coefficient / length;  // -dFi(1)/dc(0)
                    H(1, 1) = Kfactor * material->diffusion_coefficient / length;   //- dFi(1)/dc(1)
                    // capacity (diagonal ones)
                    H(0, 0) += Rfactor * 1;   
                    H(1, 1) += Rfactor * 1;  
                }
            }

            /// Invoked at the end of each time step. If the material has some
            /// custom handling of auxiliary data (states etc.) it will manage this in ComputeUpdateEndStep()
            virtual void PointUpdateEndStep(std::shared_ptr<ChFieldElement> melement,
                                            DataPerElement& data,
                                            const int i_point,
                                            const double time) override {
                // DO NOTHING because we assume that thermal materials never implement some
                // material->ComputeUpdateEndStep(...)
            }

          protected:
            /// Get the material of the domain.
            virtual std::shared_ptr<ChMaterial> GetMaterial() override { return material; };

          public:

            // EXTRACTORS, optional. For drawing stuff in postprocessors/visualization:

            class ExtractMolarFlux : public ChVisualDataExtractorScalar<ExtractMolarFlux,
                                                                           ChFieldDataAuxiliaryChemical,
                                                                           DataAtMaterialpoint> {
                virtual double ExtractImpl(const ChFieldDataAuxiliaryChemical* fdata) const override {
                    return fdata->molar_flux;
                }
            };
        };


        // Ok, that's all.
        // Now we can proceed to create the model, as usual.

        // Create a Chrono physical system
        ChSystemNSC sys;

        // FIELD
        // Create the "field", that is a collection of scalar or vectorial properties at
        // every discretization point. In this case, the chemical problem requires a field of "moles" of substances:

        auto chemical_field = chrono_types::make_shared<ChFieldChemicalConcentration>();
        sys.Add(chemical_field);

        // DOMAIN
        // Create the "domain", which is a collection of finite elements operating over 
        // some field(s). In this case we create a ChDomainChemicalNetwork, field: 

        auto chemical_domain = chrono_types::make_shared<ChDomainChemicalNetwork>(chemical_field);
        sys.Add(chemical_domain);

        // MATERIAL
        // Depending on the type of domain, you can set some properties for the material

        auto chemical_material = chrono_types::make_shared<ChMaterialChemicalDiffusion>();
        chemical_domain->material = chemical_material;  // set the material in domain
        chemical_material->diffusion_coefficient = 0.8;


        // CREATE SOME FINITE ELEMENTS AND NODES
        // The web of chemical channells has a bunch of ChNodeFEAfieldXYZ as nodes.
        // Their xyz position in 3D space is irrelevant for the 1D chemical problem at
        // hand, but we initialize xyz values because could be useful for visualization.
        
        auto node1 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0,   0,   0));
        auto node2 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0.1, 0,   0));
        auto node3 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0.1, 0.1, 0));
        auto node4 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0,   0.1, 0));
        chemical_field->AddNode(node1);
        chemical_field->AddNode(node2);
        chemical_field->AddNode(node3);
        chemical_field->AddNode(node4);

        auto element1 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node1, node2);
        auto element2 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node2, node3);
        auto element3 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node3, node4);
        auto element4 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node4, node1);
        auto element5 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node1, node3);
        chemical_domain->AddElement(element1);
        chemical_domain->AddElement(element2);
        chemical_domain->AddElement(element3);
        chemical_domain->AddElement(element4);
        chemical_domain->AddElement(element5);
        
        // Set some initial conditions: the initial concentration at node1:
        chemical_field->NodeData(node1).Concentration() = 4.0;  // initial concentration = 4 mole per m^3 at node 1

        // Set some fixed node:
        //chemical_field->NodeData(node2).SetFixed(true);  // fixed concentration (the initial zero) at node 2

        // Set some fixed node with a time-varying concentration:
        auto sine_concentr = chrono_types::make_shared<ChFunctionSine>(1.5, 0.008, 0.0, 1.5);  // sinusoidal concentration  
        auto constraint_c2 = chrono_types::make_shared<ChLinkField>();
        constraint_c2->Initialize(node2, chemical_field);
        constraint_c2->SetOffset(sine_concentr);
        sys.Add(constraint_c2);


        // POSTPROCESSING & VISUALIZATION (optional)

        class ExtractConcentration : public ChVisualDataExtractorScalar<
                               ExtractConcentration,  // (just repeat the class name here - used for CRTP to generate clone() automatically)
                               ChFieldDataChemicalConcentration,  // here put the class of the data that you want to fetch!
                               DataAtNode>  // flag - choose one option between: DataAtNode  or DataAtMaterialpoint
        {
            virtual double ExtractImpl(const ChFieldDataChemicalConcentration* fdata) const override {
                return const_cast<ChFieldDataChemicalConcentration*>(fdata)->Concentration();
            }
        };

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(chemical_domain);
        visual_nodes->SetGlyphsSize(0.01);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractConcentration(), 0.0, 3.0, "Concentration");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        chemical_domain->AddVisualShape(visual_nodes);
        
        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(chemical_domain);
        visual_mesh->GetElementDispatcher().RegisterDrawer<ChFieldElementChemicalEdge>(std::make_unique<ChDrawerChemicalEdge>());
        visual_mesh->AddPositionExtractor(ExtractPos());
        //visual_mesh->AddPropertyExtractor(ExtractConcentration(), 0.0, 3.0, "Concentration");
        visual_mesh->AddPropertyExtractor(ChDomainChemicalNetwork::ExtractMolarFlux(), -4.0, 4.0, "Molar flux");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET)); //ChColor(0, 1, 0));
        visual_mesh->SetWireframe(true);
        chemical_domain->AddVisualShape(visual_mesh);
        
        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test custom FEA multiphysics");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 0.18, -0.18));
        vis->AddTypicalLights();
        
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);

        // Simulation loop
        double timestep = 0.01;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }

    }




    if (false) {

        // A coupled thermo-elastic problem, where chrono solves at the same time
        // both broblems: the elastic (deformation) and the thermal (heat conduction).
        // Because of the thermal expansion of the solid, the 3D slab will deform when
        // heat is applied on a surface. 
        
        // Create a Chrono physical system
        ChSystemNSC sys;

        // FIELD
        // 
        // Create two "fields", because here we will solve at the same time the 
        // temperature and the deformation of a solid:

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();
        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();
        sys.Add(temperature_field);
        sys.Add(displacement_field);

        // DOMAIN
        // 
        // Create a "domain", which is a collection of finite elements operating over 
        // some field. In this case we create a ChDomainThermal, that must operate on
        // two fields: the temperature field and the displacement field.
        // Note that you can look at the implememtation of the ChDomainThermoDeformation
        // and you can implement other types of coupled domains according to your needs (ex.
        // thermo-chemical, or thermo-electric, piezoelectric, etc.)

        auto domain = chrono_types::make_shared <ChDomainThermoDeformation>(temperature_field, displacement_field);
        sys.Add(domain);

        // MATERIAL
        // 
        // The ChDomainThermoDeformation  uses a material of ChMaterial3DThermalStress type. 
        // The default ChMaterial3DThermalStress that is already in the domain, is made with two 
        // components: the material for the stress problem (which could be whatever ChMaterial3DStress model 
        // like Ogden or StVenant) and the material for the elastic problem. Then you can also set the 
        // thermal expansion coefficient, that causes the coupling. Example:

        auto thermal_material = chrono_types::make_shared<ChMaterial3DThermalNonlinear>();
        thermal_material->SetDensity(1000);
        thermal_material->SetSpecificHeatCapacity(1.11);
        auto c_T = chrono_types::make_shared<ChFunctionInterp>();
        c_T->AddPoint(0, 0.16);
        c_T->AddPoint(250, 0.17);
        c_T->AddPoint(300, 0.11);
        thermal_material->SetThermalConductivity(c_T);

        domain->material_thermalstress->material_thermal = thermal_material;

        auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(6e6);
        elastic_material->SetPoissonRatio(0.3);

        domain->material_thermalstress->material_stress = elastic_material;

        domain->material_thermalstress->SetThermalExpansionCoefficient(230 * 10e-6);

        // CREATE FINITE ELEMENTS AND NODES
        // 
        // Build a test volume discretized with a regular grid of finite elements.

        ChBuilderVolumeBox builder;
        builder.BuildVolume( ChFrame<>(),
            5, 3, 5,        // N of elements in x,y,z direction
            3, 0.5, 3);     // width in x,y,z direction

        // After Build(), the elements and the nodes must be added to domains and fields:
        for (auto& created_element : builder.elements.list())
            domain->AddElement(created_element);
        for (auto& created_node : builder.nodes.list()) {
            temperature_field->AddNode(created_node);
            displacement_field->AddNode(created_node);
        }


        // EXAMPLE INITIAL CONDITIONS (initial temperature of some nodes)

        temperature_field->NodeData(builder.nodes.at(0, 0, 4)).T() = 400;
        temperature_field->NodeData(builder.nodes.at(0, 1, 4)).T() = 400;

        // EXAMPLE DIRICHLET CONDITIONS (fixed positions of some nodes)
        for (auto mnode : builder.nodes.list()) {
            if (mnode->x() <= 0)
                displacement_field->NodeData(mnode).SetFixed(true);
        }


        // EXAMPLE OF SOME LOADS

        // First: loads must be added to "load containers",
        // and load containers must be added to your system
        auto load_container = chrono_types::make_shared<ChLoadContainer>();
        sys.Add(load_container);

        // - IMPOSED HEAT FLUX ON SURFACE
        // Create a face wrapper, an auxiliary object that references a face of an
        // element as a ChLoadableUV so that can receive a surface load affecting a field 
        auto exa_face = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(2, 0, 3), 2); // 2nd face of hexa is y down
        auto exa_face_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(exa_face, temperature_field);

        auto heat_flux = chrono_types::make_shared<ChLoaderHeatFlux>(exa_face_loadable);
        heat_flux->SetSurfaceHeatFlux(100); // the surface flux: heat in W/m^2
        load_container->Add(heat_flux);

        // - IMPOSED CONVECTION ON THE ENTIRE BOUNDARY OF VOLUME
        // Weìll use the ChDomainSurface to generate all faces of the moundary. Then, for all faces
        // create ChFieldElementLoadableSurface face wrappers to whom we can apply the convection load:
        auto outer_surface = chrono_types::make_shared<ChDomainSurface>(domain.get());
        outer_surface->AddFacesFromBoundary();
        for (auto msurf : outer_surface->GetFaces()) {
            auto exa_iface_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(msurf, temperature_field);
            auto heat_iconvection = chrono_types::make_shared<ChLoaderHeatConvection>(exa_iface_loadable, temperature_field);
            heat_iconvection->SetSurfaceConvectionCoeff(10);
            heat_iconvection->SetFluidTemperature(0);
            load_container->Add(heat_iconvection);
        }

        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(domain);
        visual_nodes->SetGlyphsSize(0.05);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        domain->AddVisualShape(visual_nodes);
        /*
        auto visual_matpoints = chrono_types::make_shared<ChVisualDomainGlyphs>(domain);
        visual_matpoints->SetGlyphsSize(0.1);
        visual_matpoints->glyph_scalelenght = 0.01;
        visual_matpoints->AddPositionExtractor(::ExtractPos());
        visual_matpoints->AddPropertyExtractor(ChDomainThermoDeformation::ExtractHeatFlux(), 0.0, 50, "q flux");
        visual_matpoints->SetColormap(ChColormap(ChColormap::Type::JET));
        domain->AddVisualShape(visual_matpoints);
        */
        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(domain);
        visual_mesh->AddPositionExtractor(ExtractPos());
        visual_mesh->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));
        visual_mesh->SetWireframe(true);
        domain->AddVisualShape(visual_mesh);

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


        // Simulation loop
        double timestep = 10;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }

    }



    return 0;

}
