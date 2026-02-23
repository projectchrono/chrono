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
#include "chrono/fea/ChBuilderVolume.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "FEAvisualization.h"

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

using namespace chrono;
using namespace chrono::fea;





int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;


    if (true) {
        
        // Example on how to develop your own multiphysics problem, that are not yet
        // available in the current Chrono API. 
        
        // Create a Chrono physical system
        ChSystemNSC sys;

        // Suppose we are solving a problem where the chemical concentration of some substance changes
        // according to soma law in a chemical field, and it also affects mechanical properties of some
        // other field, ex. elasticity.
        

        // 1) Let's implement a new data type for some field.
        //
        // We define data for a scalar field defining the concentration of some substance.
        // At each point in the discretized field, this is the data that we simulate.
        // Being a scalar, we inherit from ChFieldDataScalar. (btw. we could use directly 
        // ChFieldDataScalar in our ChDomain, but for readability we do the following, that
        // also add two helper functions Moles() etc.)

        class ChFieldDataChemicalMoles : public ChFieldDataScalar {
          public:
            double& Moles() { return State()[0]; }
            double& Moles_dt() { return StateDt()[0]; }
        };



        // 2) Let's implement a new field class.
        //
        // The field is based on the ChFieldDataChemicalMoles, i.e. per each node in the
        // discretization, we attach a ChFieldDataChemicalMoles.

        class ChFieldChemicalMoles : public ChField<ChFieldDataChemicalMoles> {};



        // 3) Let's implement a new element class.
        //
        // Suppose that the chemical field is like a network of capillary pipes, each 
        // edge of the network being an element. In the default Chrono API there's no such
        // element: let's do a minimal implementation.
        // Note that the element can be field-agnostic, and material-agnostic (although this 
        // may be customized in some special cases)
        
        class ChFieldElementChemicalEdge : public ChFieldElementLine {
          public:
            ChFieldElementChemicalEdge() { this->quadrature_order = 1; }
            virtual ~ChFieldElementChemicalEdge() {}

            /// Return the specified edge node (0 <= n <= 1).
            virtual std::shared_ptr<ChNodeFEAfieldXYZ> GetEdgeNode(unsigned int n) { return nodes[n]; }

            /// Set the nodes used by this edge.
            virtual void SetNodes(std::shared_ptr<ChNodeFEAfieldXYZ> nodeA,
                                  std::shared_ptr<ChNodeFEAfieldXYZ> nodeB);

            // Interfaces that must be implemented:

            /// Get the number of nodes used by this element.
            virtual unsigned int GetNumNodes() override { return 2; };

            /// Access the nth node.
            virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override { return nodes[n]; };

            // Return dimension of embedding space X
            virtual int GetSpatialDimensions() const override { return 3; };

            // Compute the shape function N at eta.x parametric coordinate.
            virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override { N(0) = 1. - eta.x(); N(1) = eta.x();};

            // Compute shape function material derivatives dN/d\eta 
            virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override { N(0,0) = -1; N(1,0) = 1;};

            // Compute Jacobian J, and returns its determinant. J is square 3x3
            //virtual double ComputeJ(const ChVector3d eta, ChMatrix33d& J) override;

            // Compute Jacobian Jinv, and returns its determinant. Jinv is square 3x3
            //virtual double ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv) override;

            // Tell how many Gauss points are needed for integration - NO NEED: THIS EXAMPLE DEMONSTRATES ELEMENTS WITHOUT QUADRATURE
            virtual int GetNumQuadraturePointsForOrder(const int order) const override { return 0; };

            // Get i-th Gauss point weight and parametric coordinates  - NO NEED: THIS EXAMPLE DEMONSTRATES ELEMENTS WITHOUT QUADRATURE
            virtual void GetQuadraturePointWeight(const int order,
                                                  const int i,
                                                  double& weight,
                                                  ChVector3d& coords) const override {};

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



        // 4) Let's implement a new domain class
        //
        // Domains reference one or more field, and define the physical laws that connect their variables.
        // In this case, we'll define a domain that connects the chemical field with the mechanical field, by saying
        // that the chemical concentration of some substance affects the Young modulus of the elastic material.

        // Tip: an auxiliary data stored per each material point during the ChDomainChemicalNetwork
        // computation. For example, scratch pad data to be plotted in postprocessing, etc.
        // If you need to append additional data per each matpoint, do not modify this, just
        // define your class with custom data and use it in my_material_class::T_per_materialpoint

        class ChFieldDataAuxiliaryChemical : public ChFieldDataNONE {
          public:
            double flux;  /// flux of moles
        }; 

        // Domain for FEA chemical analysis over the web of edges. 
        // It is based on a scalar field of chemical moles 
     
        class ChDomainChemicalNetwork
            : public ChDomainCustom<std::tuple<ChFieldChemicalMoles>, ChFieldDataAuxiliaryChemical, ChElementDataKRM> {
          public:
            // The following just to provide a shortcut in type naming.
            using Base =
                ChDomainCustom<std::tuple<ChFieldChemicalMoles>, ChFieldDataAuxiliaryChemical, ChElementDataKRM>;
            using DataPerElement = typename Base::DataPerElement;

            /// Construct the domain
            ChDomainChemicalNetwork(std::shared_ptr<ChFieldTemperature> mfield) : Base(mfield) {
                // attach a default material to simplify user side
                material = chrono_types::make_shared<ChMaterialChemicalMechanical>();
            }

            /// Thermal properties of this domain (conductivity,
            /// heat capacity constants etc.)
            std::shared_ptr<ChMaterial3DThermal> material;

            //
            // INTERFACES
            //

            /// Computes the internal loads Fi for one quadrature point, except quadrature weighting "...* w * |J|",
            /// and *ADD* the s-scaled result to Fi vector

            virtual void PointComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
                                                   DataPerElement& data,
                                                   const int i_point,
                                                   ChVector3d& eta,
                                                   const double s,
                                                   ChVectorDynamic<>& Fi) override {
                // Compute shape functions N at eta, and their material derivatives dNdX
                ChMatrixDynamic<> dNdX;
                ChRowVectorDynamic<> N;
                melement->ComputedNdX(eta, dNdX);
                melement->ComputeN(eta, N);

                // Compute the vector  T_h = [T_1, T_2, .. T_n] with discrete values of temperatures at nodes
                ChVectorDynamic<> T_h;
                this->GetFieldStateBlock(melement, T_h, 0);

                // B = dNdX // lucky case of thermal problem: no need to build B in \nabla_x T(x) = B * T_h because B is
                // simply dNdX

                // We have:  Fi_tot = sum (dNdX' * q_flux * w * |J|) * s
                //    with   q_flux = - k * \nabla_x T(x)
                //                  = - k * dNdX * T
                //    also   Fi_tot = - K * T * s;
                //    with        K = sum (dNdX' * k * dNdX * w * |J|)
                // We need to return   Fi in   Fi_tot = sum (Fi * w * |J|)
                //
                // so we compute  Fi += -(dNdX' * k * dNdX * T) * s
                //           or   Fi += dNdX' * q_flux * s

                // Temperature at point  (might be needed by nonlinear ChMaterial3DThermal materials with dependence on
                // T)
                double T = N * T_h;

                // Gradient of temperature
                ChVector3d T_grad = dNdX * T_h;  //  = \nabla_x T(x)

                // Heat flux.
                // (For a linearixed thermal material, this is q_flux = - [k] * T_grad; with [k] conductivity matrix.)
                ChVector3d q_flux;
                this->material->ComputeHeatFlux(
                    q_flux, T_grad, T, data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
                    &data.element_data);

                // To the discrete coordinates:
                //   Fi += dNdX' * q_flux * s
                Fi += dNdX.transpose() * q_flux.eigen() * s;  // += dNdX' * q_flux * s

                // Store auxiliary data in material point data (ex. for postprocessing)
                data.matpoints_data_aux[i_point].q_flux = q_flux;
            }

            /// Sets matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices
            /// M,R,K,: H = Mfactor*M + Rfactor*R + Kfactor*K. Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used
            /// to get just mass matrix, etc. Done here in incremental form, as H += ... Also: compute matrix *except*
            /// quadrature weighting "...* w * |J|"

            virtual void PointComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement,
                                                 DataPerElement& data,
                                                 const int i_point,
                                                 ChVector3d& eta,
                                                 ChMatrixRef H,
                                                 double Kpfactor,
                                                 double Rpfactor = 0,
                                                 double Mpfactor = 0) override {
                ChMatrixDynamic<> dNdX;
                ChRowVectorDynamic<> N;
                melement->ComputedNdX(eta, dNdX);
                melement->ComputeN(eta, N);

                // Temperature at point (might be needed by nonlinear ChMaterial3DThermal materials with dependence on
                // T) Compute the vector  T_h = [T_1, T_2, .. T_n] with discrete values of temperatures at nodes
                ChVectorDynamic<> T_h;
                this->GetFieldStateBlock(melement, T_h, 0);
                double T = N * T_h;

                // B = dNdX // in the lucky case of thermal problem, no need to build B because B is simply dNdX

                // K  matrix (jacobian of:    c dT/dt + div [C] grad T = f )
                // K = sum (dNdX' * [k] * dNdX * w * |J|)

                if (Rpfactor) {
                    ChMatrix33d tangent_conductivity;
                    this->material->ComputeTangentModulus(
                        tangent_conductivity, VNULL, T,
                        data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr, &data.element_data);

                    H += Kpfactor * (dNdX.transpose() * tangent_conductivity * dNdX);  // H += Kpfactor * (B' * [k] * B)
                }

                // R  matrix : (jacobian d / d\dot(T) of:    (c*rho) * dT/dt + div [C]*grad T = f)
                // R = sum ( N' * N * (c*rho) * w * |J|)

                if (Rpfactor) {
                    double c_rho;
                    this->material->ComputeDtMultiplier(
                        c_rho, T, data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
                        &data.element_data);
                    H += Rpfactor * c_rho * (N.transpose() * N);  // H += Rpfactor  * (N' * N) * (c*rho)
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
            //
            // EXTRACTORS for drawing stuff in postprocessors/visualization:
            //

            class ExtractHeatFlux : public ChVisualDataExtractorVector<ExtractHeatFlux,
                                                                       ChFieldDataAuxiliaryThermal,
                                                                       DataAtMaterialpoint> {
                virtual ChVector3d ExtractImpl(const ChFieldDataAuxiliaryThermal* fdata) const override {
                    return fdata->q_flux;
                }
            };
        };




        // FIELD
        // 
        // Create a "field", that is a collection of scalar or vectorial properties at
        // every discretization point. In this case, we need a field of xyz displacement 
        // per each point:

        auto displacement_field = chrono_types::make_shared <ChFieldDisplacement3D>();
        sys.Add(displacement_field);

        // DOMAIN
        // 
        // Create a "domain", which is a collection of finite elements operating over 
        // some field. In this case we create a ChDomainDeformation, referencing the displacement field: 

        auto elastic_domain = chrono_types::make_shared <ChDomainDeformation>(displacement_field);
        sys.Add(elastic_domain);

        // This means that all finite elements of the domain, if solid, will automatically receive 
        // a gravitational load:

        elastic_domain->SetAutomaticGravity(true);

        // MATERIAL
        // 
        // Depending on the type of domain, you can set some properties for the material 
        // to be used for the FEA. In this case we create an elastic meaterial, which 
        // for small deformations acts like linear elasticity.

        auto elastic_material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        elastic_domain->material = elastic_material; // set the material in domain
        elastic_material->SetDensity(1000);
        elastic_material->SetYoungModulus(3e6);
        elastic_material->SetPoissonRatio(0.3);


        // CREATE FINITE ELEMENTS AND NODES
        //
        // You could create your finite elements one by one by yourself, however here for 
        // conciseness we use a ChBuilderVolumeBox helper:

        // Build a test volume discretized with a regular grid of finite elements.
        ChBuilderVolumeBoxTetra  builder;
        builder.BuildVolume( ChFrame<>(),
            8, 3, 3,            // N of elements in x,y,z direction
            3, 0.5, 0.5);       // width in x,y,z direction

        // After Build(), the elements and the nodes must be added to domains and fields:
        for (auto& created_elementc : builder.elements.list())
            for (auto& created_element : created_elementc)
                elastic_domain->AddElement(created_element);
        for (auto& created_node : builder.nodes.list())
            displacement_field->AddNode(created_node);

        
        // Set atomic force on some node.
        // Note that the ChNodeFEAfieldXYZ created in the volume builder are generic nodes that
        // do not contain the xyz position, or speed, or applied atomic forces, because those properties
        // are attached by means of the displacement_field. 
        // So the way to access the position, force etc is:
        // 
        // displacement_field->NodeData(builder.nodes.at(4, 0, 0)).SetLoad(ChVector3d(0,1000,0));
        
        // Set initial position if different from default:
        //displacement_field->NodeData(builder.nodes.at(1, 1, 0)).SetPos(displacement_field->NodeData(builder.nodes.at(1, 1, 0)).GetPos() + ChVector3d(0, 0.1, 0));
        //displacement_field->NodeData(builder.nodes.at(1, 1, 1)).SetPos(displacement_field->NodeData(builder.nodes.at(1, 1, 1)).GetPos() + ChVector3d(0, 0, 0));
        
        // Set some node to fixed:
        for (auto mnode : builder.nodes.list()) {
            if (mnode->x() <= 0)
                displacement_field->NodeData(mnode).SetFixed(true);
        }

  
        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_nodes->SetGlyphsSize(0.1);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractPosDt(), 0.0, 2.0, "Vel");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        elastic_domain->AddVisualShape(visual_nodes);

        auto visual_stress = chrono_types::make_shared<ChVisualDomainGlyphs>(elastic_domain);
        visual_stress->SetGlyphsSize(2.2);
        visual_stress->AddPositionExtractor(ExtractPos());
        visual_stress->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain(), 0.0, 2.0, "e");
        visual_stress->SetColormap(ChColormap(ChColormap::Type::JET));
        elastic_domain->AddVisualShape(visual_stress);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh->AddPositionExtractor(ExtractPos());
        visual_mesh->AddPropertyExtractor(ExtractPosDt(), 0.0, 2.0, "Vel");
        visual_mesh->SetColormap(ChColor(0, 1, 0));
        visual_mesh->SetWireframe(true);
        visual_mesh->SetShrinkElements(true, 0.9);
        //elastic_domain->AddVisualShape(visual_mesh);

        auto visual_mesh2 = chrono_types::make_shared<ChVisualDomainMesh>(elastic_domain);
        visual_mesh2->AddPositionExtractor(ExtractPos());
        visual_mesh2->AddPropertyExtractor(ChDomainDeformation::ExtractEulerAlmansiStrain().VonMises(), -0.1, 0.1, "Stretch");
        visual_mesh2->SetColormap(ChColormap(ChColormap::Type::JET));
       // visual_mesh2->SetWireframe(true);
        visual_mesh2->SetShrinkElements(true, 0.9);
        elastic_domain->AddVisualShape(visual_mesh2);

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

        // Introduction to the multiphysics FEA system of Chrono. 
        // A transient heat problem.
        // This example shows how to setup a thermal problem including
        // some boundary conditions, like radiative heat loss, convection,
        // enforced heat flux on a surface, volumetric heat flux, etc.
        
        // Create a Chrono physical system
        ChSystemNSC sys;

        // FIELD
        // 
        // Create a "field", that is a collection of scalar or vectorial properties at
        // every discretization point. In this case, we need a scalar field, with T temperature 
        // per each point:

        auto temperature_field = chrono_types::make_shared <ChFieldTemperature>();
        sys.Add(temperature_field);

        // DOMAIN
        // 
        // Create a "domain", which is a collection of finite elements operating over 
        // some field. In this case we create a ChDomainThermal, referencing the temperature field: 

        auto thermal_domain = chrono_types::make_shared <ChDomainThermal>(temperature_field);
        sys.Add(thermal_domain);

        // MATERIAL
        // 
        // For the transient heat problem we must provide settings of: specific heat capacity, 
        // density, thermal conductivity. If we use ChMaterial3DThermalLinear, those properties
        // are constant, but here we use ChMaterial3DThermalNonlinear instead, so we can showcase
        // an example where the thermal conductivity is a ChFunction of temperature c=c(T), here defined
        // using a piecewise interpolation of three T-c pairs.

        auto thermal_material = chrono_types::make_shared<ChMaterial3DThermalNonlinear>();
        thermal_material->SetDensity(1000);
        thermal_material->SetSpecificHeatCapacity(1.11);
        auto c_T = chrono_types::make_shared<ChFunctionInterp>();
        c_T->AddPoint(0, 0.16);
        c_T->AddPoint(250, 0.17);
        c_T->AddPoint(300, 0.11);
        thermal_material->SetThermalConductivity(c_T);
        thermal_domain->material = thermal_material;

        // CREATE FINITE ELEMENTS AND NODES
        // 
        // Build a test volume discretized with a regular grid of finite elements.

        ChBuilderVolumeBox builder;
        builder.BuildVolume( ChFrame<>(),
            5, 1, 5,        // N of elements in x,y,z direction
            3, 0.5, 3);     // width in x,y,z direction

        // After Build(), the elements and the nodes must be added to domains and fields:
        for (auto& created_element : builder.elements.list())
            thermal_domain->AddElement(created_element);
        for (auto& created_node : builder.nodes.list())
            temperature_field->AddNode(created_node);

        

        // EXAMPLE INITIAL CONDITIONS (initial temperature of some nodes)

        temperature_field->NodeData(builder.nodes.at(0, 0, 4)).T() = 400;
        temperature_field->NodeData(builder.nodes.at(0, 1, 4)).T() = 400;

        // EXAMPLE DIRICHLET CONDITIONS (fixed temperature of some nodes)

        temperature_field->NodeData(builder.nodes.at(0, 0, 0)).SetFixed(true);
        temperature_field->NodeData(builder.nodes.at(0, 0, 0)).T() = 100;
        temperature_field->NodeData(builder.nodes.at(0, 1, 0)).SetFixed(true);
        temperature_field->NodeData(builder.nodes.at(0, 1, 0)).T() = 100;
        temperature_field->NodeData(builder.nodes.at(0, 0, 1)).SetFixed(true);
        temperature_field->NodeData(builder.nodes.at(0, 0, 1)).T() = 100;
        temperature_field->NodeData(builder.nodes.at(0, 1, 1)).SetFixed(true);
        temperature_field->NodeData(builder.nodes.at(0, 1, 1)).T() = 100;
        
        // APPLY SOME LOADS

        // First: loads must be added to "load containers",
        // and load containers must be added to your system
        auto load_container = chrono_types::make_shared<ChLoadContainer>();
        sys.Add(load_container);

        // - IMPOSED HEAT FLUX ON SURFACE
        // Create a face wrapper, an auxiliary object that references a face of an
        // element as a ChLoadableUV so that can receive a surface load affecting a field 
        auto exa_face = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(0,0,4), 3); // 3rd face of hexa is y up
        auto exa_face_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(exa_face, temperature_field);

        auto heat_flux = chrono_types::make_shared<ChLoaderHeatFlux>(exa_face_loadable);
        heat_flux->SetSurfaceHeatFlux(100); // the surface flux: heat in W/m^2
        load_container->Add(heat_flux);

        // - IMPOSED HEAT SOURCE ON VOLUME
        // Create a volume wrapper, an auxiliary object that references a volume of an
        // element as a ChLoadableUVW so that it can receive a volume load affecting a field 
        auto exa_volume_loadable = chrono_types::make_shared<ChFieldElementLoadableVolume>(builder.elements.at(4, 0, 4), temperature_field);

        auto heat_source = chrono_types::make_shared<ChLoaderHeatVolumetricSource>(exa_volume_loadable);
        heat_source->SetVolumeHeatFlux(300); // the volumetric source flux: heat in W/m^3
        load_container->Add(heat_source);

        // - THERMAL RADIATION (Stefan-Boltzmann boundary condition)
        // Create a volume wrapper, an auxiliary object that references a volume of an
        // element as a ChLoadableUVW so that it can receive a volume load affecting a field 
        auto exa_face2 = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(4, 0, 0), 3); // 3rd face of hexa is y up
        auto exa_face2_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(exa_face2, temperature_field);

        auto heat_radiate = chrono_types::make_shared<ChLoaderHeatRadiation>(exa_face2_loadable, temperature_field);
        heat_radiate->SetEnvironmentTemperature(0); // the surface flux: heat in W/m^2
        load_container->Add(heat_radiate);

        // - THERMAL CONVECTION 
        auto exa_face3 = chrono_types::make_shared<ChFieldHexahedron8Face>(builder.elements.at(4, 0, 0), 3); // 3rd face of hexa is y down
        auto exa_face3_loadable = chrono_types::make_shared <ChFieldElementLoadableSurface>(exa_face3, temperature_field);

        auto heat_convection = chrono_types::make_shared<ChLoaderHeatConvection>(exa_face3_loadable, temperature_field);
        heat_convection->SetSurfaceConvectionCoeff(3);
        heat_convection->SetFluidTemperature(50);
        load_container->Add(heat_convection);


        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(thermal_domain);
        visual_nodes->SetGlyphsSize(0.05);
        visual_nodes->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        thermal_domain->AddVisualShape(visual_nodes);

        auto visual_matpoints = chrono_types::make_shared<ChVisualDomainGlyphs>(thermal_domain);
        visual_matpoints->SetGlyphsSize(0.1);
        visual_matpoints->glyph_scalelenght = 0.01;
        visual_matpoints->AddPropertyExtractor(ChDomainThermal::ExtractHeatFlux(), 0.0, 50, "q flux");
        visual_matpoints->SetColormap(ChColormap(ChColormap::Type::JET));
        thermal_domain->AddVisualShape(visual_matpoints);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(thermal_domain);
        visual_mesh->AddPropertyExtractor(ExtractTemperature(), 0.0, 500, "Temp");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));
        visual_mesh->SetWireframe(true);
        thermal_domain->AddVisualShape(visual_mesh);

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
        double timestep = 50;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }

    }





    if (true) {

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

        auto visual_matpoints = chrono_types::make_shared<ChVisualDomainGlyphs>(domain);
        visual_matpoints->SetGlyphsSize(0.1);
        visual_matpoints->glyph_scalelenght = 0.01;
        visual_matpoints->AddPositionExtractor(::ExtractPos());
        visual_matpoints->AddPropertyExtractor(ChDomainThermoDeformation::ExtractHeatFlux(), 0.0, 50, "q flux");
        visual_matpoints->SetColormap(ChColormap(ChColormap::Type::JET));
        domain->AddVisualShape(visual_matpoints);

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
