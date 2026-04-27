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
// Examples on how to develop your own multiphysics problem, that are not yet
// available in the current Chrono API
// Demonstrates how to implement a custom ChDomain for a custom problem, with 
// a custom ChField and ChFieldData.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/multiphysics/ChDomainDeformation.h"
#include "chrono/fea/multiphysics/ChDomainThermal.h"
#include "chrono/fea/multiphysics/ChDomainThermoDeformation.h"
#include "chrono/fea/multiphysics/ChMaterial3DThermalNonlinear.h"
#include "chrono/fea/multiphysics/ChDrawer.h"
#include "chrono/fea/multiphysics/ChSurfaceOfDomain.h"
#include "chrono/fea/multiphysics/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/multiphysics/ChFieldElementLoadableVolume.h"
#include "chrono/fea/multiphysics/ChFieldElementLoadableSurface.h"
#include "chrono/fea/multiphysics/ChLoaderHeatFlux.h"
#include "chrono/fea/multiphysics/ChLoaderHeatRadiation.h"
#include "chrono/fea/multiphysics/ChLoaderHeatVolumetricSource.h"
#include "chrono/fea/multiphysics/ChLoaderHeatConvection.h"
#include "chrono/fea/multiphysics/ChLinkFieldNode.h"
#include "chrono/fea/multiphysics/ChBuilderVolume.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "FEAvisualization.h"

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

using namespace chrono;
using namespace chrono::fea;




int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    //
    // EXAMPLE 1
    //
    
    // Example on how to develop your own multiphysics problem, that is not yet
    // available in the current Chrono API.
    //
    // Suppose we are solving a problem where the chemical concentration "c" of some substance changes
    // according to the 3D Fick law:
    //
    //    dc/dt = D *  div [D]*grad c 
    //
    // with D the diffusion coefficient.
    // In this example, we will show how to implement a custom ChDomain for this problem, 
    // with a custom ChField and ChFieldData.
    
   
    // 1) Let's implement a new FIELD DATA for some field.
    //
    // We define data for a scalar field defining the concentration of some substance.
    // This is expressed as concentration c [mol/m^3], i.e. moles per volume.
    // At each point in the discretized field, this is the data that we simulate.
    // Being a scalar, we inherit from ChFieldDataScalar. (btw. we could use directly
    // a ChFieldDataScalar in our ChDomain, but for readability we do the following, that
    // also add two helper functions Concentration() etc.)
    // Btw. If this is used in 1D elements, concentration c [mol/m^3] is related to moles
    // per length n=[mol/m] as c=n/A, with A cross section.

    class ChFieldDataChemicalConcentration : public ChFieldDataScalar {
      public:
        /// Most ChFieldDataState are meant for 2nd order ODE, i.e. they use both State() and StateDt(), 
        /// ex.{pos, dpos/dt}. In this demo, however, the Fick law is 1st order ODE: so we do this:
        /// concentration will be the StateDt(), and State() is just a dummy variable to provide
        /// compatibility with 2nd order integrators of chrono. Note: we must flag this as 1st order via this:
        static bool IsFirstOrder() { return true; }

        double& _dummy_()       { return State()[0]; }  // dummy variable, not used
        double& Concentration() { return StateDt()[0]; }
    };

    // 2) Let's implement a new FIELD class.
    //
    // The field is based on the ChFieldDataChemicalConcentration, i.e. per each node in the
    // discretization, we attach a ChFieldDataChemicalConcentration.

    class ChFieldChemicalConcentration : public ChField<ChFieldDataChemicalConcentration> {};


    // 3) Let's implement a new DOMAIN.
    //
    // Domains reference one or more field, and computes the physical laws that connect their variables.
    // Here we implement the domain for the Fick chemical diffusivity problem in 3D. 
    // It is based on a scalar concentration field, that is ChFieldChemicalConcentration.
    // To simplify things, this domain do not use a pluggable material object: the diffusivity constant
    // is just a public data member of this domain (for advanced approach to materials, look later Example 2). 
    // Since we'll use tetrahedrons/hexahedron volume elements, we'll inherit from ChDomainIntegrating
    // and we'll implement PointComputeInternalLoads() and PointComputeKRMmatrices().

    class ChDomainChemical3D : public ChDomainIntegrating<
        std::tuple<ChFieldChemicalConcentration>, // the field(s) used by this domain, in this case just one.
        ChFieldDataNONE,                          // data stored at material points as persistent scratch data
        ChElementDataKRM> {                       // data stored at element level: default K,R,M matrices. 
      public:
        // The following just to provide a shortcut in type naming.
        using Base = ChDomainIntegrating<std::tuple<ChFieldChemicalConcentration>, ChFieldDataNONE, ChElementDataKRM>;
        using DataPerElement = typename Base::DataPerElement;

        /// Construct the domain
        ChDomainChemical3D(std::shared_ptr<ChFieldChemicalConcentration> mfield) : Base(mfield) {
            diffusivity = 1.0;
        }

        /// Get the material of the domain (not needed)
        virtual std::shared_ptr<ChMaterial> GetMaterial() override { return nullptr; };

        /// INTERFACE that must be implemented!
        /// Computes the internal loads Fi for one quadrature point, except quadrature weighting "...* w * |J|",
        /// and *ADD* the s-scaled result to Fi vector

        virtual void PointComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
                                               DataPerElement& data,
                                               const int i_point,
                                               ChVector3d& eta,
                                               const double s,
                                               ChVectorDynamic<>& Fi) override {

            // With concentration c and diffusivity D, molar flux in 3D Fick law is
            //      flux = - D * \nabla_x c
            // With discrete c_h, we need B in \nabla_x c = B * c_h, but luckily B is simply dNdX,  hence   
            //      flux = - D * dNdX * c_h
            // From the weak form of the PDE we have, summing over all integration points:
            //    Fi_tot = sum (dNdX' * flux * w * |J|) * s
            // so here we must compute
            //       Fi += -(dNdX' * flux) * s  
            
            // Compute shape functions N at eta, and their material derivatives dNdX
            ChMatrixDynamic<> dNdX;
            melement->ComputedNdX(eta, dNdX);

            // Compute the vector  c_h = [c_1, c_2, .. c_n] with discrete values of chemical concentrations at nodes.
            // Note: we use GetFieldStateBlockDt (note the ..Dt!) not the usual GetFieldStateBlock because 1st order ODE, 
            // where "c" is assumed in StateDt(), and State() is just a dummy variable.
            ChVectorDynamic<> c_h;
            this->GetFieldStateBlockDt(melement, c_h, 0); 

            // Gradient of concentration at material point 
            ChVector3d c_grad = dNdX * c_h;  //  = \nabla_x c(x)

            // Molar flux.
            ChVector3d flux = -this->diffusivity * c_grad.eigen();

            // To the discrete coordinates:
            //   Fi += dNdX' * flux * s
            Fi += dNdX.transpose() * flux.eigen() * s;  // += dNdX' * q_flux * s
        }

        /// INTERFACE that must be implemented!
        /// Sets matrix H += Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
        /// H += Mfactor*M + Rfactor*R + Kfactor*K.
        /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
        /// Done here in incremental form, as H += ... Also: compute matrix *except* quadrature weighting "...* w * |J|"

        virtual void PointComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement,
                                             DataPerElement& data,
                                             const int i_point,
                                             ChVector3d& eta,
                                             ChMatrixRef H,
                                             double Kpfactor,
                                             double Rpfactor = 0,
                                             double Mpfactor = 0) override {
            ChRowVectorDynamic<> N;
            melement->ComputeN(eta, N);
            ChMatrixDynamic<> dNdX;
            melement->ComputedNdX(eta, dNdX);

            // R  matrix (for the 1st order ODE, so d/dc jacobian of: dc/dt + div [D] grad c = f )
            // R = sum (dNdX' * [D] * dNdX * w * |J|)
            if (Rpfactor) {
                ChMatrix33d tangent_diffusivity;
                tangent_diffusivity.setZero();
                tangent_diffusivity.fillDiagonal(this->diffusivity);
                H += Rpfactor * (dNdX.transpose() * tangent_diffusivity * dNdX);  // H += Kpfactor * (B' * [D] * B)
            }

            // M  matrix : (for the 1st order ODE, so jacobian d/d\dot(c) of: dc/dt + div [D]*grad c = f)
            // M = sum ( N' * N * w * |J|)
            if (Mpfactor) {
                ChMatrixDynamic<> Hi;
                Hi = Mpfactor * (N.transpose() * N);  // H += Rpfactor  * (N' * N) 
               
                // for diffusion, mass matrix is more realistic if a linear combination of lumped and consistent
                // H = theta * Hi + (1 - theta) * diag(lumped_diag)
                double theta = 0.5;  
                ChVectorDynamic<> lumped_diag = Hi.rowwise().sum();
                H += theta * Hi;
                H.diagonal() += (1.0 - theta) * lumped_diag;
            }
        }
 
        double diffusivity;
    };




    ////////////////////////////////////////////////////////////////////////////////////////////////

    //
    // EXAMPLE 2
    //
    
    // Ok, in Example 1 we already introduced all classes that would allow us the simulation of Fick
    // chemical diffusivity in 3D, but what if we need also 1D diffusivity in a network of capillary
    // pipes? Here is how to do. 
    // Note that we skip implementing the FIELD and FIELD DATA because the 
    // 1D domain will be based on the already defined field ((scalar value of concentration, one 
    // scalar per node) through the already defined classes ChFieldChemicalConcentration and 
    // ChFieldDataChemicalConcentration of example above.
    // We need to create a class for "edge" ELEMENT and the "1D" case of chemical DOMAIN. 
   

    // 1) Let's implement a new ELEMENT class.
    //
    // Suppose that the chemical field is like a network of 1D capillary pipes, each 
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
        // - Note: this element does not require quadrature, so this function would be superfluous, but useful for postprocessing, drawing, etc.
        virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override { N.resize(GetNumNodes()); N(0) = 1. - eta.x(); N(1) = eta.x(); };

        // Compute shape function material derivatives dN/d\eta 
        virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override { dNde.setZero(GetManifoldDimensions(), GetNumNodes()); dNde(0,0) = -1; dNde(1,0) = 1;};

        // Tell how many material points are needed
        // - NOTE: THIS EXAMPLE DEMONSTRATES ELEMENTS WITHOUT QUADRATURE, BUT WE'LL RETURN "1" SO THAT 
        //   THE DOMAIN WILL ATTACH ONE MATERIAL DATA OBJECT TO THE ELEMENT  
        virtual int GetNumQuadraturePointsForOrder(const int order) const override { return 1; }

        // Get i-th material point weight and parametric coords  
        // - Note: this element does not require quadrature, so this function would be superfluous, but we implement it
        //   anyway because the (optional) postprocessing system that extrapolates per-material-point quantities for painting this edge on
        //   the screen requires a meaningful value telling where the material point is located (here 0.5 means: half length of edge). 
        virtual void GetMaterialPointWeight(const int order,
                                                const int i,
                                                double& weight,
                                                ChVector3d& coords) const override { weight = 1.0; coords={0.5,0,0};};

        /// Update, called each time the state changes, even multiple times per time step. In our case we do nothing.
        virtual void Update() override {}

        private:
        /// Initial setup (called once before start of simulation).
        /// This is used mostly to precompute matrices that do not change during the simulation, i.e. the local
        /// stiffness of each element, if any, the mass, etc. In our case we do nothing.
        virtual void SetupInitial(ChSystem* system) override {}

        std::array<std::shared_ptr<ChNodeFEAfieldXYZ>, 2> nodes;
    };


    // 2) Let's implement a MATERIAL for our domain 
    // 
    // Here: properties, constitutive equations, optional per-sample data. Assumed at each "sample" of the model. 
    // In a ChDomainIntegrating, that perform Gauss quadrature over a continuum, samples are made at each Gauss point. 
    // In a ChDomainGeneric, like in this example, samples are made where needed by elements.

    class ChMaterialChemicalDiffusion : public ChMaterial {
        public:
        ChMaterialChemicalDiffusion() : diffusivity(1.) {}
        virtual ~ChMaterialChemicalDiffusion() {}
            
        // a property: the Fick' diffusion coefficient D
        double diffusivity; 

        // a constitutive equation: the Fick law  in 1D. 
        // This basic law is : flux J=D*dc_dx, so a single dc_dx parameter would be sufficient,
        // but here we pass also a mat_point_data pointer in sake of generality, because in example 2 we will
        // override this method by passing also a material that has some damage state, as in plasticity.
        virtual double ComputeMolarFlux1D(double dc_dx, ChFieldData* mat_point_data) { return -diffusivity * dc_dx; }

        /// Implement this if the material needs custom data per material point,
        /// returning a std::make_unique<ChFieldDataCustom>()  where ChFieldDataCustom is
        /// your custom class with aditional states/properties per-point. For the moment return nullptr.
        virtual std::unique_ptr<ChFieldData> CreateMaterialPointData() const override{ return nullptr; }

        /// (This is optional, but will be used later in EXAMPLE 2 when inheriting from this material to create "damage" 
        /// effects, so we won't need an additional version of ChDomainChemical1D). 
        virtual void ComputeUpdateEndStep(ChFieldData* mat_point_data) { /* do nothing */ }
    };
        

    // 3) Let's implement a new DOMAIN class
    //
    // Tip: now an auxiliary data is stored per each material point during the ChDomainChemical1D
    // computation. For example, scratch pad data to be plotted in postprocessing, etc.
    // If you need to append additional data per each matpoint, do not modify this, just
    // define your class with custom data and use it in my_material_class::T_per_materialpoint

    class ChFieldDataAuxiliaryChemical1D : public ChFieldDataNONE {
        public:
        double molar_flux;  /// molar flux [mol/(m^2 s)]
    }; 

    // Domain for FEA chemical analysis over the web of edges. 
    // It is based on a scalar field of chemical moles 
     
    class ChDomainChemical1D : public ChDomainGeneric<std::tuple<ChFieldChemicalConcentration>,
                                                           ChFieldDataAuxiliaryChemical1D, ChElementDataKRM> {
        public:
        // The following just to provide a shortcut in type naming.
        using Base = ChDomainGeneric<std::tuple<ChFieldChemicalConcentration>,
                                       ChFieldDataAuxiliaryChemical1D,
                                        ChElementDataKRM>;
        using DataPerElement = typename Base::DataPerElement;

        /// Construct the domain
        ChDomainChemical1D(std::shared_ptr<ChFieldChemicalConcentration> mfield) : Base(mfield) {
            // attach a default material to simplify user side
            material = chrono_types::make_shared<ChMaterialChemicalDiffusion>();
        }

        /// The material assigned to this domain
        std::shared_ptr<ChMaterialChemicalDiffusion> material;

        /// For a given finite element, computes the internal loads Fi and set values in the Fi vector.
        /// Since in this example we are NOT using quadrature-based domains, we must override this with our
        /// custom implementation. See ChDomainIntegrating vs ChDomainGeneric.
        /// For our Fick 1D law, going from strong to weak discretized form, it is  d/dt c_i = Fi_i = J_i  
        virtual void ElementComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
                                                    DataPerElement& data,
                                                    ChVectorDynamic<>& Fi) override {
            int numelcoords = this->GetNumPerNodeCoordsVelLevel() * melement->GetNumNodes();
            Fi.setZero(numelcoords);
            if (auto edge = std::dynamic_pointer_cast<ChFieldElementChemicalEdge>(melement)) {
                double length = (*edge->GetNodeA() - *edge->GetNodeB()).Length();
                int i_field = 0;  // we have only one field, so index is 0
                int i_matpoint = 0;  // we have only one material point, so index is 0
                double c_1 = ((ChFieldDataChemicalConcentration*)data.nodes_data[0][i_field])->Concentration();
                double c_2 = ((ChFieldDataChemicalConcentration*)data.nodes_data[1][i_field])->Concentration();
                double dc_dx = (c_2 - c_1) / length;
                double J = material->ComputeMolarFlux1D(dc_dx, (data.matpoints_data.size()? data.matpoints_data[i_matpoint].get() : nullptr));
                Fi(0) = -J;   // load at node A
                Fi(1) = J;   // load  at node B
                data.matpoints_data_aux[i_matpoint].molar_flux = J;  // store scratch data it so that it can be plotted
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
                                                double Mfactor = 0) override {
            if (auto edge = std::dynamic_pointer_cast<ChFieldElementChemicalEdge>(melement)) {
                double length = (*edge->GetNodeA() - *edge->GetNodeB()).Length();
                // diffusion 
                double invL  = (1.0 / length);
                H(0, 0) = Rfactor * material->diffusivity * invL;   // -dFi(0)/dc(0)
                H(0, 1) = Rfactor * -material->diffusivity * invL;  // -dFi(0)/dc(1)
                H(1, 0) = Rfactor * -material->diffusivity * invL;  // -dFi(1)/dc(0)
                H(1, 1) = Rfactor * material->diffusivity * invL;   // -dFi(1)/dc(1)
                // capacity (diagonal, lumped)
                H(0, 0) += Mfactor * 0.5 * length;// * length;   
                H(1, 1) += Mfactor * 0.5 * length;//* length;  
            }
        }

        /// Invoked at the end of each time step. If the material has some
        /// custom handling of auxiliary data (states etc.) it will manage this in ComputeUpdateEndStep()
        /// NOTE! Not needed for this EXAMPLE 1, but.. assume that in future (ex. in EXAMPLE 2) someone will 
        /// implement a ChMaterialChemicalDiffusion that supports some custop per-material-point state update 
        /// like damage or plasticity: here we call material->ComputeUpdateEndStep() anyway, so later we won't need
        /// a custom version of ChDomainChemical1D for the damage case, but we can just inherit from this and
        /// override the ComputeUpdateEndStep of the material.
        virtual void PointUpdateEndStep(std::shared_ptr<ChFieldElement> melement,
                                        DataPerElement& data,
                                        const int i_point,
                                        const double time) override {
            material->ComputeUpdateEndStep(data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr);
        }

        protected:
        /// Get the material of the domain.
        virtual std::shared_ptr<ChMaterial> GetMaterial() override { return material; };

    };


    // 5) [OPTIONAL - only for postprocessing] Let's implement a DRAWER to enable plots our new element
    //
    // This is a class with functions to draw a ChFieldElementChemicalEdge finite element, for
    // the 3D visualization of the finite element in real time rendering, postprocessing, etc.
    // Later we'll do visual_mesh->GetElementDispatcher().RegisterDrawer<ChFieldElementChemicalEdge>(..);

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
            mmesh.GetIndicesVertices()[tri_offset] = ChVector3i(0, 1, 1) + ivert_offset;
            ChVector3i inorm_offset = ChVector3i((int)norm_offset, (int)norm_offset, (int)norm_offset);
            mmesh.GetIndicesNormals()[tri_offset] = ChVector3i(0, 0, 0) + inorm_offset;
            vert_offset += 2;
            tri_offset += 1;
            norm_offset += 1;
        }
    };


    // 6) [OPTIONAL - only for postprocessing] Let's implement EXTRACTORS used to enable color plots of our data

    class ExtractConcentration
        : public ChVisualDataExtractorScalar<
              ExtractConcentration,             // (just repeat the class name here - used to generate clone() automatically)
              ChFieldDataChemicalConcentration, // here put the class of the data that you want to fetch!
              DataAtNode>                       // flag - choose one option between: DataAtNode  or DataAtMaterialpoint
    {
        virtual double ExtractImpl(const ChFieldDataChemicalConcentration* fdata) const override {
            return const_cast<ChFieldDataChemicalConcentration*>(fdata)->Concentration();
        }
    };

    class ExtractMolarFlux
        : public ChVisualDataExtractorScalar<ExtractMolarFlux, ChFieldDataAuxiliaryChemical1D, DataAtMaterialpoint> {
        virtual double ExtractImpl(const ChFieldDataAuxiliaryChemical1D* fdata) const override {
            return fdata->molar_flux;
        }
    };




    //
    // Ok, that's all.
    // Now we can proceed to create the model, as usual, for a simple demo 
    //


    if (true) {
        // Create a Chrono physical system
        ChSystemNSC sys;

        // FIELD
        // Create the "field", that is a collection of scalar or vectorial properties at
        // every discretization point. In this case, the chemical problem requires a field of "moles" of substances:

        auto chemical_field = chrono_types::make_shared<ChFieldChemicalConcentration>();
        sys.Add(chemical_field);

        // DOMAIN
        // Create the "domain", which is a collection of finite elements operating over
        // some field(s). In this case we create a ChDomainChemical1D, field:

        auto chemical_domain1d = chrono_types::make_shared<ChDomainChemical1D>(chemical_field);
        sys.Add(chemical_domain1d);

        // MATERIAL
        // Depending on the type of domain, you can set some properties for the material

        auto chemical_material = chrono_types::make_shared<ChMaterialChemicalDiffusion>();
        chemical_domain1d->material = chemical_material;  // set the material in domain
        chemical_material->diffusivity = 0.01;

        // CREATE SOME FINITE ELEMENTS AND NODES
        // The web of chemical channells has a bunch of ChNodeFEAfieldXYZ as nodes.
        // Their xyz position in 3D space is irrelevant for the 1D chemical problem at
        // hand, but we initialize xyz values because could be useful for visualization.

        auto node1 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0, 0, 0));
        auto node2 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0.1, 0, 0));
        auto node3 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0.1, 0.1, 0));
        auto node4 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0, 0.1, 0));
        chemical_field->AddNode(node1);
        chemical_field->AddNode(node2);
        chemical_field->AddNode(node3);
        chemical_field->AddNode(node4);

        auto element1 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node1, node2);
        auto element2 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node2, node3);
        auto element3 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node3, node4);
        auto element4 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node4, node1);
        auto element5 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node1, node3);
        chemical_domain1d->AddElement(element1);
        chemical_domain1d->AddElement(element2);
        chemical_domain1d->AddElement(element3);
        chemical_domain1d->AddElement(element4);
        chemical_domain1d->AddElement(element5);

        // How to set some initial conditions: the initial concentration at node:
        //chemical_field->NodeData(node2).Concentration() = 4.0;  // initial concentration = 4 mole per m^3 at node 1

        // How to set some fixed node:
        //chemical_field->NodeData(node2).SetFixed(true);  // fixed concentration (the initial zero) at node 2

        // How to constraint node, ex. with harmonic time-varying  concentration=A*sin(2pi*freq*t+phase)+shift:
        auto sine_concentr = chrono_types::make_shared<ChFunctionSine>(1.5, 0.1, 0.0, 1.5);  // A,freq,phase,shift
        auto constraint_c2 = chrono_types::make_shared<ChLinkField>();
        constraint_c2->Initialize(node2, chemical_field);
        constraint_c2->SetOffset(sine_concentr);
        sys.Add(constraint_c2);
        
 
        // Just for testing, add also the 3D ChDomainChemical3D with volume elements, working together with
        // the 1D domain ChDomainChemical1D because they'll share the same field, and elements will share one node.
        auto chemical_domain3d = chrono_types::make_shared<ChDomainChemical3D>(chemical_field);
        chemical_domain3d->diffusivity = 0.01;
        sys.Add(chemical_domain3d);

        ChBuilderVolumeBox builder;                                         // this helps creating grids of ChFieldElementHexahedron8 elements
        builder.BuildVolume(ChFrame<>(ChVector3d(0.1, -0.02, 0)), 4, 1, 1,  // N of elements in x,y,z direction
                            0.1, 0.02, 0.04);                               // width in x,y,z direction
        builder.AddToDomain(chemical_domain3d);

        // make a ChFieldElementHexahedron8 element share "node2" created before for ChFieldElementChemicalEdge:
        chemical_field->RemoveNode((*builder.elements.list().begin())->GetHexahedronNode(3));
        (*builder.elements.list().begin())->SetHexahedronNode(3, node2);  


        // POSTPROCESSING & VISUALIZATION (optional)

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(chemical_domain1d);
        visual_nodes->SetGlyphsSize(0.008);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractConcentration(), 0.0, 3.0, "Concentration");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        chemical_domain1d->AddVisualShape(visual_nodes);
        
        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(chemical_domain1d);
        visual_mesh->GetElementDispatcher().RegisterDrawer<ChFieldElementChemicalEdge>(std::make_unique<ChDrawerChemicalEdge>());
        visual_mesh->AddPositionExtractor(ExtractPos());
        visual_mesh->AddPropertyExtractor(ExtractMolarFlux(), -0.3, 0.3, "Molar flux");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));  // ChColor(0, 1, 0));
        visual_mesh->SetWireframe(true);
        chemical_domain1d->AddVisualShape(visual_mesh);
     
        auto visual_meshv = chrono_types::make_shared<ChVisualDomainMesh>(chemical_domain3d);
        visual_meshv->AddPositionExtractor(ExtractPos());
        visual_meshv->AddPropertyExtractor(ExtractConcentration(), 0.0, 3.0, "Concentration");
        visual_meshv->SetColormap(ChColormap(ChColormap::Type::JET));  
        chemical_domain3d->AddVisualShape(visual_meshv);

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test custom FEA multiphysics: 1D Fick diffusion");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 0.18, -0.18));
        vis->AddTypicalLights();
        
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);
        
        // The default EULER_IMPLICIT_LINEAR is not good for problems with finite elements, switch to:
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

        // Simulation loop
        double timestep = 0.01;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }
    }
   

    ///////////////////////////////////////////////////////////////////////////////////////////////////


    //
    // EXAMPLE 3
    //

    // Same as before, i.e. chemical diffusion in 1D capillary pipes, but this time we keep track of "damage".
    // This allows us to demonstrate how to attach auxiliary data to material points (or to elements) that 
    // contain material states like damage, plasticity, etc., and that may need a custom integration per each time step. 
    //
    // We recycle all the classes used in EXAMPLE 1, but adding the following:


    // 1) Let's implement a FIELD DATA for material points, 
    // 
    // Introduce a "ChFieldDataChemicalDamage" class for storing the damage state, inheriting from 
    // ChFieldDataStateless (otherwise, inheriting from ChFieldDataState or ChFieldDataGeneric<>, Chrono could 
    // automatically integrate the state for you, but here we want to do it by hand in ComputeUpdateEndStep, so we use
    // ChFieldDataStateless).
    // This is like a persistent data structure that will be used to store the damage state of the material at each point.

    class ChFieldDataChemicalDamage : public ChFieldDataStateless {
      public:
        double damage;       /// total damage state, must be updated at the end of the step (i.e at material ComputeUpdateEndStep) 
        double flow_drift;   /// concept as the plastic flow in yeld. can be negative or positive.
        double step_flow_drift;   /// concept as the plastic flow in yeld. can be negative or positive.
    };

    // 2) Let's implement a MATERIAL for our domain
    //
    // The material inherits from the previous ChMaterialChemicalDiffusion, but: 
    // - override CreateMaterialPointData: returns our new "ChFieldDataChemicalDamage" class for storing the damage
    // - override ComputeMolarFlux1D, the constitutive law: return a "capped chemical flow", and flow in excess causess step damage
    // - override ComputeUpdateEndStep: called at the end of step (not every Newton iteration) to update the total damage

    class ChMaterialChemicalDiffusionDamaged : public ChMaterialChemicalDiffusion {
      public:
        ChMaterialChemicalDiffusionDamaged()  {}
        virtual ~ChMaterialChemicalDiffusionDamaged() {}

        // A property: maximum flow that the material can sustain before being fully damaged
        double max_flow = 6.;  

        // A property: damage multiplier, how much damage is generated by a flow > max_flow threshold
        double damage_multiplier = 0.5;

        // Override the constitutive equation: the Fick law  in 1D but with a "capping to max_flow". 
        // Chemical flow exceeding max_flow will generate damage, somehow like in plasticity. Just as example.
        double ComputeMolarFlux1D(double dc_dx, ChFieldData* mat_point_data) override { 
            // use base class to compute the flux,
            double J = ChMaterialChemicalDiffusion::ComputeMolarFlux1D(dc_dx, mat_point_data);
            
            auto mat_point_data_chemical = static_cast<ChFieldDataChemicalDamage*>(mat_point_data);
            mat_point_data_chemical->step_flow_drift = 0;
            // if flow exceeds max_flow, cap the flow to max_flow and compute how much the drift in flow:
            if (std::abs(J) > max_flow) {
                mat_point_data_chemical->step_flow_drift = J - std::copysign(max_flow, J);  
                J = std::copysign(max_flow, J);  // cap the flow to max_flow
            }
            return J; 
        }

        /// In order to have the domain automatically instancing a damage data structure per each material point,
        /// we must override this function and return our ChFieldDataChemicalDamage:
        virtual std::unique_ptr<ChFieldData> CreateMaterialPointData() const override {
            return std::make_unique<ChFieldDataChemicalDamage>();
        }

        /// This is called at the end of the step - not per each Newton iteration.
        virtual void ComputeUpdateEndStep(ChFieldData* mat_point_data) override { 
            auto mat_point_data_chemical = static_cast<ChFieldDataChemicalDamage*>(mat_point_data);
            double delta_flow_drift = 0;
            if (mat_point_data_chemical->step_flow_drift >= 0)
                delta_flow_drift = std::max(0., mat_point_data_chemical->step_flow_drift - mat_point_data_chemical->flow_drift);
            else
                delta_flow_drift = std::min(0., mat_point_data_chemical->step_flow_drift - mat_point_data_chemical->flow_drift);
            mat_point_data_chemical->damage += damage_multiplier * std::abs(delta_flow_drift);
            mat_point_data_chemical->flow_drift = mat_point_data_chemical->step_flow_drift;
        }

    };

    // 3) [OPTIONAL, only for postprocessing] Let's implement EXTRACTORS, for color plots

    class ExtractChemicalDamage
        : public ChVisualDataExtractorScalar<ExtractChemicalDamage, ChFieldDataChemicalDamage, DataAtMaterialpoint> {
        virtual double ExtractImpl(const ChFieldDataChemicalDamage* fdata) const override {
            return fdata->damage;
        }
    };


    //
    // Ok, that's all.
    // Now we can proceed as usual to create the model, this time to test the ChMaterialChemicalDiffusionDamaged
    // We create a FIELD, a MATERIAL, a DOMAIN , NODES and FINITE ELEMENTS just like we did in EXAMPLE 1,
    // so we do not repeat comments. The big difference, however, is that now we use the 
    // ChMaterialChemicalDiffusionDamaged material, that will cause damage if the chemical flow exceeds the max_flow
    // threshold.
    //

    if (true) {
        // Create a Chrono physical system
        ChSystemNSC sys;

        // FIELD

        auto chemical_field = chrono_types::make_shared<ChFieldChemicalConcentration>();
        sys.Add(chemical_field);

        // DOMAIN

        auto chemical_domain1d = chrono_types::make_shared<ChDomainChemical1D>(chemical_field);
        sys.Add(chemical_domain1d);

        // MATERIAL

        auto chemical_material = chrono_types::make_shared<ChMaterialChemicalDiffusionDamaged>();
        chemical_domain1d->material = chemical_material;  // set the material in domain
        chemical_material->diffusivity = 0.01;
        chemical_material->max_flow = 0.02;  // NEW SETTING! cap the flow to this, if beyond, damage will accumulate
        chemical_material->damage_multiplier = 4;  // NEW SETTING! how much damage is generated by a flow > max_flow threshold

        // CREATE SOME FINITE ELEMENTS AND NODES

        auto node1 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0, 0, 0));
        auto node2 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0.1, 0, 0));
        auto node3 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0.1, 0.1, 0));
        auto node4 = chrono_types::make_shared<ChNodeFEAfieldXYZ>(ChVector3d(0, 0.1, 0));
        chemical_field->AddNode(node1);
        chemical_field->AddNode(node2);
        chemical_field->AddNode(node3);
        chemical_field->AddNode(node4);

        auto element1 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node1, node2);
        auto element2 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node2, node3);
        auto element3 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node3, node4);
        auto element4 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node4, node1);
        auto element5 = chrono_types::make_shared<ChFieldElementChemicalEdge>(node1, node3);
        chemical_domain1d->AddElement(element1);
        chemical_domain1d->AddElement(element2);
        chemical_domain1d->AddElement(element3);
        chemical_domain1d->AddElement(element4);
        chemical_domain1d->AddElement(element5);

        // Set some initial conditions: the initial concentration at node1:
        //chemical_field->NodeData(node1).Concentration() = 4.0;  // initial concentration = 4 mole per m^3 at node 1

        // Set some fixed node with a time-varying concentration:
        auto sine_concentr = chrono_types::make_shared<ChFunctionSine>(1.5, 0.02, 0.0, 1.5);  // ampl,freq,phase,shift
        auto constraint_c2 = chrono_types::make_shared<ChLinkField>();
        constraint_c2->Initialize(node2, chemical_field);
        constraint_c2->SetOffset(sine_concentr);
        sys.Add(constraint_c2);

        auto visual_nodes = chrono_types::make_shared<ChVisualDomainGlyphs>(chemical_domain1d);
        visual_nodes->SetGlyphsSize(0.01);
        visual_nodes->AddPositionExtractor(ExtractPos());
        visual_nodes->AddPropertyExtractor(ExtractConcentration(), 0.0, 3.0, "Concentration");
        visual_nodes->SetColormap(ChColormap(ChColormap::Type::JET));
        chemical_domain1d->AddVisualShape(visual_nodes);

        auto visual_mesh = chrono_types::make_shared<ChVisualDomainMesh>(chemical_domain1d);
        visual_mesh->GetElementDispatcher().RegisterDrawer<ChFieldElementChemicalEdge>(
            std::make_unique<ChDrawerChemicalEdge>());
        visual_mesh->AddPositionExtractor(ExtractPos());
        //visual_mesh->AddPropertyExtractor(ExtractMolarFlux(), -3.0, 3.0, "Molar flux");
        visual_mesh->AddPropertyExtractor(ExtractChemicalDamage(), 0.0, 6.0, "Damage");
        visual_mesh->SetColormap(ChColormap(ChColormap::Type::JET));  // ChColor(0, 1, 0));
        visual_mesh->SetWireframe(true);
        chemical_domain1d->AddVisualShape(visual_mesh);

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Test custom FEA multiphysics: 1D Fick with damage");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 0.18, -0.18));
        vis->AddTypicalLights();

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);

        // The default EULER_IMPLICIT_LINEAR is not good for problems with finite elements, switch to:
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

        // Simulation loop
        double timestep = 0.01;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }
    }


    return 0;

}
