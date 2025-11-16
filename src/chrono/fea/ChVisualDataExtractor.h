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

#ifndef CHVISUALDATAEXTRACTOR_H
#define CHVISUALDATAEXTRACTOR_H

#include <optional>
#include "chrono/core/ChTensors.h"
#include "chrono/fea/ChFieldData.h"
#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"
#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChFieldElementTetrahedron4.h"

namespace chrono {
namespace fea {



/// @addtogroup chrono_fea
/// @{


/// Base class for all extractors, that are objects capable of looking into 
/// a ChFieldData and return a corresponding scalar / vector / tensor, only if
/// the type of ChFieldData allows it, otherwise a std::nullopt is returned.

class ChVisualDataExtractor {
public:
    virtual bool IsDataAtMaterialpoint() const = 0;
    virtual bool IsDataAtNode() const = 0;

    virtual ChVisualDataExtractor* clone() const = 0;
};

/// Base class for all extractors that look into a ChFieldData and 
/// return a corresponding scalar, if the type of ChFieldData allows it 

class ChVisualDataExtractorScalarBase : public ChVisualDataExtractor {
public:
    std::optional<double> virtual Extract(ChFieldData* data) const = 0;
};

/// Base class for all extractors that look into a ChFieldData and 
/// return a corresponding vector, if the type of ChFieldData allows it 

class ChVisualDataExtractorVectorBase : public ChVisualDataExtractor {
public:
    std::optional<ChVector3d> virtual Extract(ChFieldData* data) const = 0;
};

/// Base class for all extractors that look into a ChFieldData and 
/// return a corresponding 3x3 matrix, if the type of ChFieldData allows it 

class ChVisualDataExtractorMatrix33Base : public ChVisualDataExtractor {
public:
    std::optional<ChMatrix33d> virtual Extract(ChFieldData* data) const = 0;
};

/// Base class for all extractors that look into a ChFieldData and 
/// return a corresponding quaternion, if the type of ChFieldData allows it 

class ChVisualDataExtractorQuaternionBase : public ChVisualDataExtractor {
public:
    std::optional<ChQuaternion<double>> virtual Extract(ChFieldData* data) const = 0;
};

struct DataAtMaterialpoint {
    static constexpr bool is_materialpoint = true;
    static constexpr bool is_node = false;
};

struct DataAtNode {
    static constexpr bool is_materialpoint = false;
    static constexpr bool is_node = true;
};

template <typename Derived, class T_field_data, class T_data_location = DataAtNode>
class ChVisualDataExtractorScalar : public ChVisualDataExtractorScalarBase {
public:
    std::optional<double> Extract(ChFieldData* data) const override {
        if (const auto* typed_data = dynamic_cast<T_field_data*>(data)) {
            return this->ExtractImpl(typed_data);
        }
        return std::nullopt;
    }
    virtual double ExtractImpl(const T_field_data* fdata) const = 0;

    bool IsDataAtMaterialpoint() const override { return T_data_location::is_materialpoint; }
    bool IsDataAtNode() const override { return T_data_location::is_node; }

    ChVisualDataExtractor* clone()  const override {
        // Creates a new object of the derived type using its copy constructor
        return new Derived(*static_cast<const Derived*>(this));
    }
};

class ChVisualDataExtractorVectorToScalar : public ChVisualDataExtractorScalarBase {
public:
    enum eVectorConversion {
        X = 0,
        Y,
        Z,
        LENGTH,
    };

    ChVisualDataExtractorVectorToScalar(std::shared_ptr<ChVisualDataExtractorVectorBase> mvector_extractor, eVectorConversion n_component) :
        vector_extractor(mvector_extractor),
        id_component(n_component)
    {};

    std::optional<double> Extract(ChFieldData* data) const override {
        if (auto retv = vector_extractor->Extract(data)) {
            switch (id_component) {
            case X: {
                return retv.value().x();
            }
            case Y: {
                return retv.value().y();
            }
            case Z: {
                return retv.value().z();
            }
            case LENGTH: {
                return retv.value().Length();
            }
            default:
                return std::nullopt;
            }
        }
        return std::nullopt;
    }

    bool IsDataAtMaterialpoint() const override { return vector_extractor->IsDataAtMaterialpoint(); }
    bool IsDataAtNode() const override { return vector_extractor->IsDataAtMaterialpoint(); }

    ChVisualDataExtractor* clone()  const override {
        return new ChVisualDataExtractorVectorToScalar(*(this));
    }

private:
    eVectorConversion id_component; // 0=x, 1=y, 2=z;
    std::shared_ptr<ChVisualDataExtractorVectorBase> vector_extractor;
};


template <typename Derived, class T_field_data, class T_data_location = DataAtNode>
class ChVisualDataExtractorVector : public ChVisualDataExtractorVectorBase {
public:
    std::optional<ChVector3d> Extract(ChFieldData* data) const override {
        if (const auto* typed_data = dynamic_cast<T_field_data*>(data)) {
            return this->ExtractImpl(typed_data);
        }
        return std::nullopt;
    }
    virtual ChVector3d ExtractImpl(const T_field_data* fdata) const = 0;

    bool IsDataAtMaterialpoint() const override { return T_data_location::is_materialpoint; }
    bool IsDataAtNode() const override { return T_data_location::is_node; }

    ChVisualDataExtractor* clone()  const override {
        // Creates a new object of the derived type using its copy constructor
        return new Derived(*static_cast<const Derived*>(this));
    }

    // Access sub-components of the vector so that one can do things like visual_mesh->AddPropertyExtractor( ExtractHeatFlow().X() )  
    // with a fluent syntax where the .X() .Y() etc. return a scalar wrapper of this vector extractor

    ChVisualDataExtractorVectorToScalar X() const {
        return ChVisualDataExtractorVectorToScalar(std::shared_ptr<ChVisualDataExtractorVectorBase>((ChVisualDataExtractorVectorBase*)this->clone()), ChVisualDataExtractorVectorToScalar::X);
    }
    ChVisualDataExtractorVectorToScalar Y() const {
        return ChVisualDataExtractorVectorToScalar(std::shared_ptr<ChVisualDataExtractorVectorBase>((ChVisualDataExtractorVectorBase*)this->clone()), ChVisualDataExtractorVectorToScalar::Y);
    }
    ChVisualDataExtractorVectorToScalar Z() const {
        return ChVisualDataExtractorVectorToScalar(std::shared_ptr<ChVisualDataExtractorVectorBase>((ChVisualDataExtractorVectorBase*)this->clone()), ChVisualDataExtractorVectorToScalar::Z);
    }
    ChVisualDataExtractorVectorToScalar Length() const {
        return ChVisualDataExtractorVectorToScalar(std::shared_ptr<ChVisualDataExtractorVectorBase>((ChVisualDataExtractorVectorBase*)this->clone()), ChVisualDataExtractorVectorToScalar::LENGTH);
    }
};



class ChVisualDataExtractorTensorToScalar : public ChVisualDataExtractorScalarBase {
public:
    enum eTensorConversion {
        VON_MISES = 0,
        PRINCIPAL_1,
        PRINCIPAL_2,
        PRINCIPAL_3,
        INVARIANT_1,
        INVARIANT_2,
        INVARIANT_3,
    };

    ChVisualDataExtractorTensorToScalar(std::shared_ptr<ChVisualDataExtractorMatrix33Base> mtensor_extractor, eTensorConversion which_conversion) :
        tensor_extractor(mtensor_extractor),
        id_what(which_conversion)
    {};
    std::optional<double> Extract(ChFieldData* data) const override {
        if (auto retv = tensor_extractor->Extract(data)) {
            switch (id_what) {
            case VON_MISES: {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(retv.value());
                ChVector3d ei = solver.eigenvalues();
                double a = ei.x() - ei.y();
                double b = ei.y() - ei.z();
                double c = ei.z() - ei.x();
                return std::sqrt(0.5 * (a * a + b * b + c * c)); // Von Mises
            }
            case PRINCIPAL_1: {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(retv.value());
                ChVector3d ei = solver.eigenvalues();
                return std::sqrt(ei.x());
            }
            case PRINCIPAL_2: {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(retv.value());
                ChVector3d ei = solver.eigenvalues();
                return std::sqrt(ei.y());
            }
            case PRINCIPAL_3: {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(retv.value());
                ChVector3d ei = solver.eigenvalues();
                return std::sqrt(ei.z());
            }
            case INVARIANT_1: {
                // I1
                return retv.value().trace();
            }
            case INVARIANT_2: {
                // I2
                double I1 = retv.value().trace();
                double traceAA = (retv.value() * retv.value()).trace();               // trace(A^2)
                return 0.5 * (I1 * I1 - traceAA);
            }
            case INVARIANT_3: {
                // I3
                return retv.value().determinant();
            }
            default:
                return std::nullopt;
            }
        }
        return std::nullopt;
    }

    bool IsDataAtMaterialpoint() const override { return tensor_extractor->IsDataAtMaterialpoint(); }
    bool IsDataAtNode() const override { return tensor_extractor->IsDataAtMaterialpoint(); }

    ChVisualDataExtractor* clone()  const override {
        return new ChVisualDataExtractorTensorToScalar(*(this));
    }

private:
    eTensorConversion id_what;
    std::shared_ptr<ChVisualDataExtractorMatrix33Base> tensor_extractor;
};


template <typename Derived, class T_field_data, class T_data_location = DataAtNode>
class ChVisualDataExtractorMatrix33 : public ChVisualDataExtractorMatrix33Base {
public:
    std::optional<ChMatrix33d> Extract(ChFieldData* data) const override {
        if (const auto* typed_data = dynamic_cast<T_field_data*>(data)) {
            return this->ExtractImpl(typed_data);
        }
        return std::nullopt;
    }
    virtual ChMatrix33d ExtractImpl(const T_field_data* fdata) const = 0;

    bool IsDataAtMaterialpoint() const override { return T_data_location::is_materialpoint; }
    bool IsDataAtNode() const override { return T_data_location::is_node; }

    ChVisualDataExtractor* clone()  const override {
        // Creates a new object of the derived type using its copy constructor
        return new Derived(*static_cast<const Derived*>(this));
    }

    // Access scalar invariants etc of the 3x3 tensor so that one can do things like visual_mesh->AddPropertyExtractor( ExtractCauchyStress().I3() )  
    // with a fluent syntax where the .X() .Y() etc. return a scalar wrapper of this 3x3 tensor extractor

    ChVisualDataExtractorTensorToScalar VonMises() const {
        return ChVisualDataExtractorTensorToScalar(std::shared_ptr<ChVisualDataExtractorMatrix33Base>((ChVisualDataExtractorMatrix33Base*)this->clone()), ChVisualDataExtractorTensorToScalar::VON_MISES);
    }
    ChVisualDataExtractorTensorToScalar I1() const {
        return ChVisualDataExtractorTensorToScalar(std::shared_ptr<ChVisualDataExtractorMatrix33Base>((ChVisualDataExtractorMatrix33Base*)this->clone()), ChVisualDataExtractorTensorToScalar::INVARIANT_1);
    }
    ChVisualDataExtractorTensorToScalar I2() const {
        return ChVisualDataExtractorTensorToScalar(std::shared_ptr<ChVisualDataExtractorMatrix33Base>((ChVisualDataExtractorMatrix33Base*)this->clone()), ChVisualDataExtractorTensorToScalar::INVARIANT_2);
    }
    ChVisualDataExtractorTensorToScalar I3() const {
        return ChVisualDataExtractorTensorToScalar(std::shared_ptr<ChVisualDataExtractorMatrix33Base>((ChVisualDataExtractorMatrix33Base*)this->clone()), ChVisualDataExtractorTensorToScalar::INVARIANT_3);
    }
    ChVisualDataExtractorTensorToScalar Principal1() const {
        return ChVisualDataExtractorTensorToScalar(std::shared_ptr<ChVisualDataExtractorMatrix33Base>((ChVisualDataExtractorMatrix33Base*)this->clone()), ChVisualDataExtractorTensorToScalar::PRINCIPAL_1);
    }
    ChVisualDataExtractorTensorToScalar Principal2() const {
        return ChVisualDataExtractorTensorToScalar(std::shared_ptr<ChVisualDataExtractorMatrix33Base>((ChVisualDataExtractorMatrix33Base*)this->clone()), ChVisualDataExtractorTensorToScalar::PRINCIPAL_2);
    }
    ChVisualDataExtractorTensorToScalar Principal3() const {
        return ChVisualDataExtractorTensorToScalar(std::shared_ptr<ChVisualDataExtractorMatrix33Base>((ChVisualDataExtractorMatrix33Base*)this->clone()), ChVisualDataExtractorTensorToScalar::PRINCIPAL_3);
    }
};


template <typename Derived, class T_field_data, class T_data_location = DataAtNode>
class ChVisualDataExtractorQuaternion : public ChVisualDataExtractorQuaternionBase {
public:
    std::optional<ChQuaternion<double>> Extract(ChFieldData* data) const override {
        if (const auto* typed_data = dynamic_cast<T_field_data*>(data)) {
            return this->ExtractImpl(typed_data);
        }
        return std::nullopt;
    }
    virtual ChQuaternion<double> ExtractImpl(const T_field_data* fdata) const = 0;

    bool IsDataAtMaterialpoint() const override { return T_data_location::is_materialpoint; }
    bool IsDataAtNode() const override { return T_data_location::is_node; }

    ChVisualDataExtractor* clone()  const override {
        // Creates a new object of the derived type using its copy constructor
        return new Derived(*static_cast<const Derived*>(this));
    }
};







//------------------------------------------------------------------------------------------------------

// Ready to use extractors. If you implement some custom ChFieldData-inherited class, you may want to provide
// the corresponding extractor(s) to fetch the data when painting the finite element meshes. 
// Inherit from ChVisualDataExtractorVector or ChVisualDataExtractorScalar etc., like in the examples below.
//
// Example of extractor: given a generic ChFieldData* mydata, ExtractPos attempts to fetch the position: if the mydata is
// of type ChFieldDataPos3D, then  you can do  mypos = extractor.Extract(mydata).value() , otherwise 
// it returns std::nullopt. So, you can do things like: 
//   ExtractPos extractor;
//   if (auto fetched_pos = extractor->Extract(mydata)) 
//      mypos = fetched_pos.value();


class ExtractPos : public ChVisualDataExtractorVector<
                    ExtractPos,             // (just repeat the class name here - will be used for CRTP to generate the clone() function automatically)
                    ChFieldDataPos3D,       // here put the class of the data that you want to fetch!
                    DataAtNode>             // flag - choose one option between: DataAtNode  or DataAtMaterialpoint 
{
    virtual ChVector3d ExtractImpl(const ChFieldDataPos3D* fdata)  const override {
        return fdata->GetPos();
    }
};

class ExtractPosDt : public ChVisualDataExtractorVector<ExtractPosDt, ChFieldDataPos3D, DataAtNode> {
    virtual ChVector3d ExtractImpl(const ChFieldDataPos3D* fdata)  const override {
        return fdata->GetPosDt();
    }
};

class ExtractPosDt2 : public ChVisualDataExtractorVector<ExtractPosDt2, ChFieldDataPos3D, DataAtNode> {
    virtual ChVector3d ExtractImpl(const ChFieldDataPos3D* fdata)  const override {
        return fdata->GetPosDt2();
    }
};

class ExtractTemperature : public ChVisualDataExtractorScalar<ExtractTemperature, ChFieldDataTemperature, DataAtNode> {
    virtual double ExtractImpl(const ChFieldDataTemperature* fdata)  const override {
        return const_cast<ChFieldDataTemperature*>(fdata)->T();
    }
};

class ExtractTemperatureDt : public ChVisualDataExtractorScalar<ExtractTemperatureDt, ChFieldDataTemperature, DataAtNode> {
    virtual double ExtractImpl(const ChFieldDataTemperature* fdata)  const override {
        return const_cast<ChFieldDataTemperature*>(fdata)->T_dt();
    }
};

class ExtractElectricPotential : public ChVisualDataExtractorScalar<ExtractElectricPotential, ChFieldDataElectricPotential, DataAtNode> {
    virtual double ExtractImpl(const ChFieldDataElectricPotential* fdata)  const override {
        return const_cast<ChFieldDataElectricPotential*>(fdata)->V();
    }
};








/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
