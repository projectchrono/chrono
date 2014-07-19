#ifndef CHMATERIAL_H
#define CHMATERIAL_H

///////////////////////////////////////////////////
//
//   ChMaterial.h
//
//   Class for storing a color as an object asset
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
///////////////////////////////////////////////////

#include "assets/ChAsset.h"
#include "assets/ChColor.h"
#include <string>
#include <vector>

namespace chrono {

enum ChMaterialType {
  CH_MATERIAL_DIFFUSE,
  CH_MATERIAL_PHONG,
  CH_MATERIAL_CONDUCTOR,
  CH_MATERIAL_PLASTIC
};

struct material_option {
  std::string type, parameter, value;
};

class ChApi ChMaterial {
public:

  ChMaterial();
  ~ChMaterial();

  bool IsVisible() const   {return visible;}
  void SetVisible(bool mv) {visible = mv;}

  // Get the color of the surface.
  // This information could be used by visualization postprocessing.
  const ChColor& GetColor() const {
    return color;
  }

  // Set the color of the surface.
  // This information could be used by visualization postprocessing.
  void SetColor(const ChColor& mc) {
    color = mc;
  }

  // Get the fading amount, 0..1.
  // If =0, no transparency of surface, it =1 surface is completely transparent.
  float GetFading() const {
    return fading;
  }

  // Set the fading amount, 0..1.
  // If =0, no transparency of surface, it =1 surface is completely transparent.
  void SetFading(const float mc) {
    fading = mc;
  }

  void SetType(const ChMaterialType type) {
    material_type = type;
  }

  void SetOption(const std::string& type,
                 const std::string& parameter,
                 const std::string& value) {
    material_option temp;
    temp.type = type;
    temp.parameter = parameter;
    temp.value = value;
    options.push_back(temp);
  }

  ChColor color; //color of material
  float fading; //transparency of material
  ChMaterialType material_type;
  std::vector<material_option> options;
  bool visible;

};

}

#endif
