#ifndef CHMATERIAL_H
#define CHMATERIAL_H

///////////////////////////////////////////////////
//
//   ChColor.h
//
//   Class for storing a color as an object asset
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
///////////////////////////////////////////////////

#include "assets/ChAsset.h"
#include "assets/ChColor.h"
#include <string>
#include <vector>

using namespace std;
namespace chrono {
enum ChMaterialType {
	CH_MATERIAL_DIFFUSE, CH_MATERIAL_PHONG, CH_MATERIAL_CONDUCTOR, CH_MATERIAL_PLASTIC
};
/*
 #define CH_MATERIAL_DIFFUSE=0;
 #define CH_MATERIAL_PHONG=1;
 #define CH_MATERIAL_CONDUCTOR=2;
 #define CH_MATERIAL_PLASTIC=3;
 */
struct material_option {
	string type, parameter, value;
};

class ChApi ChMaterial {
public:

	ChMaterial();
	~ChMaterial();

	bool IsVisible() const {
		return visible;
	}
	void SetVisible(bool mv) {
		visible = mv;
	}

// Get the color of the surface. This information could be used by visualization postprocessing.
	ChColor GetColor() const {
		return color;
	}
// Set the color of the surface. This information could be used by visualization postprocessing.
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

	void SetOption(string type, string parameter, string value) {
		material_option temp;
		temp.type = type;
		temp.parameter = parameter;
		temp.value = value;
		options.push_back(temp);
	}
	ChColor color; //color of material
	float fading; //transparency of material
	ChMaterialType material_type;
	vector<material_option> options;
	bool visible;

};
}
#endif
