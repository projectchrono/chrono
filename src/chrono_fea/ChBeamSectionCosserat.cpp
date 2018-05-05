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


#include "chrono_fea/ChBeamSectionCosserat.h"


namespace chrono {
namespace fea {


/// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
/// at once, given the y and z widths of the beam assumed
/// with rectangular shape.
void ChElasticityCosseratSimple::SetAsRectangularSection(double width_y, double width_z) {
	
	this->Izz = (1.0 / 12.0) * width_z * pow(width_y, 3);
	this->Iyy = (1.0 / 12.0) * width_y * pow(width_z, 3);

	// use Roark's formulas for torsion of rectangular sect:
	double t = ChMin(width_y, width_z);
	double b = ChMax(width_y, width_z);
	this->J = b * pow(t, 3) * ((1.0 / 3.0) - 0.210 * (t / b) * (1.0 - (1.0 / 12.0) * pow((t / b), 4)));

	// set Ks using Timoshenko-Gere formula for solid rect.shapes
	double poisson = this->E / (2.0 * this->G) - 1.0;
	this->Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
	this->Ks_z = this->Ks_y;

}

/// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
/// at once, given the diameter of the beam assumed
/// with circular shape.
void ChElasticityCosseratSimple::SetAsCircularSection(double diameter) {
	
	this->Izz = (CH_C_PI / 4.0) * pow((0.5 * diameter), 4);
	this->Iyy = Izz;

	// exact expression for circular beam J = Ixx ,
	// where for polar theorem Ixx = Izz+Iyy
	this->J = Izz + Iyy;

	// set Ks using Timoshenko-Gere formula for solid circular shape
	double poisson = this->E / (2.0 * this->G) - 1.0;
	this->Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
	this->Ks_z = this->Ks_y;

}

void ChElasticityCosseratSimple::ComputeStress(
	ChVector<>& stress_n,      ///< return the local stress (generalized force), x component = traction along beam
	ChVector<>& stress_m,      ///< return the local stress (generalized torque), x component = torsion torque along beam
	const ChVector<>& strain_n, ///< the local strain (deformation part): x= elongation, y and z are shear
	const ChVector<>& strain_m  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
)  {
	stress_n.x() = E * section->Area * strain_n.x();
	stress_n.y() = Ks_y * G * section->Area * strain_n.y();
	stress_n.z() = Ks_z * G * section->Area * strain_n.z();
	stress_m.x() = G * J   * strain_m.x();
	stress_m.y() = E * Iyy * strain_m.y();
	stress_m.z() = E * Izz * strain_m.z();
}

/// Compute the 6x6 tangent material stiffness matrix [Km] =d\sigma/d\epsilon
void ChElasticityCosseratSimple::ComputeStiffnessMatrix(
	ChMatrixDynamic<>& K,       ///< return the 6x6 stiffness matrix
	const ChVector<>& strain_n, ///< the local strain (deformation part): x= elongation, y and z are shear
	const ChVector<>& strain_m  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
) {
	K.Reset(6, 6);
	K(0, 0) = E * section->Area;
	K(1, 1) = Ks_y * G *  section->Area;
	K(2, 2) = Ks_z * G *  section->Area;
	K(3, 3) = G * J;
	K(4, 4) = E * Iyy;
	K(5, 5) = E * Izz;
}


////////////////////////////////////////////////////////////////////////////////////



void ChElasticityCosseratGeneric::SetAsRectangularSection(double width_y, double width_z) {
	double E = 1;
	double G = 1;

	double Izz = (1.0 / 12.0) * width_z * pow(width_y, 3);
	double Iyy = (1.0 / 12.0) * width_y * pow(width_z, 3);

	// use Roark's formulas for torsion of rectangular sect:
	double t = ChMin(width_y, width_z);
	double b = ChMax(width_y, width_z);
	double J = b * pow(t, 3) * ((1.0 / 3.0) - 0.210 * (t / b) * (1.0 - (1.0 / 12.0) * pow((t / b), 4)));

	// set Ks using Timoshenko-Gere formula for solid rect.shapes
	double poisson = E / (2.0 * G) - 1.0;
	double Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
	double Ks_z = Ks_y;

	mE(0, 0) = E * section->Area;
	mE(1, 1) = Ks_y * G *  section->Area;
	mE(2, 2) = Ks_z * G *  section->Area;
	mE(3, 3) = G * J;
	mE(4, 4) = E * Iyy;
	mE(5, 5) = E * Izz;
}


void ChElasticityCosseratGeneric::SetAsCircularSection(double diameter) {
	double E = 1;
	double G = 1;

	double Izz = (CH_C_PI / 4.0) * pow((0.5 * diameter), 4);
	double Iyy = Izz;

	// exact expression for circular beam J = Ixx ,
	// where for polar theorem Ixx = Izz+Iyy
	double J = Izz + Iyy;

	// set Ks using Timoshenko-Gere formula for solid circular shape
	double poisson = E / (2.0 * G) - 1.0;
	double Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
	double Ks_z = Ks_y;

	mE(0, 0) = E * section->Area;
	mE(1, 1) = Ks_y * G *  section->Area;
	mE(2, 2) = Ks_z * G *  section->Area;
	mE(3, 3) = G * J;
	mE(4, 4) = E * Iyy;
	mE(5, 5) = E * Izz;
}

void ChElasticityCosseratGeneric::ComputeStress(
	ChVector<>& stress_n,      ///< return the local stress (generalized force), x component = traction along beam
	ChVector<>& stress_m,      ///< return the local stress (generalized torque), x component = torsion torque along beam
	const ChVector<>& strain_n, ///< the local strain (deformation part): x= elongation, y and z are shear
	const ChVector<>& strain_m  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
)  {
	ChMatrixNM<double, 6, 1> mstrain;
	ChMatrixNM<double, 6, 1> mstress;
	mstrain.PasteVector(strain_n, 0, 0);
	mstrain.PasteVector(strain_m, 3, 0);
	mstress.MatrMultiply(this->mE, mstrain);
	stress_n = mstress.ClipVector(0, 0);
	stress_m = mstress.ClipVector(3, 0);
}

/// Compute the 6x6 tangent material stiffness matrix [Km] =d\sigma/d\epsilon
void ChElasticityCosseratGeneric::ComputeStiffnessMatrix(
	ChMatrixDynamic<>& K,       ///< return the 6x6 stiffness matrix
	const ChVector<>& strain_n, ///< the local strain (deformation part): x= elongation, y and z are shear
	const ChVector<>& strain_m  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
) {
	K.CopyFromMatrix(this->mE);
}



////////////////////////////////////////////////////////////////////////////////////

void ChElasticityCosseratAdvanced::ComputeStress(
	ChVector<>& stress_n,      ///< return the local stress (generalized force), x component = traction along beam
	ChVector<>& stress_m,      ///< return the local stress (generalized torque), x component = torsion torque along beam
	const ChVector<>& strain_n, ///< the local strain (deformation part): x= elongation, y and z are shear
	const ChVector<>& strain_m  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
) {
	double Area = section->Area;
	double cos_alpha = cos(alpha);
	double sin_alpha = sin(alpha);
	double a11 = E * section->Area;
	double a22 = E * (Iyy * pow(cos_alpha, 2.) + Izz * pow(sin_alpha, 2.) + Cz * Cz * Area);
	double a33 = E * (Izz * pow(cos_alpha, 2.) + Iyy * pow(sin_alpha, 2.) + Cy * Cy * Area);
	double a12 = Cz * E * Area;
	double a13 = -Cy * E * Area;
	double a23 = (E* Iyy - E * Izz)*cos_alpha*sin_alpha - E * Cy * Cz * Area;
	stress_n.x() = a11 * strain_n.x() + a12 * strain_m.y() + a13 * strain_m.z();
	stress_m.y() = a12 * strain_n.x() + a22 * strain_m.y() + a23 * strain_m.z();
	stress_m.z() = a13 * strain_n.x() + a23 * strain_m.y() + a33 * strain_m.z();
	double cos_beta = cos(beta);
	double sin_beta = sin(beta);
	double KsyGA = Ks_y * G * Area;
	double KszGA = Ks_z * G * Area;
	double s11 = KsyGA * pow(cos_beta, 2.) + KszGA * pow(sin_beta, 2.);
	double s22 = KsyGA * pow(sin_beta, 2.) + KszGA * pow(cos_beta, 2.); // ..+s_loc_12*sin(beta)*cos(beta);
	double s33 = G * J + Sz * Sz * KsyGA + Sy * Sy * KszGA;
	double s12 = (KszGA - KsyGA) * sin_beta * cos_beta;
	double s13 = Sy * KszGA * sin_beta - Sz * KsyGA * cos_beta;
	double s23 = Sy * KszGA * cos_beta + Sz * KsyGA * sin_beta;
	stress_n.y() = s11 * strain_n.y() + s12 * strain_n.z() + s13 * strain_m.x();
	stress_n.z() = s12 * strain_n.y() + s22 * strain_n.z() + s23 * strain_m.x();
	stress_m.x() = s13 * strain_n.y() + s23 * strain_n.z() + s33 * strain_m.x();
}

/// Compute the 6x6 tangent material stiffness matrix [Km] =d\sigma/d\epsilon
void ChElasticityCosseratAdvanced::ComputeStiffnessMatrix(
	ChMatrixDynamic<>& K,       ///< return the 6x6 stiffness matrix
	const ChVector<>& strain_n, ///< the local strain (deformation part): x= elongation, y and z are shear
	const ChVector<>& strain_m  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
) {
	K.Reset(6, 6);
	double Area = section->Area;
	double cos_alpha = cos(alpha);
	double sin_alpha = sin(alpha);
	double a11 = E * section->Area;
	double a22 = E * (Iyy * pow(cos_alpha, 2.) + Izz * pow(sin_alpha, 2.) + Cz * Cz * Area);
	double a33 = E * (Izz * pow(cos_alpha, 2.) + Iyy * pow(sin_alpha, 2.) + Cy * Cy * Area);
	double a12 = Cz * E * Area;
	double a13 = -Cy * E * Area;
	double a23 = (E* Iyy - E * Izz)*cos_alpha*sin_alpha - E * Cy * Cz * Area;
	double cos_beta = cos(beta);
	double sin_beta = sin(beta);
	double KsyGA = Ks_y * G * Area;
	double KszGA = Ks_z * G * Area;
	double s11 = KsyGA * pow(cos_beta, 2.) + KszGA * pow(sin_beta, 2.);
	double s22 = KsyGA * pow(sin_beta, 2.) + KszGA * pow(cos_beta, 2.); // ..+s_loc_12*sin(beta)*cos(beta);
	double s33 = G * J + Sz * Sz * KsyGA + Sy * Sy * KszGA;
	double s12 = (KszGA - KsyGA) * sin_beta * cos_beta;
	double s13 = Sy * KszGA * sin_beta - Sz * KsyGA * cos_beta;
	double s23 = Sy * KszGA * cos_beta + Sz * KsyGA * sin_beta;

	K(0, 0) = a11;
	K(0, 4) = a12;
	K(0, 5) = a13;
	K(1, 1) = s11;
	K(1, 2) = s12;
	K(1, 3) = s13;
	K(2, 1) = s12;
	K(2, 2) = s22;
	K(2, 3) = s23;
	K(3, 1) = s13;
	K(3, 2) = s23;
	K(3, 3) = s33;
	K(4, 0) = a12;
	K(4, 4) = a22;
	K(4, 5) = a23;
	K(5, 0) = a13;
	K(5, 4) = a23;
	K(5, 5) = a33;
}


////////////////////////////////////////////////////////////////////////////////////  



bool ChPlasticityCosseratLumped::ComputeStressWithReturnMapping(
	ChVector<>& stress_n,			 ///< return the local stress (generalized force), x component = traction along beam
	ChVector<>& stress_m,			 ///< return the local stress (generalized torque), x component = torsion torque along beam
	ChVector<>& e_strain_e_new,      ///< return updated elastic strain (deformation part)
	ChVector<>& e_strain_k_new,      ///< return updated elastic strain (curvature part)
	ChBeamMaterialInternalData& data_new,///< return updated material internal variables, at this point, including {p_strain_m, p_strain_n, p_strain_acc}
	const ChVector<>& tot_strain_e,  ///< trial tot strain (deformation part): x= elongation, y and z are shear
	const ChVector<>& tot_strain_k,  ///< trial tot strain (curvature part), x= torsion, y and z are line curvatures
	const ChBeamMaterialInternalData& data ///< current material internal variables, at this point, including {p_strain_m, p_strain_n, p_strain_acc}
) {
	auto mydata = dynamic_cast<const ChInternalDataLumpedCosserat*>(&data);
	auto mydata_new = dynamic_cast<ChInternalDataLumpedCosserat*>(&data_new);
	

	if (!mydata)
		throw ChException("ComputeStressWithReturnMapping cannot cast data to ChInternalDataLumpedCosserat*.");

	// Implement return mapping for a simple 1D plasticity model.

	// Compute the elastic trial stress:
	e_strain_e_new = tot_strain_e - mydata->p_strain_e;
	e_strain_k_new = tot_strain_k - mydata->p_strain_k;
	//double p_strain_acc = mydata->p_strain_acc;
	this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);  //<<<<  elastic sigma(epsilon)

	// axial direction
	{
		double strain_yeld_x = this->n_yeld_x->Get_y(mydata->p_strain_acc_e.x());			    ///<<<< sigma_y(p_strain_acc)
		double eta_x = stress_n.x() - this->n_beta_x->Get_y(mydata->p_strain_e.x());	///<<<< beta(p_strain_e)
		double Fyeld_x = fabs(eta_x) - strain_yeld_x;  //<<<<  Phi(sigma,p_strain_acc)   

		if (Fyeld_x > 0)
		{
			double Dgamma = 0;
			double Dgamma_old = 0;
			mydata_new->p_strain_acc_e.x() = mydata->p_strain_acc_e.x();
			mydata_new->p_strain_e.x() = mydata->p_strain_e.x();
			int iters = 0;
			while ((Fyeld_x > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
				double E_x = stress_n.x() / e_strain_e_new.x(); //instead of costly evaluation of Km, =dstress/dstrain
				double H = this->n_beta_x->Get_y_dx(mydata->p_strain_e.x())
					+ this->n_yeld_x->Get_y_dx(mydata->p_strain_acc_e.x()); //<<<<  H = dyeld/dplasticflow
				Dgamma -= Fyeld_x / (-E_x - H);
				double dDgamma = Dgamma - Dgamma_old;
				Dgamma_old = Dgamma;
				mydata_new->p_strain_acc_e.x() += dDgamma;
				e_strain_e_new.x() -= dDgamma * chrono::ChSignum(stress_n.x());
				mydata_new->p_strain_e.x() += dDgamma * chrono::ChSignum(stress_n.x());
				this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);  //<<<<  elastic sigma(epsilon) 
																													// compute yeld
				strain_yeld_x = this->n_yeld_x->Get_y(mydata_new->p_strain_acc_e.x());		///<<<< sigma_y(p_strain_acc)
				eta_x = stress_n.x() - this->n_beta_x->Get_y(mydata_new->p_strain_e.x());	///<<<< beta(p_strain_acc)
				Fyeld_x = fabs(eta_x) - strain_yeld_x;  //<<<<  Phi(sigma,p_strain_acc)  

				++iters;
			}
		}
	}

	// shear direction
	{
		double strain_yeld_y = this->n_yeld_y->Get_y(mydata->p_strain_acc_e.y());
		double eta_y = stress_n.y() - this->n_beta_y->Get_y(mydata->p_strain_e.y());
		double Fyeld_y = fabs(eta_y) - strain_yeld_y;  //<<<<  Phi(sigma,p_strain_acc)   

		if (Fyeld_y < 0)
		{
			double Dgamma = 0;
			double Dgamma_old = 0;
			mydata_new->p_strain_acc_e.y() = mydata->p_strain_acc_e.y();
			mydata_new->p_strain_e.y() = mydata->p_strain_e.y();
			int iters = 0;
			while ((Fyeld_y > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
				double E_y = stress_n.y() / e_strain_e_new.y(); //instead of costly evaluation of Km, =dstress/dstrain
				double H = this->n_beta_y->Get_y_dx(mydata->p_strain_e.y())
					+ this->n_yeld_y->Get_y_dx(mydata->p_strain_acc_e.y()); //<<<<  H = dyeld/dplasticflow
				Dgamma -= Fyeld_y / (-E_y - H);
				double dDgamma = Dgamma - Dgamma_old;
				Dgamma_old = Dgamma;
				mydata_new->p_strain_acc_e.y() += dDgamma;
				e_strain_e_new.y() -= dDgamma * chrono::ChSignum(stress_n.y());
				mydata_new->p_strain_e.y() += dDgamma * chrono::ChSignum(stress_n.y());
				this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);  //<<<<  elastic sigma(epsilon) 
																													// compute yeld
				strain_yeld_y = this->n_yeld_y->Get_y(mydata_new->p_strain_acc_e.y());		///<<<< sigma_y(p_strain_acc)
				eta_y = stress_n.y() - this->n_beta_y->Get_y(mydata_new->p_strain_e.y());	///<<<< beta(p_strain_acc)
				Fyeld_y = fabs(eta_y) - strain_yeld_y;  //<<<<  Phi(sigma,p_strain_acc)  

				++iters;
			}
		}
	}

	// shear direction
	{
		double strain_yeld_z = this->n_yeld_z->Get_y(mydata->p_strain_acc_e.z());
		double eta_z = stress_n.z() - this->n_beta_z->Get_y(mydata->p_strain_e.z());
		double Fyeld_z = fabs(eta_z) - strain_yeld_z;  //<<<<  Phi(sigma,p_strain_acc)   

		if (Fyeld_z > 0)
		{
			double Dgamma = 0;
			double Dgamma_old = 0;
			mydata_new->p_strain_acc_e.z() = mydata->p_strain_acc_e.z();
			mydata_new->p_strain_e.z() = mydata->p_strain_e.z();
			int iters = 0;
			while ((Fyeld_z > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
				double E_z = stress_n.z() / e_strain_e_new.z(); //instead of costly evaluation of Km, =dstress/dstrain
				double H = this->n_beta_z->Get_y_dx(mydata->p_strain_e.z())
					+ this->n_yeld_z->Get_y_dx(mydata->p_strain_acc_e.z()); //<<<<  H = dyeld/dplasticflow
				Dgamma -= Fyeld_z / (-E_z - H);
				double dDgamma = Dgamma - Dgamma_old;
				Dgamma_old = Dgamma;
				mydata_new->p_strain_acc_e.z() += dDgamma;
				e_strain_e_new.z() -= dDgamma * chrono::ChSignum(stress_n.z());
				mydata_new->p_strain_e.z() += dDgamma * chrono::ChSignum(stress_n.z());
				this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);  //<<<<  elastic sigma(epsilon) 
																													// compute yeld
				strain_yeld_z = this->n_yeld_z->Get_y(mydata_new->p_strain_acc_e.z());		///<<<< sigma_y(p_strain_acc)
				eta_z = stress_n.z() - this->n_beta_z->Get_y(mydata_new->p_strain_e.z());	///<<<< beta(p_strain_acc)
				Fyeld_z = fabs(eta_z) - strain_yeld_z;  //<<<<  Phi(sigma,p_strain_acc)  

				++iters;
			}
		}
	}

	// torsion direction
	{
		double strain_yeld_Mx = this->n_yeld_Mx->Get_y(mydata->p_strain_acc_k.x());
		double eta_Mx = stress_m.x() - this->n_beta_Mx->Get_y(mydata->p_strain_k.x());
		double Fyeld_Mx = fabs(eta_Mx) - strain_yeld_Mx;    

		if (Fyeld_Mx > 0)
		{
			double Dgamma = 0;
			double Dgamma_old = 0;
			mydata_new->p_strain_acc_k.x() = mydata->p_strain_acc_k.x();
			mydata_new->p_strain_k.x() = mydata->p_strain_k.x();
			int iters = 0;
			while ((Fyeld_Mx > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
				double E_Mx = stress_m.x() / e_strain_k_new.x(); //instead of costly evaluation of Km, =dstress/dstrain
				double H = this->n_beta_Mx->Get_y_dx(mydata->p_strain_k.x())
					+ this->n_yeld_Mx->Get_y_dx(mydata->p_strain_acc_k.x()); //<<<<  H = dyeld/dplasticflow
				Dgamma -= Fyeld_Mx / (-E_Mx - H);
				double dDgamma = Dgamma - Dgamma_old;
				Dgamma_old = Dgamma;
				mydata_new->p_strain_acc_k.x() += dDgamma;
				e_strain_k_new.x() -= dDgamma * chrono::ChSignum(stress_m.x());
				mydata_new->p_strain_k.x() += dDgamma * chrono::ChSignum(stress_m.x());
				this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);  //<<<<  elastic sigma(epsilon) 
																													// compute yeld
				strain_yeld_Mx = this->n_yeld_Mx->Get_y(mydata_new->p_strain_acc_k.x());		///<<<< sigma_y(p_strain_acc)
				eta_Mx = stress_m.x() - this->n_beta_Mx->Get_y(mydata_new->p_strain_k.x());	///<<<< beta(p_strain_acc)
				Fyeld_Mx = fabs(eta_Mx) - strain_yeld_Mx;  //<<<<  Phi(sigma,p_strain_acc)  

				++iters;
			}
		}
	}

	// bending y direction
	{
		double strain_yeld_My = this->n_yeld_My->Get_y(mydata->p_strain_acc_k.y());
		double eta_My = stress_m.y() - this->n_beta_My->Get_y(mydata->p_strain_k.y());
		double Fyeld_My = fabs(eta_My) - strain_yeld_My;

		if (Fyeld_My > 0)
		{
			double Dgamma = 0;
			double Dgamma_old = 0;
			mydata_new->p_strain_acc_k.y() = mydata->p_strain_acc_k.y();
			mydata_new->p_strain_k.y() = mydata->p_strain_k.y();
			int iters = 0;
			while ((Fyeld_My > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
				double E_My = stress_m.y() / e_strain_k_new.y(); //instead of costly evaluation of Km, =dstress/dstrain
				double H = this->n_beta_My->Get_y_dx(mydata->p_strain_k.y())
					+ this->n_yeld_My->Get_y_dx(mydata->p_strain_acc_k.y()); //<<<<  H = dyeld/dplasticflow
				Dgamma -= Fyeld_My / (-E_My - H);
				double dDgamma = Dgamma - Dgamma_old;
				Dgamma_old = Dgamma;
				mydata_new->p_strain_acc_k.y() += dDgamma;
				e_strain_k_new.y() -= dDgamma * chrono::ChSignum(stress_m.y());
				mydata_new->p_strain_k.y() += dDgamma * chrono::ChSignum(stress_m.y());
				this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);  //<<<<  elastic sigma(epsilon) 
																													// compute yeld
				strain_yeld_My = this->n_yeld_My->Get_y(mydata_new->p_strain_acc_k.y());		///<<<< sigma_y(p_strain_acc)
				eta_My = stress_m.y() - this->n_beta_My->Get_y(mydata_new->p_strain_k.y());	///<<<< beta(p_strain_acc)
				Fyeld_My = fabs(eta_My) - strain_yeld_My;  //<<<<  Phi(sigma,p_strain_acc)  

				++iters;
			}
		}
	}

	// bending z direction
	{
		double strain_yeld_Mz = this->n_yeld_Mz->Get_y(mydata->p_strain_acc_k.z());
		double eta_Mz = stress_m.z() - this->n_beta_Mz->Get_y(mydata->p_strain_k.z());
		double Fyeld_Mz = fabs(eta_Mz) - strain_yeld_Mz;

		if (Fyeld_Mz > 0)
		{
			double Dgamma = 0;
			double Dgamma_old = 0;
			mydata_new->p_strain_acc_k.z() = mydata->p_strain_acc_k.z();
			mydata_new->p_strain_k.z() = mydata->p_strain_k.z();
			int iters = 0;
			while ((Fyeld_Mz > this->nr_yeld_tolerance) && (iters < this->nr_yeld_maxiters)) {
				double E_Mz = stress_m.z() / e_strain_k_new.z(); //instead of costly evaluation of Km, =dstress/dstrain
				double H = this->n_beta_Mz->Get_y_dx(mydata->p_strain_k.z())
					+ this->n_yeld_Mz->Get_y_dx(mydata->p_strain_acc_k.z()); //<<<<  H = dyeld/dplasticflow
				Dgamma -= Fyeld_Mz / (-E_Mz - H);
				double dDgamma = Dgamma - Dgamma_old;
				Dgamma_old = Dgamma;
				mydata_new->p_strain_acc_k.z() += dDgamma;
				e_strain_k_new.z() -= dDgamma * chrono::ChSignum(stress_m.z());
				mydata_new->p_strain_k.z() += dDgamma * chrono::ChSignum(stress_m.z());
				this->section->GetElasticity()->ComputeStress(stress_n, stress_m, e_strain_e_new, e_strain_k_new);  //<<<<  elastic sigma(epsilon) 
																													// compute yeld
				strain_yeld_Mz = this->n_yeld_Mz->Get_y(mydata_new->p_strain_acc_k.z());		///<<<< sigma_y(p_strain_acc)
				eta_Mz = stress_m.z() - this->n_beta_Mz->Get_y(mydata_new->p_strain_k.z());	///<<<< beta(p_strain_acc)
				Fyeld_Mz = fabs(eta_Mz) - strain_yeld_Mz;  //<<<<  Phi(sigma,p_strain_acc)  

				++iters;
			}
		}
	}
	// te scalar plastic accumulator in this case is the sum of the single accumulators of the various degreees of freedom of the beam:
	mydata_new->p_strain_acc = mydata_new->p_strain_acc_e.x() + mydata_new->p_strain_acc_e.y() + mydata_new->p_strain_acc_e.z() +
						       mydata_new->p_strain_acc_k.x() + mydata_new->p_strain_acc_k.y() + mydata_new->p_strain_acc_k.z();

	return true;
};



////////////////////////////////////////////////////////////////////////////////////  




void ChPlasticityCosserat::ComputeStiffnessMatrixElastoplastic(
	ChMatrixDynamic<>& K,       ///< return the 6x6 material stiffness matrix values here
	const ChVector<>& strain_n, ///< tot strain (deformation part): x= elongation, y and z are shear
	const ChVector<>& strain_m, ///< tot strain (curvature part), x= torsion, y and z are line curvatures
	const ChBeamMaterialInternalData& data ///< get & return updated material internal variables, at this point, including {p_strain_m, p_strain_n, p_strain_acc}
) {
	ChVector<> astress_n;
	ChVector<> astress_m;
	ChVector<> me_strain_n_new; // needed only as placeholder
	ChVector<> me_strain_m_new; // needed only as placeholder

	std::vector< std::unique_ptr<ChBeamMaterialInternalData> > a_plastic_data;
	this->CreatePlasticityData(1, a_plastic_data);
	std::vector< std::unique_ptr<ChBeamMaterialInternalData> > b_plastic_data;
	this->CreatePlasticityData(1, b_plastic_data);

	bool in_plastic = ComputeStressWithReturnMapping(astress_n, astress_m, me_strain_n_new, me_strain_m_new, *a_plastic_data[0], strain_n, strain_m, data);

	if (!in_plastic) {
		// if no return mapping is needed at this strain state, just use elastic matrix:
		return this->section->GetElasticity()->ComputeStiffnessMatrix(K, strain_n, strain_m);
	}
	else {
		// if return mapping is needed at this strain state, compute the elastoplastic stiffness by brute force BDF
		double epsi = 1e-6;
		double invepsi = 1.0 / epsi;
		ChVector<> bstress_n;
		ChVector<> bstress_m;
		ChVector<> strain_n_inc = strain_n;
		ChVector<> strain_m_inc = strain_m;
		for (int i = 0;i < 2; ++i) {
			strain_n_inc[i] += epsi;
			this->ComputeStressWithReturnMapping(bstress_n, bstress_m, me_strain_n_new, me_strain_m_new, *b_plastic_data[0], strain_n_inc, strain_m_inc, data);
			K.PasteVector((bstress_n - astress_n)*invepsi, 0, i);
			K.PasteVector((bstress_m - astress_m)*invepsi, 3, i);
			strain_n_inc[i] -= epsi;
		}
		for (int i = 0;i < 2; ++i) {
			strain_m_inc[i] += epsi;
			this->ComputeStressWithReturnMapping(bstress_n, bstress_m, me_strain_n_new, me_strain_m_new, *b_plastic_data[0], strain_n_inc, strain_m_inc, data);
			K.PasteVector((bstress_n - astress_n)*invepsi, 0, i + 3);
			K.PasteVector((bstress_m - astress_m)*invepsi, 3, i + 3);
			strain_m_inc[i] -= epsi;
		}
	}
}




}  // end namespace fea
}  // end namespace chrono

