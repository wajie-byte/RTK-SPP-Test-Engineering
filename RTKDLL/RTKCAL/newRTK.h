#ifndef NEWRTK_H
#define NEWRTK_H
#include"../RTKData/DefData.h"
#include"../OEMRead/Parse.h"
#include"../MATDLL/Mat.h"




struct DDOBSS
{

	Point Base_pos;
	Point Rover_pos;
	vector<string> sat_prns;
	vector<Point> SatPos;
	vector<double> elev;
	OBSS base;
	OBSS rover;
};

struct PSR_ADR
{
	double psr;
	double adr;
	double psr_sigma;
	double adr_sigma;
	PSR_ADR()
	{
		psr = adr = 0.0;
		psr_sigma = adr_sigma = 0.0;
	}
};

struct DualOBS
{
	
	GPSTime GPStime;
	BDSTime BDStime;
	vector<string> sat_prns;
	vector<double> elev;
	vector<Point> SatPos;
	Point Base_pos;
	Point Rover_pos;
	vector<PSR_ADR> Base_B1I;
	vector<PSR_ADR> Base_B3I;
	vector<PSR_ADR> Rover_B1I;
	vector<PSR_ADR> Rover_B3I;
};



OBSS Match_RoverObs_BaseLS(const OBSS& rover_obs, const SPPResult& base_lsresult);
void Match_RoverObs_BaseObs(const OBSS& rover_obs, const OBSS& base_obs, OBSS& rover, OBSS& base);
DDOBSS Match_BaseObs_BaseLS(const OBSS& base, const OBSS& rover, const SPPResult& base_lsresult, const SPPResult& rover_lsresult);
DualOBS Match_Process_DD(const DDOBSS& dd_obss);
void BuildDDMat(const DualOBS& dd, Mat& A, Mat& L, Mat& P, RTKResult& rtk_result);
bool DoubleDifMatch(const OBSS& base_obs, const OBSS& rover_obs, const SPPResult& base_lsresult, const SPPResult& rover_lsresult, DualOBS& dd);
void Solve_RTK(const OBSS& base_obs, const OBSS& rover_obs, const SPPResult& base_lsresult, const SPPResult& rover_lsresult, RTKResult& rtk_result);
void UpdatePositionWithFixedAmbiguity_r(Mat& X, const Mat& N_inv, const double* F_fixed, int n_ambi, RTKResult& rtk_result);
void disp_rtk(const RTKResult& rtk_result);
void BuildP_Matrix(int n_other,
	const vector<double>& elev_other,
	double ref_elev,
	double sigma_phase,
	double sigma_code,
	Mat& P);
#endif // !NEWRTK_H
