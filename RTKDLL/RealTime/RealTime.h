#ifndef REALTIME_H
#define REALTIME_H

#define MAXRAWLEN 40960

#define CeHuiXueYuanIP  "8.148.22.229" 
#define CeHuiXueYuanPort1 5002
#define CeHuiXueYuanPort2 7002
#define GongXueBuIP  "47.114.134.129"   
#define GongXueBuPort 7190



void RealTime_SPP(GNSSConfig gnssconfig);
void OEMRead_Universal(GNSSConfig gnssconfig);
bool ProcessDataAndSPP(unsigned char* decBuff, int& lenD,
    std::map<std::string, std::vector<BDSEPHEMERIS>>& bds_eph_map, std::map<std::string, std::vector<GPSEPHEM>>& gps_eph_map
	, Point& recpos, vector<OBSS>& Valid_obss, vector<SPPResult>& lsresults, IONUTC& ionutc,
    GNSSConfig gnssconfig, bool is_disp);
bool GetVecOBS(unsigned char* decBuff, int& lenD,
    vector<OBSS>& obss, int& message_count, int& valid_message_count);
void RTK_RealTime(GNSSConfig gnssconfig);
void ProcessRTK(vector<OBSS>& base_obss_vec, vector<OBSS>& rover_obss_vec,
    vector<SPPResult>& base_lsresult_vec, vector<SPPResult>& rover_lsresult_vec, ofstream& fout);
#endif // !REALTIME_H
