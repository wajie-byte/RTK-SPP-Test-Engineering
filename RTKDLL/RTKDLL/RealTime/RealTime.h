#ifndef REALTIME_H
#define REALTIME_H

#define MAXRAWLEN 40960

#define CeHuiXueYuanIP  "8.148.22.229" 
#define CeHuiXueYuanPort1 5002
#define CeHuiXueYuanPort2 7002
#define GongXueBuIP  "47.114.134.129"   
#define GongXueBuPort 7190

bool ProcessReceiverData(unsigned char* decBuff, int& lenD,
    std::map<std::string, std::vector<BDSEPHEMERIS>>& eph_map,
    Point& recpos, OBSS& valid_obss, int& message_count, int& valid_message_count, LSResult& lsresult, int pos_mode);

void RealTime_SPP(const std::string& server_ip, int server_port, int pos_mode);
void OEMRead_Universal(const std::string& source, int pos_mode, bool is_network, int port);
bool ProcessDataAndSPP(unsigned char* decBuff, int& lenD,std::map<std::string, std::vector<BDSEPHEMERIS>>& eph_map,
    Point& recpos, vector<OBSS>& Valid_obss, int& message_count, int& valid_message_count, vector<LSResult>& lsresults, int pos_mode);
bool GetVecOBS(unsigned char* decBuff, int& lenD,
    vector<OBSS>& obss, int& message_count, int& valid_message_count);
void RTK_RealTime(const std::string& server_ip1, int server_port1,
    const std::string& server_ip2, int server_port2, int pos_mode, const char* filename,int run_time_seconds = 0);
void ProcessRTK(vector<OBSS>& base_obss_vec, vector<OBSS>& rover_obss_vec,
    vector<LSResult>& base_lsresult_vec, vector<LSResult>& rover_lsresult_vec, ofstream& fout);
#endif // !REALTIME_H
