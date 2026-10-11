#include<iostream>
#include<vector>
#include<fstream>
#include<iomanip>
#include<map>
#include"../RTKData/DefData.h"
#include"Parse.h"
#include"../Config/config.h"
using namespace std;

#define SecInWeek 604800
#define ms2s 1000

#define BDSEPHEMERIS_ID 1696
#define RANGE_ID 43
#define GPSEPHEM_ID 7
#define IONUTC_ID 8

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static int32_t  I4(uint8_t* p) { int32_t  i; memcpy(&i, p, 4); return i; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }
static double   R8(uint8_t* p) { double   r; memcpy(&r, p, 8); return r; }


const uint8_t SYNC_BYTES[] = { 0xAA, 0x44, 0x12 };//同步头
#pragma pack(push, 1) // 按1字节对齐
struct BinHeader
{
    unsigned char sync[3];           // 同步字节: 0xAA, 0x44, 0x12
    unsigned char header_length;     // 头部长度
    unsigned short message_id;       // 消息ID (小端序)
    char message_type;      // 消息类型
    unsigned char port_address;      // 端口地址
    unsigned short message_length;   // 消息长度 (小端序)
    unsigned short sequence;         // 序列号 (小端序)
    unsigned char idle_time;         // 空闲时间
    unsigned char time_status;       // 时间状态
    unsigned short gps_week;         // GPS周数 (小端序)
    unsigned long gps_seconds;      // GPS秒数 (小端序)
    unsigned long receiver_status;  // 接收机状态 (小端序)
    unsigned short reserved;         // 保留字段 (小端序)
    unsigned short receiver_version; // 接收机版本 (小端序)
};
#pragma pack(pop) // 恢复默认对齐方式

// Novatel 消息结构
struct BinMessage
{
    BinHeader header;          // 头部
    std::vector<uint8_t> body;  // 消息体
    uint32_t crc;               // CRC校验码
    bool crc_valid;             // CRC验证结果
	BinMessage(): crc(0), crc_valid(false) {} // 构造函数初始化crc_valid为false
};









bool FindBinSync(std::ifstream& rfile);// 查找同步字节
void OEMRead(GNSSConfig gnssconfig);// 读取OEM7消息
uint32_t* generateCRC32Table();// 生成CRC32查找表
uint32_t* getCRC32Table();// 获取CRC32查找表
uint32_t calculateCRC32(const unsigned char* data, int length);//计算CRC32
bool verifyCRC(const BinMessage& msg);//校验CRC
void Decode(const BinMessage& message, map<string, vector<BDSEPHEMERIS>>& bds_eph_map, map<string, vector<GPSEPHEM>>& gps_eph_map,
    OBSS& obss, IONUTC& ionutc, bool& is_eph, bool& is_obs);
//void Decode(const BinMessage& message, std::vector<SATE>& sates, OBSS& obss, IONUTC& ionutc);
bool Single_POS(const OBSS& obss, map<string, vector<BDSEPHEMERIS>>& bds_eph_map, map<string, vector<GPSEPHEM>>& gps_eph_map
    , OBSS& valid_obss,IONUTC ionutc, Point& recpos, SPPResult& lsresult, GNSSConfig gnssconfig);

void RTK_FILE_OEM7(GNSSConfig gnssconfig);

bool DecodeBDSEphEMERIS(const BinMessage& message,
    map<string, vector<BDSEPHEMERIS>>& eph_map, double exptime);
bool DecodeGPSEphEM(const BinMessage& message,
    map<string, vector<GPSEPHEM>>& eph_map, double exptime);//expdate星历保质期