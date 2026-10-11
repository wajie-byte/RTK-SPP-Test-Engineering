#include"DefData.h"
#include"../SatePos/SatePos.h"
#include"../OEMRead/Parse.h"
void PosDataGet(vector<SatePosPara>& satepospara, vector<RecPosPara>& recPospara)
{
    recPospara.clear();  // 清空输出向量

    if (satepospara.empty()) {
        cout << "错误：输入数据为空！" << endl;
        return;
    }

    int validEpochs = 0;

    for (int i = 0; i < satepospara.size(); i++)
    {
        // 每个历元创建独立的对象
        RecPosPara recpospara;

        // 设置时间信息
        recpospara.year = satepospara[i].year;
        recpospara.month = satepospara[i].month;
        recpospara.day = satepospara[i].day;
        recpospara.hour = satepospara[i].hour;
        recpospara.min = satepospara[i].min;
        recpospara.sec = satepospara[i].sec;

        // 检查当前历元各向量长度是否一致
        int numSats = satepospara[i].commonPRN.size();
        if (numSats != satepospara[i].C1C.size() ||
            numSats != satepospara[i].Pos.size() ||
            numSats != satepospara[i].Clock_bias.size()) {

            cout << "警告：历元 " << i << " 数据长度不一致，跳过" << endl;
            continue;
        }

        // 直接赋值而不是逐个push_back（更高效）
        recpospara.commonPRN = satepospara[i].commonPRN;
        recpospara.C1C = satepospara[i].C1C;
        recpospara.SatePos = satepospara[i].Pos;
        recpospara.Clock_bias = satepospara[i].Clock_bias;

        // 只添加有足够卫星的历元
        if (recpospara.commonPRN.size() >= 4) 
        {
            recPospara.push_back(recpospara);
            validEpochs++;
        }
    }

    cout << "接收机位置数据获取完成！共处理 " << validEpochs
        << " 个有效历元，总计 " << endl;
}




