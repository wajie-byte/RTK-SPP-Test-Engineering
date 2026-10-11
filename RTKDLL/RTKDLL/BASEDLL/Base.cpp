#include"Base.h"

// 格里高利历转儒略日
double gregorianToJulianDate(int year, int month, int day, int hour, int min, double sec)
{
	// 处理月份和年份
	if (month <= 2)
	{
		year -= 1;
		month += 12;
	}
	int A = year / 100;
	int B = 2 - A + A / 4;
	// 儒略日计算公式
	double jd = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + day + B - 1524.5;
	// 添加时间部分
	double time = hour / 24.0 + min / 1440.0 + sec / 86400.0;
	jd += time;
	return jd;
}

// 儒略日转GPS时间
bool julianDateToGPSTime(double jd, GPSTime& GPStime)
{
	double seconds = 0.0;
	// GPS起始时间的儒略日(1980年1月6日午夜UTC)
	const double gpsEpochJD = 2444244.5;
	{
		// 计算天数差
		double daysDiff = jd - gpsEpochJD;
		if (daysDiff < 0)
		{
			std::cerr << "Error: Date is before GPS epoch (1980-01-06)" << std::endl;
			return false;
		}

		//计算秒数
		seconds = daysDiff * 86400.0;
		GPStime.week = static_cast<int>(seconds / 604800.0);
		GPStime.sow = fmod(seconds, 604800.0);
		return true;
	}
}
// 格里高利历转GPS时间(通过儒略日), 返回周内秒
void gregorianToGPSTime(int year, int month, int day, int hour, int min, double sec, GPSTime& GPStime)
{
	// 先转换为儒略日
	double jd = gregorianToJulianDate(year, month, day, hour, min, sec);

	// 再从儒略日转换为GPS时间
	julianDateToGPSTime(jd, GPStime);
	return;
}

// 处理日期进位（完整实现）
void handleDateCarry(int& year, int& month, int& day, int& hour, int& min, double& sec)
{
	// 首先处理秒进位到分
	if (sec >= 60.0)
	{
		int extra_minutes = static_cast<int>(sec) / 60;
		sec = fmod(sec, 60.0);
		min += extra_minutes;
	}

	// 处理分进位到时
	if (min >= 60)
	{
		int extra_hours = min / 60;
		min %= 60;
		hour += extra_hours;
	}

	// 处理时进位到日
	if (hour >= 24)
	{
		int extra_days = hour / 24;
		hour %= 24;
		day += extra_days;
	}

	// 处理日进位到月
	while (day > daysInMonth(year, month))
	{
		day -= daysInMonth(year, month);
		month++;

		// 处理月进位到年
		if (month > 12)
		{
			month = 1;
			year++;
		}
	}
}

// 获取某年某月的天数（完整实现）
int daysInMonth(int year, int month)
{
	if (month < 1 || month > 12) 
	{
		throw std::invalid_argument("月份必须在1-12之间");
	}
	static const int days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

	// 处理闰年二月
	if (month == 2 && isLeapYear(year))
		return 29;

	return days[month - 1];
}

// 判断是否为闰年（完整实现）
bool isLeapYear(int year)
{
	if (year < 0)
	{
		throw std::invalid_argument("年份不能为负数");
	}
	if (year % 4 != 0) return false;
	if (year % 100 != 0) return true;
	return (year % 400 == 0);
}


// 添加：儒略日转BDS时间
bool julianDateToBDSTime(double jd, BDSTime& BDStime)
{
	double seconds = 0.0;
	// BDS起始时间的儒略日(2006年1月1日午夜UTC)
	const double bdsEpochJD = 2453736.5;

	// 计算天数差
	double daysDiff = jd - bdsEpochJD;
	if (daysDiff < 0)
	{
		std::cerr << "Error: Date is before BDS epoch (2006-01-01)" << std::endl;
		return false;
	}

	// 计算秒数
	seconds = daysDiff * 86400.0;
	BDStime.week = static_cast<int>(seconds / 604800.0);
	BDStime.sow = fmod(seconds, 604800.0);
	return true;
}

// 添加：格里高利历转BDS时间
void gregorianToBDSTime(int year, int month, int day, int hour, int min, double sec, BDSTime& BDStime)
{
	// 先转换为儒略日
	double jd_utc = gregorianToJulianDate(year, month, day, hour, min, sec);
	double jd_bdt = jd_utc - 14.0 / 86400.0;
	// 再从儒略日转换为BDS时间
	julianDateToBDSTime(jd_bdt, BDStime);
	return;
}

YMDHMS BDT2UTC(const BDSTime bdt)
{
	YMDHMS utc;
	// BDT时间起点：2006年1月1日00:00:00 UTC
	const int BDT_START_YEAR = 2006;
	const int BDT_START_MONTH = 1;
	const int BDT_START_DAY = 1;
	const int BDT_START_WEEK = 0;  // BDT第0周开始

	// 常量定义
	const double SECONDS_PER_WEEK = 604800.0;  // 一周的秒数
	const double SECONDS_PER_DAY = 86400.0;    // 一天的秒数
	const int LEAP_SECONDS_BDT_UTC = 14;       // BDT与UTC的闰秒差
	try 
	{
		// 计算总秒数并减去闰秒差
		double total_seconds = bdt.week * SECONDS_PER_WEEK + bdt.sow - LEAP_SECONDS_BDT_UTC;

		if (total_seconds < 0) 
		{
			throw std::invalid_argument("计算后的时间早于BDT起始时间");
		}

		// 计算总天数
		int total_days = static_cast<int>(total_seconds / SECONDS_PER_DAY);
		double remaining_seconds = fmod(total_seconds, SECONDS_PER_DAY);

		// 从起始日期开始计算
		int year = BDT_START_YEAR;
		int month = BDT_START_MONTH;
		int day = BDT_START_DAY;

		// 累积天数
		for (int i = 0; i < total_days; i++) 
		{
			day++;
			if (day > daysInMonth(year, month))
			{
				day = 1;
				month++;
				if (month > 12)
				{
					month = 1;
					year++;
				}
			}
		}

		// 计算时分秒
		int hour = static_cast<int>(remaining_seconds / 3600);
		int minute = static_cast<int>(fmod(remaining_seconds, 3600) / 60);
		double second = fmod(remaining_seconds, 60);

		utc.year = year;
		utc.month = month;
		utc.day = day;
		utc.hour = hour;
		utc.min = minute;
		utc.sec = second;
	}
	catch (const std::exception& e) 
	{
		std::cerr << "BDT转UTC转换错误: " << e.what() << std::endl;
	}

	return utc;
}

// GPS时间转BDS时间（周秒形式）
BDSTime gpsToBds(const GPSTime& gps_time)
{
	const double SECONDS_PER_WEEK = 604800.0;
	const int GPS_TO_BDS_OFFSET = 14;
	const double GPS_TO_BDS_EPOCH_DIFF = 820108800.0;

	BDSTime bds_time;
	double gps_total_seconds = gps_time.week * SECONDS_PER_WEEK + gps_time.sow;
	double bds_total_seconds = gps_total_seconds - GPS_TO_BDS_OFFSET - GPS_TO_BDS_EPOCH_DIFF;

	bds_time.week = static_cast<int>(bds_total_seconds / SECONDS_PER_WEEK);
	bds_time.sow = bds_total_seconds - (bds_time.week * SECONDS_PER_WEEK);

	return bds_time;
}