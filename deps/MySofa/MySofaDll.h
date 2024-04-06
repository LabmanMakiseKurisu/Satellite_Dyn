#pragma once
#include<stdint.h>
#ifdef  MYSOFADLL_EXPORTS//这个要在IDE中预定义
#define  MYSOFA_API __declspec(dllexport)//如果在本项目中，则为导出库
#else
#define MYSOFA_API __declspec(dllimport)//其他项目调用时，导入库
#endif

struct YMD
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	double second;
};

//@brief: 使用UTC公历时间转UTC时间戳
//@para : YMD* ymd 公历时间
//@return : UTC时间戳，单位ms
MYSOFA_API int64_t UTCYMD2TimeStamp(const YMD* ymd);


//@brief: 使用UTC时间戳转UTC公历时间
//@para :	int64_t timestamp :UTC时间戳，单位s
//			YMD 公历时间(出口参数)
//@return : UTC时间戳，单位ms
MYSOFA_API YMD UTCTimeStamp2YMD(const int64_t timestamp);

/*
//@brief: 使用UTC公历时间计算UTC儒略日
//@para : utcymd 公历年 月 日 时 分 秒
		  double* d1, double* d2 2-part UTC儒略日(出口参数)
//@return : +3 = both of next two
			+2 = time is after end of day(Note 5)
			+1 = dubious year(Note 6)
			0 = OK
			-1 = bad year
			-2 = bad month
			-3 = bad day
			-4 = bad hour
			-5 = bad minute
			-6 = bad second(< 0)
*/
MYSOFA_API int GetUTCJulianDate(YMD* utcymd, double* d1, double* d2);


//@brief: UTC儒略日转TAI儒略日
//@para : double utc1, double utc2 UTC儒略日
//		  double* tai1, double* tai2 TAI儒略日(出口参数)
//@return :                 
MYSOFA_API int UTC2TAI(double utc1, double utc2, double* tai1, double* tai2);

//@brief: UTC儒略日转UT1儒略日
//@para : double utc1, double utc2 UTC儒略日
//        dut1 UTC-TU1参数
//		  double* ut11, double* ut12 TAI儒略日(出口参数)
//@return :                 
MYSOFA_API int UTC2UT1(double utc1, double utc2, double dut1, double* ut11, double* ut12);

//@brief: TAI儒略日转TT儒略日
//@para : double tai1, double tai2 TAI儒略日
//		  double* tt1, double* tt2 TT儒略日(出口参数)
//@return :  
MYSOFA_API int TAI2TT(double tai1, double tai2, double* tt1, double* tt2);


//@brief: 计算惯性系转地固系的转移矩阵
//@para : timestamp: utc时间戳(ms) deltaUT1:UTC-UT1(s) xp,yp:极移(rad)  rc2t:转移矩阵结果
//@return : none
MYSOFA_API void GetECI2ECEF(const int64_t timestamp, double rc2t[3][3], const double deltaUT1 = 0, const double xp = 0, const double yp = 0);
