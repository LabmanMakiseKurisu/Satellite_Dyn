#pragma once
#include<stdint.h>
#define MYSOFA_API 


struct YMD
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	double second;
};

//@brief: ʹ��UTC����ʱ��תUTCʱ���
//@para : YMD* ymd ����ʱ��
//@return : UTCʱ�������λms
MYSOFA_API int64_t UTCYMD2TimeStamp(const YMD* ymd);


//@brief: ʹ��UTCʱ���תUTC����ʱ��
//@para :	int64_t timestamp :UTCʱ�������λs
//			YMD ����ʱ��(���ڲ���)
//@return : UTCʱ�������λms
MYSOFA_API YMD UTCTimeStamp2YMD(const int64_t timestamp);

/*
//@brief: ʹ��UTC����ʱ�����UTC������
//@para : utcymd ������ �� �� ʱ �� ��
		  double* d1, double* d2 2-part UTC������(���ڲ���)
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


//@brief: UTC������תTAI������
//@para : double utc1, double utc2 UTC������
//		  double* tai1, double* tai2 TAI������(���ڲ���)
//@return :                 
MYSOFA_API int UTC2TAI(double utc1, double utc2, double* tai1, double* tai2);

//@brief: UTC������תUT1������
//@para : double utc1, double utc2 UTC������
//        dut1 UTC-TU1����
//		  double* ut11, double* ut12 TAI������(���ڲ���)
//@return :                 
MYSOFA_API int UTC2UT1(double utc1, double utc2, double dut1, double* ut11, double* ut12);

//@brief: TAI������תTT������
//@para : double tai1, double tai2 TAI������
//		  double* tt1, double* tt2 TT������(���ڲ���)
//@return :  
MYSOFA_API int TAI2TT(double tai1, double tai2, double* tt1, double* tt2);



//@brief: �������ϵת�ع�ϵ��ת�ƾ���
//@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
//@return : none
MYSOFA_API void GetECI2ECEF(const int64_t timestamp, double rc2t[3][3], const double deltaUT1 = 0, const double xp = 0, const double yp = 0);
