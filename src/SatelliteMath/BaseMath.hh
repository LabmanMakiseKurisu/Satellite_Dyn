#pragma once
#include <iostream>
#include <errno.h>
#include <stdint.h>
#include <cmath>
#include <Eigen/Dense>

#define M_PI 3.14159265358979323846
#define ZERO (1e-12)
#define MATH_E (2.71828182845904523536)         // e
#define HALFPI (1.57079632679489661923)         // pi/2
#define TWOPI (6.28318530717958647692)          // 2*pi
#define DEG2RAD (0.01745329251994329576)        // pi/180
#define RAD2DEG (57.2957795130823208767)        // 180/pi
#define SEC2DEG (2.777777777777777778e-4)       // 1/3600
#define SEC2RAD (4.848136811095359936e-6)       // 1/3600*DEG2RAD
#define SQRT2 (1.41421356237309504880)          // sqrt(2)
#define SQRT3 (1.73205080756887729352)          // sqrt(3)
#define SQRT5 (2.23606797749978969640)          // sqrt(5)
#define LIGHT_VELOCITY (299792458.458)          //(m/s)
#define EARTH_RADIUS_M (6371230.000000000)      //(m)
#define EARTH_ECCENTRICITY (0.081819190842621)  //
#define EARTH_RATE (7.292115859146235E-05)      // 15.04106717866910/3600* pi/180
#define EARTH_EQUATORIAL_RADIUS (6378137.0)     //(m)
#define EARTH_POLAR_RADIUS (6356752.31424)      //(m)
#define EARTH_GRAVITATIONAL (398600441500000.0) // miu_earth
#define SUN_GRAVITATIONAL (1.327E20)            // miu_sun
#define MOON_GRAVITATIONAL (4.902793455E12)     // miu_moon
#define SUN_RADIUS_M (695508000.0)              //
#define MOON_RADIUS_M (1737400.0)               //
#define ROU_SR (4.56e-6)                        //
#define UMBRA (0x00000000U)                     // 本影
#define PENUMBRA (0x11111111U)                  // 半影
#define LIGHTING (0x22222222U)                  // 光照
#define J2 (0.00108263)                         //
#define J3 (-2.5356e-6)
#define J4 (-0.00000162336) //

#define GAUSS2T(Gauss) ((Gauss) * 0.0001)

#define T2GAUSS(t) ((t) * 10000.0)

#define NT2T(nt) ((nt) * 1.0E-9)

#define T2NT(t) ((t) * 1.0E9)

#define DEG(rad) ((rad) * RAD2DEG)

//
// brief  : �Ƕ�ת����
//
#define RAD(deg) ((deg) * DEG2RAD)

//
// brief  : ����Ƕȵ�sinֵ
//
#define SIND(deg) (sin(RAD(deg)))

//
// brief  : ����Ƕȵ�cosֵ
//
#define COSD(deg) (cos(RAD(deg)))

//
// brief  : ת��rmpת���ٶ�
//
#define RPM2VEL(rpm) ((rpm) * (0.10471975511965977461)) // 2PI/60

//
// brief  : ���ٶ�תrmp
//
#define VEL2RPM(vel) ((vel) * (9.54929658551372014613)) // 60/2PI

// brief  : MJDתJD
//
#define MJD2JD(mjd) ((mjd) - 2400000.5) // MJD -> JD

//
// brief  : JDתMJD
//
#define JD2MJD(jd) ((jd) + 2400000.5) // JD -> MJD

//
// brief  : JD����1970�����������
//
#define JD2DAYS(jd) ((jd) - 2440587.5) // Total days from 1970-1-1

//
// brief  : MJD����1970�����������
//
#define MJD2DAYS(mjd) ((mjd) - 40587.0) // Total days from 1970-1-1

//
// brief  : ʱ���(s)תJD
//
#define TS2JD(timestamp) (((timestamp) - 1609459200.0) / 86400.0 + 2459215.5) // time stamp to JD

//
// brief  : ʱ���(s)ת����������
//
#define TS2CEN(timestamp) ((TS2JD(timestamp) - 2451545.0) / 36525.0) // time stamp to JD centry

//
// brief  : JDתʱ���(s)
//
#define JD2TS(jd) (((jd) - 2459215.5) * 86400.0 + 1609459200.0) // JD to time stamp

//
// brief  : ȡ����
//
#define SIGN(v) (v < 0 ? (-1) : (1))

//
// brief  : �Ƚ������������Ƿ���ȣ���ֵ��С��
//
#define EQUALS(a, b) (fabs((D64_t)((a) - (b))) < ZERO)

//
// brief  : �Ƚ�a��b�Ĳ�С��v
//
#define EQUALSVAL(a, b, v) (fabs((D64_t)((a) - (b))) < fabs((D64_t)(v)))

//
// brief  : �ж���ֵ�Ƿ�Ϊ0
//
#define IS_ZERO(v) (fabs((D64_t)(v)) < ZERO)

//
// brief  : �ж�˫���������Ƿ�Ϸ�
//
#define IS_DOUBLE(v) (FP_NORMAL == std::fpclassify(v) || FP_ZERO == std::fpclassify(v))

//
// brief  : �жϸ������Ƿ�Ϸ�
//
#define IS_FLOAT(v) IS_DOUBLE(v)

//
// brief  : ȡ���߽ϴ���
//
#define _MAX(v1, v2) ((v1) > (v2) ? (v1) : (v2))

//
// brief  : ȡ���ߵĽ�Сֵ
//
#define _MIN(v1, v2) ((v1) < (v2) ? (v1) : (v2))

//
// brief  : ��ֵ�ķ�ֵ�޷�
//
#define LIMIT(v, min, max) _MAX(_MIN(v, max), min)

//
// brief  :�ԳƱ���
//
#define SATURATION(v, max) LIMIT(v, -max, max)

//
// brief  : �ж��Ƿ�ΪNULL
//
#define IS_NULL(v) (NULL == (v))

//
// brief  : �ж��Ƿ�Ϊ��NULL
//
#define NOT_NULL(v) (NULL != (v))

// �������Ҿ���ת��
#define Dcm_X_AXIS 0U
#define Dcm_Y_AXIS 1U
#define Dcm_Z_AXIS 2U

// ŷ����ת��
#define EUL_SQE_ZYX (0x00U)
#define EUL_SQE_ZYZ (0x11U)
#define EUL_SQE_ZXY (0x22U)
#define EUL_SQE_ZXZ (0x33U)
#define EUL_SQE_YXZ (0x44U)
#define EUL_SQE_YXY (0x55U)
#define EUL_SQE_YZX (0x66U)
#define EUL_SQE_YZY (0x77U)
#define EUL_SQE_XYZ (0x88U)
#define EUL_SQE_XYX (0x99U)
#define EUL_SQE_XZY (0xAAU)
#define EUL_SQE_XZX (0xBBU)
#define EUL_SQE_DEFAULT (0x00U)

//
// brief  : ���ȵ�����0 ~ 2pi
//
double RAD_2PI(double value);

//
// brief  : ����ת��-pi ~ pi
//
double RAD_PI(double value);

//
// brief  : ���ͳ�0����
//
int PROTECT_INT(int value);

//
// brief  : ���ͳ�0����
//
unsigned PROTECT_UINT(unsigned value);

//
// brief  : �����ͳ�0����
//
double PROTECT(double value);

//
// brief  : acos���㣬������ֵ�޷�[-1,1]
//
double ACOS(double value);

//
// brief  : asin���㣬������ֵ�޷�[-1,1]
//
double ASIN(double value);

//
// brief  : sqrt����,���ƶ�����
//
double SQRT(double value);

//
// brief  : ������
//
double POW(double X, double Y);

//
// brief  : X��Yȡģ����������
//
double MOD(double X, double Y);

//
// brief  : X��Yȡģ��Int32��
//
int MOD_INT(int X, int Y);

//
// brief  : X��Yȡģ��Uint32��
//
unsigned MOD_UINT(unsigned X, unsigned Y);

//
// brief  : ������ֵ
//
double ATAN2(double Y, double X);

//
// brief  : 
//
double LOG(double Value);

//
// brief  :
//
double LOG2(double Value);

//
// brief  : 
//
double LOG10(double Value);

//
// brief  : 
//
double DoubleFactorial(double Value);

//
// brief  : 
//
double Factorial(double Value);

//
// brief  : 
//
int LunarMonthDays(int year);

//
// brief  : 
//
double DecYear(int year, double month, double day);

uint32_t IsLighting(const Eigen::Vector3d &Target, const Eigen::Vector3d &Obstruction, double ObstructionRadius, const Eigen::Vector3d &LightSource, double LightSourceRadius);