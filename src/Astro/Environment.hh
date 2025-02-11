/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:01
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-11 19:06:07
 * @FilePath: /Satellite/src/Astro/Environment.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include"Subscriber.hh"

class COrbit;
class CAttitude;

class Environment: public ::ISubscriber
{
public:
	Eigen::Vector3d SunVecInl;//惯性系太阳矢量(m)
	Eigen::Vector3d SunVecBody;//本体系太阳矢量(m)
	Eigen::Vector3d MoonVecInl;//惯性系月球矢量(m)
	Eigen::Vector3d NEDMag;//NED系地磁场(Telsa)
	Eigen::Vector3d BodyMag;//本体系地磁场(Telsa)

	Environment();
	void Init();
	static Eigen::Matrix3d ECI2ECEF(const int64_t timestamp, const double deltaUT1 = 0, const double xp = 0, const double yp = 0);

	//计算惯性系太阳矢量
	void SunPos(const int64_t timestamp);

	//计算惯性系月球矢量
	void MoonPos(const int64_t timestamp);

	void GetNEDMag(const COrbit& Orbit, const int64_t timestamp);

	//计算惯性系引力场干扰
	Eigen::Vector3d GetInlGravityAcc(const COrbit &Orbit, const int64_t timestamp);

	//计算HP模型大气密度
	double GetDensity(const COrbit& Orbit);

	void StateRenew(CAttitude& Attitude, COrbit& Orbit, const int64_t timestamp);

	virtual void Submit() override;
private:
    std::string StartCode;
    int fileds;
	double* Addr(int index);
};