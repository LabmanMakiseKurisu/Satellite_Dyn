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
	Eigen::Vector3d SunVecInl;//
	Eigen::Vector3d SunVecBody;//
	Eigen::Vector3d NEDMag;//T
	Eigen::Vector3d BodyMag;//T

	Environment();
	void Init();
	static Eigen::Matrix3d ECI2ECEF(const int64_t timestamp, const double deltaUT1 = 0, const double xp = 0, const double yp = 0);

	void SunPos(const int64_t timestamp);

	void GetNEDMag(const COrbit& Orbit, const int64_t timestamp);

	void StateRenew(CAttitude& Attitude, COrbit& Orbit, const int64_t timestamp);

	virtual void Submit() override;
private:
    std::string StartCode;
    int fileds;
	double* Addr(int index);
};