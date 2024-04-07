/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:01
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 16:54:11
 * @FilePath: /Satellite/src/Astro/Environment.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
// 2023-12-22 14:44:18
# include "AllHead.hh"

class COrbit;
class CAttitude;
class Environment
{
public:
	Eigen::Vector3d BodyMag;//
	Eigen::Vector3d NEDMag;//
	Eigen::Vector3d SunVecInl;//
	Eigen::Vector3d SunVecBody;//

	Environment();

	static Eigen::Matrix3d ECI2ECEF(const int64_t timestamp, const double deltaUT1 = 0, const double xp = 0, const double yp = 0);

	void SunPos(const int64_t timestamp);

	void GetNEDMag(const COrbit& Orbit, const int64_t timestamp);

	void StateRenew(CAttitude& Attitude, COrbit& Orbit, const int64_t timestamp);

	void record(CInfluxDB& DB);
};