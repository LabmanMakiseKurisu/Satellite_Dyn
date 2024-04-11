/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:01
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-11 19:18:33
 * @FilePath: /Satellite/src/Astro/AttitudeControl.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include"Subscriber.hh"
class GyroScope;
class SunSensor;
class StarSensor;
class MagSensor;
class GNSS;
class Flywheel;
class Com_Schedule;

class CAttitudeController: public ::ISubscriber
{
public:
	enum Mode
	{
		RATEDAMP,
		SUNPOINT,
		EARTHPOINT
	};
	Mode workmode;//
	Eigen::Vector3d TorqueRef;//Nm
	Eigen::Matrix3d Kp;//
	Eigen::Matrix3d Kd;//
	double MaxTorque;//
public:
	CAttitudeController();

	void Init();

	Eigen::Vector3d TorqueRefRenew(Com_Schedule* pCom);

	virtual void Submit() override;
private:

	//@brief: 
	void RateDamping(const GyroScope& _Gyro);

	//@brief: 
	void ToSunControl(const GyroScope& _Gyro, const SunSensor& _Sun);

	//@brief: 
	void ToEarthControl(const GyroScope& _Gyro, const StarSensor& _Star, const GNSS& _gnss);

private:
	std::string StartCode;
	int fileds;
	double *Addr(int index);
};

