/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:11
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-10 18:00:05
 * @FilePath: /Satellite/src/Componet/Com_Schedule.hh
 * @Description: 
 */
#pragma once

#include<iostream>
#include <variant>
#include<vector>
#include"BaseMath.hh"
#include"Subscriber.hh"
class CAttitude;
class COrbit;
class Environment;
class CAttitudeController;
class GyroScope;
class SunSensor;
class StarSensor;
class MagSensor;
class GNSS;
class Flywheel;

using ComsVariant = std::variant<
	std::vector<GyroScope *> *,
	std::vector<SunSensor *> *,
	std::vector<StarSensor *> *,
	std::vector<MagSensor *> *,
	std::vector<GNSS *> *,
	std::vector<Flywheel *> *>;

class Com_Schedule : public ::ISubscriber
{
public:
	static Com_Schedule* GetInstance();
	void Init(CAttitude& Att, COrbit& Obt, Environment& Env, CAttitudeController& ACtrl, int64_t timestamp);
	void StateRenew(int64_t timestamp, CAttitudeController &ACtrl);
	virtual void Submit() override;
public:
	size_t GyroNums;
	size_t FlywheelNums;
	size_t MagSensorNums;
	size_t StarSensorNums;
	size_t SunSensorNums;
	size_t GnssNums;
	std::vector<GyroScope*> Gyros;
	std::vector<SunSensor*> SunSensors;
	std::vector<StarSensor*> StarSensors;
	std::vector<MagSensor*> MagSensors;
	std::vector<GNSS*> GNSSs;
	std::vector<Flywheel*> Wheels;
	std::vector<ComsVariant> Coms;
	std::vector<double> WheelsTref;
private:
	static inline Com_Schedule* m_instance{ NULL };

	Com_Schedule();
	~Com_Schedule();
	Com_Schedule(const Com_Schedule& _CComponet) = delete;
	Com_Schedule& operator=(const Com_Schedule& _CComponet) = delete;
	void WheelsTrefCal(Eigen::Vector3d& TrefBody);

	static void ReleaseInstance();
	class DeleteHelper
	{
	public:
		DeleteHelper() = default;
		~DeleteHelper()
		{
			ReleaseInstance();
		}
	};
	static DeleteHelper helper;
};



