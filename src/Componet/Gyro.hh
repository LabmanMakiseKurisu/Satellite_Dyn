#pragma once
#include"BaseMath.hh"
#include "Sensor.hh"

class GyroScope : public Sensor<Eigen::Vector3d>
{
public:
	GyroScope(Eigen::Matrix3d &IM, int64_t timestamp, double Ts, int _id, int _size = 3, std::string Code = "SIM04");

public:
	virtual void StateRenew(int64_t NowTime, Eigen::Vector3d& Omega_b) override;
	virtual void Init(int64_t timestamp,Eigen::Vector3d& Omega_b) override;
	virtual void Submit() override;
};

