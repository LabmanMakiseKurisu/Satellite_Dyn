/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-11 19:54:44
 * @FilePath: /Satellite/src/Componet/Actuators/Flywheel.cc
 * @Description: 
 */
#include"Flywheel.hh"
#include"InfluxDB.hh"
#include"APIHandler.hh"

Flywheel::Flywheel(double *_s,
				   Eigen::Vector3d &InsVet, double _inertia,
				   double _Tau, double _MaxSpeed,
				   double _MaxTref, double _Kp, double _Ki,
				   int64_t timestamp, int _id,double _Delta,
				   int _size, std::string _StartCode)
	: Componet<double, WheelData>(_s, timestamp, _id, _size, _StartCode)
{
	Data = WheelData(InsVet, _inertia, _Tau, _MaxSpeed, _MaxTref, _Kp, _Ki);
	Delta = _Delta;
	m_DM->Subscribe(this);
}
void Flywheel::Init(int64_t timestamp) {
	auto hd = Handler::GetInstance();
	for (int i = 1; i <= Fields; i++)
	{
		int diff = i + (id - 1) * Fields;
		std::string Code = GetCode(StartCode, diff);
		hd->add(Code, Data.Addr(i - 1));
	}
}

void Flywheel::StateRenew(int64_t NowTime) {
	double TRef = SATURATION(*source, Data.MaxTref);
	double SRef = TRef * Delta / Data.intertia + Data.SpeedRef;

	double cof1 = 1 - Data.Kp / Data.Tau - Data.Ki * Delta / Data.Tau;
	double cof2 = 1 - cof1;
	double lastspeed = Data.Speed;
	Data.Speed = SATURATION(cof1 * lastspeed + cof2 * Data.SpeedRef, Data.MaxSpeed);
	Data.Torque = Data.intertia * (Data.Speed - lastspeed) / Delta;
	Data.Momentum = Data.intertia * Data.Speed;
	
	Data.SpeedRef = SRef;
	Data.TorqueRef = TRef;
}

void Flywheel::Submit() {
	if(!m_DM)
		return;
	for(int i=1;i<=Fields;i++) {
		int diff = i + (id-1)*Fields;
		std::string Code = GetCode(StartCode, diff);
		m_DM->add<double>(Code,Data[i-1]);
	}
}

Eigen::Vector3d &Flywheel::GetInsVet()
{
	return Data.InstallVet;
}

double Flywheel::GetSpeed() { return Data.Speed; }
double Flywheel::GetMomentum() { return Data.Momentum; }
double Flywheel::GetTorque() { return Data.Torque; }