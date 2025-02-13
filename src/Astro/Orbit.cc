#include "Orbit.hh"
#include "Environment.hh"
#include "Attitude.hh"
#include "Quaternions.hh"
#include "InfluxDB.hh"
#include "GlobalSetting.hh"
#include "Mediator.hh"
COrbit::COrbit() : StartCode("SIM01"), fileds(15)
{
}
int COrbit::TwoBodRK4(double Ts)
{
	if (IsRV(J2000Inertial) == false)
	{
		printf("R:%f(m) V:%f(m/s)\n", J2000Inertial.Pos.norm(), J2000Inertial.Vel.norm());
		return -1;
	}
	Eigen::VectorXd RVState(6, 1);
	RVState.block<3, 1>(0, 0) = J2000Inertial.Pos;
	RVState.block<3, 1>(3, 0) = J2000Inertial.Vel;

	Eigen::VectorXd k1(6, 1), k2(6, 1), k3(6, 1), k4(6, 1);
	k1 = TwoBodAcc(RVState);
	k2 = TwoBodAcc(RVState + k1 * (0.5 * Ts));
	k3 = TwoBodAcc(RVState + k2 * (0.5 * Ts));
	k4 = TwoBodAcc(RVState + k3 * Ts);

	RVState = RVState + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
	J2000Inertial.Pos = RVState.block<3, 1>(0, 0);
	J2000Inertial.Vel = RVState.block<3, 1>(3, 0);
	return 0;
}

int COrbit::HPOPRK4(double Ts, Environment &env, const int64_t timestamp)
{
	if (IsRV(J2000Inertial) == false)
	{
		printf("R:%f(m) V:%f(m/s)\n", J2000Inertial.Pos.norm(), J2000Inertial.Vel.norm());
		return -1;
	}

	Eigen::VectorXd RVState(6, 1);
	RVState.block<3, 1>(0, 0) = J2000Inertial.Pos;
	RVState.block<3, 1>(3, 0) = J2000Inertial.Vel;
	Eigen::VectorXd k1(6, 1), k2(6, 1), k3(6, 1), k4(6, 1), RVTemp(6, 1);

	k1 = HPOPAcc(RVState, ECEFFix.Pos, env, timestamp);
	RVTemp = RVState + k1 * (0.5 * Ts);
	k2 = HPOPAcc(RVTemp, ECEFFix.Pos, env, timestamp);
	RVTemp = RVState + k2 * (0.5 * Ts);
	k3 = HPOPAcc(RVTemp, ECEFFix.Pos, env, timestamp);
	RVTemp = RVState + k3 * Ts;
	k4 = HPOPAcc(RVTemp, ECEFFix.Pos, env, timestamp);

	RVState = RVState + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
	J2000Inertial.Pos = RVState.block<3, 1>(0, 0);
	J2000Inertial.Vel = RVState.block<3, 1>(3, 0);
	return 0;
}

void COrbit::RenewFix(const int64_t timestamp)
{

	Eigen::Matrix3d Aif;
	Aif = Environment::ECI2ECEF(timestamp);
	ECEFFix.Pos = Aif * J2000Inertial.Pos;
	Eigen::Vector3d EarthAngularVelocityFixed;
	EarthAngularVelocityFixed << 0, 0, EARTH_RATE;
	ECEFFix.Vel = Aif * J2000Inertial.Vel - EarthAngularVelocityFixed.cross(ECEFFix.Pos);
}

LLA_t COrbit::Fix2LLA(const Eigen::Vector3d &fixpos)
{
	LLA_t LLA;
	double sqrt_x2y2 = SQRT(fixpos(0) * fixpos(0) + fixpos(1) * fixpos(1));
	double e2 = 1.0 - (EARTH_POLAR_RADIUS * EARTH_POLAR_RADIUS) / (EARTH_EQUATORIAL_RADIUS * EARTH_EQUATORIAL_RADIUS);
	double e_2 = (EARTH_EQUATORIAL_RADIUS * EARTH_EQUATORIAL_RADIUS) / (EARTH_POLAR_RADIUS * EARTH_POLAR_RADIUS) - 1.0;
	double belta = ATAN2(fixpos(2) * EARTH_EQUATORIAL_RADIUS, EARTH_POLAR_RADIUS * sqrt_x2y2);
	double Lat_tmp = sqrt_x2y2 - e2 * EARTH_EQUATORIAL_RADIUS * POW(cos(belta), 3);
	LLA.Lng = ATAN2(fixpos(1), fixpos(0));
	LLA.Lat = ATAN2(fixpos(2) + e_2 * EARTH_EQUATORIAL_RADIUS * POW(sin(belta), 3), Lat_tmp);
	double cosB = cos(LLA.Lat);
	double sinB = sin(LLA.Lat);
	if (cosB != 0)
	{
		LLA.Alt = sqrt_x2y2 / cosB - (EARTH_EQUATORIAL_RADIUS / (SQRT(1.0 - e2 * sinB * sinB)));
	}
	else
	{
		LLA.Alt = fixpos(2) - EARTH_POLAR_RADIUS * SIGN(fixpos(2));
	}
	return LLA;
}

RV COrbit::Fix2Inl(const int64_t timestamp, const RV &fix)
{
	RV InlRV;
	auto Afi = Environment::ECI2ECEF(timestamp).transpose();
	InlRV.Pos = Afi * fix.Pos;
	Eigen::Vector3d EarthAngularVelocityFixed;
	EarthAngularVelocityFixed << 0, 0, EARTH_RATE;
	InlRV.Vel = Afi * (fix.Vel + EarthAngularVelocityFixed.cross(fix.Pos));
	return InlRV;
}

void COrbit::RenewLLA()
{
	LLA = Fix2LLA(ECEFFix.Pos);
}

void COrbit::RenewLLR()
{

	LLR.Lng = ATAN2(ECEFFix.Pos(1), ECEFFix.Pos(0));
	LLR.Lat = ATAN2(ECEFFix.Pos(2), SQRT(ECEFFix.Pos(0) * ECEFFix.Pos(0) + ECEFFix.Pos(1) * ECEFFix.Pos(1)));
	LLR.Rds = ECEFFix.Pos.norm();
}

Eigen::Matrix3d COrbit::NED2ECEF()
{

	Eigen::Matrix3d res;
	Eigen::Matrix3d temres;
	double sin_lng = sin(LLR.Lng);
	double cos_lng = cos(LLR.Lng);
	double sin_lat = sin(LLR.Lat);
	double cos_lat = cos(LLR.Lat);
	res << -sin_lat * cos_lng, -sin_lat * sin_lng, cos_lat,
		-sin_lng, cos_lng, 0,
		-cos_lat * cos_lng, -cos_lat * sin_lng, -sin_lat;
	temres = res.inverse();
	return temres;
}

void COrbit::StateRenew(double Ts, const int64_t timestamp, Environment &env)
{
	TwoBodRK4(Ts);
	RenewFix(timestamp);
	RenewLLR();
	RenewLLA();
}

void COrbit::Init(int64_t Timestamp)
{
	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	StartCode = pCfg->Get<std::string>("/Orbit/StartCode");
	fileds = pCfg->Get<int>("/Orbit/fields");
	J2000Inertial.Pos << pCfg->Get<double>("/Orbit/J2000Inertial/Rx"),
		pCfg->Get<double>("/Orbit/J2000Inertial/Ry"),
		pCfg->Get<double>("/Orbit/J2000Inertial/Rz");

	J2000Inertial.Vel << pCfg->Get<double>("/Orbit/J2000Inertial/Vx"),
		pCfg->Get<double>("/Orbit/J2000Inertial/Vy"),
		pCfg->Get<double>("/Orbit/J2000Inertial/Vz");

	RenewFix(Timestamp);
	RenewLLR();
	RenewLLA();
	m_DM = Publisher::GetInstance();
	m_DM->Subscribe(this);

	auto hd = Mediator::GetInstance();
	for (int i = 0; i < fileds; i++)
	{
		std::string Code = GetCode(StartCode, i + 1);
		hd->add(Code, Addr(i));
	}
}

double *COrbit::Addr(int index)
{
	if (index >= 0 && index < 6)
		return J2000Inertial.Addr(index);
	else if (index >= 6 && index < 12)
		return ECEFFix.Addr(index - 6);
	else if (index >= 12 && index < 15)
		return LLA.Addr(index - 12);
	else
		return nullptr;
}

std::ostream &operator<<(std::ostream &_cout, const RV &j2000)
{
	_cout << "J2000 Pos(km) " << j2000.Pos(0) / 1000 << " " << j2000.Pos(1) / 1000 << " " << j2000.Pos(2) / 1000 << std::endl;
	_cout << "J2000 Vel(km/s) " << j2000.Vel(0) / 1000 << " " << j2000.Vel(1) / 1000 << " " << j2000.Vel(2) / 1000 << std::endl;
	return _cout;
}

Eigen::VectorXd COrbit::TwoBodAcc(const Eigen::VectorXd &InlRV)
{
	Eigen::VectorXd RVStateAcc(6, 1);
	RVStateAcc.setZero(6, 1);

	if (InlRV.size() != 6)
	{
		printf("invalid size in RVState\r\n");
		return RVStateAcc;
	}

	Eigen::MatrixXd Accmatrix(6, 6);
	Accmatrix.setZero();
	double tmp = 1 / J2000Inertial.Pos.norm();
	double tmp2 = -EARTH_GRAVITATIONAL * tmp * tmp * tmp;
	Accmatrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
	Accmatrix.block<3, 3>(3, 0) = tmp2 * Eigen::Matrix3d::Identity();

	RVStateAcc = Accmatrix * InlRV;
	return RVStateAcc;
}

Eigen::Vector3d COrbit::InlGravityAcc(const Eigen::Vector3d &ecefpos, const int64_t timestamp)
{
	Eigen::Vector3d pos(ecefpos);

	double R_dot = SQRT(pos.x() * pos.x() + pos.y() * pos.y());
	double R = pos.norm();
	double sin_pha = pos.z() / R;
	double lamda_G = ATAN2(pos.y(), pos.x());
	double sin_lamda_G = sin(lamda_G);
	double cos_lamda_G = cos(lamda_G);
	double sin_pha2 = sin_pha * sin_pha;

	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	size_t Order = pCfg->Get<int>("/Env/Gravity/GravityOrder");

	Eigen::ArrayXXd P(Order + 2, Order + 2);
	Eigen::ArrayXXd P_dot(Order + 2, Order + 2);
	P.setZero();
	P_dot.setZero();

	// 计算P矩阵
	double DM, DL;
	for (size_t M = 0; M <= Order; M++)
	{
		for (size_t L = 0; L <= Order; L++)
		{
			DM = static_cast<double>(M);
			DL = static_cast<double>(L);

			if (L == 0 && M == 0)
				P(L, M) = 1.0;
			else if (L == 1 && M == 0)
				P(L, M) = SQRT3 * sin_pha;
			else if (L >= 2 && M == 0)
				P(L, M) = SQRT((2.0 * DL + 1.0) / (2.0 * DL - 1.0)) *
						  ((2.0 - 1.0 / DL) * sin_pha * P(L - 1, 0) -
						   SQRT((2.0 * DL - 1.0) / (2.0 * DL - 3.0)) * (1.0 - 1.0 / DL) * P(L - 2, 0));
			else if (M == 1 && L == 1)
				P(L, M) = SQRT3 * SQRT(1.0 - sin_pha2);
			else if (L >= 2 && L == M)
				P(L, M) = SQRT((2.0 * DL + 1.0) / (2.0 * DL)) * SQRT(1.0 - sin_pha2) * P(L - 1, L - 1);
			else if (L >= 2 && M >= 1 && M < L)
				P(L, M) = SQRT((2.0 * DL + 1.0) * (2.0 * DL - 1.0) / ((DL + DM) * (DL - DM))) * sin_pha * P(L - 1, M) -
						  SQRT((2.0 * DL + 1.0) * (DL - 1.0 + DM) * (DL - 1.0 - DM) /
							   ((2.0 * DL - 3.0) * (DL + DM) * (DL - DM))) *
							  P(L - 2, M);
			else if (L < M)
				P(L, M) = 0;
		}
	}

	// 计算P_dot矩阵
	double pow1 = POW(1.0 - sin_pha2, -1.0);
	double pow5 = POW(1.0 - sin_pha2, -0.5);
	double sinpha_pow5 = sin_pha * pow5;

	for (size_t M = 0; M <= Order; M++)
	{
		for (size_t L = M; L <= Order; L++)
		{
			DM = static_cast<double>(M);
			DL = static_cast<double>(L);

			if (M == 0 && L == 1)
				P_dot(L, M) = SQRT3;
			else if (M == 0 && L > 1)
				P_dot(L, M) = DL * pow1 * (SQRT((2.0 * DL + 1.0) / (2.0 * DL - 1.0)) * P(L - 1, 0) - sin_pha * P(L, 0));
			else if (M == 0)
				P_dot(L, M) = 0;
			else
				P_dot(L, M) = pow5 * (SQRT((DL + DM + 1.0) * (DL - DM)) * P(L, M + 1) -
									  DM * sinpha_pow5 * P(L, M));
		}
	}

	Eigen::Vector3d Acc;
	Acc.setZero();
	for (size_t M = 0; M <= Order; M++)
	{
		double k5 = sin(M * lamda_G);
		double k6 = cos(M * lamda_G);
		for (size_t L = M; L <= Order; L++)
		{
			if (L < 2)
				continue;
			double C = pCfg->Getstokes_c()(L, M);
			double S = pCfg->Getstokes_s()(L, M);

			// 计算所需的中间变量
			double k1 = pow(1.0 / R, L + 3.0);
			double k2 = P(L, M);
			double k3 = P_dot(L, M);
			double k4 = ((L + 1.0) * k2 + sin_pha * k3);
			double k7 = C * k6 + S * k5;
			double k8 = (M / R_dot) * pow(1.0 / R, L + 1.0) * k2;
			double k9 = C * k5 - S * k6;

			// 累加到 Acc
			Acc.x() += k1 * k4 * pos.x() * k7 - k8 * k9 * sin_lamda_G;
			Acc.y() += k1 * k4 * pos.y() * k7 + k8 * k9 * cos_lamda_G;
			Acc.z() += k1 * (k4 * pos.z() - R * k3) * k7;
		}
	}

	auto Afi = Environment::ECI2ECEF(timestamp).transpose();

	return Afi * Acc * (-1);
}

Eigen::Vector3d COrbit::InlThirdBodyAcc(const Eigen::Vector3d &inlpos, const Eigen::Vector3d &thridpos, double miu)
{
	auto v_delta = inlpos - thridpos;
	double v_delta_norm = v_delta.norm();
	double v_delta_norm3 = v_delta_norm * v_delta_norm * v_delta_norm;

	double thridpos_norm = thridpos.norm();
	double thridpos_norm3 = thridpos_norm * thridpos_norm * thridpos_norm;

	Eigen::Vector3d Acc = thridpos / thridpos_norm3 - v_delta / v_delta_norm3;

	return -miu * Acc;
}

Eigen::Vector3d COrbit::InlFDragAcc(const Eigen::VectorXd &InlRV, const Eigen::Vector3d &ecefpos, const Eigen::Vector3d &sunvec)
{
	Eigen::Vector3d acc;
	acc.setZero();
	auto Pcfg = GlobalSettings::GetInstance();
	double cd = Pcfg->Get<double>("/Satellite/DragCd");
	double area = Pcfg->Get<double>("/Satellite/DragArea");
	double mass = Pcfg->Get<double>("/Satellite/Mass");
	double density = Environment::GetDensity(ecefpos, sunvec);

	Eigen::Vector3d vr;
	vr << InlRV(3) + 0.0588336 * InlRV(1), InlRV(4) - 0.0588336 * InlRV(0), InlRV(5);
	double v_norm = vr.norm();
	acc = -0.5 * cd * area / mass * density * v_norm * vr;
	return acc;
}

Eigen::Vector3d COrbit::InlSunPressureAcc(const Eigen::Vector3d &inlpos, const Eigen::Vector3d &sunvec)
{
	auto Pcfg = GlobalSettings::GetInstance();
	double area = Pcfg->Get<double>("/Satellite/SunPressureArea");
	double mass = Pcfg->Get<double>("/Satellite/Mass");
	double cr = Pcfg->Get<double>("/Satellite/SunPressureCr");

	Eigen::Vector3d earth_pos(Eigen::Vector3d::Zero());
	uint32_t state = IsLighting(inlpos, earth_pos, EARTH_EQUATORIAL_RADIUS, sunvec, SUN_RADIUS_M);

	if (state != UMBRA)
	{
		Eigen::Vector3d sunDir = sunvec.normalized(); // 太阳方向矢量归一化
		Eigen::Vector3d solarPressureAcc = -ROU_SR * area / mass * cr * sunDir;
		return solarPressureAcc;
	}
	return Eigen::Vector3d::Zero();
}

Eigen::VectorXd COrbit::HPOPAcc(const Eigen::VectorXd &InlRV, const Eigen::Vector3d &fixpos, Environment &env, const int64_t timestamp)
{
	Eigen::Vector3d Inlpos = InlRV.block<3, 1>(0, 0);
	Eigen::Vector3d distur(Eigen::Vector3d::Zero());
	// 引力场
	distur += InlGravityAcc(fixpos, timestamp);
	// 太阳和月球三体摄动
	distur += InlThirdBodyAcc(Inlpos, env.SunVecInl, SUN_GRAVITATIONAL);
	distur += InlThirdBodyAcc(Inlpos, env.MoonVecInl, MOON_GRAVITATIONAL);
	// 大气阻力
	distur += InlFDragAcc(InlRV, fixpos, env.SunVecInl);
	// 太阳光压
	distur += InlSunPressureAcc(Inlpos, env.SunVecInl);
	// 二体
	Eigen::VectorXd RVAcc = TwoBodAcc(InlRV);
	RVAcc.block<3, 1>(3, 0) += distur;
	return RVAcc;
}

void COrbit::Submit()
{
	if (!m_DM)
		return;
	int index = 1;
	for (int i = 0; i < 6; i++)
	{
		m_DM->add<double>(GetCode(StartCode, index), J2000Inertial[i]);
		index++;
	}
	for (int i = 0; i < 6; i++)
	{
		m_DM->add<double>(GetCode(StartCode, index), ECEFFix[i]);
		index++;
	}
	for (int i = 0; i < 3; i++)
	{
		m_DM->add<double>(GetCode(StartCode, index), LLA[i]);
		index++;
	}
}