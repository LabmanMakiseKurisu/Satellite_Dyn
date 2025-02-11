#include "Environment.hh"
#include"sofaDLL.h"
#include"Orbit.hh"
#include"Attitude.hh"
#include"InfluxDB.hh"
#include"GlobalSetting.hh"
#include"Mediator.hh"
Environment::Environment()
{
	BodyMag << 0, 0, 0;
	NEDMag << 0, 0, 0;
	SunVecInl << 0, 0, 0;
	SunVecBody << 0, 0, 0;
}

double *Environment::Addr(int index)
{
	if (index >= 0 && index < 3)
		return &SunVecInl[index];
	else if (index >= 3 && index < 6)
		return &SunVecBody[index - 3];
	else if (index >= 6 && index < 9)
		return &NEDMag[index - 6];
	else if (index >= 9 && index < 12)
		return &BodyMag[index - 9];
	else
		return nullptr;
}

void Environment::Init()
{
	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	StartCode = pCfg->Get<std::string>("/Env/StartCode");
	fileds = pCfg->Get<int>("/Env/fields");
	m_DM = Publisher::GetInstance();
	m_DM->Subscribe(this);

	auto hd = Mediator::GetInstance();
	for (int i = 0; i < fileds; i++)
	{
		std::string Code = GetCode(StartCode, i + 1);
		hd->add(Code, Addr(i));
	}
}

Eigen::Matrix3d Environment::ECI2ECEF(const int64_t timestamp, const double deltaUT1, const double xp, const double yp)
{
	double tmpres[3][3];
	GetECI2ECEF(timestamp, tmpres, deltaUT1, xp, yp);
	Eigen::Matrix3d res;
	res << tmpres[0][0], tmpres[0][1], tmpres[0][2],
		tmpres[1][0], tmpres[1][1], tmpres[1][2],
		tmpres[2][0], tmpres[2][1], tmpres[2][2];
	return res;
}

void Environment::SunPos(const int64_t timestamp)
{
	double TJD; double M; long double lamM; long double rs; double bs;
	Eigen::Matrix3d Rx;
	Eigen::Vector3d sunpos;

	TJD = TS2CEN(timestamp/1000);
	M = 357.5256 + 35999.049 * TJD;
	lamM = 282.94 + M + SEC2DEG * 6892 * SIND(M) + SEC2DEG * 72 * SIND(M);
	rs = (149.619 - 2.499 * COSD(M) - 0.021 * COSD(2.0 * M)) * 1e9;  
	bs = 23.4393 - 46.815 / 3600 * TJD - 0.00059 / 3600 * TJD * TJD;
	sunpos << rs * COSD(lamM), rs* SIND(lamM), 0;  
	Rx << 1, 0, 0,
		0, COSD(-bs), SIND(-bs),
		0, -SIND(-bs), COSD(-bs);

	//SunVecInl = Rx * sunpos / rs;
	SunVecInl = Rx * sunpos;
}

void Environment::MoonPos(const int64_t timestamp)
{
	// 计算儒略世纪
	double TJD = TS2CEN(timestamp / 1000);

	// 月亮的基本角度
	double L0 = 218.31617 + 481267.88088 * TJD - 1.3972 * TJD;
	double L = 134.96292 + 477198.86753 * TJD;
	double L1 = 357.52543 + 35999.04944 * TJD;
	double F = 93.27283 + 483202.01873 * TJD;
	double D = 297.85027 + 445267.11135 * TJD;

	// 月亮的长半轴
	double lamM = L0 + 22640.0 / 3600.0 * SIND(L) + 729.0 / 3600.0 * SIND(2 * L) - 4586.0 / 3600.0 * SIND(L - 2 * D) + 2370.0 / 3600.0 * SIND(2 * D) - 668.0 / 3600.0 * SIND(L1) - 412.0 / 3600.0 * SIND(2 * F) - 212.0 / 3600.0 * SIND(2 * L - 2 * D) - 206.0 / 3600.0 * SIND(L + L1 - 2 * D) + 192.0 / 3600.0 * SIND(L + 2 * D) - 165.0 / 3600.0 * SIND(L1 - 2 * D) + 148.0 / 3600.0 * SIND(L - L1) - 125.0 / 3600.0 * SIND(D) - 110.0 / 3600.0 * SIND(L + L1) - 55.0 / 3600.0 * SIND(2 * F - 2 * D);

	// 月亮的纬度修正
	double bM = 18520.0 / 3600.0 * SIND(F + lamM - L0) + 412.0 / 3600.0 * SIND(2 * F) + 541.0 / 3600.0 * SIND(L1) - 526.0 / 3600.0 * SIND(F - 2 * D) + 44.0 / 3600.0 * SIND(L + F - 2 * D) - 31.0 / 3600.0 * SIND(F - L - 2 * D) - 25.0 / 3600.0 * SIND(F - 2 * L) - 23.0 / 3600.0 * SIND(L1 + F - 2 * D) + 21.0 / 3600.0 * SIND(F - L) + 11.0 / 3600.0 * SIND(F - L1 - 2 * D);

	// 月亮的距离修正
	double rM = (385000.0 - 20905.0 * COSD(L)) - 3699.0 * COSD(2 * D - L) - 2956.0 * COSD(2 * D) - 570.0 * COSD(2 * L) + 246.0 * COSD(2 * L - 2 * D) - 205.0 * COSD(L1 - 2 * D) - 171.0 * COSD(L + 2 * D) - 152.0 * COSD(L + L1 - 2 * D);

	rM *= 1e3; // 转换为米

	// 计算地球黄道平面到赤道的变换矩阵
	Eigen::Matrix3d Rx;
	Rx << 1, 0, 0,
		0, COSD(-23.43929111), SIND(-23.43929111),
		0, -SIND(-23.43929111), COSD(-23.43929111);

	// 根据计算得到的参数，得到月球位置向量
	Eigen::Vector3d moonpos;
	moonpos << rM * COSD(lamM) * COSD(bM),
		rM * SIND(lamM) * COSD(bM),
		rM * SIND(bM);

	// 将月球位置向量转换为地心地固坐标系
	MoonVecInl = Rx * moonpos;
}

void Environment::GetNEDMag(const COrbit& Orbit, const int64_t timestamp)
{

	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	size_t Order = pCfg->Get<int>("/Env/Mag/MagOrder");

	YMD m_ymd = UTCTimeStamp2YMD(timestamp);
	double epoch = DecYear(2020,1,1);
	double dt_change = DecYear(m_ymd.year, m_ymd.month, m_ymd.day) - epoch;

	Eigen::ArrayXXd g(Order + 1, Order + 1);
	Eigen::ArrayXXd h(Order + 1, Order + 1);
	g = pCfg->Getgauss_g() + pCfg->Getgauss_gdot() * dt_change;
	h = pCfg->Getgauss_h() + pCfg->Getgauss_hdot() * dt_change;

	Eigen::ArrayXXd P(Order + 2, Order + 2);
	P.setZero();
	double x = sin(Orbit.LLR.Lat);

	P(0, 0) = 1;
	for (int n = 1; n < P.rows(); n++)
	{
		P(n, n) = (-1)*(2 * n - 1) * POW(1 - x * x, 0.5) * P(n - 1, n - 1);
	}
	
	for (int n = 0; n < P.rows()-1; n++)
	{
		P(n + 1, n) = x * (2 * n + 1) * P(n, n);
	}

	for (int n = 2; n < P.rows(); n++)
	{
		for (int m = 0; m < n - 1; m++)
		{
			P(n, m) = 1.0 / (n - m) * (x * (2 * n - 1) * P((n - 1), m) - (n + m - 1) * P((n - 2), m));
		}
	}

	for (int n = 0; n < P.rows(); n++)
	{
		for (int m = 0; m < n + 1; m++)
		{
			if (m > 0)
			{
				P(n, m) = POW(-1.0, m) * SQRT(2.0 * Factorial(n - m) / Factorial(n + m)) * P(n, m);
			}
		}
	}
	
	Eigen::ArrayXXd dP(Order+1, Order+1);
	dP.setZero();
	for (int n = 0; n < dP.rows(); n++)
	{
		for (int m = 0; m < n + 1; m++)
		{
			dP(n, m) = (n + 1) * tan(Orbit.LLR.Lat) * P(n, m) - SQRT((n + 1) * (n + 1) - m * m) * (1 / cos(Orbit.LLR.Lat)) * P(n + 1, m);
		}
	}

	double X_prime = 0;
	double Y_prime = 0;
	double Z_prime = 0;
	double sumtempX = 0;
	double sumtempY = 0;
	double sumtempZ = 0;
	Eigen::ArrayXXd tempX(1, Order + 1);
	Eigen::ArrayXXd tempY(1, Order + 1);
	Eigen::ArrayXXd tempZ(1, Order + 1);

	for (int n = 1; n < Order + 1; n++)
	{
		tempX.setZero();
		tempY.setZero();
		tempZ.setZero();
		sumtempX = 0;
		sumtempY = 0;
		sumtempZ = 0;

		for (int m = 0; m < n + 1; m++)
		{
			tempX(m) = (g(n, m) * cos(m * (Orbit.LLR.Lng) + h(n, m) * sin(m * (Orbit.LLR.Lng)))) * dP(n, m);
			tempY(m) = m * (g(n, m) * sin(m * (Orbit.LLR.Lng)) - h(n, m) * cos(m * (Orbit.LLR.Lng))) * P(n, m);
			tempZ(m) = (g(n, m) * cos(m * (Orbit.LLR.Lng) + h(n, m) * sin(m * (Orbit.LLR.Lng)))) * P(n, m);
		}
	
		for (int i = 0; i < n + 1; i++)
		{
			sumtempX += tempX(i);
			sumtempY += tempY(i);
			sumtempZ += tempZ(i);
		}

		X_prime += POW((EARTH_RADIUS_M / Orbit.LLR.Rds), (n + 2)) * sumtempX;
		Y_prime += POW((EARTH_RADIUS_M / Orbit.LLR.Rds), (n + 2)) * sumtempY;
		Z_prime += (n + 1) * POW((EARTH_RADIUS_M / Orbit.LLR.Rds), (n + 2)) * sumtempZ;
	}
	
	X_prime = - X_prime;
	Y_prime = 1 / cos(Orbit.LLR.Lat) * Y_prime;
	Z_prime = - Z_prime;

	double BN = X_prime * cos(Orbit.LLR.Lat - Orbit.LLA.Lat) - Z_prime * sin(Orbit.LLR.Lat - Orbit.LLA.Lat);
	double BE = Y_prime;
	double BD = X_prime * sin(Orbit.LLR.Lat - Orbit.LLA.Lat) + Z_prime * cos(Orbit.LLR.Lat - Orbit.LLA.Lat);

	NEDMag << BN, BE, BD;
	NEDMag = NT2T(NEDMag);
}

Eigen::Vector3d Environment::GetInlGravityAcc(const COrbit &Orbit, const int64_t timestamp)
{
	Eigen::Vector3d pos(Orbit.ECEFFix.Pos);

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

void CalcPolarAngles(Eigen::Vector3d& m_Vec, double& m_phi, double& m_theta, double& m_r) {
    double rhoSqr = m_Vec[0] * m_Vec[0] + m_Vec[1] * m_Vec[1];
    m_r = SQRT(rhoSqr + m_Vec[2] * m_Vec[2]); 

    // 计算方位角 m_phi
    if (m_Vec[0] == 0 && m_Vec[1] == 0)
        m_phi = 0;
    else
        m_phi = ATAN2(m_Vec[1], m_Vec[0]);
    
    if (m_phi < 0)
        m_phi += 2 * M_PI;

    double rho = SQRT(rhoSqr);
    if (m_Vec[2] == 0 && rho == 0)
        m_theta = 0;
    else
        m_theta = ATAN2(m_Vec[2], rho);
}

double Environment::GetDensity(const COrbit &Orbit)
{
	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	int cof = pCfg->Get<int>("/Env/Density/F107");
	auto &param_vec = pCfg->GetF107()[cof];

	// 获取卫星的地理坐标
	auto &r_TOD = Orbit.ECEFFix.Pos;
	double lon = Orbit.LLA.Lng;
	double lat = Orbit.LLA.Lat;
	double height = Orbit.LLA.Alt / 1000.0;

	double ra_lag = 0.523599; // 右升交点延迟 [rad]
	int n_prm = 4;			  // Harris-Priester模型参数

	// 计算太阳的右升交点和赤纬角
	double ra_Sun, dec_Sun, r_Sun_mag;
	CalcPolarAngles(SunVecInl, ra_Sun, dec_Sun, r_Sun_mag);

	// 计算单位向量 u，指向日膨胀极点
	double c_dec = cos(dec_Sun);
	std::vector<double> u = {c_dec * cos(ra_Sun + ra_lag), c_dec * sin(ra_Sun + ra_lag), sin(dec_Sun)};

	// 计算卫星位置与日膨胀极点的夹角余弦
	double dot_product = r_TOD[0] * u[0] + r_TOD[1] * u[1] + r_TOD[2] * u[2];
	double norm_r_TOD = SQRT(r_TOD[0] * r_TOD[0] + r_TOD[1] * r_TOD[1] + r_TOD[2] * r_TOD[2]);
	double c_psi2 = 0.5 + 0.5 * (dot_product / norm_r_TOD);

	// 寻找高度区间
	int ih = 0;
	for (size_t i = 0; i < param_vec.size() - 1; ++i)
	{
		if (height >= param_vec[i][0] && height < param_vec[i + 1][0])
		{
			ih = i;
			break;
		}
	}

	// 计算指数插值的密度
	double h_min = (param_vec[ih][0] - param_vec[ih + 1][0]) / LOG(param_vec[ih + 1][1] / param_vec[ih][1]);
	double h_max = (param_vec[ih][0] - param_vec[ih + 1][0]) / LOG(param_vec[ih + 1][2] / param_vec[ih][2]);

	double d_min = param_vec[ih][1] * exp((param_vec[ih][0] - height) / h_min);
	double d_max = param_vec[ih][2] * exp((param_vec[ih][0] - height) / h_max);

	// 计算最终密度
	double density = d_min + (d_max - d_min) * pow(c_psi2, n_prm);
	return density * 1e-9; // 转换为 kg/m^3
}

void Environment::StateRenew(CAttitude &Attitude, COrbit &Orbit, const int64_t timestamp)
{
	GetNEDMag(Orbit, timestamp);
	Eigen::Matrix3d Ane = Orbit.NED2ECEF();
	Eigen::Vector3d ECEFMag = Ane * NEDMag;
	//
	Eigen::Matrix3d  Aif = Environment::ECI2ECEF(timestamp);
	Eigen::Vector3d ECIMag = Aif.inverse() * ECEFMag;
	//
	BodyMag = Attitude.Qib.ToDcm() * ECIMag;
	SunPos(timestamp);
	MoonPos(timestamp);
	SunVecBody = Attitude.Qib.ToDcm() * SunVecInl;
}

void Environment::Submit()
{
	if (!m_DM)
		return;
	int index = 1;
	for (int i = 0; i < 3; i++)
	{
		m_DM->add<double>(GetCode(StartCode, index), SunVecInl[i]);
		index++;
	}
	for (int i = 0; i < 3; i++)
	{
		m_DM->add<double>(GetCode(StartCode, index), SunVecBody[i]);
		index++;
	}
	for (int i = 0; i < 3; i++)
	{
		m_DM->add<double>(GetCode(StartCode, index), T2GAUSS(NEDMag[i]));
		index++;
	}
	for (int i = 0; i < 3; i++)
	{
		m_DM->add<double>(GetCode(StartCode, index), T2GAUSS(BodyMag[i]));
		index++;
	}
}