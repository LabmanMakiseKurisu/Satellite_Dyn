/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-02-26 09:17:49
 * @FilePath: /Satellite/src/General/CConfig.cpp
 * @Description: 
 */
#include"CConfig.h"
#include<fstream>
#include"IniConfig.h"
CConfig::DeleteHelper CConfig::helper;

CConfig::CConfig()
{
	CIniConfig data("Config/Init.ini");
	SatelliteTime = data.ReadInt64_t("Satellite", "SatelliteTime");

	Rx = data.ReadDouble("J2000Inertial", "Rx");
	Ry = data.ReadDouble("J2000Inertial", "Ry");
	Rz = data.ReadDouble("J2000Inertial", "Rz");

	Vx = data.ReadDouble("J2000Inertial", "Vx");
	Vy = data.ReadDouble("J2000Inertial", "Vy");
	Vz = data.ReadDouble("J2000Inertial", "Vz");

	a = data.ReadDouble("OrbitElement", "a");
	e = data.ReadDouble("OrbitElement", "e");
	i = RAD(data.ReadDouble("OrbitElement", "i"));
	RAAN = data.ReadDouble("OrbitElement", "RAAN");
	omega = data.ReadDouble("OrbitElement", "omega");
	M = data.ReadDouble("OrbitElement", "M");

	Wx = data.ReadDouble("Omega_b", "Wx");
	Wy = data.ReadDouble("Omega_b", "Wy");
	Wz = data.ReadDouble("Omega_b", "Wz");

	Q0 = data.ReadDouble("Qib", "Q0");
	Q1 = data.ReadDouble("Qib", "Q1");
	Q2 = data.ReadDouble("Qib", "Q2");
	Q3 = data.ReadDouble("Qib", "Q3");

	Jxx = data.ReadDouble("SatInaMat", "Jxx");
	Jyy = data.ReadDouble("SatInaMat", "Jyy");
	Jzz = data.ReadDouble("SatInaMat", "Jzz");
	Jxy = data.ReadDouble("SatInaMat", "Jxy");
	Jxz = data.ReadDouble("SatInaMat", "Jxz");
	Jyz = data.ReadDouble("SatInaMat", "Jyz");

	//�ش����
	MagOrder = data.ReadDouble("Mag", "MagOrder");
	if ((MagOrder < 1) || (MagOrder > 12))
	{
		MagOrder = 2;
	}
	Eigen::Index rows = (MagOrder + 1) * (MagOrder + 2)/2 - 1;
	gauss_g.resize(MagOrder+1, MagOrder+1);
	gauss_h.resize(MagOrder + 1, MagOrder + 1);
	gauss_gdot.resize(MagOrder + 1, MagOrder + 1);
	gauss_hdot.resize(MagOrder + 1, MagOrder + 1);
	gauss_g.setZero();
	gauss_h.setZero();
	gauss_gdot.setZero();
	gauss_hdot.setZero();

	//�������ļ���ȡ����
	std::ifstream file("Config/wmm_2020_data.txt");
	if (file.is_open())
	{
		//������
		int row, col;
		for (int i = 0; i < rows; i++)
		{
			//ÿ��6�����ݷֱ��� r c g h gdot hdot
			file >> row;
			file >> col;
			file >> gauss_g(row, col);
			file >> gauss_h(row, col);
			file >> gauss_gdot(row, col);
			file >> gauss_hdot(row, col);
		}
		file.close();
	}
	else
	{
		std::cerr << "Unable to open file" << std::endl;
		exit(0);
	}
}
CConfig* CConfig::GetInstance()
{
	if (m_instance == NULL)
		m_instance = new CConfig;
	return m_instance;
}

void CConfig::ReleaseInstance()
{
	CConfig* tmp = m_instance;
	m_instance = NULL;
	delete tmp;
}
