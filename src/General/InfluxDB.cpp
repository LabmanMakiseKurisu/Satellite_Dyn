/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-02-26 10:44:53
 * @FilePath: /Satellite/src/General/InfluxDB.cpp
 * @Description: 
 */
#include"InfluxDB.h"
#include"SimTime.h"
#include"IniConfig.h"
CInfluxDB::CInfluxDB()
{
	CIniConfig data("Config/Database.ini");
	setMeasurement(data.ReadString("InfluxDB", "Measurement"));
    #ifdef _WIN32
	WSADATA wsaData;
        // ��ʼ��Winsock
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        {
            std::cerr << "Failed. Error Code : " << strerror(errno);
            exit(0);
        }
    #endif
	// ���� socket
	if ((s = socket(PF_INET, SOCK_DGRAM, 0)) == SOCKET_ERROR)
	{
		std::cerr << "socket() failed with error code : " << strerror(errno);
		exit(0);
	}
	// ���÷�������ַ
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = inet_addr(data.ReadString("InfluxDB", "HostName").c_str()); // ���� InfluxDB ��ַ
	serverAddr.sin_port = htons(data.ReadInt("InfluxDB", "Port"));
	LastRenewTime = GetTimeStampMs();
}

CInfluxDB::~CInfluxDB()
{   // ����������������
    closesocket(s);
    #ifdef _WIN32
        WSACleanup();
    #endif
}

void CInfluxDB::setMeasurement(const std::string& measurement)
{
    str1.clear();
    str1 = measurement + " SIM000=1";
    //"measurement,tag1=value1,tag2=value2 field1=value1,field2=value2 timestamp"
}

void CInfluxDB::printMeasurement() const {
    if (!str1.empty()) {
        std::cout << "measurement: " << str1 << std::endl;
    }
    else {
        std::cout << "measurement is empty." << std::endl;
    }
}


void CInfluxDB::printStr2() const
{
    std::cout << "str2: " << str2 << std::endl;
}

void CInfluxDB::sendUdp()
{
    if (sendto(s, str2.c_str(), (int)str2.length(), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "sendto() failed with error code : " << strerror(errno);
    }
    else {
        //std::cout << "Message sent successfully." << std::endl;
    }
    ResetStr2();
}

void CInfluxDB::ResetStr2() {
    str2.clear();
    str2 = str1;
}

bool CInfluxDB::IsSend(double Period) const
{
    if (GetTimeStampMs() - LastRenewTime > Period * 1e3)
    {
        LastRenewTime = GetTimeStampMs();
        return true;
    }
    return false;
}