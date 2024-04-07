/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:53:10
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-02-26 11:00:24
 * @FilePath: /Satellite/include/General/InfluxDB.h
 * @Description: 
 */
#pragma once
#include <string>
#include <ctime> // ���ڻ�ȡʱ��
#include <cstring> // ����memset�Ⱥ���

// ���ݲ�ͬ�Ĳ���ϵͳ������ͬ��ͷ�ļ�
#ifdef _WIN32
    #include<Windows.h>
    #pragma comment(lib,"ws2_32.lib") // Windows��Ҫ�Ŀ�
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h> // �ṩclose����
    typedef int SOCKET; // �ڷ�Windowsϵͳ�ж���SOCKETΪint
    #define closesocket close
    #define INVALID_SOCKET (-1)
    #define SOCKET_ERROR   (-1)
#endif
class CInfluxDB
{
public:
    CInfluxDB();
    ~CInfluxDB();
    // ����measurement
    void setMeasurement(const std::string& measurement);
    // �鿴measurement
    void printMeasurement() const;
    // ���Ӽ�ֵ�� ==> str2
    template<typename type1>
    void addKeyValue(const std::string& field, const type1& value);
    // �鿴str2
    void printStr2() const;
    // UDP����InfluxDB
    void sendUdp();
    // �����ϴη������ݿ�ʱ�両����
    bool IsSend(double Period) const;
    // �����������str2
    void ResetStr2();
private:
    SOCKET s;
    std::string str1; // �洢measurements
    std::string str2; // �洢��ֵ��key ==> value
    // �������ݿ�
    sockaddr_in serverAddr;
    mutable int64_t LastRenewTime;
};

template<typename type1>
inline void CInfluxDB::addKeyValue(const std::string& field, const type1& value)
{
    str2 += "," + field + "=" + std::to_string(value);
}

