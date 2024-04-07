/*
 * @Author: Amadeus
 * @Date: 2024-04-07 18:47:35
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 20:01:25
 * @FilePath: /Satellite/src/Componet/Sensor.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include"Subscriber.hh"
#include"InfluxDB.hh"
#include<iostream>
template <typename T>
class Sensor : public ::ISubscriber
{
protected:
    Eigen::Matrix3d InstallMatrix;
    int64_t LastRenewTime;
    double SamplePeriod;
    int id;//设备ID
    int Fields;//字段数
    std::string StartCode;//起始码
    T Data; // ֵ

public:
    Sensor(Eigen::Matrix3d &IM, int64_t timestamp, double Ts, int _id, int _size, std::string _StartCode)
    {
        InstallMatrix = IM;
        LastRenewTime = timestamp;
        SamplePeriod = Ts;
        id = _id;
        Fields = _size;
        StartCode = _StartCode;
        m_DM = DataManager::GetInstance();
    }
    virtual ~Sensor() {}

    virtual void StateRenew(int64_t NowTime, T &_Data) = 0;
    virtual void Init(int64_t timestamp, T &_Data) = 0;
};
