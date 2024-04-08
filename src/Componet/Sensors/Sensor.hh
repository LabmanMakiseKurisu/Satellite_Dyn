/*
 * @Author: Amadeus
 * @Date: 2024-04-07 18:47:35
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 18:53:48
 * @FilePath: /Satellite/src/Componet/Sensors/Sensor.hh
 * @Description: 
 */
#pragma once
#include<iostream>
#include"BaseMath.hh"
#include"Componet.hh"
#include"InfluxDB.hh"
template <typename T1, typename T2>
class Sensor : public ::Componet<T1, T2>
{
protected:
    Eigen::Matrix3d InstallMatrix;
    double SamplePeriod;

public:
    Sensor(T1 *_s, Eigen::Matrix3d &IM,
           int64_t timestamp,
           double Ts, int _id,
           int _size, std::string _StartCode)
        : Componet<T1, T2>(_s,
                           timestamp,
                           _id,
                           _size, _StartCode)
    {
        InstallMatrix = IM;
        SamplePeriod = Ts;
    }
    virtual ~Sensor() {}
    const Eigen::Matrix3d& GetInstallMatrix() const { return InstallMatrix; }
};
