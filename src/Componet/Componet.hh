/*
 * @Author: Amadeus
 * @Date: 2024-04-08 11:19:26
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 18:49:46
 * @FilePath: /Satellite/src/Componet/Componet.hh
 * @Description: 
 */
#pragma once
#include<iostream>
#include"Subscriber.hh"
#include"InfluxDB.hh"

template <typename T1 ,typename T2>
class Componet : public ::ISubscriber
{
protected:
    T1 *source;//数据源
    T2 Data; // ֵ数据
    int64_t LastRenewTime;//上次更新时间
    int id;//设备ID
    int Fields;//字段数
    std::string StartCode;//起始码

public:
    Componet(T1 *_s, int64_t timestamp, int _id, int _size, std::string _StartCode)
    {
        source = _s;
        LastRenewTime = timestamp;
        id = _id;
        Fields = _size;
        StartCode = _StartCode;
        m_DM = DataManager::GetInstance();
    }
    virtual ~Componet() {}
    virtual void Init(int64_t timestamp) = 0;
    virtual void StateRenew(int64_t NowTime) = 0;
    const T2 &GetData() const { return Data; }
};
