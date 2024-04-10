/*
 * @Author: Amadeus
 * @Date: 2024-04-07 17:53:45
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 18:56:08
 * @FilePath: /Satellite/src/General/InfluxDB.hh
 * @Description: 
 */
#pragma once
#include <nlohmann/json.hpp>
#include<iostream>
#include<vector>
#include"Subscriber.hh"

class Publisher
{
public:
    static Publisher *GetInstance();

    void Subscribe(ISubscriber *subscriber);

    void write();
    
    template<typename T>
    void add(std::string key, T value) {
        data[key] = value;
    }
private:
    static inline Publisher *m_instance{NULL};
    std::vector<ISubscriber *> subscribers;
    nlohmann::json data;
    std::string m_host;
    int m_port;
    std::string m_dbName;
    std::string m_Measurement;
    int64_t m_LastSendTime;
    int m_interval;
private:
    Publisher();
    Publisher(const Publisher &) = delete;
    Publisher &operator=(const Publisher &) = delete;
    Publisher(Publisher &&) = delete;
    Publisher &operator=(Publisher &&) = delete;
    void SendToInfluxDB();
    static void ReleaseInstance();
    class DeleteHelper
    {
    public:
        DeleteHelper() = default;
        ~DeleteHelper()
        {
            ReleaseInstance();
        }
    };
    static DeleteHelper helper;
};

std::string GetCode(std::string StartCode, int index);