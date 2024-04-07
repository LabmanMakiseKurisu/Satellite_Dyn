/*
 * @Author: Amadeus
 * @Date: 2024-04-07 17:53:45
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 19:36:21
 * @FilePath: /Satellite/src/General/InfluxDB.hh
 * @Description: 
 */
#pragma once
#include <nlohmann/json.hpp>
#include<iostream>
#include<vector>
#include"Subscriber.hh"

class DataManager
{
public:
    static DataManager *GetInstance();

    void Subscribe(ISubscriber *subscriber);

    void write();
    
    template<typename T>
    void add(std::string key, T value) {
        data[key] = value;
    }
private:
    static inline DataManager *m_instance{NULL};
    std::vector<ISubscriber *> subscribers;
    nlohmann::json data;
    std::string m_host;
    int m_port;
    std::string m_dbName;
    std::string m_Measurement;
    int64_t m_LastSendTime;
    int m_interval;
private:
    DataManager();
    DataManager(const DataManager &) = delete;
    DataManager &operator=(const DataManager &) = delete;
    DataManager(DataManager &&) = delete;
    DataManager &operator=(DataManager &&) = delete;
    void DataManager::SendToInfluxDB();
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
