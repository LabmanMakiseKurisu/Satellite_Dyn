/*
 * @Author: Amadeus
 * @Date: 2024-04-07 18:01:03
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 18:54:39
 * @FilePath: /Satellite/src/General/InfluxDB.cc
 * @Description: 
 */
#include"InfluxDB.hh"
#include"GlobalSetting.hh"
#include "httplib.h"
#include"SimTime.hh"
static Publisher* m = Publisher::GetInstance();
Publisher::DeleteHelper Publisher::helper;

Publisher *Publisher::GetInstance()
{
    if (m_instance == NULL)
        m_instance = new Publisher;
    return m_instance;
}

void Publisher::ReleaseInstance()
{
    if (Publisher::m_instance != nullptr)
    {
        delete Publisher::m_instance;
        Publisher::m_instance = nullptr;
    }
}

Publisher::Publisher() {
    GlobalSettings* pCfg = GlobalSettings::GetInstance();
    m_host = pCfg->Get<std::string>("/InfluxDB/Host");
    m_port = pCfg->Get<int>("/InfluxDB/Port");
    m_dbName = pCfg->Get<std::string>("/InfluxDB/DBName");
    m_Measurement = pCfg->Get<std::string>("/InfluxDB/Measurement");
    m_interval = pCfg->Get<int>("/InfluxDB/Interval");
    m_LastSendTime = GetTimeStampMs();

    CreateDB();
}

void Publisher::Subscribe(ISubscriber *subscriber)
{
    subscribers.push_back(subscriber);
}

void Publisher::write()
{
    if(GetTimeStampMs() - m_LastSendTime < m_interval*1000)
        return;
    m_LastSendTime = GetTimeStampMs();
    for (const auto &subscriber : subscribers)
    {
        subscriber->Submit();
    }
    SendToInfluxDB();
}

void Publisher::SendToInfluxDB()
{
    //std::cout << "json:" << std::endl;
    // 创建一个客户端实例，连接到 InfluxDB
    httplib::Client cli(m_host.c_str(), m_port);

    // 构造 InfluxDB Line Protocol 数据
    std::string payload = m_Measurement + " ";
    for (const auto &item : data.items())
    {
        payload += item.key() + "=" + item.value().dump() + ",";
    }
    // 删除最后一个逗号
    if (!payload.empty())
    {
        payload.pop_back();
    }

    // 发送 POST 请求到 InfluxDB
    auto res = cli.Post(("/write?db=" + m_dbName).c_str(), payload, "text/plain");

    if (res && res->status == 204)
    { // InfluxDB通常使用状态码204表示成功接收数据
        //std::cout << "Data sent to InfluxDB successfully." << std::endl;
    }
    else
    {
        std::cerr << "Failed to send data to InfluxDB." << std::endl;
        if (res)
        {
            std::cerr << "Status code: " << res->status << std::endl;
        }
    }
}

std::string GetCode(std::string StartCode, int index) {
    std::string Code = StartCode;
    if (index < 10)
        Code.append("0");
    Code.append(std::to_string(index));
    return Code;
}

void Publisher ::CreateDB()
{
    httplib::Client cli(m_host, m_port);

    // 查询数据库列表
    auto res = cli.Get("/query?q=SHOW%20DATABASES");
    if (res && res->status == 200)
    {
        // 检查数据库是否存在
        if (res->body.find(m_dbName) == std::string::npos)
        {
            // 数据库不存在，创建数据库
            std::string query = "q=CREATE DATABASE " + m_dbName;
            auto create_res = cli.Post("/query", query, "application/x-www-form-urlencoded");
            if (create_res && create_res->status == 200)
            {
                std::cout << "Database '" << m_dbName << "' created successfully." << std::endl;
            }
            else
            {
                std::cerr << "Failed to create database '" << m_dbName << "'. Status code: " << (create_res ? std::to_string(create_res->status) : "No response") << std::endl;
            }
        }
        else
        {
            std::cout << "Database '" << m_dbName << "' already exists." << std::endl;
        }
    }
    else
    {
        std::cerr << "Failed to query databases. Status code: " << (res ? std::to_string(res->status) : "No response") << std::endl;
    }
}