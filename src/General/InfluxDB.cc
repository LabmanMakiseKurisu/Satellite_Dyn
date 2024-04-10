/*
 * @Author: Amadeus
 * @Date: 2024-04-07 18:01:03
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 19:24:20
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
    std::cout << "json:" << std::endl;
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
        std::cout << "Data sent to InfluxDB successfully." << std::endl;
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