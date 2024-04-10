/*
 * @Author: Amadeus
 * @Date: 2024-04-07 17:36:12
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 19:30:30
 * @FilePath: /Satellite/src/General/Subscriber.hh
 * @Description: 
 */

#pragma once
#include <nlohmann/json.hpp>
#include<iostream>
#include<vector>
class Publisher;
class ISubscriber
{
public:
    virtual ~ISubscriber() {}
    virtual void Submit() = 0;

public:
    Publisher *m_DM;
};