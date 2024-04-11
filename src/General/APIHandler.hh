/*
 * @Author: Amadeus
 * @Date: 2024-04-11 18:31:10
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-11 20:14:29
 * @FilePath: /Satellite/src/General/APIHandler.hh
 * @Description: 
 */
#pragma once
#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>
#include "Subscriber.hh"
#include<unordered_map>
class Handler
{
public:
    static Handler *GetInstance();

    void add(std::string key, double *value);
    
private:
    static inline Handler *m_instance{NULL};
    std::unordered_map<std::string, double*> data;
private:
    Handler();
    Handler(const Handler &) = delete;
    Handler &operator=(const Handler &) = delete;
    Handler(Handler &&) = delete;
    Handler &operator=(Handler &&) = delete;
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

/*
方法：
GET:查询数据
PUT:修改数据

URL部分: http:ip:port/Data/SIMXXXX （指明要访问的数据id）

PUT的请求体和GET回应体以json的形式，包含4个字段
type:number，string，vector，matrix
row(可选，vector和matrix时有效)
col(可选，vector和matrix时有效)
data
*/