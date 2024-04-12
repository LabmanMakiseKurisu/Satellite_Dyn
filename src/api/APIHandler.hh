/*
 * @Author: Amadeus
 * @Date: 2024-04-12 11:11:18
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 15:44:03
 * @FilePath: /Satellite/src/api/APIHandler.hh
 * @Description: 
 */

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
#include <nlohmann/json.hpp>
#include "httplib.h"
#include "Mediator.hh"

using json = nlohmann::json;

class APIHandler
{
public:
    // 处理OPTIONS请求
    static void handleOptions(const httplib::Request &req, httplib::Response &res);

    // 处理GET请求
    static void handleGet(const httplib::Request &req, httplib::Response &res);

    // 处理PUT请求
    static void handlePut(const httplib::Request &req, httplib::Response &res);
private:
    static void SetHeader(httplib::Response &res);
};
