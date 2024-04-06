/*
 * @Author: Amadeus
 * @Date: 2024-04-06 14:00:38
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-06 14:00:58
 * @FilePath: /Satellite/client_test.cc
 * @Description: 
 */
#include "httplib.h"
#include "json.hpp"

using json = nlohmann::json;

int main() {
    using namespace httplib;

    Client cli("http://localhost:8080");

    // 发送POST请求设置SamplePeriod
    json postBody = {{"SamplePeriod", 0.5}};
    auto res = cli.Post("/sampleperiod", postBody.dump(), "application/json");
    if (res && res->status == 200) {
        std::cout << "SamplePeriod update response: " << res->body << std::endl;
    } else {
        std::cout << "Failed to update SamplePeriod." << std::endl;
    }

    // 发送GET请求获取Data
    res = cli.Get("/data");
    if (res && res->status == 200) {
        json responseBody = json::parse(res->body);
        std::cout << "StarSensor Data: " << responseBody.dump(4) << std::endl;
    } else {
        std::cout << "Failed to get StarSensor Data." << std::endl;
    }

    return 0;
}
