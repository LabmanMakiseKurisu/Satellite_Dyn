/*
 * @Author: Amadeus
 * @Date: 2024-04-06 14:00:28
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-06 14:01:22
 * @FilePath: /Satellite/server_test.cc
 * @Description: 
 */
#include "httplib.h"
#include "json.hpp"

using json = nlohmann::json;

// 假设StarSensor已经定义
StarSensor sensor;

int main() {
    using namespace httplib;

    Server svr;

    svr.Get("/data", [&](const Request&, Response& res) {
        json response = {
            {"Data", {sensor.Data.q[0], sensor.Data.q[1], sensor.Data.q[2], sensor.Data.q[3]}}
        };
        res.set_content(response.dump(), "application/json");
    });

    svr.Post("/sampleperiod", [&](const Request& req, Response& res) {
        try {
            auto j = json::parse(req.body);
            sensor.SamplePeriod = j.at("SamplePeriod").get<double>();
            json response = {{"message", "SamplePeriod updated"}};
            res.set_content(response.dump(), "application/json");
        } catch (json::exception& e) {
            json error = {{"error", "Invalid request format."}};
            res.status = 400;
            res.set_content(error.dump(), "application/json");
        }
    });

    std::cout << "Server started at port 8080\n";
    svr.listen("localhost", 8080);
}
