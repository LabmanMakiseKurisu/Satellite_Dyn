#!/bin/bash
###
 # @Author: Amadeus
 # @Date: 2024-04-12 18:01:03
 # @LastEditors: Amadeus
 # @LastEditTime: 2024-04-12 19:17:17
 # @FilePath: /Satellite/run.sh
 # @Description: 
### 

# 获取当前目录路径
PROJECT_DIR=$(pwd)

# 启动InfluxDB容器
docker run -d --rm --net=host --name influxdb_container influxdb:1.8.1

# 启动Grafana容器
docker run -d --rm --net=host --name grafana_container -v $PROJECT_DIR/Config/grafana.json:/etc/grafana/provisioning/dashboards/grafana.json grafana/grafana

# 启动Swagger UI容器
docker run -d --rm --net=host --name swagger_container -e SWAGGER_JSON=/foo/API.json -v $PROJECT_DIR/Config/API.json:/foo/API.json swaggerapi/swagger-ui

# 启动仿真服务容器
docker run -d --rm --net=host --name sate_container sate_sim

echo "All containers are up and running."