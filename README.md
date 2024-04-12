<!--
 * @Author: Amadeus
 * @Date: 2024-04-06 14:42:32
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 19:17:28
 * @FilePath: /Satellite/README.md
 * @Description: 
-->
# Satellite Project

## Introduction

本项目为基于C++的卫星轨道姿态仿真系统，内含高精度动力学、执行器和敏感器的模型、数据可视化工具，以及基于RESTful API的接口。

## Features

- **姿轨动力学**
- **太阳矢量、地磁场等飞行环境**
- **常用敏感器、执行器单机模型**
- **基于InfluxDB和Grafana的数据监视**
- **基于HTTP1.1的RESTful API** 

## Development

目前待开发功能:
- 代码重构，提升可扩展性[done]
- RESTful风格的HTTP API[done]
- 支持容器化部署
- 高精度轨道动力学HPOP
- 敏感器噪声
- 磁力矩器单机模型
- 太阳敏感器单机模型
- 轨道控制

## Getting Started
基本依赖环境：
- docker
# Submodule
```
$ git submodule update --init --recursive                 
```
# Build
```
$ docker build -t sate_sim .                
```
# run
```
$ service influxdb start     
$ bash ./run.sh
```

