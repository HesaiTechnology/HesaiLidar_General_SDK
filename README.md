# HesaiLidar_General_SDK

## About the project
HesaiLidar_General_SDK project is the software development kit for:
**Pandar40P/Pandar64/Pandar20A/Pandar20B/PandarQT/Pandar40M/PandarXT**
LiDAR sensor manufactured by Hesai Technology.

Branches: 
-   master:  The software development kit for Ubuntu16.04，18.04 and 20.04

## Environment and Dependencies
**System environment requirement: Linux**

　Recommanded:  
- Ubuntu 16.04
- Ubuntu 18.04 
- Ubuntu 20.04

**Compiler version requirement**
 Cmake version requirement: Cmake 3.8.0 or above
 G++ version requirement: G++ 7.5.0 or above
 
**Library Dependencies: libpcap-dev + libyaml-cpp-dev**  
```
$sudo apt install libpcap-dev libyaml-cpp-dev
```
## Clone
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_General_SDK.git
```
## Build
```
$ cd HesaiLidar_General_SDK
$ mkdir build
$ cd build
$ cmake ..
$ make
```
## Add to your project
### Cmake
```
add_subdirectory(<path_to>HesaiLidar_General_SDK)

include_directories(
	<path_to>HesaiLidar_General_SDK/include
	<path_to>HesaiLidar_General_SDK/src/PandarGeneralRaw/include
)

target_link_libraries(<Your project>
  PandarGeneralSDK
)
```
### C++
```
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
// for Pandar40P
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar40P"));
// for Pandar64
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar64"));
// for Pandar20A
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar20A"));
// for Pandar20B
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar20B"));
// for PandarQT
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("PandarQT"));
// for Pandar40M
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar40M"));
// for PandarXT-32
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("PandarXT-32"));
// for PandarXT-16
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("PandarXT-16"));
// for PandarXTM
PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
    lidarCallback, gpsCallback, 0, 0, 1, std::string("PandarXTM"));
```
