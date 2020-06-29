# HesaiLidar_PandarGeneral_sdk
Hesai Lidar General SDK
## Clone
```
git clone https://github.com/HesaiTechnology/HesaiLidar_PandarGeneral_sdk.git
```
## Build
```
cd <project>
mkdir build
cd build
cmake ..
make
```
## Add to your project
### Cmake
```
add_subdirectory(<path_to>HesaiLidar_PandarGeneral_sdk)

include_directories(
	<path_to>HesaiLidar_PandarGeneral_sdk/include
	<path_to>HesaiLidar_PandarGeneral_sdk/src/PandarGeneralRaw/include
)

target_link_libraries(<Your project>
  PandarGeneralSDK
)
```
### C++
```
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
```
