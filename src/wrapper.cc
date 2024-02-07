#include "wrapper.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#ifdef __cplusplus
extern "C" {
#endif

#define PRINT_FLAG
int printFlag = 1;         // 1:print, other number: don't print
int pcdFileWriteFlag = 0;  // 1:write pcd, other number: don't write pcd
bool running = true;
int frameItem = 0;
int saveFrameIndex = 10;
std::string saveFileName = "./cloudpoints.csv";

void gpsCallback(double timestamp) {
  if (printFlag == 1)
    printf("gps: %lf\n", timestamp);
}

void lidarCallback(boost::shared_ptr<SVPointCloud> cld, double timestamp) {
  if (printFlag == 1)
    printf("timestamp: %lf,point_size: %ld\n", timestamp, cld->points.size());
  if (pcdFileWriteFlag == 1) {
    frameItem++;
    if (saveFrameIndex == frameItem) {
      int Num = cld->points.size();
      std::ofstream zos(saveFileName);
      for (int i = 0; i < Num; i++) {
        zos << cld->points[i].x << "," << cld->points[i].y << "," << cld->points[i].z << "," << cld->points[i].intensity << "," << cld->points[i].timestamp << "," << cld->points[i].ring << std::endl;
      }
    }
  }
}

void lidarAlgorithmCallback(HS_Object3D_Object_List* object_t) {
  HS_Object3D_Object* object;
  if (printFlag == 1) {
    printf("----------------------\n");
    printf("total objects: %d\n", object_t->valid_size);
    for (size_t i = 0; i < object_t->valid_size; i++) {
      object = &object_t->data[i];
      printf("id: %u, type: %u\n", object->data.id, object->type);
    }
    printf("----------------------\n");
  }
}

void RunPcapPandarGeneralSDK(char* correctionFile, char* pcapFile, char* lidarType, char* timestampType, int runTime) {
  PandarGeneralSDK pandarGeneral(pcapFile,
                                 lidarCallback, 0, 0, 1, lidarType, "", timestampType, true);
  std::string filePath = correctionFile;
  std::ifstream fin(filePath);
  if (fin.is_open()) {
    std::cout << "Open correction file " << filePath << " succeed" << std::endl;
    int length = 0;
    std::string strlidarCalibration;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char* buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    strlidarCalibration = buffer;
    int ret = pandarGeneral.LoadLidarCorrectionFile(strlidarCalibration);
    if (ret != 0) {
      std::cout << "Load correction file from " << filePath << " failed" << std::endl;
    } else {
      std::cout << "Load correction file from " << filePath << " succeed" << std::endl;
    }
  }
  pandarGeneral.Start();
  sleep(runTime);
  return;
}

void RunLidarPandarGeneralSDK(char* deviceipaddr, int lidarport, int gpsport, char* correctionfile, char* lidarType, char* timestampType,
                              char* multiCastIp, int runTime) {
  PandarGeneralSDK pandarGeneral(deviceipaddr, lidarport, 0, gpsport,
                                 lidarCallback, lidarAlgorithmCallback, gpsCallback, 0, 0, 1, lidarType, std::string("frame_id"), timestampType, correctionfile, multiCastIp, false);
  pandarGeneral.Start();
  sleep(runTime);
  return;
}

void SetPcdFileWriteFlag(int flag, int frameNum, char* fileName) {
  saveFrameIndex = frameNum;
  pcdFileWriteFlag = flag;
  saveFileName = fileName;
  printf("Save one frame points to : \"%s\"\n", fileName);
  return;
}

void SetPrintFlag(int flag) {
  printFlag = flag;
  return;
}
#ifdef __cplusplus
};
#endif