/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
// #define PRINT_FLAG
// #define PCD_FILE_WRITE_FLAG

int frameItem = 0;

void gpsCallback(int timestamp) {
#ifdef PRINT_FLAG
  printf("gps: %d\n", timestamp);
#endif
}

void lidarCallback(boost::shared_ptr<SVPointCloud> cld, double timestamp) {
#ifdef PRINT_FLAG
  printf("timestamp: %lf,point_size: %ld\n", timestamp, cld->points.size());
#endif
#ifdef PCD_FILE_WRITE_FLAG
  frameItem++;
  pcl::PCDWriter writer;
  std::string fileName = "PointCloudFrame" + std::to_string(frameItem) + ".pcd";
  writer.write(fileName, *cld);
  printf("save frame %d\n", frameItem);
#endif
}
void lidarAlgorithmCallback(HS_Object3D_Object_List* object_t) {
  HS_Object3D_Object* object;
#ifdef PRINT_FLAG
  printf("----------------------\n");
  printf("total objects: %d\n", object_t->valid_size);
  for (size_t i = 0; i < object_t->valid_size; i++) {
    object = &object_t->data[i];
    printf("id: %u, type: %u\n", object->data.id, object->type);
  }
  printf("----------------------\n");
#endif
}

int main(int argc, char** argv) {
  PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 0, 10110,
                                 lidarCallback, lidarAlgorithmCallback, gpsCallback, 0, 0, 0, std::string("PandarXT-32"), std::string("frame_id"), "", "", "", false);

  // PandarGeneralSDK pandarGeneral(std::string("/path/to/pcapFile"), \
  // lidarCallback, 0, 0, 1, std::string("PandarXT-16"), "", "", false);
  // std::string filePath = "/path/to/correctionFile";
  // std::ifstream fin(filePath);
  // if (fin.is_open()) {
  //   std::cout << "Open correction file " << filePath << " succeed" << std::endl;
  //   int length = 0;
  //   std::string strlidarCalibration;
  //   fin.seekg(0, std::ios::end);
  //   length = fin.tellg();
  //   fin.seekg(0, std::ios::beg);
  //   char *buffer = new char[length];
  //   fin.read(buffer, length);
  //   fin.close();
  //   strlidarCalibration = buffer;
  //   int ret = pandarGeneral.LoadLidarCorrectionFile(strlidarCalibration);
  //   if (ret != 0) {
  //     std::cout << "Load correction file from " << filePath <<" failed" << std::endl;
  //   } else {
  //     std::cout << "Load correction file from " << filePath << " succeed" << std::endl;
  //   }
  // }
  // else{
  //   std::cout << "Open correction file " << filePath << " failed" << std::endl;
  // }

  pandarGeneral.Start();

  while (true) {
    sleep(100);
  }

  return 0;
}
