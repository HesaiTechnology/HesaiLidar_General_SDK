#ifdef __cplusplus
extern "C" {
#endif
void RunPcapPandarGeneralSDK(char* correctionFile, char* pcapFile, char* lidarType, char* timestampType, int runTime);  
void RunLidarPandarGeneralSDK(char* deviceipaddr, int lidarport, int gpsport, char* correctionfile, char* lidarType, char* timestampType,
                                char* multiCastIp, int runTime);
void SetPcdFileWriteFlag(int flag ,int frameNum, char* fileName);
void SetPrintFlag(int flag);

#ifdef __cplusplus
};
#endif