#include "dataLogger.h"

#include <time.h>
#include <stdio.h>
#include <fstream>

using namespace std;

//https://learnopencv.com/getting-started-opencv-cuda-module/

void printL(const char* sc)
{
    printf("%s\n", sc);
}

DataLogger::DataLogger()
: bWriteImg(false)
, bWriteMask(false)
{
}

DataLogger::~DataLogger()
{
}

void DataLogger::run(void)
{
    printf("DataLogger Thread\n");

    Signal* pSigImage = new Signal(sigNewData); // Deleted in destructor
    time_t timeNow;
    struct tm* pTimeStruct;

    fstream fileLog;
    vector<string> listStringsT;
    char scFileNameText[128];
    timeNow = time(nullptr);
    pTimeStruct = localtime(&timeNow);
    snprintf(scFileNameText, 
            127, 
            "log_%d_%d_%d_%d_%d.txt", 
            pTimeStruct->tm_year + 1900, 
            pTimeStruct->tm_mon + 1,
            pTimeStruct->tm_mday,
            pTimeStruct->tm_hour,
            pTimeStruct->tm_min);

    cv::Mat imageWrite;
    bool bWriteImgLocal = false;

    while (!ss.shouldStop()) {
        // Wait on signal
        pSigImage->wait();

        // check text list, if not empty write
        mt.lock();
        listStringsT.assign(listStrings.begin(), listStrings.end());
        mt.unlock();
        if (listStringsT.size() > 0) {
            fileLog.open(scFileNameText, ios::out);
            if (!fileLog) {
                printf("Failed to open log files\n");
            } else {
                do {
                    string str = listStringsT.back();
                    fileLog << str << endl;
                    listStringsT.pop_back();
                } while (listStringsT.size() > 0);
                fileLog.close();
            }
        }

        // Check if there's an image we should write
        mt.lock();
        if (bWriteImg) {
            bWriteImg = false;
            bWriteImgLocal = true;
            imageWrite = image;
        }
        mt.unlock();
        if (bWriteImgLocal) {
            bWriteImgLocal = false;
            timeNow = time(nullptr);
            pTimeStruct = localtime(&timeNow);
            char scFileName[128];
            snprintf(scFileName, 
                    127, 
                    "images/img_%d_%d_%d_%d_%d_%d.jpg", 
                    pTimeStruct->tm_year + 1900, 
                    pTimeStruct->tm_mon + 1,
                    pTimeStruct->tm_mday,
                    pTimeStruct->tm_hour,
                    pTimeStruct->tm_min,
                    pTimeStruct->tm_sec);
            cv::imwrite(scFileName, imageWrite);
        }

        // Check if there's a mask we should write
        mt.lock();
        if (bWriteMask) {
            bWriteMask = false;
            bWriteImgLocal = true;
            imageWrite = mask;
        }
        mt.unlock();
        if (bWriteImgLocal) {
            bWriteImgLocal = false;
            timeNow = time(nullptr);
            pTimeStruct = localtime(&timeNow);
            char scFileName[128];
            snprintf(scFileName, 
                    127, 
                    "masks/mask_%d_%d_%d_%d_%d_%d.jpg", 
                    pTimeStruct->tm_year + 1900, 
                    pTimeStruct->tm_mon + 1,
                    pTimeStruct->tm_mday,
                    pTimeStruct->tm_hour,
                    pTimeStruct->tm_min,
                    pTimeStruct->tm_sec);
            cv::imwrite(scFileName, imageWrite);
        }
    }

    delete pSigImage;
}

void DataLogger::LogImage(cv::Mat& im)
{
    mt.lock();
    image = im.clone();
    bWriteImg = true;
    mt.unlock();
    sigNewData.signal();
}

void DataLogger::LogMask(float4* pImg, int width, int height)
{
    cv::Mat tempMat(height, width, CV_32FC4, (void*)pImg);
    mt.lock();
    mask = tempMat.clone();
    bWriteMask = true;
    mt.unlock();
    sigNewData.signal();
}

void DataLogger::LogString(const char* scData)
{
    char scText[256];
    time_t timeNow = time(nullptr);
    struct tm* pTimeStruct;
    pTimeStruct = localtime(&timeNow);
    snprintf(scText,
            255,
            "%2d:%2d:%2d %s",
            pTimeStruct->tm_hour,
            pTimeStruct->tm_min,
            pTimeStruct->tm_sec,
            scData);

    mt.lock();
    listStrings.insert(listStrings.begin(), scData);
    mt.unlock();
    sigNewData.signal();
}
