#include "dataLogger.h"

#include <time.h>
#include <stdio.h>
#include <fstream>
#include "planner.h"

using namespace std;
const int SEC_BETWEEN_IMAGES = 3;

//https://learnopencv.com/getting-started-opencv-cuda-module/

void printL(const char* sc)
{
    printf("%s\n", sc);
}

DataLogger::DataLogger()
: bWriteImg(false)
, bWriteImgIr(false)
, bWriteMask(false)
, pThread(nullptr)
{
}

DataLogger::~DataLogger()
{
    if (pThread != nullptr) {
        if (pThread->joinable()) pThread->join();
        delete pThread;
        pThread = nullptr;
    }
}

void DataLogger::Start()
{
    pThread = new std::thread(&DataLogger::run, this, sigNewData, sigStopThread);
}

void DataLogger::Stop()
{
    if (pThread != nullptr) {
        sigStopThread.signal();
        if (pThread->joinable()) pThread->join();
    }
}


void DataLogger::run(Signal sigImage, Signal sigStop)
{
    printf("DataLogger Thread\n");

    time_t timeNow;
    struct tm* pTimeStruct;

    fstream fileLog;
    vector<char*> listStringsT;
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
    ImageLogData dataWrite;

    while (!sigStop.isSignaled()) {
        // Wait on signal
        sigImage.waitFor(1000);

        // check text list, if not empty write
        mt.lock();
        listStringsT.assign(listStrings.begin(), listStrings.end());
        listStrings.clear();
        mt.unlock();
        if (listStringsT.size() > 0) {
            fileLog.open(scFileNameText, ios::out | ios::app);
            if (!fileLog) {
                printf("Failed to open log files\n");
            } else {
                do {
                    char* sc = listStringsT.back();
                    fileLog << sc << endl;
                    delete[] sc;
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
            dataWrite = dataImage;
        }
        mt.unlock();
        if (bWriteImgLocal) {
            bWriteImgLocal = false;
            SaveImage("img", imageWrite, dataWrite);
        }

        // Check if there's a mask we should write
        mt.lock();
        if (bWriteMask) {
            bWriteMask = false;
            bWriteImgLocal = true;
            imageWrite = mask;
            dataWrite = dataMask;
        }
        mt.unlock();
        if (bWriteImgLocal) {
            bWriteImgLocal = false;
            timeNow = time(nullptr);
            pTimeStruct = localtime(&timeNow);
            char scFileName[128];
            snprintf(scFileName, 
                    127, 
                    "masks/mask[%d]_%d_%d_%d_%d_%d_%d.jpg", 
                    dataWrite.id,
                    pTimeStruct->tm_year + 1900, 
                    pTimeStruct->tm_mon + 1,
                    pTimeStruct->tm_mday,
                    pTimeStruct->tm_hour,
                    pTimeStruct->tm_min,
                    pTimeStruct->tm_sec);
            cv::imwrite(scFileName, imageWrite);
        }

        // Check if there's an IR image we should write
        mt.lock();
        if (bWriteImgIr) {
            bWriteImgIr = false;
            bWriteImgLocal = true;
            imageWrite = imageIr;
            dataWrite = dataImageIr;
        }
        mt.unlock();
        if (bWriteImgLocal) {
            bWriteImgLocal = false;
            SaveImage("flir", imageWrite, dataWrite);
        }
    }
}

void DataLogger::LogImage(CameraFrame& frm, std::string strIdSource)
{
    ImageLogData& data = mapImageData[strIdSource];
    time_t timeNow = time(nullptr);
    time_t timeDifference = timeNow - data.timeLast;
    data.timeLast = timeNow;
    if (timeDifference < SEC_BETWEEN_IMAGES) return;
    data.id = frm.GetId();
    if (!FlightController::GetPosition(data.fLat, data.fLon)) {
        data.fLat = 0;
        data.fLon = 0;
    }
    if (!FlightController::GetHeading(data.fHeading)) {
        data.fHeading = 999; // Obvious incorrect value
    }
    if (!FlightController::GetAltitude(data.fAltitude)) {
        data.fAltitude = -999;
    }
    
    mt.lock();
    image = frm.matrix.clone();
    dataImage = data;
    bWriteImg = true;
    mt.unlock();
    sigNewData.signal();
}

void DataLogger::LogImageIr(CameraFrame& frm)
{
    ImageLogData& data = mapImageData["IR"];
    time_t timeNow = time(nullptr);
    time_t timeDifference = timeNow - data.timeLast;
    data.timeLast = timeNow;
    if (timeDifference < SEC_BETWEEN_IMAGES) return;
    data.id = frm.GetId();
    if (!FlightController::GetPosition(data.fLat, data.fLon)) {
        data.fLat = 0;
        data.fLon = 0;
    }
    if (!FlightController::GetHeading(data.fHeading)) {
        data.fHeading = 999; // Obvious incorrect value
    }
    if (!FlightController::GetAltitude(data.fAltitude)) {
        data.fAltitude = -999;
    }
    
    mt.lock();
    imageIr = frm.matrix.clone();
    dataImageIr = data;
    bWriteImgIr = true;
    mt.unlock();
    sigNewData.signal();
}

void DataLogger::LogMask(float4* pImg, int iImageId, int width, int height)
{
    cv::Mat tempMat(height, width, CV_32FC4, (void*)pImg);
    mt.lock();
    dataMask.id = iImageId;
    mask = tempMat.clone();
    bWriteMask = true;
    mt.unlock();
    sigNewData.signal();
}

void DataLogger::LogString(const char* scData)
{
    char* scText = new char[strlen(scData) + 32];
    time_t timeNow = time(nullptr);
    struct tm* pTimeStruct;
    pTimeStruct = localtime(&timeNow);
    snprintf(scText,
            strlen(scData) + 31,
            "%02d:%02d:%02d %s",
            pTimeStruct->tm_hour,
            pTimeStruct->tm_min,
            pTimeStruct->tm_sec,
            scData);

    mt.lock();
    listStrings.insert(listStrings.begin(), scText);
    mt.unlock();
    sigNewData.signal();
}

void DataLogger::WriteImageData(cv::Mat& image, ImageLogData& imageData)
{
    cv::Mat imageLarger = cv::Mat::zeros(cv::Size(image.cols, image.rows + 55), CV_8UC3);
    memcpy(imageLarger.data, image.data, image.cols * image.rows * 3);
    // Now write stuff at the bottom, 2 rows, 5 pixel padding
    double dFontSize = cv::getFontScaleFromHeight(cv::FONT_HERSHEY_SIMPLEX, 20, 1);
    char sc[512];
    snprintf(sc, 511, "Image:%d Lat:%.7f Lon:%.7f Alt:%.1fm Head:%.2f",
                imageData.id, imageData.fLat, imageData.fLon, imageData.fAltitude, imageData.fHeading);
    cv::putText(imageLarger, 
                sc,
                cv::Point(5, image.rows + 25),
                cv::FONT_HERSHEY_SIMPLEX,
                dFontSize,
                cv::Scalar(255, 255, 255));
    image = imageLarger;
}

void DataLogger::SaveImage(std::string strPrefix, cv::Mat& image, ImageLogData& imageData)
{
    WriteImageData(image, imageData);
    time_t timeNow = time(nullptr);
    struct tm* pTimeStruct = localtime(&timeNow);
    char scFileName[128];
    snprintf(scFileName, 
            127, 
            "images/%s_%d_%d_%d_%d_%d_%d.jpg",
            strPrefix.c_str(),
            pTimeStruct->tm_year + 1900, 
            pTimeStruct->tm_mon + 1,
            pTimeStruct->tm_mday,
            pTimeStruct->tm_hour,
            pTimeStruct->tm_min,
            pTimeStruct->tm_sec);
    cv::imwrite(scFileName, image);
}
