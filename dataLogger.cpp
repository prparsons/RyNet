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
, bWriteMask(false)
, pThread(nullptr)
{
}

DataLogger::~DataLogger()
{
    Stop();
}

void DataLogger::Start()
{
    pThread = new std::thread(&DataLogger::run, this);
}

void DataLogger::Stop()
{
    if (pThread != nullptr) {
        ss.stop();
        pThread->join();
        delete pThread;
        pThread = nullptr;
    }
}


void DataLogger::run(void)
{
    printf("DataLogger Thread\n");

    Signal* pSigImage = new Signal(sigNewData); // Deleted in destructor
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

    while (!ss.shouldStop()) {
        // Wait on signal
        pSigImage->wait();

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
            WriteImageData(imageWrite, dataImage);
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

void DataLogger::LogImage(cv::Mat& im, std::string strIdSource)
{
    ImageLogData& data = mapImageData[strIdSource];
    time_t timeNow = time(nullptr);
    time_t timeDifference = timeNow - data.timeLast;
    data.timeLast = timeNow;
    if (timeDifference < SEC_BETWEEN_IMAGES) return;
    
    if (!FlightController::GetPosition(data.fLat, data.fLon)) {
        data.fLat = 0;
        data.fLon = 0;
    }
    if (!FlightController::GetHeading(data.fHeading)) {
        data.fHeading = 999; // Obvious incorrect value
    }
    
    mt.lock();
    image = im.clone();
    dataImage = data;
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
    snprintf(sc, 511, "Lat:%.7f Lon:%.7f Alt:%.1fm Head:%.2f Pit:%.2f",
                imageData.fLat, imageData.fLon, 3.0, imageData.fHeading, imageData.fPitch);
    cv::putText(imageLarger, 
                sc,
                cv::Point(5, image.rows + 25),
                cv::FONT_HERSHEY_SIMPLEX,
                dFontSize,
                cv::Scalar(255, 255, 255));
    image = imageLarger;
}
