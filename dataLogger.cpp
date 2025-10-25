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

DataLoggerWriter::DataLoggerWriter(LogSender_String* pStrings, LogSender_Image* pImages, LogSender_Image* pIr)
: strings(pStrings)
, images(pImages)
, ir(pIr)
{
}

DataLoggerWriter::DataLoggerWriter(DataLoggerWriter& other)
: strings(other.strings->CreateSender())
, images(other.images->CreateSender())
, ir(other.ir->CreateSender())
{
}

void DataLoggerWriter::LogImage(const CameraFrame& frm, std::string strData)
{
    CameraFrame* pNew = new CameraFrame();
    pNew->DeepCopy(frm);
    pNew->SetString(pNew->GetString() + strData);

    if (!images->AddData(pNew, 0)) {//todo timeouts?
        delete pNew;
    }
}

void DataLoggerWriter::LogImageIr(const CameraFrame& frm)
{
    CameraFrame* pNew = new CameraFrame();
    pNew->DeepCopy(frm);

    if (!ir->AddData(pNew, 0)) {//todo timeouts?
        delete pNew;
    }
}

void DataLoggerWriter::LogString(const char* scData)
{
    std::string* pNewLogEntry = new std::string();

    // No, I don't care that this is silly...c++ string formatting sucks
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
    
    *pNewLogEntry = scText;
    delete[] scText;

    if (!strings->AddData(pNewLogEntry, 0)) {
        delete pNewLogEntry;
    }
}

DataLogger::DataLogger()
: bWriteMask(false)
, pThread(nullptr)
, pipeImage(9, true)
, pipeIR(3, true)
, pipeStrings(9, true)
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
    pThread = new std::thread(&DataLogger::run, this, sigStopThread);
}

void DataLogger::Stop()
{
    if (pThread != nullptr) {
        sigStopThread.signal();
        if (pThread->joinable()) pThread->join();
    }
}

DataLoggerWriter* DataLogger::GetLogger()
{
    return new DataLoggerWriter(pipeStrings.CreateSender(), pipeImage.CreateSender(), pipeIR.CreateSender());
}

void DataLogger::run(Signal sigStop)
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
        //sigImage.wait(1000);

        // Text
        //no timeout
        std::shared_ptr<std::string> pNewString;
        if (pipeStrings.DataReady()) {
            fileLog.open(scFileNameText, ios::out | ios::app);
            if (!fileLog) {
                printf("Failed to open log files\n");
            } else {
                while (pipeStrings.RetrieveData(pNewString, 500)) {
                    fileLog << *pNewString << endl;
                }
                fileLog.close();
            }
        }

        // Image
        //timeout or not to timeout
        std::shared_ptr<CameraFrame> pNewImage;
        while (pipeImage.RetrieveData(pNewImage, 500)) {
            SaveImage("img", *pNewImage);
        }

        //IR
        while (pipeIR.RetrieveData(pNewImage, 500)) {
            SaveImage("flir", *pNewImage);
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
                    0,//todo (is protected) dataWrite.id,
                    pTimeStruct->tm_year + 1900, 
                    pTimeStruct->tm_mon + 1,
                    pTimeStruct->tm_mday,
                    pTimeStruct->tm_hour,
                    pTimeStruct->tm_min,
                    pTimeStruct->tm_sec);
            cv::imwrite(scFileName, imageWrite);
        }
    }
}

void DataLogger::LogMask(float4* pImg, int iImageId, int width, int height)
{
    cv::Mat tempMat(height, width, CV_32FC4, (void*)pImg);
    mt.lock();
    //dataMask.id = iImageId;
    mask = tempMat.clone();
    bWriteMask = true;
    mt.unlock();
    // sigNewData.signal();
}

void DataLogger::WriteImageData(CameraFrame& frame)
{
    cv::Mat& image = frame.GetMatrix();
    cv::Mat imageLarger = cv::Mat::zeros(cv::Size(image.cols, image.rows + 55), CV_8UC3);
    memcpy(imageLarger.data, image.data, image.cols * image.rows * 3);
    // Now write stuff at the bottom, 2 rows, 5 pixel padding
    double dFontSize = cv::getFontScaleFromHeight(cv::FONT_HERSHEY_SIMPLEX, 20, 1);
    char sc[512];
    snprintf(sc, 511, "Image:%d Lat:%.7f Lon:%.7f Alt:%.1fm Head:%.2f",
                frame.GetId(), frame.GetLat(), frame.GetLon(), frame.GetAltitude(), frame.GetHeading());
    cv::putText(imageLarger, 
                sc,
                cv::Point(5, image.rows + 25),
                cv::FONT_HERSHEY_SIMPLEX,
                dFontSize,
                cv::Scalar(255, 255, 255));
    image = imageLarger;
}

void DataLogger::SaveImage(std::string strPrefix, CameraFrame& image)
{
    CameraFrame imageCopy;
    imageCopy.DeepCopy(image); // So we get access to cv::Mat&

    WriteImageData(imageCopy);
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
    cv::imwrite(scFileName, imageCopy.GetMatrix());
}
