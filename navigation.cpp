#include "navigation.h"

#include <thread>
#include <unistd.h>
#include <jetson-utils/cudaMappedMemory.h>

// ** ObjectDetection

ObjectDetection::ObjectDetection(DataLogger& _logger)
: logger(_logger)
{
	// For Jetson Nano (just let it use gpu)
    pNetwork = detectNet::Create(detectNet::SSD_MOBILENET_V2);
    // SSD_INCEPTION_V2 ?
	// For Xavier (try to use one of the DLA engines so we can use the gpu for opencv stuff)
	//detectNet* net = detectNet::Create(detectNet::SSD_MOBILENET_V2, 0.5f, 1, TYPE_FASTEST, DEVICE_DLA_0);DEVICE_DLA_1 DEVICE_GPU
}

void ObjectDetection::SetNewImage(image& img)
{
    mt.lock();
    imgRecent = img;
    mt.unlock();
    sigNewImage.signal();
}

bool ObjectDetection::ProcessImage(image& imgRGB)
{
    // these variables will store the image data pointer and dimensions
    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)
    uchar3* u3Img;

    if (CUDA_FAILED(cudaMalloc(&u3Img, imgWidth*imgHeight*3))) {
        printL("ObjectDetection::ProcessImage cudaMalloc fail");
        return false;
    }
    cudaMemcpy(u3Img, (void*)imgRGB.data, imgWidth*imgHeight*3, cudaMemcpyHostToDevice);   // shared CPU/GPU pointer to image
    detectNet::Detection* arDetections = nullptr;
    int iDetections = pNetwork->Detect(u3Img, imgWidth, imgHeight, &arDetections);
    
    if (iDetections < 1) {
        return false;
    }

    bool bRet = false;
    char sc[128];
    for (int i = 0; i < iDetections; i++) {
        //if (arDetections[i].Confidence < 0.5) continue; // This isn't necessary since the argument is passed to detectNet
        //if (arDetections[i].ClassID == 1) continue; // Should be a person (looking for people)

        bRet = true;
        snprintf(sc, 127, "Detection Class,Confidence[%d,%d] LTRB:%d %d %d %d", 
                int(arDetections[i].ClassID), 
                int(arDetections[i].Confidence * 100),
                int(arDetections[i].Left), 
                int(arDetections[i].Top), 
                int(arDetections[i].Right), 
                int(arDetections[i].Bottom));
        logger.LogString(sc);
    }

    cudaFree(u3Img);
    return bRet;
}

void ObjectDetection::run()
{
    printL("ObjectDetection::run");
    Signal* pSigRGB = new Signal(sigNewImage);
    image imgProcess;

    while (!ss.shouldStop()) {
        pSigRGB->wait();
        mt.lock();
        imgProcess = imgRecent;
        mt.unlock();
        
        //if (ProcessImage(imgProcess)) {
            // This works, but at 2fps, this could still add up to way more than necessary.
            // Need a rate limit 
            //logger.LogImage(imgProcess);
        //}
    }
    delete pSigRGB;
}

// ** TerrainSegmentation

#define NETWORK_PRINT_LABELS

TerrainSegmentation::TerrainSegmentation(DataLogger& _logger)
: bHabitatVisible(false)
, fImageFOV(75) // I MADE UP THIS NUMBER
, logger(_logger)
, pMask(nullptr)
{
    pNetwork = segNet::Create(segNet::FCN_RESNET18_SUNRGB_640x512);//segNet::FCN_RESNET18_DEEPSCENE_576x320);FCN_ALEXNET_AERIAL_FPV_720p
    vecClassLabel.reserve(pNetwork->GetNumClasses());
    for (int i = 0; i < pNetwork->GetNumClasses(); i++) {
        vecClassLabel[i] = pNetwork->GetClassDesc(i);
        #ifdef NETWORK_PRINT_LABELS
        printf("[%d]  %s\n", i, vecClassLabel[i]);
        #endif
    }
}

TerrainSegmentation::~TerrainSegmentation()
{
    if (pMask != nullptr) delete[] pMask;// cudaFree(pMask);
}

bool TerrainSegmentation::ProcessImage(image& imgRGB)
{
    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)
    uchar3* u3Img;
    if (CUDA_FAILED(cudaMalloc(&u3Img, imgWidth*imgHeight*3))) {
        printL("TerrainSegmentation::ProcessImage cudaMalloc fail");
        return false;
    }
    cudaMemcpy(u3Img, (void*)imgRGB.data, imgWidth*imgHeight*3, cudaMemcpyHostToDevice);

    if(!pNetwork->Process(u3Img, imgWidth, imgHeight)) {
        printf("TerrainSegmentation::ProcessImage fail");
        cudaFree(u3Img);        
        return false;
    }

    if (pMask == nullptr) {
        pMask = new uint8_t[imgWidth*imgHeight];
    }
    if(!pNetwork->Mask(pMask, imgWidth, imgHeight)) {
        printL("TerrainSegmentation::ProcessImage mask fail");
        cudaFree(u3Img);
        return false;
    }
    cudaFree(u3Img);
    
    return ScanMask(pMask, imgWidth, imgHeight);
}

bool TerrainSegmentation::IsHabitatVisible()
{
    return bHabitatVisible;
}

float TerrainSegmentation::GetHabitatDirection()
{
    return 1.0;
}

void TerrainSegmentation::SetNewImage(image& img)
{
    mt.lock();
    imgRecent = img;
    mt.unlock();
    sigNewImage.signal();
}

void TerrainSegmentation::run()
{
    printL("TerrainSegmentation::run");
    Signal* pSigRGB = new Signal(sigNewImage);
    image imgProcess;

    while (!ss.shouldStop()) {
        pSigRGB->wait();
        mt.lock();
        imgProcess = imgRecent;
        mt.unlock();
        
        if (ProcessImage(imgProcess)) {
        //    logger.LogImage(imgProcess);
            char sc[128];
            sprintf(sc, "Good Habitat Found %2.2f", fBestDirection);
            logger.LogString(sc);
        }
    }
    delete pSigRGB;
}

bool TerrainSegmentation::ScanMask(uint8_t* pMask, int iWidth, int iHeight)
{
    int size = iWidth * iHeight;
    float fAngleAvgSum = 0;
    int iAngleAvgCount = 0;
    uint8_t pixel;
    for (int i = 0; i < size; i++) {
        pixel = pMask[i];
        if (pixel == HABITAT) {
            int x = i / iWidth;
            fAngleAvgSum += (abs(x - (iWidth * .5)) / (iWidth * .5)) * (fImageFOV * .5);
            iAngleAvgCount++;
        }
    }

    if (iAngleAvgCount == 0) {
        fBestDirection = -1;
        return false;
    }
    // We've found the desired category
    int maskWidth = 640;
    int maskHeight = 360;
    float4* imgMask;
    if (!CUDA_FAILED(cudaHostAlloc(&imgMask, maskWidth*maskHeight*16, cudaHostAllocMapped))) {
        pNetwork->Mask(imgMask, maskWidth, maskHeight, segNet::FILTER_POINT);
        logger.LogMask(imgMask, maskWidth, maskHeight);
        cudaFreeHost(imgMask);
    }
    fBestDirection = fAngleAvgSum / iAngleAvgCount;
    return true;
}

Navigator::Navigator(FlightController& _control, DataLogger& _logger, CameraController& _camera)
: control(_control)
, logger(_logger)
, camera(_camera)
, pThreadNav(nullptr)
, pThreadDet(nullptr)
, pThreadSeg(nullptr)
, detector(logger)
, segmenter(logger)
{

}

void Navigator::runNavigator()
{
    printf("Navigator::runNavigator");
    cameraFrame* pFrame = nullptr;
    Signal* pSigRGB = camera.GetSignalRGB();

    bool bGridSearching = true;
    float fMargin = 0;

    std::thread thrObject(&ObjectDetection::run, &detector);
    std::thread thrSegment(&TerrainSegmentation::run, &segmenter);

    while (!ssMain.shouldStop()) {
        pSigRGB->wait();
        if (pFrame) delete pFrame;
        pFrame = camera.GetFrameRGB();
        
        image imgNew = pFrame->getImage();
        detector.SetNewImage(imgNew);
        segmenter.SetNewImage(imgNew);

        if (bGridSearching) {
            if (segmenter.IsHabitatVisible()) { // good terrain visible?
                control.SetDirection(segmenter.GetHabitatDirection());
                char sc[64];
                sprintf(sc, "Grid Search -> Good Habitat %1.3f", segmenter.GetHabitatDirection());
                logger.LogString(sc);
            }
        } else {
	    if (segmenter.IsHabitatVisible()) {
                if (abs(segmenter.GetHabitatDirection()) < fMargin) continue; // good terrain in current direction?
                control.SetDirection(segmenter.GetHabitatDirection());
                char sc[64];
                sprintf(sc, "Grid Search Change -> %1.3f", segmenter.GetHabitatDirection());
                logger.LogString(sc);
            } else {
                control.RestoreRouteNaive();
                logger.LogString("Good Habitat -> Grid Search");
            }
        }
    }
    if (pFrame != nullptr) delete pFrame;
    delete pSigRGB;

    thrObject.join();
    thrSegment.join();
}
