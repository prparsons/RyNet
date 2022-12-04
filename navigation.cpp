#include "navigation.h"

#include <thread>
#include <unistd.h>
#include <jetson-utils/cudaMappedMemory.h>

// ** ObjectDetection

ObjectDetection::ObjectDetection(DataLogger& _logger)
: logger(_logger)
, pNetwork(nullptr)
{
    pNetwork = detectNet::Create(detectNet::SSD_MOBILENET_V2);
	//detectNet* net = detectNet::Create(detectNet::SSD_MOBILENET_V2, 0.5f, 1, TYPE_FASTEST, DEVICE_DLA_0);DEVICE_DLA_1 DEVICE_GPU
    // DLA fails with segmentation fault using detectnet with that network. even resizing images
}

ObjectDetection::~ObjectDetection()
{
    if (pNetwork != nullptr) delete pNetwork;
}

void ObjectDetection::SetNewImage(image& img)
{
    mt.lock();
    imgRecent = img;
    mt.unlock();
    sigNewImage.signal();
}

bool ObjectDetection::ProcessImage(image& _imgRGB)
{
    cv::Mat imgRGB;
    cv::resize(_imgRGB, imgRGB, cv::Size(640, 360), cv::INTER_LINEAR);

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
        
        if (ProcessImage(imgProcess)) {
            // This works, but at 2fps, this could still add up to way more than necessary.
            // Need a rate limit 
            //logger.LogImage(imgProcess);
        }
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
    pNetwork = segNet::Create(segNet::FCN_RESNET18_VOC_512x320);//FCN_RESNET18_SUNRGB_640x512);//segNet::FCN_RESNET18_DEEPSCENE_576x320);FCN_ALEXNET_AERIAL_FPV_720p
//  static segNet* Create( NetworkType networkType=FCN_ALEXNET_CITYSCAPES_SD, uint32_t maxBatchSize=DEFAULT_MAX_BATCH_SIZE,
//					   precisionType precision=TYPE_FASTEST, deviceType device=DEVICE_GPU, bool allowGPUFallback=true );
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
    mt.lock();
    bool bReturn = bHabitatVisible;
    mt.unlock();
    return bReturn;
}

float TerrainSegmentation::GetHabitatDirection()
{
    mt.lock();
    float fReturn = fBestDirection;
    mt.unlock();
    return fReturn;
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
            mt.lock();
            sprintf(sc, "Good Habitat Found %2.2f", fBestDirection);
            mt.unlock();
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
        mt.lock();
        bHabitatVisible = false;
        mt.unlock();
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
    mt.lock();
    fBestDirection = fAngleAvgSum / iAngleAvgCount;
    bHabitatVisible = true;
    mt.unlock();
    return true;
}

Navigator::Navigator(FlightController& _control, DataLogger& _logger, CameraController& _camera)
: tLastChange(0)
, tMinTimeBetweenChanges(5)
, control(_control)
, logger(_logger)
, camera(_camera)
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
    float fMargin = 5;

    std::thread thrObject(&ObjectDetection::run, &detector);
    std::thread thrSegment(&TerrainSegmentation::run, &segmenter);

    while (!ssMain.shouldStop()) {
        pSigRGB->wait();
        if (pFrame) delete pFrame;
        pFrame = camera.GetFrameRGB();
        
        image imgNew = pFrame->getImage();
        detector.SetNewImage(imgNew);
        segmenter.SetNewImage(imgNew);

        if (bGridSearching) { // Habitat not visible, we're currently running search pattern
            // Good terrain visible?
            if (segmenter.IsHabitatVisible()) {
                char sc[64];
                // If enough time since last command, then we'll change
                if (time(nullptr) - tLastChange > tMinTimeBetweenChanges) {
                    control.SetDirection(segmenter.GetHabitatDirection());
                    sprintf(sc, "Grid Search -> Good Habitat %1.3f", segmenter.GetHabitatDirection());
                    bGridSearching = false;
                    tLastChange = time(nullptr);
                } else {
                    sprintf(sc, "[delay]Grid Search -> Good Habitat %1.3f", segmenter.GetHabitatDirection());
                }
                logger.LogString(sc);
            }
        } else { // Habitat already visible
            // Good terrain still visible?
	        if (segmenter.IsHabitatVisible()) {
                // If it's in the same direction then do nothing
                if (abs(segmenter.GetHabitatDirection()) < fMargin) continue;
                char sc[64];
                // If enough time since last command, then we can change
                if (time(nullptr) - tLastChange > tMinTimeBetweenChanges) {
                    control.SetDirection(segmenter.GetHabitatDirection());
                    sprintf(sc, "Good Habitat Change -> %1.3f", segmenter.GetHabitatDirection());
                    tLastChange = time(nullptr);
                } else {
                    sprintf(sc, "[delay]Good Habitat Change -> %1.3f", segmenter.GetHabitatDirection());
                }
                logger.LogString(sc);
            } else { // No good terrain is visible right now
                // So if there's been enough time we'll switch back to grid
                if (time(nullptr) - tLastChange > tMinTimeBetweenChanges) {
                    control.RestoreRouteNaive();
                    logger.LogString("Good Habitat -> Grid Search");
                    bGridSearching = true;
                    tLastChange = time(nullptr);
                } else {
                    logger.LogString("[delay]Good Habitat -> Grid Search");
                }
            }
        }
    }

    // Handle all the cleanup
    StopSignal* pss = detector.GetStopSignal();
    pss->stop();
    delete pss;
    pss = segmenter.GetStopSignal();
    pss->stop();
    delete pss;
    if (pFrame != nullptr) delete pFrame;
    delete pSigRGB;
    // Do we really need to join them?
    thrObject.join();
    thrSegment.join();
}
