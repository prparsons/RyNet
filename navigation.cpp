#include "navigation.h"

#include <thread>
#include <unistd.h>
#include <jetson-utils/cudaMappedMemory.h>

detectNet::NetworkType networkDetection = detectNet::SSD_MOBILENET_V2;
segNet::NetworkType networkSegmentation = segNet::FCN_RESNET18_VOC_512x320;

// ** ObjectDetection

ObjectDetection::ObjectDetection(DataLogger& _logger)
: iNumSlices(3)
, logger(_logger)
, pNetwork(nullptr)
{
    pNetwork = detectNet::Create(networkDetection);
	//detectNet* net = detectNet::Create(detectNet::SSD_MOBILENET_V2, 0.5f, 1, TYPE_FASTEST, DEVICE_DLA_0);DEVICE_DLA_1 DEVICE_GPU
    // DLA fails with segmentation fault using detectnet with that network. even resizing images
}

ObjectDetection::~ObjectDetection()
{
    if (pNetwork != nullptr) delete pNetwork;
}

void ObjectDetection::SetNewImage(CameraFrame& frm)
{
    mt.lock();
    frmRecent.DeepCopy(frm);
    mt.unlock();
    sigNewImage.signal();
}

bool ObjectDetection::ProcessImage(cv::Mat& _imgRGB)
{
    cv::Mat imgRGB;
    cv::resize(_imgRGB, imgRGB, cv::Size(_imgRGB.cols / 2, _imgRGB.rows / 2), cv::INTER_LINEAR);
    // 1080x1920 -> 540x960  --- 3 images at 540x320

    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)
    // Going to do iNumSlices horizontal slices of the image. 
    if (imgHeight % iNumSlices != 0) printL("ObjectDetection::ProcessImage doesn't divide by 3");
    int iSliceHeight = imgHeight / iNumSlices;
    uchar3* u3Img;
    if (CUDA_FAILED(cudaMalloc(&u3Img, imgWidth * iSliceHeight * 3))) {
        printL("ObjectDetection::ProcessImage cudaMalloc fail");
        return false;
    }
    bool bRet = false;
    for (int s = 0; s < iNumSlices; s++) {
        intptr_t imgRGBOffset = s * imgWidth * imgHeight;
        cudaMemcpy(u3Img, ((uint8_t*)imgRGB.data) + imgRGBOffset, imgWidth * iSliceHeight * 3, cudaMemcpyHostToDevice);   // shared CPU/GPU pointer to image
        detectNet::Detection* arDetections = nullptr;
        int iDetections = pNetwork->Detect(u3Img, imgWidth, iSliceHeight, &arDetections);
        if (iDetections < 1) {
            return false;
        }

        char sc[128];
        for (int i = 0; i < iDetections; i++) {
            bRet = true;
            snprintf(sc, 127, "Detection Slice,Class,Confidence[%d,%d,%d] LTRB:%d %d %d %d", 
                    s,
                    int(arDetections[i].ClassID), 
                    int(arDetections[i].Confidence * 100),
                    int(arDetections[i].Left), 
                    int(arDetections[i].Top), 
                    int(arDetections[i].Right), 
                    int(arDetections[i].Bottom));
            logger.LogString(sc);
        }
    }

    cudaFree(u3Img);
    return bRet;
}

void ObjectDetection::run()
{
    printL("ObjectDetection::run");
    Signal* pSigRGB = new Signal(sigNewImage);
    CameraFrame frmProcess;
    int iFramesProcessed = 0;
    while (!ss.shouldStop()) {
        pSigRGB->wait();
        mt.lock();
        frmProcess = frmRecent; // This doesn't need to be a deep copy
        mt.unlock();
        if (frmProcess.matrix.empty()) continue;
        iFramesProcessed++;

        if (ProcessImage(frmProcess.matrix)) {
            logger.LogImage(frmProcess, "detect");
        }
    }
    delete pSigRGB;
    printf("ObjectDetection::run Frames Processed:%d\n", iFramesProcessed);
}

void ObjectDetection::SendStopSignal()
{
    ss.stop();
    // If the camera has stopped, or isn't running, then we need to signal or else it won't ever
    //  get to the ss.shouldStop check.
    sigNewImage.signal();
}

void ObjectDetection::SetNumberImageSlices(int _iNumSlices)
{
    iNumSlices = _iNumSlices;
}

bool ObjectDetection::SetNetwork(const char* scNetwork)
{
    if (strcmp(scNetwork, "SSD_MOBILENET_V2") == 0) {
        networkDetection = detectNet::SSD_MOBILENET_V2;
    } else if (strcmp(scNetwork, "SSD_MOBILENET_V1") == 0) {
        networkDetection = detectNet::SSD_MOBILENET_V1;
    } else if (strcmp(scNetwork, "SSD_INCEPTION_V2") == 0) {
        networkDetection = detectNet::SSD_INCEPTION_V2;
    } else {
        return false;
    }
    return true;
}

// ** TerrainSegmentation

TerrainSegmentation::TerrainSegmentation(DataLogger& _logger)
: bHabitatVisible(false)
, uTargetClass(HABITAT)
, iNumSlices(3)
, fImageFOV(50) // I calculated this number based on the imx477 sensor size with a 6mm lens
, logger(_logger)
, pMask(nullptr)
{
    pNetwork = segNet::Create(networkSegmentation);//FCN_RESNET18_SUNRGB_640x512);//segNet::FCN_RESNET18_DEEPSCENE_576x320);FCN_ALEXNET_AERIAL_FPV_720p
//  static segNet* Create( NetworkType networkType=FCN_ALEXNET_CITYSCAPES_SD, uint32_t maxBatchSize=DEFAULT_MAX_BATCH_SIZE,
//					   precisionType precision=TYPE_FASTEST, deviceType device=DEVICE_GPU, bool allowGPUFallback=true );
}

TerrainSegmentation::~TerrainSegmentation()
{
    if (pMask != nullptr) delete[] pMask;// cudaFree(pMask);
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

void TerrainSegmentation::SetNewImage(CameraFrame& frm)
{
    mt.lock();
    frmRecent.DeepCopy(frm);
    mt.unlock();
    sigNewImage.signal();
}

void TerrainSegmentation::run()
{
    printL("TerrainSegmentation::run");
    Signal* pSigRGB = new Signal(sigNewImage);
    CameraFrame frmProcess;
    int iFramesProcessed = 0;
    while (!ss.shouldStop()) {
        pSigRGB->wait();
        mt.lock();
        frmProcess = frmRecent; // This doesn't need to be a deep copy?
        mt.unlock();
        if (frmProcess.matrix.empty()) continue;
        
        iFramesProcessed++;
        if (ProcessImage(frmProcess.matrix)) {
            logger.LogImage(frmProcess, "segment");
            char sc[128];
            mt.lock();
            sprintf(sc, "Good Habitat Found %2.2f", fBestDirection);
            mt.unlock();
            logger.LogString(sc);
        }
    }
    delete pSigRGB;
    printf("TerrainSegmentation::run Frames Processed: %d\n", iFramesProcessed);
}

void TerrainSegmentation::SendStopSignal()
{
    ss.stop();
    // If the camera has stopped, or isn't running, then we need to signal or else it won't ever
    //  get to the ss.shouldStop check.
    sigNewImage.signal();
}

void TerrainSegmentation::SetTargetClass(uint8_t _uTargetClass)
{
    uTargetClass = _uTargetClass;
}

void TerrainSegmentation::SetNumberImageSlices(int _iNumSlices)
{
    iNumSlices = _iNumSlices;
}

bool TerrainSegmentation::SetNetwork(const char* scNetwork)
{
    if (strcmp("FCN_RESNET18_CITYSCAPES_512x256", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_CITYSCAPES_512x256;
        return true;
    }
    if (strcmp("FCN_RESNET18_CITYSCAPES_1024x512", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_CITYSCAPES_1024x512;
        return true;
    }
    if (strcmp("FCN_RESNET18_CITYSCAPES_2048x1024", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_CITYSCAPES_2048x1024;
        return true;
    }
    if (strcmp("FCN_RESNET18_DEEPSCENE_576x320", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_DEEPSCENE_576x320;
        return true;
    }
    if (strcmp("FCN_RESNET18_DEEPSCENE_864x480", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_DEEPSCENE_864x480;
        return true;
    }
    if (strcmp("FCN_RESNET18_VOC_320x320", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_VOC_320x320;
        return true;
    }
    if (strcmp("FCN_RESNET18_VOC_512x320", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_VOC_512x320;
        return true;
    }
    if (strcmp("FCN_RESNET18_SUNRGB_512x400", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_SUNRGB_512x400;
        return true;
    }
    if (strcmp("FCN_RESNET18_SUNRGB_640x512", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_RESNET18_SUNRGB_640x512;
        return true;
    }
    if (strcmp("FCN_ALEXNET_AERIAL_FPV_720p", scNetwork) == 0) {
        networkSegmentation = segNet::FCN_ALEXNET_AERIAL_FPV_720p;
        return true;
    }
    return false;
}

bool TerrainSegmentation::ProcessImage(cv::Mat& imgRGB)
{
    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)
    // 1080x1920  -- 3 images at 1080x640
    uchar3* u3Img;
    // Going to do iNumSlices horizontal slices of the image. That and it being a 3 channel image is why size is only widthxheight
    if (imgHeight % iNumSlices != 0) printL("ObjectDetection::ProcessImage doesn't divide by 3");
    int iSliceHeight = imgHeight / iNumSlices;
    if (CUDA_FAILED(cudaMalloc(&u3Img, imgWidth * iSliceHeight * 3))) {
        printL("TerrainSegmentation::ProcessImage cudaMalloc fail");
        return false;
    }
    bool bRet = false;
    for (int s = 0; s < iNumSlices; s++) {
        intptr_t imgRGBOffset = s * imgWidth * iSliceHeight;
        cudaMemcpy(u3Img, ((uint8_t*)imgRGB.data) + imgRGBOffset, imgWidth * iSliceHeight * 3, cudaMemcpyHostToDevice);

        if(!pNetwork->Process(u3Img, imgWidth, iSliceHeight)) {
            printf("TerrainSegmentation::ProcessImage fail");
            cudaFree(u3Img);        
            return false;
        }

        if (pMask == nullptr) {
            pMask = new uint8_t[imgWidth*iSliceHeight];
        }
        if(!pNetwork->Mask(pMask, imgWidth, iSliceHeight)) {
            printL("TerrainSegmentation::ProcessImage mask fail");
            cudaFree(u3Img);
            return false;
        }
        if (ScanMask(pMask, imgWidth, iSliceHeight)) {
            // We're starting at the top, which would be the furthest away.
            //  So as soon as we find our target, then just stop scanning
            bRet = true;
            char sc[64];
            sprintf(sc, "ScanMask Success slice %d", s);
            logger.LogString(sc);
            break;
        }
    }
    cudaFree(u3Img);
    
    return bRet;
}

bool TerrainSegmentation::ScanMask(uint8_t* pMask, int iWidth, int iHeight)
{
    int size = iWidth * iHeight;
    float fAngleAvgSum = 0;
    int iAngleAvgCount = 0;
    uint8_t pixel;
    // Should be able to SIMD this
    for (int i = 0; i < size; i++) {
        pixel = pMask[i];
        if (pixel == uTargetClass) {
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
    int maskWidth = iWidth / 2;
    int maskHeight = iHeight / 2;
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
, pThread(nullptr)
, detector(logger)
, segmenter(logger)
{

}

Navigator::~Navigator()
{
    Stop();
}
    
void Navigator::Start()
{
    pThread = new std::thread(&Navigator::runNavigator, this);
}

void Navigator::Join()
{
    if (pThread != nullptr) {
        pThread->join();
        delete pThread;
        pThread = nullptr;
    }
}

void Navigator::SignalStop()
{
    if (pThread != nullptr) {
        ssMain.stop();
        // If there's nothing coming from the camera, we need to signal it or else the thread
        //  will never get to the ss.shouldStop().
        Signal* pSigRGB = camera.GetSignalRGB();
        pSigRGB->signal();
        delete pSigRGB;
    }
}

void Navigator::Stop()
{
    if (pThread != nullptr) {
        ssMain.stop();
        // If there's nothing coming from the camera, we need to signal it or else the thread
        //  will never get to the ss.shouldStop().
        Signal* pSigRGB = camera.GetSignalRGB();
        pSigRGB->signal();
        pThread->join();
        delete pSigRGB;
        delete pThread;
        pThread = nullptr;
    }
}

void Navigator::SetNumberImageSlices(int iNumSlices)
{
    detector.SetNumberImageSlices(iNumSlices);
    segmenter.SetNumberImageSlices(iNumSlices);
}

void Navigator::runNavigator()
{
    printL("Navigator::runNavigator");
    Signal* pSigRGB = camera.GetSignalRGB();

    bool bGridSearching = true;
    float fMargin = 5;

    std::thread thrObject(&ObjectDetection::run, &detector);
    std::thread thrSegment(&TerrainSegmentation::run, &segmenter);
    while (!ssMain.shouldStop()) {
        pSigRGB->wait();
        CameraFrame frameNew;
        if (!camera.GetFrameRGB(frameNew)) {
            printL("runNavigator frame was not valid");
            continue;
        }
        detector.SetNewImage(frameNew);
        segmenter.SetNewImage(frameNew);

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

    printL("Navigator::runNavigator loop done, stopping segmenter and detector");
    // Handle all the cleanup
    detector.SendStopSignal();
    segmenter.SendStopSignal();
    delete pSigRGB;
    // Wait until the threads actually stop (don't want to delete the object while thread is running).
    thrObject.join();
    thrSegment.join();
}
