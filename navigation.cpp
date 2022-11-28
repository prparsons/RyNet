#include "navigation.h"

#include <thread>
#include <unistd.h>


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
        snprintf(sc, 127, "Detection Class[%d] LTRB:%d %d %d %d", 
                int(arDetections[i].ClassID), 
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
            //logger.LogImage(imgProcess);
        }
    }
    delete pSigRGB;
}

// ** TerrainSegmentation

TerrainSegmentation::TerrainSegmentation(DataLogger& _logger)
: bHabitatVisible(false)
, logger(_logger)
{
    pNetwork = nullptr;//segNet::Create(segNet::FCN_RESNET18_SUNRGB_640x512);//segNet::FCN_RESNET18_DEEPSCENE_576x320);FCN_ALEXNET_AERIAL_FPV_720p
}

bool TerrainSegmentation::ProcessImage(image& imgRGB)
{
    printL("TerrainSegmentation::ProcessImage");
    uchar3* u3Img = (uchar3*)imgRGB.data;   // shared CPU/GPU pointer to image
    int imgWidth = imgRGB.cols;      // width of the image (in pixels)
    int imgHeight = imgRGB.rows;      // height of the image (in pixels)

    if(!pNetwork->Process(u3Img, imgWidth, imgHeight)) {
        printf("TerrainSegmentation::ProcessImage fail");
        return false;
    }
    
    if(!pNetwork->Mask(u3Mask, imgWidth, imgHeight, segNet::FILTER_POINT)) {
        return false;
    }

    return ScanMask(u3Mask, imgWidth, imgHeight);
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
        
        //if (ProcessImage(imgProcess)) {
        //    logger.LogImage(imgProcess);
        //    logger.LogString("Good Habitat Found");
        //}
    }
    delete pSigRGB;
}

bool TerrainSegmentation::ScanMask(uchar3* imgMask, int iWidth, int iHeight)
{
    printL("TerrainSegmentation::ScanMask");
    int size = iWidth * iHeight;
    float fAngleAvgSum = 0;
    int iAngleAvgCount = 0;
    uint32_t pixel;
    for (int i = 0; i < size; i++) {
        pixel = *((uint32_t*)(&imgMask[i])) >> 8;
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
    logger.LogMask(imgMask, iWidth, iHeight);
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
