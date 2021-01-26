#include <jetson-inference/segNet.h>
#include <jetson-inference/detectNet.h>
#include <jetson-utils/loadImage.h>
#include <sys/socket.h>

#include <thread>

#include "droneNavigation.h"
#include "serverDroneStatus.h"

int main( int argc, char** argv )
{
	// a command line argument containing the image filename is expected,
	// so make sure we have at least 2 args (the first arg is the program)
	if( argc < 2 )
	{
		printf("my-recognition:  expected image filename as argument\n");
		printf("example usage:   ./my-recognition my_image.jpg\n");
		return 0;
	}

	// retrieve the image filename from the array of command line args
	const char* imgFilename = argv[1];

	// these variables will store the image data pointer and dimensions
	uchar3* imgPtr = NULL;   // shared CPU/GPU pointer to image
	int imgWidth   = 0;      // width of the image (in pixels)
	int imgHeight  = 0;      // height of the image (in pixels)
		
	// load the image from disk as uchar3 RGB (24 bits per pixel)
	if( !loadImage(imgFilename, &imgPtr, &imgWidth, &imgHeight) )
	{
		printf("failed to load image '%s'\n", imgFilename);
		return 0;
	}

	// load the GoogleNet image recognition network with TensorRT
	// you can use imageNet::RESNET_18 to load ResNet-18 model instead
	//imageNet* net = imageNet::Create(imageNet::GOOGLENET);
	detectNet* net = detectNet::Create(detectNet::SSD_MOBILENET_V2);

	// check to make sure that the network model loaded properly
	if( !net )
	{
		printf("failed to load image recognition network\n");
		return 0;
	}

	detectNet::Detection* arDetections = nullptr;
	int iDetections = net->Detect(imgPtr, imgWidth, imgHeight, &arDetections);
	printf("Results    Class    Confidence    Left Top Right Bottom\n");
	int i = 0;
	for (; i < iDetections; i++) {
		printf("Result[%d]: %s    %.4f    %.1f %.1f %.1f %.1f\n", i
									, net->GetClassDesc(arDetections[i].ClassID)
									, arDetections[i].Confidence
									, arDetections[i].Left
									, arDetections[i].Top
									, arDetections[i].Right
									, arDetections[i].Bottom); 
	}


    //This is the main app loop, the part that I'm adding
    // Allocate cuda memory: cudaAllocMapped
    // Wait for gpu to finish processing (join): cudaDeviceSynchronize
    // Clear (delete) memory: cudaFreeHost
    bool bContinueFlight = true;
    std::thread server((ServerLoop()));
    Navigator_SimpleObstruct2D nav;
	Drive_TwinTrack drive;
    DroneNavigation navSystem(nav, drive);
    std::thread client(navSystem);
    //std::thread client((DroneNavigation()));
    do {
        server.join();
        client.join();
        break;
    } while (bContinueFlight);




	// Not sure if this is right for freeing the memory
	cudaFree(imgPtr);
	cudaFree(arDetections);

	delete net;
	return 0;
}
