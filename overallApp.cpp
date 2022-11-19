#include <jetson-inference/segNet.h>
#include <jetson-inference/detectNet.h>
#include <jetson-utils/loadImage.h>
#include <sys/socket.h>

#include <thread>

#include "navigation.h"
#include "dataLogger.h"

int main( int argc, char** argv )
{
	// retrieve the image filename from the array of command line args
	char* imgFilename = nullptr;
	// a command line argument containing the image filename is expected,
	// so make sure we have at least 2 args (the first arg is the program)
	if( argc < 2 )
	{
		printf("my-recognition:  expected image filename as argument\n");
		printf("example usage:   ./my-recognition my_image.jpg\n");
		//
	} else {
		imgFilename = argv[1];
	}

    //This is the main app loop, the part that I'm adding
    // Allocate cuda memory: cudaAllocMapped
    // Wait for gpu to finish processing (join): cudaDeviceSynchronize
    // Clear (delete) memory: cudaFreeHost
    bool bContinueFlight = true;
    //std::thread server((ServerLoop()));

    DataLogger logger;
	FlightController controller(logger);
	CameraController* pCamera = CameraController::OpenCamera(CameraController::CameraController_RGB);
	Navigator nav(controller, logger, *pCamera);

	try {
		//std::thread client(&DroneNavigation<Drive_TwinTrackAdv>::run, &navSystem);
		std::thread thrNav(&Navigator::runNavigator, &nav);
		if (thrNav.joinable()) {
			std::cout << "Joinable\n";
		} else {
			std::cout << "Not joinable\n";
		}
		// std::system_error if the thread could not be started.
		do {
			thrNav.join();
			break;
		} while (bContinueFlight);
	} catch (std::exception e) {
		std::cout << "Exception: " << e.what();
	}

	return 0;
}
