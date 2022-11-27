#include <jetson-inference/segNet.h>
#include <jetson-inference/detectNet.h>
#include <jetson-utils/loadImage.h>
#include <sys/socket.h>

#include <thread>

#include "navigation.h"
#include "dataLogger.h"

int main( int argc, char** argv )
{
    //This is the main app loop, the part that I'm adding
    // Allocate cuda memory: cudaAllocMapped
    // Wait for gpu to finish processing (join): cudaDeviceSynchronize
    // Clear (delete) memory: cudaFreeHost
    bool bContinueFlight = true;

    DataLogger logger;
	CameraController* pCamera = CameraController::OpenCamera(CameraController::CameraController_RGB);
	FlightController controller(logger, *pCamera);
	Navigator nav(controller, logger, *pCamera);
        printL("Starting main loop");
	try {
		std::thread thrLogger(&DataLogger::run, &logger);
		pCamera->run();
		std::thread thrNav(&Navigator::runNavigator, &nav);
		if (thrNav.joinable()) {
			std::cout << "Joinable\n";
		} else {
			std::cout << "Not joinable\n";
		}
		// std::system_error if the thread could not be started.
		do {
			thrNav.join();
			thrLogger.join();
			break;
		} while (bContinueFlight);
	} catch (std::exception e) {
		std::cout << "Exception: " << e.what();
	}

	return 0;
}
