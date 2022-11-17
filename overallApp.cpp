#include <jetson-inference/segNet.h>
#include <jetson-inference/detectNet.h>
#include <jetson-utils/loadImage.h>
#include <sys/socket.h>

#include <thread>

#include "droneNavigation.h"
#include "serverDroneStatus.h"

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

    DroneNavigation<Drive_TwinTrackAdv> navSystem;
	Navigator_PersonFinder* pNavPerson = new Navigator_PersonFinder();
    Navigator_SimpleDepthObstruct* pNavDepth = new Navigator_SimpleDepthObstruct();
	navSystem.addNav(pNavPerson);
    navSystem.addNav(pNavDepth);

	try {
		std::thread client(&DroneNavigation<Drive_TwinTrackAdv>::run, &navSystem);
		if (client.joinable()) {
			std::cout << "Joinable\n";
		} else {
			std::cout << "Not joinable\n";
		}
		// std::system_error if the thread could not be started.
		do {
			//server.join();
			client.join();
			break;
		} while (bContinueFlight);
	} catch (std::exception e) {
		cout << "Exception: " << e.what();
	}

	return 0;
}
