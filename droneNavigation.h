#pragma once

#include <vector>

#include "comms.h"
#include "locksignal.h"
#include "planner.h"


template<class DriveType>
class DroneNavigation {
public:
    DroneNavigation();
    ~DroneNavigation();

    void run();
    StopSignal ss;

    void addNav(Navigator* pNav);

private:
    void _sendGuidance();

    Signal signalMainMessage;
    std::vector<Navigator*> arNav;
    DriveType drive;
};

// ***************
// DroneNavigation

template<class DriveType>
DroneNavigation<DriveType>::DroneNavigation()
: arNav()
, drive(arNav)
{
    printf("DroneNavigation\n");
}

template<class DriveType>
DroneNavigation<DriveType>::~DroneNavigation()
{
    for (int i = 0; i < arNav.size(); i++) {
        delete arNav[i];
    }
}

template<class DriveType>
void DroneNavigation<DriveType>::run(void)
{
    std::cout << "DroneNavigation Thread\n";
    // std::system_error if the thread could not be started.
    std::vector<std::thread*> arNavThreads(arNav.size());
    for (int i = 0; i < arNav.size(); i++) {
        arNavThreads[i] = new std::thread(&Navigator::run, arNav[i], new Signal(signalMainMessage));
    }

    do {
        printf("MainLoop\n");
        for (int i = 0; i < arNav.size(); i++) {
            printf("MainLoop[%d]\n", i);
            Navigator::dirty isDirty = arNav[i]->isDirty();
            if (isDirty) {
                printf("DroneNavigation::run isDirty true\n");
                arNav[i]->setClean();
                if (!drive.isPlanValid(*(arNav[i]), isDirty)) {
                    printf("Creating new plan\n");
                    drive.createNewPlan();
                    _sendGuidance();
                }
            }
        }
        signalMainMessage.wait(drive.getLoopTimeoutMS());
    } while (!ss.shouldStop());
    for (int i = 0; i< arNav.size(); i++) {
        arNav[i]->ss.stop();
        arNavThreads[i]->join();
    }
}

template<class DriveType>
void DroneNavigation<DriveType>::addNav(Navigator* pNav)
{
    arNav.push_back(pNav);
}

template<class DriveType>
void DroneNavigation<DriveType>::_sendGuidance()
{
    char strCommand[500];
    snprintf(   strCommand, 
                500,
                "<page><command><speed>%1.5f</speed></command><command><steer>%1.5f</steer></command><command><climb>%1.5f</climb></command></page>",
                drive.getSpeed(),
                drive.getSteer(),
                drive.getClimb());
    if (!sendXML(strCommand)) {
        printf("Failed to send guidance\n");
    }
}
