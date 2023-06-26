#!/usr/bin/env python

# %%
from __future__ import print_function
from dronekit import connect, Command, VehicleMode, LocationGlobal
import time
import socket
import xml.etree.ElementTree as ET
import haversine as hs
import selectors
import argparse  
from tsp import *
import numpy as np

"""
Command Documentation:

<altitude_retrieve />
    Get current altitude
    --livetest enabled

<battery_retrieve />
    Get current battery level, percent level from 0-100.
    --livetest enabled

<clean_dist>DISTANCE_M</clean_dist>
    Sets the distance from visited point that is close enough to remove a waypoint from mission list.
    --livetest disabled

<dir dist='200'>OFFSET_HEADING_DEGREES</dir>
    Sets mode to GUIDED, and then turns indicated amount
        dist -- distance to set waypoint in meters[default:1000]
    --livetest disabled

<heading_retrieve />
    Get current vehicle heading
    --livetest enabled

<mode_retrieve />
    Retrieves the current mode and returns it to the client as a string ("AUTO", "GUIDED")
    --livetest enabled

<obs>
    <dist dir='left'>DISTANCE_CM</dist>
    <dist dir='center'>DISTANCE_CM</dist>
    <dist dir='right'>DISTANCE_CM</dist>
</obs>
    Sends obstacle distance sensor message
        dir -- direction of the provided distance (valid: 'left', 'center', or 'right')
    --livetest enabled

<pos_retrieve />
    Gets the current position (lat,lon) and returns it back to the client separated by \n
    --livetest enabled

<route_load setauto='true'>MISSION_FILENAME</route_load>
    Loads a waypoints file as the current mission
        setauto -- if present and true, will set the autopilot's mode to auto after setting the route
    --livetest disabled

<route_restore loadsaved='true' lat='35.1' lon='-110.1' clean='simple' setauto='true' />
    Restores a route, optionally cleaning it and setting vehicle mode to AUTO
        loadsave -- if present and true, will load route from savedMission.txt[default:current mission]
        lat,lon -- if present, will be used as the current location in cleaning routine
        clean -- [simple] will find closest waypoint and remove everything prior to it
                 [2opt] will remove visited waypoints and use 2opt to reconstruct route
        setauto -- if present and true, will set the autopilot's mode to auto after setting the route
    --livetest disabled

<route_retrieve />
    Gets current route and returns it back to client as a string
    --livetest disabled

<route_save />
    Saves current route to savedMission.txt
    --livetest disabled

<route_set setauto='true'>ROUTE</route_set>
    Sets the text of element as the current route
        setauto -- if present and true, will set the autopilot's mode to auto after setting the route
    --livetest disabled

<rtl />
    Sets vehicle mode to RTL
    --livetest disabled

<speed_retrieve />
    Gets vehicle groundspeed in meters per second
    --livetest enabled

<takeoff setauto='true'>ALTITUDE_METERS</takeoff>
    Will arm and take off (simple example from dronekit)
        setauto -- will change the vehicle mode to AUTO 3 sec after takeoff
    --livetest disabled


SITL Testing instructions:
https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html

Jetson Xavier NX connection string UART:
/dev/ttyTHS0

Note: dronekit-sitl will not run on ARM (Jetson, pi, etc...)

-Open Mission Planner
-Go to the Simulation tab
-Select Multi-Rotor
    Mission Planner will download whatever it needs and startup the SITL
    or you can check the Do not download checkbox
-Mision Planner will start the SITL and automatically connect to it, bringing up its control interface. It will also
 open a console window which should also tell you what other ports it's opened (likely 5762 and 5763).
-If you're getting connect failures from dronekit, try increasing the timeout to 360
"""

parser = argparse.ArgumentParser(description='UDP Dronekit server')
parser.add_argument('--connect', 
                    help="Vehicle connection target string.\nMission Planner SITL example: 'tcp:127.0.0.1:5762'\nIf not specified, dronekit SITL used")
parser.add_argument('--port', type=int, default=3636, help="Port to listen on [default:3636]")
parser.add_argument('--livetest', default='False', action='store_true', help='If True, will only send <obs /> messages to the drone.')
parser.add_argument('--clean_dist_visit', type=int, default=10, help='Distance between visited points that is close enough to consider them duplicates.')
parser.add_argument('--clean_dist_way', type=int, default=100, help='Distance from visited point that is close enough to remove a waypoint from mission list.')

#args = parser.parse_args()
#Testing without command line, use below
args = parser.parse_args("--port 3636 --connect tcp:127.0.0.1:5762 --livetest".split())
visitedList = []

def readmissionF(aFileName, vehicle):
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def readmission(strMission, vehicle):
    cmds = vehicle.commands
    missionlist=[]
    strMissionLines = strMission.split('\n')
    for line in strMissionLines:
        if len(line) == 0:
            continue # should have a trailing \n that this will catch
        linearray=line.split('\t')
        ln_index=int(linearray[0])
        ln_currentwp=int(linearray[1])
        ln_frame=int(linearray[2])
        ln_command=int(linearray[3])
        ln_param1=float(linearray[4])
        ln_param2=float(linearray[5])
        ln_param3=float(linearray[6])
        ln_param4=float(linearray[7])
        ln_param5=float(linearray[8])
        ln_param6=float(linearray[9])
        ln_param7=float(linearray[10])
        ln_autocontinue=int(linearray[11].strip())
        cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
        missionlist.append(cmd)
    return missionlist

def upload_missionF(aFileName, vehicle):
    print("\nUpload mission from a file: %s" % aFileName)
    missionlist = readmissionF(aFileName, vehicle)
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

def upload_missionStr(strMission, vehicle):
    print("\nUpload mission from a string")
    missionlist = readmission(strMission, vehicle)
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

def upload_mission(missionlist, vehicle):
    print("\nUpload mission from a list")
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

# Download current mission and return it in a list
def download_mission(vehicle, skipVisited=False):
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    #from docs, https://dronekit-python.readthedocs.io/en/stable/automodule.html, cmds.next will return the index of current waypoint
    i = 0
    for cmd in cmds:
        if skipVisited:
            if i >= cmds.next:
                missionlist.append(cmd)
            i += 1
        else:
            missionlist.append(cmd)
    return missionlist

def save_missionF(aFileName, vehicle):
    print("\nSave mission from Vehicle to file: %s" % aFileName)
    missionlist = download_mission(vehicle)
    output='QGC WPL 110\n'
    # Home location is first waypoint (Do we really want that?)
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)

def mission_to_str(missionList):
    output = ''
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    for cmd in missionList:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    return output

def mission_cleanSimple(missionList, nextWaypoint, fLat, fLon):
    missionListClean = []
    fDistClosest = 9999999999.0
    for cmd in missionList[nextWaypoint:]:
        fDist = hs.haversine((fLat, fLon), (cmd.x, cmd.y), unit=hs.Unit.METERS)
        if fDist < fDistClosest:
            fDistClosest = fDist
            missionListClean = []
        missionListClean.append(cmd)
    return missionListClean

class reducedPoint:
    def __init__(self, pt):
        self.Xt = int(pt[0] * 10000) * 0.0001
        self.Yt = int(pt[1] * 10000) * 0.0001
        self.pt = (pt[0], pt[1])

    def __str__(self):
        return "(%f, %f)" % (self.Xt, self.Yt)

    def __repr__(self):
        return "(%f, %f)" % (self.Xt, self.Yt)


def mission_sortVisted(visitedList):
    sortedList = []
    # Create a list of lists...reduce precision so that close latitude points will
    #   be grouped in the same list, which is then sorted by longitude in the same
    #   way.
    for point in visitedList:
        newPoint = reducedPoint(point)
        inserted = False
        for i in range(len(sortedList)):
            if newPoint.Xt >= sortedList[i][0].Xt:
                j = 0
                if newPoint.Xt == sortedList[i][0].Xt:
                    while j < len(sortedList[i]):
                        if newPoint.Yt > sortedList[i][j].Yt:
                            inserted = True
                            sortedList[i].insert(j, newPoint)
                            break
                        j += 1
                else:
                    #Insert new sublist in sortedList
                    inserted = True
                    sortedList.insert(i, [newPoint])

                if inserted is False:
                    # If we make it here, it needs to go the end of the sublist [i]
                    sortedList[i].append(newPoint)
                    inserted = True
                break
        if inserted is False:
            sortedList.append([newPoint])
    return sortedList

def sortLat(visitedList, point, clean_dist_way):
    # VisitedList is the sublist of points that share the same reduced precision latitude.
    #   Now search the sub list for the thing with the closest longitude to point. Once 
    #   that is found, it should be the closest visited point to point, so then check the
    #   distance of that and compare it to clean_dist_way, returning True if it's closer 
    #   than that and should be removed
    deltaToY = abs(point.y - visitedList[0].Yt)
    bSearched = False
    removePoint = False
    lenMin1 = len(visitedList) - 1
    for j in range(lenMin1):
        newDelta = abs(point.y - visitedList[j+1].Yt)
        if newDelta >= deltaToY:
            #check the distance
            bSearched = True
            fDist = hs.haversine(visitedList[j].pt, (point.x, point.y), unit=hs.Unit.METERS)
            if fDist < clean_dist_way:
                removePoint = True
            break

        deltaToY = newDelta
    if not bSearched:
        fDist = hs.haversine(visitedList[lenMin1].pt, (point.x, point.y), unit=hs.Unit.METERS)
        if fDist < clean_dist_way:
            removePoint = True
    
    return removePoint

def sort(visitedList, missionList, clean_dist_way):
    # First, sort the mission list
    visitedSortedList = mission_sortVisted(visitedList)
    missionListClean = []
    lenMin1 = len(visitedSortedList) - 1
    for cmd in missionList:
        removePoint = False
        #Only check waypoints...but we need to get rid of takeoff, and all the waypoints we've
        #visited while following the route before we went off Auto. so visitedList is a bad
        #way to do this as currently implemented
        if cmd.command == 16:
            # Look for the points with the closest latitude, then we'll search that sublist
            #   for things with the closest longitude
            deltaToX = abs(cmd.x - visitedSortedList[0][0].Xt)
            bSearched1 = False
            for i in range(lenMin1):
                newDelta = abs(cmd.x - visitedSortedList[i+1][0].Xt)
                if newDelta >= deltaToX:
                    bSearched1 = True
                    removePoint = sortLat(visitedSortedList[i], cmd, clean_dist_way)
                    break 
                deltaToX = newDelta
            if not bSearched1:
                removePoint = sortLat(visitedSortedList[lenMin1], cmd, clean_dist_way)
                
        if not removePoint:
            missionListClean.append(cmd)
    
    return missionListClean

def mission_removeVisited(missionList):
    global visitedList
    """
    Best way to optimize this i've thought of so far, sort both lists. Then basically use a search algorithm for each point in
    the missionList to see if it's been near a visited point.
    """
    #First, go over the visitedList and remove points that are too close to each other
    pointLast = visitedList[0]
    visitedReplace = [visitedList[0]]
    for point in visitedList[1:]:
        fDist = hs.haversine((pointLast[0], pointLast[1]), (point[0], point[1]), unit=hs.Unit.METERS)
        if fDist > args.clean_dist_visit:
            visitedReplace.append(point)
            pointLast = point
    visitedList = visitedReplace

    testOptim = True
    
    #Now, go over missionList and remove any points close to one that we've visited while off the pre-programmed route
    missionListClean = []

    if not testOptim:
        for cmd in missionList:
            removePoint = False
            #Only check waypoints...but we need to get rid of takeoff, and all the waypoints we've
            #visited while following the route before we went off Auto. so visitedList is a bad
            #way to do this as currently implemented
            if cmd.command == 16:
                for point in visitedList:
                    fDist = hs.haversine((point[0], point[1]), (cmd.x, cmd.y), unit=hs.Unit.METERS)
                    if fDist < args.clean_dist_way:
                        removePoint = True
                        break
            if not removePoint:
                missionListClean.append(cmd)
    else:
        missionListClean = sort(visitedList, missionList, args.clean_dist_way)

    return missionListClean

def mission_cleanBetter(missionList, nextWaypoint, fLat, fLon):
    """
    Assume the mission will be a grid search
    So, this could be a traveling salesman problem I suppose.
    One thing to consider is battery life: if you have just enough battery to finish your desired grid and
    go off the grid search to follow something, you likely won't have enough battery to finish the original
    route. So leave reserve (possibly not going as far as you could have), plan exactly enough and don't 
    worry if you can't visit it all, or plan more than you could visit anyways knowing you can't visit it
    all. 
    Further thoughts on battery life...figure out the best route...do we have enough battery to do that?
        no:
            remove the furthest point, try again? (that's expensive)
            or
            figure out how much distance we need to remove, start at the furthest point...go to the next 
            furthest point (whichever direction that is from the furthest). Continue that strategy until
            the tour we remove matches the distance we need to recover, then re-run tsp
    Avoid revisiting areas maximizes area covered...and we know the start point.

    """
    missionList = mission_removeVisited(missionList)
    cmdCurrent = Command(missionList[0].target_system,
                         missionList[0].target_component,
                         missionList[0].seq,
                         missionList[0].frame,
                         missionList[0].command,
                         0,
                         0,
                         missionList[0].param1,
                         missionList[0].param2,
                         missionList[0].param3,
                         missionList[0].param4,
                         vehicle.location.global_frame.lat,
                         vehicle.location.global_frame.lon,
                         100)#this needs to be a parameter
    missionList.insert(0, cmdCurrent)
    return tsp2opt(missionList)

def parseXML(message, vehicle, UDPServerSocket, address):
    try:
        root = ET.fromstring(message)
        print(root)
        handleCommand(root, vehicle, UDPServerSocket, address)
    except:
        print("parseXML error")
        return

def cmdTakeoff(vehicle, element):
    arm_and_takeoff(vehicle, float(element.text))
    time.sleep(3)

    if 'setauto' in element.attrib:
        if element.attrib['setauto'] == 'true':
            vehicle.mode = VehicleMode("AUTO")
            while not vehicle.mode.name=='AUTO':
                time.sleep(0.01)

def cmdDir(vehicle, element):
    ptCurrent = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    distanceM = 1000
    if 'dist' in element.attrib:
        distanceM = float(element.attrib['dist'])
    dirDeg = float(element.text) + vehicle.heading;
    dirRad = dirDeg * 3.14159 / 180;
    latNew, lonNew = hs.inverse_haversine(ptCurrent, distanceM*.001, dirRad)
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name=='GUIDED':
        time.sleep(0.01)
    locationNew = LocationGlobal(latNew, lonNew, vehicle.location.global_frame.alt)
    vehicle.simple_goto(locationNew)

def cmdModeRetrieve(vehicle, UDPServerSocket, address):
    strReply = vehicle.mode.name
    UDPServerSocket.sendto(str.encode(strReply), address)

def cmdDirRetrieve(vehicle, UDPServerSocket, address):
    heading = vehicle.heading
    strReply = "%.1f" % heading
    UDPServerSocket.sendto(str.encode(strReply), address)

def cmdAltitudeRetrieve(vehicle, UDPServerSocket, address):
    altitude = vehicle.location.global_relative_frame.alt
    strReply = "%.3f" % altitude
    UDPServerSocket.sendto(str.encode(strReply), address)

def cmdSpeedRetrieve(vehicle, UDPServerSocket, address):
    speed = vehicle.groundspeed
    strReply = "%.1f" % speed
    UDPServerSocket.sendto(str.encode(strReply), address)

def cmdObs(vehicle, element): 
    if 'dir' in element.attrib:
        print("dir found")
    current_time_us = np.uint64(time.time() * 1000000)
    depthMin_cm = 15
    depthMax_cm = 2000
    # d435i fov: 87 × 58
    degreeOffset = -45.0#-(58.0 / 2.0)
    degreeIncrement = 45#1.875#(58.0 / 72.0)
    distances = np.ones((72,), dtype=np.uint16) * (depthMax_cm + 1)
    distances[3:72] = 65535
    distances[0:3] = depthMax_cm + 1
    for dist in element.iter(tag='dist'):
        if dist.attrib['dir'] == 'left':
            distances[0] = int(dist.text)
        elif dist.attrib['dir'] == 'center':
            distances[1] = int(dist.text)
        elif dist.attrib['dir'] == 'right':
            distances[2] = int(dist.text)
        else:
            print("Invalid <obs><dist> dir attribute" + dist.attrib['dir'])

    vehicle._master.mav.obstacle_distance_send(
            current_time_us,    # us Timestamp (UNIX time or time since system boot)
            0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
            distances,          # distances,    uint16_t[72],   cm
            0,                  # increment,    uint8_t,        deg
            depthMin_cm,        # min_distance, uint16_t,       cm
            depthMax_cm,       # max_distance, uint16_t,       cm
            degreeIncrement,   # increment_f,  float,          deg
            degreeOffset,       # angle_offset, float,          deg
            12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
        )

    #vehicle._master.mav.distance_sensor_send(0, 30, 2000, 400, 0, 0, 0, 0)
    #distance_sensor_send(ignored,min_depth, max_depth, cur_dist, ignored, ignored, cameraFacingAngleDeg/45, ignored

def cmdRouteSet(vehicle, element):
    upload_missionStr(element.text, vehicle)
    if 'setauto' in element.attrib:
        if element.attrib['setauto'] == 'true':
            vehicle.mode = VehicleMode("AUTO")
            while not vehicle.mode.name=='AUTO':
                time.sleep(0.01)

def cmdRouteLoad(vehicle, element):
    missionName = element.text
    missionList = readmissionF(missionName, vehicle)
    upload_mission(missionList, vehicle)
    if 'setauto' in element.attrib:
        if element.attrib['setauto'] == 'true':
            vehicle.mode = VehicleMode("AUTO")
            while not vehicle.mode.name=='AUTO':
                time.sleep(0.01)

def cmdRouteRestore(vehicle, element):
    if 'loadsaved' in element.attrib:
        if element.attrib['loadsaved'] == 'true':
            missionList = readmissionF('savedMission.txt', vehicle)
    else:
        missionList = missionList = download_mission(vehicle, skipVisited=True)
        
    if 'lat' in element.attrib:
        fLat = float(element.attrib['lat'])
        fLon = float(element.attrib['lon'])
    else:
        fLat = vehicle.location.global_frame.lat
        fLon = vehicle.location.global_frame.lon
    if 'clean' in element.attrib:
        if element.attrib['clean'] == 'simple':
            missionList = mission_cleanSimple(missionList, vehicle.commands.next, fLat, fLon)
        elif element.attrib['clean'] == '2opt':
            missionList = mission_cleanBetter(missionList, vehicle.commands.next, fLat, fLon)

    upload_mission(missionList, vehicle)
    if 'setauto' in element.attrib:
        if element.attrib['setauto'] == 'true':
            vehicle.mode = VehicleMode("AUTO")
            while not vehicle.mode.name=='AUTO':
                time.sleep(0.01)

def cmdRoutRetrieve(vehicle, UDPServerSocket, address):
    missionList = download_mission(vehicle)
    missionStr = mission_to_str(missionList)
    UDPServerSocket.sendto(str.encode(missionStr), address)

def cmdPosRetrieve(vehicle, UDPServerSocket, address):
    strReply = "%f\n%f\n" % (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    print(strReply)
    UDPServerSocket.sendto(str.encode(strReply), address)

def cmdBatteryRetrieve(vehicle, UDPServerSocket, address):
    strReply = "%d\n" % (vehicle.battery.level)
    UDPServerSocket.sendto(str.encode(strReply), address)

def handleCommand(element, vehicle, UDPServerSocket, address):
    global visitedList
    if args.livetest == 'True':
        if element.tag == 'obs':
            cmdObs(vehicle, element)
        else:
            print("Testing Cmd: " % element.tag)
        return
    
    #if element.tag == 'clean_dist':
        #if args.livetest == True:
            #return
        #NOT_IMPLEMENTED

    if element.tag == 'battery_retrieve':
        cmdBatteryRetrieve(vehicle, UDPServerSocket, address)
        
    elif element.tag == 'takeoff':
        if args.livetest == True:
            return
        cmdTakeoff(vehicle, element)

    elif element.tag == 'dir':
        if args.livetest == True:
            return
        cmdDir(vehicle, element)
        #Reset the visited point list so we only have points visited while in Guided mode

    elif element.tag == 'mode_retrieve':
        cmdModeRetrieve(vehicle, UDPServerSocket, address)

    elif element.tag == 'heading_retrieve':
        cmdDirRetrieve(vehicle, UDPServerSocket, address)

    elif element.tag == 'obs':
        cmdObs(vehicle, element)

    elif element.tag == 'route_save':
        if args.livetest == True:
            return
        save_missionF('savedMission.txt', vehicle)

    elif element.tag == 'route_retrieve':
        if args.livetest == True:
            return
        cmdRoutRetrieve(vehicle, UDPServerSocket, address)

    elif element.tag == 'route_set':
        if args.livetest == True:
            return
        cmdRouteSet(vehicle, element)

    elif element.tag == 'route_load':
        if args.livetest == True:
            return
        cmdRouteLoad(vehicle, element)

    elif element.tag == 'pos_retrieve':
        cmdPosRetrieve(vehicle, UDPServerSocket, address)
        visitedList.append([vehicle.location.global_frame.lat, vehicle.location.global_frame.lon])

    elif element.tag == 'altitude_retrieve':
        cmdAltitudeRetrieve(vehicle, UDPServerSocket, address)

    #elif element.tag == 'altitude_set':
        #need to read through the whole route and set each one

    elif element.tag == 'rtl':
        if args.livetest == True:
            return
        vehicle.mode = VehicleMode("RTL")

    elif element.tag == 'speed_retrieve':
        cmdSpeedRetrieve(vehicle, UDPServerSocket, address)
        
    elif element.tag == 'route_restore':
        if args.livetest == True:
            return
        cmdRouteRestore(vehicle, element)
        visitedList = []

    else:
        print("Unknown command")
        print(element.tag)

def arm_and_takeoff(vehicle, aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def recvUDP(sock, vehicle):
    try:
        message,address = sock.recvfrom(1024)
        parseXML(message, vehicle, sock, address)
    except:
        print('recv failure')

# Set up connection string to the drone
connection_string = args.connect
sitl = None
#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, timeout=360)
# Check that vehicle is armable. 
# This ensures home_location is set (needed when saving WP file)
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

selectorDefault = selectors.DefaultSelector()
# Create a datagram socket
sockUDP = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
sockUDP.bind(("192.168.2.103", args.port))
sockUDP.setblocking(False)
selectorDefault.register(sockUDP, selectors.EVENT_READ, recvUDP)

print("UDP server up and listening")
try:
    while True:
        events = selectorDefault.select(timeout=None)
        for key, mask in events:
            key.data(key.fileobj, vehicle)
except KeyboardInterrupt:
    print("caught keyboard interrupt, exiting")

selectorDefault.close()
print("Close vehicle object")
vehicle.close()
# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

# %%
