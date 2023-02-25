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

"""
Command Documentation:

<dir dist='200'>OFFSET_HEADING_DEGREES</dir>
    Sets mode to GUIDED, and then turns indicated amount
        dist -- distance to set waypoint in meters[default:1000]

<obs dir='30'>DISTANCE_CM</obs>
    Sends obstacle distance sensor message
        dir -- direction the sensor is facing

<pos_retrieve />
    Gets the current position (lat,lon) and returns it back to the client separated by \n

<route_load setauto='true'>MISSION_FILENAME</route_load>
    Loads a waypoints file as the current mission
        setauto -- if present and true, will set the autopilot's mode to auto after setting the route

<route_restore loadsaved='true' lat='35.1' lon='-110.1' clean='simple' setauto='true' />
    Restores a route, optionally cleaning it and setting vehicle mode to AUTO
        loadsave -- if present and true, will load route from savedMission.txt[default:current mission]
        lat,lon -- if present, will be used as the current location in cleaning routine
        clean -- [simple] will find closest waypoint and remove everything prior to it
        setauto -- if present and true, will set the autopilot's mode to auto after setting the route

<route_retrieve />
    Gets current route and returns it back to client as a string

<route_save />
    Saves current route to savedMission.txt

<route_set setauto='true'>ROUTE</route_set>
    Sets the text of element as the current route
        setauto -- if present and true, will set the autopilot's mode to auto after setting the route

<rtl />
    Sets vehicle mode to RTL

<takeoff setauto='true'>ALTITUDE_METERS</takeoff>
    Will arm and take off (simple example from dronekit)
        setauto -- will change the vehicle mode to AUTO 3 sec after takeoff


SITL Testing instructions:
https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html

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
parser.add_argument('--livetest', default='False', help='If True, will only send <obs /> messages to the drone.')
args = parser.parse_args()

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
def download_mission(vehicle):
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
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
        fDist = hs.haversine((fLat, fLon), (cmd.x, cmd.y)) * 0.00001 #returns km, we want cm
        if fDist < fDistClosest:
            fDistClosest = fDist
            missionListClean = []
        missionListClean.append(cmd)
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

def parseXML(message, vehicle, UDPServerSocket, address):
    parser = ET.XMLPullParser()
    parser.feed(message)
    
    try:
        for event, elem in parser.read_events():
            print(elem)
            handleCommand(elem, vehicle, UDPServerSocket, address)  
        parser.close()
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

def cmdObs(vehicle, element): 
    if 'dir' in element.attrib:
        print("dir found")
    vehicle._master.mav.distance_sensor_send(0, 30, 2000, 400, 0, 0, 0, 0)
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
        missionList = missionList = download_mission(vehicle)
        
    if 'lat' in element.attrib:
        fLat = float(element.attrib['lat'])
        fLon = float(element.attrib['lon'])
    else:
        fLat = vehicle.location.global_frame.lat
        fLon = vehicle.location.global_frame.lon
    if 'clean' in element.attrib:
        if element.attrib['clean'] == 'simple':
            missionList = mission_cleanStart(missionList, vehicle.commands.next, fLat, fLon)
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
    UDPServerSocket.sendto(str.encode(strReply), address)

def handleCommand(element, vehicle, UDPServerSocket, address):
    if args.livetest == 'True':
        if element.tag == 'obs':
            cmdObs(vehicle, element)
        else:
            print("Testing Cmd: " % element.tag)
        return
    
    if element.tag == 'takeoff':
        cmdTakeoff(vehicle, element)

    elif element.tag == 'dir':
        cmdDir(vehicle, element)

    elif element.tag == 'obs':
        cmdObs(vehicle, element)

    elif element.tag == 'route_save':
        save_missionF('savedMission.txt', vehicle)

    elif element.tag == 'route_retrieve':
        cmdRoutRetrieve(vehicle, UDPServerSocket, address)

    elif element.tag == 'route_set':
        cmdRouteSet(vehicle, element)

    elif element.tag == 'route_load':
        cmdRouteLoad(vehicle, element)

    elif element.tag == 'pos_retrieve':
        cmdPosRetrieve(vehicle, UDPServerSocket, address)

    #elif element.tag == 'altitude_set':
        #need to read through the whole route and set each one

    elif element.tag == 'rtl':
        vehicle.mode = VehicleMode("RTL")
        
    elif element.tag == 'route_restore':
        cmdRouteRestore(vehicle, element)

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
vehicle = connect(connection_string, wait_ready=True)
# Check that vehicle is armable. 
# This ensures home_location is set (needed when saving WP file)
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

selectorDefault = selectors.DefaultSelector()
# Create a datagram socket
sockUDP = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
sockUDP.bind(("127.0.0.1", args.port))
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
