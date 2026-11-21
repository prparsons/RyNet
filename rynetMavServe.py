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
import hashlib
import hmac
import numpy as np
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import datetime
import struct

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
    ***Currently prepends message with a little endian uint32_t of the whole message size
    --livetest enabled

<route_load setauto='true'>MISSION_FILENAME</route_load>
    Loads a waypoints file as the current mission
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

<waypoint_current_retrieve />
    Gets the next waypoint


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
parser.add_argument('--udp_hmac', type=str, default='', help='If specified, all udp commands will be verified. String key should be provided.')
args = parser.parse_args()
#Testing without command line, use below
#args = parser.parse_args("--port 3636 --connect /dev/ttyTHS0 --livetest".split())#  --livetest --udp_hmac SecretKey
visitedList = []

def hmacVerifyMessage(signature, bytesSalt, bytesMessage, key):
    try: hmacVerifyMessage.salt
    except AttributeError: hmacVerifyMessage.salt = -1

    intSalt = int.from_bytes(bytesSalt, "little")
    if intSalt <= hmacVerifyMessage.salt:
        print("salt reuse; Current: %d, received: %d" % (hmacVerifyMessage.salt, intSalt))
        return False
        
    h = hmac.new(key.encode(), bytesSalt + bytesMessage, hashlib.sha256)
    retValue = h.digest() == signature
    if retValue == True:
        hmacVerifyMessage.salt = intSalt
    return retValue

def hmacCreateResponse(stringMessage, key, saltBytes):
    #this needs to grab the salt from hmacVerifyMessage
    h = hmac.new(key.encode(), saltBytes + stringMessage.encode(), hashlib.sha256)
    # print("Creation Digest: " + h.hexdigest())
    return h.digest() + saltBytes + str.encode(stringMessage)

def hmacCreateResponseBytes(bytesMessage, key, saltBytes):
    #this needs to grab the salt from hmacVerifyMessage
    outputBytes = saltBytes + bytesMessage
    h = hmac.new(key.encode(), outputBytes, hashlib.sha256)
    # print("Creation Digest: " + h.hexdigest())
    # print("Size bytes: ", len(outputBytes))
    return h.digest() + outputBytes
    
def sendReply(UDPServerSocket, address, strReply, saltBytes):
    # print(" Using Salt:%d\nReply:%s\n" % (int.from_bytes(saltBytes, 'little'), strReply))
    strReplyBytes = hmacCreateResponse(strReply, args.udp_hmac, saltBytes)
    strlen = len(strReplyBytes) + 4
    UDPServerSocket.sendto(strlen.to_bytes(4, 'little') + strReplyBytes, address)
    
def sendReplyBytes(UDPServerSocket, address, bytesReply, saltBytes):
    # print(" Using Salt:%d\nReply:%s\n" % (int.from_bytes(saltBytes, 'little'), strReply))
    strReplyBytes = hmacCreateResponseBytes(bytesReply, args.udp_hmac, saltBytes)
    strlen = len(strReplyBytes) + 4
    UDPServerSocket.sendto(strlen.to_bytes(4, 'little') + strReplyBytes, address)

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
    print(" Upload mission from a list[%d]" % len(missionlist))
    cmds = vehicle.commands
    cmds.clear()
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

# Download current mission and return it in a list
def download_mission(vehicle, skipVisited=False):
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print(" Download mission from vehicle[%d]" % len(cmds))
    #from docs, https://dronekit-python.readthedocs.io/en/stable/automodule.html, cmds.next will return the index of current waypoint
    if skipVisited:
        missionlist = cmds[cmds.next:]
    else:
        missionlist = cmds[:]
    print(" Downloaded %d" % len(missionlist))
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

def mission_to_hex(missionList):
    home = vehicle.home_location
    homeList = [0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1]
    outputBytes = b''
    
    for cmd in missionList:
        #outputBytes += int(cmd.target_system).to_bytes(4, 'little')
        #outputBytes += int(cmd.target_component).to_bytes(4, 'little')
        outputBytes += int(cmd.seq).to_bytes(2, 'little')
        outputBytes += int(cmd.current).to_bytes(2, 'little')
        outputBytes += int(cmd.frame).to_bytes(2, 'little')
        outputBytes += int(cmd.command).to_bytes(2, 'little')
        #outputBytes += int(cmd.current).to_bytes(4, 'little')
        #outputBytes += int(cmd.autocontinue).to_bytes(4, 'little')
        outputBytes += struct.pack("f", float(cmd.param1))
        outputBytes += struct.pack("f", float(cmd.param2))
        outputBytes += struct.pack("f", float(cmd.param3))
        outputBytes += struct.pack("f", float(cmd.param4))
        outputBytes += struct.pack("f", float(cmd.x))
        outputBytes += struct.pack("f", float(cmd.y))
        outputBytes += struct.pack("f", float(cmd.z))
        outputBytes += int(cmd.autocontinue).to_bytes(4, 'little')
        #outputBytes += int(cmd.target_system).to_bytes(4, 'little')

    return outputBytes

def parseXML(message, vehicle, UDPServerSocket, address):
    saltBytes = message[:4]
    message = message[4:]
    try:
        root = ET.fromstring(message)
        #print(root)
        handleCommand(root, vehicle, UDPServerSocket, address, saltBytes)
    except:
        print("parseXML error")
    return

def cmdTakeoff(vehicle, element):
    arm_and_takeoff(vehicle, float(element.text))
    time.sleep(3)

    if 'setauto' in element.attrib:
        if element.attrib['setauto'] == 'true':
            vehicle.mode = VehicleMode("AUTO")
            while not vehicle.mode.name =='AUTO':
                time.sleep(0.01)

def cmdWaypointRetrieve(vehicle, UDPServerSocket, address, saltBytes):
    cmds = vehicle.commands
    print(cmds.next)
    strReply = "%d" % cmds.next
    print(strReply)
    sendReply(UDPServerSocket, address, strReply, saltBytes)

def cmdDir(vehicle, element):
    ptCurrent = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    distanceM = 1000
    if 'dist' in element.attrib:
        distanceM = float(element.attrib['dist'])
    dirDeg = float(element.text) + vehicle.heading;
    dirRad = dirDeg * 3.14159 / 180;
    latNew, lonNew = hs.inverse_haversine(ptCurrent, distanceM*.001, dirRad)
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name =='GUIDED':
        time.sleep(0.01)
    locationNew = LocationGlobal(latNew, lonNew, vehicle.location.global_frame.alt)
    vehicle.simple_goto(locationNew)

def cmdModeRetrieve(vehicle, UDPServerSocket, address, saltBytes):
    strReply = vehicle.mode.name
    sendReply(UDPServerSocket, address, strReply, saltBytes)

def cmdDirRetrieve(vehicle, UDPServerSocket, address, saltBytes):
    heading = vehicle.heading
    strReply = "%.1f" % heading
    sendReply(UDPServerSocket, address, strReply, saltBytes)

def cmdAltitudeRetrieve(vehicle, UDPServerSocket, address, saltBytes):
    altitude = vehicle.location.global_relative_frame.alt
    strReply = "%.3f" % altitude
    sendReply(UDPServerSocket, address, strReply, saltBytes)

def cmdSpeedRetrieve(vehicle, UDPServerSocket, address, saltBytes):
    speed = vehicle.groundspeed
    strReply = "%.1f" % speed
    sendReply(UDPServerSocket, address, strReply, saltBytes)

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
    msg = mavlink2.MAVLink_obstacle_distance_message(
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
    time.sleep(0.01)
    vehicle.send_mavlink(msg) # _master.mav.send(msg) alternate way of sending?

def cmdRouteSet(vehicle, element):
    upload_missionStr(element.text, vehicle)
    if 'setauto' in element.attrib:
        if element.attrib['setauto'] == 'true':
            vehicle.mode = VehicleMode("AUTO")
            while not vehicle.mode.name =='AUTO':
                time.sleep(0.01)

def cmdRouteLoad(vehicle, element):
    missionName = element.text
    missionList = readmissionF(missionName, vehicle)
    upload_mission(missionList, vehicle)
    if 'setauto' in element.attrib:
        if element.attrib['setauto'] == 'true':
            vehicle.mode = VehicleMode("AUTO")
            while not vehicle.mode.name =='AUTO':
                time.sleep(0.01)

def cmdRoutRetrieve(vehicle, UDPServerSocket, address, saltBytes):
    missionList = download_mission(vehicle)
    #missionStr = mission_to_str(missionList)
    #print("MissionHex: %d" % len(mission_to_hex(missionList)))
    #sendReply(UDPServerSocket, address, missionStr, saltBytes)
    sendReplyBytes(UDPServerSocket, address, mission_to_hex(missionList), saltBytes)

def cmdPosRetrieve(vehicle, UDPServerSocket, address, saltBytes):
    strReply = "%f\n%f\n" % (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    sendReply(UDPServerSocket, address, strReply, saltBytes)

def cmdBatteryRetrieve(vehicle, UDPServerSocket, address, saltBytes):
    strReply = "%d\n" % (vehicle.battery.level)
    sendReply(UDPServerSocket, address, strReply, saltBytes)

def handleCommand(element, vehicle, UDPServerSocket, address, saltBytes):
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
        cmdBatteryRetrieve(vehicle, UDPServerSocket, address, saltBytes)
        
    elif element.tag == 'takeoff':
        if args.livetest == True:
            return
        if vehicle.mode.name == 'RTL':
            print("Current mode is RTL, not setting direction");
            return
        cmdTakeoff(vehicle, element)
    
    elif element.tag == 'waypoint_current_retrieve':
        cmdWaypointRetrieve(vehicle, UDPServerSocket, address, saltBytes)

    elif element.tag == 'dir':
        if args.livetest == True:
            return
        if vehicle.mode.name == 'RTL':
            print("Current mode is RTL, not setting direction");
            return
        cmdDir(vehicle, element)
        #Reset the visited point list so we only have points visited while in Guided mode

    elif element.tag == 'mode_retrieve':
        cmdModeRetrieve(vehicle, UDPServerSocket, address, saltBytes)

    elif element.tag == 'heading_retrieve':
        cmdDirRetrieve(vehicle, UDPServerSocket, address, saltBytes)

    elif element.tag == 'obs':
        cmdObs(vehicle, element)

    elif element.tag == 'route_save':
        if args.livetest == True:
            return
        save_missionF('savedMission.txt', vehicle)

    elif element.tag == 'route_retrieve':
        if args.livetest == True:
            return
        cmdRoutRetrieve(vehicle, UDPServerSocket, address, saltBytes)

    elif element.tag == 'route_set':
        if args.livetest == True:
            return
        if vehicle.mode.name =='RTL':
            print("Current mode is RTL, not setting route");
            return
        cmdRouteSet(vehicle, element)

    elif element.tag == 'route_load':
        if args.livetest == True:
            return
        if vehicle.mode.name =='RTL':
            print("Current mode is RTL, not setting route file");
            return
        cmdRouteLoad(vehicle, element)

    elif element.tag == 'pos_retrieve':
        cmdPosRetrieve(vehicle, UDPServerSocket, address, saltBytes)
        visitedList.append([vehicle.location.global_frame.lat, vehicle.location.global_frame.lon])

    elif element.tag == 'altitude_retrieve':
        cmdAltitudeRetrieve(vehicle, UDPServerSocket, address, saltBytes)

    #elif element.tag == 'altitude_set':
        #need to read through the whole route and set each one

    elif element.tag == 'rtl':
        if args.livetest == True:
            return
        vehicle.mode = VehicleMode("RTL")

    elif element.tag == 'speed_retrieve':
        cmdSpeedRetrieve(vehicle, UDPServerSocket, address, saltBytes)

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
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

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
        message,address = sock.recvfrom(8192)
        if args.udp_hmac != '':
            if len(message) < 32:
                print('message verification, too short failure')
                print(message)
                return
            if hmacVerifyMessage(message[:32], message[32:36], message[36:], args.udp_hmac) == False:
                print('message verification failure')
                print(message[36:])
                return
            parseXML(message[32:], vehicle, sock, address)
        else:
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
vehicle = connect(connection_string, wait_ready=True, timeout=360, baud=57600)
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
    print("caught keyboard interrupt, exiting: %d" % hmacVerifyMessage.salt)

selectorDefault.close()
print("Close vehicle object")
vehicle.close()
# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

# %%
