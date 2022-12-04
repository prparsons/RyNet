# %%

"""
#!/usr/bin/env python
"""

from __future__ import print_function
from dronekit import connect, Command, VehicleMode, LocationGlobal
import time
import socket
import xml.etree.ElementTree as ET
import haversine as hs

"""
#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates mission import/export from a file.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
"""

def readmissionF(aFileName, vehicle):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
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
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
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
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmissionF(aFileName, vehicle)
    
    print("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()


def upload_missionStr(strMission, vehicle):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(strMission, vehicle)
    
    print("\nUpload mission from a string")
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

def upload_mission(missionlist, vehicle):
    print("\nUpload mission from a list")
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

def download_mission(vehicle):
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def save_missionF(aFileName, vehicle):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)    
    #Download mission from vehicle
    missionlist = download_mission(vehicle)
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
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

def mission_cleanStart(missionList, nextWaypoint, fLat, fLon):
    missionListClean = []
    fDistClosest = 9999999999.0
    for cmd in missionList[nextWaypoint:]:
        fDist = hs.haversine((fLat, fLon), (cmd.x, cmd.y)) * 0.00001 #returns km, we want cm
        if fDist < fDistClosest:
            fDistClosest = fDist
            missionListClean = []
        missionListClean.append(cmd)
    return missionListClean

def parseXML(message, vehicle, UDPServerSocket, address):
    parser = ET.XMLPullParser()
    parser.feed(message)
    
    #try:
    for event, elem in parser.read_events():
        print(elem)
        handleCommand(elem, vehicle, UDPServerSocket, address)
        #for command in elem:
        #    handleCommand(command)
        
    parser.close()
    #except:
    #    print("XML Parsing Error")
    #    return
    
def handleCommand(element, vehicle, UDPServerSocket, address):
    if element.tag == 'test':
        print("<test>")
        if 'things' in element.attrib:
            print("things")

    elif element.tag == 'takeoff':
        arm_and_takeoff(vehicle, 100)
        time.sleep(3)
        
        if 'setauto' in element.attrib:
            if element.attrib['setauto'] == 'true':
                vehicle.mode = VehicleMode("AUTO")
                while not vehicle.mode.name=='AUTO':
                    time.sleep(0.01)
        UDPServerSocket.sendto(bytesToSend, address)

    elif element.tag == 'dir':
        ptCurrent = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
        distanceM = 1000
        if 'dist' in element.attrib:
            distanceM = float(element.attrib['dist'])
        dirDeg = float(element.text) + vehicle.heading;
        dirRad = dirDeg * 3.14159 / 180;
        latNew, lonNew = hs.inverse_haversine(ptCurrent, distanceM*.001, dirRad)
        #this is where we definitely need threads...
        #so all of dronekit is non guaranteed...we need to verify the changes requested have been made
        #now this especially, we're changing from AUTO to GUIDED
        vehicle.mode = VehicleMode("GUIDED")
        while not vehicle.mode.name=='GUIDED':
            time.sleep(0.01)
        locationNew = LocationGlobal(latNew, lonNew, vehicle.location.global_frame.alt)
        vehicle.simple_goto(locationNew)
        UDPServerSocket.sendto(bytesToSend, address)

    elif element.tag == 'obs':
        print("Obstacle distance")
        if 'dir' in element.attrib:
            print("dir found")
        vehicle._master.mav.distance_sensor_send(0, 30, 2000, 400, 0, 0, 0, 0)
        #distance_sensor_send(ignored,min_depth, max_depth, cur_dist, ignored, ignored, cameraFacingAngleDeg/45, ignored
        UDPServerSocket.sendto(bytesToSend, address)

    elif element.tag == 'route_save':
        save_missionF('savedMission.txt', vehicle)
        UDPServerSocket.sendto(bytesToSend, address)

    elif element.tag == 'route_retrieve':
        missionList = download_mission(vehicle)
        missionStr = mission_to_str(missionList)
        UDPServerSocket.sendto(str.encode(missionStr), address)

    elif element.tag == 'route_set':
        vehicle.mode = VehicleMode("AUTO")
        while not vehicle.mode.name=='AUTO':
            time.sleep(0.01)
        upload_missionStr(element.text, vehicle)
        UDPServerSocket.sendto(bytesToSend, address)

    elif element.tag == 'route_load':
        missionName = element.text
        missionList = readmissionF(missionName, vehicle)
        upload_mission(missionList, vehicle)
        if 'setauto' in element.attrib:
            if element.attrib['setauto'] == 'true':
                vehicle.mode = VehicleMode("AUTO")
                while not vehicle.mode.name=='AUTO':
                    time.sleep(0.01)
        UDPServerSocket.sendto(bytesToSend, address)

    elif element.tag == 'pos_retrieve':
        print("Position retrieve")
        strReply = "%f\n%f\n" % (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
        UDPServerSocket.sendto(str.encode(strReply), address)

    #elif element.tag == 'altitude_set':
        #need to read through the whole route and set each one, at least I'm pretty sure that's the case

    elif element.tag == 'rtl':
        vehicle.mode = VehicleMode("RTL")
        UDPServerSocket.sendto(bytesToSend, address)
        
    elif element.tag == 'route_restore':
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
                missionList = mission_cleanStart(missionList, 
                                                vehicle.commands.next, 
                                                fLat,
                                                fLon)
        upload_mission(missionList, vehicle)

        vehicle.mode = VehicleMode("AUTO")
        while not vehicle.mode.name=='AUTO':
            time.sleep(0.01)
        UDPServerSocket.sendto(bytesToSend, address)

    else:
        print("Unknown command")
        print(element.tag)
    
    for attribute in element.attrib:
        print(attribute)

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

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

# Set up connection string to the drone
connection_string = 'tcp:127.0.0.1:5762'#None#args.connect
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
#import_mission_filename = 'mpmission.txt'
#export_mission_filename = 'exportedmission.txt'

# Now set up everything for the UDP server
localIP = "127.0.0.1"
localPort = 3636
bytesToSend = str.encode("ack")
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")

# Main loop
# Listen for incoming datagrams
while(True):
    message,address = UDPServerSocket.recvfrom(1024)
    parseXML(message, vehicle, UDPServerSocket, address)
    # Sending a reply to client
    #UDPServerSocket.sendto(bytesToSend, address)

print("Close vehicle object")
vehicle.close()
# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()


# %%
