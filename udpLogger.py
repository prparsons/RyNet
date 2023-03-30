#!/usr/bin/env python3
import socket
import xml.etree.ElementTree as ET
from datetime import datetime
   
#Open log file
fileLog = open("logUDP.txt", "w")
lat = 35.0
lon = 110.0

try:
    # Now set up everything for the UDP server
    localIP = "127.0.0.1"
    localPort = 3636
    # Create a datagram socket
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # Bind to address and ip
    UDPServerSocket.bind((localIP, localPort))
    print("UDP server up and listening")

    # Main loop
    # Listen for incoming datagrams
    while(True):
        message,address = UDPServerSocket.recvfrom(1024)
        message = message.decode('ascii')
        
        current_time = datetime.now()
        time_stamp = current_time.timestamp()
        date_time = datetime.fromtimestamp(time_stamp)
        
        strMessStamped = "[%s]  %s\n" % (date_time, message)
        print(message)
        fileLog.write(strMessStamped)
        
        if message == "<pos_retrieve />":
            strReply = "%f\n%f\n" % (lat, lon)
            lat += 0.001
            lon += 0.001
            UDPServerSocket.sendto(str.encode(strReply), address)
except:
    print("done")
    fileLog.close()
