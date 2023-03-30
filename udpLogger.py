#!/usr/bin/env python3
import socket
import xml.etree.ElementTree as ET
from datetime import datetime
   
#Open log file
fileLog = open("logUDP.txt", "w")

try:
    # Now set up everything for the UDP server
    localIP = "127.0.0.1" #need to use argument or something...needs to spell out actual IP to listen to other addresses
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

except:
    print("done")
    fileLog.close()
