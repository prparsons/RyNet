#!/usr/bin/env python3
import socket
import xml.etree.ElementTree as ET
from datetime import datetime
   
#Open log file
fileLog = open("logUDP.txt", "w")

try:
    # Now set up everything for the UDP server
    localIP = "192.168.2.40"#"127.0.0.1"
    localPort = 3636
    #bytesToSend = str.encode("ack")
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
        
        # Sending a reply to client
        #UDPServerSocket.sendto(bytesToSend, address)
except:
    print("done")
    fileLog.close()
