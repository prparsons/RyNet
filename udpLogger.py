#!/usr/bin/env python3
import socket
import xml.etree.ElementTree as ET
from datetime import datetime
import hashlib
import hmac
   

def hmacVerifyMessage(signature, bytesSalt, bytesMessage, key):
    try: hmacVerifyMessage.salt
    except AttributeError: hmacVerifyMessage.salt = -1

    intSalt = int.from_bytes(bytesSalt, "big")
    print("Verification Salt %d" % intSalt)
    if intSalt <= hmacVerifyMessage.salt:
        print("salt reuse")
        return False
        
    h = hmac.new(key.encode(), bytesSalt + bytesMessage, hashlib.sha256)
    print("Verification Digest: " + h.hexdigest())
    retValue = h.digest() == signature
    if retValue == True:
        hmacVerifyMessage.salt = intSalt
    return retValue

def hmacCreateMessage(stringMessage, key):
    try: hmacCreateMessage.salt
    except AttributeError: hmacCreateMessage.salt = -1
        
    hmacCreateMessage.salt += 1
    print("Creation Salt: %d" % hmacCreateMessage.salt)
    h = hmac.new(key.encode(), hmacCreateMessage.salt.to_bytes(4, 'big') + stringMessage.encode(), hashlib.sha256)
    print("Creation Digest: " + h.hexdigest())
    return h.digest() + hmacCreateMessage.salt.to_bytes(4, 'big') + str.encode(stringMessage)

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

        if len(message) < 32:
            print('message verification, too short failure')
            continue
        if hmacVerifyMessage(message[:32], message[32:36], message[36:], args.udp_hmac) == False:
            print('message verification failure')
            continue

        message = message[36:].decode('ascii')
        
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
