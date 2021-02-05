#pragma once

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>	//inet_addr

#define HOST "192.168.2.41"
//"192.168.2.15"
#define PORT 3636

bool sendXML(const char* strMessage);