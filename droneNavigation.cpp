#include "droneNavigation.h"

#include <stdio.h>


bool sendXML(const char* strMessage)
{
	int socket_desc;
	struct sockaddr_in server;
	
	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1)
	{
		printf("Could not create socket");
	}
		
	server.sin_addr.s_addr = inet_addr(HOST);
	server.sin_family = AF_INET;
	server.sin_port = htons( PORT );

	//Connect to remote server
	if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		puts("connect error");
		return 1;
	}
	
	puts("Connected\n");
	
	//Send some data
	if( send(socket_desc , strMessage , strlen(strMessage) , 0) < 0)
	{
		puts("Send failed");
		return false;
	}
	puts("Data Send\n");
	
	return true;
}