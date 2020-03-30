
// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

// include for repeated input
#include <ncurses.h>

// Tells us that the network is running.
static volatile int networkActive=0;
static volatile int ready_flag = 1;
void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			ready_flag = 1; //avoid commands sent when system not ready
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", data[0]);
	printf("Right Forward Ticks:\t\t%d\n", data[1]);
	printf("Left Reverse Ticks:\t\t%d\n", data[2]);
	printf("Right Reverse Ticks:\t\t%d\n", data[3]);
	printf("Left Forward Ticks Turns:\t%d\n", data[4]);
	printf("Right Forward Ticks Turns:\t%d\n", data[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printf("Forward Distance:\t\t%d\n", data[8]);
	printf("Reverse Distance:\t\t%d\n", data[9]);
	printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}

void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		/* (DONE) TODO: Insert SSL write here to write buffer to network */

		c = sslWrite(conn, buffer, len);

		if (c < 0) {
			perror("Error writing to server: ");
		}

		/* END TODO */	
		networkActive = (c > 0);
	}
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		/* (DONE) TODO: Insert SSL read here into buffer */
		len = sslRead(conn, buffer, sizeof(buffer));

        printf("read %d bytes from server.\n", len);
		
		/* END TODO */

		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
    
    /* (DONE) TODO: Stop the client loop and call EXIT_THREAD */
	stopClient();
	EXIT_THREAD(conn);
    /* END TODO */
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(int32_t *params)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &params[0], &params[1]);
	flushInput();
}

void *writerThread(void *conn)
{
	int quit=0;
	int stop_flag = 0; //indicate when machine has stopped
	while(!quit)
	{
		char ch;
		ch = getch();
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, c=clear stats, g=get stats q=exit)\n");
		//scanf("%c", &ch);

		// Purge extraneous characters from input stream
		//flushInput();
		
		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		if(ready_flag)
		{
			switch(ch)
			{
				case 'w':
				case 'W':
					buffer[1] = 'f';
					params[0] = 5;
					params[1] = 100;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
					ready_flag = 0;
					stop_flag = 0;
					break;
				case 's':
				case 'S':
					buffer[1] = 'b';
					params[0] = 5;
					params[1] = 100;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
					ready_flag = 0;
					stop_flag = 0;
					break;	
				case 'a':
				case 'A':
					buffer[1] = 'l';
					params[0] = 20;
					params[1] = 100;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
					ready_flag = 0;
					stop_flag = 0;
					break;
				case 'd':
				case 'D':
							//getParams(params);
					params[0] = 20;
					params[1] = 100;
					buffer[1] = 'r';
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
					ready_flag = 0;
					stop_flag = 0;
					break;
				case 'c':
				case 'C':
				case 'g':
				case 'G':
						params[0]=0;
						params[1]=0;
						memcpy(&buffer[2], params, sizeof(params));
						buffer[1] = ch;
						sendData(conn, buffer, sizeof(buffer));
						ready_flag = 0;
						break;
				case 'q':
				case 'Q':
					quit=1;
					break;
				case 'b':
				case 'B':
				default:
					if (stop_flag == 0) 
					{
						params[0] = 0;
						params[1] = 0;
						memcpy(&buffer[2], params, sizeof(params));
						buffer[1] = 's';
						sendData(conn, buffer, sizeof(buffer));
						ready_flag = 0;
						stop_flag = 1;
						printf("BAD COMMAND\n");
					}
					break;
			}
		}
	}

	printf("Exiting keyboard thread\n");

    /* (DONE) TODO: Stop the client loop and call EXIT_THREAD */
	stopClient();
	EXIT_THREAD(conn);
    /* END TODO */
}

/* (DONE) TODO: #define filenames for the client private key, certificatea,
   CA filename, etc. that you need to create a client */

//#define SERVER_NAME "192.168.137.93"
#define CA_CERT_FNAME "signing.pem"
//#define PORT_NUM 5000
#define CLIENT_CERT_FNAME "laptop.crt"
#define CLIENT_KEY_FNAME "laptop.key"
#define SERVER_NAME_ON_CERT "Alex Pi"

/* END TODO */
void connectToServer(const char *serverName, int portNum)
{
    /* (DONE) TODO: Create a new client */
	
	//createClient(SERVER_NAME, PORT_NUM, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);
	createClient(serverName, portNum, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);

    /* END TODO */
}

int main(int ac, char **av)
{
	initscr();
	cbreak();
	noecho();
	halfdelay(3);
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

    /* (DONE) TODO: Add in while loop to prevent main from exiting while the
    client loop is running */

	while (client_is_running());

    /* END TODO */
	printf("\nMAIN exiting\n\n");
	endwin();
}
