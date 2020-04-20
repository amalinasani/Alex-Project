
// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

// include for repeated input
#include <ncurses.h>

//indicates which movement state the machine is in
enum state
{
	FORWARD = 1,
	BACKWARD = 2,
	LEFT = 3,
	RIGHT = 4,
	STOP = 5,
}; 

//distance and turning counters
static volatile int forward_dist = 0;
static volatile int backward_dist = 0;
static volatile int left_dist = 0;
static volatile int right_dist = 0;



//limits, machine cannot go too fast or too slow
const int velocity_high = 100;
const int velocity_low = 35;
const int angle_high = 45;
const int angle_low = 0;
const int step_high = 15;
const int step_low = 0;

//initial settings for the machine
static volatile int velocity = 75;
static volatile int angle = 20;
static volatile int step = 5;

// Tells us that the network is running.
static volatile int networkActive=0;
static volatile int ready_flag = 1;
static volatile bool dist_process = false;

void reset_count()
{
	forward_dist = 0;
	backward_dist = 0;
	left_dist = 0;
	right_dist = 0;
}

void infinite_report(char *buffer, int32_t *params)
{
	//first, send out the command to retrieve how far each of the items has travelled
	//once you send out the get status command and processed the status, we can then proceed
	buffer[1] = 'g';
	params[0] = 0;
	params[1] = 0;
	memcpy(&buffer[2], params, sizeof(params));

	
}

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			
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
	//when the getstatus command is sent, instead of printing it out, 
	//update the current set of variables


	forward_dist += data[8];
	backward_dist += data[9];
	left_dist += data[10];
	right_dist += data[11];
	dist_process = true;
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

	if(networkActive)
	{


		c = sslWrite(conn, buffer, len);

		if (c < 0) {
			perror("Error writing to server: ");
		}


		networkActive = (c > 0);
	}
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{

		len = sslRead(conn, buffer, sizeof(buffer));



		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
    

	stopClient();
	EXIT_THREAD(conn);

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

void print_instructions(WINDOW* win) //printing commands for instruction pannel
{
	wmove(win, 1, 1);
	wprintw(win, " Use the WASD keys to control the \n movement of your machine\n");
	wprintw(win, " Use the arrow keys to change machine\n settings\n");
	wprintw(win, " A value of inf means the machine will\n continue to move/turn as long as \n the key is held down\n");
	wprintw(win, " W - step forward\n");
	wprintw(win, " S - step backward\n");
	wprintw(win, " A - turn left\n");
	wprintw(win, " D - turn right\n");
	wprintw(win, " UP - increase speed\n");
	wprintw(win, " DOWN - decrease speed\n");
	wprintw(win, " LEFT - reduce degree\n");
	wprintw(win, " RIGHT - increase degree\n");
	wprintw(win, " + - increase step dist\n");
	wprintw(win, " - - decrease step dist\n");
	wprintw(win, " Q - Quit program\n");
	wborder(win, '|', '|', '=', '=', '@', '@', '@', '@');
	wmove(win, 0,0);
	wprintw(win, "INSTRUCTIONS");

}

void update_state(WINDOW *win, enum state now) //print out current machine state
{
	wclear(win);

	wmove(win, 0, 0);

	wmove(win, 1, 1);
	
	switch(now)
	{
		case FORWARD:
			wprintw(win, "FORWARD");
			break;
		case BACKWARD:
			wprintw(win, "BACKWARD");
			break;
		case LEFT:
			wprintw(win, "TURN LEFT");
			break;
		case RIGHT:
			wprintw(win, "TURN RIGHT");
			break;
		case STOP:
			wprintw(win, "STOP");
			break;
	}
	wborder(win, '|', '|', '=', '=', '@', '@', '@', '@');
	wmove(win, 0,0);
	wprintw(win, "MACHINE STATE");
}

void update_settings(WINDOW* win) //prints current vehicle settings on a panel
{
	wclear(win);

	

	wmove(win, 1, 1);
	
	if (angle == 0)
	{
		wprintw(win, "ANGLE: inf deg\n");
	}
	else
	{
		wprintw(win, "ANGLE: %d deg\n", angle);
	}

	wprintw(win, " SPEED: %d %% \n", velocity);
	if (step == 0)
	{
		wprintw(win, " STEP: inf cm\n");
	}
	else
	{
		wprintw(win, " STEP: %dcm\n", step);
	}
	wborder(win, '|', '|', '=', '=', '@', '@', '@', '@');
	wmove(win, 0,0);
	wprintw(win, "MACHINE SETTING");
}

void update_dist(enum state now, WINDOW* screen) //records down vehicle movements
{
	int y, x;	
	getyx(screen, y, x);
	wmove(screen, y, 0);
	wclrtoeol(screen);
	switch(now)
	{
		case FORWARD:
			wprintw(screen, "Forward: %d", forward_dist);
			break;
			
		case BACKWARD:
			wprintw(screen, "Backward: %d", backward_dist);
			break;
		case LEFT:
			wprintw(screen, "Turn Left: %d", left_dist);
			break;
		case RIGHT:
			wprintw(screen, "Turn Right: %d", right_dist);
			break;
	}
}

void *writerThread(void *conn)
{
	/*This section contains the variables necessary to create windows*/
	int h, w;
	int x, y; //cursor positioning
	getmaxyx(stdscr, h, w); //gets the size of the current screen
	int height_ins, width_ins;
	int height_log, width_log;
	int height_state, width_state;
	int height_set, width_set;
	//dimensions of windows
	height_ins = (3*h)/4;
	width_ins = w/2;
	height_log = h;
	width_log = w/2;
	height_state = 3;
	width_state = w/2;
	height_set = 5;
	width_set = w/2;

	//windows creation
	WINDOW* ins = newwin(height_ins, width_ins, 0, w/2);
	WINDOW* log = newwin(height_log, width_log, 0, 0);
	WINDOW* state = newwin(height_state, width_state, h-3, w/2);
	WINDOW* set = newwin(height_set, width_set, h-height_set-height_state, w/2);
	
	//allow for the log window to scroll when it is filled with data
	scrollok(log, TRUE);
	print_instructions(ins);
	//end of window setup
	

	enum state current;
	enum state prev;
	prev = STOP;
	current = STOP;
		
	//keeps track of the current and previous direction that we are facing	


	int quit=0;
	int stop_flag = 0; //indicate when machine has stopped
	bool change_dir = false; //indicates any direction change
	printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, c=clear stats, g=get stats q=exit)\n");
	while(!quit)
	{
		int ch;
		ch = getch();

		
		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		if(ready_flag)
		{
			change_dir = (current == STOP) ? true: false;
			wrefresh(log);
			switch(ch)
			{
				case KEY_LEFT:
					angle = (angle == angle_low) ? angle : angle - 5;
					break;
				case KEY_RIGHT:
					angle = (angle > angle_high) ? angle : angle + 5;
					break;
				case KEY_UP:
					velocity = (velocity == velocity_high) ? velocity : velocity + 5;
					break;
				case KEY_DOWN:
					velocity = (velocity < velocity_low) ? velocity : velocity - 5;
					break;
				case '-':
				case '_':
					step = (step == step_low) ? step : step - 1;
					break;
				case '+':
				case '=':
					step = (step == step_high) ? step : step + 1;
					break;
				case 'w':
				case 'W':
					current = FORWARD;
					if (prev != current)
					{
						reset_count();
						wprintw(log, "\n");
						prev = current;
						change_dir = true;
					}
					
					forward_dist += step;
					update_dist(current, log);
					buffer[1] = 'f';
					params[0] = step;
					params[1] = velocity;
					memcpy(&buffer[2], params, sizeof(params));
					if (step != 0 || change_dir == true) //infinity step no need to repeatedly send
					{					
						sendData(conn, buffer, sizeof(buffer));
						ready_flag = 0;
						stop_flag = 0;
					}

					break;
				case 's':
				case 'S':
					
					current = BACKWARD;
					if (prev != current)
					{
						reset_count();
						wprintw(log, "\n");
						prev = current;
						change_dir = true;
					}
					
					backward_dist += step;
					update_dist(current, log);
					buffer[1] = 'b';
					params[0] = step;
					params[1] = velocity;
					memcpy(&buffer[2], params, sizeof(params));
					if (step != 0 || change_dir == true) //infinity step no need to repeatedly send
					{					
						sendData(conn, buffer, sizeof(buffer));
						ready_flag = 0;
						stop_flag = 0;
					}

					break;	
				case 'a':
				case 'A':
					current = LEFT;
					if (prev != current)
					{
						reset_count();
						wprintw(log, "\n");
						prev = current;
						change_dir = true;
					}
					
					left_dist += angle;
					update_dist(current, log);
					buffer[1] = 'l';
					params[0] = angle;
					params[1] = velocity;
					memcpy(&buffer[2], params, sizeof(params));
					if (angle != 0 || change_dir == true) //infinity step no need to repeatedly send
					{					
						sendData(conn, buffer, sizeof(buffer));
						ready_flag = 0;
						stop_flag = 0;
					}

					break;
				case 'd':
				case 'D':
					current = RIGHT;
					if (prev != current)
					{
						reset_count();
						wprintw(log, "\n");
						prev = current;
						change_dir = true;
					}
					
					right_dist += angle;
					update_dist(current, log);
					params[0] = angle;
					params[1] = velocity;
					buffer[1] = 'r';
					memcpy(&buffer[2], params, sizeof(params));
					if (angle != 0 || change_dir == true) //infinity step no need to repeatedly send
					{					
						sendData(conn, buffer, sizeof(buffer));
						ready_flag = 0;
						stop_flag = 0;
					}

					break;

				case 'q':
				case 'Q':
					quit=1;
					break;
				
				case 'b':
				case 'B':
				default:
					//when stopped, can add to cumulative distance
					if (stop_flag == 0) 
					{
						halfdelay(3);
						current = STOP;
						change_dir = true;
						params[0] = 0;
						params[1] = 0;
						memcpy(&buffer[2], params, sizeof(params));
						buffer[1] = 's';
						sendData(conn, buffer, sizeof(buffer));
						ready_flag = 0;
						stop_flag = 1;
						if((step == 0 && (prev == FORWARD || prev == BACKWARD)) || (angle == 0 && (prev == LEFT || prev == RIGHT)))
						{
							//infinity turning or moving
							while(ready_flag == 0);
							infinite_report(buffer, params);
							sendData(conn, buffer, sizeof(buffer));
							dist_process = false;
							while (dist_process == false);
						}
					}
					break;
			}
		
			if (current == prev)
			{
				halfdelay(1); //reduce waiting time to improve responsiveness
			}
			if (step == 0 || angle == 0)//infinity, need to update one more time
			{
				update_dist(prev, log);
			}
		}
	//updates and refresh what is displayed on each panel 
	update_state(state, current);
	update_settings(set);
	wrefresh(log);
	wrefresh(ins);
	wrefresh(state);
	wrefresh(set);
	}

	printf("Exiting keyboard thread\n");


	stopClient();
	EXIT_THREAD(conn);

}


//#define SERVER_NAME "192.168.137.93"
#define CA_CERT_FNAME "signing.pem"
//#define PORT_NUM 5000
#define CLIENT_CERT_FNAME "laptop.crt"
#define CLIENT_KEY_FNAME "laptop.key"
#define SERVER_NAME_ON_CERT "Alex Pi"


void connectToServer(const char *serverName, int portNum)
{

	createClient(serverName, portNum, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);

}

int main(int ac, char **av)
{
	//initialisation of window creation code to allow for ncurses commands
	initscr();
	cbreak();
	noecho();
	halfdelay(3);
	keypad(stdscr, TRUE);// enable arrow key detection
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));



	while (client_is_running());

	printf("\nMAIN exiting\n\n");
	endwin();
}
