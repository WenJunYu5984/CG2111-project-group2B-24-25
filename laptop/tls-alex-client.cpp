
// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

//TODO: add in libraries for checking keypresses
#include <unistd.h>
#include <termios.h>

// Tells us that the network is running.
static volatile int networkActive=0;

// Variables for different gears
int speed = 70;
int distance = 5;
int angle = 30;
int mode = 0;
int readyToReceive = 1; //1 if Arduino is ready to receive command

// Read a single character from terminal without a newline
// character. Done by modifyig terminal attributes.
char getKeypress() {
  char key = 0;
  struct termios term_state = {0};
  if (tcgetattr(0, &term_state) < 0)
    perror("tcsetattr()");
  term_state.c_lflag &= ~ICANON; //temporarily disable the canonical state
  term_state.c_lflag &= ~ECHO; //temporarily disable echo
  term_state.c_cc[VMIN] = 1;
  term_state.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &term_state) < 0)
    perror("tcsetattr ICANNON");
  if (read(0, &key, 1) < 0)
    perror("read()");
  term_state.c_lflag |= ICANON; //re-enable canonical state
  term_state.c_lflag |= ECHO; //re-enable echo
  if (tcsetattr(0, TCSADRAIN, &term_state) < 0)
    perror("tcsetattr ~ICANON");
  return (key);
}

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
		    readyToReceive = 1;
			printf("Command / Status OK\n");
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
  if (readyToReceive) {
    if (networkActive) {
      printf("\nSENDING %d BYTES DATA\n\n", len);
      c = sslWrite(conn, buffer, len);

      if (c < 0) {
        perror("Error writing to server: ");}

      networkActive = (c > 0);
      readyToReceive = 0; // set to 0 while waiting for Arduino to say OK
    }
  } else {
    printf("Arduino not ready\n");
  }
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		/* TODO: Insert SSL read here into buffer */
        len = sslRead(conn, buffer, sizeof(buffer));
        printf("read %d bytes from server.\n", len);
		/* END TODO */

		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
    
    /* TODO: Stop the client loop and call EXIT_THREAD */
    stopClient();
    /* END TODO */

    return NULL;
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

void writeWASD(void *conn, int *quit){
      printf("Keys: w=forward, s=reverse, d=turn left, a=turn right\n");
      printf("m = change mode, c=clear stats, g=get stats, i=get colour, o=open claw, p=close claw, u=deliver medpad, q=exit\n");
      char ch = getKeypress();
      char buffer[10];
      int32_t params[2];
      buffer[0] = NET_COMMAND_PACKET;

      switch (ch)
      {
        case 'w': // go forward
          buffer[1] = 'f';
          params[0] = distance;
          params[1] = speed;
          memcpy(&buffer[2], params, sizeof(params));
          sendData(conn, buffer, sizeof(buffer));
          break;
        case 's': // go backwards
          buffer[1] = 'b';
          params[0] = distance;
          params[1] = speed;
          memcpy(&buffer[2], params, sizeof(params));
          sendData(conn, buffer, sizeof(buffer));
          break;
        case 'a': // go left
          buffer[1] = 'l';
          params[0] = angle;
          params[1] = speed;
          memcpy(&buffer[2], params, sizeof(params));
          sendData(conn, buffer, sizeof(buffer));
          break;
        case 'd': // go right
          buffer[1] = 'r';
          params[0] = 25;
          params[1] = speed;
          memcpy(&buffer[2], params, sizeof(params));
          sendData(conn, buffer, sizeof(buffer));
          break;
        // todo: commands
        case 'c':
          buffer[1] = 'c';
          params[0] = 0;
          params[1] = 0;
          memcpy(&buffer[2], params, sizeof(params));
          buffer[1] = ch;
          sendData(conn, buffer, sizeof(buffer));
          break;
        case 'g':
          buffer[1] = 'g';
          params[0] = 0;
          params[1] = 0;
          memcpy(&buffer[2], params, sizeof(params));
          buffer[1] = ch;
          sendData(conn, buffer, sizeof(buffer));
          break;
        case 'i':
          buffer[1] = 'i';
          params[0] = 0;
          params[1] = 0;
          memcpy(&buffer[2], params, sizeof(params));
          buffer[1] = ch;
          sendData(conn, buffer, sizeof(buffer));
          break;
        case 'o':
          buffer[1] = 'o';
          params[0] = 0;
          params[1] = 0;
          memcpy(&buffer[2], params, sizeof(params));
          buffer[1] = ch;
          sendData(conn, buffer, sizeof(buffer));
          break;
        case 'p':
          buffer[1] = 'p';
          params[0] = 0;
          params[1] = 0;
          memcpy(&buffer[2], params, sizeof(params));
          buffer[1] = ch;
          sendData(conn, buffer, sizeof(buffer));
          break;
		case 'u':
          buffer[1] = 'u';
          params[0] = 0;
          params[1] = 0;
          memcpy(&buffer[2], params, sizeof(params));
          buffer[1] = ch;
          sendData(conn, buffer, sizeof(buffer));
          break;
    case 'k':
          buffer[1] = 'k';
          params[0] = 0;
          params[1] = 0;
          memcpy(&buffer[2], params, sizeof(params));
          buffer[1] = ch;
          sendData(conn, buffer, sizeof(buffer));
          break;
        case 'm':
          printf("Changing mode to TYPING COMMAND...\n");
          mode = 0;
          break;
        case 'q':
        case 'Q':
          *quit=1;
          break;
        default:
          printf("BAD COMMAND\n");
        }
}

void writeCommand(void *conn, int *quit){
  char ch;
  printf("Commands: f=forward, b=reverse, l=turn left, r=turn right, s=stop\n");
  printf("m = change mode, c=clear stats, g=get stats, i=color sensor, o=open claw, p=close claw, u=deliver medpad, q=exit\n");
  scanf("%c", &ch);
  // Purge extraneous characters from input stream
  flushInput();

  char buffer[10];
  int32_t params[2];

  buffer[0] = NET_COMMAND_PACKET;

  switch (ch)
  {
    case 'f':
    case 'F':
    case 'b':
    case 'B':
    case 'l':
    case 'L':
    case 'r':
    case 'R':
      getParams(params);
      buffer[1] = ch;
      memcpy(&buffer[2], params, sizeof(params));
      sendData(conn, buffer, sizeof(buffer));
      break;
    case 's':
    case 'S':
    case 'c':
    case 'C':
    case 'g':
    case 'G':
    case 'i':
    case 'I':
    case 'o':
    case 'O':
    case 'p':
    case 'P':
	  case 'u':
	  case 'U':
    case 'k':
    case 'K':
      params[0] = 0;
      params[1] = 0;
      memcpy(&buffer[2], params, sizeof(params));
      buffer[1] = ch;
      sendData(conn, buffer, sizeof(buffer));
      break;
    case 'm':
    case 'M':
      printf("Changing mode to WASD...\n");
      mode = 1;
      break;
    case 'q':
    case 'Q':
      *quit = 1;
      break;
    default:
      printf("BAD COMMAND\n");
  }
}

void *writerThread(void *conn)
{
    int quit=0;
    while(!quit){
        if(mode) {
            writeWASD(conn, &quit);
        }
        else{
            writeCommand(conn, &quit);
        }
    }
    printf("Exiting keyboard thread\n");

    /* TODO: Stop the client loop and call EXIT_THREAD */
	stopClient();
	EXIT_THREAD(conn);
    /* END TODO */
    return NULL;
}

/* TODO: #define filenames for the client private key, certificatea,
   CA filename, etc. that you need to create a client */
#define SERVER_NAME "192.168.230.184"
#define SERVER_PORT 5001
#define VERIFY_SERVER 1
#define CA_CERT_FNAME "signing.pem"
#define SERVER_NAME_ON_CERT "mine.com"
#define SEND_CERT 1
#define CLIENT_CERT_FNAME "laptop.crt"
#define CLIENT_PRIVATE_KEY "laptop.key"
/* END TODO */
void connectToServer(const char *serverName, int portNum)
{
    /* TODO: Create a new client */
    createClient(serverName, SERVER_PORT, VERIFY_SERVER, CA_CERT_FNAME,
                 SERVER_NAME_ON_CERT, SEND_CERT, CLIENT_CERT_FNAME, CLIENT_PRIVATE_KEY,
                 readerThread, writerThread);
    /* END TODO */
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

    /* TODO: Add in while loop to prevent main from exiting while the
    client loop is running */
    while(client_is_running());
    /* END TODO */
	printf("\nMAIN exiting\n\n");
}
