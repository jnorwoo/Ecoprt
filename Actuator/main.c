

#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include <termios.h>

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

void changemode(int);
int  kbhit(void);


void changemode(int dir)
{
  static struct termios oldt, newt;

  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int kbhit (void) // alloows you to get keyboard character typed without having to press enter like in
{                //get char which would halt the program and interupt serial comminucation
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);

  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);

}


void respond_to_command(char *read_buffer, int chars_read);
void serial_port_close();
int serial_port_open(void);
int serial_port_read(char *read_buffer, size_t max_chars_to_read);
void sigint_handler(int sig);

#define MAX_COMMAND_LENGTH 100
#define MCFLAGS_ONOFF 0x1

#define MCFLAGS_THROTTLEFORWARD 0x2
#define MCFLAGS_THROTTLEREVERSE 0x4

static const char *PORT_NAME = "/dev/ttyUSB0";

int serial_port;
struct termios options_original;
typedef unsigned char BYTE;

typedef struct
{
	BYTE command[2];
    BYTE actuator[3];
    BYTE motorcontrolflags;
    BYTE motorthrottle;
	BYTE ending[2];
} serial_command_t;
void serial_port_write(char *write_buffer);

enum  actuator_type { ACTSTEERING=0, ACTLEFTBRAKE=1, ACTRIGHTBRAKE=2};


// Opens the USB serial port and continuously attempts to read from the port.
// On receiving data, looks for a defined command.

int main(int argc, char* argv[])
{



	printf("Instructions:\n");
	printf("[A] Left\n");
	printf("[d] Right\n");
	printf("[q] Exit\n");

	int chars_read; //how many chars read from the serial buffer
	char read_buffer[MAX_COMMAND_LENGTH + 1] = {0}; // put charachters into a buffer

    int ch;
    changemode(1); //not sure if needed
	serial_port_open();
    // the serial port is going to send a constant stream of commands
    // folowing this data structure
    // right now we want to send a defualt  do nothing command
	serial_command_t serialcommand;
	serialcommand.command[0] = 'E'; // start of command is ec
	serialcommand.command[1] = 'C';
	serialcommand.ending[0] = '\r'; //end of comand is return and newline
	serialcommand.ending[1] = '\n';
	serialcommand.actuator[ACTSTEERING] = 180; //180 is defualt number to send which is no turning for actuator
	serialcommand.actuator[ACTLEFTBRAKE] = 0x00;
	serialcommand.actuator[ACTRIGHTBRAKE] = 0x00;
	serialcommand.motorthrottle = 0;
	serialcommand.motorcontrolflags = 0;
	BYTE OnState = 0, ForState = 0, RevState = 0;
	int icount=0;    //aprintf("%s", 0x454300000000000d0a);

	BYTE Steering1 = 48;
	int flagpres = 0;
	int jcount = 0;
	while(1) // main loop of program will exit when you send q command
	{

		char inputstate;
        changemode(1); // this changes how the keyboard responds


		if (kbhit()) // check to see if you hit a character on the keyboard
		{
            flagpres = 1;
            changemode(0); // ignore need to press enter
			inputstate = getchar( ); // get character here
			if (inputstate == 'q') // if q quit program
				break;
			serialcommand.command[0] = 'E';
			serialcommand.command[1] = 'C';
			serialcommand.ending[0] = '\r';
			serialcommand.ending[1] = '\n';
			BYTE reverse_button =  (inputstate == 'a'); // if press a reverse acuator go left
			BYTE forward_button = (inputstate == 'd'); // if d forward acuator go right
			BYTE neutral_button = (inputstate == 's'); // s will stop the movement neutral
			BYTE on_button = (inputstate == 'c'); // c will turn on a relay for the acuator turn it on
			BYTE off_button = (inputstate == 'z'); // z will turn relay off acuator is off

			BYTE Throttle = 0;
			BYTE Braking = 0;

			if (on_button)
				OnState = 1;
			if (off_button)
				OnState = 0;
			if (forward_button)
			{
				Steering1 = 255; // just move the acuator to highest position
				ForState = 1;
				RevState = 0;
			}
			if (reverse_button)
			{
				Steering1 = 0; // move acuator to lowest positon
				ForState = 0;
				RevState = 1;
			}
			if (neutral_button)
			{
				if (neutral_button)
					Steering1 = 48;  //This is not correct anymore you will have to do something else for neutral postion
				ForState = 0;
				RevState = 0;
			}

			if (Steering1 < 0)
			{
				Steering1 = 0;
			}

			serialcommand.actuator[ACTSTEERING] = Steering1;
			serialcommand.actuator[ACTLEFTBRAKE] = Braking;
			serialcommand.actuator[ACTRIGHTBRAKE] = Braking;

			// very basic error detection and correciton
			serialcommand.motorthrottle = 0;
			if (OnState && (ForState || RevState))
				serialcommand.motorthrottle = Throttle;

			serialcommand.motorcontrolflags = (OnState?MCFLAGS_ONOFF:0)|(ForState?MCFLAGS_THROTTLEFORWARD:0)|(RevState?MCFLAGS_THROTTLEREVERSE:0);
		}
		icount++;
		if (icount >=10)
		{
			jcount++;
		serial_port_write(&serialcommand); // write the command to the serial port

        icount = 0;
		}
		int readflag = 0;
        usleep(1000000 /50); // sleep for .02 seconds so we are sending commands to about 50 hz
		{
			// Read received data into the buffer
			// up to but not including the buffer's terminating null.

            chars_read = serial_port_read(read_buffer, MAX_COMMAND_LENGTH);
			if (chars_read > 0)
			{
				// Data was read.

                int i;
                for (i = 0; i < chars_read; i++)
                {
                printf("%c", read_buffer[i]);
                }
			}
        }


	}

	//final command
	serialcommand.command[0] = 'E';
	serialcommand.command[1] = 'C';
	serialcommand.ending[0] = '\r';
	serialcommand.ending[1] = '\n';
	serialcommand.actuator[ACTSTEERING] = 0;
	serialcommand.actuator[ACTLEFTBRAKE] = 0xFF;
	serialcommand.actuator[ACTRIGHTBRAKE] = 0xFF;
	serialcommand.motorthrottle = 0;
	serialcommand.motorcontrolflags = 0;

		serial_port_write(&serialcommand);
			serial_port_close();
		exit(EXIT_SUCCESS);
	return( 0 );
}





// Resets the terminal and closes the serial port.

void serial_port_close()
{
	tcsetattr(serial_port,TCSANOW,&options_original);
	close(serial_port);
}

// Opens a USB virtual serial port at ttyUSB0.
//
// returns - the port's file descriptor or -1 on error.

int serial_port_open(void)
{
//open the serial port  which acts like a file object
  struct termios options;

  serial_port = open(PORT_NAME, O_RDWR | O_NONBLOCK);

  if (serial_port != -1)
  {
	  printf("Serial Port open\n");
	  tcgetattr(serial_port,&options_original); // now we need to set the port options
 	  tcgetattr(serial_port, &options);             // so that it has a defualt run behavior.
	  cfsetispeed(&options, B921600);               // which meshes well with the motor control board
	  cfsetospeed(&options, B921600);
	  options.c_cflag |= (CLOCAL | CREAD | CS8);
      options.c_lflag |= ICANON;
      //hard coded values which has been shown to work
  options.c_iflag = 0x0;
  options.c_oflag = 0x0;
  options.c_cflag = 0x1cb7;
  options.c_lflag = 0x8a22;
  options.c_line = 0x0;
  options.c_cc[0] = 0x3;
  options.c_cc[1] = 0x1c;
  options.c_cc[2] = 0x7f;
  options.c_cc[3] = 0x15;
  options.c_cc[4] = 0x4;
  options.c_cc[5] = 0x0;
  options.c_cc[6] = 0x1;
  options.c_cc[7] = 0x0;
  options.c_cc[8] = 0x11;
  options.c_cc[9] = 0x13;
  options.c_cc[10] = 0x1a;
  options.c_cc[11] = 0x0;
  options.c_cc[12] = 0x12;
  options.c_cc[13] = 0xf;
  options.c_cc[14] = 0x17;
  options.c_cc[15] = 0x16;
  options.c_cc[16] = 0x0 ;
  options.c_cc[17] = 0x0 ;
  options.c_cc[18] = 0x0 ;
  options.c_cc[19] = 0x0 ;
  options.c_cc[20] = 0x0 ;
  options.c_cc[21] = 0x0 ;
  options.c_cc[22] = 0x0 ;
  options.c_cc[23] = 0x0 ;
  options.c_cc[24] = 0x0 ;
  options.c_cc[25] = 0x0 ;
  options.c_cc[26] = 0x0 ;
  options.c_cc[27] = 0x0 ;
  options.c_cc[28] = 0x0 ;
  options.c_cc[29] = 0x0 ;
  options.c_cc[20] = 0x0 ;
  options.c_cc[31] = 0x0 ;
  options.c_ispeed = 0x1007;
  options.c_ospeed = 0x1007;

    tcsetattr(serial_port, TCSANOW, &options);
  }
  else
	  printf("Unable to open /dev/ttyUSB0\n");
  return (serial_port);
}

// Attempts to read up to 10 bytes from the serial port.
// If data was read, calls a routine to examine the received data
// and take action.
// *read_buffer - the buffer that will contain the data read.
// max_chars_to_read - the maximum number of characters to read.
// returns - 0 if data was received, -1 if no data received.

int serial_port_read(char *read_buffer, size_t max_chars_to_read)
{
	int chars_read = read(serial_port, read_buffer, max_chars_to_read);

	return chars_read;
}

// Writes data to the port.
// Parameter:  write_buffer - the data to write to the port.
// *write_buffer - the buffer that contains the data to write.

void serial_port_write(char *write_buffer)
{
	int bytes_written;
	size_t len = 0;

    char example[9] = "EC1234567";
    example[7] = (char ) 13;
    example[8] = (char ) 10;

	len =  sizeof(serial_command_t);
	bytes_written = write(serial_port, write_buffer, len );
    //bytes_written = write(serial_port, example, 9 );

	if (bytes_written < len)
	{
		printf("Write failed \n");
	}

}

// Executes when the user presses Ctrl+C.
// Closes the port, resets the terminal, and exits the program.

void  sigint_handler(int sig)
{
	serial_port_close();
	exit (sig);
}



