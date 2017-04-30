#include <stdlib.h>
#include <stdio.h>
#include <string.h> // Required for memcpy
#include "crc.h"
#include <math.h> // Required for floor, roundf
#include "CPFrames.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>

#define SERIAL_DEVICE "/dev/serial0"
#define RESEND_TIME 5
#define RX_PROTOCOL_VERSION CPV01_VERSION
#define RX_PROTOCOL_SIZE CPV01_SIZE
#define TX_PROTOCOL_VERSION CPV02_VERSION
#define TX_PROTOCOL_SIZE CPV02_SIZE

int main(int argc, char** argv)
{
  if (argc != 5)
  {
    fprintf(stdout,"Usage: %s code theta1 theta2 d3\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  int raw_code = atoi(argv[1]);
  int raw_theta1 = atoi(argv[2]);
  int raw_theta2 = atoi(argv[3]);
  int raw_d3 = atoi(argv[4]);

  crcInit();
	short  code, theta1, theta2, d3;
	code = floor(raw_code); theta1 = floor(roundf(raw_theta1*437.04)); theta2 = floor(roundf(raw_theta2*437.04)); d3 = floor(raw_d3);
	CPFrameVersion02 frame = {StartFrameDelimiter, CPV02_VERSION, raw_code, theta1, theta2, d3, 0, EndOfFrame};
	frame.CRC = crcFast((unsigned char *) &frame, CPV02_SIZE-3);

	unsigned char buffer[CPV02_SIZE];
	memcpy(buffer, &frame, CPV02_SIZE);
  printf("----> TX: %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10], buffer[11]);
  printf("      SFD: %4d, V: %4d, CODE: %4d, THETA1: %6d, THETA2 %6d, D3 %6d, CRC: %4d, EFD: %4d\n", frame.SFD, frame.VERSION, frame.CODE,frame.THETA1, frame.THETA2, frame.D3, frame.CRC, frame.EFD);
  

  // -------------------------
  //----- SETUP USART 0 -----
  //-------------------------
  //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
  int serial = -1;
  
  //OPEN THE UART
  //  The flags (defined in fcntl.h):
  //  Access modes (use 1 of these):
  //    O_RDONLY - Open for reading only.
  //    O_RDWR - Open for reading and writing.
  //    O_WRONLY - Open for writing only.
  //
  //  O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
  //                      if there is no input immediately available (instead of blocking). Likewise, write requests can also return
  //                      immediately with a failure status if the output can't be written immediately.
  //
  //  O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.

  serial = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);    //Open in non blocking read/write mode
  if (serial == -1)
  {
    // send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("UART Error", "error", "Unable to open UART.  Ensure it is not in use by another application"));
    fprintf(stderr,"UART Error <%s>: %s\n", argv[1], "Unable to open UART.  Ensure it is not in use by another application");
    exit(EXIT_FAILURE);
  }
  
  //CONFIGURE THE UART
  //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  //  Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
  //  CSIZE:- CS5, CS6, CS7, CS8
  //  CLOCAL - Ignore modem status lines
  //  CREAD - Enable receiver
  //  IGNPAR = Ignore characters with parity errors
  //  ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
  //  PARENB - Parity enable
  //  PARODD - Odd parity (else even)
  struct termios options;
  tcgetattr(serial, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;   //<Set baud rate
  options.c_iflag = 0;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(serial, TCIFLUSH);
  tcsetattr(serial, TCSANOW, &options);

  // Send Frame to Arduino
  int bytes_written;
  bytes_written = write(serial, &buffer, TX_PROTOCOL_SIZE);
  if (bytes_written < 0)
  {
    {
      char message_buffer[50];
      sprintf(message_buffer, "UART TX error on serial connection <%s>.", SERIAL_DEVICE);
      // send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("UART Error", "error", message_buffer));
      fprintf(stderr,"      ! %s\n", message_buffer);
    }
    exit(EXIT_FAILURE);
  }
  printf("      ----> Sent <%d> bytes to Motor Controller\n", bytes_written);
}