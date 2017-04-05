#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <arpa/inet.h>
#include <errno.h> // Error Checking
#include <string.h> // Required for strerror()

#include "crc.h"

#include "CPFrames.h"

#define SERIAL_DEVICE "/dev/serial0"
#define RESEND_TIME 5
#define PROTOCOL_VERSION CPV01_VERSION
#define PROTOCOL_SIZE CPV01_SIZE

int main(int argc, char** argv)
{

  if (argc != 2)
  {
    fprintf(stdout,"Usage: %s packets\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  FILE* file = fopen(argv[1], "rb");
  if(file == NULL)
  {
    fprintf(stderr,"File Null Error <%s>: %s\n", argv[1], strerror(errno));
    exit(EXIT_FAILURE);
  }

  crcInit();

  //-------------------------
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
  options.c_cflag = B115200 | CS8 | CLOCAL;   //<Set baud rate
  options.c_iflag = 0;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(serial, TCIFLUSH);
  tcsetattr(serial, TCSANOW, &options);

  int receiveState = 0;

  // We are keeping track of the last received package because
  // from time to time, packages get lost. If we get no packages for 5 seconds
  // we'll retry the last package
  time_t lastPackage = time(NULL);
  time_t now;

  int packets = 0;
  char buffer[12];

  // While Still Have Packets in File Queue
  while(!feof(file))
  {
    // Recieved Acknowledgement or First Loop
    if (receiveState == 0){
      // Read in next frame from file / fgets reads size-1 into buffer
      if (fgets(buffer, sizeof(CPFrameVersion02)+1, file) != NULL){
        CPFrameVersion02 *frame = (CPFrameVersion02 *) &buffer;

        // Check frame is uncorrupted using CRC
        if (crcFast((unsigned char *) &buffer, 10) != 0)
        {
          fprintf(stderr,"CRC Check on Packet <%d>: Failed\n", packets);
          exit(EXIT_FAILURE);
        }
        
        printf("%02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10]);
        printf("SFD: %4d, V: %4d, THETA1: %6d, THETA2 %6d, D3 %6d, CRC: %4d, EFD: %4d\n", frame->SFD, frame->VERSION, frame->THETA1, frame->THETA2, frame->D3, frame->CRC, frame->EFD);
        
        // Send Frame to Arduino
        time(&lastPackage);
        int bytes_written = write(serial, &buffer, 11);
        if (bytes_written < 0)
        {
          fprintf(stderr,"UART TX error on serial connection <%s>.\n", SERIAL_DEVICE);
        }

        receiveState = 1;
        packets++;   
      }
    }

    if (receiveState == 1)
    {

      time(&now);
      if (difftime(now, lastPackage) > RESEND_TIME)
      {
        fprintf(stderr,"Expected Response. Resending Packet <%d>.\n", packets);
        
        int bytes_written = write(serial, &buffer, 11);
        if (bytes_written < 0)
        {
          fprintf(stderr,"UART TX error on serial connection <%s>.\n", SERIAL_DEVICE);
        }
        time(&lastPackage);
      }

      int message_recieved = 0;
      int bytesRead = 0;
      char SFD = StartFrameDelimiter;
      char rx_buffer[PROTOCOL_SIZE] = {0};

      while(1)
      {

        char data[1] = {0};
        if (read(serial, (void*)data, 1) == 0)
        // if (fread(data, sizeof(data[0]), 1, file) == 0)
          break;

        if (bytesRead == 0)
        {
          // Look for header
          if (data[0] != SFD)
          {
            continue;
          }

          rx_buffer[bytesRead] = data[0];
          ++bytesRead;
          continue;
        }

        if (bytesRead == 1)
        {
          // Look for version size
          if (data[0] != PROTOCOL_VERSION)
          {
              bytesRead = 0;
              fprintf(stderr,"      Recieved Invalid Communication Protocol Version. Recieved <%d> expected <%d>\n", data[0], PROTOCOL_VERSION);
              continue;
          }

          rx_buffer[bytesRead] = data[0];
          ++bytesRead;
          continue;
        }

        // The > check ensures that we don't overflow and
        // that we discard bad messages
        if (bytesRead >= PROTOCOL_SIZE)
        {
          // We are done with the message. Regardless of the outcome
          // only a new message can come next      
          bytesRead = 0;

          // Check frame is uncorrupted using CRC
          if (crcFast((unsigned char *) &rx_buffer, PROTOCOL_SIZE-1) != 0)
          {
            fprintf(stderr,"Recieved Malformed Communication Protocol Frame: CRC Failed\n");
            message_recieved = 1;
          }
          break;
        }
        
        rx_buffer[bytesRead] = data[0];    
        ++bytesRead;
      }


      if (message_recieved == 1)
      {
        CPFrameVersion01 *frame = (CPFrameVersion01 *) &rx_buffer;
        printf("%02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX\n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4], rx_buffer[5], rx_buffer[6]);
        printf("SFD: %4d, V: %4d, CODE: %6d, CRC: %4d, EFD: %4d\n", frame->SFD, frame->VERSION, frame->CODE, frame->CRC, frame->EFD);

        // Time to Trigger some Messages

        if (frame->CODE == 040){
          receiveState = 0;
          printf("Recieved Request to Resend. Sending Immediately.\n");
          int bytes_written = write(serial, &buffer, 11);
          if (bytes_written < 0)
          {
            fprintf(stderr,"UART TX error on serial connection <%s>.\n", SERIAL_DEVICE);
          }
          time(&lastPackage);
        }
        if (frame->CODE == 041){
          receiveState = 0;
          printf("Recieved Acknowledgement\n");
        }
      }

    }
  }

  // Cleanup 
  printf("Sucessfully sent all <%d> scheduled packets in <%s> to Motor Controller. Closing serial connection at <%s>.\n", packets, argv[1], SERIAL_DEVICE);
  fclose(file);
  close(serial);
  return 0;
}