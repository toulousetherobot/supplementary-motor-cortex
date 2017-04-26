#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <arpa/inet.h>
#include <errno.h> // Error Checking
#include <string.h> // Required for strerror()
#include "json.h"

#include "crc.h"

#include "CPFrames.h"

#define SERIAL_DEVICE "/dev/serial0"
#define RESEND_TIME 5
#define RX_PROTOCOL_VERSION CPV01_VERSION
#define RX_PROTOCOL_SIZE CPV01_SIZE
#define TX_PROTOCOL_VERSION CPV02_VERSION
#define TX_PROTOCOL_SIZE CPV02_SIZE

#define JITTER 0
#define HOLDING 1

const char *form_update_os_payload(int frame_number, CPFrameVersion02 *frame){
  /*Creating a json object*/
  json_object * jobj = json_object_new_object();

  /*Creating a json integer*/
  json_object *jfrm = json_object_new_int(frame_number);
  json_object *jtheta1 = json_object_new_int(frame->THETA2);
  json_object *jtheta2 = json_object_new_int(frame->THETA1);
  json_object *jd3 = json_object_new_int(frame->D3);

  /*Form the json object*/
  /*Each of these is like a key value pair*/
  json_object_object_add(jobj,"frame", jfrm);
  json_object_object_add(jobj,"theta1", jtheta1);
  json_object_object_add(jobj,"theta2", jtheta2);
  json_object_object_add(jobj,"d3", jd3);

  /*Now printing the json object*/
  return json_object_to_json_string(jobj);
}

const char *form_message_payload(const char *title, const char *type, const char *footnote){
  /*Creating a json object*/
  json_object * jobj = json_object_new_object();

  /*Creating a json string*/
  json_object *jtitle = json_object_new_string(title);
  json_object *jtype = json_object_new_string(type);
  json_object *jfootnote = json_object_new_string(footnote);

  /*Form the json object*/
  /*Each of these is like a key value pair*/
  json_object_object_add(jobj,"title", jtitle);
  json_object_object_add(jobj,"type", jtype);
  json_object_object_add(jobj,"footnote", jfootnote);

  /*Now printing the json object*/
  return json_object_to_json_string(jobj);
}

int main(int argc, char** argv)
{
  srand(time(NULL));   // should only be called once

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
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;   //<Set baud rate
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
  char buffer[TX_PROTOCOL_SIZE];
  char corrupted_buffer[TX_PROTOCOL_SIZE];

  // While Still Have Packets in File Queue
  while(!feof(file))
  {
    // Recieved Acknowledgement or First Loop
    if (receiveState == 0){
      // Read in next frame from file / fgets reads size-1 into buffer
      if (fread(buffer, 1, TX_PROTOCOL_SIZE, file) > 0){
        CPFrameVersion02 *frame = (CPFrameVersion02 *) &buffer;

        printf("----> TX F<%d>: %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX\n", packets, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10], buffer[11]);
        printf("      SFD: %4d, V: %4d, CODE: %4d, THETA1: %6d, THETA2 %6d, D3 %6d, CRC: %4d, EFD: %4d\n", frame->SFD, frame->VERSION, frame->CODE,frame->THETA1, frame->THETA2, frame->D3, frame->CRC, frame->EFD);

        // Check frame is uncorrupted using CRC
        if (crcFast((unsigned char *) &buffer, TX_PROTOCOL_SIZE-3) != frame->CRC)
        {
          fprintf(stderr,"      ! CRC Check on Packet <%d>: Failed\n", packets);
          exit(EXIT_FAILURE);
        }
        
        // Send Frame to Arduino
        int bytes_written;
        if (JITTER && rand() % 3 == 0) {
          memcpy(corrupted_buffer, buffer, TX_PROTOCOL_SIZE);
          corrupted_buffer[rand() % TX_PROTOCOL_SIZE] = 12;
          printf("      Corrupted TX F<%d>: %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX %02hhX%02hhX\n", packets, corrupted_buffer[0], corrupted_buffer[1], corrupted_buffer[2], corrupted_buffer[3], corrupted_buffer[4], corrupted_buffer[5], corrupted_buffer[6], corrupted_buffer[7], corrupted_buffer[8], corrupted_buffer[9], corrupted_buffer[10], corrupted_buffer[11]);
          bytes_written = write(serial, &corrupted_buffer, TX_PROTOCOL_SIZE);
        }else{
          bytes_written = write(serial, &buffer, TX_PROTOCOL_SIZE);
        }
        time(&lastPackage);
        if (bytes_written < 0)
        {
          fprintf(stderr,"      ! UART TX error on serial connection <%s>.\n", SERIAL_DEVICE);
        }
        printf("      ----> Sent <%d> bytes to Motor Controller\n", bytes_written);

        receiveState = 1;
        packets++;   
      }
    }

    if (receiveState == 1)
    {

      time(&now);
      if (difftime(now, lastPackage) > RESEND_TIME)
      {
        fprintf(stderr,"      Expected Response. Resending Packet <%d>.\n", packets);
        
        int bytes_written = write(serial, &buffer, TX_PROTOCOL_SIZE);
        time(&lastPackage);
        if (bytes_written < 0)
        {
          fprintf(stderr,"      ! UART TX error on serial connection <%s>.\n", SERIAL_DEVICE);
        }
        printf("      ----> Sent <%d> bytes to Motor Controller\n", bytes_written);
      }

      int message_recieved = 0;
      int bytesRead = 0;
      char SFD = StartFrameDelimiter;
      char rx_buffer[RX_PROTOCOL_SIZE] = {0};

      while(1)
      {
        char data[1] = {0};
        if (read(serial, (void*)data, 1) <= 0)
          break;

        printf("%02hhX%s", data[0], bytesRead%2 ? " ": "");

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
          if (data[0] != RX_PROTOCOL_VERSION)
          {
              bytesRead = 0;
              fprintf(stderr,"      Recieved Invalid Communication Protocol Version. Recieved <%d> expected <%d>\n", data[0], RX_PROTOCOL_VERSION);
              continue;
          }

          rx_buffer[bytesRead] = data[0];
          ++bytesRead;
          continue;
        }

        rx_buffer[bytesRead] = data[0];    
        ++bytesRead;

        // The > check ensures that we don't overflow and
        // that we discard bad messages
        if (bytesRead >= RX_PROTOCOL_SIZE)
        {

          // We are done with the message. Regardless of the outcome
          // only a new message can come next      
          message_recieved = 1;
          break;
        }
      }

      if (message_recieved == 1)
      {
        CPFrameVersion01 *frame = (CPFrameVersion01 *) &rx_buffer;
        printf("\n----> Recieved <%d> bytes from Motor Controller\n", bytesRead);
        bytesRead = 0;
        // printf("%02hhX%02hhX %02hhX%02hhX\n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
        printf("SFD: %4d, V: %4d, CODE: %6d, EFD: %4d\n", frame->SFD, frame->VERSION, frame->CODE, frame->EFD);

        // Trigger some Messages
        if (frame->CODE == 40){
          printf("Recieved Request to Resend Packet <%d>. Sending Immediately.\n", packets);

          int bytes_written;
          if (JITTER && rand() % 3 == 0){
            bytes_written = write(serial, &corrupted_buffer, TX_PROTOCOL_SIZE);
          }else{
            bytes_written = write(serial, &buffer, TX_PROTOCOL_SIZE);            
          }
          time(&lastPackage);
          if (bytes_written < 0)
          {
            fprintf(stderr,"UART TX error on serial connection <%s>.\n", SERIAL_DEVICE);
          }
        }
        if (frame->CODE == 41){
          receiveState = 0;
          if (HOLDING){
            printf("Recieved Acknowledgement. Waiting for User to send to Next Packet. Press Any Key to Continue: ");
            fflush(stdout);
            getchar();
            printf("Moving to Next Packet.\n");

          }else{
            printf("Recieved Acknowledgement. Moving to Next Packet\n");
          }
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