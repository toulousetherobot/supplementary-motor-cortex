#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <arpa/inet.h>
#include <errno.h> // Error Checking
#include <string.h> // Required for strerror()
#include <amqp_tcp_socket.h>
#include <amqp_framing.h>
#include <amqp.h>

#include "crc.h"

#include "CPFrames.h"
#include "os_communication.h"

#define SERIAL_DEVICE "/dev/serial0"
#define RESEND_TIME 5
#define RX_PROTOCOL_VERSION CPV01_VERSION
#define RX_PROTOCOL_SIZE CPV01_SIZE
#define TX_PROTOCOL_VERSION CPV02_VERSION
#define TX_PROTOCOL_SIZE CPV02_SIZE

#define JITTER 0
#define HOLDING 1

int main(int argc, char** argv)
{
  amqp_connection_state_t conn;

  open_amqp_conn(&conn);

  srand(time(NULL));   // should only be called once

  if (argc != 2)
  {
    fprintf(stdout,"Usage: %s packets\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  FILE* file = fopen(argv[1], "rb");
  if(file == NULL)
  {
    send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("File Null Error", "error", strerror(errno)));
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
    send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("UART Error", "error", "Unable to open UART.  Ensure it is not in use by another application"));
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
          {
            char message_buffer[50];
            sprintf(message_buffer, "CRC Check on Packet <%d> Failed.", packets);
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Checksum Failed", "error", message_buffer));
            fprintf(stderr,"      %s\n", message_buffer);
          }
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
          {
            char message_buffer[50];
            sprintf(message_buffer, "UART TX error on serial connection <%s>.", SERIAL_DEVICE);
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("UART Error", "error", message_buffer));
            fprintf(stderr,"      ! %s\n", message_buffer);
          }
          exit(EXIT_FAILURE);
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
        {
          char message_buffer[50];
          sprintf(message_buffer, "Expected Response. Resending Packet <%d>.", packets-1);
          send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Response Timeout", "warning", message_buffer));
          fprintf(stderr,"      %s\n", message_buffer);
        }
        
        int bytes_written = write(serial, &buffer, TX_PROTOCOL_SIZE);
        time(&lastPackage);
        if (bytes_written < 0)
        {
          {
            char message_buffer[50];
            sprintf(message_buffer, "UART TX error on serial connection <%s>.", SERIAL_DEVICE);
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("UART Error", "error", message_buffer));
            fprintf(stderr,"      ! %s\n", message_buffer);
          }
          exit(EXIT_FAILURE);
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
              {
                char message_buffer[50];
                sprintf(message_buffer, "Recieved Invalid Communication Protocol Version. Recieved <%d> expected <%d>", data[0], RX_PROTOCOL_VERSION);
                send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("CP Protocol Error", "warning", message_buffer));
                fprintf(stderr,"      %s\n", message_buffer);
              }
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
        if (frame->CODE == 1){ // Category 0 Emergency Stop
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Emergency Stop (0)", "error", "An uncontrolled stop by immediately removing power to the machine actuators."));
        }
        if (frame->CODE == 2){ // Category 1 Emergency Stop
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Emergency Stop (1)", "error", "A controlled stop with power to the machine actuators available to achieve the stop then remove power when the stop is achieved."));
        }
        if (frame->CODE == 3){ // Category 2 Emergency Stop
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Emergency Stop (2)", "error", "A controlled stop with power left available to the machine actuators."));
        }
        if (frame->CODE == 10){ // Shoulder Pan Link Limit Switch 1 Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Shoulder Pan Limit Switch 1 Hit", "error", "Shoulder Pan Link has exceeded the movement limits set by the physical hard stop through excessive motion clockwise."));
        }
        if (frame->CODE == 11){ // Shoulder Pan Link Limit Switch 2 Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Shoulder Pan Limit Switch 2 Hit", "error", "Shoulder Pan Link has exceeded the movement limits set by the physical hard stop through excessive motion counter-clockwise."));
        }
        if (frame->CODE == 12){ // Elbow Pan Link Limit Switch 1 Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Elbow Pan Limit Switch 1 Hit", "error", "Elbow Pan Link has exceeded the movement limits set by the physical hard stop through excessive motion clockwise."));
        }
        if (frame->CODE == 13){ // Elbow Pan Link Limit Switch 2 Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Elbow Pan Limit Switch 2 Hit", "error", "Elbow Pan Link has exceeded the movement limits set by the physical hard stop through excessive motion counter-clockwise."));
        }
        if (frame->CODE == 14){ // Wrist Flex Link Limit Switch Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Wrist Flex Limit Switch Hit", "error", "Wrist Flex Link has exceeded the movement limits set by the physical hard stop through excessive motion clockwise."));
        }
        if (frame->CODE == 15){ // Wrist Flex Link Soft Limit Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Wrist Flex Soft Limit Hit", "warning", "Wrist Flex Link has exceeded the movement limits set by software through excessive motion counter-clockwise."));
        }
        if (frame->CODE == 16){ // Wrist Roll Link Limit Switch Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Wrist Roll Limit Switch Hit", "error", "Wrist Roll Link has exceeded the movement limits set by the physical hard stop through excessive motion clockwise."));
        }
        if (frame->CODE == 17){ // Wrist Roll Link Soft Limit Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Wrist Roll Soft Limit Hit", "warning", "Wrist Roll Link has exceeded the movement limits set by software through excessive motion counter-clockwise."));
        }
        if (frame->CODE == 18){ // Wrist Extension Link End of Travel Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Wrist Extension End of Travel Hit", "error", "Wrist Roll Link has exceeded the movement limits set by the physical hard stop through excessive motion driving down into the page."));
        }
        if (frame->CODE == 19){ // Wrist Extension Link Start of Travel Hit
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Wrist Extension Start of Travel Hit", "error", "Wrist Roll Link has exceeded the movement limits set by the physical hard stop through excessive motion driving up out of the page."));
        }
        if (frame->CODE == 20){ // Complex Collision Detected
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Complex Collision Detected", "error", "Some complex combination of motor joints has caused the Robot wrist to collide with the Robot shelf."));
        }
        if (frame->CODE == 40){ // Resend Message
          {
            char message_buffer[50];
            sprintf(message_buffer, "Recieved Request to Resend Packet <%d>. Sending Immediately.", packets-1);
            send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Requested to Resend Packet", "info", message_buffer));
            printf("%s\n", message_buffer);
          }

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
        if (frame->CODE == 41){ // Message Acknowledge
          receiveState = 0;
          if (HOLDING){
            printf("Recieved Acknowledgement. Waiting for User to send to Next Packet. Press Any Key to Continue: ");
            fflush(stdout);
            getchar();
            printf("Moving to Next Packet.\n");

          }else{
            printf("Recieved Acknowledgement. Moving to Next Packet\n");
          }

          CPFrameVersion02 *processed_frame = (CPFrameVersion02 *) &buffer;
          send_amqp_message(&conn, TOULOUSE_AMPQ_STATE_ROUTING_KEY, form_update_os_payload(packets, processed_frame));
        }
      }
    }
  }

  // Cleanup 
  {
    char message_buffer[50];
    sprintf(message_buffer, "Sucessfully sent all <%d> scheduled packets in <%s> to Motor Controller. Closing serial connection at <%s>.", packets, argv[1], SERIAL_DEVICE);
    send_amqp_message(&conn, TOULOUSE_AMPQ_MESSAGE_ROUTING_KEY, form_message_payload("Sent All Packets", "success", message_buffer));
    printf("%s\n", message_buffer);
  }
  fclose(file);
  close(serial);
  close_amqp_conn(&conn);
  return 0;
}