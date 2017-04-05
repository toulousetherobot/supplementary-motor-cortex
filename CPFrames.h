#ifndef CPFRAMES_H
#define CPFRAMES_H

int foo(int x);  /* An example function declaration */

#define StartFrameDelimiter 0xAB
#define EndOfFrame 0xCD

#define CPV01_SIZE 7
#define CPV01_VERSION 1
#define CPV02_SIZE 11
#define CPV02_VERSION 2
#define CPV03_SIZE 15
#define CPV03_VERSION 3

// The structure is maked with __attribute((packed))
// because we don't want any structure padding. Otherwise we might
// send an invalid message to the device.
typedef struct {
    unsigned char SFD;
    unsigned char VERSION;
    int16_t CODE;
    unsigned short CRC;
    unsigned char EFD;
} __attribute__((packed)) CPFrameVersion01;

typedef struct {
    unsigned char SFD;
    unsigned char VERSION;
    int16_t THETA1;
    int16_t THETA2;
    int16_t D3;
    unsigned short CRC;
    unsigned char EFD;
} __attribute__((packed)) CPFrameVersion02;

typedef struct {
    unsigned char SFD;
    unsigned char VERSION;
    int16_t THETA1;
    int16_t THETA2;
    int16_t THETA3;
    int16_t THETA4;
    int16_t D5;
    unsigned short CRC;
    unsigned char EFD;
} __attribute__((packed)) CPFrameVersion03;

#endif // CPFRAMES_H