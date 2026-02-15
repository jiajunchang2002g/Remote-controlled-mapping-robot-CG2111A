
#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
void left(float ang, float speed) {
    ccw(ang, speed);
}
void right(float ang, float speed) {
    cw(ang, speed);
}
volatile TDirection dir;
/*
   Alex's configuration constants
 */

#define ALEX_LENGTH 26 
#define ALEX_BREADTH 15 
float alexDiagonal = 6.5; 
float alexCirc = 0.0;

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 3

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 18

/*
   Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;
unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

// Def for Colour sensing pins
#define PinOut 39 // (1 << 4) // OUT, PL4, 39
#define S0 35 //(1 << 0) // PL0, 35
#define S1 36//(1 << 1) // PL1, 36
#define S2 37//(1 << 2) // PL2, 37
#define S3 38//(1 << 3) // PL3, 38
int Red = 0;
int Blue = 0;
int Green = 0;
float curr_time;
void sendColour();
/* 

   Alex Communication Routines.

 */


TResult readPacket(TPacket *packet) {
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".

    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if (len == 0)
        return PACKET_INCOMPLETE;
    else
        return deserialize(buffer, len, packet);
}

void sendStatus() {
    // Implement code to send back a packet containing key
    // information like leftTicks, rightTicks, leftRevs, rightRevs
    // forwardDist and reverseDist
    // Use the params array to store this information, and set the
    // packetType and command files accordingly, then use sendResponse
    // to send out the packet. See sendMessage on how to use sendResponse.
    //
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    statusPacket.params[0] = leftForwardTicks;
    statusPacket.params[1] = rightForwardTicks;
    statusPacket.params[2] = leftReverseTicks;
    statusPacket.params[3] = rightReverseTicks;
    statusPacket.params[4] = leftForwardTicksTurns;
    statusPacket.params[5] = rightForwardTicksTurns;
    statusPacket.params[6] = leftReverseTicksTurns;
    statusPacket.params[7] = rightReverseTicksTurns;
    statusPacket.params[8] = forwardDist;
    statusPacket.params[9] = reverseDist;
    sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
    // Sends text messages back to the Pi. Useful
    // for debugging.

    TPacket messagePacket;
    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
    va_list args;
    char buffer[128];
    va_start(args, format);
    vsprintf(buffer, format, args);
    sendMessage(buffer);
}
void sendBadPacket() {
    // Tell the Pi that it sent us a packet with a bad
    // magic number.

    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);
}

void sendBadChecksum() {
    // Tell the Pi that it sent us a packet with a bad
    // checksum.

    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

void sendBadCommand() {
    // Tell the Pi that we don't understand its
    // command sent to us.

    TPacket badCommand;
    badCommand.packetType = PACKET_TYPE_ERROR;
    badCommand.command = RESP_BAD_COMMAND;
    sendResponse(&badCommand);
}

void sendBadResponse() {
    TPacket badResponse;
    badResponse.packetType = PACKET_TYPE_ERROR;
    badResponse.command = RESP_BAD_RESPONSE;
    sendResponse(&badResponse);
}

void sendOK() {
    TPacket okPacket;
    okPacket.packetType = PACKET_TYPE_RESPONSE;
    okPacket.command = RESP_OK;
    sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
    // Takes a packet, serializes it then sends it out
    // over the serial port.
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

 */
// Enable pull up resistors on pins 18 and 19
void enablePullups() {
    // Use bare-metal to enable the pull-up resistors on pins
    // 19 and 18. These are pins PD2 and PD3 respectively.
    // We set bits 2 and 3 in DDRD to 0 to make them inputs.
    DDRD |= 0b00001100;
    PORTD |= 0b00001100;
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR() {
    if (dir == FORWARD )
    {
        leftForwardTicks++;
        forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
        //dbprintf("LEFTFORWARD: ");
        //dbprintf(leftForwardTicks);
    }
    else if (dir == BACKWARD)
    {
        leftReverseTicks++;
        //dbprintf("LEFTREVERSE: ");
        //dbprintf(leftReverseTicks);
        reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
    }
    else if (dir == LEFT)
    {
        leftReverseTicksTurns++;

    }
    else if (dir == RIGHT)
    {
        leftForwardTicksTurns++;
    }


}

void rightISR() {
    if (dir == FORWARD )
    {
        rightForwardTicks++;
        //dbprintf("LEFTFORWARD: ");
        //dbprintf(leftForwardTicks);

    }
    else if (dir == BACKWARD)
    {
        rightReverseTicks++;
        //dbprintf("LEFTREVERSE: ");
        ///dbprintf(leftReverseTicks);

    }
    else if (dir == LEFT)
    {
        rightForwardTicksTurns++;

    }
    else if (dir == RIGHT)
    {
        rightReverseTicksTurns++;
    }

}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT() {
    // Use bare-metal to configure pins 18 and 19 to be
    // falling edge triggered. Remember to enable
    // the INT2 and INT3 interrupts.
    // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
    EICRA |= 0b10100000;
    EICRB = 0b0;
    EIMSK |= 0b00001100;
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.
ISR(INT3_vect) {
    leftISR();
}

ISR(INT2_vect) {
    rightISR();
}

// Implement INT2 and INT3 ISRs above.

/*
   Setup and start codes for serial communications

 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
    // To replace later with bare-metal.
    Serial.begin(9600);
    // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
    // Empty for now. To be replaced with bare-metal code
    // later on.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {

    int count = 0;

    // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

    while (Serial.available())
        buffer[count++] = Serial.read();

    return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
    Serial.write(buffer, len);
    // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
   Alex's setup and run codes

 */

// Clears all our counters
void clearCounters() {
    leftForwardTicks = 0;
    rightForwardTicks = 0;
    leftReverseTicks = 0;
    rightReverseTicks = 0;

    leftForwardTicksTurns = 0;
    rightForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightReverseTicksTurns = 0;


    leftRevs = 0;
    rightRevs = 0;
    forwardDist = 0;
    reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which) {
    clearCounters();
}
// Intialize Alex's internal states

void initializeState() {
    clearCounters();
}

void handleCommand(TPacket *command) {
    switch (command->command) {
        // For movement commands, param[0] = distance, param[1] = speed.
        case COMMAND_FORWARD:
            sendOK();
            forward((double)command->params[0], (float)command->params[1]);
            break;

        case COMMAND_REVERSE:
            sendOK();
            backward((double)command->params[0], (float)command->params[1]);
            break;

        case COMMAND_TURN_LEFT:
            sendOK();
            ccw((double)command->params[0], (float)command->params[1]);
            break;

        case COMMAND_TURN_RIGHT:
            sendOK();
            cw((double)command->params[0], (float)command->params[1]);
            break;
        case COMMAND_STOP:
            sendOK();
            stop();
            break;
        case COMMAND_GET_STATS:
            sendStatus();
            break;
        case COMMAND_CLEAR_STATS:
            sendOK();
            clearOneCounter(command->params[0]);
            break;
        case COMMAND_GET_COLOUR:
            sendOK();
            sendColour();
            break;

        default:
            sendBadCommand();
    }
}

void getColour(TPacket* colourPacket)
{
    /*
    // Shine red S2, S3: L L
    PORTL &= ~(S2);
    PORTL &= ~(S3);
    while(!(PINL & PinOut));
    curr_time = micros();
    while(PINL & PinOut);
    Red = micros() - curr_time;
    delay(20);

    // Shine blue S2 S3 L H
    PORTL |= S3;
    while (!(PINL & PinOut));
    curr_time = micros();
    while ((PINL & PinOut));
    Blue = micros() - curr_time;
    delay(20);

    // shine Green, S2 S4 H H
    PORTL |= S2;
    while (!(PINL & PinOut));
    curr_time = micros();
    while ((PINL & PinOut));
    Green = micros() - curr_time;
    delay(20);
     */
    digitalWrite(S2,  LOW);                                          
    digitalWrite(S3, LOW);                                           
    Red = pulseIn(PinOut, digitalRead(PinOut) == HIGH ? LOW : HIGH);        
    delay(20);  
    digitalWrite(S3, HIGH);                                         
    Blue = pulseIn(PinOut, digitalRead(PinOut) == HIGH ? LOW  : HIGH);
    delay(20);  
    digitalWrite(S2, HIGH);  
    Green = pulseIn(PinOut,  digitalRead(PinOut) == HIGH ? LOW : HIGH);
    delay(20);  
    // Detection, Try KNN, Need to Change This Part
    if (Red < Blue && Red <= Green && Red < 45) {
        colourPacket->params[0] = 0;
    } else if (Green < Red && Green < Blue && Green < 40) {
        colourPacket->params[0] = 1;
    } else if (Blue < Green && Blue < Red && Blue < 20) {
        colourPacket->params[0] = 2;
    } else {
        colourPacket->params[0] = 3;
        colourPacket->params[1] = Red;
        colourPacket->params[2] = Green;
        colourPacket->params[3] = Blue;
    }
}
void sendColour() {
    TPacket colorPacket;
    colorPacket.packetType = PACKET_TYPE_RESPONSE;
    colorPacket.command = RESP_COLOUR;
    getColour(&colorPacket);
    sendResponse(&colorPacket);
}
void ColourSetup()
{
    /*
       DDRL = (S0|S1|S2|S3) & ~PinOut;
       PORTL |= (S0 | S1);
     */
    pinMode(S0,OUTPUT);   //pin modes
    pinMode(S1,OUTPUT);
    pinMode(S2,OUTPUT);
    pinMode(S3,OUTPUT);
    pinMode(PinOut,INPUT);
    digitalWrite(S0, HIGH);
    digitalWrite(S1, HIGH);
}
void waitForHello() {
    int exit = 0;

    while (!exit) {
        TPacket hello;
        TResult result;

        do {
            result = readPacket(&hello);
        } while (result == PACKET_INCOMPLETE);

        if (result == PACKET_OK) {
            if (hello.packetType == PACKET_TYPE_HELLO) {


                sendOK();
                exit = 1;
            } else
                sendBadResponse();
        } else if (result == PACKET_BAD) {
            sendBadPacket();
        } else if (result == PACKET_CHECKSUM_BAD)
            sendBadChecksum();
    }  // !exit
}

void setup() {
    // put your setup code here, to run once:
    alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
    alexCirc = PI * alexDiagonal;

    cli();
    setupEINT();
    setupSerial();
    startSerial();
    enablePullups();
    initializeState();
    ColourSetup();
    sei();
}

void handlePacket(TPacket *packet) {
    switch (packet->packetType) {
        case PACKET_TYPE_COMMAND:
            handleCommand(packet);
            break;

        case PACKET_TYPE_RESPONSE:
            break;

        case PACKET_TYPE_ERROR:
            break;

        case PACKET_TYPE_MESSAGE:
            break;

        case PACKET_TYPE_HELLO:
            break;
    }
}

void loop() {
    // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

    //forward(0, 100);
    //elay(1000);
    //backward(0,100);
    //delay(1000);
    //ccw(0,100);
    //delay(1000);
    //ccw(0,100);
    //delay(1000);

    // Uncomment the code below for Week 9 Studio 2


    // put your main code here, to run repeatedly:
    TPacket recvPacket; // This holds commands from the Pi

    TResult result = readPacket(&recvPacket);

    if (result == PACKET_OK)
        handlePacket(&recvPacket);
    else if (result == PACKET_BAD)
    {
        sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
    {
        sendBadChecksum();
    }
    if (deltaDist > 0)
    {
        if (dir == FORWARD)
        {

            if (forwardDist > newDist)
            {
                deltaDist = 0;
                newDist = 0;
                stop();
            }
        }
        else if (dir == BACKWARD)
        {
            if (reverseDist > newDist)
            {
                deltaDist = 0;
                newDist = 0;
                stop();
            }
        }
        else if (dir == (TDirection)STOP)
        {
            deltaDist = 0;
            newDist = 0;
            stop();
        }
    }
    if(deltaTicks>0)
    {
        if(dir == LEFT)
        {
            if(leftReverseTicksTurns >= targetTicks)
            {
                deltaTicks =0;
                targetTicks =0;
                stop();
            }
        }
        else if(dir==RIGHT)
        {
            if(rightReverseTicksTurns >= targetTicks)
            {
                deltaTicks =0;
                targetTicks =0;
                stop();
            }
        }
        else if (dir == (TDirection)STOP)
        {
            deltaTicks =0;
            targetTicks =0;
            stop();
        }
    }



}
