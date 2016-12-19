/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <sstream>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include "global.h"

#define DEFAULT_BAUDRATE 9600
#define DEFAULT_SERIALPORT "/dev/ttyUSB0"

// #define MOCK_SERIAL

//Global data
FILE *fpSerial = NULL; //serial port file pointer
int fdSerial = -1; // serial port file descriptor
ros::Publisher ucResponseMsg;
ros::Subscriber ucCommandMsg;
ros::Publisher rWheelEncoderMsg;
ros::Publisher lWheelEncoderMsg;
int ucIndex; //ucontroller index number

static pthread_mutex_t printf_mutex;


//Initialize serial port, return file descriptor
FILE* serialInit(char* port, int baud)
{
    int BAUD = 0;
    int fd = -1;
    struct termios newtio;
    FILE *fp = NULL;

    //Open the serial port as a file descriptor for low level configuration
    // read/write, not controlling terminal for process,
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY );
    if ( fd<0 )
    {
        ROS_ERROR("serialInit: Could not open serial device %s",port);
        return fp;
    }

    // set up new settings
    memset(&newtio, 0,sizeof(newtio));
    newtio.c_cflag =  CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit

    newtio.c_iflag = IGNCR;    //ignore CR, other options off
    newtio.c_iflag |= IGNBRK;  //ignore break condition

    newtio.c_oflag = 0;        //all options off

    newtio.c_lflag = ICANON;     //process input as lines

    // activate new settings
    tcflush(fd, TCIFLUSH);
    //Look up appropriate baud rate constant
    switch (baud)
    {
        case 38400:
        default:
            BAUD = B38400;
            break;
        case 19200:
            BAUD  = B19200;
            break;
        case 9600:
            BAUD  = B9600;
            break;
        case 4800:
            BAUD  = B4800;
            break;
        case 2400:
            BAUD  = B2400;
            break;
        case 1800:
            BAUD  = B1800;
            break;
        case 1200:
            BAUD  = B1200;
            break;
    }  //end of switch baud_rate
    if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
    {
        ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
        close(fd);
        return NULL;
    }
    tcsetattr(fd, TCSANOW, &newtio);
    tcflush(fd, TCIOFLUSH);

    fdSerial = fd;

    //Open file as a standard I/O stream
    fp = fdopen(fd, "r+");
    if (!fp) {
      ROS_ERROR("serialInit: Failed to open serial stream %s", port);
      fp = NULL;
    }
    return fp;
} //serialInit


//Process ROS command message, send to uController
void ucCommandCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("uc%dCommand: %s", ucIndex, msg->data.c_str());
#ifndef MOCK_SERIAL
    pthread_mutex_lock(&printf_mutex);
    fprintf(fpSerial, "%s\n", msg->data.c_str()); //appends newline
    pthread_mutex_unlock(&printf_mutex);
    ROS_INFO("mutex unlocked");
#endif
} //ucCommandCallback


//Request encoder values from robot
void *encThread(void *arg)
{
    int rcvBufSize = 200;
    char ucResponse[rcvBufSize]; //response string from uController
    char *bufPos;
    std_msgs::String sMsg;
    std_msgs::Int16 iMsg;
    int rEncoderVal, lEncoderVal;

    ROS_INFO("encThread: encoder thread running");

    while (ros::ok()) {
        pthread_mutex_lock(&printf_mutex);
        fprintf(fpSerial, "%s\n", "1 POS");
        pthread_mutex_unlock(&printf_mutex);

        pthread_mutex_lock(&printf_mutex);
        bufPos = fgets(ucResponse, rcvBufSize, fpSerial);
        pthread_mutex_unlock(&printf_mutex);
        if (bufPos != NULL)
            rEncoderVal = atoi(ucResponse);

        usleep(40000);

        pthread_mutex_lock(&printf_mutex);
        fprintf(fpSerial, "%s\n", "2 POS");
        pthread_mutex_unlock(&printf_mutex);

        pthread_mutex_lock(&printf_mutex);
        bufPos = fgets(ucResponse, rcvBufSize, fpSerial);
        pthread_mutex_unlock(&printf_mutex);
        if (bufPos != NULL)
            lEncoderVal = atoi(ucResponse);

        if (rEncoderVal != 0 && lEncoderVal != 0)
        {
            iMsg.data = rEncoderVal;
            rWheelEncoderMsg.publish(iMsg);
            iMsg.data = lEncoderVal;
            lWheelEncoderMsg.publish(iMsg);
        }

        usleep(100000);
    }
    return NULL;
}

//Receive command responses from robot uController
//and publish as a ROS message
void *rcvThread(void *arg)
{
    int rcvBufSize = 200;
    char ucResponse[rcvBufSize]; //response string from uController
    char *bufPos;
    std_msgs::String sMsg;
    std_msgs::Int16 iMsg;

    ROS_INFO("rcvThread: receive thread running");

    while (ros::ok()) {
      bufPos = fgets(ucResponse, rcvBufSize, fpSerial);
      if (bufPos != NULL) {
        ROS_INFO("uc%dResponse: %s", ucIndex, ucResponse);
        // sMsg.data = ucResponse;
        // ucResponseMsg.publish(sMsg);
      }
      bufPos = fgets(ucResponse, rcvBufSize, fpSerial);
      if (bufPos != NULL) {
        ROS_INFO("uc%dResponse: %s", ucIndex, ucResponse);
        // sMsg.data = ucResponse;
        // ucResponseMsg.publish(sMsg);
      }
    }
    return NULL;
}


int main(int argc, char **argv)
{
    char port[20]; //port name
    int baud; //baud rate

    char topicSubscribe[20];
    char topicPublish[20];

    pthread_t rcvThrID;   //receive thread ID
    pthread_t encThrID;   //encoder thread ID
    pthread_mutex_init(&printf_mutex, NULL);
    int err;

    //Initialize ROS
    ros::init(argc, argv, "r2serial");
    ros::NodeHandle rosNode;
    ROS_INFO("r2serial starting");

    strcpy(port, DEFAULT_SERIALPORT);
    baud = DEFAULT_BAUDRATE;
    ROS_INFO("connection initializing (%s) at %d baud", port, baud);

#ifndef MOCK_SERIAL
    fpSerial = serialInit(port, baud);
    if (!fpSerial )
    {
        ROS_ERROR("unable to create a new serial port");
        return 1;
    }

    ROS_INFO("serial connection successful");
#endif

    //Subscribe to ROS messages
    ucCommandMsg = rosNode.subscribe(SERIAL_CMD, 100, ucCommandCallback);

    //Setup to publish ROS messages
    ucResponseMsg = rosNode.advertise<std_msgs::String>(SERIAL_RSP, 100);
    rWheelEncoderMsg = rosNode.advertise<std_msgs::Int16>(SERIAL_R_WHEEL_ENCODER_VALUE, 100);
    lWheelEncoderMsg = rosNode.advertise<std_msgs::Int16>(SERIAL_L_WHEEL_ENCODER_VALUE, 100);

#ifndef MOCK_SERIAL
    //Create receive thread
    // err = pthread_create(&rcvThrID, NULL, rcvThread, NULL);
    // if (err != 0) {
    //     ROS_ERROR("unable to create receive thread");
    //     return 1;
    // }

    //create encoder thread
    err = pthread_create(&encThrID, NULL, encThread, NULL);
    if (err != 0) {
        ROS_ERROR("unable to create encoder thread");
        return 1;
    }
#endif

    //Process ROS messages and send serial commands to uController
    ros::spin();

#ifndef MOCK_SERIAL
    fclose(fpSerial);
#endif
    ROS_INFO("r2serial stopping");
    return 0;
}
