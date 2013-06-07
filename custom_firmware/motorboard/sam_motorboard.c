/* main_motorboard.c - AR.Drone motor demo program

    Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <cstdlib>
#include <pthread.h>
#include <ctype.h>    /* For tolower() function */
#include <math.h>

#include "../util/type.h"
#include "../util/util.h"
#include "sam_motorboard.h"
#include <TIME.H>
#include <sys/time.h>

// For video stuff
#include "../video/video.c"
#include <arpa/inet.h>

// Constants
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
#define NUM_BUFFERS 4

// Note: AR.Drone is server and listens for connection opening
// IP:192.168.1.1 Port: 7777
#define PORT_NUM 7777

// Drone lateral positioning calibration parameter
#define POSITION_MARGIN_OF_ERROR 5
#define ANTI_DURATION 1
#define WAIT_DURATION 1
#define DEFAULT_POWER 0.01

// NOTE: 0.5-0.9 max total for ascending in zero g.
// This is ADDITIONAL (to the defaults, default_n below)
#define FLYUP_POWER 0.8

// Direction dependent durations
#define LEFT_TURN_DURATION 1.3
#define LEFT_STOP_DURATION .15
#define RIGHT_TURN_DURATION 1.3
#define RIGHT_STOP_DURATION .15

// Cancellation of acceleration when we turn in search of red
#define ACCEL_CANCEL 0

#define TURN_DURATION .5
#define STOP_DURATION .5
#define SPACER_DURATION .5

#define DEFAULT_FLYUP_DURATION 5.0
#define ADDITIONAL_POWER 0.1

// Parameters for completely neutral flight of this drone
// Any drift should tend to go right (clockwise) 
// (but there should not be drift)
#define DEFAULT_1 0.1
#define DEFAULT_2 0.18
#define DEFAULT_3 0.1
#define DEFAULT_4 0.18

// Global motor speeds
float motor1 = DEFAULT_1;
float motor2 = DEFAULT_2;
float motor3 = DEFAULT_3;
float motor4 = DEFAULT_4;

///////////////////////////////////////////////////
// THREADING
pthread_mutex_t video_results_mutex = PTHREAD_MUTEX_INITIALIZER;

// Shared thread vars
int position_value = 9999;
time_t img_recv_timestamp;


// Global pthread for fetching image detection values
pthread_t image_processing_thread;

// END THREADING
///////////////////////////////////////////////////

// Global time counting
PositionTimePair prevTimePair;

// Global control vars
bool stopLoop = false;
bool waitToStart = true;

// Declare global variables for chopping image
double percent = 0.20;
int y_upper = VIDEO_WIDTH*VIDEO_HEIGHT/2*(1-percent);
int y_lower = VIDEO_WIDTH*VIDEO_HEIGHT/2*(1+percent)-1;
int cr_upper = VIDEO_WIDTH*VIDEO_HEIGHT/8*(9-percent);
int cr_lower = VIDEO_WIDTH*VIDEO_HEIGHT/8*(9+percent)-1;
int cb_upper = VIDEO_WIDTH*VIDEO_HEIGHT/8*(11-percent);
int cb_lower = VIDEO_WIDTH*VIDEO_HEIGHT/8*(11+percent)-1;
int chopped_size = (y_lower-y_upper+1)+(cr_lower-cr_upper+1)+(cb_lower-cb_upper+1);

// Exit and print error message
void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void turnRight(float seconds_to_sleep, float additional_power){
    // Add power to two motors
    motor2 = motor2 + additional_power;
    motor4 = motor4 + additional_power;
    mot_Run(motor1, motor2, motor3, motor4);

    // Sleep - usleep uses microseconds
    usleep(seconds_to_sleep * 1000000);
    motor2 = motor2 - additional_power;
    motor4 = motor4 - additional_power;
    mot_Run(motor1, motor2, motor3, motor4);
}

void turnLeft(float seconds_to_sleep, float additional_power){
    // Add power to two motors
    motor1 = motor1 + additional_power;
    motor3 = motor3 + additional_power;
    mot_Run(motor1, motor2, motor3, motor4);

    // Sleep - usleep uses microseconds
    usleep(seconds_to_sleep * 1000000);
    motor1 = motor1 - additional_power;
    motor3 = motor3 - additional_power;
    mot_Run(motor1, motor2, motor3, motor4);
}

void setAllMotors(float seconds_to_sleep, float additional_power){
    // Start the motor
    motor1 = DEFAULT_1 + additional_power;
    motor2 = DEFAULT_2 + additional_power;
    motor3 = DEFAULT_3 + additional_power;
    motor4 = DEFAULT_4 + additional_power;

    mot_Run(motor1, motor2, motor3, motor4);

    // Sleep
    sleep(seconds_to_sleep);
}

void controller(){
    // Fetch most recent position / timestamp
    PositionTimePair posTimePair = getPositionAndTimestamp();

    // If we already executed for this iteration, don't do anything
    if (prevTimePair.timestamp == posTimePair.timestamp) return;

    // Populate current and past positions
    int curr_pos = posTimePair.position;

    // Set previous timestamp
    prevTimePair = posTimePair;

    printf("Current Position: %i \n", curr_pos);

    // If we detect no red, turn a small amount.
    if(curr_pos == 9999){
        // Just turn left for now
        turnLeft(LEFT_TURN_DURATION, ADDITIONAL_POWER);

        // Add a little more time to the stopper than the original turn
        // command, to eliminate potential snowballing of acceleration
        // TUNE THIS IN FLIGHT: more velocity cancelation if 
        // the search turns are too big. Less if the drone isn't getting 
        // anywhere.
        turnRight(LEFT_STOP_DURATION + ACCEL_CANCEL, ADDITIONAL_POWER);

        // Return the motors to neutral speed
        setAllMotors(SPACER_DURATION, 0);
        return;
    }

    // If we're here, red was detected. Find out if we turn right or left.
    if (curr_pos < -POSITION_MARGIN_OF_ERROR){
        // Turn right for duration w/ additional power.
        turnRight(RIGHT_TURN_DURATION, ADDITIONAL_POWER);

        // Stop turning right by canceling out the velocity
        turnLeft(RIGHT_STOP_DURATION, ADDITIONAL_POWER);

        // Go to neutral
        setAllMotors(SPACER_DURATION, 0);
    } else if (curr_pos > POSITION_MARGIN_OF_ERROR){
        // Turn left for duration w/ additional power.
        turnLeft(LEFT_TURN_DURATION, ADDITIONAL_POWER);

        // Stop turning left by canceling out the velocity
        turnRight(LEFT_STOP_DURATION, ADDITIONAL_POWER);

        // Go to neutral
        setAllMotors(SPACER_DURATION, 0);
    } else{
        // Fly up for default duration at 0.1 additional power.
        setAllMotors(DEFAULT_FLYUP_DURATION, FLYUP_POWER);
    }

    // TODO: Make it so that we can detect oscillation between parameters.
    // Oscillation should result in the default duration getting incrementally
    // smaller. e.g 0.9, 0.8...
}

// Handle user input
void checkKeypress(){
    int c=tolower(util_getch());

    if(c=='q') stopLoop = true;
    if(c=='r') {
        turnRight(TURN_DURATION, 0.2);
        //turnLeft(STOP_DURATION, 0.1);
        setAllMotors(1, DEFAULT_POWER);
    }
    if(c=='l') {
        turnLeft(TURN_DURATION, 0.2);
        //turnRight(STOP_DURATION, 0.1);
        setAllMotors(1, DEFAULT_POWER);
    }
    if(c=='n') {
        // Neutral
        setAllMotors(0, 0);
    }
    if(c=='u') {
        // fly up
        setAllMotors(0, FLYUP_POWER);
    }
    if(c==' ') {
        printf("\rStop            ");
        mot_Stop();
    }
    if(c=='a') {
        printf("\rLeds off            ");
        mot_SetLeds(MOT_LEDOFF,MOT_LEDOFF,MOT_LEDOFF,MOT_LEDOFF);
    }
    if(c=='s') {
        waitToStart = false;
        //printf("\rLeds green            ");
        //mot_SetLeds(MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN);
    }
    if(c=='d') {
        printf("\rLeds orange            ");
        mot_SetLeds(MOT_LEDORANGE,MOT_LEDORANGE,MOT_LEDORANGE,MOT_LEDORANGE);
    }
    if(c=='f') {
        printf("\rLeds red            ");
        mot_SetLeds(MOT_LEDRED,MOT_LEDRED,MOT_LEDRED,MOT_LEDRED);
    }
    if(c=='s') {
        waitToStart = false;
        //printf("\rLeds green            ");
        //mot_SetLeds(MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN);
    }
    if(c=='p') {
        waitToStart = true;
    }

}

// Gets the position returned by the video
int getPosition()
{
    int position = 0;
    // Lock the value mutex
    pthread_mutex_lock(&video_results_mutex);

        // Retrieve the value
        position = position_value;


    // Unlock
    pthread_mutex_unlock(&video_results_mutex);

    // Returns 9999 if no angle found, otherwise returns between -50 and 50
    return position;
}

// Gets the value and timestamp returned by the video
PositionTimePair getPositionAndTimestamp()
{
    PositionTimePair pos_time_pair;

    // Lock the value mutex
    pthread_mutex_lock(&video_results_mutex);

        // Retrieve the value
        pos_time_pair.timestamp = img_recv_timestamp;
        pos_time_pair.position = position_value;

    // Unlock
    pthread_mutex_unlock(&video_results_mutex);

    // Returns a struct containing the last known position and timestamp
    return pos_time_pair;
}

// Get the time elapsed since last image in *SECONDS*
double getTimeSinceLastImage()
{
    double timeSinceLastImage = 0.0;
    // Lock the value mutex
    pthread_mutex_lock(&video_results_mutex);

    // Generate nowtime
    time_t now;
    time(&now);

    // Calculate difference in times
    timeSinceLastImage = difftime(now, img_recv_timestamp);

    pthread_mutex_unlock(&video_results_mutex);
    return timeSinceLastImage;
}

void * process_images(void * param)
{
    // Initialize used variables
    int n, sum;

    // Buffer for image data
    unsigned char * buf1;

    // Buffer for message passing info
    char buffer[4];


    // Initialize getting a picture
    vid_struct vid;

    // Device location
    // Video0 is front camera, video1 is bottom camera
    vid.device = (char*)"/dev/video0";

    // Other params for video
    vid.w = VIDEO_WIDTH;
    vid.h = VIDEO_HEIGHT;
    vid.n_buffers = NUM_BUFFERS;

    // Initialize video thread for streaming
    video_Init(&vid);

    // Create blank image
    img_struct * img = video_CreateImage(&vid);

    // INITIALIZE TCP CONNECTION


    // Socket file descriptor, new socket file descriptor, port number
    int sockfd, newsockfd, portno;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;

    // Open tcp socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
    }

    //zero-initialize serv_addr
    bzero((char *) &serv_addr, sizeof(serv_addr));

    //set port number
    portno = PORT_NUM;

    //set parameters for serv_addr
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);

    //bind socket
    if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        error("ERROR on binding");
    }

    //listen for client to open connection
    listen(sockfd,5);
    clilen = sizeof(cli_addr);

    //open new socket
    newsockfd = accept(sockfd,(struct sockaddr *) &cli_addr,&clilen);
    if (newsockfd < 0) {
        error("ERROR on accept");
    }

    // Now constantly fetch this and update the global variables
    while(1) {
        // Get picture into image buffer from video thread
        video_GrabImage(&vid, img);
        buf1 = img->buf;

        // Set image buffer
        unsigned char image[chopped_size];

        // Copy over data from buf1 to image
        memcpy(image, buf1 + y_upper, y_lower - y_upper + 1); // Copy Y values
        memcpy(image + (y_lower - y_upper + 1), buf1 + cr_upper, cr_lower - cr_upper + 1); // Copy Cr values
        memcpy(image + (y_lower - y_upper + 1)+ (cr_lower - cr_upper + 1), buf1 + cb_upper, cb_lower - cb_upper + 1); // Copy Cb values

        // Send packet to client
        n = 0;
        sum = 0;
        while (sum < chopped_size) {
            n = write(newsockfd, image + sum, chopped_size - sum);
            if (n < 0) {
                error("ERROR reading image data from socket!");
            }
            sum += n;
        }

        // Read 4 character return message from client
        bzero(buffer,4);
        n = 0;
        sum = 0;
        while (sum < 4) {
            n = read(newsockfd, buffer + sum, 4 - sum);
            if (n < 0) {
                error("ERROR reading client message!");
            }
            sum += n;
        }


        // Convert buffer to integer or NULL
        char none[] = "None";
        int equality = 0;

        //Check that buffer is "None"
        int check;
        for (check=0;check<4;check++) {
            if (buffer[check]==none[check]) {
                equality = 1;
            }
            else {
                equality = 0;
                break;
            }
        }

        // Message received is integer string if not equal
        if (equality==0) {
            // Lock the position value and update it + timestamp
            pthread_mutex_lock(&video_results_mutex);
            position_value = atoi(buffer);
            time(&img_recv_timestamp);
            pthread_mutex_unlock(&video_results_mutex);
        }
        else {
            // Lock the position value and update it
            pthread_mutex_lock(&video_results_mutex);
            position_value = 9999;
            time(&img_recv_timestamp);
            pthread_mutex_unlock(&video_results_mutex);
        }

        // Relinquish CPU before starting again
        pthread_yield();
    }

}

///////////////////////////////////////////////////////////////////////
//////////////////////// END DEFINITIONS AND INITIALIZATIONS //////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

// Begin control algorithm main method
int main()
{
    printf("Starting control algorithm...\r\n");
    mot_Init();

    // Kick off value getting thing in a separate thread!
    pthread_create(&image_processing_thread, NULL, process_images, NULL);

    prevTimePair = getPositionAndTimestamp();

    // Controller Loop
    while(1) {
        checkKeypress();
        if(stopLoop) break;
        if(!waitToStart) controller();

        //yield to other threads
        pthread_yield();
    }

    // Cleanup
    pthread_mutex_destroy(&video_results_mutex);
    mot_Close();
    printf("\nDone. Thanks for flying.\n");

    return 0;
}
