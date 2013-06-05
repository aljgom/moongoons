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
#define POSITION_MARGIN_OF_ERROR 15
#define ANTI_DURATION 0.3
#define WAIT_DURATION 0.3

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

// Sam's version of pulse ccw
void turnLeft(float duration_in_seconds, float power){
    // Start the motor
    mot_Run(0, power, 0, power);

    // Sleep - usleep uses microseconds
    usleep(duration_in_seconds * 1000000);

    // Stop the motor
	mot_Run(power, 0, power, 0);
	usleep(ANTI_DURATION * 1000000);
    mot_Run(0, 0, 0, 0);
	usleep(WAIT_DURATION * 1000000);
}

// Sam's version of pulse cw
void turnRight(float duration_in_seconds, float power){
    // Start the motor
    mot_Run(power, 0, power, 0);

    // Sleep - usleep uses microseconds
    usleep(duration_in_seconds * 1000000);

    // Stop the motor
    mot_Run(0, power, 0, power);
	usleep(ANTI_DURATION * 1000000);
    mot_Run(0, 0, 0, 0);
	usleep(WAIT_DURATION * 1000000);
}

// Drone flys up
void flyUp(float duration_in_seconds, float power){
    // Start the motor
    mot_Run(power, power, power, power);

    // Sleep - usleep uses microseconds
    usleep(duration_in_seconds * 1000000);

    // Stop the motor
    mot_Run(0, 0, 0, 0);
	usleep(WAIT_DURATION * 1000000);
}

void controller(){
    float default_duration = 2;
    float default_flyup_duration = 5.0;

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
        // Turn right for default duration at 0.01 power.
        turnRight(default_duration, 0.1);
        return;
    }

    // If we're here, red was detected. Find out if we turn right or left.
    if (curr_pos < -POSITION_MARGIN_OF_ERROR){
        // Turn right for default duration at 0.01 power.
        turnRight(default_duration, 0.1);
    } else if (curr_pos > POSITION_MARGIN_OF_ERROR){
        // Turn left for default duration at 0.01 power.
        turnLeft(default_duration, 0.1);
    } else{
        // Fly up for default duration at 0.1 power.
        flyUp(default_flyup_duration, 0.1);
    }

    // TODO: Make it so that we can detect oscillation between parameters.
    // Oscillation should result in the default duration getting incrementally
    // smaller. e.g 0.9, 0.8...
}

// Handle user input
void checkKeypress(){
    int c=tolower(util_getch());

    if(c=='q') stopLoop = true;
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
        if(waitToStart) continue;
        if(stopLoop) break;

        controller();

        //yield to other threads
        pthread_yield();
    }

    // Cleanup
    pthread_mutex_destroy(&video_results_mutex);
    mot_Close();
    printf("\nDone. Thanks for flying.\n");

    return 0;
}
