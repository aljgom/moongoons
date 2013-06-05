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
#include "mot.h"
#include <TIME.H>
#include <sys/time.h>

// For video stuff
#include "../video/video.c"
#include <arpa/inet.h>

// Constants
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
#define NUM_BUFFERS 4
#define PORT_NUM 7777

//used to exit and print error message
void error(const char *msg)
{
    perror(msg);
    exit(0);
}

// PID Controller Variables
int distance = 0;
float pulseDuration = 1.3;//1.4;
int k = 1;
int pulseAAccel = 30;
int pulseLAccel = k*pulseAAccel;

// From initial measurements, looks like it's a third of pulseAAccel
int minAAccel = pulseAAccel/3;

int minLAccel = k*minAAccel;
bool correctError = true;
bool stopLoop = false;

float throttle1 = 0.06;
float throttle2 = 0;
float throttle3 = 0.06;
float throttle4 = 0;
float step=0.01;
float speedSmallPulse = 15;
int prevAngle;
clock_t prevTime;


timeval t1, t2;
int previous_error = 0;
float integral = 0 ;
float prevDuration = 0;

int angleGlobal;

// Declare global variables for chopping image
double percent = 0.20;
int y_upper = VIDEO_WIDTH*VIDEO_HEIGHT/2*(1-percent);
int y_lower = VIDEO_WIDTH*VIDEO_HEIGHT/2*(1+percent)-1;
int cr_upper = VIDEO_WIDTH*VIDEO_HEIGHT/8*(9-percent);
int cr_lower = VIDEO_WIDTH*VIDEO_HEIGHT/8*(9+percent)-1;
int cb_upper = VIDEO_WIDTH*VIDEO_HEIGHT/8*(11-percent);
int cb_lower = VIDEO_WIDTH*VIDEO_HEIGHT/8*(11+percent)-1;
int chopped_size = (y_lower-y_upper+1)+(cr_lower-cr_upper+1)+(cb_lower-cb_upper+1);

// Motor Control Functions
void pulse(float dir,float t){
    if(dir>0)   pulseCCW(t);
    else        pulseCW(t);
}

void turnOn(float dir){
    if(dir>0)   turnOnCCW();
    else        turnOnCW();
}

// Returns a time in milliseconds representing how long to wait after the first pulse
// before pulsing again.
float wait(float angle){

    // solve for t: angle = 2.5 * 1/2*pulseAAccel*(pulseDuration/1000)^2 +
    // (pulseAAccel*pulseDuration/1000) * ( t  - pulseDuration/1000 )   ----
    // 2.5 for pulse beginning + pulse end + some extra to fix positioning at
    // end of pulses
    int t = ( angle - 2.5 * 1/2 * pulseAAccel* pow(pulseDuration/1000,2) ) / (pulseAAccel*pulseDuration/1000)  + pulseDuration/1000;

    // console.log(angle + " " +t*1000)
    // return abs(t *1000);

    // milliseconds
    return abs(angle) *8/180- pulseDuration;
}

// Returns a time in milliseconds
float smallWait(float angle){
    // solve for t: angle = 2.5 * 1/2*pulseAAccel*(pulseDuration/1000)^2 +
    // (pulseAAccel*pulseDuration/1000) * ( t  - pulseDuration/1000 )   ----
    // 2.5 for pulse beginning + pulse end + some extra to fix positioning at
    // end of pulses

    int t = ( angle - 2.5 * 1/2 * pulseAAccel*2/5* pow(pulseDuration/1000,2) ) / (pulseAAccel*2/5*pulseDuration/1000)  + pulseDuration/1000;

    // console.log(angle + " " +t*1000)
    // abs(t *1000)
    return 600;
}

// Pulse motors such that drone moves clockwise
void pulseCW(float t){
    mot_Run(.01,0,.01,0);
    usleep(t*1000000);
    mot_Run(0,0,0,0);
}

// Pulse motors such that drone moves counterclockwise
void pulseCCW(float t){
    mot_Run(0,.01,0,.01);
    usleep(t*1000000);
    mot_Run(0,0,0,0);
}

// Turn on functions: start spinning the motors then reduce to low speed

// Clockwise
void turnOnCW(){
    mot_Run(.01,0,.01,0);
}

// Counterclockwise
void turnOnCCW(){
    mot_Run(0,.01,0,.01);
}

void smallPulse(float dir,float angle){
    if(dir>0)   smallPulseCCW(abs(angle));
    else        smallPulseCW(abs(angle));
}

// Small clockwise motor pulse
void smallPulseCW(float t){  //starts with a pulse, then lowers speed
    float duration = 9; //1.1
    mot_Run(.01,0,.01,0);
    if( t < duration)   usleep(t*1000000);
    else            usleep(duration*1000000);
    mot_Run(0,0,0,0);
    //mot_Run(.01,0,.01,0);
}

// Small counterclockwise motor pulse
void smallPulseCCW(float t){
    float duration = 9; //1.1
    mot_Run(0,t,0,t);
    if( t < duration)   usleep(t*1000000);
    else                usleep(duration*1000000);
    mot_Run(0,0,0,0);
    //mot_Run(0,.01,0,.01);
}

void looper(vid_struct * vid, img_struct * img_new, int newsockfd){
	checkKeypress();
/*clock_t time = clock();
	int angle = getAngle(vid, img_new,newsockfd);
	float dir = angle != 0 ? angle/abs(angle) : 0;
*/
	//printf("dir:%f",dir);
	//pid would go here
  printf("\n");
  
  float dt=.4;
  int angle = getAngle(vid, img_new,newsockfd);	
  printf("Angle: %i      prevAngle: %i\n",angle,prevAngle);	  
  if(angle == 9999) return;
  float vel = prevAngle == 9999 ? 0 : (float)(angle - prevAngle)/(dt);//( (float)(time - prevTime)/CLOCKS_PER_SEC );
  printf("Vel: %f\n",vel);	  
  float error = vel - ( - 2 * angle/50 );	  
  integral = integral*.5 + error*dt;
  float derivative = (error - previous_error)/dt;
  float output = - 1.5*error; // - 1*integral //- 1*derivative/dt;
  printf("Error: %f       Output: %f\n",error,output);	 
  previous_error = error;
  float dir = output == 0 ? 0 : -output/abs(output);
  float pulseStrength=abs(output);///90;//*.9);;
  printf("smallPulse(%f,%f)\n",dir,pulseStrength);
  //smallPulse(dir,pulseStrength;); 
  
  prevDuration = abs(output)>90 ? .9 : abs(output)/90*.9;
  prevAngle = angle;

	/*

    float elapsedTime;

    gettimeofday(&t2, NULL);

    // compute and print the elapsed time in millisec
    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms



    printf("angle:%i  prevAngle:%i   t:%f\n",angle,prevAngle,(elapsedTime/1000));//(float)(time - prevTime)/CLOCKS_PER_SEC);
    float vel = (float)(angle - prevAngle)/(elapsedTime/1000);//( (float)(time - prevTime)/CLOCKS_PER_SEC );
    printf("veldir:%f\n   dir:%f\n",(vel)/abs(vel),dir);

    if( vel != 0 && (vel)/abs(vel) != dir ){
        // still moving toward center
        printf("moving towards\n");
        usleep( .1 * 1000000);
        looper();
        return;
    }

    // increase speed for a moment
    smallPulse(dir,angle);
    //maybe we can change the pulse durations if we do measurements for different accels?
    int newAngle = getAngle();


        printf("*********b");
    //if angle/vel < 2 seconds pulse again

    if( newAngle == prevAngle || (newAngle - prevAngle)/abs(newAngle - prevAngle) == dir ){ // still moving in farther from center after pulse
        prevAngle = newAngle;

        prevTime = clock();
        looper();
        return;
    }
        printf("*********c");
    usleep( angle/vel * 1000000);//  smallWait(angle) * 1000000);
    */

}

// Handle user input
void checkKeypress(){
    int c=tolower(util_getch());

    if(c=='q') stopLoop = true;

    if(c=='1') {
        printf("\rCounterclockwise pulse\n");
        pulseCCW(pulseDuration);
    }
    if(c=='2') {
        printf("\rClockwise pulse\n");
        pulseCW(pulseDuration);
    }
    if(c=='3') {
        smallPulseCCW(.1);
    }
    if(c=='4') {
        smallPulseCW(.1);
    }
    if(c=='5') {
        printf("\rRun All Motors 50%            ");
        throttle1 = .01;
        throttle2 = .01;
        throttle3 = .01;
        throttle4 = .01;
        mot_Run(throttle1,throttle2,throttle3,throttle4);
    }
    if(c==',') {/*
        printf("\rThrottle down            ");
        if(throttle1>step) throttle1 -= step;
        if(throttle2>step) throttle2 -= step;
        if(throttle3>step) throttle3 -= step;
        if(throttle4>step) throttle4 -= step;
        mot_Run(throttle1,throttle2,throttle3,throttle4);*/
        angleGlobal--;
        printf("a:%i\n",angleGlobal);
    }

    if(c=='j') {
        printf("\rClockwise            ");
        if(throttle1==.40) throttle1 = .80; else throttle1 = .40;
        if(throttle3==.25) throttle3 = .50; else throttle3 = .25;
        mot_Run(throttle1,throttle2,throttle3,throttle4);
    }
    if(c=='k') {
        printf("\rCounterclockwise             ");
        if(throttle2==.25) throttle2 = .50; else throttle2 = .25;
        if(throttle4==.25) throttle4 = .50; else throttle4 = .25;
        mot_Run(throttle1,throttle2,throttle3,throttle4);
    }

    if(c=='.') {/*
        printf("\rThrottle up  %f          ",throttle2+step);
        if(throttle1>0) throttle1 += step;
        if(throttle2>0) throttle2 += step;
        if(throttle3>0) throttle3 += step;
        if(throttle4>0) throttle4 += step;
        mot_Run(throttle1,throttle2,throttle3,throttle4);*/
        angleGlobal++;
        printf("a:%i\n",angleGlobal);
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
        printf("\rLeds green            ");
        mot_SetLeds(MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN,MOT_LEDGREEN);
    }
    if(c=='d') {
        printf("\rLeds orange            ");
        mot_SetLeds(MOT_LEDORANGE,MOT_LEDORANGE,MOT_LEDORANGE,MOT_LEDORANGE);
    }
    if(c=='f') {
        printf("\rLeds red            ");
        mot_SetLeds(MOT_LEDRED,MOT_LEDRED,MOT_LEDRED,MOT_LEDRED);
    }

}

// Gets the value returned by the video
int getAngle(vid_struct * vid, img_struct * img_new, int newsockfd)
{
    printf("getAngle\n");
    int n;
    int sum;
    // Get picture into image buffer from video thread
    video_GrabImage(vid, img_new);
    unsigned char * buf1 = img_new->buf;

    // Set image buffer
    unsigned char image[chopped_size];
		
    // Copy over data from buf1 to image
    memcpy(image,buf1+y_upper,y_lower-y_upper+1); // Copy Y values
    memcpy(image+(y_lower-y_upper+1),buf1+cr_upper,cr_lower-cr_upper+1); // Copy Cr values
    memcpy(image+(y_lower-y_upper+1)+(cr_lower-cr_upper+1),buf1+cb_upper,cb_lower-cb_upper+1); // Copy Cb values

    // Loop to send entire buffer to server
    char buffer[4];

    // Send packet to client
    n = 0;
    sum = 0;
    while (sum < chopped_size) {
        n = write(newsockfd, image + sum, chopped_size - sum);
        if (n < 0) {
            error("ERROR reading image data from socket!");
        }
        printf("resend %d bytes\n",n);
        sum += n;
    }

    // Read 4 character message from client
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

    /*
    Convert buffer to integer or NULL
    */
    char none[] = "None";
    int equal = 0;

    //Check that buffer is "None"
    int check;
    for (check=0;check<4;check++) {
        if (buffer[check]==none[check]) {
            equal = 1;
        }
        else {
            equal = 0;
            break;
        }
    }

    // Message received is integer string if not equal
    if (equal==0) {
        return atoi(buffer);
    }
    else {
        return 9999;
    }

}

// Begin control algorithm main method
int main()
{
    printf("Start of Control\r\n");
    mot_Init();

    // Initialize getting a picture
    vid_struct vid;

    // Video0 is front camera, video1 is bottom camera
    vid.device = (char*)"/dev/video0";

    // Other params for video
    vid.w = VIDEO_WIDTH;
    vid.h = VIDEO_HEIGHT;
    vid.n_buffers = NUM_BUFFERS;

    // Initialize video thread for streaming
    video_Init(&vid);

    // Create blank image
    img_struct * img_new = video_CreateImage(&vid);

    /*
    INITIALIZE TCP CONNECTION
    Note: AR.Drone is server and listens for connection opening
    IP:192.168.1.1 Port: 7777
    */

    // Socket file descriptor, new socket file descriptor, port number
    int sockfd, newsockfd, portno;
    socklen_t clilen;

    // Declare additional variables
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

    int angle = getAngle(&vid, img_new,newsockfd);
    printf("Angle: %i\r\n",angle);

    int dir = angle != 0 ? angle/abs(angle) : 0;
    /*
    //first pulse
    pulse(dir,pulseDuration);

    printf("Wait: %f\r\n",wait(angle)*1000000);
    for(int i=0; i<100; i++){
        usleep(wait(angle)/100*1000000);
        checkKeypress();
    }
    pulse(-dir,pulseDuration -.04);
    */

    /*
    if(correctError){
    // turn on oposite motors
    turnOn(-dir);
    x=minAAccel;y=pulseDuration/1000;z=pulseAAccel;
    t= Math.sqrt( (z-x)*(Math.pow(y,2))/x ) ;console.log(x+" "+ y + " "+ z + " " +t)
    setTimeout(function(){
    // first pulse
    turnOn(dir);
    setTimeout(looper,pulseDuration);
    // fix errors
    //setTimeout( "document.location.reload(true)", 3000);
    },t*1000 );
    // solve for t:  1/2*minAAccel* t^2 =  1/2* (pulseAAccel-minAAccel) * pulseDuration^2  distance from third pulse equal to distance from minspeed of first turned on motors
    }*/

    prevAngle = getAngle(&vid, img_new,newsockfd);
    prevTime = clock();

    // start timer
    gettimeofday(&t1, NULL);

    //main loop
    while(1) {
		if(stopLoop) break;
        looper(&vid, img_new,newsockfd);

        //yield to other threads
        pthread_yield();
    }

    // Cleanup
	close(sockfd);
    close(newsockfd); // Close TCP socket
    video_Close(&vid); // Close video thread
    mot_Close(); // Close motor thread
    printf("\nDone!\n");

    return 0;
}
