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

// For navdata
#include "../navboard/navboard.c"
#include "../navboard/navboard.h"
// Constants
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
#define NUM_BUFFERS 4
#define PORT_NUM 7777

// THREADING
pthread_mutex_t video_results_mutex = PTHREAD_MUTEX_INITIALIZER;

// SHARED THREAD VARS
int position_value = 9999;
time_t img_recv_timestamp;


// Global pthread for fetching image detection values
pthread_t image_processing_thread;



/*****************************************************************/
/*****************************************************************/
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
bool waitToStart = true;

float throttle1 = 0.06;
float throttle2 = 0;
float throttle3 = 0.06;
float throttle4 = 0;
float step=0.3;
float speedSmallPulse = 15;
int prevAngle;
clock_t prevTime;
time_t prevTimestamp = 0;


timeval t1, t2;
int previous_error = 0;
float integral = 0 ;
float prevDuration = 0;
int counter = 0;
bool UseOnlySingleMotor = false;
float straightSpeed = .01;

int angleGlobal;
float smallPulseSpeed = .3;
/*****************************************************************/
/*****************************************************************/


// Declare global variables for chopping image
double percent = 0.20;
int y_upper = VIDEO_WIDTH*VIDEO_HEIGHT/2*(1-percent);
int y_lower = VIDEO_WIDTH*VIDEO_HEIGHT/2*(1+percent)-1;
int cr_upper = VIDEO_WIDTH*VIDEO_HEIGHT/8*(9-percent);
int cr_lower = VIDEO_WIDTH*VIDEO_HEIGHT/8*(9+percent)-1;
int cb_upper = VIDEO_WIDTH*VIDEO_HEIGHT/8*(11-percent);
int cb_lower = VIDEO_WIDTH*VIDEO_HEIGHT/8*(11+percent)-1;
int chopped_size = (y_lower-y_upper+1)+(cr_lower-cr_upper+1)+(cb_lower-cb_upper+1);

// Declare global variables to ignore navdata check
int ignore_navdata = 0;
int ignore_datawrite = 0;

// Exit and print error message
void error(const char *msg)
{
    perror(msg);
    exit(0);
}


// Motor Control Functions
void pulse(float dir,float t){
    if(dir>0)   pulseCCW(t);
    else        pulseCW(t);
}

void turnOn(float dir){
    if(dir>0)   turnCCW(straightSpeed);
    else        turnCW(straightSpeed);
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
bool cWSwitcher = true;
bool cCWSwitcher = false;
void pulseCW(float t){
	turnCW(smallPulseSpeed);
	usleep(t*1000000);
    stopMotors();
}

// Pulse motors such that drone moves counterclockwise
void pulseCCW(float t){
	turnCCW(smallPulseSpeed);
	usleep(t*1000000);
    stopMotors();
}

// Turn on functions: start spinning the motors then reduce to low speed



void stopMotors(){
	mot_Run(0,0,0,0);	
}
void turnCCW(float speed){
	mot_Run(speed,straightSpeed,speed,straightSpeed);	
}
void turnCW(float speed){
	mot_Run(straightSpeed,speed,straightSpeed,speed);	
}
float maxDuration = .5;
void smallPulse(float dir,float duration){
	if(duration > maxDuration) duration = maxDuration;
    printf("dir %f,  duration %f\n",dir,duration);
    if(dir>0)   smallPulseCW(duration);
    else        smallPulseCCW(duration);
}

// Small clockwise motor pulse
void smallPulseCW(float t){  //starts with a pulse, then lowers speed
	printf("Pulse Clockwise\n");
    if(UseOnlySingleMotor){ //changed
        if(cWSwitcher)  mot_Run(straightSpeed,straightSpeed,straightSpeed,smallPulseSpeed);
		else			mot_Run(straightSpeed,smallPulseSpeed,straightSpeed,straightSpeed);
		cWSwitcher = !cWSwitcher;
    }
    else{
       turnCW(smallPulseSpeed);
    }
    usleep(t*1000000);
    goStraight(straightSpeed);
}

// Small counterclockwise motor pulse
void smallPulseCCW(float t){
    printf("Pulse Counterclockwise\n");
	if(UseOnlySingleMotor){
        if(cCWSwitcher) mot_Run(straightSpeed,straightSpeed,smallPulseSpeed,straightSpeed);
		else			mot_Run(smallPulseSpeed,straightSpeed,straightSpeed,straightSpeed);
		cCWSwitcher = !cCWSwitcher;
    }
    else{ 
		turnCCW(smallPulseSpeed);
    }
    usleep(t*1000000);
    goStraight(straightSpeed);
}
void goStraight(float speed){
	// make it not spin	.1, 1.8 , 1 ,1.8
	mot_Run(speed,speed,speed,speed);	
}



void pid_controller(FILE * data_file){
    // First thing: check for user input


    // Control algorithm stuff
    // compute and print the elapsed time in millisec

	PositionTimePair posTimePair = getPositionAndTimestamp();
	if(prevTimestamp == posTimePair.timestamp) return;
	
	gettimeofday(&t2, NULL);
    float dt = (t2.tv_sec - t1.tv_sec) ;      // sec
    dt += (t2.tv_usec - t1.tv_usec) / 1000000.0;   // us to s
    t1 = t2;

	printf("\n");
	prevTimestamp = posTimePair.timestamp;
    int angle = posTimePair.position;

	// Write to file if ignore data not set
	if (ignore_datawrite == 0) {
		// Convert timestamp to string
		char buff[20];
		time_t now = posTimePair.timestamp;
		strftime(buff, 20, "%Y-%m-%d_%H:%M:%S", localtime(&now));
	
		fprintf(data_file,buff); // Write timestamp to file
	}
	
    printf("Angle: %i      prevAngle: %i    dt: %f \n",angle,prevAngle,dt);
	
	// Write to file if ignore data not set
	if (ignore_datawrite == 0) {
		fprintf(data_file,"\nAngle: %i      prevAngle: %i    dt: %f \n",angle,prevAngle,dt);
    }
	
	if(angle == 9999){
        integral = 0;
        counter = (counter+1);
        if(counter%5 == 4) smallPulse(1,.5);
        return;
    }
    else{
        if(counter){
	    int x;
            for(x = 0; x < counter/10; x++){		//half as many pulses in opposite direction
                smallPulse(-1, .5);
            }
            counter = 0;
        }
	}
	
    // Velocity Calculation
    //( (float)(time - prevTime)/CLOCKS_PER_SEC );
    float vel = prevAngle == 9999 ? 0 : (float)(angle - prevAngle)/(dt);
    printf("Vel: %f\n",vel);
	
	// Write to file if ignore data not set
	if (ignore_datawrite == 0) {
		fprintf(data_file,"Vel: %f\n",vel);
	}
	
	if( abs(angle) < 10 && fabsf(vel) <10 ) 	{
		usleep(.1*1000000);		//not sure if this line necessary? (to prevent pulses from merging)
		goStraight(straightSpeed);
		usleep(.1*1000000);
	}
    // Error Calculation
    float error = vel - ( - 2 * float(angle)/50 );
    integral = integral*.8 + error*dt;
    float derivative = (error - previous_error)/dt;
    float output =  -2*error - 3*integral - 1*derivative;
    printf("Error: %f     integral: %f     derivative: %f  \n", error,integral,derivative);
	printf("Prev error: %f    Output: %f  \n",previous_error,output);
	
	// Write data to file if ignore data not set
	if (ignore_datawrite == 0) {
		fprintf(data_file,"Error: %f     integral: %f     derivative: %f  \n", error,integral,derivative);
		fprintf(data_file,"Prev error: %f    Output: %f  \n",previous_error,output);
	}
	
	// Print what pulse was given as a response
    previous_error = error;
    float dir = output == 0 ? 0 : output/fabsf(output);
    float pulseStrength = fabsf(output)/120 * .7 ;		// try to keep the range between .8 and 1.5
    printf("smallPulse(%f,%f)\n",dir,pulseStrength);
	
	// Write to file if ignore data not set
	if (ignore_datawrite == 0) {
		fprintf(data_file,"smallPulse(%f,%f)\n",dir,pulseStrength);
	}
	
//    if(output < 150){										// sometimes output is huge randomly? TODO: find out why
		smallPulse(dir,pulseStrength);
//	}
    //usleep(.1 * 1000000);

    prevDuration = fabsf(output)>90 ? .9 : fabsf(output)/90*.9;
    prevAngle = angle;
}

// Handle user input
void checkKeypress(){
    int c=tolower(util_getch());
	if(c=='n') { // Ignore navdata
		printf("\nIgnoring Navdata\n");
		ignore_navdata = 1;
	}
	if(c=='w') { // Ignore write data to file
		printf("\nIgnore writing data to file\n");
		ignore_datawrite = 1;
	}
    if(c=='q') {
		stopLoop = true;
	}
    if(c=='1') {
        printf("\rCounterclockwise pulse\n");
        pulseCCW(2);
    }
    if(c=='2') {
        printf("\rClockwise pulse\n");
        pulseCW(2);
    }
    if(c=='3') {
        smallPulseCCW(.1);
    }
    if(c=='4') {
        smallPulseCW(.1);
    }
    if(c=='5') {
        printf("\rRun All Motors 50%            ");
        throttle1 = .3;
        throttle2 = .3;
        throttle3 = .3;
        throttle4 = .3;
        mot_Run(throttle1,throttle2,throttle3,throttle4);
    }
    if(c==',') {
        // printf("\rThrottle down            ");
        // if(throttle1>step) throttle1 -= step;
        // if(throttle2>step) throttle2 -= step;
        // if(throttle3>step) throttle3 -= step;
        // if(throttle4>step) throttle4 -= step;
        // mot_Run(throttle1,throttle2,throttle3,throttle4);
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

    if(c=='.') {
        // printf("\rThrottle up  %f          ",throttle2+step);
        // if(throttle1>0) throttle1 += step;
        // if(throttle2>0) throttle2 += step;
        // if(throttle3>0) throttle3 += step;
        // if(throttle4>0) throttle4 += step;
        // mot_Run(throttle1,throttle2,throttle3,throttle4);
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

// Gets the value returned by the video
int getAngle()
{
    int angle = 0;
    // Lock the value mutex
    pthread_mutex_lock(&video_results_mutex);

        // Retrieve the value
        angle = position_value;


    // Unlock
    pthread_mutex_unlock(&video_results_mutex);

    // Returns 9999 if no angle found, otherwise returns between -50 and 50
    return angle;
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
    // Note: AR.Drone is server and listens for connection opening
    // IP:192.168.1.1 Port: 7777

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

// Use navigation data to set motors
void checkNavdata (nav_struct * nav,float * nav_posptr,FILE * data_file)
{
		// Get navdata sample
		int rc = nav_GetSample(nav);
		if(rc) {
			printf("ERROR: nav_GetSample return code=%d\n",rc); 
		}
		
		// Get timestamp and convert to string
		char buff[20];
		PositionTimePair cur_timepair = getPositionAndTimestamp();
		time_t now = cur_timepair.timestamp;
		strftime(buff, 20, "%Y-%m-%d_%H:%M:%S", localtime(&now));
		
		// Print timestamp into file
		fprintf(data_file,buff);
		
		// Print Navdata 
		nav_Print(nav,data_file);
		
		// Calculate position
		*nav_posptr += (RAD2DEG(nav->gz))*(nav->dt);
		
		fprintf (data_file,"new position is %f and gyro is %f\n",*nav_posptr,nav->gy);
}

///////////////////////////////////////////////////////////////////////
//////////////////////// END DEFINITIONS AND INITIALIZATIONS //////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

// Begin control algorithm main method
int main()
{
    printf("Start of Control\r\n");
    mot_Init();

	// Declare variables for navdata
	int rc;
	nav_struct nav;
	
	// Initialize navdata
	rc = nav_Init(&nav);
	
	// Check if navdata initializes
	int nav_fail = 0;
	if (rc==1) {
		printf("navdata failed to initialize\n");
		nav_fail = 1;
	}
	
	// Calibrate navdata
	rc=nav_FlatTrim();
	if(rc) {
		printf("Failed navdata: retcode=%d\r\n",rc); 
		nav_fail = 1;
	}
	
	// Initialize Nav position
	float nav_position = 0;
	
	// Open file for write to store data
	char buff[20];
	time_t now = time(NULL);
	strftime(buff, 20, "%Y-%m-%d_%H:%M:%S", localtime(&now));
	FILE * data_fd;
	data_fd = fopen(buff,"w");
    
	// Kick off value getting thing in a separate thread!
    pthread_create(&image_processing_thread, NULL, process_images, NULL);


    int angle = getAngle();
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

    prevAngle = angle;
    // start timer
    gettimeofday(&t1, NULL);

    // PID Loop
    float s = .3;
    dir= 1;
	mot_Run(straightSpeed,straightSpeed,straightSpeed,straightSpeed);
	bool runningMotors = true;
    while(1) {
        checkKeypress();
		if(waitToStart){
			mot_Run(0,0,0,0);
			runningMotors = false;
			continue;
		}
		if(!runningMotors){
			mot_Run(straightSpeed,straightSpeed,straightSpeed,straightSpeed);
			runningMotors = true;
		}
        if(stopLoop) break;
		
		// Check navdata if navdata initiates & ignore navdata not set
		if (nav_fail == 0 && ignore_navdata == 0 && waitToStart == false) {
			checkNavdata(&nav,&nav_position,data_fd);
		}
        
		pid_controller(data_fd);
/*      smallPulse(dir,.9);
        usleep(s * 1000000);
        smallPulse(dir,.9);
        usleep(1.5 * 1000000);
        printf("%f\n",s);
        if(dir > 0) dir = -1; else dir = 1;
        s+=.3;
        smallPulse(dir,.9);
        usleep(s * 1000000);
        smallPulse(dir,.9);
        usleep(1.5 * 1000000);
        printf("%f\n",s);
        s+=.3;
*/
        //yield to other threads
        pthread_yield();
    }

    // Cleanup
    // Delete the mutex
    pthread_mutex_destroy(&video_results_mutex);
    //close(sockfd);
    //close(newsockfd); // Close TCP socket
    //video_Close(&vid); // Close video thread
	fclose(data_fd);
    mot_Close(); // Close motor thread
	nav_Close(); // Close navdata
    printf("\nDone!\n");

    return 0;
}
