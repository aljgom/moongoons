/*
    main_motorboard.c - AR.Drone motor demo program

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
//#include <fcntl.h>   /* File control definitions */
//#include <errno.h>   /* Error number definitions */
//#include <termios.h> /* POSIX terminal control definitions */
//#include <stdlib.h>  //exit()

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
//#include "../video/video.h"
#include "../video/video.c"
#include <arpa/inet.h>

// Constants
#define IMAGE_DATA_SIZE 460800
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


int distance = 0;  
float pulseDuration = 1.3;//1.4;	
int k = 1;
int pulseAAccel = 30;
int pulseLAccel = k*pulseAAccel;
int minAAccel = pulseAAccel/3; //from initial measurements, looks like it's a third of pulseAAccel
int minLAccel = k*minAAccel;
bool correctError = true;
  
float throttle1 = 0.06;
float throttle2 = 0;
float throttle3 = 0.06;
float throttle4 = 0;
float step=0.01;
float speedSmallPulse = 15;
int prevAngle;
clock_t prevTime;

int angleGlobal;/*
int getAngle(){/*
	int ang;
	printf("angle?\n");
	scanf("%d",&ang);
	return ang;

	for(int i=0; i<10; i++){
	  usleep(.2/10*1000000);
	  checkKeypress();
    }
	printf("\nAngle:%i\n",angleGlobal);

	return angleGlobal;

}*/

void pulse(float dir,float t){
	if(dir>0)	pulseCounter(t);
	else		pulseClock(t);
}

void smallPulse(float dir,float angle){
	if(dir>0)	smallPulseCounter(abs(angle));
	else		smallPulseClock(abs(angle));
}

void turnOn(float dir){
	if(dir>0) 	turnOnCounter();
	else		turnOnClock();
}

float wait(float angle){

	// solve for t: angle = 2.5 * 1/2*pulseAAccel*(pulseDuration/1000)^2 + (pulseAAccel*pulseDuration/1000) * ( t  - pulseDuration/1000 )   ---- 2.5 for pulse beginning + pulse end + some extra to fix positioning at end of pulses
	int t = ( angle - 2.5 * 1/2 * pulseAAccel* pow(pulseDuration/1000,2) ) / (pulseAAccel*pulseDuration/1000)  + pulseDuration/1000;
	//console.log(angle + " " +t*1000)
	//return abs(t *1000);
	return abs(angle) *8/180- pulseDuration; // seconds
}


float smallWait(float angle){

	// solve for t: angle = 2.5 * 1/2*pulseAAccel*(pulseDuration/1000)^2 + (pulseAAccel*pulseDuration/1000) * ( t  - pulseDuration/1000 )   ---- 2.5 for pulse beginning + pulse end + some extra to fix positioning at end of pulses
	int t = ( angle - 2.5 * 1/2 * pulseAAccel*2/5* pow(pulseDuration/1000,2) ) / (pulseAAccel*2/5*pulseDuration/1000)  + pulseDuration/1000;
	//console.log(angle + " " +t*1000)
	return 600;//abs(t *1000)
}


void pulseClock(float t){
	mot_Run(.01,0,.01,0);
	usleep(t*1000000);
	mot_Run(0,0,0,0);
}

void pulseCounter(float t){
	mot_Run(0,.01,0,.01);
	usleep(t*1000000);
	mot_Run(0,0,0,0);
}

void turnOnClock(){	//starts with a pulse, then lowers speed
	mot_Run(.01,0,.01,0);
}

void turnOnCounter(){
	mot_Run(0,.01,0,.01);
}

void smallPulseClock(float t){	//starts with a pulse, then lowers speed
	float duration = .9; //1.1
	mot_Run(.01,0,.01,0);
	if( t < duration)	usleep(t*1000000);
	else			usleep(duration*1000000); 
	mot_Run(0,0,0,0);
	//mot_Run(.01,0,.01,0);
}

void smallPulseCounter(float t){
	float duration = .9; //1.1
	mot_Run(0,t,0,t);
	if( t < duration)	usleep(t*1000000);
	else				usleep(duration*1000000); 
	mot_Run(0,0,0,0);
	//mot_Run(0,.01,0,.01);
}

timeval t1, t2;
int previous_error = 0;
float integral = 0 ;
float prevDuration = 0;
void looper(vid_struct * vid, img_struct * img_new, int newsockfd){
	checkKeypress();
	/*int angle = getAngle();
	clock_t time = clock();
	float dir = angle != 0 ? angle/abs(angle) : 0;
	*/
	//printf("dir:%f",dir);
	//pid would go here
	printf("looping\n");
  printf("Angle: %i\n",getAngle(vid, img_new,newsockfd));
/*
  float dt = .2 + prevDuration;
  int angle = getAngle();
  integral = integral + angle*dt;
  float derivative = (angle - previous_error)/dt;
  float output = 1*angle + 1*integral + 1*derivative;
  previous_error = angle;
  smallPulse(-output/abs(output),abs(output)/90*.9); 
  
  prevDuration = abs(output)>90 ? .9 : abs(output)/90*.9;
  */


	/*
    float elapsedTime;

    gettimeofday(&t2, NULL);

    // compute and print the elapsed time in millisec
    elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
    


	printf("angle:%i  prevAngle:%i   t:%f\n",angle,prevAngle,(elapsedTime/1000));//(float)(time - prevTime)/CLOCKS_PER_SEC);
	float vel = (float)(angle - prevAngle)/(elapsedTime/1000);//( (float)(time - prevTime)/CLOCKS_PER_SEC );
	printf("veldir:%f\n   dir:%f\n",(vel)/abs(vel),dir);

	if( vel != 0 && (vel)/abs(vel) != dir ){	// still moving toward center
		printf("moving towards\n");	
		usleep( .1 * 1000000);
		looper();
		return;
	}
	smallPulse(dir,angle);						// increase speed for a moment		//maybe we can change the pulse durations if we do measurements for different accels?
	int newAngle = getAngle();


		printf("*********b");
	//if angle/vel < 2 seconds pulse again






	if( newAngle == prevAngle || (newAngle - prevAngle)/abs(newAngle - prevAngle) == dir ){	// still moving in farther from center after pulse
		prevAngle = newAngle;
		
		prevTime = clock();
		looper();
		return;
	}
		printf("*********c");
	usleep( angle/vel * 1000000);//  smallWait(angle) * 1000000);
	*/
	
}

void checkKeypress(){
 //handle user input
	int c=tolower(util_getch());
	//if(c=='q') break;
	if(c=='1') {
		printf("\rCounterclockwise pulse\n");
		pulseCounter(pulseDuration);
	}
	if(c=='2') {
		printf("\rClockwise pulse\n");
		pulseClock(pulseDuration);
	}
	if(c=='3') {
		smallPulseCounter(.1);
	}
	if(c=='4') {
		smallPulseClock(.1);
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



int getAngle(vid_struct * vid, img_struct * img_new, int newsockfd)
{
    printf("getAngle\n");
		int n;
		int sum;
        // Get picture into image buffer from video thread
        video_GrabImage(vid, img_new);
		printf("2\n");
		unsigned char * image = img_new->buf;
		printf("3\n");
		// Loop to send entire buffer to server
		char buffer[4];
       // Send packet to client
        n = 0;
        sum = 0;
        while (sum < IMAGE_DATA_SIZE) {
            n = write(newsockfd, image + sum, IMAGE_DATA_SIZE - sum);
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

	    printf("1\n");
  int angle = getAngle(&vid, img_new,newsockfd);
  printf("Angle: %i\r\n",angle);

  int dir = angle != 0 ? angle/abs(angle) : 0;
/*  pulse(dir,pulseDuration);		//first pulse
  
  printf("Wait: %f\r\n",wait(angle)*1000000);
  for(int i=0; i<100; i++){
	  usleep(wait(angle)/100*1000000);
	  checkKeypress();
  }
  pulse(-dir,pulseDuration -.04);
*/  //if(correctError){		
	  
	  /*
	turnOn(-dir);					// turn on oposite motors
	x=minAAccel;y=pulseDuration/1000;z=pulseAAccel;
	t= Math.sqrt( (z-x)*(Math.pow(y,2))/x ) ;console.log(x+" "+ y + " "+ z + " " +t)
	setTimeout(function(){
		turnOn(dir);							// first pulse
		setTimeout(looper,pulseDuration);					// fix errors
		//setTimeout( "document.location.reload(true)", 3000);
	},t*1000 );		 // solve for t:  1/2*minAAccel* t^2 =  1/2* (pulseAAccel-minAAccel) * pulseDuration^2  distance from third pulse equal to distance from minspeed of first turned on motors
  *///}	

  prevAngle = getAngle(&vid, img_new,newsockfd);
  prevTime = clock();
      // start timer
    gettimeofday(&t1, NULL);
  //main loop	

  while(1) {
	  looper(&vid, img_new,newsockfd);  
	
	//yield to other threads
	pthread_yield();
  }
	// If the program is done, return 0
	close(newsockfd); // Close TCP socket
	video_Close(&vid); // Close video thread
	mot_Close(); // Close motor thread
	printf("\nDone!\n");
	return 0;
}
