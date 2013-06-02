/*
    main_motorboard.c - AR.Drone motor demo program

    Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com

    Modified by Moon Goons NASA Microgravity Team (2013)

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
#include <arpa/inet.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <stdlib.h>  //exit()
#include <pthread.h>
#include <ctype.h>    /* For tolower() function */
#include <math.h>

// For definitions
#include "../util/type.h"
#include "../util/util.h"

// Motor control library
#include "mot.h"

// Setting GPIO pins
#include "../gpio/gpio.h"

// Motor control
#include "motorboard.h"

// For video stuff
#include "../video/video.h"
#include "../video/video.c"

//used to exit and print error message
void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main()
{
    printf("Motorboard Test Program ... Press q to quit\r\n");
    printf("Motor: 1,2,3,4=run single motor at 50%  5=run all motors at 50% ,=throttle up 1%  .=throttle down 1%  space=stop\r\n");
    printf("Leds:  a=loff  s=green  d=orange  f=red\r\n");

    mot_Init();

    float throttle1 = 0;
    float throttle2 = 0;
    float throttle3 = 0;
    float throttle4 = 0;
    float step=0.01;

    // Initialize getting a picture
    vid_struct vid;

    // Video0 is front camera, video1 is bottom camera
    vid.device = (char*)"/dev/video0";

    // Other params for video
    vid.w=640;
    vid.h=480;
    vid.n_buffers = 4;
    
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
    int n;

    // Open tcp socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
    }

    //zero-initialize serv_addr
    bzero((char *) &serv_addr, sizeof(serv_addr));

    //set port number
    portno = 7777;

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

    //Main loop
    while(1) {
        // Get picture into image buffer from video thread
        video_GrabImage(&vid, img_new);
		unsigned char * image = img_new->buf;

		// Loop to send entire buffer to server
        char buffer[4];
        unsigned char packet[9216];
        int m;
        for (m=0;m<50;m++) {
            // Load packet buffer to send to server
            int p = 0;
            while (p<9216) {
                packet[p] = image[m*9216+p];
                p++;
            }

   	    // Send packet to client
            n = write(newsockfd,packet,9216);//strlen(packet)
            if (n < 0) {
                error("ERROR writing to socket");
            }
			
	    // Make sure all 9216 bytes have been sent
            // If not, resend rest of bytes
            int sum = n;
            while (sum < 9216) {
                n = write(newsockfd,packet+sum,9216-sum);
                if (n < 0) {
                    error("ERROR reading from socket");
                }
                sum += n;
			}
		}

		/* 
		GET ANGLE DISPLACEMENT FROM IMAGE PROCESSING CLIENT
		*/
	
		// Read message from client
        bzero(buffer,4);
        n = read(newsockfd,buffer,4);
        if (n < 0) {
            error("ERROR reading from socket");
        }

		//Make sure all 5 bytes have been read
        // If not, reread rest of bytes
        int sum = n;
        while (sum < 4) {
            n = read(newsockfd,buffer+sum,4-sum);
            if (n < 0) {
                error("ERROR reading from socket");
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
			int ang_dspl = atoi(buffer);
		}
		else {
			int ang_dspl = NULL;
		}
		
	/*
	ANGLE DISPLACEMENT MESSAGE IS IN buffer
	USE THIS SECTION TO CONVERT/EXTRACT DISPLACEMENT
	FROM buffer AND PROCESS IN PID CONTROLLER 
	*/
		/*
        if(c=='q') break;
        if(c=='1') {
            printf("\rRun Motor1 50%            ");
            throttle1 = .50;
            throttle2 = 0;
            throttle3 = 0;
            throttle4 = 0;
            mot_Run(throttle1,throttle2,throttle3,throttle4);
        }
        if(c=='2') {
            printf("\rRun Motor2 50%            ");
            throttle1 = 0;
            throttle2 = .50;
            throttle3 = 0;
            throttle4 = 0;
            mot_Run(throttle1,throttle2,throttle3,throttle4);
        }
        if(c=='3') {
            printf("\rRun Motor3 50%            ");
            throttle1 = 0;
            throttle2 = 0;
            throttle3 = .50;
            throttle4 = 0;
            mot_Run(throttle1,throttle2,throttle3,throttle4);
        }
        if(c=='4') {
            printf("\rRun Motor4 50%            ");
            throttle1 = 0;
            throttle2 = 0;
            throttle3 = 0;
            throttle4 = .50;
            mot_Run(throttle1,throttle2,throttle3,throttle4);
        }
        if(c=='5') {
            printf("\rRun All Motors 50%            ");
            throttle1 = .50;
            throttle2 = .50;
            throttle3 = .50;
            throttle4 = .50;
            mot_Run(throttle1,throttle2,throttle3,throttle4);
        }
        if(c==',') {
            printf("\rThrottle down            ");
            if(throttle1>step) throttle1 -= step;
            if(throttle2>step) throttle2 -= step;
            if(throttle3>step) throttle3 -= step;
            if(throttle4>step) throttle4 -= step;
            mot_Run(throttle1,throttle2,throttle3,throttle4);
        }
        if(c=='.') {
            printf("\rThrottle up            ");
            if(throttle1>0) throttle1 += step;
            if(throttle2>0) throttle2 += step;
            if(throttle3>0) throttle3 += step;
            if(throttle4>0) throttle4 += step;
            mot_Run(throttle1,throttle2,throttle3,throttle4);
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
		*/
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
