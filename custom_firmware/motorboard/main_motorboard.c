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
#include <arpa/inet.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <stdlib.h>  //exit()
#include <pthread.h>
#include <ctype.h>    /* For tolower() function */
#include <math.h>

// For...?
#include "../util/type.h"
#include "../util/util.h"

// Motor control library
#include "mot.h"

// Setting GPIO pins
#include "../gpio/gpio.h"

// Motor control
#include "../udp/udp.h"
#include "motorboard.h"

// For video stuff
#include "../video/video.h"

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
    vid.device = (char*)"/dev/video1";
    vid.w=176;
    vid.h=144;
    vid.n_buffers = 4;
    video_Init(&vid);

    // Keep a copy of the old image for efficiency reasons
    img_struct* img_old = video_CreateImage(&vid);
    img_struct* img_new = video_CreateImage(&vid);

    // To keep track of changes in the video feed
    int dx,dy;
    int x=0,y=0;

    // Load the old image
    video_GrabImage(&vid, img_old);

    // Blocking wait for a udp packet on port 7777
    udp_struct udpCmd;
    udpServer_Init(&udpCmd, 7777, 1);
    char buf[1024];
    printf("Waiting for UDP wakeup on port 7777\n");

    // TODO: EXPLAIN WHAT THIS DOES
    int bufcnt = udpServer_Receive(&udpCmd, buf, 1024);
    if (bufcnt <= 0) return 1;
    buf[bufcnt] = 0;
    printf("UDP wakeup received from %s\n",inet_ntoa(udpCmd.si_other.sin_addr));

    //Main loop
    while(1) {
        // 1. Get picture into buffer
        video_GrabImage(&vid, img_new);

        // 2. Update the image variables
        video_blocksum(img_old, img_new, &dx, &dy);
        x+=dx;
        y+=dy;
        printf("diff between img %5d and %5d -> dx=%2d dy=%2d x=%4d y=%4d\n",img_old->seq,img_new->seq,dx,dy,x,y);
        if(dx!=0 || dy!=0) {
            img_struct* tmp = img_new;
            img_new = img_old;
            img_old = tmp;
        }

        // 3. Send picture via udp with timeout TO sam's python server
        // A timeout is needed as we need to use ping/ack. Otherwise, our
        // program might freeze!! OR we need to use TCP.
        // ???? HOW TO DO THIS?
        // TODO: EXPERIMENT WITH JUST SENDING A PARAGRAPH OF TEXT OR SOMETHING
        // Another todo: WHAT FORMAT IS THIS PICTURE?!
        // WE CAN FIGURE THAT OUT LATER THOUGH.




        // 4. Wait for a response.
        // Use client side input instead
        // Wait for next packet on cmd port
        bufcnt = udpServer_Receive(&udpCmd, buf, 1024);
        if (bufcnt <= 0) continue;
        printf("buf is %s\n",buf);
        buf[bufcnt] = 0;
        printf("buf after 0 is %s\n",buf);
        char c = buf[0];

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

        //yield to other threads
        pthread_yield();
    }

    // If the program is done, return 0
    video_Close(&vid);
    mot_Close();
    printf("\nDone!\n");
    return 0;
}
