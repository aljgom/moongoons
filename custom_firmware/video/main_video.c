/*
    main_video.h - video driver demo program

    Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com

	Modified by Moon Goons NASA Microgravity Team
	
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "video.h"
#include <unistd.h>
#include <netinet/in.h>
#include <netdb.h> 

void video_cg(img_struct* img) 
{
	int h=img->h;
	int w=img->w;
	int n=w*h;
	
	int p=0;
	int px=0;
	int py=0;
	int x=0;
	int y=0;
	unsigned char* buf = img->buf;
	for(int i=0;i<n;i++) {
		p+=buf[i];
		px+=x*buf[i];
		py+=y*buf[i];
		x++;
		if(x==w) {
			x=0;
			y++;
		}
	}
	float cg_x = (float)px/p;
	float cg_y = (float)py/p;
	printf("x=%10.6f y=%10.6f\n",cg_x,cg_y);
}

void video_blocksum(img_struct* img1, img_struct* img2, int* dx_out, int* dy_out) 
{
	int h=img1->h;
	int w=img1->w;
	int n=w*h;
	unsigned char* buf1 = img1->buf;
	unsigned char* buf2 = img2->buf;
	printf("%u %u\n%u %u\n%u %u\n%u %u\n",buf1[h*w],buf1[h*w+1],buf1[(h+1)*w],buf1[(h+1)*w+1],buf1[1/2*h*w],buf1[1/2*h*w+1],buf1[5/4*(h+1)*w],buf1[5/4*(h+1)*w+1]);
	
	int dmax = 3;
	int min_sum = 2000000000;
	int min_dx = -99;
	int min_dy = -99;
	for(int dy=-dmax;dy<=dmax;dy++) {
		for(int dx=-dmax;dx<=dmax;dx++) {
			int sum=0;
			for(int y=dmax;y<h-dmax;y++) {
				int i1 = y*w + dmax;
				int i2 = (y+dy)*w + dmax+dx;
				for(int x=dmax;x<w-dmax;x++) {
					//printf("x=%d y=%d i1=%d i2=%d\n",x,y,i1,i2);
					sum += abs(buf1[i1] - buf2[i2]);
					i1++;
					i2++;
				}
			}
			if(min_sum>sum) {
				min_sum = sum;
				min_dx = dx;
				min_dy = dy;
			}
		}
	}
			
	*dx_out=min_dx;
	*dy_out=min_dy;
}

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc,char ** argv)
{
	vid_struct vid;
	vid.device = (char*)"/dev/video0";
	vid.w=640;
	vid.h=480;
	int w=vid.w;
	int h=vid.h;
	vid.n_buffers = 4;
	video_Init(&vid);
	
	//socket file descriptor, new socket file descriptor, port number
	int sockfd, newsockfd, portno;
    socklen_t clilen;
	
	//Declare additional variables
	struct sockaddr_in serv_addr, cli_addr;
    int n;	
	
	//open tcp socket
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
	if (bind(sockfd, (struct sockaddr *) &serv_addr,
        sizeof(serv_addr)) < 0) { 
        error("ERROR on binding");
    }
	 
	//listen for packet
	listen(sockfd,5);
    clilen = sizeof(cli_addr);
     
	//open new socket
	newsockfd = accept(sockfd, 
                (struct sockaddr *) &cli_addr, 
                &clilen);
    if (newsockfd < 0) {
        error("ERROR on accept");
	}
	
	printf("Connected!\n");
	
	//Create blank image
	img_struct* img_new = video_CreateImage(&vid);
	
    for (;;) {
	
		// Get image from video stream
		video_GrabImage(&vid, img_new);
		unsigned char * image = img_new->buf;
		
		// Loop to send entire buffer to server
		char buffer[19];
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
			printf("sent %d bytes\n",n);
			
			// Make sure all 9216 bytes have been sent
			// If not, resend rest of bytes
			int sum = n;
			while (sum < 9216) {
				n = write(newsockfd,packet+sum,9216-sum);
				if (n < 0) {
					error("ERROR reading from socket");
				}
				sum += n;
				printf("resent %d bytes\n",n);
			}
		}
		
		// Read message from client
		bzero(buffer,19);
		n = read(newsockfd,buffer,19);
		if (n < 0) {
			 error("ERROR reading from socket");
		}
		
		//Make sure all 19 btes have been read
		// If not, reread rest of bytes
		int sum = n;
		while (sum < 19) {
			n = read(newsockfd,buffer+sum,19-sum);
			if (n < 0) {
				error("ERROR reading from socket");
			}
			sum += n;
			printf("reread %d bytes\n",n);
		}
		
		printf("message received is %s\n",buffer);
	}
	close(newsockfd);
	video_Close(&vid);
	
    return 0;
}
