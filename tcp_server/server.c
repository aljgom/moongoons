#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{
	//socket file descriptor,port number,n
    int sockfd, portno, n;
	
    struct sockaddr_in serv_addr;
    struct hostent *server;
	
	//declare receiving packet buffer and image buffer
	unsigned char image [460800];
	unsigned char buffer[9216];
	 
	//set port number
    portno = 7777;
	
	//open tcp socket
	//AF_INET:use IPv4 protocols,SOCK_STREAM:use TCP,0:default protocol
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
	
	//check if socket is open
	while (sockfd < 0) {
        error("ERROR opening socket");
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
	}
	
	//convert hostname to proper server info i.e. ip address
    char hostname[11];
	strcpy(hostname,"192.168.1.1");
	server = gethostbyname(hostname);
    
	//check to make sure there is server from hostname
	if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

	//zero-initialize serv_addr
    bzero((char *) &serv_addr, sizeof(serv_addr));
    
	//load IPv4 protocols
	serv_addr.sin_family = AF_INET;
    
	//connect to server
	bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) { 
        error("ERROR connecting");
    }
	
	//loop to receive all packets from server
	//concatenate all packets consecutively to form image buffer
	int t;
	int im;
	for (im=0;im<5;im++) {
		for (t=0;t<50;t++) {
			//get packet
			bzero(buffer,9216);
			n = read(sockfd,buffer,9216);
			printf("read %d bytes\n",n);
			if (n < 0) {
				error("ERROR reading from socket");
			}

			//make sure all 9216 bytes have been sent
			//if not, read rest of bytes
			int sum = n;
			while (sum < 9216) {
				n = read(sockfd,buffer+sum,9216-sum);
				if (n < 0) {
					error("ERROR reading from socket");
				}
				sum += n;
				printf("reread %d bytes\n",n);
			}

			//concatenate image buffer with packet buffer
			int m = 0;
			while (m<9216) {
				image[t*9216+m] = buffer[m];
				m++;
			}
		}
		
		// Send message back to server
		char out_msg[19];
		sprintf(out_msg,"Received %d\n",im);
		n = write(sockfd,out_msg,19);
		if (n < 0) {
			error("ERROR writing to socket");
		}

		// Make sure all 19 bytes have been sent
		// If not, read rest of bytes
		int sum = n;
		while (sum < 19) {
			n = write(sockfd,out_msg+sum,19-sum);
			if (n < 0) {
				error("ERROR writing to socket");
			}
			sum += n;
			printf("resent %d bytes\n",n);
		}		
	
		printf("image %d received\n",im);
		
		//write image<im>.yuv with image buffer
		FILE * file_fd;
		char im_title[10];
		sprintf(im_title,"image%d.yuv",im);
		file_fd = fopen(im_title,"wb");
		fwrite(image,460800,1,file_fd);
		fclose(file_fd);
	}	
	close(sockfd);
}
