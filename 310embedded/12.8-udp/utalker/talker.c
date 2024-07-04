
#include <stdio.h> 
#include <stdlib.h>
#include <errno.h> 
#include <string.h> 
#include <netdb.h> 
#include <sys/types.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 5000                      // The port which is communicate with server
#define LENGTH 512                     // Buffer length

int main(int argc, char *argv[])
{   int sockfd;                        // Socket file descriptor
    int num;                           // Counter of received bytes  
    char sdbuf[LENGTH];                // Receive buffer
    struct sockaddr_in addr_remote;    // Host address information
    char sdstr[]= {"Welcome to ZJUT ^_^"};    

    
    /* Check parameters number */
    if (argc != 2)                     
    {    
        printf ("Usage: talker HOST IP (ex: ./talker 127.0.0.1).\n");
        return (0);
    }

    /* Get the Socket file descriptor */
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        printf("ERROR: Failed to obtain Socket Descriptor!\n");
        return (0);
    }
    
    /* Fill the socket address struct */
    addr_remote.sin_family = AF_INET;                   // Protocol Family
    addr_remote.sin_port = htons(PORT);                 // Port number
    inet_pton(AF_INET, argv[1], &addr_remote.sin_addr); // Net Address
    bzero(&(addr_remote.sin_zero), 8);                  // Flush the rest of struct
  
    /* Try to connect the server */
    bzero(sdbuf,LENGTH);
    num = sendto(sockfd, sdstr, strlen(sdstr), 0, (struct sockaddr *)&addr_remote, sizeof(struct sockaddr_in));
    if( num < 0 )
    {
            printf ("ERROR: Failed to send your data!\n");
    }
    else
    {
            printf ("OK: Sent to %s total %d bytes !\n", argv[1], num);
    }
    close (sockfd);
    return (0);
}
