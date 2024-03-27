
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
                                  
#define PORT 81                        // The port which is communicate with server
#define BACKLOG 10
#define LENGTH 1024                    // Buffer length    

char  httpweb[]={
    "HTTP/1.0 200 OK\r\n"
    "Date: Mon, 24 Nov 2005 10:26:00 GMT\r\n"
    "Server: microHttp/1.0 zenesion Technology CO.,LTD\r\n"
    "Accept-Ranges: bytes\r\n"
    "Connection: Keep-Close\r\n"
    "Content-Type: text/html\r\n"
    "\r\n"};
 char  web[]={
 				"<HTML>\r\n"
 				"<HEAD>\r\n"
 				"<TITLE>s210x演示网页（中智讯（武汉）科技有限公司）</TITLE>\r\n"
 				"<BODY aLink=green bgColor=#f1f1dd link=red\r\n"
				"vLink=#321afd>\r\n"
 				"<H1>Welcome To zonesion s4412 Platform</H1>\r\n"
 				"<UL>\r\n"
 				"<LI> <A HREF=\"http://www.zonesion.com/\">s4412 Android 3G移动互联网开发平台</A>\r\n"
 				"</BODY>\r\n"
 				"</HTML>\r\n"
				};
char  httpgif[]={
    "HTTP/1.0 200 OK\r\n"
    "Date: Mon, 24 Nov 2005 10:26:00 GMT\r\n"
    "Server: microHttp/1.0 zenesion Technology CO.,LTD\r\n"
    "Accept-Ranges: bytes\r\n"
    "Connection: Keep-Close\r\n"
    "Content-Type: image/bmp\r\n"
    "\r\n"};
 
                                                                             
int main ()
{   
		int sockfd;                        // Socket file descriptor
    int nsockfd;                       // New Socket file descriptor
    int num;
    int sin_size;                      // to store struct size
    char revbuf[LENGTH]; 
    struct sockaddr_in addr_local;     
    struct sockaddr_in addr_remote;    
               
    /* Get the Socket file descriptor */  
    if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 )  
    {   
        printf ("ERROR: Cannot obtain Socket Despcritor.\n");
        return (0);
    }
    else
    {
        printf ("OK: Obtain Socket Despcritor sucessfully.\n");
    }
    
    /* Fill the local socket address struct */
    addr_local.sin_family = AF_INET;           // Protocol Family
    addr_local.sin_port = htons(PORT);         // Port number
    addr_local.sin_addr.s_addr  = INADDR_ANY;  // AutoFill local address
    bzero(&(addr_local.sin_zero), 8);          // Flush the rest of struct

    /*  Blind a special Port */
    if( bind(sockfd, (struct sockaddr*)&addr_local, sizeof(struct sockaddr)) == -1 )
    {  
    	  printf ("ERROR: Cannot bind Port %d\n.",PORT);
        return (0);
    }
    else
    {
        printf("OK: Bind the Port %d sucessfully.\n",PORT);
    }
   
    /*  Listen remote connect/calling */
    if(listen(sockfd,BACKLOG) == -1)    
    {  
        printf ("ERROR: Cannot listen Port %d\n.", PORT);
        return (0);
    }
    else
    {
        printf ("OK: Listening the Port %d sucessfully\n.", PORT);
    }
   
    while(1)
    {  
        sin_size = sizeof(struct sockaddr_in);  
        
        /*  Wait a connection, and obtain a new socket file despriptor for single connection */
        if ((nsockfd = accept(sockfd, (struct sockaddr *)&addr_remote, &sin_size)) == -1)
        {  
            printf ("ERROR: Obtain new Socket Despcritor error\n");
            continue;
        }
        else
        {
            printf ("OK: Server has got connect from %s\n", inet_ntoa(addr_remote.sin_addr)); 
        }
        
        num = recv(nsockfd, revbuf, LENGTH, 0);
        
        /* Child process */
        if(!fork())                    
        {  
            printf("OK: Http web is servering.\n");

            if(revbuf[5]==' ')
            {
                send(nsockfd, httpweb, sizeof(httpweb), 0); 
	        			send(nsockfd, web, sizeof(web), 0); 
	    			}
	  			  else if(revbuf[5]=='1')
                send(nsockfd, httpgif, sizeof(httpgif), 0);
        }      
        close(nsockfd);  
        while(waitpid(-1, NULL, WNOHANG) > 0);    
     }    
}
