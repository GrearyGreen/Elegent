
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <pthread.h>
                                  
#define PORT 8081                      // 设置端口信息
#define BUFFER_SIZE 1024               //设置最大发送信息字节
#define MAX_SIZE 10                     // 最大连接数    

struct Msg{                  //发送消息的结构体包含姓名 发送的信息 该客户端的套接字描述符
    char name[20];
    char message[BUFFER_SIZE];
    int socket;
};
int connfd[MAX_SIZE];  //把链接进来的客户端放在数组中
void *recv_fun(void *arg); //接受信息函数
int main ()
{   
	  struct sockaddr_in sin,cin;//声明两个套接字
      int sfd,cfd;
      int Length = sizeof(struct sockaddr_in); //套接字结构体的大小
      int n;

      if((sfd=socket(AF_INET,SOCK_STREAM,0))==-1){  //创建套接字
        printf("socket error\n");
        exit(1);
      }
      //为套接字设置ip协议 设置端口号 并自动获取本机Ip转化为网络Ip
      sin.sin_family=AF_INET;
      sin.sin_port=htons(PORT);
      sin.sin_addr.s_addr=htonl(INADDR_ANY);
      bzero(&(sin.sin_zero),8);

      if(bind(sfd,(struct sockaddr *)&sin,sizeof(sin))<0){//绑定套接字
                 printf("bind error\n");
                 exit(1);
      }
      printf("Bind success!\n");
                 
    if(listen(sfd,MAX_SIZE)==-1){//监听最大连接数
               printf("listen error\n");
               exit(1);
    }
    printf("listening....\n");

    int i;

    for(i=0; i<MAX_SIZE; i++) //刚开始先把存放客户端的数组设置初始值为-1
           connfd[i] = -1;

    printf("Waiting......\n");//等待客户端连接

    while(1){
        int i;
        for(i=0; i<MAX_SIZE; i++){//寻找到没有保存客户端的一个位置
              if(connfd[i] == -1){
                break;
              }
        }
         //让该位置保存下一个连接进来的客户端
         if((connfd[i]=accept(sfd,(struct sockaddr *)&cin, (socklen_t*)&Length))== -1){//连接客户端
             printf("accept error\n");
             exit(1);
         }else
         printf("有人加入编号为： %d\n", i); //输入连接进来的客户端的编号

         pthread_t th1;
         pthread_create(&th1, NULL, recv_fun, (void*)&connfd[i]);//创建一个线程把接收信息的函数加入到主函数中
    }
    return 0;
}
void *recv_fun(void *arg){
    int fd = *((int *)arg); //用来保存刚才发消息过来的客户端的编号
    struct Msg msg; //声明一个结构提用来接收发送过来的信息

    int flge, num_send, num_recv;

    while(1){
        int n=recv(fd, (char*)&msg, sizeof(msg), 0); //接受发送过来的信息保存到msg的结构体中
        if(n<=0){//判断是否接收成功
                printf("recv error\n");
                connfd[msg.socket] = -1;
                pthread_exit(NULL);
                exit(1);
        }

        int i;
        //在服务器中显示一下发送的信息
        printf("接受到编号为: %d, %s 的消息为： %s\n",msg.socket, msg.name, msg.message);
        //从开始位置一直找到最后一个存放客户端的位置除去自己其他的客户端都发送一边消息
             for(i=0; i<MAX_SIZE; i++){
                if(connfd[i] != fd && connfd[i] !=-1){
                    num_send=send(connfd[i], (char*)&msg, sizeof(msg), 0);
                    if(num_send < 0){
                        printf("send error\n");
                    }
                }
             }
    }
}