#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

// 设置端口信息这是端口号要与服务器端口号相同
#define PORT 8081
// 设置最大发送信息字节
#define BUFFER_SIZE 1024
// 服务器的ip,本地连接即服务器的端口为127.0.0.1
#define HOST_ADDR "192.168.196.144"
// 这里和之前UDP实验有点不同是，客户端不需要bind,可以随机分配
// 当然如果想要绑定也是可以的，然后用虚拟名字代替各个用户

// 发送信息的结构体包含客户端的信息，发送信息 套接字描述符
struct Msg{
    char name[20];
    char message[BUFFER_SIZE];
    int socket;
};
struct Msg msg;

// 发送信息函数
void *send_fun(void *arg);
// 接受信息函数
void *recv_fun(void *arg);

int main()
{
    int sfd;
    struct sockaddr_in sin;

    // 创建一个套接字AF_INET代表ipv4协议 
    // SOCK——STREAM 是用TCP连接类型的有序字节流协议
    if((sfd = socket(AF_INET,SOCK_STREAM,0)) == -1)
    {
        printf("socket error\n");
        exit(1);
    }

    // 设置套接字地址信息AF——INET代表使用IPV4，
    // htons(PORT)把本地端口转化为网络端口s代表转化为短型的
    // inte_pton函数就是把开始设置的ip转化为网络的ip
    // 并使用ipv4协议给套接字sin的ip
    sin.sin_family = AF_INET;
    sin.sin_port = htons(PORT);

    // IP地址转换函数，可以在将点分文本的IP地址转换为
    // 二进制网络字节序的IP地址
    // inet_aton(AF_INET,HOST_ADDR,&SIN.SIN_ADDR);
    // 与上面转化一样
    if(inet_aton(HOST_ADDR,&sin.sin_addr) == -1)
    {
        printf("inet_aton error.\n");
        return -1;
    }

    // 为套接字补8个0
    bzero(&(sin.sin_zero),8);

    // 连接客户端，sfd为套接字描述符，
    // 第二个参数表示把sockaddr_in类型的sin转化为sockaddr类型的
    // 第三个参数为sin的字节数
    if(connect(sfd,(struct sockaddr *)&sin,sizeof(sin)) == -1)
    {
        printf("connet error\n");
        close(sfd);
        exit(1);
    }

    printf("输入姓名：");
    scanf("%s",msg.name);

    // 声明两个线程变量
    pthread_t th1,th2;

    // 分别把send_fun函数和recv_fun函数加入到主函数中
    if(pthread_create(&th1,NULL,send_fun,(void*)&sfd))
    {
        perror("Create phtread error\n");
        exit(1);
    }

    if(pthread_create(&th2,NULL,recv_fun,(void*)&sfd))
    {
        perror("Create phtread error\n");
        exit(1);
    }

    // 等待线程结束
    pthread_join(th1,NULL);
    pthread_join(th2,NULL);
    close(sfd);

    return 0;
}

void *send_fun(void *arg)
{
    printf("输入聊天内容：\n");
    int sfd = *((int *)arg);
    while(1)
    {
        // 聊天信息
        fgets(msg.message,1024,stdin);
        printf("\n");
        // 发送信息
        int n = send(sfd,(char *)&msg,sizeof(msg),0);
        if(n!=sizeof(msg))
        {
            perror("send error\n");
            exit(1);
        }
    }
}

void *recv_fun(void *arg)
{
    int sfd = *((int*)arg);
    struct Msg msg;
    while(1)
    {
        // 接受信息函数
        int n = recv(sfd,(char*)&msg,sizeof(msg),0);
        if(n == -1)
        {
            perror("recv error\n");
            exit(1);
        }
        // 输出接受的信息
        printf("编号为：%d,姓名为：%s,发的信息为：%s\n",msg.socket,msg.name,msg.message);
    }
}

