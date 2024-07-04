#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <pthread.h>

char ipp[50];
struct sockaddr_in other;

void *readMsg(void *arg);

int main(int argc, char *argv[])
{
    // 创建socket套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("socket error");
        exit(-1);
    }

    // bind绑定自己的IP地址和端口号
    struct sockaddr_in myself;
    myself.sin_family = AF_INET;
    myself.sin_port = htons(12345);
    myself.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr *)&myself, sizeof(myself)) < 0)
    {
        perror("bind error");
        exit(-1);
    }

    char port[] = "12345"; // 这里为了方便直接固定了，有弊端可以改成输入的
    printf("Please enter the IP you want to connect to:\n");
    printf("Please enter IP: ");
    fgets(ipp, 50, stdin);
    printf("\n");

    // 对方的IP地址和端口号
    other.sin_family = AF_INET;
    other.sin_port = htons(atoi(port));
    other.sin_addr.s_addr = inet_addr(ipp);

    // 创建线程
    pthread_t pthread;
    if (pthread_create(&pthread, NULL, readMsg, &sockfd) != 0)
    {
        perror("pthread create error");
        exit(-1);
    }

    char buf[50];
    int ret;

    while (1)
    {
        bzero(buf, sizeof(buf));
        fgets(buf, sizeof(buf), stdin);
        printf("\n");
        ret = sendto(sockfd, buf, strlen(buf), 0, (struct sockaddr *)&other, sizeof(other));
        // 发送数据
        if (ret < 0)
        {
            perror("sendto error");
            exit(-1);
        }
    }

    return 0;
}

// 线程的操作
void *readMsg(void *arg)
{
    int sockfd = *(int *)arg; // 将传进来的参数转换
    char buf[50];
    int len = sizeof(other);
    while (1)
    {
        bzero(buf, sizeof(buf));
        // 接收数据
        int ret = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr *)&other, (socklen_t *)&len);
        if (ret < 0)
        {
            perror("recvfrom error");
            exit(-1);
        }
        printf("from: %s\n", inet_ntoa(other.sin_addr));
        printf("message: %s\n", buf);
    }
}
