// #include <stdio.h>
// #include <stdlib.h> 
// #include <fcntl.h>
// #include <sys/stat.h>

#include<stdlib.h>
#include<string.h>
#include<fcntl.h>
#include<unistd.h>
#include<stdio.h>
#include<sys/types.h>
#include<sys/stat.h>

int main(void)
{
    // FILE *fp;
    int fd;
    int num;
    int folder;
   
    char a[] = "Hello ARM Linux!";
    
    //fp = fopen("/tmp/linux.txt","w+");
    // fd = open("linux.txt","w+");
    //打开文件  以 只读方式打开 用户可读写，同组用户只读权限管理可以用 & 表示这些权限同时成立
    fd = open("linux.txt",O_RDWR|O_CREAT,S_IRWXU&S_IRGRP);
    if(fd == NULL)
    {
        printf("\nFail to open linux.txt!\n");
        exit(-1);
    }

    // num = fwrite(a, sizeof(a), 1, fd);
    num = write(fd,a,strlen(a));
    printf("%ld byte data has written to linux.txt\n", num*sizeof(a));

    folder =  mkdir("linux",0777);
    if(folder == -1)
    {
        printf("\n Fail to create folder linux!\nIt has existed or the path is error!\n");
        exit(-1);
    }
    printf("Folder linux created success!\n");


    close(fd);
    return 0;
}
