#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>  
#include <termios.h>
#include <errno.h>
int main()
{
    int fd;
    int xi = 5;
    fd = open("/dev/ttyS1",O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd == -1){
        perror("open ttys1\n");
        return 0;
    }
    printf("open ttys1 ok!\n");
    

    struct termios oldtio = { 0 };
    struct termios newtio = { 0 };
    tcgetattr(fd, &oldtio);

    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = 0; // IGNPAR | ICRNL
    newtio.c_oflag = 0;
    newtio.c_lflag = 0; // ICANON
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);
    //设置为非阻塞模式
    fcntl(fd, F_SETFL, O_NONBLOCK);

    while (xi>0) {
        unsigned char buffer[1024] = {0};
        unsigned char buffer_hello[] = {"helloworld\r\n"};
        int ret = read(fd, buffer, sizeof(buffer));
        // 不断发送helloworld
        int n = write(fd, buffer_hello, sizeof(buffer_hello));
        if (n != ret)
            printf("发送失败");
        if (ret > 0) {
            //依次将读取到的数据输出到日志
            for (int i = 0; i < ret; ++i)
                printf("收到%02x", buffer[i]);

            //当收到数据时，再将收到的数据原样发送
            int n = write(fd, buffer, ret);
            if (n != ret)
                printf("发送失败");

            //当收到0xFF时，跳出循环
            if (buffer[0] == 0xFF)
                break;
        } else {
            //没收到数据时，休眠50ms，防止过度消耗cpu
            usleep(1000 * 50);
        }
    }

    close(fd);
    return 0;
}
