/*6-7.c程序：父进程运行比子进程快，避免子进程成为僵尸进程*/
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdlib.h>

void display0();
void display1();
void display2();

void display0()
{
    printf("您选择进入了菜单模式\n");
}

void display1()
{
    printf("您选择进入了命令行模式\n");
}

void display2()
{
    printf("您选择进入了IP地址模式\n");
}

int main()
{
    pid_t result;
    int status,select,num;
    void (*fun[3])();
    fun[0] = display0;
    fun[1] = display1;
    fun[2] = display2;

    printf("1.复制进程(不用waitpid)\n2.复制进程(用waitpid)\n3.不复制进程(退出)请输入您的选择：\n");
    scanf("%d",&select);
    if(select == 1)
    {
        result = fork();
        if(result == -1)
        {
            perror("复制进程出错\n");
            exit(1);
        }
    }
    if(select == 2)
    {
        result = fork();
        if(result == -1)
        {
            perror("复制进程出错\n");
            exit(1);
        }
    }
    if(result == 0)
    {
        printf("这是子进程（进程号：%d)，父进程号：%d：\n",getpid(),getppid());
        printf("进入思科（Cisco）1912交换机开机界面\n");
        printf("1位用户现在激活管理控制台\n");
        printf("\t用户界面菜单\n");
        printf("\t[0] 菜单模式\n");
        printf("\t[1] 命令行模式\n");
        printf("\t[2] IP地址模式\n");
        printf("输入你的选择：\n");
        
        scanf("%d",&num);
        if(num >= 0 && num <=2)
        {
            (*fun[num])();
        }

        printf("%d\n",result);
        exit(0);
    }else if(select != 3){
        if(select == 2)
        {
            waitpid(result,&status,0);
        }
        
        printf("这是父进程（进程号：%d)，父进程号：%d：\n",getpid(),getppid());
        // printf("%d\n",int(true));
        // printf("%d\n",int(false));

        /* Nonzero if STATUS indicates normal termination.  */
        // #define	__WIFEXITED(status)	(__WTERMSIG(status) == 0)
        // 正常退出是1，异常退出是0
        if(WIFEXITED(status) == 0)
        {
            printf("子进程非正常终止，子进程终止状态：%d\n",WIFEXITED(status));
        }else{
            printf("子进程正常终止，子进程终止状态：%d\n",WIFEXITED(status));
        }
        exit(0);
    }else{
        exit(0);
    }
}