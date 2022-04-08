#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <pthread.h>
#include <iostream>
#include <unistd.h>
#include </usr/xenomai/include/trank/native/task.h>
#include </usr/xenomai/include/trank/native/timer.h>
#include </usr/xenomai/include/trank/rtdk.h>
#include </usr/xenomai/include/trank/native/sem.h>
#include </usr/xenomai/include/trank/native/mutex.h>
#include <rtdm/testing.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <rtdm/ipc.h>
#include <malloc.h>
#include <sys/time.h>
#include <ctime>


RT_MUTEX m_mutex;
RT_TASK m_rttask1;
RT_TASK m_rttask2;
RT_MUTEX_INFO *info = new RT_MUTEX_INFO;
#define MAX 100
// 全局变量
int number;



static void rtCtrlProc1(void* arg)
{

    for(int i=0;i<10;++i)
    {
        int res = 0;
        res = rt_mutex_acquire(&m_mutex,TM_INFINITE);
        rt_printf(" 1 res : %d\n", res);
        number +=i;
        rt_printf("rt task , thread = %lu, handle = %lu, number is: %d\n", rt_task_self()->thread,rt_task_self()->handle, number);
        RT_MUTEX_INFO info1;


        rt_mutex_inquire(&m_mutex,&info1);
        usleep(100000);
        rt_printf("m_mutex %s owner thread = %lu ,handle = %lu,\n", info1.name, info1.owner.thread,info1.owner.handle);
        rt_mutex_release(&m_mutex);
         usleep(100);
    }
}

static void rtCtrlProc2(void* arg)
{

    for(int i=0;i<10;++i)
    {
        int res =0;
        res = rt_mutex_acquire(&m_mutex,TM_INFINITE );
        rt_printf(" 2 res : %d\n", res);
        number -=i;
        rt_printf("rt task , thread = %lu, handle = %lu, number is: %d\n", rt_task_self()->thread,rt_task_self()->handle, number);
        RT_MUTEX_INFO *info2 = new RT_MUTEX_INFO;
        rt_mutex_inquire(&m_mutex,info2);
        usleep(100000);

        rt_printf("m_mutex %s owner thread = %lu ,handle = %lu,\n",  info2->name,  info2->owner.thread,info2->owner.handle);
        rt_mutex_release(&m_mutex);
         usleep(100);
    }
}

int main(int argc, const char* argv[])
{
    pthread_t p1, p2;

    // 初始化互斥锁
    rt_mutex_create(&m_mutex,"myrtlock");
    rt_mutex_inquire(&m_mutex,info);

    std::cout << info->name << std::endl;
    std::cout << info->owner.thread<< std::endl;
    std::cout << info->owner.handle <<std::endl;

    
    int err = rt_task_create(&m_rttask1, "rtCtrlTask1", 0, 60, 0);
    if (err) {
        printf("receivetest: Failed to create rt task1, code %d\n",
               errno);
    }

    err = rt_task_create(&m_rttask2, "rtCtrlTask2", 0, 60, 0);
    if (err) {
        printf("receivetest: Failed to create rt task2, code %d\n",
               errno);
    }

    rt_task_start(&m_rttask1, rtCtrlProc1,NULL);
    rt_task_start(&m_rttask2, rtCtrlProc2,NULL);

    while (true)
    {
        RT_MUTEX_INFO maininfo;
        rt_mutex_inquire(&m_mutex,&maininfo);
        rt_printf("main thread inquire : mutex name : %s, owner thread = %lu , handle = %lu\n", maininfo.name, maininfo.owner.thread, maininfo.owner.handle);
        usleep(10000);
    }

    return 0;
}

