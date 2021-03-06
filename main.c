/****************************************************************************/
/*                                                                          */
/*              Frequency Meter Module Test ioctl. Rev. 1.0.0.1             */
/*              Volkovs, Andrejs, GPL, 2018-2020                            */
/*                                                                          */
/****************************************************************************/
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>

#include "freq_ioctl.h"

int main(int argc, char *argv[])
{
    freq_arg_t resp;
    char file_name[20] = "/dev/freq69";
    char key = 0;
    if (argc>2) {
        sprintf(file_name,"%s",argv[2]);
        if (argv[1][0]=='-') key=argv[1][1];
    }
    else if (argc>1) sprintf(file_name,"%s",argv[1]);
    int fd;
    fd = open(file_name, O_RDWR);
    if (fd == -1)
    {
        perror("freq open");
        return 2;
    }
    if (key=='c') {
        if (ioctl(fd, FREQ_CLR_VARIABLES) == -1)
        {
             perror("freq ioctl clear");
             return 1;
        } else {
             printf("freq %s cleared\n",file_name);
        }
        return 0;
    }
    if (ioctl(fd, FREQ_GET_VARIABLES, &resp) == -1)
    {
        perror("freq ioctl get");
        return 1;
    }
    else
    {
        printf("freq %s: %llu:%llu\n", file_name, resp.nsec[0], resp.nsec[1]);
    }
    close(fd);
    return 0;
}