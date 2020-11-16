#ifndef PS_MON_IOCTL_H
#define PS_MON_IOCTL_H
#include <linux/ioctl.h>

typedef struct
{
    unsigned long long int nsec[2];
} freq_arg_t;

#define FREQ_GET_VARIABLES _IOR('q', 1, freq_arg_t *)
#define FREQ_CLR_VARIABLES _IO('q', 2)

#endif
