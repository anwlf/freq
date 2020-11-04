#ifndef PS_MON_IOCTL_H
#define PS_MON_IOCTL_H
#include <linux/ioctl.h>
 
typedef struct
{
    unsigned short var1, var2;
} freq_arg_t;
 
#define FREQ_GET_VARIABLES _IOR('q', 1, freq_arg_t *)
#define FREQ_CLR_VARIABLES _IO('q', 2)
#define FREQ_SET_VARIABLES _IOW('q', 3, freq_arg_t *)

#endif
