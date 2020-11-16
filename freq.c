/*******************************************************************************/
/*                                                                             */
/*              GPIO Frequency/Period Reading Module. Rev. 1.0.0.1             */
/*              Volkovs, Andrejs, (GPL) 2018-2020                              */
/*                                                                             */
/*******************************************************************************/
#include <linux/init.h>                 ///< Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/fs.h>                   ///< Header for the Linux file system support
#include <linux/gpio.h>
#include <linux/interrupt.h>
//#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/time.h>                 ///< timespec
#include <asm/div64.h>                  ///< for fast div
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>

#include "freq_ioctl.h"

/* device definition */
#define  DEVICE_NAME "freq"             ///< The device will appear at /dev/freq using this value
#define  CLASS_NAME  "freq"             ///< The device class -- this is a character device driver
#define  MAX_DEV_CNT 10                 ///< Maximum devices

MODULE_LICENSE("GPL");                  ///< The license type -- this affects available functionality
MODULE_AUTHOR("Volkovs, Andrejs");      ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A Linux char driver for frequency reading from GPIO"); ///< The description -- see modinfo
MODULE_VERSION("1.0.0.1");              ///< A version number to inform users

static int    majorNumber;              ///< Stores the device number -- determined automatically
static char   message[2256] = {""};     ///< Memory for the string that is passed from userspace
static short  size_of_message = 0;      ///< Used to remember the size of the string stored
static int    numberOpens = 0;          ///< Counts the number of times the device is opened
static struct class*  freqClass = NULL; ///< The device-driver class struct pointer
static struct device* freqDevice[MAX_DEV_CNT] = {NULL}; ///< The device-driver device struct pointer

static int count=2;                     ///< devices counter

/* The prototype functions for the character driver -- must come before the struct definition */
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int dev_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg);
#else
static long dev_ioctl(struct file *f, unsigned int cmd, unsigned long arg);
#endif

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = dev_ioctl
#else
    .unlocked_ioctl = dev_ioctl
#endif

};

static unsigned int cnt_gpio[MAX_DEV_CNT]={69,73};
module_param_array(cnt_gpio,int,&count,0660);

static const char* cnt_names[MAX_DEV_CNT] = {
"GPIO Pin 1",				//0, 69
"GPIO Pin 2",				//1, 73
"GPIO Pin 3",				//2,
"GPIO Pin 4",				//3,
"GPIO Pin 5",				//4,
"GPIO Pin 6",				//5,
"GPIO Pin 7",				//6,
"GPIO Pin 8",				//7,
"GPIO Pin 9",				//8,
"GPIO Pin 10",				//9,
};
static int cnt_irq_input_pin[MAX_DEV_CNT];
//static struct timespec cnt_pin_time[2][MAX_DEV_CNT];
//static struct timespec cnt_pin_time_diff[MAX_DEV_CNT];
static unsigned short timespec_ind[MAX_DEV_CNT];
//static unsigned short int p_counter[MAX_DEV_CNT];
static unsigned int counter[MAX_DEV_CNT];
static unsigned int frequency[MAX_DEV_CNT];
static bool cnt_irq_off[MAX_DEV_CNT];

//test fast time
static u64 cnt_pin_ns[2][MAX_DEV_CNT];
static u64 cnt_pin_ns_diff[MAX_DEV_CNT];
static u64 cnt_pin_ns_pulse[MAX_DEV_CNT];

static u64 cnt_pin_ns_diff_sum[MAX_DEV_CNT];
static u64 cnt_pin_ns_pulse_sum[MAX_DEV_CNT];

static u64 cnt_pin_ns_diff_avg[MAX_DEV_CNT];
static u64 cnt_pin_ns_pulse_avg[MAX_DEV_CNT];

static void process_cnt_irq(int cnt_num) {
    counter[cnt_num]++;
    timespec_ind[cnt_num]=(timespec_ind[cnt_num]+1)&1;
    cnt_pin_ns[timespec_ind[cnt_num]][cnt_num]=ktime_get_mono_fast_ns();
    cnt_pin_ns_diff_sum[cnt_num]+=cnt_pin_ns_diff[cnt_num]=cnt_pin_ns[timespec_ind[cnt_num]][cnt_num] - cnt_pin_ns[(timespec_ind[cnt_num]+1)&1][cnt_num];
}

static irqreturn_t irq_handler( int irq, void *dev_id )
{
    int i;
    for (i=0; i<count; i++) {
        if (irq == cnt_irq_input_pin[i]) {
            if (gpio_get_value(cnt_gpio[i]))
                process_cnt_irq(i);
            else
               cnt_pin_ns_pulse_sum[i]+=cnt_pin_ns_pulse[i]=ktime_get_mono_fast_ns()-cnt_pin_ns[timespec_ind[i]][i];
        }
    }
    return IRQ_HANDLED;
}


// Frequency meter timer 
static struct hrtimer htimer;
static ktime_t kt_periode;

static enum hrtimer_restart timer_function(struct hrtimer * timer)
{
    int i;
    hrtimer_forward_now(timer, kt_periode);
    for (i=0; i<count; i++) {
        if (counter[i]>1) {
            cnt_pin_ns_diff_avg[i] = div64_ul(cnt_pin_ns_diff_sum[i],counter[i]);
            cnt_pin_ns_pulse_avg[i] = div64_ul(cnt_pin_ns_pulse_sum[i],counter[i]);
        } else {
            cnt_pin_ns_diff_avg[i] = cnt_pin_ns_diff[i];
            cnt_pin_ns_pulse_avg[i] = cnt_pin_ns_pulse[i];
        }
        cnt_pin_ns_diff_sum[i]=0;
        cnt_pin_ns_pulse_sum[i]=0;
        if (cnt_irq_off[i]) { enable_irq(cnt_irq_input_pin[i]); cnt_irq_off[i] = false;}
        if (counter[i]>80000) { disable_irq(cnt_irq_input_pin[i]); cnt_irq_off[i] = true; }
        frequency[i]=counter[i];
        counter[i]=0;
    }
#ifdef _DEBUG
    printk(KERN_INFO "freq: 1 Sec Timer\n");
#endif
    return HRTIMER_RESTART;
}

static int cnt_gpio_init(int num, char* name) {
    int rc = gpio_request(cnt_gpio[num], name);
    if( rc < 0 ) {
        printk(KERN_ERR "freq: gpio_request for pin %u (%s) failed with error %d\n", cnt_gpio[num], name, rc );
        return rc;
    } else {
        rc = gpio_direction_input(cnt_gpio[num]);
        if ( rc < 0 ) {
            printk(KERN_ERR "freq: gpio_in_direction_request for pin %u (%s) failed with error %d\n", cnt_gpio[num], name, rc );
            return rc;
        }
    }
    cnt_irq_input_pin[num] = gpio_to_irq(cnt_gpio[num]);
    if( cnt_irq_input_pin[num] < 0 ) {
        printk(KERN_ERR "freq: gpio_to_irq failed with error %d\n", cnt_irq_input_pin[num] );
        gpio_free(cnt_gpio[num]);
        return cnt_irq_input_pin[num];
    }
    // the string "cnt_gpio_handler" can be found in cat /proc/interrupts when module is loaded
    rc = request_irq( cnt_irq_input_pin[num], &irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "cnt_gpio_handler", NULL );
    if( rc < 0 ) {
        gpio_free(cnt_gpio[num]);
        printk(KERN_ERR "freq: request_irq failed with error %d\n", rc );
    }
    return rc;
}

static void cnt_gpio_exit(int num) {
    free_irq( cnt_irq_input_pin[num], NULL );
    gpio_free(cnt_gpio[num]);
}

static int __init my_init(void)
{
    int i,rc;
    char str[50];
    char dev_name[10];

    printk(KERN_INFO "freq module: init\n");

    // Counter GPIOs
    for (i=0; i<count; i++) {
        sprintf(str,"CNT_GPIO%u",i);
        rc=cnt_gpio_init(i,str);
        if (rc < 0) {
            printk(KERN_ERR "freq: cnt_gpio init %i-%u failed with error %d\n", i, cnt_gpio[i], rc );
            return rc;
        }
    }
    // timer
    kt_periode = ktime_set(0, 1000000000); //seconds,nanoseconds
//    hrtimer_init (&htimer, CLOCK_REALTIME, HRTIMER_MODE_REL);
    hrtimer_init (&htimer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    htimer.function = timer_function;
//    hrtimer_start(&htimer, kt_periode, HRTIMER_MODE_REL);
    hrtimer_start(&htimer, kt_periode, HRTIMER_MODE_ABS);

    // Init freq char device
#ifdef _DEBUG
    printk(KERN_INFO "freq: Initializing the freq LKM\n");
#endif
    // Register the device class
    freqClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(freqClass)){                // Check for error and clean up if there is
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(freqClass);          // Correct way to return an error on a pointer
    }
    printk(KERN_INFO "freq: device class registered correctly\n");

    // Try to dynamically allocate a major number for the device -- more difficult but worth it
    majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
    if (majorNumber<0){
        printk(KERN_ALERT "freq failed to register a major number\n");
        return majorNumber;
    }
    printk(KERN_INFO "%s: registered correctly with major number %d\n", DEVICE_NAME, majorNumber);

    // Register the device driver
    for (i=0; i<count; i++) {
        sprintf(dev_name,"%s%d",DEVICE_NAME,cnt_gpio[i]);
        freqDevice[i] = device_create(freqClass, NULL, MKDEV(majorNumber, i), NULL, dev_name);
        if (IS_ERR(freqDevice)){               // Clean up if there is an error
            class_destroy(freqClass);           // Repeated code but the alternative is goto statements
            unregister_chrdev(majorNumber, DEVICE_NAME);
            printk(KERN_ALERT "Failed to create the device\n");
            return PTR_ERR(freqDevice);
        }
        printk(KERN_INFO "freq: device %s  created correctly\n",dev_name); // Made it! device was initialized
    }
    printk(KERN_INFO "freq: device class created correctly\n"); // Made it! device was initialized
    return 0;
}


static void __exit my_exit(void)
{
    int i;
    char dev_name[10];
    printk(KERN_INFO "freq: exit\n" );

    for (i=0; i<count; i++) {
        cnt_gpio_exit(i);
    }

    // cancel timer
    hrtimer_cancel(&htimer);

    // exit device
    for (i=0; i<count; i++) {
        sprintf(dev_name,"%s%d",DEVICE_NAME,cnt_gpio[i]);
        device_destroy(freqClass, MKDEV(majorNumber, i));	// remove the device
        unregister_chrdev(majorNumber, dev_name);		// unregister the major number
        printk(KERN_INFO "freq: device %s unregistered\n",dev_name);
    }
    class_unregister(freqClass);				// unregister the device class
    class_destroy(freqClass);				// remove the device class
    printk(KERN_INFO "freq: class unregistered\n");
}

// device functions
/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
   numberOpens++;
#ifdef _DEBUG
   printk(KERN_INFO "freq: Device has been opened %d time(s)\n", numberOpens);
#endif
   return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){

    int error_count = 0;
    int i;

    u64 ns,pns,f100=0,f=0,fdec=0;

    char* dev_name  = filep->f_path.dentry->d_iname;//(int)filep->private_data;

    int minor = MINOR(filep->f_path.dentry->d_inode->i_rdev);

    i=minor;

    size_of_message=0;

    ns=cnt_pin_ns_diff_avg[i];
    pns=cnt_pin_ns_pulse_avg[i];
    if (ns>0) {
        f100=div64_u64(100000000000,ns);
        f=div64_u64_rem(f100,100,&fdec);
    }
    size_of_message+=sprintf(&message[size_of_message],"%i %s %s (GPIO%u): F=%llu.%02llu Hz, P=%llu:%llu ns\n",minor, dev_name,cnt_names[i], cnt_gpio[i], /*frequency[i]*/f,fdec, ns, pns );

    if (*offset>=size_of_message) return 0;

   // copy_to_user has the format ( * to, *from, size) and returns 0 on success
   error_count = copy_to_user(buffer, message, size_of_message);

   if (error_count==0){            // if true then have success
        printk(KERN_INFO "freq: Sent %d characters to the user\n", size_of_message);
        *offset+=size_of_message;
        // clear the position to the start and return size_ofMessage
        error_count=size_of_message;
        size_of_message=0;
        return error_count;
   }
   else {
        printk(KERN_INFO "freq: Failed to send %d characters to the user\n", error_count);
        return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
   }
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
    int error_count = 0;
    error_count = copy_from_user(message, buffer, len);
    message[len]=0;
#ifdef _DEBUG
    printk(KERN_INFO "freq: Received %u characters from the user: %s\n", len, message);
#endif
    return len;
}

// ioctl
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int dev_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg)
#else
static long dev_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
#endif
{
    int minor = MINOR(f->f_path.dentry->d_inode->i_rdev);
    freq_arg_t fr;
    switch (cmd) {
           case FREQ_GET_VARIABLES:
            fr.nsec[0]=cnt_pin_ns_diff_avg[minor];
            fr.nsec[1]=cnt_pin_ns_pulse_avg[minor];
            if (copy_to_user((freq_arg_t *)arg, &fr, sizeof(freq_arg_t)))
            {
                return -EACCES;
            }
            break;
        case FREQ_CLR_VARIABLES:
            //resp.inputs=0;
            //resp.outputs=0;
            break;
        default:
            return -EINVAL;
    }
    return 0;
}


/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
#ifdef _DEBUG
   printk(KERN_INFO "freq: Device successfully closed\n");
#endif
   return 0;
}
//

module_init(my_init);
module_exit(my_exit);


MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("A.V.");
MODULE_DESCRIPTION("Freq GPIO IRQ Driver");