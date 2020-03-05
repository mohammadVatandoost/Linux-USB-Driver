#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/uaccess.h> // for copy_from_user()

#define MIN(a,b) (((a) <= (b)) ? (a) : (b))
#define BULK_EP_OUT 0x01
#define BULK_EP_IN 0x82
#define MAX_PKT_SIZE 512

// Direct commands
#define OPBOX_SN         0xD0
#define RESET           0xD1
#define RESET_FIFO         0xD2
#define DIRECT_SW_TRIG         0xD3
#define DIRECT_ACK         0xD4
#define DIRECT_DATA_READY         0xD5
#define PULSE_AMPLITUDE         0xD6
#define USB_MODE         0xD7

// control registers
#define DEV_REV         0xD0
#define RESET           0xD1
#define RESET_FIFO         0xD2
#define DIRECT_SW_TRIG         0xD3
#define DIRECT_ACK         0xD4
#define DIRECT_DATA_READY         0xD5
#define PULSE_AMPLITUDE         0xD6
#define USB_MODE         0xD7



static struct usb_device *device;
static struct usb_class_driver class;
static unsigned char bulk_buf[MAX_PKT_SIZE];

struct usb_ml {
    /* One structure for each connected device */
};


static int opbox_open(struct inode *i, struct file *f)
{
    return 0;
}
static int opbox_close(struct inode *i, struct file *f)
{
    return 0;
}
static ssize_t opbox_read(struct file *f, char __user *buf, size_t cnt, loff_t *off)
{
    int retval;
    int read_cnt;

    /* Read the data from the bulk endpoint */
    retval = usb_bulk_msg(device, usb_rcvbulkpipe(device, BULK_EP_IN),
            bulk_buf, MAX_PKT_SIZE, &read_cnt, 5000);
    if (retval)
    {
        printk(KERN_ERR "Bulk message returned %d\n", retval);
        return retval;
    }
    if (copy_to_user(buf, bulk_buf, MIN(cnt, read_cnt)))
    {
        return -EFAULT;
    }

    return MIN(cnt, read_cnt);
}
static ssize_t opbox_write(struct file *f, const char __user *buf, size_t cnt,
                                    loff_t *off)
{
    int retval;
    int wrote_cnt = MIN(cnt, MAX_PKT_SIZE);

    if (copy_from_user(bulk_buf, buf, MIN(cnt, MAX_PKT_SIZE)))
    {
        return -EFAULT;
    }

    /* Write the data into the bulk endpoint */
    retval = usb_bulk_msg(device, usb_sndbulkpipe(device, BULK_EP_OUT),
            bulk_buf, MIN(cnt, MAX_PKT_SIZE), &wrote_cnt, 5000);
    if (retval)
    {
        printk(KERN_ERR "Bulk message returned %d\n", retval);
        return retval;
    }

    return wrote_cnt;
}

static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .open = opbox_open,
    .release = opbox_close,
    .read = opbox_read,
    .write = opbox_write,
};

// called when plugged in, if does not other driver  
static int op_box(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct usb_host_interface *iface_desc;  // An array of interface structures containing all of the alternate settings that may
    //  be selected for this interface. Each struct usb_host_interface consists of a set of endpoint configurations as
    // defined by the struct usb_host_endpoint structure described above. Note that these interface structures are in no particular order.
    struct usb_endpoint_descriptor *endpoint;
    int i;
    
    iface_desc = interface->cur_altsetting;  // A pointer into the array altsetting , denoting the currently active setting for this interface
    printk(KERN_INFO "OPBox i/f %d now plugged in: (%04X:%04X)\n",
            iface_desc->desc.bInterfaceNumber,
            id->idVendor, id->idProduct); // USB_DEVICE_INTERFACE_NUMBER 
    printk(KERN_INFO "ID->bNumEndpoints: %02X\n",
            iface_desc->desc.bNumEndpoints);
    printk(KERN_INFO "ID->bInterfaceClass: %02X\n",
            iface_desc->desc.bInterfaceClass); // describe a usb device with a specific interface class

    for (i = 0; i < iface_desc->desc.bNumEndpoints; i++)
    {
        endpoint = &iface_desc->endpoint[i].desc;

        printk(KERN_INFO "ED[%d]->bEndpointAddress: 0x%02X\n",
                i, endpoint->bEndpointAddress);
        printk(KERN_INFO "ED[%d]->bmAttributes: 0x%02X\n",
                i, endpoint->bmAttributes); // This is the type of endpoint.USB_ENDPOINT_XFER_ISOC , USB_ENDPOINT_XFER_BULK , or of type USB_ENDPOINT_XFER_INT
        printk(KERN_INFO "ED[%d]->wMaxPacketSize: 0x%04X (%d)\n",
                i, endpoint->wMaxPacketSize,
                endpoint->wMaxPacketSize);
    }
 
    int retval;
    struct usb_ml *dev = NULL; // contains run-time status information about the connected device
    device = interface_to_usbdev(interface);

    if (! device)
    {
        DBG_ERR("device is NULL");
        goto exit;
    }

    dev = kzalloc(sizeof(struct usb_ml), GFP_KERNEL); // allocate memory. The memory is set to zero.
    // GFP : get free pages ,  GFP_KERNEL : Memory for the kernel data structures, DMAable memory, inode cache
    if (! dev)
    {
        DBG_ERR("cannot allocate memory for struct usb_ml");
        retval = -ENOMEM;
        goto exit;
    }

    class.name = "usb/opbox%d";
    class.fops = &fops;
    if ((retval = usb_register_dev(interface, &class)) < 0)
    {
        /* Something prevented us from registering this driver */
        printk(KERN_ERR "Not able to get a minor for this device.");
    }
    else
    {
        // If the USB driver bound to this interface uses the USB major number, this variable contains the minor number assigned by the USB core 
        //to the interface. This is valid only after a successful call to usb_register_dev 
        printk(KERN_INFO "Minor obtained: %d\n", interface->minor);
    }

    return retval;
}

static void op_box_disconnect(struct usb_interface *interface)
{
    printk(KERN_INFO "op_box driver removed\n");
    usb_deregister_dev(interface, &class);
}

static struct usb_device_id op_box_table[] =
{
    { USB_DEVICE(0x0547, 0x1003) },
    {} /* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, op_box_table);

static struct usb_driver op_box_driver =
{
    .name = "op_box_driver",
    .id_table = op_box_table,
    .probe = op_box,
    .disconnect = op_box_disconnect,
};

static int __init op_box_init(void)
{
     int result;

    /* Register this driver with the USB subsystem */
    if ((result = usb_register(&op_box_driver)))
    {
        printk(KERN_ERR "usb_register failed. Error number %d", result);
    }
    return result;
}

static void __exit op_box_exit(void)
{
    usb_deregister(&op_box_driver);
}

module_init(op_box_init);
module_exit(op_box_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mohammad Vatandoost mohammad19951374.s@gmail.com");
MODULE_DESCRIPTION("OPBOX Driver");