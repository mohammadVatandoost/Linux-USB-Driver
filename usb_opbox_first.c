#include "usb_opbox.h" 


static struct usb_device *device;

static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .open = opbox_open,
    .release = opbox_close,
    .read = opbox_read,
    .write = opbox_write,
};

/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */
static struct usb_class_driver opbox_class = {
	.name = "usb/opbox%d";
	.fops = &fops,
};


static unsigned char bulk_buf[MAX_PKT_SIZE];
// one structure for each device
struct usb_opbox {
	struct usb_device *	udev;			/* the usb device for this device */
	struct usb_interface *	interface;		/* the interface for this device */
	unsigned char *		bulk_in_buffer;		/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	struct kref		kref; //krefs allow you to add reference counters to your objects.  If you
                          // have objects that are used in multiple places and passed around, and
                          // you don't have refcounts, your code is almost certainly broken.  If
                          // you want refcounts, krefs are the way to go.
};

// container_of : This macro takes a pointer to a field named container_field , within a structure of
// type container_type , and returns a pointer to the containing structure
#define to_opbox_dev(d) container_of(d, struct usb_opbox, kref)

static struct usb_driver op_box_driver;

static void opbox_delete(struct kref *kref)
{	
	struct usb_opbox *dev = to_opbox_dev(kref);

	usb_put_dev(dev->udev);
	kfree (dev->bulk_in_buffer);
	kfree (dev);
}




MODULE_DEVICE_TABLE (usb, op_box_table);


static int opbox_open(struct inode *inode, struct file *file)
{
    struct usb_opbox *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);
                            
	interface = usb_find_interface(&op_box_driver, subminor);
	if (!interface) {
		err ("%s - error, can't find device for minor %d",
		     __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}
	
	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

exit:
	return retval;
}
static int opbox_close(struct inode *i, struct file *f)
{
    return 0;
}
static ssize_t opbox_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
   struct usb_opbox *dev;
	int retval = 0;

	dev = (struct usb_opbox *)file->private_data;
	
	/* do a blocking bulk read to get data from the device */
	retval = usb_bulk_msg(dev->udev,
			      usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
			      dev->bulk_in_buffer,
			      min(dev->bulk_in_size, count),
			      &count, HZ*10);

	/* if the read was successful, copy the data to userspace */
	if (!retval) {
		if (copy_to_user(buffer, dev->bulk_in_buffer, count))
			retval = -EFAULT;
		else
			retval = count;
	}

	return retval;
}

static ssize_t opbox_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *ppos)
{
    struct usb_opbox *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;

	dev = (struct usb_opbox *)file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		goto exit;

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_buffer_alloc(dev->udev, count, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}
	if (copy_from_user(buf, user_buffer, count)) {
		retval = -EFAULT;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, count, opbox_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		err("%s - failed submitting write urb, error %d", __FUNCTION__, retval);
		goto error;
	}

	/* release our reference to this urb, the USB core will eventually free it entirely */
	usb_free_urb(urb);

exit:
	return count;

error:
	usb_buffer_free(dev->udev, count, buf, urb->transfer_dma);
	usb_free_urb(urb);
	kfree(buf);
	return retval;
}

static void opbox_write_bulk_callback(struct urb *urb, struct pt_regs *regs)
{
	/* sync/async unlink faults aren't errors */
	if (urb->status && 
	    !(urb->status == -ENOENT || 
	      urb->status == -ECONNRESET ||
	      urb->status == -ESHUTDOWN)) {
		dbg("%s - nonzero write bulk status received: %d",
		    __FUNCTION__, urb->status);
	}

	/* free up our allocated buffer */
	usb_buffer_free(urb->dev, urb->transfer_buffer_length, 
			urb->transfer_buffer, urb->transfer_dma);
}

// called when plugged in, if does not other driver  
static int op_box(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct usb_host_interface *iface_desc;  // An array of interface structures containing all of the alternate settings that may
    //  be selected for this interface. Each struct usb_host_interface consists of a set of endpoint configurations as
    // defined by the struct usb_host_endpoint structure described above. Note that these interface structures are in no particular order.
    struct usb_endpoint_descriptor *endpoint;
    int i;
    int retval;
    struct usb_opbox *dev = NULL; // contains run-time status information about the connected device

    /* allocate memory for our device state and initialize it */
	dev = kmalloc(sizeof(struct usb_skel), GFP_KERNEL);// allocate memory. The memory is set to zero.
    // GFP : get free pages ,  GFP_KERNEL : Memory for the kernel data structures, DMAable memory, inode cache
	if (dev == NULL) {
		err("Out of memory");
		goto error;
	}
	memset(dev, 0x00, sizeof (*dev));
	kref_init(&dev->kref);
    
    dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

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

        if (!dev->bulk_in_endpointAddr &&
		    (endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {

                        printk(KERN_INFO "op_box driver found a bulk in endpoint\n");
			/* we found a bulk in endpoint */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				err("Could not allocate bulk_in_buffer");
				goto error;
			}
		}

		if (!dev->bulk_out_endpointAddr &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
                        printk(KERN_INFO "op_box driver found a bulk out endpoint\n");
			/* we found a bulk out endpoint */
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
    }

    if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		err("Could not find both bulk-in and bulk-out endpoints");
		goto error;
	}
 
    /* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);//Because the USB driver needs to retrieve the local data structure that is associated
                                      // with this struct usb_interface later in the lifecycle of the device



     /* we can register the device now, as it is ready */   
    if ((retval = usb_register_dev(interface, &opbox_class)) < 0)
    {
        /* Something prevented us from registering this driver */
        printk(KERN_ERR "Not able to get a minor for this device.");
        usb_set_intfdata(interface, NULL);
		goto error;
    }
    else
    {
        // If the USB driver bound to this interface uses the USB major number, this variable contains the minor number assigned by the USB core 
        //to the interface. This is valid only after a successful call to usb_register_dev 
        //All devices associated with this driver are created with unique, increasing minor numbers beginning with this value
        printk(KERN_INFO "Minor obtained: %d\n", interface->minor);
        /* let the user know what node this device is now attached to */
	    info("USB OpBox device now attached to USBSkel-%d", interface->minor);
    }

    return retval;

    error:
	if (dev)
		kref_put(&dev->kref, opbox_delete);
	return retval;
}

static void op_box_disconnect(struct usb_interface *interface)
{
    struct usb_opbox *dev;
	int minor = interface->minor;

	/* prevent opbox_open() from racing op_box_disconnect() */
	lock_kernel();

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &opbox_class);

	unlock_kernel();

	/* decrement our usage count */
	kref_put(&dev->kref, opbox_delete);

	info("USB Opbox #%d now disconnected", minor);
    printk(KERN_INFO "op_box driver removed\n");
}

static struct usb_driver op_box_driver =
{
    .name = "op_box_driver",  // It must be unique among all USB drivers in the kernel
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