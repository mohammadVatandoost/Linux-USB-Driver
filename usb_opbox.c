/*
 * Dream Cheeky USB Missile Launcher driver
 *
 * Copyright (C) 2007 Matthias Vallentin <vallentin@icsi.berkeley.edu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * derived from USB Skeleton driver 2.0
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 * also inspired from LEGO USB Tower driver
 * Copyright (C) 2003 David Glance <davidgsf@sourceforge.net>
 *               2001-2004 Juergen Stuber <starblue@users.sourceforge.net>
 *
 * Notes:
 * - Apparently it fails sometimes to submit the correction control URB in the
 *   interrupt-in-endpoint hanlder (-EINVAL).
 */


#include "usb_opbox.h" 






MODULE_DEVICE_TABLE (usb, op_box_table);

static int debug_level = DEBUG_LEVEL_INFO;
static int debug_trace = 0;
module_param(debug_level, int, S_IRUGO | S_IWUSR);
module_param(debug_trace, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "debug level (bitmask)");
MODULE_PARM_DESC(debug_trace, "enable function tracing");

/* Prevent races between open() and disconnect */
static DEFINE_MUTEX(disconnect_mutex);
static struct usb_driver op_box_driver;

static inline void op_box_debug_data(const char *function, int size, const unsigned char *data)
{
	int i;
	if ((debug_level & DEBUG_LEVEL_DEBUG) == DEBUG_LEVEL_DEBUG) {
		printk(KERN_DEBUG "[debug] %s: length = %d, data = ",
		       function, size);
		for (i = 0; i < size; ++i)
			printk("%.2x ", data[i]);
		printk("\n");
	}
}


static void op_box_abort_transfers(struct usb_opbox *dev)
{
	if (! dev) {
		DBG_ERR("dev is NULL");
		return;
	}

	if (! dev->udev) {
		DBG_ERR("udev is NULL");
		return;
	}

	if (dev->udev->state == USB_STATE_NOTATTACHED) {
		DBG_ERR("udev not attached");
		return;
	}

	/* Shutdown transfer */
	if (dev->int_in_running) {
		dev->int_in_running = 0;
		mb();
		if (dev->int_in_urb)
			usb_kill_urb(dev->int_in_urb);
	}

	if (dev->ctrl_urb)
		usb_kill_urb(dev->ctrl_urb);
}

static inline void op_box_delete(struct usb_opbox *dev)
{
	op_box_abort_transfers(dev);

	/* Free data structures. */
	if (dev->int_in_urb)
		usb_free_urb(dev->int_in_urb);
	if (dev->ctrl_urb)
		usb_free_urb(dev->ctrl_urb);

	kfree(dev->int_in_buffer);
	kfree(dev->ctrl_buffer);
	kfree(dev->ctrl_dr);
	kfree(dev);
}

static void op_box_ctrl_callback(struct urb *urb)
{
	DBG_INFO("op_box_ctrl_callback");
	struct usb_opbox *dev = urb->context;
	dev->correction_required = 0;	/* TODO: do we need race protection? */
}

static void op_box_int_in_callback(struct urb *urb)
{
	struct usb_opbox *dev = urb->context;
	int retval;
    DBG_INFO("op_box_int_in_callback");
	op_box_debug_data(__FUNCTION__, urb->actual_length, urb->transfer_buffer);

	if (urb->status) {
		if (urb->status == -ENOENT ||
				urb->status == -ECONNRESET ||
				urb->status == -ESHUTDOWN) {
			return;
		} else {
			DBG_ERR("non-zero urb status (%d)", urb->status);
			goto resubmit; /* Maybe we can recover. */
		}
	}

	if (urb->actual_length > 0) {
		spin_lock(&dev->cmd_spinlock); //spinlock itself will guarantee the global lock, so it will guarantee that
                                       // there is only one thread-of-control within the region(s) protected by that lock
         DBG_INFO("init callback actual_length: %d",urb->actual_length);
        // for specefic device
		// if (dev->int_in_buffer[0] & ML_MAX_UP && dev->command & ML_UP) {
		// 	dev->command &= ~ML_UP;
		// 	dev->correction_required = 1;
		// } else if (dev->int_in_buffer[0] & ML_MAX_DOWN &&
		// 		dev->command & ML_DOWN) {
		// 	dev->command &= ~ML_DOWN;
		// 	dev->correction_required = 1;
		// }

		// if (dev->int_in_buffer[1] & ML_MAX_LEFT
		//     && dev->command & ML_LEFT) {
		// 	dev->command &= ~ML_LEFT;
		// 	dev->correction_required = 1;
		// } else if (dev->int_in_buffer[1] & ML_MAX_RIGHT
		// 	   && dev->command & ML_RIGHT) {
		// 	dev->command &= ~ML_RIGHT;
		// 	dev->correction_required = 1;
		// }


		// if (dev->correction_required) {
		// 	dev->ctrl_buffer[0] = dev->command;
		// 	spin_unlock(&dev->cmd_spinlock);
		// 	retval = usb_submit_urb(dev->ctrl_urb, GFP_ATOMIC);
		// 	if (retval) {
		// 		DBG_ERR("submitting correction control URB failed (%d)",
		// 				retval);
		// 	}
		// } else {
		// 	spin_unlock(&dev->cmd_spinlock);
		// }

        spin_unlock(&dev->cmd_spinlock);
	}

resubmit:
	/* Resubmit if we're still running. */
	if (dev->int_in_running && dev->udev) {
		retval = usb_submit_urb(dev->int_in_urb, GFP_ATOMIC);
		if (retval) {
			DBG_ERR("resubmitting urb failed (%d)", retval);
			dev->int_in_running = 0;
		}
	}
}


static int op_box_open(struct inode *inode, struct file *file)
{
	struct usb_opbox *dev = NULL;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	DBG_INFO("op_box_open Open device");
	subminor = iminor(inode);

	mutex_lock(&disconnect_mutex);

	interface = usb_find_interface(&op_box_driver, subminor);
	if (! interface) {
		DBG_ERR("can't find device for minor %d", subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (! dev) {
		retval = -ENODEV;
		goto exit;
	}

	/* lock this device */
	if (down_interruptible (&dev->sem)) {
		DBG_ERR("sem down failed");
		retval = -ERESTARTSYS;
		goto exit;
	}

	/* Increment our usage count for the device. */
	++dev->open_count;
	if (dev->open_count > 1)
		DBG_DEBUG("open_count = %d", dev->open_count);

	/* Initialize interrupt URB. */
    // is a helper function to properly initialize a urb to be sent to an interrupt endpoint of a USB device
	// usb_fill_int_urb(dev->int_in_urb, dev->udev,
	// 		usb_rcvintpipe(dev->udev, dev->int_in_endpoint->bEndpointAddress), // Specifies an interrupt IN endpoint for the specified USB device with the  specified endpoint number
	// 		dev->int_in_buffer,
	// 		le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize), // convert a value to whatever the CPU uses
	// 		op_box_int_in_callback,
	// 		dev,
	// 		dev->int_in_endpoint->bInterval); // set interval zero because is not isochronous.

	// dev->int_in_running = 1;
	// mb();

	// retval = usb_submit_urb(dev->int_in_urb, GFP_KERNEL);
	// if (retval) {
	// 	DBG_ERR("submitting int urb failed (%d)", retval);
	// 	dev->int_in_running = 0;
	// 	--dev->open_count;
	// 	goto unlock_exit;
	// }

	/* Save our object in the file's private structure. */
	file->private_data = dev;

     DBG_INFO("op_box_open Opened successfully");
unlock_exit:
	up(&dev->sem);

exit:
	mutex_unlock(&disconnect_mutex);
	return retval;
}

static int op_box_release(struct inode *inode, struct file *file)
{
	struct usb_opbox *dev = NULL;
	int retval = 0;

	DBG_INFO("Release driver");
	dev = file->private_data;

	if (! dev) {
		DBG_ERR("dev is NULL");
		retval =  -ENODEV;
		goto exit;
	}

	/* Lock our device */
	if (down_interruptible(&dev->sem)) {
		retval = -ERESTARTSYS;
		goto exit;
	}

	if (dev->open_count <= 0) {
		DBG_ERR("device not opened");
		retval = -ENODEV;
		goto unlock_exit;
	}

	if (! dev->udev) {
		DBG_DEBUG("device unplugged before the file was released");
		up (&dev->sem);	/* Unlock here as op_box_delete frees dev. */
		op_box_delete(dev);
		goto exit;
	}

	if (dev->open_count > 1)
		DBG_DEBUG("open_count = %d", dev->open_count);

	op_box_abort_transfers(dev);
	--dev->open_count;

unlock_exit:
	up(&dev->sem);

exit:
	return retval;
}

static ssize_t op_box_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct usb_opbox *dev;
	int retval = 0;
	bool policy;
   // #if MISSILE_LAUNCHER == ML_THUNDER
   // 	static int ml_led = 1;
   // #endif
	// u8 buf[8];
	// __u8 cmd = ML_STOP;

	DBG_INFO("op_box_write");
	dev = file->private_data;

	// /* Lock this object. */
	// if (down_interruptible(&dev->sem)) {
	// 	retval = -ERESTARTSYS;
	// 	goto exit;
	// }

	/* Verify that the device wasn't unplugged. */
	if (! dev->udev) {
		retval = -ENODEV;
		DBG_ERR("No device or device unplugged (%d)", retval);
		goto exit;
	}

	// /* Verify that we actually have some data to write. */
	// if (count == 0)
	// 	goto unlock_exit;

	// /* We only accept one-byte writes. */
	// if (count != 1)
	// 	count = 1;

	// if (copy_from_user(&cmd, user_buf, count)) {
	// 	retval = -EFAULT;
	// 	goto unlock_exit;
	// }

	// policy = (cmd == ML_STOP || cmd == ML_UP || cmd == ML_DOWN
	// 	  || cmd == ML_LEFT || cmd == ML_RIGHT || cmd == ML_UP_LEFT
	// 	  || cmd == ML_DOWN_LEFT || cmd == ML_UP_RIGHT
	// 	  || cmd == ML_DOWN_RIGHT || cmd == ML_FIRE);
    // #if MISSILE_LAUNCHER == ML_THUNDER
	//    policy = policy || (cmd == ML_LED);
    // #endif
	// if (!policy) {
	// 	DBG_ERR("illegal command issued");
	// 	retval = -0x2a;		/* scnr */
	// 	goto unlock_exit;
	// }

	// memset(&buf, 0, sizeof(buf));
    // #if MISSILE_LAUNCHER == ML_THUNDER
	//   if (cmd == ML_LED) {
	//  	buf[0] = ML_LED;
	//  	buf[1] = ml_led;
	//  	ml_led = 1 - ml_led;
	//   } else {
	// 	buf[0] = 0x02;
	// 	buf[1] = cmd;
    // 	}
    // #else
	//     buf[0] = 0x02;
    // 	buf[1] = cmd;
    // #endif
	// /* The interrupt-in-endpoint handler also modifies dev->command. */
	// spin_lock(&dev->cmd_spinlock);
	// dev->command = cmd;
	// spin_unlock(&dev->cmd_spinlock);
    char *dmadata = kmalloc(count, GFP_KERNEL);
	if (!dmadata)
		return -ENOMEM;

	dev = (struct usb_opbox *)file->private_data;
	
	// DBG_INFO("op_box_read control: %s", buffer);
    DBG_INFO("buffer[0] : %02x", buffer[0]);
    DBG_INFO("buffer[1] : %02x", buffer[1]);
	dmadata[0] = 0x01;
	dmadata[1] = 0x00;
	retval = usb_control_msg(dev->udev,
			usb_sndctrlpipe(dev->udev, 0),
			OPBOX_CTRL_REGISTER_WRITE_VENDOER_REQUEST,
			OPBOX_CTRL_REGISTER_WRITE_REQUEST_TYPE,
			OPBOX_CTRL_REGISTER_WRITE_VALUE,
			POWER_CTLR,
			(void *)dmadata,
			count,
			2000);


	if (retval < 0) {
		DBG_ERR("usb_control_msg failed (%d)", retval);
		// goto unlock_exit;
	} else {
		DBG_INFO("usb_control_msg writed succesfully");
		/* We should have written only one byte. */
    	retval = count;
	}

	

exit:
	return retval;
}

static ssize_t op_box_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
   struct usb_opbox *dev;
	int retval = 0;
    // u8 buf[2];
	// memset(&buf, 0, sizeof(buf));
	// void *dmadata = kmalloc(count, GFP_KERNEL);
	char *dmadata = kmalloc(count, GFP_KERNEL);
	if (!dmadata)
		return -ENOMEM;
	dev = (struct usb_opbox *)file->private_data;
	
	// DBG_INFO("op_box_read control: %s", buffer);
    DBG_INFO("buffer[0] : %02x", buffer[0]);
    DBG_INFO("buffer[1] : %02x", buffer[1]);
    if(count == 2) {
		if(buffer[0] == READ_CONTROL) {
             DBG_INFO("op_box_read control read");
	         retval = usb_control_msg(dev->udev,
			   usb_rcvctrlpipe(dev->udev, 0),
			   OPBOX_CTRL_REGISTER_READ_VENDOER_REQUEST,
			   OPBOX_CTRL_REGISTER_READ_REQUEST_TYPE,
			   OPBOX_CTRL_REGISTER_READ_VALUE,
			   POWER_CTLR,
			   (void *)dmadata,  // &dmadata,
			   sizeof(count),
			   2000);

    	    if (retval < 0) {
		       DBG_ERR("usb_control_msg failed (%d)", retval);
		       // goto unlock_exit;
	        } else {
		      DBG_INFO("op_box_read usb_control_msg sended succefully");
			  // if (copy_to_user(buffer, buf, count)) {
               //     retval = -EFAULT;
			  // } else {
              //     retval = count;
			  // 	DBG_INFO("op_box_read usb_control_msg readed succefully: %d", retval);
			  // }
			  retval = count;
			   memset(&buffer, 0, count);
			   memcpy(buffer, dmadata, count);
			   DBG_INFO("buffer[0] : %02x", buffer[0]);
               DBG_INFO("buffer[1] : %02x", buffer[1]);
	        }
		}
      

// 			int usb_control_msg(struct usb_device *dev, unsigned int pipe,
// __u8 request, __u8 requesttype,
// __u16 value, __u16 index,
// void *data, __u16 size, int timeout);


		// memcpy(buffer, dmadata, count);
     	kfree(dmadata);
	} else {
      DBG_INFO("op_box_read control data read");
	  /* do a blocking bulk read to get data from the device */
    	// retval = usb_bulk_msg(dev->udev,
		// 	      usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
		// 	      dev->bulk_in_buffer,
		// 	      min(dev->bulk_in_size, count),
		// 	      &count, HZ*10);
			/* if the read was successful, copy the data to userspace */
	    if (!retval) {
		    if (copy_to_user(buffer, dev->bulk_in_buffer, count)) {
                retval = -EFAULT;
			} else {
                retval = count;
			}			   			   
	    } else {
           DBG_ERR("read msg failed (%d)", retval);
		}
	}
	
	

	

	return retval;
}


static ssize_t show_switches(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_opbox *opbox = usb_get_intfdata(intf);

	return sprintf(buf, "%d\n", opbox->testData);
}

static DEVICE_ATTR( switches, S_IRUGO, show_switches, NULL );
// static DEVICE_ATTR(testData, S_IRUGO, show_test_data, NULL);

static struct file_operations opbox_fops = {
	.owner =	THIS_MODULE,
	.write =	op_box_write,
	.open =		op_box_open,
	.release =	op_box_release,
	.read = op_box_read,
};

static struct usb_class_driver opbox_class = {
	.name = "usb/opbox%d",
	.fops = &opbox_fops,
    .minor_base = ML_MINOR_BASE,
};

static int op_box_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_opbox *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i, int_end_size;
	int retval = -ENODEV;
	DBG_INFO("Probe missile launcher");

	if (! udev) {
		DBG_ERR("udev is NULL");
		goto exit;
	}

	dev = kzalloc(sizeof(struct usb_opbox), GFP_KERNEL);
	if (! dev) {
		DBG_ERR("cannot allocate memory for struct usb_opbox");
		retval = -ENOMEM;
		goto exit;
	}

	// dev->command = ML_STOP;

	sema_init(&dev->sem, 1);
	spin_lock_init(&dev->cmd_spinlock);

	dev->udev = udev;
	dev->interface = interface;
	
	iface_desc = interface->cur_altsetting;
    printk(KERN_INFO "OPBox i/f %d now plugged in: (%04X:%04X)\n",
            iface_desc->desc.bInterfaceNumber,
            id->idVendor, id->idProduct); // USB_DEVICE_INTERFACE_NUMBER 
    printk(KERN_INFO "ID->bNumEndpoints: %02X\n",
            iface_desc->desc.bNumEndpoints);
	//bInterfaceClass=0xFF means that the interface is vendor-specific, and the protocol to talk to it is proprietary
    printk(KERN_INFO "ID->bInterfaceClass: %02X\n",
            iface_desc->desc.bInterfaceClass); // describe a usb device with a specific interface class
	/* Set up interrupt endpoint information. */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
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
			// buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = endpoint->wMaxPacketSize;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(endpoint->wMaxPacketSize, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				printk(KERN_INFO "err :Could not allocate bulk_in_buffer\n");
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



    // -----------------------
        dev->testData = 24;
	//-------------------------

	// if (! dev->int_in_endpoint) {
	// 	DBG_ERR("could not find interrupt in endpoint");
	// 	goto error;
	// }

	// int_end_size = le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize);

	// dev->int_in_buffer = kmalloc(int_end_size, GFP_KERNEL);
	// if (! dev->int_in_buffer) {
	// 	DBG_ERR("could not allocate int_in_buffer");
	// 	retval = -ENOMEM;
	// 	goto error;
	// }

	// dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	// if (! dev->int_in_urb) {
	// 	DBG_ERR("could not allocate int_in_urb");
	// 	retval = -ENOMEM;
	// 	goto error;
	// }

	/* Set up the control URB. */
	dev->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL); // zero : If you do not want to create an isochronous urb, 
	                                              // this variable should be set to 0
	if (! dev->ctrl_urb) {
		DBG_ERR("could not allocate ctrl_urb");
		retval = -ENOMEM;
		goto error;
	}

	dev->ctrl_buffer = kzalloc(OPBOX_CTRL_REGISTER_WRITE_REQUEST_LENGTH, GFP_KERNEL);
	if (! dev->ctrl_buffer) {
		DBG_ERR("could not allocate ctrl_buffer");
		retval = -ENOMEM;
		goto error;
	}

	dev->ctrl_dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
	if (! dev->ctrl_dr) {
		DBG_ERR("could not allocate usb_ctrlrequest");
		retval = -ENOMEM;
		goto error;
	}

    // Power Enable
	dev->ctrl_dr->bRequestType = OPBOX_CTRL_REGISTER_WRITE_VENDOER_REQUEST;
	dev->ctrl_dr->bRequest = OPBOX_CTRL_REGISTER_WRITE_REQUEST_TYPE;
	dev->ctrl_dr->wValue = cpu_to_le16(0x00);
	dev->ctrl_dr->wIndex = cpu_to_le16(POWER_CTLR);
	dev->ctrl_dr->wLength = cpu_to_le16(OPBOX_CTRL_REGISTER_WRITE_REQUEST_LENGTH);
    dev->ctrl_buffer[0] = 0x01;
	dev->ctrl_buffer[1] = 0x00;
	usb_fill_control_urb(dev->ctrl_urb, dev->udev,
			usb_sndctrlpipe(dev->udev, 0), // zero is control endpoint
			(unsigned char *)dev->ctrl_dr,
			dev->ctrl_buffer,
			OPBOX_CTRL_REGISTER_WRITE_REQUEST_LENGTH,
			op_box_ctrl_callback,
			dev);

    int result = usb_submit_urb( dev->ctrl_urb, GFP_KERNEL ) ;
    if(result < 0) {
           printk( KERN_INFO "control urb submitted wrong %d", result );
	} else {
		printk( KERN_INFO "control urb submitted correctly %d", result );
	}
    
	/* Retrieve a serial. */
	if (! usb_string(udev, udev->descriptor.iSerialNumber,
			 dev->serial_number, sizeof(dev->serial_number))) {
		DBG_ERR("could not retrieve serial number");
		goto error;
	}
    
	/* Save our data pointer in this interface device. */
	usb_set_intfdata(interface, dev); 
    device_create_file(&interface->dev, &dev_attr_switches);
	/* We can register the device now, as it is ready. */
	retval = usb_register_dev(interface, &opbox_class);
	if (retval) {
		DBG_ERR("not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	dev->minor = interface->minor;

	DBG_INFO("USB missile launcher now attached to /dev/opbox%d, retval: %d",
			interface->minor - ML_MINOR_BASE , retval);

exit:
	return retval;

error:
	op_box_delete(dev);
	return retval;
}

static void op_box_disconnect(struct usb_interface *interface)
{
	struct usb_opbox *dev;
	int minor;

	mutex_lock(&disconnect_mutex);	/* Not interruptible */

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
    device_remove_file(&interface->dev, &dev_attr_switches);
	down(&dev->sem); /* Not interruptible */

	minor = dev->minor;

	/* Give back our minor. */
	usb_deregister_dev(interface, &opbox_class);

	/* If the device is not opened, then we clean up right now. */
	if (! dev->open_count) {
		up(&dev->sem);
		op_box_delete(dev);
	} else {
		dev->udev = NULL;
		up(&dev->sem);
	}

	mutex_unlock(&disconnect_mutex);

	DBG_INFO("USB missile launcher /dev/opbox%d now disconnected",
			minor - ML_MINOR_BASE);
}

static struct usb_driver op_box_driver = {
	.name = "op_box_driver",
	.id_table = op_box_table,
	.probe = op_box_probe,
	.disconnect = op_box_disconnect,
};


static int __init usb_opbox_init(void)
{
	int result;

	DBG_INFO("Register driver");
	result = usb_register(&op_box_driver);
	if (result) {
		DBG_ERR("registering driver failed");
	} else {
		DBG_INFO("driver registered successfully");
	}

	return result;
}


static void __exit usb_opbox_exit(void)
{
	usb_deregister(&op_box_driver);
	DBG_INFO("module deregistered");
}

module_init(usb_opbox_init);
module_exit(usb_opbox_exit);

MODULE_AUTHOR("Matthias Vallentin");
MODULE_LICENSE("GPL");