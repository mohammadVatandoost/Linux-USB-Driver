#include <linux/module.h>
#include <linux/init.h>

#include <linux/slab.h>			/* kmalloc() */
#include <linux/usb.h>			/* USB stuff */
#include <linux/mutex.h>		/* mutexes */
#include <linux/ioctl.h>

#include <linux/uaccess.h> // for copy_from_user()

#define READ_CONTROL	0x00

#define OPBOX_CTRL_BUFFER_SIZE 	64 // Direction: IN/OUT, Type: Control, Packets: max. 64Bytes;

#define OPBOX_CTRL_REQUEST_TYPE	0x21 // http://support.tenasys.com/INtimeHelp_4/st_usbctrlrequest.html
#define OPBOX_CTRL_REQUEST		0x09
#define OPBOX_CTRL_VALUE		0x0
#define OPBOX_CTRL_INDEX		0x0

#define OPBOX_CTRL_REGISTER_WRITE_VENDOER_REQUEST 0xE0
#define OPBOX_CTRL_REGISTER_WRITE_REQUEST_TYPE 0x40
#define OPBOX_CTRL_REGISTER_WRITE_VALUE 0x00 // this field is not used in register write
#define OPBOX_CTRL_REGISTER_WRITE_REQUEST_LENGTH 2

#define OPBOX_CTRL_REGISTER_READ_VENDOER_REQUEST 0xE1
#define OPBOX_CTRL_REGISTER_READ_REQUEST_TYPE 0xC0
#define OPBOX_CTRL_REGISTER_READ_VALUE 0x00 // this field is not used in register write
#define OPBOX_CTRL_REGISTER_READ_REQUEST_LENGTH 2

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
#define DEV_REV         0x00

#define POWER_CTLR           0x02
#define POWER_ENABLE           0x0001

#define PACKET_LEN         0x04
#define FRAME_IDX         0x06
#define FRAME_CNT         0x08
#define CAPT_REG         0x0A
#define GP_INPUTS         0x0C
#define GP_OUTPUTS         0x0E
#define TRIGGER         0x10
#define TRG_OVERRUN           0x12
#define XY_DIVIDER         0x14
#define TIMER         0x16
#define TIMER_CAPT         0x18
#define ANALOG_CTRL         0x1A
#define PULSER_TIME         0x1C
#define BURST         0x1E
#define MEASURE         0x20
#define DELAY           0x22
#define DEPTH_L         0x24
#define DEPTH_H         0x26
#define CONST_GAIN         0x28
#define PEAKDET_CTRL         0x2A
#define PDA_START_L         0x2C
#define PDA_START_H         0x2E
#define PDA_STOP_L         0x30
#define PDA_STOP_H           0x32
#define PDA_REF_VAL         0x34
#define PDA_REF_POS_L         0x36
#define PDA_REF_POS_H         0x38
#define PDA_MAX_VAL         0x3A
#define PDA_MAX_POS_L         0x3C
#define PDA_MAX_POS_H         0x3E

#define PDB_START_L         0x40
#define PDB_START_H           0x42
#define PDB_STOP_L         0x44
#define PDB_STOP_H         0x46
#define PDB_REF_VAL         0x48
#define PDB_REF_POS_L         0x4A
#define PDB_REF_POS_H         0x4C
#define PDB_MAX_VAL         0x4E
#define PDB_MAX_POS_L         0x50
#define PDB_MAX_POS_H           0x52
#define PDC_START_L         0x54
#define PDC_START_H         0x56
#define PDC_STOP_L         0x58
#define PDC_STOP_H         0x5A
#define PDC_REF_VAL         0x5C
#define PDC_REF_POS_L         0x5E
#define PDC_REF_POS_H         0x60
#define PDC_MAX_VAL           0x62
#define PDC_MAX_POS_L         0x64
#define PDC_MAX_POS_H         0x66
#define ENC1_CTRL         0x68
#define ENC1_POS_L         0x6A
#define ENC1_POS_H         0x6C
#define ENC1_CAPT_L         0x6E
#define ENC1_CAPT_H         0x70
#define ENC1_FILTER           0x72
#define ENC2_CTRL         0x74
#define ENC2_POS_L         0x76
#define ENC2_POS_H         0x78
#define ENC2_CAPT_L         0x7A
#define ENC2_CAPT_H         0x7C
#define ENC2_FILTER         0x7E

// ---------------------------------------
#define DEBUG_LEVEL_DEBUG		0x1F
#define DEBUG_LEVEL_INFO		0x0F
#define DEBUG_LEVEL_WARN		0x07
#define DEBUG_LEVEL_ERROR		0x03
#define DEBUG_LEVEL_CRITICAL	0x01

#define DBG_DEBUG(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_DEBUG) == DEBUG_LEVEL_DEBUG) \
	printk( KERN_DEBUG "[debug] %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)
#define DBG_INFO(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_INFO) == DEBUG_LEVEL_INFO) \
	printk( KERN_DEBUG "[info]  %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)
#define DBG_WARN(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_WARN) == DEBUG_LEVEL_WARN) \
	printk( KERN_DEBUG "[warn]  %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)
#define DBG_ERR(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_ERROR) == DEBUG_LEVEL_ERROR) \
	printk( KERN_DEBUG "[err]   %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)
#define DBG_CRIT(fmt, args...) \
if ((debug_level & DEBUG_LEVEL_CRITICAL) == DEBUG_LEVEL_CRITICAL) \
	printk( KERN_DEBUG "[crit]  %s(%d): " fmt "\n", \
			__FUNCTION__, __LINE__, ## args)

#define ML_DEFAULT 0
#define ML_THUNDER 1

#define MISSILE_LAUNCHER ML_DEFAULT


#define ML_CTRL_REQUEST		0x09
#define ML_CTRL_VALUE		0x0
#define ML_CTRL_INDEX		0x0

#define ML_STOP			0x00
#define ML_UP			0x01
#define ML_DOWN			0x02
#if MISSILE_LAUNCHER == ML_THUNDER
#define ML_LED			0x03
#endif
#define ML_LEFT			0x04
#define ML_RIGHT		0x08
#define ML_UP_LEFT		(ML_UP | ML_LEFT)
#define ML_DOWN_LEFT		(ML_DOWN | ML_LEFT)
#define ML_UP_RIGHT		(ML_UP | ML_RIGHT)
#define ML_DOWN_RIGHT		(ML_DOWN | ML_RIGHT)
#define ML_FIRE			0x10

#define ML_MAX_UP		0x80 		/* 80 00 00 00 00 00 00 00 */
#define ML_MAX_DOWN		0x40		/* 40 00 00 00 00 00 00 00 */
#define ML_MAX_LEFT		0x04		/* 00 04 00 00 00 00 00 00 */
#define ML_MAX_RIGHT		0x08 		/* 00 08 00 00 00 00 00 00 */

#ifdef CONFIG_USB_DYNAMIC_MINORS
#define ML_MINOR_BASE	0
#else
#define ML_MINOR_BASE	96
#endif



//-----------------------------------------



#define USB_OPBOX_VENDOR_ID	0x0547
#define USB_OPBOXL_PRODUCT_ID	0x1003


// provides a list of different types of USB devices that this driver supports
static struct usb_device_id op_box_table[] =
{
    { USB_DEVICE(USB_OPBOX_VENDOR_ID, USB_OPBOXL_PRODUCT_ID) },
    {} /* Terminating entry */
};


struct usb_opbox {
	struct usb_device 	*udev;
	struct usb_interface 	*interface;
	unsigned char		minor;
	char			serial_number[8];

	int			open_count;     /* Open count for this port */
	struct 			semaphore sem;	/* Locks this structure */
	spinlock_t		cmd_spinlock;	/* locks dev->command */

	char				*int_in_buffer;
	struct usb_endpoint_descriptor  *int_in_endpoint;
	struct urb 			*int_in_urb;
	int				int_in_running;

	char			*ctrl_buffer; /* 8 byte buffer for ctrl msg */
	struct urb		*ctrl_urb;
	struct usb_ctrlrequest  *ctrl_dr;     /* Setup packet information https://www.beyondlogic.org/usbnutshell/usb6.shtml */
	int			correction_required;
	unsigned char *		bulk_in_buffer;		/* the buffer to receive data */
    size_t			bulk_in_size;		/* the size of the receive buffer */
	__u8			command;/* Last issued command */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	int testData;
};

