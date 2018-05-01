/* vdma.c
 * Linux kernel driver for the Xilinx VDMA engine that works in
 * conjunction with the cmabuffer driver.
 *
 * Steven Bell <sebell@stanford.edu>
 * 15 December 2015, based on earlier work
 * Keyi Zhang <keyi@stanford.edu>
 * Updated the driver to support multiple cameras and DT
 */

// TODO: some of these are probably unnecessary
#include <linux/module.h>
#include <linux/cdev.h> // Character device
#include <linux/slab.h> // kmalloc
#include <asm/io.h> // ioremap and friends
#include <asm/uaccess.h> // Copy to/from userspace pointers
#include <linux/sched.h> // current task struct
#include <linux/fs.h> // File node numbers
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>

#include "common.h"
#include "buffer.h"
#include "ioctl_cmds.h"

MODULE_LICENSE("GPL");

#define CLASSNAME "xilcam" // Shows up in /sys/class
#define DEVNAME "xilcam" // Shows up in /dev

#define N_VDMA_BUFFERS 3 // Number of "live" buffers in the VDMA buffer ring

/* drvdata to hold all info */
struct xilcam_drvdata {
   struct device *dev;
   struct cdev cdev;
   dev_t devt;

   /* control info */
   /* Wait queue to pend on a frame finishing. Threads waiting on this are
    * woken up each time a frame is finished and an interrupt occurs.
    */
   wait_queue_head_t wq_frame;
   /* Whether we have a new (unread) output frame or not */
   atomic_t new_frame;
   /* Handles for buffers in the ring */
   Buffer *vdma_buf[N_VDMA_BUFFERS];
   unsigned char *vdma_controller;
};

struct class *vdma_class;

int debug_level = 3; // 0 is errors only, increasing numbers print more stuff

static int dev_open(struct inode *inode, struct file *file)
{
  int i;
  unsigned long status;
  unsigned char *vdma_controller;
  Buffer **vdma_buf;
  struct xilcam_drvdata *drvdata;

  /* get the drvdata from container */
  drvdata = container_of(inode->i_cdev, struct xilcam_drvdata, cdev);
  file->private_data = drvdata;
  vdma_controller = drvdata->vdma_controller;
  vdma_buf = drvdata->vdma_buf;

  // reset, so we can configure
  iowrite32(0x00010044, vdma_controller + 0x30);

  // Acquire buffers and hand them to the VDMA engine
  for (i = 0; i < N_VDMA_BUFFERS; i++) {
    vdma_buf[i] = acquire_buffer(1920, 1080, 1, 2048);
    if (vdma_buf[i] == NULL) {
      return(-ENOMEM);
    }
    iowrite32(vdma_buf[i]->phys_addr,
              vdma_controller + 0xac + i*4);
  }
  // Set number of buffers
  iowrite32(N_VDMA_BUFFERS, vdma_controller + 0x48);

  status = ioread32(vdma_controller + 0x34);
  DEBUG("dev_open: ioread32 at offset 0x34 returned %08lx\n", status);
  iowrite32(0xffffffff, vdma_controller + 0x34); // clear errors
  DEBUG("dev_open: iowrite32 at offset 0x34 with %08x\n", 0xffffffff);
  status = ioread32(vdma_controller + 0x34);
  DEBUG("dev_open: ioread32 at offset 0x34 returned %08lx\n", status);

  // Run in circular mode, and turn on only the frame complete interrupt
  iowrite32(0x00011043, vdma_controller + 0x30);
  status = ioread32(vdma_controller + 0x30);
  DEBUG("dev_open: ioread32 at offset 0x30 returned %08lx\n", status);
  status = ioread32(vdma_controller + 0x34);
  DEBUG("dev_open: ioread32 at offset 0x34 returned %08lx\n", status);

  // Write the size.  This also commits the settings and begins transfer
  // Horizontal size (1 byte/pixel)
  iowrite32(1920, vdma_controller + 0xa4);
  // Stride (1 byte/pixel)
  iowrite32(2048, vdma_controller + 0xa8);
  // Vertical size, start transfer
  iowrite32(1080, vdma_controller + 0xa0);

  TRACE("dev_open: Started VDMA\n");
  return(0);
}

static int dev_close(struct inode *inode, struct file *file)
{
  int i;
  unsigned char *vdma_controller;
  Buffer **vdma_buf;
  struct xilcam_drvdata *drvdata;

  drvdata = file->private_data;
  vdma_controller = drvdata->vdma_controller;
  vdma_buf = drvdata->vdma_buf;

  // Stop, so we can configure
  iowrite32(0x00010040, vdma_controller + 0x30);
  // Free our collection of big buffers
  for (i = 0; i < N_VDMA_BUFFERS; i++) {
    release_buffer(vdma_buf[i]);
  }

  TRACE("dev_close: Stopped VDMA\n");
  return(0);
}


int grab_image(Buffer* buf, struct xilcam_drvdata *drvdata)
{
  Buffer* tmp;
  unsigned long slot; // Slot VDMA S2MM is working on
  //unsigned long status; // For printing debug messages


  // Wait until there's a new image
  wait_event_interruptible(drvdata->wq_frame,
                           atomic_read(&drvdata->new_frame) == 1);
  atomic_set(&drvdata->new_frame, 0); // Mark the image as read

  // Allocate a new buffer to swap in
  tmp = acquire_buffer(1920, 1080, 1, 2048);

  // If this fails, return failure
  if (tmp == NULL) {
    return(-ENOBUFS);
  }

  // Grab the most recently completed image
  // The current frame store location is in 0x24 (FRMPTR_STS), bits 20-16
  // Note 0x24 (FRMPTR_STS) is only available if C_ENABLE_DEBUG_INFO_12=1
  slot = ioread32(drvdata->vdma_controller + 0x24);
  DEBUG("grab_image: ioread32 at offset 0x24 returned %08lx\n", slot);
  slot = (slot & 0x001F0000) >> 16;
  /*
  // The current frame store location is in 0x28 (PARK_PTR_REG), bits 28-24
  // Note 0x28 (PARK_PTR_REG) is only useful for our purpose
  // if there is no VDMA error (see register 0x34)
  status = ioread32(vdma_controller + 0x34);
  DEBUG("grab_image: ioread32 at offset 0x34 returned %08lx\n", status);
  slot = ioread32(vdma_controller + 0x28);
  DEBUG("grab_image: ioread32 at offset 0x28 returned %08lx\n", slot);
  slot = slot >> 24;
  */

  // Get the previous one, which is the most recently finished
  DEBUG("grab_image: VMDA current working frame in slot %lu\n", slot);
  slot = (slot + N_VDMA_BUFFERS - 1) % N_VDMA_BUFFERS;
  DEBUG("grab_image: most recently finished frame in slot %lu\n", slot);

  // Copy the buffer object for the caller
  *buf = *drvdata->vdma_buf[slot];

  // Replace it with the new buffer
  drvdata->vdma_buf[slot] = tmp;

  // Write the change to the VDMA engine
  iowrite32(drvdata->vdma_buf[slot]->phys_addr,
            drvdata->vdma_controller + 0xac + slot*4);

  // Write the vertical size again so the settings take effect
  iowrite32(1080, drvdata->vdma_controller + 0xa0);

  TRACE("grab_image: replaced %d with %d\n", buf->id,
        drvdata->vdma_buf[slot]->id);
  return(0);
}


long dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  Buffer tmp;
  struct xilcam_drvdata *drvdata;

  drvdata = filp->private_data;

  switch(cmd){
    case GRAB_IMAGE:
      TRACE("ioctl: GRAB_IMAGE\n");
      if(grab_image(&tmp, drvdata) == 0 &&
        access_ok(VERIFY_WRITE, (void*)arg, sizeof(Buffer)))
      {
        TRACE("Copying raw buffer object to user\n");
        // copy_to_user returns number of uncopied bytes, which should be 0
        if(copy_to_user((void*)arg, &tmp, sizeof(Buffer)) != 0){
          return(-EIO);
        }
      }
      else{
        return(-ENOBUFS);
      }
      break;

    default:
      return(-EINVAL); // Unknown command, return an error
      break;
  }
  return(0); // Success
}

struct file_operations fops = {
  // No read/write; everything is handled by ioctl
  .open = dev_open,
  .release = dev_close,
  .unlocked_ioctl = dev_ioctl,
};

// Interrupt handler for when a frame finishes
irqreturn_t frame_finished_handler(int irq, void* dev_id)
{
  struct xilcam_drvdata *drvdata;

  drvdata = dev_id;

  // Acknowledge/clear interrupt
  iowrite32(0x00001000, drvdata->vdma_controller + 0x34);
  // TODO: reset the frame count back to 1?
  //e.g., iowrite32(0x00011043, vdma_controller + 0x30);

  // TODO: get the current time and save it somewhere
  // Should be able to use do_gettimeofday()
  atomic_set(&drvdata->new_frame, 1);
  wake_up_interruptible(&drvdata->wq_frame);
  DEBUG("irq: VDMA frame finished.\n");
  return(IRQ_HANDLED);
}

static int vdma_probe(struct platform_device *pdev)
{
  int irqok, irq, retval;
  struct resource *mem;
  dev_t devt;
  struct device *dev;
  struct xilcam_drvdata *drvdata;
  DECLARE_WAIT_QUEUE_HEAD(wq);

  dev = &pdev->dev;

  // Get character device numbers up to NUM_CAM
  alloc_chrdev_region(&devt, 0, 1, DEVNAME);
  DEBUG("VDMA device registered with major %d, minor: %d\n",
        MAJOR(devt), MINOR(devt));

  /* set up drvdata */
  drvdata = kzalloc(sizeof(struct xilcam_drvdata), GFP_KERNEL);
  if (!drvdata) {
    dev_err(dev, "Couldn't allocate device private record\n");
    retval = -ENOMEM;
    goto failed0;
  }
  dev_set_drvdata(dev, (void *)drvdata);

  // Register the IRQ
  irq = platform_get_irq(pdev, 0);
  TRACE("IRQ number is %d\n", irq);
  if (irq < 0) {
    dev_err(dev, "IRQ lookup failed.  Check the device tree.\n");
    retval = -ENODEV;
    goto failed1;
  }

  /* passing drvdata as a device cookie */
  irqok = request_irq(irq, frame_finished_handler, 0, CLASSNAME,
                     (void *)drvdata);
  if (irqok < 0) {
    dev_err(dev, "Couldn't request IRQ for the device\n");
    retval = -ENODEV;
    goto failed1;
  }

  drvdata->devt = devt;
  drvdata->dev = dev;

  /* set up vdma controller */
  mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!mem) {
    dev_err(dev, "Could't get memory information. Check the device tree.\n");
    retval = -ENODEV;
    goto failed1;
  }

  drvdata->vdma_controller = devm_ioremap_resource(&pdev->dev, mem);

  if(!drvdata->vdma_controller){
    dev_err(dev, "ioremap() failed.\n");
    retval = -ENODEV;
    goto failed1;
  }

  drvdata->wq_frame = wq; /* memory copy */

  // Register the driver with the kernel
  cdev_init(&drvdata->cdev, &fops);
  drvdata->cdev.owner = THIS_MODULE;
  /* add the device to the core */
  retval = cdev_add(&drvdata->cdev, devt, 1);

  if (retval) {
    dev_err(dev, "cdev_add() failed\n");
    goto failed1;
  }

  /* create a device node in /dev/ */
  // TODO: fix the count using atomic counter
  device_create(vdma_class,
                dev,         /* pdev as parrent device */
                devt,
                NULL,         /* no additional data */
                DEVNAME "%d", MINOR(devt));

  DEBUG("VDMA driver initialized\n");
  return(0);

failed1:
  iounmap(drvdata->vdma_controller);

failed0:
  kfree(drvdata);

  return retval;
}

static int vdma_remove(struct platform_device *pdev)
{
  int irq;
  struct xilcam_drvdata *drvdata;

  drvdata = (struct xilcam_drvdata*)dev_get_drvdata(&pdev->dev);

  if (!drvdata)
    return 0;

  // Release the IRQ line
  irq = platform_get_irq(pdev, 0);
  if (irq < 0) {
    ERROR("IRQ lookup failed on release.  Check the device tree.\n");
  } else {
    free_irq(irq, NULL);
  }

  device_unregister(drvdata->dev);
  cdev_del(&drvdata->cdev);
  unregister_chrdev_region(drvdata->devt, 1);

  iounmap(drvdata->vdma_controller);

  return(0);
}

static struct of_device_id vdma_of_match[] = {
  { .compatible = "xilcam", },
  {}
};

static struct platform_driver vdma_driver = {
	.driver = {
		.name = "xilcam",
		.of_match_table = vdma_of_match,
	},
	.probe = vdma_probe,
	.remove = vdma_remove,
};

/* register device class and other init */
static int __init vdma_init(void)
{
  // Set up the device and class structures so we show up in sysfs,
  // and so we have a device we can hand to the DMA request
  vdma_class = class_create(THIS_MODULE, CLASSNAME);
  return platform_driver_register(&vdma_driver);
}

/* clean up the device class */
static void __exit vdma_exit(void)
{
  class_destroy(vdma_class);
  platform_driver_unregister(&vdma_driver);
}

/* because we need to make sure the device class created first, we need to
 * manually set the init and exit code instead of using built-in macro
 */
module_init(vdma_init);
module_exit(vdma_exit);
