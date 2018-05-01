#include <linux/module.h>
#include <linux/cdev.h> // Character device
#include <linux/device.h>
#include <asm/uaccess.h> // Copy to/from userspace pointers
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/slab.h>

MODULE_LICENSE("GPL");

#include "common.h"
#include "buffer.h"
#include "ioctl_cmds.h"

#define CLASSNAME "cmabuffer" // Shows up in /sys/class
#define DEVNAME "cmabuffer" // Shows up in /dev
#define NUM_DEV 8 // in total 8 cma buffers

/* meta data for the driver */
struct cmabuf_drvdata {
  struct device *dev;
  struct cdev cdev; /* Char device structure */
  dev_t devt;
};

dev_t device_num;
struct cmabuf_drvdata *cmabuf_drvdata[NUM_DEV];
struct class *cmabuf_class;

// 0 is errors only, increasing numbers print more stuff
const int debug_level = 4;

// TODO: need to make sure the device can only be accessed one
// at a time
static int dev_open(struct inode *inode, struct file *file)
{
  struct cmabuf_drvdata *drvdata;
  TRACE("cmabuffer: dev_open\n");

  drvdata = container_of(inode->i_cdev, struct cmabuf_drvdata, cdev);

  /* set private data */
  file->private_data = drvdata;

  // Set up the image buffers; fail if it fails.
  if(init_buffers(drvdata->dev) < 0){
    return(-ENOMEM);
  }

  return(0);
}

static int dev_close(struct inode *inode, struct file *file)
{
  struct cmabuf_drvdata *drvdata;
  TRACE("cmabuffer: dev_close\n");

  drvdata = file->private_data;
  cleanup_buffers(drvdata->dev); // Release all the buffer memory

  return(0);
}

void free_image(Buffer* buf)
{
  release_buffer(buf);
  DEBUG("Releasing image\n");
}


long dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  Buffer tmp, *tmpptr;
  DEBUG("ioctl cmd %d | %lu (%lx) \n", cmd, arg, arg);
  switch(cmd){
    case GET_BUFFER:
      TRACE("ioctl: GET_BUFFER\n");
      // Get the desired buffer parameters from the object passed to us
      if(access_ok(VERIFY_READ, (void*)arg, sizeof(Buffer)) &&
         copy_from_user(&tmp, (void*)arg, sizeof(Buffer)) == 0){
        tmpptr = acquire_buffer(tmp.width, tmp.height, tmp.depth, tmp.stride);
        if(tmpptr == NULL){
          return(-ENOBUFS);
        }
      }
      else{
        return(-EIO);
      }

      // Now copy the retrieved buffer back to the user
      if(access_ok(VERIFY_WRITE, (void*)arg, sizeof(Buffer)) &&
         (copy_to_user((void*)arg, tmpptr, sizeof(Buffer)) == 0)) { } // All ok, nothing to do
      else{
        return(-EIO);
      }
    break;

    case FREE_IMAGE:
      TRACE("ioctl: FREE_IMAGE\n");
      // Copy the object into our tmp copy
      if(access_ok(VERIFY_READ, (void*)arg, sizeof(Buffer)) &&
         copy_from_user(&tmp, (void*)arg, sizeof(Buffer)) == 0){
          free_image(&tmp);
      }
      else{
        return(-EACCES);
      }
      break;

    default:
      return(-EINVAL); // Unknown command, return an error
      break;
  }
  return(0); // Success
}

int vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
  struct page* pageptr;

  // Calculate with physical address, ala udmabuf/LDD3
  uint64_t offset             = vmf->pgoff << PAGE_SHIFT;
  uint64_t phys_addr          = get_phys_addr() + offset;
  unsigned long pageframe     = phys_addr >> PAGE_SHIFT;

  DEBUG("vma_fault() offset: %llx  phys_addr: %llx pageframe: %lx\n",
          offset, phys_addr, pageframe);

  if(!pfn_valid(pageframe)) {
    return(-1);
  }
  pageptr = pfn_to_page(pageframe);
  get_page(pageptr);
  vmf->page = pageptr;

  return(0);
}

static struct vm_operations_struct vma_operations = {
  .fault = vma_fault,
};

int dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
  TRACE("dev_mmap\n");
  // Just set up the operations; fault operation does all the hard work
  vma->vm_ops = &vma_operations;
  return 0;
}

struct file_operations fops = {
  // No read/write; everything is handled by ioctl and mmap
  .open = dev_open,
  .release = dev_close,
  .unlocked_ioctl = dev_ioctl,
  .mmap = dev_mmap,
};

static int cmabuf_driver_init(void)
{
  u64 i; /* not portable to 32-bit system */
  dev_t curr_dev;
  struct cmabuf_drvdata *drvdata;
  struct device *dev;

  // Get a single character device number
  alloc_chrdev_region(&device_num, 0, NUM_DEV, DEVNAME);
  DEBUG("Device registered with major %d, minor: %d\n", MAJOR(device_num),
        MINOR(device_num));

  // Set up the device and class structures so we show up in sysfs,
  // and so we have a device we can hand to the DMA request
  cmabuf_class = class_create(THIS_MODULE, CLASSNAME);

  // If we had multiple devices, we could break it apart with
  // MAJOR(device_num), and then add in our own minor number, with
  // MKDEV(MAJOR(device_num), minor_num)
  for (i = 0; i < NUM_DEV; i++) {
    curr_dev = MKDEV(MAJOR(device_num), MINOR(device_num) + i);

    /* allocate and set the drv data */
    drvdata = kzalloc(sizeof(struct cmabuf_drvdata), GFP_KERNEL);
    if (!drvdata) {
      ERROR("Couldn't allocate device private data\n");
      return -ENOMEM;
    }
    drvdata->devt = curr_dev;

    // Register the driver with the kernel
    cdev_init(&drvdata->cdev, &fops);
    drvdata->cdev.owner = THIS_MODULE;
    cdev_add(&drvdata->cdev, curr_dev, 1);

    dev = device_create(cmabuf_class,
                        NULL,         /* no parent device */
                        curr_dev,
                        NULL,         /* pass nothing */
                        DEVNAME "%lld", i);
    dev_set_drvdata(dev, (void *)drvdata);

    /* store the drv data for cleaning up */
    cmabuf_drvdata[i] = drvdata;
  }
  DEBUG("Driver initialized\n");

  return(0);
}

static void cmabuf_driver_exit(void)
{
  int i;
  struct cmabuf_drvdata *drvdata;

  for (i = 0; i < NUM_DEV; i++) {
    drvdata = cmabuf_drvdata[i];
    device_unregister(drvdata->dev);
    cdev_del(&drvdata->cdev);

    /* free kernel memory */
    kfree(drvdata);
  }
  unregister_chrdev_region(device_num, NUM_DEV);
  class_destroy(cmabuf_class);
}

module_init(cmabuf_driver_init);
module_exit(cmabuf_driver_exit);

