/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Dazea Eleni & Zoitaki Charikleia
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
unsigned command = 1;		//0 gia raw, 1 gia cooked 
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));	//put it to kernel log
	if(sensor->msr_data[state->type]->last_update > state->buf_timestamp)
		return 1;			//1 an xreiazetai, 0 an oxi
	else
		return 0;
	
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	struct lunix_sensor_struct temp;
	sensor = state->sensor;
	long data;
	debug("entering update\n");
	

	spin_lock(&sensor->lock);
	temp = *sensor;
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	spin_unlock(&sensor->lock);


	/*
	 * Any new data available?
	 */
	if(lunix_chrdev_state_needs_refresh(state)==1) {
		
	
	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
		
		state->buf_timestamp = temp.msr_data[BATT]->last_update;	
		
		if(command == 1){
			switch(state->type) {					
				case 0:
					data = lookup_voltage[temp.msr_data[0]->values[0]];	
					break;									
				case 1:
					data = lookup_light[temp.msr_data[1]->values[0]];
					break;
				case 2:
					data = lookup_light[temp.msr_data[2]->values[0]];
					break;					
				default:
					return 1; }
			sprintf(state->buf_data, "%ld.%ld \n", data/1000, data%1000);
			
			
			}
		else {
			
			data = temp.msr_data[0]->values[0];  
			sprintf(state->buf_data, "%ld",data);
			} 
		state->buf_lim = strlen(state->buf_data);	
		

			
		}
	

	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	int ret=0;
	unsigned min;
	struct lunix_chrdev_state_struct *neo;
	

	
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	neo = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if(neo == NULL) {
		debug("could not allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}	
	

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	debug("entering\n");
	min = iminor(inode);
	neo->type = min%8;
	neo->sensor = &lunix_sensors[min/8];
	neo->buf_lim = 0;
	neo->buf_data[0]='\0';				
	neo->buf_timestamp=0;
	sema_init(&(neo->lock),1);				

	filp->private_data = neo;
	ret = 0;
	
	/* Allocate a new Lunix character device private state structure */
	/* ? */
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	kfree(filp->private_data);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	if(cmd==0||cmd==1){
		command = cmd;
		return 0;
	}
	else{	
		return -EINVAL;
	}		//upotithetai tha epilegei 16bit i dekadiko me upodiastoli gia update, alla poio noumero cmd tha einai gia poio? =/
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret = 0;

	struct lunix_sensor_struct *sensor;		//f_pos to pou vrisketai ston buffer
	struct lunix_chrdev_state_struct *state;
 
	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);
	debug("entering read\n");

	/* Lock? */
	if(down_interruptible(&state->lock)) {
		return -ERESTARTSYS;
	}

	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_needs_refresh(state) == 0) {	//NA THYMHTHOYME NA FTIAKSOUME THN UPDATE NA EPISTREFEI KATI AN DE DOULEPSEI H COPY TO USER
			up(&state->lock);
			/* The process needs to sleep */
			if(wait_event_interruptible(sensor->wq,(lunix_chrdev_state_needs_refresh(state)!=0)))
				return -ERESTARTSYS;
			/* See LDD3, page 153 for a hint */
			if(down_interruptible(&state->lock)) 
				return -ERESTARTSYS;
			
		}
		lunix_chrdev_state_update(state);
	}
	

	/* End of file */
	
	
	/* Determine the number of cached bytes to copy to userspace */
	if(*f_pos + cnt > state->buf_lim) {
		cnt = state->buf_lim - *f_pos;}
	if(copy_to_user(usrbuf,state->buf_data + *f_pos,cnt)) {
		ret = -EFAULT;
		goto out;
	}

	/* Auto-rewind on EOF mode? */
	*f_pos += cnt;
	if(*f_pos >= state->buf_lim) {
		*f_pos = 0;
	}
	ret = cnt;
out:
	/* Unlock? */
	debug("leaving read with ret=%d \n", ret);
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
        .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	/* register_chrdev_region? */
	ret = register_chrdev_region(dev_no, lunix_minor_cnt,"lunix" );	
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	/* ? */
	/* cdev_add? */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);		
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
