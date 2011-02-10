#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>	/* for copy_from_user */


#include "../../../arch/arm/mach-msm/smd_private.h"
#include "diagfwd.h"

#define MAGIC_WORD_UNLOCK	"unlock_john"
#define MAGIC_WORD_LOCK		"lock_john"


static int read_flag = 0;

static int proc_debug_smem_read(struct file *file, char *buf, size_t size, loff_t *ppos);
static int proc_debug_smem_write(struct file *file, const char *buf, size_t size, loff_t *ppos);

struct file_operations proc_debug_smem_fops = {
	read:proc_debug_smem_read,
	write:proc_debug_smem_write,
};

static int proc_debug_smem_read(struct file *file, char *buf, size_t size, loff_t *ppos)
{
	int len = 0;
	int data = 0;
	acer_smem_flag_t *acer_smem_flag = NULL;
	char *ptmp = NULL;

	ptmp = kmalloc(sizeof(acer_smem_flag->acer_amss_boot_mode),GFP_KERNEL);
	memset(ptmp, 0x00, sizeof(acer_smem_flag->acer_amss_boot_mode));

	len = size;

	if(len + *ppos > sizeof(acer_smem_flag->acer_amss_boot_mode) ) //over the limit
		len = sizeof(acer_smem_flag->acer_amss_boot_mode) - *ppos;
	if(len == 0){
		return 0;
	}

	if(!buf){
		return -2;
	}

	if( read_flag++ <1 ){
		acer_smem_flag = smem_alloc(SMEM_ID_VENDOR0, sizeof(acer_smem_flag_t));
		data = acer_smem_flag->acer_amss_boot_mode;
	}else{
		data = 0;
	}
	sprintf(ptmp, "%d", data);

	memcpy(buf, ptmp, len);

	*ppos += len;

	return len;
}


static int proc_debug_smem_write(struct file *file, const char *buf, size_t size, loff_t *ppos)
{
	char buff[64];
	int ret = -EFAULT;

	if(size > 64 || ( size != sizeof(MAGIC_WORD_LOCK) && size!= sizeof(MAGIC_WORD_UNLOCK) )){
		pr_debug("Invalid Parameter!\n");
		return ret;
	}

	memset(buff, 0x00, sizeof(buff));
	/* write data to the buffer */
	if ( copy_from_user(buff, buf, size) ) {
		pr_err("COPY_FROM_USER ERR!\n");
		return ret;
	}

	if(strncmp(buff, MAGIC_WORD_LOCK, sizeof(MAGIC_WORD_LOCK)-1) == 0){
		//Lock
		debug_mode_enable = 0;
		ret = sizeof(MAGIC_WORD_LOCK);
		pr_info("diag lock!!!\n");
	}else if(strncmp(buff, MAGIC_WORD_UNLOCK, sizeof(MAGIC_WORD_UNLOCK)-1) == 0){
		debug_mode_enable = 1;
		ret = sizeof(MAGIC_WORD_UNLOCK);
		pr_info("diag unlock!!!\n");
	}

	return ret;
}



static int __init init_debug_status(void)
{

	struct proc_dir_entry   *p;
	p = create_proc_entry ("debug_smem", S_IFREG | S_IRUGO | S_IWUGO, NULL);
	if(p)
		p->proc_fops = &proc_debug_smem_fops;
	return 0;
}

static void __exit cleanup_debug_status(void)
{
	pr_debug("cleanup testdebug\n");
	remove_proc_entry ("debug_smem", NULL);
}

module_init(init_debug_status);
module_exit(cleanup_debug_status);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jungchu Hsu (jungchu_hsu@acer.com.tw)");
MODULE_DESCRIPTION("For reading the debugging sizes");
