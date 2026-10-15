/*
 * ttygsm - virtual serial ports for a userspace modem daemon
 *
 * Registers /dev/ttyGSM<n> as real tty devices (so they get an entry in
 * /sys/class/tty and uevents, which tools like ModemManager and udev need)
 * and a matching "master" misc device /dev/ttygsm-master<n>. What is written
 * to the tty can be read from the master and what is written to the master
 * shows up as input on the tty, like the two ends of a pty, except that the
 * tty side is a proper registered tty with a raw default line setup.
 *
 * Used by samsungipcd, which emulates a modem's AT/PPP interface on top of
 * Samsung's IPC protocol. Only the tty/master plumbing lives here.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/serial.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define TTYGSM_PORTS	2
#define TTYGSM_FIFO	8192
#define TTYGSM_CHUNK	512

struct ttygsm {
	struct tty_port port;
	struct miscdevice master;
	char master_name[24];
	struct kfifo to_master;		/* tty write() -> master read() */
	spinlock_t lock;		/* serialises users of to_master */
	wait_queue_head_t read_wait;
	atomic_t master_open;
};

static struct tty_driver *ttygsm_driver;
static struct ttygsm ttygsm_ports[TTYGSM_PORTS];

static struct ttygsm *tty_to_gsm(struct tty_struct *tty)
{
	return tty->driver_data;
}

/* tty side */

static int ttygsm_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct ttygsm *gsm = &ttygsm_ports[tty->index];

	tty->driver_data = gsm;
	return tty_port_install(&gsm->port, driver, tty);
}

static int ttygsm_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(&tty_to_gsm(tty)->port, tty, filp);
}

static void ttygsm_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(&tty_to_gsm(tty)->port, tty, filp);
}

static void ttygsm_hangup(struct tty_struct *tty)
{
	tty_port_hangup(&tty_to_gsm(tty)->port);
}

static int ttygsm_write(struct tty_struct *tty, const unsigned char *buf,
			int count)
{
	struct ttygsm *gsm = tty_to_gsm(tty);
	unsigned long flags;
	int done;

	spin_lock_irqsave(&gsm->lock, flags);
	done = kfifo_in(&gsm->to_master, buf, count);
	spin_unlock_irqrestore(&gsm->lock, flags);

	if (done)
		wake_up_interruptible(&gsm->read_wait);
	return done;
}

static int ttygsm_write_room(struct tty_struct *tty)
{
	struct ttygsm *gsm = tty_to_gsm(tty);
	unsigned long flags;
	int room;

	spin_lock_irqsave(&gsm->lock, flags);
	room = kfifo_avail(&gsm->to_master);
	spin_unlock_irqrestore(&gsm->lock, flags);
	return room;
}

static int ttygsm_chars_in_buffer(struct tty_struct *tty)
{
	/*
	 * There is no transmitter to wait for: data counts as sent once it is
	 * queued for the daemon. Reporting the queue here would make tcdrain()
	 * and the final close() block for as long as the daemon is not
	 * reading, which hangs clients such as ModemManager and pppd.
	 */
	return 0;
}

static void ttygsm_flush_buffer(struct tty_struct *tty)
{
	struct ttygsm *gsm = tty_to_gsm(tty);
	unsigned long flags;

	spin_lock_irqsave(&gsm->lock, flags);
	kfifo_reset(&gsm->to_master);
	spin_unlock_irqrestore(&gsm->lock, flags);
	tty_wakeup(tty);
}

static void ttygsm_set_termios(struct tty_struct *tty, struct ktermios *old)
{
	/* nothing to program, there is no real line */
}

static int ttygsm_tiocmget(struct tty_struct *tty)
{
	/* always connected and ready, so clients do not wait for a carrier */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS | TIOCM_DTR | TIOCM_RTS;
}

static int ttygsm_tiocmset(struct tty_struct *tty, unsigned int set,
			   unsigned int clear)
{
	return 0;
}

static const struct tty_operations ttygsm_ops = {
	.install	= ttygsm_install,
	.open		= ttygsm_open,
	.close		= ttygsm_close,
	.hangup		= ttygsm_hangup,
	.write		= ttygsm_write,
	.write_room	= ttygsm_write_room,
	.chars_in_buffer = ttygsm_chars_in_buffer,
	.flush_buffer	= ttygsm_flush_buffer,
	.set_termios	= ttygsm_set_termios,
	.tiocmget	= ttygsm_tiocmget,
	.tiocmset	= ttygsm_tiocmset,
};

static int ttygsm_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void ttygsm_dtr_rts(struct tty_port *port, int raise)
{
}

static void ttygsm_port_shutdown(struct tty_port *port)
{
	struct ttygsm *gsm = container_of(port, struct ttygsm, port);
	unsigned long flags;

	/* the last close drops whatever the daemon has not picked up yet */
	spin_lock_irqsave(&gsm->lock, flags);
	kfifo_reset(&gsm->to_master);
	spin_unlock_irqrestore(&gsm->lock, flags);
}

static const struct tty_port_operations ttygsm_port_ops = {
	.carrier_raised	= ttygsm_carrier_raised,
	.dtr_rts	= ttygsm_dtr_rts,
	.shutdown	= ttygsm_port_shutdown,
};

/* master side */

static int ttygsm_master_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct ttygsm *gsm = container_of(misc, struct ttygsm, master);

	if (atomic_cmpxchg(&gsm->master_open, 0, 1) != 0)
		return -EBUSY;
	filp->private_data = gsm;
	return nonseekable_open(inode, filp);
}

static int ttygsm_master_release(struct inode *inode, struct file *filp)
{
	struct ttygsm *gsm = filp->private_data;

	atomic_set(&gsm->master_open, 0);
	return 0;
}

static ssize_t ttygsm_master_read(struct file *filp, char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	struct ttygsm *gsm = filp->private_data;
	struct tty_struct *tty;
	unsigned char tmp[TTYGSM_CHUNK];
	unsigned long flags;
	unsigned int n;
	int ret;

	if (!count)
		return 0;
	if (count > sizeof(tmp))
		count = sizeof(tmp);

	for (;;) {
		spin_lock_irqsave(&gsm->lock, flags);
		n = kfifo_out(&gsm->to_master, tmp, count);
		spin_unlock_irqrestore(&gsm->lock, flags);
		if (n)
			break;
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(gsm->read_wait,
					       !kfifo_is_empty(&gsm->to_master));
		if (ret)
			return ret;
	}

	if (copy_to_user(ubuf, tmp, n))
		return -EFAULT;

	/* there is room again for writers on the tty side */
	tty = tty_port_tty_get(&gsm->port);
	if (tty) {
		tty_wakeup(tty);
		tty_kref_put(tty);
	}
	return n;
}

static ssize_t ttygsm_master_write(struct file *filp, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct ttygsm *gsm = filp->private_data;
	unsigned char tmp[TTYGSM_CHUNK];
	struct tty_struct *tty;
	size_t done = 0;

	while (done < count) {
		size_t chunk = min_t(size_t, count - done, sizeof(tmp));
		int pushed;

		if (copy_from_user(tmp, ubuf + done, chunk))
			return done ? (ssize_t)done : -EFAULT;

		/* nobody has the tty open: input to a closed line is dropped */
		tty = tty_port_tty_get(&gsm->port);
		if (!tty)
			return count;
		tty_kref_put(tty);

		pushed = tty_insert_flip_string(&gsm->port, tmp, chunk);
		if (pushed > 0)
			tty_flip_buffer_push(&gsm->port);
		if (pushed < (int)chunk) {
			/* receive buffer full, let the caller retry the rest */
			done += pushed > 0 ? pushed : 0;
			return done ? (ssize_t)done : -EAGAIN;
		}
		done += chunk;
	}
	return done;
}

static unsigned int ttygsm_master_poll(struct file *filp, poll_table *wait)
{
	struct ttygsm *gsm = filp->private_data;
	unsigned int mask = POLLOUT | POLLWRNORM;

	poll_wait(filp, &gsm->read_wait, wait);
	if (!kfifo_is_empty(&gsm->to_master))
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static const struct file_operations ttygsm_master_fops = {
	.owner		= THIS_MODULE,
	.open		= ttygsm_master_open,
	.release	= ttygsm_master_release,
	.read		= ttygsm_master_read,
	.write		= ttygsm_master_write,
	.poll		= ttygsm_master_poll,
	.llseek		= no_llseek,
};

static int __init ttygsm_init(void)
{
	struct device *dev;
	int i, ret;

	ttygsm_driver = tty_alloc_driver(TTYGSM_PORTS,
					 TTY_DRIVER_REAL_RAW |
					 TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(ttygsm_driver))
		return PTR_ERR(ttygsm_driver);

	ttygsm_driver->driver_name = "ttygsm";
	ttygsm_driver->name = "ttyGSM";
	ttygsm_driver->type = TTY_DRIVER_TYPE_SERIAL;
	ttygsm_driver->subtype = SERIAL_TYPE_NORMAL;
	ttygsm_driver->init_termios = tty_std_termios;
	/* raw 8N1, no echo: this is a modem line, not a terminal */
	ttygsm_driver->init_termios.c_iflag = 0;
	ttygsm_driver->init_termios.c_oflag = 0;
	ttygsm_driver->init_termios.c_lflag = 0;
	ttygsm_driver->init_termios.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	tty_set_operations(ttygsm_driver, &ttygsm_ops);

	ret = tty_register_driver(ttygsm_driver);
	if (ret)
		goto err_put;

	for (i = 0; i < TTYGSM_PORTS; i++) {
		struct ttygsm *gsm = &ttygsm_ports[i];

		spin_lock_init(&gsm->lock);
		init_waitqueue_head(&gsm->read_wait);
		atomic_set(&gsm->master_open, 0);
		ret = kfifo_alloc(&gsm->to_master, TTYGSM_FIFO, GFP_KERNEL);
		if (ret)
			goto err_ports;
		tty_port_init(&gsm->port);
		gsm->port.ops = &ttygsm_port_ops;
		gsm->port.close_delay = 0;
		gsm->port.closing_wait = ASYNC_CLOSING_WAIT_NONE;

		snprintf(gsm->master_name, sizeof(gsm->master_name),
			 "ttygsm-master%d", i);
		gsm->master.minor = MISC_DYNAMIC_MINOR;
		gsm->master.name = gsm->master_name;
		gsm->master.fops = &ttygsm_master_fops;
		ret = misc_register(&gsm->master);
		if (ret) {
			kfifo_free(&gsm->to_master);
			goto err_ports;
		}

		dev = tty_port_register_device(&gsm->port, ttygsm_driver, i,
					       NULL);
		if (IS_ERR(dev)) {
			ret = PTR_ERR(dev);
			misc_deregister(&gsm->master);
			kfifo_free(&gsm->to_master);
			goto err_ports;
		}
	}

	pr_info("ttygsm: %d virtual modem ports registered\n", TTYGSM_PORTS);
	return 0;

err_ports:
	while (--i >= 0) {
		tty_unregister_device(ttygsm_driver, i);
		misc_deregister(&ttygsm_ports[i].master);
		kfifo_free(&ttygsm_ports[i].to_master);
	}
	tty_unregister_driver(ttygsm_driver);
err_put:
	put_tty_driver(ttygsm_driver);
	return ret;
}

static void __exit ttygsm_exit(void)
{
	int i;

	for (i = 0; i < TTYGSM_PORTS; i++) {
		tty_unregister_device(ttygsm_driver, i);
		misc_deregister(&ttygsm_ports[i].master);
		kfifo_free(&ttygsm_ports[i].to_master);
	}
	tty_unregister_driver(ttygsm_driver);
	put_tty_driver(ttygsm_driver);
}

module_init(ttygsm_init);
module_exit(ttygsm_exit);

MODULE_DESCRIPTION("Virtual modem tty ports for a userspace modem daemon");
MODULE_LICENSE("GPL");
