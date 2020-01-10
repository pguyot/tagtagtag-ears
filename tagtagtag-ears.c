// Signal of encoder GPIO when the ear is turning (motors are on) looks like
// this:
//
// 0    1    2    3    4    5    6    7    8    9   10   11   12   13   14
//   __   __   __   __   __   __   __   __   __   __   __   __   __   __   __
// _|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |
//
// 15  16   17                   0    1    2    3    4    5    6    7    8
//   __   __   _________________   __   __   __   __   __   __   __   __   __
// _|  |_|  |_|                 |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |_|  |
//
// Quick measurements on "normally workings" Tag & TagTag show that:
// signal is high for 0.12-0.15 sec (average 0.13) with the exception of the gap
// where it is high for 0.75 sec (as wide as 3 holes)
// signal is low for 0.06-0.09 sec (average 0.07)
// A complete turn takes 4 seconds.


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

// Definitions

#define DRV_NAME "tagtagtag-ears"
#define DEVICE_NAME "ear"
#define NUM_HOLES 17
#define BROKEN_TIMEOUT_SECS 4

// Data structures

enum ear_state_e {
    testing,
    detecting,
    idle,
    running,
    broken,
};

enum detecting_post_state_e {
    goto_position,
    read_position,
};

struct ear_state_testing {
    int holes_count;
    ktime_t last_hole_time;
    unsigned long hole_deltas[NUM_HOLES];
    int forward_position;
};

struct ear_state_detecting {
    enum detecting_post_state_e post_state;
    int direction;  // 1: forward, -1: backward
    int new_position;
    int holes_count;   // counter of found holes_count
    ktime_t last_hole_time;
};

struct ear_state_idle {
    int position;   // 0-16 or -1 for unknown.
};

struct ear_state_running {
    int position;   // 0-16 or -1 for unknown.
    int count;
    int direction;  // 1: forward, -1: backward
};

union ear_state {
    struct ear_state_testing testing;
    struct ear_state_detecting detecting;
    struct ear_state_idle idle;
    struct ear_state_running running;
};

struct tagtagtagear_data {
    struct cdev cdev;
    struct device *device;
    struct gpio_desc *encoder_gpio;
    struct gpio_descs *motor_gpios;
	struct timer_list broken_timer;
    unsigned long detect_boundary_us;
	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
    int read_result_available;
    char read_result;
	char buffer[1];
	int buffer_size;
	int opened;
    enum ear_state_e state_e;
    union ear_state state;
};

struct tagtagtagears_data {
    dev_t chrdev;
    struct class *ears_class;
    struct tagtagtagear_data ear[2];
};

// Prototypes
static void start_motors_backward(struct tagtagtagear_data *priv);
static void start_motors_forward(struct tagtagtagear_data *priv);
static void stop_motors(struct tagtagtagear_data *priv);

static void tagtagtagear_broken_timer_cb(struct timer_list *t);
static void reset_broken_timer(struct tagtagtagear_data *priv);

static void transition_to_testing(struct tagtagtagear_data *priv);
static void transition_to_broken(struct tagtagtagear_data *priv);
static void transition_to_idle(struct tagtagtagear_data *priv, int position);
static void transition_to_running(struct tagtagtagear_data *priv, int position, int delta);
static void transition_to_detecting(struct tagtagtagear_data *priv, enum detecting_post_state_e post_state, int direction, int new_position);

static void irq_handler_testing(struct tagtagtagear_data *priv);
static void irq_handler_idle(struct tagtagtagear_data *priv);
static void irq_handler_running(struct tagtagtagear_data *priv);
static void irq_handler_detecting(struct tagtagtagear_data *priv);
static irqreturn_t tagtagtagear_irq_handler(int irq, void *dev_id);

static int ear_open(struct inode *inode, struct file *file);
static int ear_release(struct inode *inode, struct file *file);
static int ear_read(struct file *file, char __user *buffer, size_t len, loff_t *offset);
static int ear_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset);

static int init_ear(struct device *dev, struct tagtagtagear_data *priv, struct class *ears_class, int major, int minor, const char* encoder_name, const char* motor_name);
static int tagtagtagears_probe(struct platform_device *pdev);
static int tagtagtagears_remove(struct platform_device *pdev);


// ========================================================================== //
// Motors
// ========================================================================== //

static void start_motors_backward(struct tagtagtagear_data *priv) {
    gpiod_set_value(priv->motor_gpios->desc[0], 0);
    gpiod_set_value(priv->motor_gpios->desc[1], 1);
}

static void start_motors_forward(struct tagtagtagear_data *priv) {
    gpiod_set_value(priv->motor_gpios->desc[0], 1);
    gpiod_set_value(priv->motor_gpios->desc[1], 0);
}

static void stop_motors(struct tagtagtagear_data *priv) {
    gpiod_set_value(priv->motor_gpios->desc[0], 0);
    gpiod_set_value(priv->motor_gpios->desc[1], 0);
}

// ========================================================================== //
// Broken timer
// ========================================================================== //

//
// Callback when timer is fired.
// In testing mode, declare ear as broken.
// In any other mode, transition to idle with unknown position.
// Always stop motors.
//
static void tagtagtagear_broken_timer_cb(struct timer_list *t) {
    struct tagtagtagear_data *priv = from_timer(priv, t, broken_timer);
    stop_motors(priv);
    if (priv->state_e == testing) {
        dev_err(priv->device, "timeout, declaring ear as broken");
        transition_to_broken(priv);
    } else {
        dev_err(priv->device, "timeout, giving up (position is thereupon unknown)");
        transition_to_idle(priv, -1);
    }
}

static void reset_broken_timer(struct tagtagtagear_data *priv) {
    del_timer_sync(&priv->broken_timer);
    mod_timer(&priv->broken_timer, jiffies + BROKEN_TIMEOUT_SECS * HZ);
}

// ========================================================================== //
// State transitions
// ========================================================================== //

static void transition_to_testing(struct tagtagtagear_data *priv) {
    priv->state_e = testing;
    memset(&priv->state, 0, sizeof(priv->state));
    reset_broken_timer(priv);
    start_motors_forward(priv);
}

static void transition_to_broken(struct tagtagtagear_data *priv) {
    priv->state_e = broken;
    memset(&priv->state, 0, sizeof(priv->state));
    wake_up_interruptible(&priv->write_wq);
}

static void transition_to_idle(struct tagtagtagear_data *priv, int position) {
    priv->state_e = idle;
    memset(&priv->state, 0, sizeof(priv->state));
    priv->state.idle.position = position;
    wake_up_interruptible(&priv->write_wq);
}

static void transition_to_running(struct tagtagtagear_data *priv, int position, int delta) {
    priv->state_e = running;
    memset(&priv->state, 0, sizeof(priv->state));
    priv->state.running.position = position;
    reset_broken_timer(priv);
    if (delta > 0) {
        priv->state.running.count = delta;
        priv->state.running.direction = 1;
        start_motors_forward(priv);
    } else if (delta < 0) {
        priv->state.running.count = -delta;
        priv->state.running.direction = -1;
        start_motors_backward(priv);
    } else {
        del_timer_sync(&priv->broken_timer);
        stop_motors(priv);  // We need to stop motors if we transitioned from detecting.
        if (priv->read_result_available == 1) {
            priv->read_result = position;
        }
        transition_to_idle(priv, position);
    }
}

static void transition_to_detecting(struct tagtagtagear_data *priv, enum detecting_post_state_e post_state, int direction, int new_position) {
    priv->state_e = detecting;
    memset(&priv->state, 0, sizeof(priv->state));
    priv->state.detecting.post_state = post_state;
    priv->state.detecting.direction = direction;
    priv->state.detecting.new_position = new_position;
    if (gpiod_get_value(priv->encoder_gpio) == 0) {
        priv->state.detecting.last_hole_time = ktime_get_raw();
    } else {
        priv->state.detecting.last_hole_time = 0;
    }
    reset_broken_timer(priv);
    if (direction > 0) {
        start_motors_forward(priv);
    } else {
        start_motors_backward(priv);
    }
}

// ========================================================================== //
// IRQ Handler
// ========================================================================== //

//
// IRQ Handler in testing state
//
// Count the number of holes, stop at NUM_HOLES.
// For every hole, compute the delta time with the previous FALLING IRQ.
// Eventually, use the average between the maximum "normal" delta and the "gap"
// delta as a boundary for future detection.
//
// If "gap" delta < maximum "normal" delta * 1.5, declare the ear as broken.
//
// At every hole, reset broken ear timer.
//
static void irq_handler_testing(struct tagtagtagear_data *priv) {
    if (priv->state.testing.last_hole_time == 0) {
        priv->state.testing.last_hole_time = ktime_get_raw();
        reset_broken_timer(priv);
    } else {
        ktime_t now = ktime_get_raw();
        if (priv->state.testing.holes_count < NUM_HOLES) {
            priv->state.testing.hole_deltas[priv->state.testing.holes_count] = ktime_us_delta(now, priv->state.testing.last_hole_time);
            priv->state.testing.last_hole_time = now;
            priv->state.testing.holes_count++;

            if (priv->state.testing.holes_count == NUM_HOLES) {
                unsigned long min, max, gap, half_max, first_delta, second_delta;
                int gap_ix = 0;
                int ix;

                // End of forward testing. Stop motors.
                del_timer_sync(&priv->broken_timer);
                stop_motors(priv);
                // We should have 16 approximatively equivalent deltas and one at least twice larger.
                first_delta = priv->state.testing.hole_deltas[0];
                second_delta = priv->state.testing.hole_deltas[1];
                min = min(first_delta, second_delta);
                max = min;
                gap = max(first_delta, second_delta);
                for (ix = 2; ix < NUM_HOLES; ix++) {
                    unsigned long this_delta = priv->state.testing.hole_deltas[ix];
                    if (min > this_delta) {
                        min = this_delta;
                    } else if (gap < this_delta) {
                        max = gap;
                        gap = this_delta;
                        gap_ix = ix;
                    } else if (max < this_delta) {
                        max = this_delta;
                    }
                }
                half_max = max >> 1;
                if (gap < (max + half_max)) {
                    dev_err(priv->device, "gap is not obvious (max = %lu, gap = %lu), declaring ear as broken", max, gap);
                    transition_to_broken(priv);
                } else {
                    // if gap_ix was the first delta (0), we ran a full turn and position is 16
                    // if gap_ix was the last delta (16), we are at 0.
                    priv->state.testing.forward_position = NUM_HOLES - 1 - gap_ix;
                    priv->detect_boundary_us = (max + gap) >> 1;
                    if (priv->detect_boundary_us > 1000000) {
                        dev_warn(priv->device, "Ear is abnormally slow (gap = %lu usec, typically 800ms)", gap);
                    }
                    start_motors_backward(priv);
                    reset_broken_timer(priv);
                }
            } else {
                reset_broken_timer(priv);
            }
        } else {
            unsigned long backward_delta = ktime_us_delta(now, priv->state.testing.last_hole_time);
            int broken = 0;
            int position;
            // We were running backward one position to test backward motor.
            // End of backward testing. Stop motors.
            del_timer_sync(&priv->broken_timer);
            stop_motors(priv);
            if (priv->state.testing.forward_position == 0) {
                if (backward_delta < priv->detect_boundary_us) {
                    dev_err(priv->device, "Incoherent backward delta, got %lu, expected more than %lu", backward_delta, priv->detect_boundary_us);
                    broken = 1;
                }
            } else {
                if (backward_delta > priv->detect_boundary_us) {
                    dev_err(priv->device, "Incoherent backward delta, got %lu, expected less than %lu", backward_delta, priv->detect_boundary_us);
                    broken = 1;
                }
            }
            position = priv->state.testing.forward_position - 1;
            if (position < 0) {
                position += NUM_HOLES;
            }
            if (broken == 0) {
                if (priv->read_result_available == 1) {
                    priv->read_result = position;
                }
                transition_to_idle(priv, position);
            } else {
                transition_to_broken(priv);
            }
        }
    }
}

//
// IRQ Handler in idle state
//
// User moved the ear. Position is now unknown.
// Signal blocked reader if read_result_available is clear.
//
static void irq_handler_idle(struct tagtagtagear_data *priv) {
    priv->state.idle.position = -1;
    if (priv->read_result_available == 0) {
        priv->read_result_available = 1;
        priv->read_result = 'm';
        wake_up_interruptible(&priv->read_wq);
    }
}

//
// IRQ Handler in running state
//
// Decrement counter and stop motors if it reached zero.
// Update position if it is known.
//
static void irq_handler_running(struct tagtagtagear_data *priv) {
    if (priv->state.running.position != -1) {
        priv->state.running.position = (priv->state.running.position + priv->state.running.direction) % NUM_HOLES;
        if (priv->state.running.position < 0) {
            priv->state.running.position += NUM_HOLES;
        }
    }
    priv->state.running.count--;
    if (priv->state.running.count == 0) {
        del_timer_sync(&priv->broken_timer);
        stop_motors(priv);
        transition_to_idle(priv, priv->state.running.position);
    } else {
        reset_broken_timer(priv);
    }
}

//
// IRQ Handler in detecting state
//
// If elapsed time is greater than detect_boundary_us, we found the gap.
//
static void irq_handler_detecting(struct tagtagtagear_data *priv) {
    ktime_t now = ktime_get_raw();
    if (priv->state.detecting.last_hole_time == 0) {
        // We were between two holes.
        // Synchronize on the next hole in forward direction:
        // if we turn backward, consider the ear was on previous hole
        if (priv->state.detecting.direction < 0) {
            priv->state.detecting.holes_count++;
        }
        priv->state.detecting.last_hole_time = ktime_get_raw();
        reset_broken_timer(priv);
    } else {
        unsigned long delta = (unsigned long) ktime_us_delta(now, priv->state.detecting.last_hole_time);
        priv->state.detecting.holes_count++;
        if (delta > priv->detect_boundary_us) {
            // Found gap.
            int running_delta;
            if (priv->state.detecting.post_state == read_position) {
                // We moved priv->state.detecting.position steps before reaching 0.
                // Previous position was x + priv->state.detecting.position = NUM_HOLES
                // x = NUM_HOLES - priv->state.detecting.position
                running_delta = NUM_HOLES - priv->state.detecting.holes_count;
                priv->read_result_available = 1;
                priv->read_result = running_delta;
                wake_up_interruptible(&priv->read_wq);
            } else {
                running_delta = priv->state.detecting.new_position;
                if (priv->state.detecting.direction < 0) {
                    running_delta = NUM_HOLES - running_delta;
                }
            }
            // Minimize movement.
            while (running_delta > 9) {
                running_delta -= NUM_HOLES;
            }
            while (running_delta < -9) {
                running_delta += NUM_HOLES;
            }
            transition_to_running(priv, 0, running_delta);
        } else {
            priv->state.detecting.last_hole_time = now;
            reset_broken_timer(priv);
        }
    }
}

static irqreturn_t tagtagtagear_irq_handler(int irq, void *dev_id) {
    struct tagtagtagear_data *priv = dev_id;
    switch (priv->state_e) {
        case testing:
            irq_handler_testing(priv);
            break;

        case idle:
            irq_handler_idle(priv);
            break;

        case running:
            irq_handler_running(priv);
            break;

        case detecting:
            irq_handler_detecting(priv);
            break;

        default:
            // Do nothing.
            break;
    }
    return IRQ_HANDLED;
}

// ========================================================================== //
// File operations & commands
// ========================================================================== //

// Protocol:
// 1. Can only be opened once
// 2. Writing is blocking/non-blocking depending on the state:
// - in idle mode, writing is non blocking, command is executed and ear might
//   transition to running/detecting
// - in testing/running/detecting mode, writing is blocked until ear is in
//   idle/broken mode.
// - in broken mode, writing fails.
// 3. Reading is blocking until a value is to be read.

// NOP command
// Command = '.'
// Blocks until ear is in idle mode.
// $ echo -n -e '.' > /dev/ear0

// Turn forward command
// Command = '+'
// Parameter = N (single byte)
// Turn forward N steps. Transition to running mode then idle or broken.
// $ echo -n -e '+\x01' > /dev/ear0

// Turn backward command
// Command = '-'
// Parameter = N (single byte)
// Turn backward N steps. Transition to running mode then idle or broken.
// $ echo -n -e '-\x01' > /dev/ear0

// Move to specific position, forward.
// Command = '>'
// Parameter = P (single byte)
// May not turn at all.
// Turn forward until reaching position P mod NUM_HOLES
// If P >= NUM_HOLES, perform P div NUM_HOLES complete turns.
// If position is unknown, perform first a position detection, forward.
// $ echo -n -e '>\x00' > /dev/ear0

// Move to specific position, backward.
// Command = '<'
// Parameter = P (single byte)
// May not turn at all.
// Turn backward until reaching position P mod NUM_HOLES
// If P >= NUM_HOLES, perform P div NUM_HOLES complete turns.
// If position is unknown, perform first a position detection, backward.
// $ echo -n -e '<\x00' > /dev/ear0

// Get current position.
// Command = '?'
// Will not turn.
// Next read byte is 0-16 (position) or -1 (unknown)
// $ echo -n -e '?' > /dev/ear0 && dd if=/dev/ear0 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'

// Get current position, performing a position detection if required (forward)
// Command = '!'
// May not turn.
// Next read byte is 0-16 (position).
// $ echo -n -e '!' > /dev/ear0 && dd if=/dev/ear0 of=/dev/stdout count=1 bs=1 status=none | hexdump -e '/1 "%d\n"'

// When a get current position command finishes, read returns -1 or 0-16.
// Otherwise, reading blocks until ear is moved by user. Then it returns 'm'.
// Any move operation clears the "moved flag". When this flag is cleared, driver
// returns the last known position (before/after move depending on when read
// is performed).

// Get position commands also overwrite this "moved flag".

static void move_forward(struct tagtagtagear_data *priv, unsigned char arg) {
    priv->read_result = priv->state.idle.position;
    transition_to_running(priv, priv->state.idle.position, arg);
}

static void move_backward(struct tagtagtagear_data *priv, unsigned char arg) {
    priv->read_result = priv->state.idle.position;
    transition_to_running(priv, priv->state.idle.position, -arg);
}

static void goto_forward(struct tagtagtagear_data *priv, unsigned char arg) {
    priv->read_result = priv->state.idle.position;
    if (priv->state.idle.position == -1) {
        transition_to_detecting(priv, goto_position, 1, arg);
    } else if (priv->state.idle.position != arg) {
        int delta = arg - priv->state.idle.position;
        if (delta < 0) {
            delta += NUM_HOLES;
        }
        transition_to_running(priv, priv->state.idle.position, delta);
    }
}

static void goto_backward(struct tagtagtagear_data *priv, unsigned char arg) {
    priv->read_result = priv->state.idle.position;
    if (priv->state.idle.position == -1) {
        transition_to_detecting(priv, goto_position, -1, arg);
    } else if (priv->state.idle.position != arg) {
        int delta = priv->state.idle.position - arg;
        if (delta > 0) {
            delta -= NUM_HOLES;
        }
        transition_to_running(priv, priv->state.idle.position, delta);
    }
}

static void get_position(struct tagtagtagear_data *priv, int run_detection) {
    if (priv->state.idle.position == -1) {
        if (run_detection) {
            transition_to_detecting(priv, read_position, 1, 0);
        } else {
            priv->read_result_available = 1;
            priv->read_result = -1;
            wake_up_interruptible(&priv->read_wq);
        }
    } else {
        priv->read_result_available = 1;
        priv->read_result = priv->state.idle.position;
        wake_up_interruptible(&priv->read_wq);
    }
}

static int ear_open(struct inode *inode, struct file *file) {
    struct tagtagtagear_data *ear_data;
    ear_data = container_of(inode->i_cdev, struct tagtagtagear_data, cdev);
    file->private_data = ear_data;

    if (ear_data->opened) {
        return -EBUSY;
    }
    ear_data->opened = 1;
    return 0;
}

static int ear_release(struct inode *inode, struct file *file) {
    struct tagtagtagear_data *ear_data;
    ear_data = container_of(inode->i_cdev, struct tagtagtagear_data, cdev);
    ear_data->opened = 0;
    return 0;
}

static int ear_read(struct file *file, char __user *buffer, size_t len, loff_t *offset) {
    struct tagtagtagear_data *priv = (struct tagtagtagear_data *) file->private_data;
    if (priv->state_e == broken) {
        return 0;
    }
    if (wait_event_interruptible(priv->read_wq, priv->read_result_available != 0)) {
        return -ERESTARTSYS;
    }
    if (len <= 0) {
        return 0;
    }
    if (priv->read_result_available) {
        if (copy_to_user(buffer, &priv->read_result, 1)) {
            return -EFAULT;
        }
        priv->read_result_available = 0;
        return 1;
    }
    return 0;
}

static int ear_write(struct file *file, const char __user *buffer, size_t len, loff_t *offset) {
    struct tagtagtagear_data *priv = (struct tagtagtagear_data *) file->private_data;
    if (wait_event_interruptible(priv->write_wq, priv->state_e == broken || priv->state_e == idle)) {
        return -ERESTARTSYS;
    }
    if (len <= 0) {
        return 0;
    }
    if (priv->state_e == broken) {
        return -EFAULT;
    }
    if (priv->state_e == idle) {
        // I need 1 or 2 bytes.
        char kbuffer[2];
        int kbuffer_size = 0;
        int read = 0;
        if (priv->buffer_size > 0) {
            // Just missing parameter
            kbuffer[0] = priv->buffer[0];
            if (copy_from_user(kbuffer + 1, buffer, 1)) {
                return -EFAULT;
            }
            kbuffer_size = 2;
            read = 1;
        } else {
            if (copy_from_user(kbuffer, buffer, 1)) {
                return -EFAULT;
            }
            read = 1;
            if (kbuffer[0] == '+' || kbuffer[0] == '-' || kbuffer[0] == '>' || kbuffer[0] == '<') {
                if (len == 1) {
                    priv->buffer_size = 1;
                    priv->buffer[0] = kbuffer[0];
                    *offset += 1;
                    return 1;
                }
                if (copy_from_user(kbuffer + 1, buffer + 1, 1)) {
                    return -EFAULT;
                }
                read = 2;
            }
        }
        priv->buffer_size = 0;
        switch (kbuffer[0]) {
            case '.':
                // NOP.
                break;

            case '+':
                move_forward(priv, (unsigned char) kbuffer[1]);
                break;

            case '-':
                move_backward(priv, (unsigned char) kbuffer[1]);
                break;

            case '>':
                goto_forward(priv, (unsigned char) kbuffer[1]);
                break;

            case '<':
                goto_backward(priv, (unsigned char) kbuffer[1]);
                break;

            case '?':
                get_position(priv, 0);
                break;

            case '!':
                get_position(priv, 1);
                break;
        }
        *offset += read;
        return read;
    }
    return 0;
}

static unsigned int ear_poll(struct file *file, poll_table *wait) {
    struct tagtagtagear_data *priv = (struct tagtagtagear_data *) file->private_data;
    unsigned int mask = 0;

    poll_wait(file, &priv->write_wq,  wait);
    poll_wait(file, &priv->read_wq, wait);

    if (priv->state_e == broken) {
        mask |= POLLHUP;
    } else {
        if (priv->state_e == idle) {
            mask |= POLLOUT | POLLWRNORM;
        }
        if (priv->read_result_available != 0) {
            mask |= POLLIN | POLLRDNORM;
        }
    }
    return mask;
}

static struct file_operations ear_fops = {
    .owner = THIS_MODULE,
    .open = ear_open,
    .read = ear_read,
    .write = ear_write,
    .release = ear_release,
    .poll = ear_poll,
};

// ========================================================================== //
// Probing, initialization and cleanup
// ========================================================================== //

static int init_ear(struct device *dev, struct tagtagtagear_data *priv, struct class *ears_class, int major, int minor, const char* encoder_name, const char* motor_name) {
    dev_t devno = MKDEV(major, minor);
    int err;
    int irq;

    priv->encoder_gpio = devm_gpiod_get(dev, encoder_name, GPIOD_IN);
    if (IS_ERR(priv->encoder_gpio)) {
        err = PTR_ERR(priv->encoder_gpio);
        if (err != -EPROBE_DEFER)
            dev_err(dev, "Failed to get 'encoder' gpio for %s: %d", encoder_name, err);
        return err;
    }

    priv->motor_gpios = devm_gpiod_get_array(dev, motor_name, GPIOD_OUT_LOW);
    if (IS_ERR(priv->motor_gpios)) {
        err = PTR_ERR(priv->motor_gpios);
        if (err != -EPROBE_DEFER)
            dev_err(dev, "Failed to get 'motor' gpios for %s: %d", motor_name, err);
        return err;
    }

    cdev_init(&priv->cdev, &ear_fops);
    err = cdev_add(&priv->cdev, devno, 1);
    if (err) {
        dev_err(dev, "Failed to add cdev for %d: %d", minor, err);
        return err;
    }

	priv->device = device_create(ears_class, dev, devno, NULL, /* no additional data */
		DEVICE_NAME "%d", minor);
	if (IS_ERR(priv->device)) {
		err = PTR_ERR(priv->device);
        dev_err(dev, "Failed to create device for %d: %d", minor, err);
        return err;
    }

    // Setup timer for broken ears
    timer_setup(&priv->broken_timer, tagtagtagear_broken_timer_cb, 0);

    // Request interrupts from encoder GPIOs
    irq = gpiod_to_irq(priv->encoder_gpio);
    err = devm_request_any_context_irq(dev, irq,
                    tagtagtagear_irq_handler, IRQF_TRIGGER_FALLING,
                    DRV_NAME, priv);
    if (err < 0)
        return err;

    // Setup wait queues
    init_waitqueue_head(&priv->read_wq);
    init_waitqueue_head(&priv->write_wq);

    transition_to_testing(priv);

    return 0;
}

static int tagtagtagears_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct tagtagtagears_data *priv;
    int err;

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    platform_set_drvdata(pdev, priv);

    // Register device.
    err = alloc_chrdev_region(&priv->chrdev, 0, 2, DEVICE_NAME);
    if (err < 0) {
        dev_err(dev, "Failed to registering character device: %d", err);
        tagtagtagears_remove(pdev);
        return err;
    }

	// Create device class
	priv->ears_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(priv->ears_class)) {
		err = PTR_ERR(priv->ears_class);
        dev_err(dev, "class_create failed: %d", err);
        tagtagtagears_remove(pdev);
        return err;
	}

    err = init_ear(dev, &priv->ear[0], priv->ears_class, MAJOR(priv->chrdev), MINOR(priv->chrdev), "left-encoder", "left-motor");
    if (err < 0) {
        dev_err(dev, "init_ear failed for left ear: %d", err);
        tagtagtagears_remove(pdev);
        return err;
    }

    err = init_ear(dev, &priv->ear[1], priv->ears_class, MAJOR(priv->chrdev), MINOR(priv->chrdev) + 1, "right-encoder", "right-motor");
    if (err < 0) {
        dev_err(dev, "init_ear failed for right ear: %d", err);
        tagtagtagears_remove(pdev);
        return err;
    }

    return 0;
}

static int tagtagtagears_remove(struct platform_device *pdev) {
    struct tagtagtagears_data *priv;
    int ix;
    priv = platform_get_drvdata(pdev);

    if (priv->chrdev) {
        if (priv->ears_class) {
            for (ix = 1; ix >= 0; ix--) {
                if (priv->ear[ix].cdev.ops) {
                    del_timer_sync(&priv->ear[ix].broken_timer);
                    device_destroy(priv->ears_class, MKDEV(MAJOR(priv->chrdev), MINOR(priv->chrdev) + ix));
                    cdev_del(&priv->ear[ix].cdev);
                }
            }
            class_destroy(priv->ears_class);
        }
        unregister_chrdev_region(priv->chrdev, 2);
    }
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tagtagtagears_ids[] = {
    { .compatible = "linux,tagtagtag-ears", },
    { }
};
MODULE_DEVICE_TABLE(of, tagtagtagears_ids);
#endif

static struct platform_driver tagtagtagears_driver = {
    .driver = {
        .name = DRV_NAME,
        .of_match_table = of_match_ptr(tagtagtagears_ids),
    },
    .probe              = tagtagtagears_probe,
    .remove             = tagtagtagears_remove,
};

module_platform_driver(tagtagtagears_driver);

MODULE_DESCRIPTION("Nabaztagtagtag ears driver");
MODULE_AUTHOR("Paul Guyot <pguyot@kallisys.net>");
MODULE_LICENSE("GPL");
