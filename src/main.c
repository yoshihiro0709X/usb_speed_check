#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// User queue
static struct k_work_q uq_button[4];
K_THREAD_STACK_DEFINE(us_button0, 512);
K_THREAD_STACK_DEFINE(us_button1, 512);
K_THREAD_STACK_DEFINE(us_button2, 512);
K_THREAD_STACK_DEFINE(us_button3, 512);

// Semaphore
K_SEM_DEFINE(sem_timer, 0, 1);

// Global variable(s)
static uint8_t ring_buffer[4096];
static struct ring_buf ringbuf;
static uint32_t time_spent;
static uint32_t count_sent;

static const struct device *dev_usbd = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

static void timer_stop_handler(struct k_timer *timer);
static void timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(timer, timer_handler, timer_stop_handler);

/* ----- USB Function Start ----- */
static void usbd_send_work_thread(void)
{
	int sent_size;
	uint8_t data[1024];

	while (true) {
		k_sem_take(&sem_timer, K_FOREVER);

		sent_size = uart_fifo_fill(dev_usbd, data, sizeof(data));
		if (sent_size <= 0) {
			LOG_ERR("Cannot send data via USB.");
			k_timer_stop(&timer);
		} else if (sent_size != sizeof(data)) {
			LOG_WRN("Not an error, but data is only partially transmitted.");
			k_timer_stop(&timer);
		}

		count_sent++;
		if ((count_sent % 1000) == 0) {
			LOG_INF("%d Mbytes sent.", (count_sent / 1000));
		}
	}
}
K_THREAD_DEFINE(usbd_send, 2048, usbd_send_work_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(0), 0, 0);

static void usbd_callback_handler(const struct device *dev, void *user_data)
{
	int recv_len;
	uint8_t usb_buffer[64];
	static uint8_t command_buffer[256] = {0};
	static uint16_t p_command_buffer = 0;

	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		// Receive
		if (uart_irq_rx_ready(dev)) {
			// Receive command
			while (true) {
				memset(usb_buffer, 0, sizeof(usb_buffer));
				size_t len = MIN(sizeof(command_buffer) - p_command_buffer - 1, sizeof(usb_buffer));
				recv_len = uart_fifo_read(dev, usb_buffer, len);

				if (recv_len == 0) {
					break;
				} else if (p_command_buffer + recv_len < (sizeof(command_buffer) - 1)) {
					LOG_INF("Received %d byte(s)", recv_len);
					memcpy(&command_buffer[p_command_buffer], usb_buffer, recv_len);
					p_command_buffer += recv_len;
				} else {
					// Buffer is full
					LOG_WRN("Received %d byte(s), Buffer is full.", recv_len);
					memcpy(&command_buffer[p_command_buffer], usb_buffer, (sizeof(command_buffer) - p_command_buffer - 1));
					p_command_buffer = (sizeof(command_buffer) - 1);
					break;
				}
				k_msleep(3);
			}

			// Clear
			memset(command_buffer, 0, sizeof(command_buffer));
			p_command_buffer = 0;
		}
	}
}
/* ----- USB Function End ----- */

/* ----- Timer Function Start ----- */
static void timer_stop_handler(struct k_timer *timer)
{
	LOG_DBG("Timer stopped.");
}

static void timer_handler(struct k_timer *timer)
{
	// Resume thread
	LOG_DBG("Send confirmation packet.");
	k_sem_give(&sem_timer);
}
/* ----- Timer Function End ----- */

#if DT_NODE_EXISTS(DT_ALIAS(sw2))
static void button3_holding_work_handler(struct k_work *work)
{
	LOG_INF("button 4 holding.");
}
K_WORK_DELAYABLE_DEFINE(button3_holding, button3_holding_work_handler);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(sw3))
static void button2_holding_work_handler(struct k_work *work)
{
	LOG_INF("button 3 holding.");
}
K_WORK_DELAYABLE_DEFINE(button2_holding, button2_holding_work_handler);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(sw2))
static void button1_holding_work_handler(struct k_work *work)
{
	LOG_INF("button 2 holding.");
	k_timer_stop(&timer);
    // Measure the time spent
    time_spent = k_cycle_get_32() - time_spent;
    LOG_INF("Time spent:%d(msec)", (time_spent / 1000));
}
K_WORK_DELAYABLE_DEFINE(button1_holding, button1_holding_work_handler);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(sw1))
static void button0_holding_work_handler(struct k_work *work)
{
	LOG_INF("button 1 holding.");
	k_timer_start(&timer, K_NO_WAIT, K_USEC(2500));
    // Measure the time spent
	count_sent = 0;
    time_spent = k_cycle_get_32();
}
K_WORK_DELAYABLE_DEFINE(button0_holding, button0_holding_work_handler);
#endif

void button_changed(uint32_t button_state, uint32_t has_changed)
{
#if DT_NODE_EXISTS(DT_ALIAS(sw0))
	if (has_changed & 0x01) {
		if (button_state & has_changed) {
			LOG_INF("button 1 changes to on");
			k_work_schedule_for_queue(&uq_button[0], &button0_holding, K_MSEC(2000));
		} else {
			LOG_INF("button 1 changes to off.");
			k_work_cancel_delayable(&button0_holding);
		}
	}
#endif

#if DT_NODE_EXISTS(DT_ALIAS(sw1))
	if (has_changed & 0x02) {
		if (button_state & has_changed) {
			LOG_INF("button 2 changes to on");
			k_work_schedule_for_queue(&uq_button[1], &button1_holding, K_MSEC(2000));
		} else {
			LOG_INF("button 2 changes to off.");
			k_work_cancel_delayable(&button1_holding);
		}
	}
#endif

#if DT_NODE_EXISTS(DT_ALIAS(sw2))
	if (has_changed & 0x04) {
		if (button_state & has_changed) {
			LOG_INF("button 3 changes to on");
			k_work_schedule_for_queue(&uq_button[2], &button2_holding, K_MSEC(2000));
		} else {
			LOG_INF("button 3 changes to off.");
			k_work_cancel_delayable(&button2_holding);
		}
	}
#endif

#if DT_NODE_EXISTS(DT_ALIAS(sw3))
	if (has_changed & 0x08) {
		if (button_state & has_changed) {
			LOG_INF("button 4 changes to on");
			k_work_schedule_for_queue(&uq_button[3], &button3_holding, K_MSEC(2000));
		} else {
			LOG_INF("button 4 changes to off.");
			k_work_cancel_delayable(&button3_holding);
		}
	}
#endif
}

int main(void)
{
	int err;

    // Initialize Buttons and LEDs
    err = dk_buttons_init(button_changed);
    if (err) {
        LOG_ERR("Cannot init buttons.(%d)", err);
		return -1;
	}
	err = dk_leds_init();
	if (err) {
        LOG_ERR("Cannot init LEDs.(%d)", err);
		return -1;
    }

	// User queue for button(s) and led(s)
	k_work_queue_start(&uq_button[0], us_button0, K_THREAD_STACK_SIZEOF(us_button0), K_PRIO_PREEMPT(15), NULL);
	k_work_queue_start(&uq_button[1], us_button1, K_THREAD_STACK_SIZEOF(us_button1), K_PRIO_PREEMPT(15), NULL);
	k_work_queue_start(&uq_button[2], us_button2, K_THREAD_STACK_SIZEOF(us_button2), K_PRIO_PREEMPT(15), NULL);
	k_work_queue_start(&uq_button[3], us_button3, K_THREAD_STACK_SIZEOF(us_button3), K_PRIO_PREEMPT(15), NULL);

    // USB
    err = usb_enable(NULL);
	if (err) {
		LOG_ERR("Failed to enable USB.(%d)", err);
		return -1;
	}
	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);
	uart_irq_callback_set(dev_usbd, usbd_callback_handler);
	uart_irq_rx_enable(dev_usbd);

	// Main Thread
	while (true) {
        k_msleep(2000);
	}
}
