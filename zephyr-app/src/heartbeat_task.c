#include "heartbeat_task.h"

#include <zephyr/kernel.h>

#include "modem_task.h"
#include "log.h"

#define TAG "heartbeat"

#define HEARTBEAT_PERIOD_MS (5 * MSEC_PER_SEC)

static void heartbeat_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	char buf[224];

	while (true) {
		modem_status_format(buf, sizeof(buf));
		info(TAG, "%s", buf);

		k_msleep(HEARTBEAT_PERIOD_MS);
	}
}

#define HEARTBEAT_STACK_SIZE 1024
#define HEARTBEAT_PRIORITY   7

K_THREAD_DEFINE(heartbeat_tid, HEARTBEAT_STACK_SIZE, heartbeat_task, NULL, NULL, NULL,
		 HEARTBEAT_PRIORITY, 0, 0);
