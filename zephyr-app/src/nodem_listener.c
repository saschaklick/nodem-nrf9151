#include "nodem_listener.h"

#include <zephyr/sys/reboot.h>

void nodem_hw_reset(void)
{
	sys_reboot(SYS_REBOOT_COLD);
}
