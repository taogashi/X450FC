#ifndef _DISKTASK_H_
#define _DISKTASK_H_

#include "ff.h"
#include "diskio.h"
#include "OSConfig.h"
#include <string.h>

extern xQueueHandle xDiskWrQueue1;
extern xQueueHandle xDiskLogQueue;
void vDiskOperation(void* vParameter);

#endif
