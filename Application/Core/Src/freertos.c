#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


osThreadId gimbalTaskHandle;
osThreadId launcherTaskHandle;
osThreadId chassisTaskHandle;
osThreadId imuTaskHandle;
osThreadId monitorTaskHandle;
osThreadId robotTaskHandle;
osThreadId controlTaskHandle;


extern void gimbal_task(void const * argument);
extern void launcher_task(void const * argument);
extern void chassis_task(void const * argument);
extern void imu_task(void const * argument);
extern void monitor_task(void const * argument);
extern void robot_task(void const * argument);
extern void control_task(void const * argument);


void StartDefaultTask(void const * argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );


static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}


void MX_FREERTOS_Init(void) {
  
	osThreadDef(gimbalTask, gimbal_task, osPriorityAboveNormal, 0, 512);
	gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

	
	osThreadDef(launcherTask, launcher_task, osPriorityAboveNormal, 0, 512);
	gimbalTaskHandle = osThreadCreate(osThread(launcherTask), NULL);

	
	osThreadDef(chassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
	gimbalTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

	
 	osThreadDef(imuTask, imu_task, osPriorityRealtime, 0, 512);
 	imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
	
	
	osThreadDef(monitorTask, monitor_task, osPriorityRealtime, 0, 256);
 	imuTaskHandle = osThreadCreate(osThread(monitorTask), NULL);

	
 	osThreadDef(robotTask, robot_task, osPriorityHigh, 0, 128);
 	imuTaskHandle = osThreadCreate(osThread(robotTask), NULL);

	
 	osThreadDef(controlTask, control_task, osPriorityNormal, 0, 128);
 	imuTaskHandle = osThreadCreate(osThread(controlTask), NULL);

}

void StartDefaultTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

