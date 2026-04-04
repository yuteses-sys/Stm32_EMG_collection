#include "app_task.h"


TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xFourGTaskHandle = NULL;
TaskHandle_t xRC522TaskHandle = NULL;
TaskHandle_t xbrainTaskHandle = NULL;



/* 
*2. 任务实体函数 
 */
static void vFourGTask(void *pvParameters) {
    for(;;) {

        } 

  

}





/* * 3. 任务初始化大管家
 */
void System_Task_Init(void) {

    //4G短信模块
    // xRet = xTaskCreate(vFourGTask,     "4G_Task",   512, NULL, tskIDLE_PRIORITY + 2, &xFourGTaskHandle);
    // if(xRet != pdPASS) while(1);


}