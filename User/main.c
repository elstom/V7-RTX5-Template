/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块
*	文件名称 : main.c
*	版    本 : V1.0
*	说    明 : 创建RTX5工程模板。
*              实验目的：
*                1. 创建RTX5工程模板。
*              实验内容：
*                1. K1按键按下，串口打印。
*                2. 各个任务实现的功能如下：
*                   AppTaskUserIF任务   : 按键消息处理。
*                   AppTaskLED任务      : LED闪烁。
*                   AppTaskMsgPro任务   : 消息处理。
*                   AppTaskStart任务    : 启动任务，也是最高优先级任务，这里实现按键扫描。
*                   osRtxTimerThread任务: 定时器任务，暂未使用。
*              注意事项：
*                1. 为了RTX5的调试组件正常使用，RTX5的工程路径切不要有中文，路径不要太长。
*                   而且退出调试状态要清除所有断点。
*                2. 本实验推荐使用串口软件SecureCRT查看打印信息，波特率115200，数据位8，奇偶校验位无，停止位1。
*                3. 务必将编辑器的缩进参数和TAB设置为4来阅读本文件，要不代码显示不整齐。
*
*	修改记录 :
*		版本号   日期         作者        说明
*		V1.0    2019-04-10   Eric2013     1. CMSIS软包版本 V5.5.1
*                                         2. HAL库版本 V1.3.0
*                                         3. RTX5版本5.5.0
*                                         4. Event Recorder版本1.4.0
*
*	Copyright (C), 2019-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/	
#include "includes.h"



/*
**********************************************************************************************************
											函数声明
**********************************************************************************************************
*/
static void AppTaskCreate (void);
void AppTaskUserIF(void *argument);
void AppTaskLED(void *argument);
void AppTaskMsgPro(void *argument);
void AppTaskStart(void *argument);
void AppTaskMLX90640(void *argument);


/*
**********************************************************************************************************
											 变量
**********************************************************************************************************
*/
/* 任务的属性设置 */
const osThreadAttr_t ThreadStart_Attr = 
{
	/* 未使用 */
//	.cb_mem = &worker_thread_tcb_1,
//	.cb_size = sizeof(worker_thread_tcb_1),
//	.stack_mem = &worker_thread_stk_1[0],
//	.stack_size = sizeof(worker_thread_stk_1),
//	.priority = osPriorityAboveNormal,
//	.tz_module = 0
	
	.name = "osRtxStartThread",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh4,
	.stack_size = 2048,
};

const osThreadAttr_t ThreadMsgPro_Attr = 
{
	.name = "osRtxMsgProThread",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh3,
	.stack_size = 1024,
};

const osThreadAttr_t ThreadLED_Attr = 
{
	.name = "osRtxLEDThread",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh2,
	.stack_size = 512,
};

const osThreadAttr_t ThreadUserIF_Attr = 
{
	.name = "osRtxThreadUserIF",
	.attr_bits = osThreadDetached, 
	.priority = osPriorityHigh1,
	.stack_size = 1024,
};

const osThreadAttr_t ThreadMLX90640_Attr = 
{
    .name = "osRtxMLX90640Thread",
    .attr_bits = osThreadDetached,
    .priority = osPriorityNormal,
    .stack_size = 2048,  // 设置合适的堆栈大小
};


/* 任务句柄 */
osThreadId_t ThreadIdTaskUserIF = NULL;
osThreadId_t ThreadIdTaskMsgPro = NULL;
osThreadId_t ThreadIdTaskLED = NULL;
osThreadId_t ThreadIdTaskMLX90640 = NULL;
osThreadId_t ThreadIdStart = NULL;


/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int main (void) 
{	
	/* HAL库，MPU，Cache，时钟等系统初始化 */
	System_Init();

	/* 内核开启前关闭HAL的时间基准 */
	HAL_SuspendTick();
	
	/* 内核初始化 */
	osKernelInitialize();                                  

	/* 创建启动任务 */
	ThreadIdStart = osThreadNew(AppTaskStart, NULL, &ThreadStart_Attr);  

	/* 开启多任务 */
	osKernelStart();
	
	while(1);
}

#define MLX90640_SLAVE_ADDR  0x33  // 7位地址
#define MLX90640_FRAME_DELAY 1000  // 每帧延迟1秒（1000ms）

static paramsMLX90640 mlx90640;  // 存储校准参数

// 任务函数：MLX90640 传感器任务
void AppTaskMLX90640(void *argument)
{
    uint16_t eeData[832];         // EEPROM 数据
    uint16_t frameData[834];      // 存储传感器帧数据
    float temperatureData[768];   // 存储计算出的温度
    float emissivity = 1.0f;      // 辐射率（一般为 1.0）
    float tr = 23.0f;             // 环境反射温度（单位：摄氏度）
    
    int error;
    uint16_t controlRegister;

    // 1. 初始化 I2C
    // MLX90640_I2CInit();

    // 2. 检查设备响应 - 通过读取控制寄存器确认
    error = MLX90640_I2CRead(MLX90640_SLAVE_ADDR, MLX90640_CTRL_REG, 1, &controlRegister);
    if (error != 0) {
        printf("MLX90640 连接失败，无法读取控制寄存器，错误代码: %d\r\n", error);
        osThreadExit();  // 任务退出
    }

    // 3. 从 EEPROM 读取校准参数
    error = MLX90640_DumpEE(MLX90640_SLAVE_ADDR, eeData);
    if (error != 0) {
        printf("读取 EEPROM 数据失败，错误代码: %d\r\n", error);
        osThreadExit();  // 任务退出
    }

    // 4. 提取校准参数
    error = MLX90640_ExtractParameters(eeData, &mlx90640);
    if (error != 0) {
        printf("提取校准参数失败，错误代码: %d\r\n", error);
        osThreadExit();  // 任务退出
    }

    // 5. 设置刷新率（例如 16 Hz）
    error = MLX90640_SetRefreshRate(MLX90640_SLAVE_ADDR, 0x03);  // 16Hz
    if (error != 0) {
        printf("设置刷新率失败，错误代码: %d\r\n", error);
        osThreadExit();  // 任务退出
    }

    // 循环读取温度数据
    while (1) {
        // 6. 获取帧数据
        error = MLX90640_GetFrameData(MLX90640_SLAVE_ADDR, frameData);
        if (error != 0) {
            printf("读取帧数据失败，错误代码: %d\r\n", error);
            continue;
        }

        // 7. 计算每个像素的温度
        MLX90640_CalculateTo(frameData, &mlx90640, emissivity, tr, temperatureData);

        // 8. 输出或处理温度数据
        for (int i = 0; i < 768; i++) {
            printf("Pixel %d: 温度 = %.2f°C\r\n", i, temperatureData[i]);
        }

        // 9. 等待下一帧读取（每秒钟读取一次）
        osDelay(MLX90640_FRAME_DELAY);
    }
}


/*
*********************************************************************************************************
*	函 数 名: AppTaskUserIF
*	功能说明: 按键消息处理		
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh1 (数值越小优先级越低，这个跟uCOS相反)
*********************************************************************************************************
*/
void AppTaskUserIF(void *argument)
{
	uint8_t ucKeyCode;

    while(1)
    {
		ucKeyCode = bsp_GetKey();
		
		if (ucKeyCode != KEY_NONE)
		{
			switch (ucKeyCode)
			{
				/* K1键按下，打印调试说明 */
				case KEY_DOWN_K1:
					printf("KEY_DOWN_K1\r\n");
					break;	

				/* 其他的键值不处理 */
				default:                     
					break;
			}
		}
		
		osDelay(20);
	}
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskLED
*	功能说明: LED闪烁。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh2 
*********************************************************************************************************
*/
void AppTaskLED(void *argument)
{
	const uint16_t usFrequency = 200; /* 延迟周期 */
	uint32_t tick;

	/* 获取当前时间 */
	tick = osKernelGetTickCount(); 
	
    while(1)
    {
		bsp_LedToggle(2);
		/* 相对延迟 */
		tick += usFrequency;                          
		osDelayUntil(tick);
    }
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskMsgPro
*	功能说明: 消息处理，这里用作验证Event Recoder。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh3  
*********************************************************************************************************
*/
void AppTaskMsgPro(void *argument)
{
	while(1)
	{
		/* 验证Event Recoder的时间统计功能 */
		EventStartA(1);
		osDelay(10);
		EventStopA(1);
	}	
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskStart
*	功能说明: 启动任务，这里用作BSP驱动包处理。
*	形    参: 无
*	返 回 值: 无
*   优 先 级: osPriorityHigh4  
*********************************************************************************************************
*/
void AppTaskStart(void *argument)
{
	const uint16_t usFrequency = 1; /* 延迟周期 */
	uint32_t tick;
	
	/* 初始化外设 */
	HAL_ResumeTick();
	bsp_Init();

	/* 创建任务 */
	AppTaskCreate();

	/* 获取当前时间 */
	tick = osKernelGetTickCount(); 
	
    while(1)
    {
		/* 需要周期性处理的程序，对应裸机工程调用的SysTick_ISR */
		bsp_ProPer1ms();
		
		/* 相对延迟 */
		tick += usFrequency;                          
		osDelayUntil(tick);
    }
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
	ThreadIdTaskMsgPro = osThreadNew(AppTaskMsgPro, NULL, &ThreadMsgPro_Attr);  
	ThreadIdTaskLED = osThreadNew(AppTaskLED, NULL, &ThreadLED_Attr);  
	ThreadIdTaskUserIF = osThreadNew(AppTaskUserIF, NULL, &ThreadUserIF_Attr);  
	ThreadIdTaskMLX90640 = osThreadNew(AppTaskMLX90640, NULL, &ThreadMLX90640_Attr);  
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
