/*
*********************************************************************************************************
*
*	模块名称 : 安富莱LED-485-XXX系列数码管的驱动程序
*	文件名称 : bsp_rs485_led.c
*	版    本 : V1.0
*	说    明 : 驱动安富莱电子生产的RS485 LED数码管显示屏。 使用了 bsp_modbus.c 文件。
*			  型号: LED-485-043	 三位0.4寸数码管
*				    LED-485-034  四位0.3寸数码管
*				    LED-485-083  三位0.8寸数码管
*				    LED-485-054  四位0.56寸数码管
*
*			  支持ASCII协议 和 Modbus RTU协议。可以通过485指令进行切换。
*
*	Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

/* RTU 应答代码 */
#define RSP_OK				0		/* 成功 */
#define RSP_ERR_CMD			0x01	/* 不支持的功能码 */
#define RSP_ERR_REG_ADDR	0x02	/* 寄存器地址错误 */
#define RSP_ERR_VALUE		0x03	/* 数据值域错误 */
#define RSP_ERR_WRITE		0x04	/* 写入失败 */

static void MODH_RxTimeOut(void);
static uint8_t g_rtu_timeout = 0;

MODH_T g_tModH;

extern void MODBUS_AnalyzeApp(void);

/*
*********************************************************************************************************
*	函 数 名: MODH_SendWithCRC
*	功能说明: 发送一串数据, 自动追加2字节CRC
*	形    参: g_tModH.TxBuf : 数据
*			  g_tModH.TxLen : 数据长度（不带CRC）
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_SendWithCRC(void)
{
	uint16_t crc;

	crc = CRC16_Modbus(g_tModH.TxBuf, g_tModH.TxLen);
	g_tModH.TxBuf[g_tModH.TxLen++] = crc >> 8;
	g_tModH.TxBuf[g_tModH.TxLen++] = crc;
	RS485_SendBuf(g_tModH.TxBuf, g_tModH.TxLen);
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send06H
*	功能说明: 发送06H指令，写一个寄存器
*	形    参: _RS485Addr : 485地址
*			  _RegAddr : 寄存器地址
*			  _RegValue : 寄存器值
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send06H(uint8_t _RS485Addr, uint16_t _RegAddr, uint16_t _RegValue)
{
	g_tModH.TxLen = 0;
	g_tModH.TxBuf[g_tModH.TxLen++] = _RS485Addr;			/* 485地址 */
	g_tModH.TxBuf[g_tModH.TxLen++] = 0x06;			/* 功能码 */
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegAddr >> 8;		/* 寄存器地址*/
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegAddr;
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegValue >> 8;	/* 寄存器值 */
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegValue;	
	MODH_SendWithCRC();
	g_tModH.fAckOK = 0;
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send06H
*	功能说明: 发送06H指令，写一个寄存器
*	形    参: _RS485Addr : 485地址
*			  _RegAddr : 寄存器起始地址
*			  _RegNum : 寄存器个数
*			  _RegValue : 寄存器值数组
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send10H(uint8_t _RS485Addr, uint16_t _RegAddr, uint16_t _RegNum, uint16_t *_RegValue)
{
	uint8_t i;
	
	g_tModH.TxLen = 0;
	g_tModH.TxBuf[g_tModH.TxLen++] = _RS485Addr;			/* 485地址 */
	g_tModH.TxBuf[g_tModH.TxLen++] = 0x10;			/* 功能码 */
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegAddr >> 8;	/* 寄存器起始地址*/
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegAddr;
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegNum >> 8;	/* 寄存器个数 */
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegNum;	
	g_tModH.TxBuf[g_tModH.TxLen++] = _RegNum * 2;	/* 数据区字节数 */
	for (i = 0; i < _RegNum; i++)
	{
		g_tModH.TxBuf[g_tModH.TxLen++] = _RegValue[i] >> 8;
		g_tModH.TxBuf[g_tModH.TxLen++] = _RegValue[i];
	}
	MODH_SendWithCRC();
	g_tModH.fAckOK = 0;
}

/*
*********************************************************************************************************
*	函 数 名: MODH_ReciveNew
*	功能说明: 串口接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。MODBUS HOST，
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_ReciveNew(uint8_t _byte)
{
	/*
		3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
		两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
		所以就是3.5个字符的时间，波特率高，这个时间间隔就小，波特率低，这个时间间隔相应就大

        波特率	延时3.5字符(ms)
        1200	29.16666667
        2400	14.58333333
        4800	7.291666667
        
        ----- 以下都取4ms ----
        
        9600	3.645833333
        19200	1.822916667
        38400	0.911458333
        57600	0.607638889
        115200	0.303819444
        
		const uint32_t TimeOut[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
    	const uint8_t TimeOut[] = {30, 15, 8, 4, 4, 4, 4, 4};
	*/
	uint32_t timeout;

	g_rtu_timeout = 0;
	
	if (g_tModH.Baud >= 9600)
	{
		timeout = 4000;		/* 4000us */
	}
	else
	{
		timeout =  35000000 / g_tModH.Baud;	/* 计算超时时间，单位us */
	}

	/* H743支持串口空闲中断，LINE TIMEOUT中断实现 超时检测. 此处可以优化 */
	/* 硬件定时中断，定时精度us 定时器4用于Modbus */
	bsp_StartHardTimer(1, timeout, (void *)MODH_RxTimeOut);
	
	if (g_tModH.RxCount < MODH_RX_SIZE)
	{
		g_tModH.RxBuf[g_tModH.RxCount++] = _byte;
	}
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Poll
*	功能说明: 解析数据包. 在主程序中轮流调用。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Poll(void)
{
	uint16_t crc1;

	if (g_rtu_timeout == 0)
	{
		/* 没有超时，继续接收。不要清零 g_tModH.RxCount */
		return;
	}

	/* 收到命令
		05 06 00 88 04 57 3B70 (8 字节)
			05    :  数码管屏的号站，
			06    :  指令
			00 88 :  数码管屏的显示寄存器
			04 57 :  数据,,,转换成 10 进制是 1111.高位在前,
			3B70  :  二个字节 CRC 码	从05到 57的校验
	*/
	g_rtu_timeout = 0;

	if (g_tModH.RxCount < 4)
	{
		goto err_ret;
	}

	/* 计算CRC校验和 */
	crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
	if (crc1 != 0)
	{
		/* 将接收的数据复制到另外一个缓冲区，等待APP程序读取 */
		memcpy(g_tModH.AppRxBuf, g_tModH.RxBuf, g_tModH.RxCount);
		g_tModH.AppRxCount = g_tModH.RxCount;		
		bsp_PutKey(MSG_485_RX_NOT_RTU);		/* 借用按键FIFO，发送一个收到485数据帧的消息 */
		goto err_ret;
	}
	else
	{
		/* 将接收的数据复制到另外一个缓冲区，等待APP程序读取 */
		memcpy(g_tModH.AppRxBuf, g_tModH.RxBuf, g_tModH.RxCount);
		g_tModH.AppRxCount = g_tModH.RxCount;
		bsp_PutKey(MSG_485_RX_RTU);		/* 借用按键FIFO，发送一个收到485数据帧的消息 */
	}
err_ret:
	g_tModH.RxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_RxTimeOut
*	功能说明: 超过3.5个字符时间后执行本函数。 设置全局变量 g_rtu_timeout = 1; 通知主程序开始解码。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_RxTimeOut(void)
{
	g_rtu_timeout = 1;
}

/*
*********************************************************************************************************
*	函 数 名: LED485_TestOk
*	功能说明: 测试数码管应答. 数码管会应答OK。 需要先切换到ASCII协议。
*	形    参: _addr : 从机的485地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_TestOk(uint8_t _addr)
{
	char buf[10];

	sprintf(buf, "$%03d,01&", _addr);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_ReadModel
*	功能说明: 读取数码管型号. 需要先切换到ASCII协议。 改命令只是发送指令，主程序根据 MSG_485_RX 消息获得
*			  结果
*	形    参: _addr : 从机的485地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_ReadModel(uint8_t _addr)
{
	char buf[10];

	sprintf(buf, "$%03d,02&", _addr);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_ReadVersion
*	功能说明: 读取数码管固件版本. 需要先切换到ASCII协议。 本函数只是发送指令，主程序根据 MSG_485_RX 消息获得
*			  结果
*	形    参: _addr : 从机的485地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_ReadVersion(uint8_t _addr)
{
	char buf[10];

	sprintf(buf, "$%03d,03&", _addr);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_ReadBright
*	功能说明: 读取数码管亮度参数。需要先切换到ASCII协议。 本函数只是发送指令，主程序根据 MSG_485_RX 消息获得
*			  结果
*	形    参: _addr : 从机的485地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_ReadBright(uint8_t _addr)
{
	char buf[10];

	sprintf(buf, "$%03d,04&", _addr);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_SetBrightA
*	功能说明: 设置LED数码管的亮度 （采用ASCII协议)
*	形    参: _addr : 从机的485地址
*			  _bright : 亮度值 0 - 7
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_SetBrightA(uint8_t _addr, uint8_t _bright)
{
	char buf[10];

	sprintf(buf, "$%03d,%d%%", _addr, _bright);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_ModifyAddrA
*	功能说明: 修改LED数码管的地址. （采用ASCII协议)
*	形    参: _addr : 从机的485地址
*			  _NewAddr : 新地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_ModifyAddrA(uint8_t _addr, uint8_t _NewAddr)
{
	char buf[10];

	sprintf(buf, "$%03d,%03d@", _addr, _NewAddr);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_DispNumberA
*	功能说明: 显示1个整数.  使用ASCII协议。
*	形    参: _addr : 从机的485地址
*			  _iNumber : 整数，负数用补码表示
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_DispNumberA(uint8_t _addr, int16_t _iNumber)
{
	char buf[16];

	sprintf(buf, "$%03d,%3d#", _addr, _iNumber);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_DispStrA
*	功能说明: 显示1个ASCII字符串； 使用ASCII协议。
*	形    参: _addr : 从机的485地址
*			  _iNumber : 整数，负数用补码表示
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_DispStrA(uint8_t _addr, char *_str)
{
	char buf[16];

	sprintf(buf, "$%03d,%s#", _addr, _str);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_SetProtRTU
*	功能说明: 设置从机的通信协议为 Modbus RTU
*	形    参: _addr : 从机的485地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_SetProtRTU(uint8_t _addr)
{
	char buf[10];

	sprintf(buf, "$%03d,81&", _addr);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_SetProtAscii
*	功能说明: 设置从机的通信协议为 ASCII
*	形    参: _addr : 从机的485地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_SetProtAscii(uint8_t _addr)
{
	char buf[10];

	sprintf(buf, "$%03d,80&", _addr);
	RS485_SendStr(buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_DispNumber
*	功能说明: 显示1个整数. 不等待应答
*	形    参: _addr : 从机的485地址
*			  _iNumber : 整数，负数用补码表示
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_DispNumber(uint8_t _addr, int16_t _iNumber)
{
	MODH_Send06H(_addr, 0x0088, _iNumber);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_SetDispDot
*	功能说明: 设置小数点位数
*	形    参: _addr : 从机的485地址
*			  _dot : 小数点位数。数码管上电后缺省是0，表示无小数点。
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_SetDispDot(uint8_t _addr, uint8_t _dot)
{
	MODH_Send06H(_addr, 0x0025, _dot);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_SetBright
*	功能说明: 设置LED数码管的亮度
*	形    参: _addr : 从机的485地址
*			  _bright : 亮度值 0 - 7
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_SetBright(uint8_t _addr, uint8_t _bright)
{
	MODH_Send06H(_addr, 0x0024, _bright);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_ModifyAddr
*	功能说明: 修改LED数码管的地址
*	形    参: _addr : 从机的485地址
*			  _NewAddr : 新地址
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_ModifyAddr(uint8_t _addr, uint8_t _NewAddr)
{
	MODH_Send06H(_addr, 0x0023, _NewAddr);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_DispNumberWithDot
*	功能说明: 显示带小数点的整数
*	形    参: _addr : 从机的485地址
*			  _iNumber : 要显示的整数
*			  _dot : 小数点位数
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_DispNumberWithDot(uint8_t _addr, int16_t _iNumber, uint8_t _dot)
{
/*
	PLC发送  :0110 00 90 00 02 04 00 0201 EA DB 1C
	?  01:   数码管屏的站号（RS485地址）
	?  10 :   功能码，表示写多个寄存器
	?  00 90 :   数码管屏的显示寄存器(带小数点和正负号的整数)
	?  00 02:  寄存器个数
	?  04:  数据个数（字节数）
	?  00 02：  00 表示正负号（00=正数；01=负数，数字前显示-）
				02 表示小数点位数，0表示无小数点。2表示小数点后有2位数字
	?  01 EA:   2位整数，高字节在前。01 EA表示十进制 490
	?  DB 1C  :   二个字节CRC码
	此命令将显示“4.90”
	数码管屏返回 ：01 10 00 90 00 02 41 E5
*/
	uint16_t buf[2];

	if (_iNumber < 0)			/* 显示正负号 */
	{
		buf[0] = 0x0100 | _dot;
	}
	else
	{
		buf[0] = 0x0000 | _dot;
	}
	if (_iNumber < 0)
	{
		_iNumber = -_iNumber;
	}
	buf[1] = _iNumber;
	
	MODH_Send10H(_addr,0x0090, 2, buf);
}

/*
*********************************************************************************************************
*	函 数 名: LED485_DispStr
*	功能说明: 显示字符串
*	形    参: _addr : 从机的485地址
*			  _str : 要显示的字符串
*	返 回 值: 无
*********************************************************************************************************
*/
void LED485_DispStr(uint8_t _addr, char *_str)
{
/*
	PLC发送  :
	0110 00 70 00 06 0C 50 32 2E 33 00 00 00 00 00 00 00 00 3B 25
	?  01:   数码管屏的站号（RS485地址）
	?  10 :   功能码，表示写多个寄存器
	?  0070 :   数码管屏的显示寄存器(ASCII)
	?  00 06:  寄存器个数
	?  0C:   数据段的字节数
	?  50 32 2E 33 00 00 00 00 00 00 00 00  :
	ASCII字符串。固定长度12字节，长度不足12位的字符串右边必须填00。本例
	表示ASCII字符串”P2.3”
	?  3B 25  :   二个字节CRC码
	此命令将显示“P2.3”
	数码管屏返回 ：01 10 00 70 00 06 41 D0
*/
	uint16_t buf[6];
	uint8_t i;

	for (i = 0; i < 6; i++)
	{
		buf[i] = 0;
	}
	
	for (i = 0; i < 12; i++)
	{
		if (_str[i] == 0)
		{
			break;
		}
		
		if (i % 2)
		{
			buf[i / 2] += _str[i] << 8;
		}
		else
		{
			buf[i / 2] = _str[i];
		}
	}
	
	MODH_Send10H(_addr, 0x0070, 2, buf);
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
