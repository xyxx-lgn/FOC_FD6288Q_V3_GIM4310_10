#include "My_Can.h"
#include "fdcan.h"

extern AllFlag allflag;                //标志位变量
extern CANFD_Message CanFD_Message;    //CAN通信消息结构体
extern Encoder_Struct encoder_str;     //编码器结构体
extern AdcValue adcvalue;              //ADC采样变量
extern PID pid_m1;                     //PID参数结构体
extern Motor_Factor motor_factor;      //电机参数结构体


FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_FilterTypeDef sFilter;
uint8_t CANFD_TxData[8];
uint8_t CANFD_RxData[8];
uint8_t Motor_Can_ID[6]={0x11,0x12,0x13,0x14,0x15,0x16};
uint8_t six_direction[6] = {1,0,0,1,0,1};
float Line_Planning_All_Points[150];
uint16_t Line_Planning_All_Speed[150];
uint16_t Line_Planning_All_Gap_Time[150];
uint16_t Line_Planning_nums = 0;
uint16_t Last_Line_Planning_nums = 0;  //记录Line_Planning_nums数目，确保接收到路径点的角度速度值
uint16_t now_nums = 0;
int Line_Planning_Flag = 0;
uint16_t Line_Planning_One_Time = 0;

void CANFD_Init_Config(void)  //电机关节4，只接受0x14数据
{
	//过滤器配置,一共有14个过滤器，根据需求开启
    /* ---------- 过滤器 0： ---------- */
    sFilter.IdType       = FDCAN_STANDARD_ID;         //使用标准帧
    sFilter.FilterIndex  = 0;                         // 第 0 个过滤器
    sFilter.FilterType   = FDCAN_FILTER_MASK;         // 掩码模式
    sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;   // 放进 FIFO0
    sFilter.FilterID1    = Motor_Can_ID[CanFD_Message.Motor_Select-1];   // 期望 ID  0x0F    000 0000 1111
    sFilter.FilterID2    = 0x7FF;                      // 掩码：           111 1111 1111
                                                      //掩码1对应的位置意味着要比对一致的，0代表通过

    /* ---------- 过滤器 1： ---------- */
//    sFilter.IdType       = FDCAN_STANDARD_ID;         //使用标准帧
//    sFilter.FilterIndex  = 1;                         // 第 1 个过滤器
//    sFilter.FilterType   = FDCAN_FILTER_MASK;         // 掩码模式
//    sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;   // 放进 FIFO0
//    sFilter.FilterID1    = 0x00F;                      // 期望 ID    
//    sFilter.FilterID2    = 0x7FF;                      // 掩码：

	/*
		注意：要使得过滤器生效一定要配置完过滤器和全局过滤器，缺失一个都没用
		HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);                //配置过滤器
		HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,                     //配置全局过滤器
							     FDCAN_REJECT,              // 拒绝不匹配的ID
                                 FDCAN_REJECT,              // 拒绝不匹配的ID
                                 FDCAN_FILTER_REJECT,       //拒绝所有标准远程帧(遥控帧)，
                                 FDCAN_FILTER_REJECT);      ////拒绝所有扩展远程帧(遥控帧)
	*/
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);                //配置过滤器
	
	/*
	第二三个参数取值：
		ID帧不匹配任何已配置过滤器时的处理方式：
		FDCAN_REJECT：拒绝接收
		FDCAN_ACCEPT_IN_RX_FIFO0：接受并存入RX FIFO0
		FDCAN_ACCEPT_IN_RX_FIFO1：接受并存入RX FIFO1
	第四五个参数取值：
		FDCAN_FILTER_REMOTE：不过滤，可被正常接收
		FDCAN_FILTER_REJECT：拒绝远程帧
	*/
	//配置全局过滤器
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
							     FDCAN_REJECT,              // 拒绝不匹配的ID
                                 FDCAN_REJECT,              // 拒绝不匹配的ID
                                 FDCAN_FILTER_REJECT,       //拒绝所有标准远程帧(遥控帧)，
                                 FDCAN_FILTER_REJECT);      ////拒绝所有扩展远程帧(遥控帧)
									 
    /* 使能 FIFO0 新消息中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	
	//开启CAN
	HAL_FDCAN_Start(&hfdcan1);
}

/********************
CANFD数据发送：
	目前是经典CAN
	参数：ID，指令类型，目标值，方向
********************/
uint8_t CANFD_Send(uint8_t motor_id,uint8_t command_type,uint32_t target_value,uint8_t direction,uint16_t rotate_speed)
{
	//配置发送信息
	TxHeader.Identifier = motor_id;                        //设置标准帧ID为电机ID,0-7FF,这里是8位数据，取0-FF够用了
	TxHeader.IdType = FDCAN_STANDARD_ID;				   //使用标准帧
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;               //数据帧
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;               //数据长度为8字节
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;       //主动错误状态
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;                //不启用比特率切换
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                 //经典CAN格式
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;      //不存储发送事件到事件FIFIO,默认不存储
	TxHeader.MessageMarker = 0x52;                         //消息标记,默认任意给值
	//TxHeader.TxEventFifoControl如果开启，会在发送中断里面将本帧时间戳等写入Tx Event FIFO，默认不存储
	//如果开启TxEventFifoControl，那么MessageMarker才有用，只是定义一个标签随意给值，方便让你知道是哪个有问题
	
	//发送数据填入
	CANFD_TxData[0] = command_type;                           //指令类型
	CANFD_TxData[1] = (target_value>>16)&0xFF;                //目标位置高字节  //目前只用了24位数据，有需要可以继续移位到32位
	CANFD_TxData[2] = (target_value>>8)&0xFF;                 //目标位置中字节 
	CANFD_TxData[3] = target_value&0xFF;                      //目标位置低字节
	CANFD_TxData[4] = direction;                          	  //方向
	CANFD_TxData[5] = (rotate_speed>>8)&0xFF;                 //预留高字节
	CANFD_TxData[6] = rotate_speed&0xFF;                      //预留低字节
	//CANFD_TxData[7] = 0x00;                                   //轨迹规划的轨迹点数
	
	//发送数据
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader,CANFD_TxData)!=HAL_OK) return 1;
	return 0;
}

/********************
CANFD数据接收：
	目前是经典CAN
********************/
void CANFD_ReceiveDate(uint8_t * recbuf,uint16_t Identifier,uint16_t len)
{
	//接收数据定义
	uint8_t command_type = 0;        //指令类型
	uint32_t target_value = 0;       //目标位置
	uint8_t direction = 0;           //方向
	float Control_Target = 0;        //目标位置(带方向)
	uint16_t rotate_speed = 0;       //旋转速度
	
	//检查数据长度
	if(len!=8) return ;
	
	//解析数据
	command_type = recbuf[0];
	target_value = (recbuf[1]<<16)|(recbuf[2]<<8)|recbuf[3];			//磁编码器数值大小
	direction = recbuf[4];
	rotate_speed = (recbuf[5]<<8)|recbuf[6];
	

	//方向处理
	if(direction==0) Control_Target = target_value;
	else if(direction==1) Control_Target = (-1.0f)*target_value;
	
	//将此编码器数值大小转化为设定角度值
	float factor = 360.0f/(16384.0f*motor_factor.motor_gear);  //0.002197265625f = 360.0f/(16384.0f*10.0f)
	Control_Target = Control_Target*factor;  //(Control_Target*360.0f)/(16384.0f*10.0f)
	
	CanFD_Message.CAN_ID = Identifier;
	CanFD_Message.direction = direction;
	CanFD_Message.Control_Target = Control_Target;
	CanFD_Message.command_type = command_type;
	CanFD_Message.rotate_speed = rotate_speed;
}

/********************
CANFD状态标志位检测：
********************/
void CAN_State(uint16_t Identifier)
{
	uint32_t target_value = 0;       //目标位置
	float temp_angle = 0;            //临时角度接收变量
	uint8_t Motor_ID = Motor_Can_ID[CanFD_Message.Motor_Select-1];
	//根据电机ID和指令类型处理数据
	if(Identifier == Motor_ID)
	{
			
			switch(CanFD_Message.command_type)
			{
				case 1:   //方向取反
					six_direction[CanFD_Message.Motor_Select-1]	= CanFD_Message.direction;             //保存新的方向设定
					break;
				case 2:   //指定某关节旋转角度(相对原来的角度增量)
					pid_m1.Position_aim = encoder_str.output_shaft_angle+CanFD_Message.Control_Target;
					pid_m1.target_speed = CanFD_Message.rotate_speed;
//					if(encoder_str.output_shaft_angle<pid_m1.Position_aim+0.01f && encoder_str.output_shaft_angle>pid_m1.Position_aim-0.01f)
//					{
//						CANFD_Send(Motor_ID+16,2,target_value,six_direction[CanFD_Message.Motor_Select-1],(uint16_t)pid_m1.target_speed);			
//					}
					break;
				case 3:   //设定某个轴的初始值(即设置轴的初始角度)
					CanFD_Message.Initial_Angle = encoder_str.output_shaft_angle-CanFD_Message.Control_Target;
					break;
				case 4:   //指定某关节进行绝对角度旋转，相对于校准后的初始角度
					pid_m1.Position_aim = CanFD_Message.Control_Target+CanFD_Message.Initial_Angle;
					pid_m1.target_speed = CanFD_Message.rotate_speed;
//					if(encoder_str.output_shaft_angle<pid_m1.Position_aim+0.01f && encoder_str.output_shaft_angle>pid_m1.Position_aim-0.01f)
//					{
//						CANFD_Send(Motor_ID+16,2,target_value,six_direction[CanFD_Message.Motor_Select-1],(uint16_t)pid_m1.target_speed);			
//					}
					break;
				case 5:   //返回当前电机角度(输出轴的角度，算上减速器后的角度，加上初始化校准角度)	
					temp_angle = encoder_str.output_shaft_angle-CanFD_Message.Initial_Angle;
					while(temp_angle>360.0f) temp_angle -= 360.0f;
					while(temp_angle<0.0f) temp_angle += 360.0f;
					target_value = (uint32_t)((temp_angle * motor_factor.motor_gear * 16384.0f) / 360.0f);
				
					CANFD_Send(Motor_ID+16,5,target_value,six_direction[CanFD_Message.Motor_Select-1],(uint16_t)pid_m1.target_speed);            //发送当前角度给控制器
					break;
				case 6:  //直线规划前清0操作
					Line_Planning_nums = 0;
					Line_Planning_Flag = 0;
					CANFD_Send(Motor_ID+16,6,0,six_direction[CanFD_Message.Motor_Select-1],0);//发送指令6，确保已清零
					break;
				case 7:  //存储路径规划的所有目标值以及速度值，由主控下发
					if(Line_Planning_nums == Last_Line_Planning_nums-1)
					{
						Line_Planning_All_Points[Line_Planning_nums] = CanFD_Message.Control_Target+CanFD_Message.Initial_Angle;
						Line_Planning_All_Speed[Line_Planning_nums] = CanFD_Message.rotate_speed / 6; //将度/秒换成转/分
						Line_Planning_nums++;
					}
					break;
				case 8: //直线路径规划启动指令
					Line_Planning_Flag = 1;
				
					/*  下面这段开始执行前的时间间隔赋值，是为了之前在直线路径规划使用，现在不用主控指令13，15了，如需使用，得去主控端修改代码*/
					// 主控在 CAN_Send_Motor() 里把 rotate_speed 乘了减速比发送，
					// 这里必须按当前电机减速比还原回真实时间(ms)
					Line_Planning_One_Time = CanFD_Message.rotate_speed/motor_factor.motor_gear;   //将速度接收位换成时间,单位：ms
					if (Line_Planning_One_Time < 1) Line_Planning_One_Time = 1;
					Line_Planning_One_Time = 8*Line_Planning_One_Time;     //定时器计次次数
				
					//接收到后给主机发送反馈10
					CANFD_Send(Motor_ID+16,8,0,six_direction[CanFD_Message.Motor_Select-1],0);
					break;
				case 9: //直线路径规划完成指令(当到达完成时间后，主控仍未接收完成指令，就会发送指令9查询)
					if(Line_Planning_Flag == 0)
						CANFD_Send(Motor_ID+16,9,Line_Planning_nums,six_direction[CanFD_Message.Motor_Select-1],0);//发送指令9，确保已到达终点
					break;
				case 10: //接收判断指令
					CANFD_TxData[7] = Line_Planning_nums;
					Last_Line_Planning_nums = CANFD_RxData[7];                               //轨迹规划点数
					if(Line_Planning_nums == Last_Line_Planning_nums && Line_Planning_nums>0)   //接收到点时再进入
					{
						Line_Planning_All_Gap_Time[Line_Planning_nums-1] = (CanFD_Message.rotate_speed/motor_factor.motor_gear)*8;
						if(Line_Planning_All_Gap_Time[Line_Planning_nums-1]<8) Line_Planning_All_Gap_Time[Line_Planning_nums-1] = 8;  //小于1ms时，赋值为1ms
						CANFD_Send(Motor_ID+16,10,0,six_direction[CanFD_Message.Motor_Select-1],0);//发送指令10，确保已接收
					}
					else
					{
						CANFD_Send(Motor_ID+16,11,0,six_direction[CanFD_Message.Motor_Select-1],0);
					}	
//					else if(Line_Planning_nums == Last_Line_Planning_nums-1)	//未接收到指令7，只接收到了指令10
//					{
//						CANFD_Send(Motor_ID+16,10,0,six_direction[CanFD_Message.Motor_Select-1],0);//让速度值大于0，让主机重新发送一次点位
//					}
					break;
				case 12: //启用重力补偿模式
					pid_m1.i_zl_flag = 1;
					pid_m1.i_zl_qk  = CanFD_Message.Control_Target;   //接收前馈重力补偿值
					break;
				case 13://关闭重力补偿
					pid_m1.i_zl_flag = 0;
					pid_m1.i_zl_qk = 0.0f;
					break;
				default:
					break;
			}
			CanFD_Message.command_type = 0;  //避免一直发送占用总线
			RxHeader.Identifier = 0x01;
	}
}


//FIFO0新消息接收中断
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)  //如果是FIFO0新消息
    {

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CANFD_RxData) == HAL_OK)
        {
            /* 只处理标准帧且长度为 8 */
            if (RxHeader.IdType == FDCAN_STANDARD_ID && RxHeader.DataLength == FDCAN_DLC_BYTES_8)   //电机关节4
            {
				//可以在这进行ID判断，符合的接收，但是用JLink看看不出来，因为在外面用CANFD_RxData接收了，可以自己定义一个变量看
                CANFD_ReceiveDate(CANFD_RxData, RxHeader.Identifier, 8);
            }
        }
    }
}


//float test_angle[10] = {0.0f,1.3f,2.7f,3.45f,5.11f,5.95f,7.35f,8.93f,9.45f,12.11f}; 
//float test_angle[30] = {0.0f,3.5f,7.2f,9.05f,13.3f,16.72f,19.02f,19.87f,21.12f,22.89f,25.35f,26.88f,28.74f,30.6f,  32.4f,  34.2f,
//    36.0f,  37.8f,  39.6f,  41.4f,  43.2f,  45.0f,  46.8f,  48.6f,  50.4f,52.12f,54.7f,56.21f,59.33f,60.3f}; 
//float time_cnt = 800;
//uint8_t test_flag = 0;
//uint8_t test_index = 0;
//float tttttt_angle = 0.05f;
//void test_angle_1()
//{
//	static uint16_t count_cnts = 0;
//	if(test_flag == 1)
//	{
//		count_cnts++;
//		if(count_cnts>time_cnt || fabs(pid_m1.Position_aim - encoder_str.output_shaft_angle)<tttttt_angle)
//		{
//			count_cnts = 0;
//			if(test_index<10)
//			{
//				pid_m1.Position_aim = test_angle[test_index];
//				test_index++;
//			}
//			else
//			{
//				test_flag = 3;
//			}
//			
//		}
//	}
//	else if(test_flag ==2)
//	{
//		count_cnts++;
//		if(count_cnts>time_cnt || fabs(pid_m1.Position_aim - encoder_str.output_shaft_angle)<tttttt_angle)
//		{
//			count_cnts = 0;
//			if(test_index>0)
//			{
//				test_index --;
//				pid_m1.Position_aim = test_angle[test_index];
//			}
//			else
//			{
//				test_flag = 3;
//			}
//		}
//	}
//}


//// 在错误回调中增加总线恢复逻辑
//void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
//{
//  // 简单恢复：任何错误都重启CAN
//  HAL_FDCAN_Stop(hfdcan);
//  HAL_Delay(10);
//  HAL_FDCAN_Start(hfdcan);
//  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//}

uint8_t erro_flag;
uint16_t cnt = 0;

//Timer7的8KHz定时中断，用于CAN接收解析，125us
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t i=0;
  if (htim->Instance == TIM7)          // 125us 到
  { 
	  CAN_State(RxHeader.Identifier);
	  i++;
//	  test_angle_1();
	  if(Line_Planning_Flag == 1)
	  {
			if(cnt<Line_Planning_nums)
			{
			  if((i>Line_Planning_All_Gap_Time[cnt])||(encoder_str.output_shaft_angle<pid_m1.Position_aim+0.25f && encoder_str.output_shaft_angle>pid_m1.Position_aim-0.25f)) //提前5ms发送
			  {
				i = 0;
				if(cnt  < Line_Planning_nums)
				{
					pid_m1.Position_aim = Line_Planning_All_Points[cnt];
					pid_m1.target_speed = Line_Planning_All_Speed[cnt]; 
					cnt++;
				}
			  }
			}
			if((cnt == Line_Planning_nums) && (encoder_str.output_shaft_angle<pid_m1.Position_aim+0.08f && encoder_str.output_shaft_angle>pid_m1.Position_aim-0.08f))
			{
				Line_Planning_Flag = 0;
				cnt = 0;
				CANFD_Send(Motor_Can_ID[CanFD_Message.Motor_Select-1]+16,9,Line_Planning_nums,six_direction[CanFD_Message.Motor_Select-1],0);//发送指令9，确保已到达终点
			}
		  
	 }
	  i%=60000;    //避免i疯狂累加
//	  if(i>=4)   //1K发送频率
//	  {
////		  allflag.CANFD_Tx_flag=CANFD_Send(0x21,3,encoder_str.Shaft_Angle,0,0);  //0代表发送成功，1代表发送失败
//		  if (allflag.CANFD_Tx_flag == 0);
//		  else
//		  {
//			erro_flag++;
//			HAL_FDCAN_Stop(&hfdcan1);
//		  }
//		  if(erro_flag>6)
//		  {
//			erro_flag=0;  
//			CANFD_Init_Config();
//		  }
//	
//		  i=0;
//	  }
  }
}
