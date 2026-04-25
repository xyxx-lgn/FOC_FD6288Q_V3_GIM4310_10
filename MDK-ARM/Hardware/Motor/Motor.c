#include "Motor.h"
#include "MT6701.h"
#include "FOC.h"
#include "PID.h"
#include "fdcan.h"

extern uint16_t ADC1InjectDate[4];     //注入组采样数组
extern float Fir_Output[4];            //Fir滤波后的值

extern AdcValue adcvalue;              //ADC采样变量
extern AllFlag allflag;                //标志位变量
extern Encoder_Struct encoder_str;     //编码器结构体
extern SVPWM_Struct svpwm_str;         //svpwm结构体
extern PID pid_m1;                     //PID参数结构体
extern CANFD_Message CanFD_Message;    //CAN通信消息结构体
extern Motor_Factor motor_factor;      //电机参数结构体

float theta=0,Iwidth=150;  //电流环带宽测试变量，不用时屏蔽

/*
函数功能：
	电机参数选择，先执行Data_Init(void),再执行Motor_Seletct，该函数会覆盖Data_Init取值
	电机需要重新调试时，请注释该函数,或者参数给0
	参数：
		uint8_t motor_select  电机选择，1-6表示选择电机1-6，0无意义;
*/
uint8_t Motor_Seletct(uint8_t motor_select)
{
	//电机123的电流环带宽取150，电机456带宽取350 
	if(motor_select == 0)
		return 0;
	else if(motor_select == 1)//GIM8108-8
	{
		//电机基本参数
		motor_factor.motor_gear=8.0f;         	 //电机减速比
		motor_factor.motor_phaseL=0.00045f;      //电机相电感
		motor_factor.motor_phaseR=0.67f;         //电机相电阻
		motor_factor.motor_sampleR=0.002;        //电机采样电阻
		motor_factor.motor_pole=21;              //电机极对数
		
		//电流增益倍数 Gain*Rs=50*0.005=0.25
		adcvalue.Gain_I=50.0f*motor_factor.motor_sampleR;  
		
		//零偏校准值(需要重新校准值，请赋值为0)
		encoder_str.Zero_Angle=4.24072f;     //6.10839f
		
		//电流采样是否错误，0表示正常，123分别代表ABC相不正常，切换两相采样
		allflag.CurrentABCErro_flag=0;           //A相采样值不正常
		
		//母线电压系数     
		motor_factor.udc_factor=0.011279296f;   
		
		//三环PID参数
		//Kp_i=相电感*350*2*pi=0.000042*330*2*pi=0.0870848f  
		//Ki_i=相电阻*350*2*pi/电流环执行频率=0.08*330*2*pi/20000=0.008293758f   
		pid_m1.Kp_i=motor_factor.motor_phaseL*2.0f*M_PI*150.0f;           //0.4241149f(150) 0.2827433f(100)
		pid_m1.Ki_i=motor_factor.motor_phaseR*2.0f*M_PI*150.0f/20000.0f;  //0.0315729f(150) 0.0210486f(100)
		
		pid_m1.Kp_speed=0.05f;      //速度环Kp值    
		pid_m1.Ki_speed=0.00018f;    //速度环Ki值  
		
		pid_m1.Kp_position=30.0f;   //位置环Kp值
	}
	else if(motor_select == 2)//GIM4310-36
	{
		//电机基本参数
		motor_factor.motor_gear=36.0f;         	 //电机减速比
		motor_factor.motor_phaseL=0.00075f;      //电机相电感
		motor_factor.motor_phaseR=1.89f;         //电机相电阻
		motor_factor.motor_sampleR=0.005;        //电机采样电阻
		motor_factor.motor_pole=14;              //电机极对数
		
		//电流增益倍数 Gain*Rs=50*0.005=0.25
		adcvalue.Gain_I=50.0f*motor_factor.motor_sampleR;  
		
		//零偏校准值(需要重新校准值，请赋值为0)
		encoder_str.Zero_Angle=26.2792f;
		
		//电流采样是否错误，0表示正常，123分别代表ABC相不正常，切换两相采样
		allflag.CurrentABCErro_flag=0;           //A相采样值不正常
		
		//母线电压系数  
		motor_factor.udc_factor=0.0088623f; 
		
		//三环PID参数
		//Kp_i=相电感*350*2*pi=0.000042*330*2*pi=0.0870848f  
		//Ki_i=相电阻*350*2*pi/电流环执行频率=0.08*330*2*pi/20000=0.008293758f   
		pid_m1.Kp_i=motor_factor.motor_phaseL*2.0f*M_PI*150.0f;           //0.7068583f     
		pid_m1.Ki_i=motor_factor.motor_phaseR*2.0f*M_PI*150.0f/20000.0f;  //0.0890641f    
		
		pid_m1.Kp_speed=0.01f;      //速度环Kp值    
		pid_m1.Ki_speed=0.0001f;    //速度环Ki值  
		
		pid_m1.Kp_position=25.0f;   //位置环Kp值
	}
	else if(motor_select == 3)//GIM4310-36
	{
		//电机基本参数
		motor_factor.motor_gear=36.0f;         	 //电机减速比
		motor_factor.motor_phaseL=0.00075f;      //电机相电感
		motor_factor.motor_phaseR=1.89f;         //电机相电阻
		motor_factor.motor_sampleR=0.005;        //电机采样电阻
		motor_factor.motor_pole=14;              //电机极对数
		
		//电流增益倍数 Gain*Rs=50*0.005=0.25
		adcvalue.Gain_I=50.0f*motor_factor.motor_sampleR;  
		
		//零偏校准值(需要重新校准值，请赋值为0)
		encoder_str.Zero_Angle=6.9873f;
		
		//电流采样是否错误，0表示正常，123分别代表ABC相不正常，切换两相采样
		allflag.CurrentABCErro_flag=0;           //A相采样值不正常
				
		//母线电压系数  
		motor_factor.udc_factor=0.0088623f; 
		
		//三环PID参数
		//Kp_i=相电感*350*2*pi=0.000042*330*2*pi=0.0870848f  
		//Ki_i=相电阻*350*2*pi/电流环执行频率=0.08*330*2*pi/20000=0.008293758f   
		pid_m1.Kp_i=motor_factor.motor_phaseL*2.0f*M_PI*150.0f;           //0.7068583f     
		pid_m1.Ki_i=motor_factor.motor_phaseR*2.0f*M_PI*150.0f/20000.0f;  //0.0890641f    
		
		pid_m1.Kp_speed=0.01f;      //速度环Kp值    
		pid_m1.Ki_speed=0.0001f;    //速度环Ki值  
		
		pid_m1.Kp_position=25.0f;   //位置环Kp值
	}
	else if(motor_select == 4)//GIM4310-10
	{
		//电机基本参数
		motor_factor.motor_gear=10.0f;         	 //电机减速比
		motor_factor.motor_phaseL=0.000344f;      //电机相电感
		motor_factor.motor_phaseR=1.046f;         //电机相电阻
		motor_factor.motor_sampleR=0.005;        //电机采样电阻
		motor_factor.motor_pole=14;              //电机极对数
		
		//电流增益倍数 Gain*Rs=50*0.005=0.25
		adcvalue.Gain_I=50.0f*motor_factor.motor_sampleR;  
		
		//零偏校准值(需要重新校准值，请赋值为0)
		encoder_str.Zero_Angle=15.6006f;
		
		//电流采样是否错误，0表示正常，123分别代表ABC相不正常，切换两相采样
		allflag.CurrentABCErro_flag=0;           //A相采样值不正常
						
		//母线电压系数  
		motor_factor.udc_factor=0.0088623f; 
		
		//三环PID参数
		//Kp_i=相电感*350*2*pi=0.000042*330*2*pi=0.0870848f  
		//Ki_i=相电阻*350*2*pi/电流环执行频率=0.08*330*2*pi/20000=0.008293758f   
		pid_m1.Kp_i=motor_factor.motor_phaseL*2.0f*M_PI*350.0f;           //0.7564955f    
		pid_m1.Ki_i=motor_factor.motor_phaseR*2.0f*M_PI*350.0f/20000.0f;  //0.1150137f   
		
		pid_m1.Kp_speed=0.005f;      //速度环Kp值    
		pid_m1.Ki_speed=0.0001f;    //速度环Ki值  
		
		pid_m1.Kp_position=25.0f;   //位置环Kp值
	}
	else if(motor_select == 5)//GIM4310-10
	{
		//电机基本参数
		motor_factor.motor_gear=10.0f;         	 //电机减速比
		motor_factor.motor_phaseL=0.000344f;      //电机相电感
		motor_factor.motor_phaseR=1.046f;         //电机相电阻
		motor_factor.motor_sampleR=0.005;        //电机采样电阻
		motor_factor.motor_pole=14;              //电机极对数
		
		//电流增益倍数 Gain*Rs=50*0.005=0.25
		adcvalue.Gain_I=50.0f*motor_factor.motor_sampleR;  
		
		//零偏校准值(需要重新校准值，请赋值为0)
		encoder_str.Zero_Angle=17.3584f;
		
		//电流采样是否错误，0表示正常，123分别代表ABC相不正常，切换两相采样
		allflag.CurrentABCErro_flag=0;           //A相采样值不正常
						
		//母线电压系数  
		motor_factor.udc_factor=0.0088623f; 
		
		//三环PID参数
		//Kp_i=相电感*350*2*pi=0.000042*330*2*pi=0.0870848f  
		//Ki_i=相电阻*350*2*pi/电流环执行频率=0.08*330*2*pi/20000=0.008293758f   
		pid_m1.Kp_i=motor_factor.motor_phaseL*2.0f*M_PI*350.0f;           //0.7564955f    
		pid_m1.Ki_i=motor_factor.motor_phaseR*2.0f*M_PI*350.0f/20000.0f;  //0.1150137f   
		
		pid_m1.Kp_speed=0.005f;      //速度环Kp值    
		pid_m1.Ki_speed=0.0001f;    //速度环Ki值  
		
		pid_m1.Kp_position=25.0f;   //位置环Kp值
	}
	else if(motor_select == 6)//GIM4305-10
	{
		//电机基本参数
		motor_factor.motor_gear=10.0f;         	 //电机减速比
		motor_factor.motor_phaseL=0.000169f;      //电机相电感
		motor_factor.motor_phaseR=0.638f;         //电机相电阻
		motor_factor.motor_sampleR=0.005;        //电机采样电阻
		motor_factor.motor_pole=14;              //电机极对数
		
		//电流增益倍数 Gain*Rs=50*0.005=0.25
		adcvalue.Gain_I=50.0f*motor_factor.motor_sampleR;  
		
		//零偏校准值(需要重新校准值，请赋值为0)
		encoder_str.Zero_Angle=12.1289f;
		
		//电流采样是否错误，0表示正常，123分别代表ABC相不正常，切换两相采样
		allflag.CurrentABCErro_flag=0;           //A相采样值不正常
						
		//母线电压系数  
		motor_factor.udc_factor=0.0088623f; 
		
		//三环PID参数
		//Kp_i=相电感*350*2*pi=0.000042*330*2*pi=0.0870848f  
		//Ki_i=相电阻*350*2*pi/电流环执行频率=0.08*330*2*pi/20000=0.008293758f   
		pid_m1.Kp_i=motor_factor.motor_phaseL*2.0f*M_PI*350.0f;           //0.3716504f    
		pid_m1.Ki_i=motor_factor.motor_phaseR*2.0f*M_PI*350.0f/20000.0f;  //0.0701517f   
		
		pid_m1.Kp_speed=0.005f;      //速度环Kp值    
		pid_m1.Ki_speed=0.0001f;    //速度环Ki值  
		
		pid_m1.Kp_position=25.0f;   //位置环Kp值
	}
	
	pid_m1.target_speed = 10;    //总体赋值一个初速度，使得模式6能回归0点
	
	return 1; 
}
	
void Data_Init(void)
{
	//电机参数结构体
	motor_factor.motor_gear=8.0f;         	 //电机减速比
	motor_factor.motor_phaseL=0.000344f;     //电机相电感
	motor_factor.motor_phaseR=1.046f;        //电机相电阻
	motor_factor.motor_sampleR=0.005;        //电机采样电阻
	motor_factor.motor_pole=14;              //电机极对数
	motor_factor.udc_factor=0.011279296f;    //母线电压系数  0.0088623f
	
	//adcvalue结构体初始化
	adcvalue.supply_Udc=24.0f;           //输入电压值最大值
	adcvalue.supply_IMax=5.0f;           //电流最大值    (3.3/2) =1.65  1.65/50/Rs = 1.65/50/0.05 = 6.6A相电流采样最大值
	adcvalue.Ia_Sample=0.0f;
	adcvalue.Ib_Sample=0.0f;
	adcvalue.Ic_Sample=0.0f;
	adcvalue.Udc_Sample=0.0f;           //母线电压采样值
	adcvalue.Ia_offect=1.65f;
	adcvalue.Ib_offect=1.65f;
	adcvalue.Ic_offect=1.65f;
	adcvalue.Ia=0.0f;
	adcvalue.Ib=0.0f;
	adcvalue.Ic=0.0f;
	adcvalue.Udc=0.0f;
	adcvalue.Iadc_count=0;
	adcvalue.Gain_I=50.0f*motor_factor.motor_sampleR;        //电流增益倍数 Gain*Rs=50*0.005=0.25
	

	
	//allflag结构体初始化
	allflag.Erro_flag=0;          //错误化标志位，0代表正常，1代表电压异常,2代表电流超限
	allflag.Adc_Adjust_flag=0;    //Adc校准标志位，0代表未校准，1代表校准完成，2代表过流
	allflag.Zero_flag=1;          //编码器零点校准标志位,0代表未校准，1代表校准完成  
	allflag.Encoder_flag=0;       //编码器模式 1.编码器开环控制 2.编码器闭环控制
	allflag.Mode_flag=0;          //电机控制模式选择
	allflag.Erro_counts=0;        //错误计次
	allflag.CurrentABCErro_flag=0;//ABC相某相电流采样有问题，0表示ABC三相无问题，1表示A相有问题，2，3表示B，C有问题

	
	//编码器结构体
	encoder_str.motordir=1;               //电机方向 取值{0,1}
	encoder_str.r_s_speed=0;              //开环设置的转速，单位：圈/秒(指的是机械角度)
	encoder_str.Encoder=0;  		      //编码器原始值
	encoder_str.Encoder_raw=0;  		  //编码器修正原始值
	encoder_str.Encoder_old_raw=0;        //上一次编码器原始值
	encoder_str.Encoder_raw_erro=0;       //编码器误差值
	encoder_str.Encoder_raw_sum=0;        //编码器原始值和
	encoder_str.Shaft_Angle=0.0f;         //机械角度
	encoder_str.Elect_Angle=0.0f;         //电角度
	encoder_str.Encoder_Mode1_Angle=0.0f; //模式1临时角度变量
	encoder_str.Encoder_Mode3_Angle=10.0f;//模式3临时角度变量
	encoder_str.Return_Rad=0.0f;          //真实返回角度,归一化到[0-2PI]
	encoder_str.Return_Angle=0.0f;        //真实返回角度(输出给SVPWM),归一化到[-PI-PI]
	encoder_str.Zero_Angle_cal=0.0f;      //零点偏移角度计算中间变量
	encoder_str.Zero_Angle=0.0f;          //零点偏移角度
	encoder_str.zero_count=0;             //编码器方向校准和零点校准
	encoder_str.rotation_counts=0;        //电机轴旋转圈数计次
	encoder_str.last_shaft_angle=0.0f;    //上一次电机轴角度，用于判断圈数
	encoder_str.shaft_total_angle=0.0f;   //电机轴累计角度
	encoder_str.output_shaft_angle=0.0f;  //输出轴绝对角度
	
	//svpwm_str结构体初始化
	svpwm_str.Udc=0.0f;
	svpwm_str.U1=0.0f;
	svpwm_str.U2=0.0f;
	svpwm_str.U3=0.0f;
	svpwm_str.A=0;
	svpwm_str.B=0;
	svpwm_str.C=0;
	svpwm_str.N=0;
	svpwm_str.Sector=0;
	svpwm_str.X=0.0f;
	svpwm_str.Y=0.0f;
	svpwm_str.Z=0.0f;
	svpwm_str.Ts=4200;      //PWM寄存器值
	svpwm_str.T4=0.0f;
	svpwm_str.T6=0.0f;
	svpwm_str.T4_temp=0.0f;
	svpwm_str.T6_temp=0.0f;
	svpwm_str.Ta=0.0f;
	svpwm_str.Tb=0.0f;
	svpwm_str.Tc=0.0f;
	svpwm_str.PWMA=0;
	svpwm_str.PWMB=0;
	svpwm_str.PWMC=0;
	svpwm_str.sin_dsp=0.0f;
	svpwm_str.cos_dsp=0.0f;
	
	/*
		GIM4305-10 Rs=0.638欧姆  L=0.000169亨	Kp = 0.000169*2*PI*350 = 0.3716504f   Ki = 0.0701517f	
		GIM4310-10 Rs=1.046欧姆  L=0.000344亨	Kp = 0.000344*2*PI*350 = 0.7564955f   Ki = 0.1150137f
		GIM4310-36 Rs=1.89欧姆   L=0.00075亨    Kp = 0.00075*2*PI*350 = 1.6493361f    Ki = 0.2078163f
		GIM8108-8  Rs=0.67欧姆   L=0.00045亨    Kp = 0.00045*2*PI*350 = 0.9896016f    Ki = 0.0736703f
	*/
	//PID结构体
	//下面为电流环参数 //带宽150                         //Iwidth=0.35/Tr
	pid_m1.Kp_i=0.7068583f;            //iq的Kp值        相电感*350*2*pi=0.000042*330*2*pi=0.0870848f   里面的350是指电角度多少转每秒，换算为350/7*60=3000转每分(机械角度)
	pid_m1.Ki_i=0.0890641f;         //iq的Ki值        相电阻*350*2*pi/电流环执行频率=0.08*330*2*pi/20000=0.008293758f   

	pid_m1.Iq_aim=0.0f;        //iq目标值          //GIM4310-10 额定电流3.5A，堵转电流15A
	pid_m1.Iq_current=0.0f;    //iq实际值
	pid_m1.erro_iq=0.0f;       //PI控制器里面的误差，等于目标值-实际值
	pid_m1.erro_iq_sum=0.0f;   //Ki的积分项
	pid_m1.Uq=0.0f;            //输出的Uq值
	
	pid_m1.Ki_SumMax=0.0f;     //KI的积分限幅最大值  Udc/sqrt(3),一般再乘以0.9,但本工程出问题，具体看pid_m1.speed_out_max处注释
	pid_m1.Ialfa=0.0f;         //电流环帕克变换
	pid_m1.Ibeta=0.0f;
	pid_m1.Vmax=0.0f;          //电流环输出最大电压矢量
	pid_m1.Iqd_MAX=3.5f;       //Iq,Id设定最大值

	pid_m1.Id_aim=0.0f;        //id目标值
	pid_m1.Id_current=0.0f;    //id实际值
	pid_m1.Iq_current_OK=0.0f;
	pid_m1.erro_id=0.0f;       //PI控制器里面的误差，等于目标值-实际值
	pid_m1.erro_id_sum=0.0f;   //Ki的积分项
	pid_m1.Ud=0.0f;            //输出的Ud值
	
	/*
	L6:Kp = ; Ki = 
	*/
	//速度环参数   //带宽约为35HZ       //GIM4310-10  额定转速100*10 最大转速330*10
	pid_m1.Kp_speed=0.01f;      //速度环Kp值       0.005   0.002
	pid_m1.Ki_speed=0.0001f;      //速度环Ki值       0.0001   0.0001
	
	pid_m1.Speed_aim=0.0f;     //目标速度，单位：转/分 
	pid_m1.Speed_last=0.0f;    //实际速度
	pid_m1.Speed_now=0.0f;     //实际速度
	pid_m1.erro_speed=0.0f;    //速度误差
	pid_m1.erro_speed_sum=0.0f;//速度积分误差
	pid_m1.speed_out=0.0f;     //PI输出结果，输出为电流环Iq_aim
	
	pid_m1.speed_out_max=3.5f; //速度环输出Iq限幅值   Imax=(3.3/2)/(G*S)=1.65/0.8=2.06, 一般取0.9，但是测试发现GVDD欠压有问题，可能电容原因，只能取1A，因此要乘系数为1/2.06
	pid_m1.speed_max=2000;      //速度设定限幅  [-360,360]
	pid_m1.speed_count=0;      //速度环计次
	
	/*
	L1: Kp = 35.0f L6: Kp = 25.0f
	*/
	//下面为位置环  //带宽大约2.6HZ
	pid_m1.Kp_position=25.0f;        //位置环Kp值    9.2
	pid_m1.Ki_position=0.0f;        //位置环Ki值 
	
	pid_m1.Position_aim=0.0f;       //目标位置，取值[0,360]
	pid_m1.erro_positon=0.0f;       //位置误差=Position_aim-Shaft_Angle
	pid_m1.erro_position_sum=0.0f;  //位置积分误差 
	pid_m1.position_out=0.0f;       //位置环PI输出，输出为速度环的Speed_aim  
	pid_m1.position_count=0;        //位置环计次
	
	pid_m1.position_out_max=1000.0f;   //位置环的输出限幅
	
	pid_m1.i_zl_flag = 0;             //重力补偿标志位,1为启用重力补偿
	pid_m1.i_zl_qk = 0.0f;            //重力补偿
}

/********************
限幅函数，用于浮点数限制，一般用于角度弧度限制 
	超限一般加减一个周期
********************/
float Angle_Limit(float raw,float Limit)
{
	while(raw>Limit)
		raw -= Limit;
	while(raw<0)
		raw += Limit;
	return raw;
}
/********************
限幅函数，用于Cordic库角度限制 [-PI,PI]
********************/
float Coridc_Angle(float angle) 
{
    while (angle >= M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}


/********************
ADC数据采样处理
	电流计算公式
		Vo=1/2Vref-G*V(Sn-Sp)
		Vrs=V(Sn-Sp)
		即-(Vo-1/2Vref)/G=Vrs
		因此I=Vrs/Rs(采样电阻值)
		I=-(Vo-1/2Vref)/(G*Rs)   ,此处的Vref最好是先校准，一般不是1.65V
********************/
void Adc_Task(AdcValue *adcvalue,AllFlag *allflag,uint16_t *adc_raw)
{
	adcvalue->Ia_Sample = adc_raw[0];  //A相adc采样值
	adcvalue->Ib_Sample = adc_raw[1];  //B相adc采样值
	adcvalue->Ic_Sample = adc_raw[2];  //C相adc采样值
	
	adcvalue->Udc_Sample = ADC1InjectDate[3]; //Udc电压adc采样值
	//0.0088623=1/4096*3.3 * (2+20)/2   GIM4310-10/36     0.0064243=1/4096*3.3*(34.8+4.99)/4.99  3508   0.011279296=1/4096*3.3*(39+3)/3   GIM8108-8
	adcvalue->Udc = (adcvalue->Udc_Sample-0)*motor_factor.udc_factor;   //母线电压采样的一阶低通滤波的截止频率一般设置为采样周期的[1/20，1/10]  0.00642432326f
	

	//1.对电流ADC偏置值校准
	if(allflag->Adc_Adjust_flag==0 && allflag->Erro_flag==0)    //电流adc未校准
	{
		if(adcvalue->Iadc_count<24000)  //PWM频率20K，进行12000次，用时0.6s
		{
			if(adcvalue->Iadc_count>22000)
			{
				adcvalue->Ia = adcvalue->Ia_Sample*3.3f/4096.0f;
				adcvalue->Ib = adcvalue->Ib_Sample*3.3f/4096.0f;
				adcvalue->Ic = adcvalue->Ic_Sample*3.3f/4096.0f;
				
				adcvalue->Ia_offect = adcvalue->Ia_offect*0.99f+adcvalue->Ia*0.01f;      //一阶滤波
				adcvalue->Ib_offect = adcvalue->Ib_offect*0.99f+adcvalue->Ib*0.01f;
				adcvalue->Ic_offect = adcvalue->Ic_offect*0.99f+adcvalue->Ic*0.01f;
			}
			adcvalue->Iadc_count++;
		}
		else
		{
			allflag->Adc_Adjust_flag = 1;     //校准完成
			adcvalue->Iadc_count = 0;
		}
	}
	else if(allflag->Adc_Adjust_flag==1)
	{   //ADC系数转换 
		//由于FD6288电流方向不一样，所以取消电流值前的符号，否则电流环的PI参数改为负值
		adcvalue->Ia = ((adcvalue->Ia_Sample*3.3f/4096.0f-adcvalue->Ia_offect)/adcvalue->Gain_I);    
		adcvalue->Ib = ((adcvalue->Ib_Sample*3.3f/4096.0f-adcvalue->Ib_offect)/adcvalue->Gain_I);
		adcvalue->Ic = ((adcvalue->Ic_Sample*3.3f/4096.0f-adcvalue->Ic_offect)/adcvalue->Gain_I);   //三电阻采样
//		adcvalue->Ib = -(adcvalue->Ia+adcvalue->Ic);                                                 //双电阻采样
		//当某相电流采样有问题时，使用双电阻采样,这部分可以后面代码优化，我先用标志位选择
//		allflag->CurrentABCErro_flag=0;
		if(allflag->CurrentABCErro_flag==1)
			adcvalue->Ia = -(adcvalue->Ib+adcvalue->Ic); 
		else if(allflag->CurrentABCErro_flag==2)
			adcvalue->Ib = -(adcvalue->Ia+adcvalue->Ic); 
		else if(allflag->CurrentABCErro_flag==3)
			adcvalue->Ic = -(adcvalue->Ia+adcvalue->Ib); 
		
        if(adcvalue->Udc>adcvalue->supply_Udc+1.0f)   //防止意外超出
			adcvalue->Udc=adcvalue->supply_Udc;
		svpwm_str.Udc = adcvalue->Udc*0.57735027f;
		pid_m1.Ki_SumMax = adcvalue->Udc*one_sqrt3*0.461880216f;    //1/sqrt(3)*0.8
	}
	
	//2.过压或欠压报警 //不能再重新校准ABC三相的偏置值了，否则三相电流值会相差很大
	if(adcvalue->Udc>(adcvalue->supply_Udc+1.0f) || adcvalue->Udc<(adcvalue->supply_Udc-1.0f)) //电压预警
	{
		allflag->Erro_flag = 1;
	}
	else if(adcvalue->Ia>adcvalue->supply_IMax || adcvalue->Ic>adcvalue->supply_IMax || adcvalue->Ic>adcvalue->supply_IMax) //电流预警
	{
		allflag->Erro_counts++;
		if(allflag->Erro_counts>4000)  //200ms还过流就报错
			allflag->Erro_flag = 2;
	}
	else
	{
		allflag->Erro_counts = 0;
		allflag->Erro_flag = 0;
	}
}


/********************
磁编码器数据处理
********************/
void Encoder_Task(Encoder_Struct *encoder_str,AllFlag *allflag)
{
//	encoder_str->Encoder = MT6701_ReadRaw();//10us
	//确定编码器方向
	if(encoder_str->motordir ==1)
		encoder_str->Encoder_raw = 16384-encoder_str->Encoder;
	else
		encoder_str->Encoder_raw = encoder_str->Encoder;
	
	//角度转换
	//(L1)4.24072f (L2)25.8616f (L3)6.9873f  (L4)15.6006f  (L5)17.3584f  (L6)12.1289f
	encoder_str->Shaft_Angle = (float)encoder_str->Encoder_raw * 0.0219726f - encoder_str->Zero_Angle;   //机械角度获取   
	encoder_str->Shaft_Angle = Angle_Limit(encoder_str->Shaft_Angle,360.f);
	
	encoder_str->Elect_Angle = ElectAngle_Limit(encoder_str->Shaft_Angle,motor_factor.motor_pole); //电角度获取，0-360
	encoder_str->Elect_Angle = Angle_Limit(encoder_str->Elect_Angle,360.f);

	
	//圈数统计和角度累计(用于含有减速器时)
	//1、计算这次角度和上次角度差
	float delta_angle = encoder_str->Shaft_Angle - encoder_str->last_shaft_angle;
	
	//2、过零点判断
	if(delta_angle>180.0f) //反转过零点
	{
		delta_angle -= 360.0f;
		encoder_str->rotation_counts--;
	}
	else if(delta_angle<-180.0f) //正转过零点
	{
		delta_angle += 360.0f;
		encoder_str->rotation_counts++;
	}

	//3、正常在一圈内
	//计算电机轴累计角度
	encoder_str->shaft_total_angle = encoder_str->rotation_counts * 360.0f + encoder_str->Shaft_Angle;
	
	
	if(motor_factor.motor_gear == 1.0f)
	{
		// 无减速器模式：直接使用单圈机械角度作为输出轴角度
		encoder_str->output_shaft_angle = encoder_str->Shaft_Angle;
	}
	else 
	{
		//计算输出轴绝对角度(根据减速比)
		encoder_str->output_shaft_angle = Angle_Limit(encoder_str->shaft_total_angle / motor_factor.motor_gear,360.0f);
	}

	encoder_str->last_shaft_angle = encoder_str->Shaft_Angle;
	
	
	
	
//	encoder_str->Encoder_raw_erro = (encoder_str->Encoder_raw-encoder_str->Encoder_old_raw);
//	//16384/2，判断正反转过零点
//	if(encoder_str->Encoder_raw_erro>8192)          //反转过零点
//		encoder_str->Encoder_raw_erro = encoder_str->Encoder_raw-encoder_str->Encoder_old_raw-16384;
//	else if(encoder_str->Encoder_raw_erro<-8192)    //正转过零点  
//		encoder_str->Encoder_raw_erro = 16384-encoder_str->Encoder_old_raw+encoder_str->Encoder_raw;
//	
//	encoder_str->Encoder_raw_sum += (float)encoder_str->Encoder_raw_erro;
//	
//	encoder_str->Encoder_old_raw = encoder_str->Encoder_raw;               //保留上一次编码器数值
	
	//零点对齐，只进行一次
	if(allflag->Adc_Adjust_flag==1 && allflag->Zero_flag==0)               //ADC电流采样校准完成后进行磁编码器校准
	{
		SVPWM_Zero(0,1.0f,0,&svpwm_str);
		encoder_str->zero_count++;
		if(encoder_str->zero_count>19500)        //让对齐0.95s后再校准
		{
			encoder_str->Zero_Angle_cal += encoder_str->Shaft_Angle;
			if(encoder_str->zero_count>=20000)  //0.05s采集求平均值 
			{
				encoder_str->Zero_Angle = encoder_str->Zero_Angle_cal/500.0f;
				encoder_str->Zero_Angle_cal = 0;
				encoder_str->zero_count = 0;
				allflag->Zero_flag = 1;
				
			}
		}	
	
	}
	
	if(allflag->Encoder_flag==1)  //编码器模式1：开环角度自增(方向由motordir决定)
	{
		encoder_str->r_s_speed = 1;         //转/s
		encoder_str->Encoder_Mode1_Angle += encoder_str->r_s_speed*0.018f;  
		encoder_str->Encoder_Mode1_Angle = Angle_Limit(encoder_str->Encoder_Mode1_Angle,360.f);//机械角度
  
		encoder_str->Return_Rad = ElectAngle_Limit(encoder_str->Encoder_Mode1_Angle,14);        //电角度
		
		encoder_str->Return_Rad = encoder_str->Return_Rad*0.01745329f; //转换为0-2PI
		encoder_str->Return_Rad = Angle_Limit(encoder_str->Return_Rad,2*PI);
		

		encoder_str->Return_Angle = Coridc_Angle(encoder_str->Return_Rad); //转换为[-PI,PI]
	}
	else if(allflag->Encoder_flag==2) //编码器模式2：闭环角度控制  返回校准后的电角度值
	{
		encoder_str->Return_Rad = encoder_str->Elect_Angle;
		encoder_str->Return_Rad = encoder_str->Return_Rad*0.01745329f; //转换为0-2PI
		encoder_str->Return_Rad = Angle_Limit(encoder_str->Return_Rad,2*PI);
		

		encoder_str->Return_Angle = Coridc_Angle(encoder_str->Return_Rad); //转换为[-PI,PI]
	}
	else if(allflag->Encoder_flag==3) //编码器模式3：角度定点控制 取值[0,360]
	{
		encoder_str->Encoder_Mode3_Angle = 40; //设定角度
		
		encoder_str->Return_Rad = ElectAngle_Limit(encoder_str->Encoder_Mode3_Angle,14);        //电角度
		encoder_str->Return_Rad = encoder_str->Return_Rad*0.01745329f; //转换为0-2PI
		encoder_str->Return_Rad = Angle_Limit(encoder_str->Return_Rad,2*PI);
		

		encoder_str->Return_Angle = Coridc_Angle(encoder_str->Return_Rad); //转换为[-PI,PI]
	}

}

void Mode_Task(Encoder_Struct *encoder_str,AllFlag *allflag)
{
	//电压开环控制
	if(allflag->Mode_flag==1)
	{
		allflag->Encoder_flag = 1;
		SVPWM_Zero(1.0f,0,encoder_str->Return_Angle,&svpwm_str);

	}
	//电流环控制
	else if(allflag->Mode_flag==2)
	{
		/****************电流环带宽测试，不用时屏蔽*******************/
//		theta += Iwidth*2*PI/20000;
//		theta = Angle_Limit(theta,2*PI);
//		
//		allflag->Encoder_flag = 3;
//		pid_m1.Id_aim = encoder_str->Encoder_Mode1_Angle*arm_sin_f32(theta);
//		PID_I_Control(&pid_m1);
//		SVPWM_Zero(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
		/***********************************/
		
		allflag->Encoder_flag = 2;
//		pid_m1.Iq_aim = pid_m1.i_zl_qk;
		PID_I_Control(&pid_m1);
		SVPWM_Zero(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
	//速度环-电流环
	else if(allflag->Mode_flag==3)
	{		
		allflag->Encoder_flag = 2;
		PID_Speed_Control(&pid_m1,encoder_str);      //速度环
		pid_m1.Iq_aim = pid_m1.speed_out;
		pid_m1.Id_aim = 0;
		PID_I_Control(&pid_m1);
		SVPWM_Zero(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
	//位置环-速度环-电流环
	else if(allflag->Mode_flag==4)
	{
		allflag->Encoder_flag = 2;
//		pid_m1.Position_aim=120.0f;
//		pid_m1.Position_aim=CanFD_Message.Control_Target;
		PID_Position_Control(&pid_m1,encoder_str);    //位置环
		pid_m1.Speed_aim = pid_m1.position_out;
		PID_Speed_Control(&pid_m1,encoder_str);      //速度环
		pid_m1.Iq_aim = pid_m1.speed_out;
		pid_m1.Id_aim = 0;
		PID_I_Control(&pid_m1);
		SVPWM_Zero(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
	//带速度规划的速度环-电流环
	else if(allflag->Mode_flag==5)
	{
		static float last_speed_set = 0.0f;
		
		if(fabs(last_speed_set-pid_m1.target_speed) > 0.1f)
		{
			Planner_Speed_Init(&pid_m1.simple_plan,pid_m1.target_speed,500.0f,pid_m1.Speed_now);   
			last_speed_set = pid_m1.target_speed;
		}
		float plan_speed = Planner_Speed_Update(&pid_m1.simple_plan,0.0005f); //2kHz执行频率
		
		pid_m1.Speed_aim = plan_speed;
		
		allflag->Encoder_flag = 2;
		PID_Speed_Control(&pid_m1,encoder_str);      //速度环
		pid_m1.Iq_aim = pid_m1.speed_out;
		pid_m1.Id_aim = 0;
		PID_I_Control(&pid_m1);
		SVPWM_Zero(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
	//位置环(速度可设定的位置调节)-速度环-电流环
	else if(allflag->Mode_flag==6)
	{
		static float last_angle_set = 370.0f;     //确保起始能回归0点,同时在上面也给规划速度赋值10了
		
		pid_m1.Position_aim = Angle_Limit(pid_m1.Position_aim,360.0f);
		if(fabs(last_angle_set-pid_m1.Position_aim)> 0.2f)
		{
			Planner_Position_Init(&pid_m1.simple_plan,pid_m1.Position_aim,pid_m1.Speed_now,pid_m1.target_speed);
			last_angle_set = pid_m1.Position_aim;
		}
		
		float position_erro = pid_m1.Position_aim - encoder_str->output_shaft_angle;
		if (position_erro >  180.0f) position_erro -= 360.0f;
        else if (position_erro < -180.0f) position_erro += 360.0f;
		
		if(position_erro>0.2f || position_erro<-0.2f)
			pid_m1.Speed_aim = Planner_Position_Update(&pid_m1.simple_plan,encoder_str->output_shaft_angle,0.0005f);
		else
		{
			PID_Position_Control(&pid_m1,encoder_str);    //位置环
			pid_m1.Speed_aim = pid_m1.position_out;
		}
			
		allflag->Encoder_flag = 2;	
		PID_Speed_Control(&pid_m1,encoder_str);      //速度环
		pid_m1.Iq_aim = pid_m1.speed_out;
		pid_m1.Id_aim = 0;
		PID_I_Control(&pid_m1);
		SVPWM_Zero(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}	

	
}








void My_Task(void)
{
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	Adc_Task(&adcvalue,&allflag,ADC1InjectDate);       //电流计算
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
	Encoder_Task(&encoder_str,&allflag);               //角度处理
	Clark_Park(&adcvalue,&encoder_str,&pid_m1,&svpwm_str);   //用于计算电流环的Iq和Id
	
	allflag.Mode_flag = 6;
	if(allflag.Adc_Adjust_flag==1)
	{
		Mode_Task(&encoder_str,&allflag);              //电机运行模式选择
	}
//	else if(allflag.Erro_flag==2)
//	{
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2100);   //Duty=4225/8500
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2100);
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2100);
//	}

	
}
