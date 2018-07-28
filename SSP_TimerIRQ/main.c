#include <kernel.h>
#include "kernel_cfg.h"

#include "rpi_lib/rpi.h"
#include <stdio.h>
#include <stdint.h>

#include "kernel_impl.h"
#include "target_kernel.h"

#if defined(RPIBPLUS) || defined(RPI2) || defined(RPI3)
#define	LED_ACT_PIN	47	//TYPE B PLUS
#else
#define LED_ACT_PIN	16	//TYPE B
#endif


#define CORE0_IRQ_SOURCE 	0x40000060
#define INT_SRC_GPU 		0x00000100			//なぜかタイマー割り込み

volatile int count = 0;


int auto_steer;		//自動で戻す


extern int 		pulseWidth;

#define SPEED_MAX	145
#define SPEED_MIN	110




int setup(void)
{
	initializeJtagPin();
	rpi_init();

	// 起動確認用
	pinMode(LED_ACT_PIN,OUTPUT);
	digitalWrite(LED_ACT_PIN, LOW);

	// すべての割り込み不許可
//	*INTERRUPT_DISABLE_BASIC_IRQS = 0xffffffff;
//	*INTERRUPT_DISABLE_IRQS1 = 0xffffffff;
//	*INTERRUPT_DISABLE_IRQS2 = 0xffffffff;
//	*INTERRUPT_FIQ_CTRL = 0;

	// タイマ割り込み設定
	*INTERRUPT_ENABLE_BASIC_IRQS = 0x01;

	// 設定のため一旦タイマー停止
	*TIMER_CONTROL &= 0xffffff00;

	// timer clock を1MHzに設定
	//（0xF9=249: timer clock=250MHz/(249+1)）
	*TIMER_PREDIVIDER = 0x000000F9;

	// タイマー値設定(4sec)
//	*TIMER_LOAD = 4000000-1;
//	*TIMER_RELOAD = 4000000-1;
	// タイマー値設定(1msec) <--間違い　10nsecだと思う。
	//							1000で1ms 10 で　10ms
	*TIMER_LOAD = 10-1;
	*TIMER_RELOAD = 10-1;

	// 割り込みフラグをクリア
	*TIMER_IRQ_CLR = 0;

	// タイマー開始
	// Timer enable, 32bit Timer
	*TIMER_CONTROL |= 0x000000A2;

	// 割り込み許可
	*INTERRUPT_ENABLE_BASIC_IRQS = 0x01;

	// IRQ許可
//	enable_IRQ();

	sta_ker();

	while(1);
//
//	return 0;
}

void task1(intptr_t arg)
{
	uint32_t chi;
	uint32_t clo;

	unsigned long long int now_time;

	now_time = get_systime();

	chi = *SYST_CHI;
	clo = *SYST_CLO;


	now_time = ((unsigned long long int)chi) << 32;
	now_time += clo;
//	printf("system time = %d\n",(int)now_time);

//	printf("task1 RUNNING-----------------------------------------------------\n");
}
void task2(intptr_t arg)
{
	char name[100];

	name[0] = 0x00;
	name[1] = 0x00;
	while(1)
	{
	      printf("動作するならキーをヒットせよ w or s or 4 5 6\n");
	      name[0] = uart0_getc();
	      if (name[0] == 'w')
	      {
	    	  servo_main(45);
	    	  printf("モーター動作開始　GPIO17\n");
	      }
	      else if (name[0] == 's')
	      {
			  stp_cyc(MAIN_CYC);
	    	  printf("モーター動作停止　GPIO17\n");
	    	  digitalWrite(16, HIGH);
	      }
	      else if (name[0] == '3')
	      {
 		  	if (pulseWidth > SPEED_MIN)
		 	{
 				pulseWidth--;
		 	}   	  
	    	printf("モーター動作速度アップGPIO17\n");
	      }
	      else if (name[0] == '1')
	      {
			if (pulseWidth < SPEED_MAX)
			{
				pulseWidth++;
			} 
   	  		printf("モーター動作速度ダウンGPIO17=%d\n",pulseWidth);
	      }
	      else if (name[0] == '4')	//左
	      {
	    	  printf("ステア左 GPIO18\n");
       	  	  pwm_main(5,500,0);
	      }
	      else if (name[0] == '5')	//中
	      {
	    	  pwm_main(60,500,0);
	    	  printf("ステア真ん中GPIO18\n");
	      }
	      else if (name[0] == '6')	//右
	      {
	    	  printf("ステア右GPIO18\n");
	    	  pwm_main(95,500,0);
	      }
	      else if (name[0] == '7')	//ちょい左
	      {
	    	  printf("ちょい左GPIO18\n");
	    	  pwm_main(30,300,1);
	      }
	      else if (name[0] == '9')	//ちょい右
	      {
	    	  printf("ちょい右GPIO18\n");
	    	  pwm_main(75,300,1);
	      }
	}
}

#ifdef TAMIYA_NO_IMPREMENT
void cyclic_handler(intptr_t exinf)
{
}
void alarm_handler(intptr_t exinf)
{
}
void alarm_handler2(intptr_t exinf)
{
}
#endif