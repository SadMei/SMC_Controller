#include "stm32f4xx_hal.h"
#include "math.h"
#ifndef _SMC_H_
#define _SMC_H_


class SMC{
 public:
	float C;
	float K;
	float ref; //初始目标值
	float error_eps;//误差下限
	float u_max;//输出最大值
	float J;//估计惯量
	float angle; //角度反馈，°
	float ang_vel;//角速度反馈，rad/s
	float epsilon;

	float u;
	//初始化列表
	SMC(float C,float K,float ref,float error_eps,float u_max,float J,float epsilon):
	C(C),K(K),ref(ref),error_eps(error_eps),u_max(u_max),J(J),epsilon(epsilon){};
	//更新函数
	void SMC_Tick(float angle_now,float angle_vel);

 private:
	float error;
	float error_last;
	float dref;//目标值一阶导
	float ddref;//目标值二阶导
	float refl;//上一次的目标值

	float s;//滑模面

	// 饱和函数
	float Sat(float y)
	{
		if (fabs(y) <= 1)
			return y;
		else
			return Signal(y);
	}
	// 符号函数,若有抖动可以换个陡峭的饱和函数
	int8_t Signal(float y)
	{
		if (y > 0)
			return 1;
		else if (y == 0)
			return 0;
		else
			return -1;
	}
};
extern SMC YawSMC;
#endif
