#include"slidingmodec.h"
#include<cmath>

SMC YawSMC(20, 120, 0, 0.001, 25000, 0.8, 0.5); //这是一个yaw轴参考电机参数

void SMC::SMC_Tick(float angle_now,float angle_vel) //anlge为当前位置(°),ang_vel为角速度(°/s)
{
	//读取参数
	angle = angle_now;
    ang_vel = angle_vel;
	error = angle - ref;
	ddref = (ref - refl) - dref; //这里对前馈进行了处理，没有严格单位统一
	dref = (ref - refl);
	//误差下限处理
	if (fabs(error) < error_eps)
	{
		u = 0;
		return;
	}
	//smc surface
	s = C * error + (ang_vel - dref);
	u = J * (ddref - C * (ang_vel - dref) - epsilon * Sat(s) - K * s);
	//控制量限幅
	if (u > u_max)
		u = u_max;
	if (u < -u_max)
		u = -u_max;
	//参数更新
	refl = ref;
}