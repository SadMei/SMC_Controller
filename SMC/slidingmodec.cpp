#include"slidingmodec.h"
#include<cmath>

SMC YawSMC(20, 120, 0, 0.001, 25000, 0.8, 0.5); //����һ��yaw��ο��������

void SMC::SMC_Tick(float angle_now,float angle_vel) //anlgeΪ��ǰλ��(��),ang_velΪ���ٶ�(��/s)
{
	//��ȡ����
	angle = angle_now;
    ang_vel = angle_vel;
	error = angle - ref;
	ddref = (ref - refl) - dref; //�����ǰ�������˴���û���ϸ�λͳһ
	dref = (ref - refl);
	//������޴���
	if (fabs(error) < error_eps)
	{
		u = 0;
		return;
	}
	//smc surface
	s = C * error + (ang_vel - dref);
	u = J * (ddref - C * (ang_vel - dref) - epsilon * Sat(s) - K * s);
	//�������޷�
	if (u > u_max)
		u = u_max;
	if (u < -u_max)
		u = -u_max;
	//��������
	refl = ref;
}