#include "stm32f4xx_hal.h"
#include "math.h"
#ifndef _SMC_H_
#define _SMC_H_


class SMC{
 public:
	float C;
	float K;
	float ref; //��ʼĿ��ֵ
	float error_eps;//�������
	float u_max;//������ֵ
	float J;//���ƹ���
	float angle; //�Ƕȷ�������
	float ang_vel;//���ٶȷ�����rad/s
	float epsilon;

	float u;
	//��ʼ���б�
	SMC(float C,float K,float ref,float error_eps,float u_max,float J,float epsilon):
	C(C),K(K),ref(ref),error_eps(error_eps),u_max(u_max),J(J),epsilon(epsilon){};
	//���º���
	void SMC_Tick(float angle_now,float angle_vel);

 private:
	float error;
	float error_last;
	float dref;//Ŀ��ֵһ�׵�
	float ddref;//Ŀ��ֵ���׵�
	float refl;//��һ�ε�Ŀ��ֵ

	float s;//��ģ��

	// ���ͺ���
	float Sat(float y)
	{
		if (fabs(y) <= 1)
			return y;
		else
			return Signal(y);
	}
	// ���ź���,���ж������Ի������͵ı��ͺ���
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
