#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include "Matrix.h"

#define  KF_STATE_NUM	  6
#define  KF_MEASURE_NUM   4

class KALMAN
{
public:
	KALMAN();

	/*位置滤波参数*/
	Matrix Xkk,Pkk;
	
	void Reset(float x,float y,float theta,float v,float a,float w);

	void Predict();
	void SetMatrix_R();
	void CorrectDR(float measure[4]);

	void Init();

	void gener_A(float T);

	/*角度滤波参数*/
	Matrix Xkk_c, Pkk_c;

	void Reset_c(float theta, float w);

	void Predict_c();
	void SetMatrix_R_c();
	void CorrectDR_c(float measure[1]);

	void Init_c();

	void gener_A_c(float T);

	/*观测值滤波参数*/
	Matrix Xkk_meas, Pkk_meas;

	void Reset_meas(float x, float y, float theta);

	void Predict_meas();
	void SetMatrix_R_meas(float a, float b, float c, float var[3]);
	void CorrectDR_meas(int flag,float measure[3]);

	void Init_meas();

protected:
	
private:

	/*位置滤波参数*/
	int x_r = KF_STATE_NUM;
	int h_r = KF_MEASURE_NUM;
	Matrix A, X, G, H, Xk, Z, R, Q, K, P_temp, Pk, I;

	
	/*角度滤波参数*/
	int x_t = 2;
	int h_t = 1;
	Matrix A_c, X_c, H_c, Xk_c, Z_c, R_c, Q_c, K_c, P_temp_c, Pk_c, I_c;\

	/*观测值滤波参数*/
	int x_m = 3;
	int h_m = 3;
	Matrix A_meas, X_meas, H_meas, Xk_meas, Z_meas, R_meas, Q_meas, K_meas, P_temp_meas, Pk_meas, I_meas;
};




#endif

