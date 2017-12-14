
#include <math.h>
#include "KalmanFilter.h"

#ifndef CV_PI
#define CV_PI 3.14159265358979323846264338327950288
#endif

KALMAN::KALMAN()
{
	/*位置滤波参数*/
	X.resize(x_r, 1);
	Xk.resize(x_r, 1);
	Xkk.resize(x_r, 1);
	A.resize(x_r, x_r);
	G.resize(x_r, 1);
	H.resize(h_r, x_r);
	Z.resize(h_r, 1);
	R.resize(h_r, h_r);
	P_temp.resize(x_r, x_r);
	Pk.resize(x_r, x_r);
	Pkk.resize(x_r, x_r);
	K.resize(x_r, h_r);
	I.resize(x_r, x_r);
	Q.resize(x_r, x_r);

	/*角度滤波参数*/
	X_c.resize(x_t, 1);
	Xk_c.resize(x_t, 1);
	Xkk_c.resize(x_t, 1);
	A_c.resize(x_t, x_t);
	H_c.resize(h_t, x_t);
	Z_c.resize(h_t, 1);
	R_c.resize(h_t, h_t);
	P_temp_c.resize(x_t, x_t);
	Pk_c.resize(x_t, x_t);
	Pkk_c.resize(x_t, x_t);
	K_c.resize(x_t, h_t);
	I_c.resize(x_t, x_t);
	Q_c.resize(x_t, x_t);

	/*观测值滤波参数*/
	X_meas.resize(x_m, 1);
	Xk_meas.resize(x_m, 1);
	Xkk_meas.resize(x_m, 1);
	Z_meas.resize(h_m, 1);
	A_meas.resize(x_m, x_m);
	H_meas.resize(h_m, x_m);
	R_meas.resize(h_m, h_m);
	P_temp_meas.resize(x_m, x_m);
	Pk_meas.resize(x_m, x_m);
	Pkk_meas.resize(x_m, x_m);
	K_meas.resize(x_m, h_m);
	I_meas.resize(x_m, x_m);
	Q_meas.resize(x_m, x_m);

}



#if 1

/*匀加速模型  KF  位置滤波函数*/

/*
	线性卡尔曼滤波
	状态量： x,v_x,a_x,y,v_y,a_y  
	观测量： v_x,a_x,v_y,a_y  (观测量a_x,a_y 为车辆坐标系下，需转换到绝对坐标系。a_x = meas_a_x*cos(theta),a_y = meas_a_y*sin(theta))
	
*/

void KALMAN::Reset(float x,float vx,float ax,float y,float vy,float ay)
{
	//单位阵
	for (int i = 0; i < x_r; i++){
		for (int j = 0; j < x_r; j++){
			if (i == j){
				I[i][j] = 1;
			}
		}
	}
	
	H[0][1] = 1;
	H[1][2] = 1;
	H[2][4] = 1;
	H[3][5] = 1;
	//H[4][4] = 1;
	//H[5][5] = 1;

	//状态量赋初值
	X[0][0] = x;
	X[1][0] = vx;
	X[2][0] = ax;
	X[3][0] = y;
	X[4][0] = vy;
	X[5][0] = ay;

	Xkk[0][0] = x;
	Xkk[1][0] = vx;
	Xkk[2][0] = ax;
	Xkk[3][0] = y;
	Xkk[4][0] = vy;
	Xkk[5][0] = ay;

	/*	*/
	//方差赋初值
	P_temp[0][0] = 0.1;
	P_temp[1][1] = 0.1;
	P_temp[2][2] = 0.1;
	P_temp[3][3] = 0.1;
	P_temp[4][4] = 0.1;
	P_temp[5][5] = 0.1;
	
	//过程噪声估计
	Q[0][0] = 0.1;
	Q[1][1] = 0.01;
	Q[2][2] = 0.001;
	Q[3][3] = 0.1;
	Q[4][4] = 0.01;
	Q[5][5] = 0.001;

}


void KALMAN::SetMatrix_R()
{
	//观测噪声估计

	R.clear();
	R[0][0] = 0.1;
	R[1][1] = 0.01;
	R[2][2] = 0.1;
	R[3][3] = 0.01;

}

void KALMAN::Predict()
{
	X[0][0] = Xkk_meas[0][0];
	X[3][0] = Xkk_meas[1][0];

	Xk =  A*X;
	
	Pk = A * P_temp * trans(A) + Q;

}


/***************************
	measure : v_x,a_x,v_y,a_y
****************************/

void KALMAN::CorrectDR(float measure[4])
{

	Z[0][0] = measure[0];
	Z[1][0] = measure[1];
	Z[2][0] = measure[2];
	Z[3][0] = measure[3];

	//状态更新
	K = Pk * trans(H) * inverse(H * Pk * trans(H) + R);
	Xkk = Xk + K * (Z - H * Xk);
	Pkk = (I - K * H) * Pk;
	
	//估计值作为下次滤波的起始
	X = Xkk;
	P_temp = Pkk;
	
}


void KALMAN::Init()
{
	X.clear();
	Xk.clear();
	Xkk.clear();
	P_temp.clear();
	Pk.clear();
	Pkk.clear();
	Q.clear();
	R.clear();
	Z.clear();
	K.clear();
	X_c.clear();
	Pk_c.clear();
}


void KALMAN::gener_A(float T)
{
	A[0][0] = 1;
	A[0][1] = T;
	A[0][2] = T*T/2;
	A[1][1] = 1;
	A[1][2] = T;
	A[2][2] = 1;

	A[3][3] = 1;
	A[3][4] = T;
	A[3][5] = T*T / 2;
	A[4][4] = 1;
	A[4][5] = T;
	A[5][5] = 1;

}

#endif


#if 1

/*角度滤波函数*/

/*
线性卡尔曼滤波
状态量： theta,w
观测量： w  

*/

void KALMAN::Reset_c(float theta, float w)
{
	//单位阵
	for (int i = 0; i < x_t; i++){
		for (int j = 0; j < x_t; j++){
			if (i == j){
				I_c[i][j] = 1;
			}
		}
	}

	H_c[0][1] = 1;
	//H_c[1][1] = 1;


	//状态量赋初值
	X_c[0][0] = theta;
	X_c[1][0] = w;

	Xkk_c[0][0] = theta;
	Xkk_c[1][0] = w;

	/*	*/
	//方差赋初值
	P_temp_c[0][0] = 0.1;
	P_temp_c[1][1] = 0.1;


	//过程噪声估计
	Q_c[0][0] = 0.01;
	Q_c[1][1] = 0.001;


}


void KALMAN::SetMatrix_R_c()
{
	//观测噪声估计

	R_c.clear();

	R_c[0][0] = 0.01;

}

void KALMAN::Predict_c()
{
	X_c[0][0] = Xkk_meas[2][0];

	Xk_c = A_c*X_c;

	Pk_c = A_c * P_temp_c * trans(A_c) + Q_c;

}



/***************************
measure : w
****************************/

void KALMAN::CorrectDR_c(float measure[1])
{

	Z_c[0][0] = measure[0];

	//状态更新
	K_c = Pk_c * trans(H_c) * inverse(H_c * Pk_c * trans(H_c) + R_c);
	Xkk_c = Xk_c + K_c * (Z_c - H_c * Xk_c);
	Pkk_c = (I_c - K_c * H_c) * Pk_c;

	//估计值作为下次滤波的起始
	X_c = Xkk_c;
	P_temp_c = Pkk_c;

}


void KALMAN::Init_c()
{
	X_c.clear();
	Xk_c.clear();
	Xkk_c.clear();
	P_temp_c.clear();
	Pk_c.clear();
	Pkk_c.clear();
	Q_c.clear();
	R_c.clear();
	Z_c.clear();
	K_c.clear();
	X_c.clear();
	Pk_c.clear();
}


void KALMAN::gener_A_c(float T)
{
	A_c[0][0] = 1;
	A_c[0][1] = T;
	A_c[1][0] = 0;
	A_c[1][1] = 1;
}

#endif


#if 1

/*VPS等观测值滤波函数*/

/*
线性卡尔曼滤波
状态量： x,y,theta (DR 结果)
观测量： x,y,theta

*/

void KALMAN::Reset_meas(float x, float y,float theta)
{
	//单位阵
	for (int i = 0; i < x_m; i++){
		for (int j = 0; j < x_m; j++){
			if (i == j){
				I_meas[i][j] = 1;
			}
		}
	}

	A_meas[0][0] = 1;
	A_meas[1][1] = 1;
	A_meas[2][2] = 1;

	H_meas[0][0] = 1;
	H_meas[1][1] = 1;
	H_meas[2][2] = 1;

	//状态量赋初值
	X_meas[0][0] = x;
	X_meas[1][0] = y;
	X_meas[2][0] = theta;

	Xkk_meas[0][0] = x;
	Xkk_meas[1][0] = y;
	Xkk_meas[2][0] = theta;

	/*	*/
	//方差赋初值
	P_temp_meas[0][0] = 0.1;
	P_temp_meas[1][1] = 0.1;
	P_temp_meas[2][2] = 0.1;


	//过程噪声估计
	Q_meas[0][0] = 0.08;
	Q_meas[1][1] = 0.08;
	Q_meas[2][2] = 0.1;


}


void KALMAN::SetMatrix_R_meas(float a, float b, float c, float var[3])
{
	//观测噪声估计

	R_meas.clear();

	R_meas[0][0] = var[0] * a;
	R_meas[1][1] = var[1] * b;


	if (var[2] > 0.5)
	{
		R_meas[2][2] = 100000000;
	}
	else
	{
		R_meas[2][2] = var[2] * c;
	}

}

void KALMAN::Predict_meas()
{
	X_meas[0][0] = Xkk[0][0];
	X_meas[1][0] = Xkk[3][0];
	X_meas[2][0] = Xkk_c[0][0];

	Xk_meas = A_meas*X_meas;

	Pk_meas = A_meas * P_temp_meas * trans(A_meas) + Q_meas;

}



/***************************
measure : v_x,a_x,v_y,a_y
****************************/

void KALMAN::CorrectDR_meas(int flag,float measure[6])
{
	float a = 10;
	float b = 10;
	float c = 10;
	float var[3] = {0};

	var[0] = measure[3];
	var[1] = measure[4];
	var[2] = measure[5];


	Z_meas[0][0] = measure[0];
	Z_meas[1][0] = measure[1];
	Z_meas[2][0] = measure[2];

	SetMatrix_R_meas(a,b,c,&var[0]);

	if (flag == 1)
	{
		//状态更新
		K_meas = Pk_meas * trans(H_meas) * inverse(H_meas * Pk_meas * trans(H_meas) + R_meas);
		Xkk_meas = Xk_meas + K_meas * (Z_meas - H_meas * Xk_meas);
		Pkk_meas = (I_meas - K_meas * H_meas) * Pk_meas;

	}
	else
	{
		Xkk_meas = Xk_meas;
		Pkk_meas = Pk_meas;

	}

	//估计值作为下次滤波的起始
	X_meas = Xkk_meas;
	P_temp_meas = Pkk_meas;

}


void KALMAN::Init_meas()
{
	X_meas.clear();
	Xk_meas.clear();
	Xkk_meas.clear();
	P_temp_meas.clear();
	Pk_meas.clear();
	Pkk_meas.clear();
	Q_meas.clear();
	R_meas.clear();
	Z_meas.clear();
	K_meas.clear();
	X_meas.clear();
	Pk_meas.clear();
}



#endif











