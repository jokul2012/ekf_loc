

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include "KalmanFilter.h"

#ifndef ABS
#define ABS(x) ((x) >= 0 ? (x):-(x))
#endif

using namespace std;

KALMAN kalman;

#ifndef PI
#define PI ((float)3.14159265358979323846264338327950288)
#endif

FILE *pEKFfile;

FILE *pDRfile;


void steeringwheel_radius(float str_whl_angle,int shft_pos,float &radius)//convert from steering wheel angle to steering radius
{
#if 0 // saic
	if (2 == shft_pos)
	{
		if (str_whl_angle < 0) // steering wheel turns clockwisely
		{
			radius = 2.80 *1.0 / tan((-5.6777e-08F*str_whl_angle*str_whl_angle*str_whl_angle - 4.2675e-5F*str_whl_angle*str_whl_angle - 6.8360e-02*str_whl_angle + 3.9392e-03)*PI / 180) - 1.9 / 2.0;
		}
		else
		{
			radius = 2.80 *1.0 / tan((7.8907e-08F*str_whl_angle*str_whl_angle*str_whl_angle + 2.5257e-6F*str_whl_angle*str_whl_angle + 6.8725e-02*str_whl_angle + 6.3143e-03)*PI / 180) + 1.9 / 2.0;
		}
	}
	else
	{
		if (str_whl_angle < 0) // steering wheel turns clockwisely
		{
			radius = 2.80 *1.0 / tan((-7.1131e-08F*str_whl_angle*str_whl_angle*str_whl_angle - 5.0327e-5F*str_whl_angle*str_whl_angle - 6.7484e-02*str_whl_angle + 8.2275e-03)*PI / 180) - 1.9 / 2.0;
		}
		else
		{
			radius = 2.80 *1.0 / tan((1.2227e-07F*str_whl_angle*str_whl_angle*str_whl_angle - 2.493e-5F*str_whl_angle*str_whl_angle + 7.0171e-02*str_whl_angle + 4.7559e-03)*PI / 180) + 1.9 / 2.0;
		}
	}
#endif

#if 1  //Ofilm

	if (2 == shft_pos)
	{

		if (str_whl_angle < 0) // steering wheel turns clockwisely
		{
			radius = 2.75 *1.0 / tan((-3.269e-08F*str_whl_angle*str_whl_angle*str_whl_angle - 2.488e-05F*str_whl_angle*str_whl_angle - 0.06131*str_whl_angle + 0.004912)*PI / 180) - 0.9195;

		}
		else
		{
			radius = 2.75 *1.0 / tan((3.466e-08F*str_whl_angle*str_whl_angle*str_whl_angle + 2.798e-05F*str_whl_angle*str_whl_angle + 0.05994*str_whl_angle + 0.0446)*PI / 180) + 0.9195;
		}
	}
	else
	{

		if (str_whl_angle < 0) // steering wheel turns clockwisely
		{
			radius = 2.75 *1.0 / tan((-6.667e-08F*str_whl_angle*str_whl_angle*str_whl_angle - 5.163e-05F*str_whl_angle*str_whl_angle - 0.06626*str_whl_angle + 0.007778)*PI / 180) - 0.9195;
		}
		else
		{
			radius = 2.75 *1.0 / tan((8.628e-08F*str_whl_angle*str_whl_angle*str_whl_angle - 1.086e-05F*str_whl_angle*str_whl_angle + 0.06691*str_whl_angle + 0.005639)*PI / 180) + 0.9195;
		}
	}

#endif

}

float fusion_get_distance_from_pulse(int LeftPulse, int LeftPulse_Last, int RightPulse, int RightPulse_Last)
{
	//LOGF("get_distance_from_pulse");
	int pulse_curent[4];
	int pulse_pre[4];
	int pulse_delta[4];

	pulse_curent[0] = LeftPulse;//v_data->wheel_pulse[rear_left_whl];
	pulse_pre[0] = LeftPulse_Last;//v_data->pre_wheel_pulse[rear_left_whl];
	pulse_curent[1] = RightPulse;//v_data->wheel_pulse[rear_right_whl];
	pulse_pre[1] = RightPulse_Last;//v_data->pre_wheel_pulse[rear_right_whl];//



	for (int i = 0; i < 4; i++)
	{
		if (pulse_curent[i] < pulse_pre[i])
		{
			pulse_delta[i] = -pulse_pre[i] + 1024 + pulse_curent[i];
		}
		else
		{
			pulse_delta[i] = -pulse_pre[i] + pulse_curent[i];
		}
	}

	float distance_rt = (pulse_delta[0] + pulse_delta[1])*0.022;
	float distance_rt2 = (pulse_delta[2] + pulse_delta[3])*0.022;

	return distance_rt;

}



float HandleThetaMeasure(float theta_measure, float theta_pre)
{

	if (theta_measure*theta_pre < 1e-8&&fabs(theta_measure)>2 && fabs(theta_pre) > 2)
	{
		theta_pre = (theta_measure / fabs(theta_measure)) * 2 * PI + theta_pre;
	}


	return theta_pre;
}


void main()
{
	int i = 0;

	pEKFfile = fopen("fusion_data.txt", "a+");
	pDRfile = fopen("dr_data.txt", "a+");


	//(-3.269e-08F*str_whl_angle*str_whl_angle*str_whl_angle - 2.488e-05F*str_whl_angle*str_whl_angle - 0.06131*str_whl_angle + 0.004912)


	/************************************************

	Kalman fusion location  add by WJQ   2017-11-16

	************************************************/


	//1)get  common_vehicle_data    and   IMU data 
	/*
		0-time ;1-shiftPos;2-steeringAngle;3-speedKmh;4-yawrate;5-FLspeed;6-FRspeed;7-RLspeed;8-RRspeed;9-LDpls;10-LNDpls;11-RDpls;12-RNDpls;
		13-lon;14-lat;15-heading;16-alt;17-AccX;18-AccY;19-AccZ;20-GyroX;21-GyroY;22-GyroZ
	
	*/
	
	float can[1832][23];
	FILE *fp;                 /*定义一个文件指针*/
	//fp = fopen("F:\\0-work\\Project\\Fusion_Location\\EKF_Loc_Test\\103003\\can_data14.csv", "r");   /*打开文件csv文件*/

	fp = fopen("F:\\0-work\\Project\\Fusion_Location\\DR_KF\\data\\can_log.csv", "r");

	/*利用逗号间隔，读取文本数据到一个数组里*/
	while (fscanf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
		&can[i][0], &can[i][1], &can[i][2], &can[i][3], &can[i][4], &can[i][5], &can[i][6], &can[i][7], &can[i][8], &can[i][9], &can[i][10], 
		&can[i][11], &can[i][12], &can[i][13], &can[i][14], &can[i][15], &can[i][16], &can[i][17], &can[i][18], &can[i][19], &can[i][20], 
		&can[i][21],&can[i][22]) != EOF)
	{  
		//if (feof(fp))
		//	break;
		i++;
	}

	fclose(fp);


	// 2) get   DR   VPS  SLAM  PLD result  :  X ,Y  ,theta 及其方差
	/*
	   0-time;1-x;2-y;3-yaw;4-xvar;5-yvar;6-yawvar
	*/

	i = 0;
	float vps[836][7];
	FILE *fpVPS;                 /*定义一个文件指针*/

	fpVPS = fopen("F:\\0-work\\Project\\Fusion_Location\\DR_KF\\data\\KLF_VPS_log.csv", "r");   /*打开文件csv文件*/

	//读取无法结束问题：
	/*利用逗号间隔，读取文本数据到一个数组里*/
	while (fscanf(fpVPS, "%f,%f,%f,%f,%f,%f,%f",
		&vps[i][0], &vps[i][1], &vps[i][2], &vps[i][3], &vps[i][4], &vps[i][5], &vps[i][6]) != EOF)
	{
		//if (feof(fp))
		//	break;
		i++;		
	}

	fclose(fpVPS);


	static float theta_kf = 0;
	i = 0;

	for (i = 0; i < 1832; i++)
	{
		int LeftPulse = can[i][9];
		static int LeftPulse_Last = 0;

		int RightPulse = can[i][11];
		static int RightPulse_Last = 0;

		int timeCam = can[i][0];
		static int timeLast = 0;
		float TimeoffsetKF = 0;


		static int KFflag = 0;
		static int KFcount = 0;


		int	shift_pos = can[i][1];
		//上汽轮速用前轮，ofilm用后轮
		//float wheel_speed_rl = a[i][6];// forward
		//float wheel_speed_rr = a[i][7];

		float wheel_speed_rl = can[i][7];//rear
		float wheel_speed_rr = can[i][8];


		float steering_angle = can[i][2];
		float yaw_rate = can[i][4];

		float ax_car = can[i][17];
		float ay_car = can[i][18];

		TimeoffsetKF = (timeCam - timeLast)/1000000.0;

		float v = 0;

		v = (wheel_speed_rr + wheel_speed_rl) / 2.0f / 3.6f;//上汽轮速用前轮，ofilm用后轮
		v = v*1.08;

		float w = 0;

		w = yaw_rate*PI / 180;
		//w = w *0.9752;

		/**/
		float radius_kf;
		steeringwheel_radius(steering_angle, shift_pos, radius_kf);
		radius_kf *= 0.975;

		//int turn_sign_kf = get_turn_dir(v_data);
		//float theta_offset_paluse_kf = turn_sign_kf*g_KalmanFilter_PLD.get_distance_from_pulse(v_data) / radius_kf;
		//float theta_offset_kf = theta_offset_paluse_kf;

		float s = v*TimeoffsetKF;

		//float theta_offset_paluse_kf = fusion_get_distance_from_pulse(LeftPulse,LeftPulse_Last,RightPulse,RightPulse_Last) / radius_kf;
		float theta_offset_paluse_kf = s / radius_kf;
#if (USE_SPEED)
		float theta_offset_kf = theta_offset_speed_kf;
#else
		float theta_offset_kf = theta_offset_paluse_kf;
#endif

		//theta_kf = kalman.Xkk_c[0][0];
		w = -1*can[i][22]*1;

		float vx = 0;
		float vy = 0;
		float ax = 0;
		float ay = 0;



		/*滤波初始化*/

		if (0 == KFflag)
		{
			//if (false == tloc_vps_mm.isEmpty())
			{

				float x		= vps[i][1];
				float y		= vps[i][2];
				float theta = vps[i][3];

				float x_var = vps[i][4];
				float y_var = vps[i][5];
				float theta_var = vps[i][6];

				theta_kf = theta;

				vx = v*sin(theta_kf);
				vy = v*cos(theta_kf);
				ax = ax_car*sin(theta_kf);
				ay = ay_car*cos(theta_kf);

				kalman.Init();
				kalman.Reset(x, vx,ax,y,vy,ay);

				kalman.Init_c();
				kalman.Reset_c(theta,w);

				kalman.Init_meas();
				kalman.Reset_meas(x, y,theta);
				KFflag = 1;
			}
		}/**/

		KFcount++;

		if (1 == KFflag&&KFcount>1)
		{
#if 0
			/*************************************
			VPS 观测值修正滤波
			**************************************/
			double dert_T = -1;
			float measureZ[3] = { 0 };
			float measureVar[3] = { 0 };

			int measureFlag[5] = { 0 };
			float VPSdert_T = 0, VPSmeasure[3] = { 0 }, VPSmeasureVar[3] = { 0 };
			float LDWdert_T = 0, LDWmeasure[3] = { 0 }, LDWmeasureVar[3] = { 0 };
			float SLAMdert_T = 0, SLAMmeasure[3] = { 0 }, SLAMmeasureVar[3] = { 0 };
			float PLDdert_T = 0, PLDmeasure[3] = { 0 }, PLDmeasureVar[3] = { 0 };

			static float VPSmeasure_last[3] = { 0 };
			static double TimeMeasureLast = 0;
			double TimeMeasure = 0;

			/*********************************************
			get the measure for vps fusion
			********************************************/

			int vps_pos = 0;
			for (int m = 0; m < 836; m++)
			{
				if ((can[i][0]) == vps[m][0])
				{
					vps_pos = m;
					break;
				}
			}


			if (vps_pos>0)
			{

				TimeMeasure = vps[vps_pos][0];

				dert_T = (timeCam - TimeMeasure) / 1000000.0;

				/* search the nearest time DR result */

				VPSmeasure[0] = vps[vps_pos][1];
				VPSmeasure[1] = vps[vps_pos][2];
				VPSmeasure[2] = vps[vps_pos][3];

				VPSmeasureVar[0] = vps[vps_pos][4];
				VPSmeasureVar[1] = vps[vps_pos][5];
				VPSmeasureVar[2] = vps[vps_pos][6];

				/*vps measure  abnormal value process
				1) var <1;   2)dertX dertY <   3)dert_theta < ;
				*/
				if ((VPSmeasureVar[0]<1 && VPSmeasureVar[1]<1 && VPSmeasureVar[2]<1) ||
					(fabs(VPSmeasure_last[2]) - fabs(VPSmeasure[2])<5 && fabs(VPSmeasure_last[2]) - fabs(VPSmeasure[2])<5) ||
					(steering_angle<50 && (fabs(VPSmeasure_last[2]) - fabs(VPSmeasure[2]))<0.2) ||
					(steering_angle >= 50 && (fabs(VPSmeasure_last[2]) - fabs(VPSmeasure[2]))<0.5))
				{
					measureFlag[0] = 1;
					memcpy(&VPSmeasure_last[0], &VPSmeasure[0], 3 * sizeof(float));
				}

			}

#endif

			theta_kf = kalman.Xkk_c[0][0];

#if 1
			/*********************************** 
		      DR	角度通道滤波 
			***********************************/
			//int VPSflag = measureFlag[0];

			kalman.gener_A_c(TimeoffsetKF);
			kalman.Predict_c();

			float IMU_measure_c[1] = { 0 };

			IMU_measure_c[0] = w;

			kalman.SetMatrix_R_c();
			kalman.CorrectDR_c(&IMU_measure_c[0]);
#endif
			/*
			float  theta_rad_offset = 0;
			if (shift_pos == 2)
			{
				if (steering_angle < 0)
				{
					theta_rad_offset = theta_offset_kf;

					theta_kf += theta_rad_offset;
				}
				else
				{
					theta_rad_offset = -theta_offset_kf;

					theta_kf += theta_rad_offset;
				}
			}
			else
			{
				if (steering_angle < 0)
				{
					theta_rad_offset = -theta_offset_kf;

					theta_kf += theta_rad_offset;
				}
				else
				{
					theta_rad_offset = theta_offset_kf;

					theta_kf += theta_rad_offset;
				}
			}
			*/



#if 1
			/********************************************
			DR 位置通道滤波
			Correct  for   DR  use the IMU inf :  acc  and Gyro
			measure:  v_x,a_x,v_y,a_y
			*********************************************/

			//theta_kf = kalman.Xkk_c[0][0];

			vx = v*sin(theta_kf);
			vy = v*cos(theta_kf);
			ax = ax_car*sin(theta_kf);
			ay = ay_car*cos(theta_kf);


			kalman.gener_A(TimeoffsetKF);
			kalman.Predict();

			float IMU_measure[6] = { 0 };

			IMU_measure[0] = v*sin(theta_kf);
			IMU_measure[1] = ax;
			IMU_measure[2] = v*cos(theta_kf);
			IMU_measure[3] = ay;

			kalman.SetMatrix_R();
			kalman.CorrectDR(&IMU_measure[0]);

			fprintf(pDRfile, "%f\t%f\t%f\n", kalman.Xkk[0][0], kalman.Xkk[3][0], kalman.Xkk_c[0][0]);

#endif
		
			/************************************* 
				VPS 观测值 滤波修正
			**************************************/

			kalman.Predict_meas();

			double dert_T = -1;
			float measureZ[3] = { 0 };
			float measureVar[3] = { 0 };

			int measureFlag[5] = { 0 };
			float VPSdert_T = 0, VPSmeasure[6] = { 0 }, VPSmeasureVar[3] = { 0 };
			float LDWdert_T = 0, LDWmeasure[3] = { 0 }, LDWmeasureVar[3] = { 0 };
			float SLAMdert_T = 0, SLAMmeasure[3] = { 0 }, SLAMmeasureVar[3] = { 0 };
			float PLDdert_T = 0, PLDmeasure[3] = { 0 }, PLDmeasureVar[3] = { 0 };

			static float VPSmeasure_last[6] = { 0 };
			static double TimeMeasureLast = 0;
			double TimeMeasure = 0;

			/*********************************************
			get the measure for vps fusion
			********************************************/

			int vps_pos = 0;
			for (int m = 0; m < 836; m++)
			{
				if ((can[i][0]) == vps[m][0])
				{
					vps_pos = m;
					break;
				}
			}

			if (vps_pos>0)
			{
				TimeMeasure = vps[vps_pos][0];

				dert_T = (timeCam - TimeMeasure) / 1000000.0;

				/* search the nearest time DR result */

				VPSmeasure[0] = vps[vps_pos][1];
				VPSmeasure[1] = vps[vps_pos][2];
				VPSmeasure[2] = vps[vps_pos][3];

				VPSmeasure[3] = vps[vps_pos][4];
				VPSmeasure[4] = vps[vps_pos][5];
				VPSmeasure[5] = vps[vps_pos][6];

				/*vps measure  abnormal value process
				1) var <1;   2)dertX dertY <   3)dert_theta < ;
				*/
				if ((VPSmeasure[3]<1 && VPSmeasure[4]<1 && VPSmeasure[5]<1) ||
					(fabs(VPSmeasure_last[2]) - fabs(VPSmeasure[2])<5 && fabs(VPSmeasure_last[2]) - fabs(VPSmeasure[2])<5) ||
					(steering_angle<50 && (fabs(VPSmeasure_last[2]) - fabs(VPSmeasure[2]))<0.2) ||
					(steering_angle >= 50 && (fabs(VPSmeasure_last[2]) - fabs(VPSmeasure[2]))<0.5))
				{
					measureFlag[0] = 1;
					memcpy(&VPSmeasure_last[0], &VPSmeasure[0], 6 * sizeof(float));
				}

			}


			kalman.CorrectDR_meas(measureFlag[0], &VPSmeasure[0]);

		}


		
		//record last pulse
		LeftPulse_Last = LeftPulse;
		RightPulse_Last = RightPulse;

		timeLast = timeCam;

		/*********************************************
		Fusion Result
		**********************************************/

		fprintf(pEKFfile, "%f\t%f\t%f\n", kalman.Xkk_meas[0][0], kalman.Xkk_meas[1][0], kalman.Xkk_meas[2][0]);
		
	}
	fclose(pEKFfile);
	fclose(pDRfile);

	//printf("\n*******fusion****x=%f,y=%f,yaw=%f\n",kalman.Xkk[0][0],kalman.Xkk[1][0],kalman.Xkk[2][0]);

}

