//#include "stdafx.h"
#include "calibrationDLL.h"
#include <stdexcept>
#include <Windows.h>
#include <vector>
#include <iostream>

#include <string> 
#include <fstream>
#include <sstream>
#include<time.h>
#include <stdio.h>
#include <string.h>
#include <io.h> //c_file
#include <direct.h> 
#include <iomanip>
#include <stdint.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>


using namespace std;
using namespace cv;


namespace calibrationdll
{
	
	double *** org_matrix3D;
	double calibrationdll::init(char* file_dir, int* sensor_size, double***LUT, double* KG_ary, double*** matrix, double* in_matrix)
	{
		char sdir[256], filter[256], path[256];
	
		
		int csv_id;
		struct _finddata_t c_file;
		long hFile;
		strcpy(sdir, file_dir); //C:\UNEO\LUT_CSV
		strcpy(filter, "*.csv");
		_chdir(sdir);
		hFile = _findfirst(filter, &c_file);
		if (hFile != -1)
		{
			csv_id = 0;
			do{
				sprintf( path, "%s\\%s", sdir, c_file.name);
				printf( "%s\n", path); //parsedata
				parsedata( path, sensor_size , csv_id++ ,matrix );
			} while (_findnext(hFile, &c_file) == 0);
		}
		for (int csv = 0; csv < 6; csv++)
			for (int height = 0; height < sensor_size[0]; height++)
				for (int wide = 0; wide < sensor_size[1]; wide++)
				{
					in_matrix[csv*sensor_size[0] * sensor_size[1] + height* sensor_size[1] + wide] = matrix[height][wide][csv];
					//cout << in_matrix[csv*sensor_size[0] * sensor_size[1] + height* sensor_size[1] + wide] << " ";
				
				}
		int m = sensor_size[0];
		int n = sensor_size[1];
		printf("sensor_size: %d ,%d\n", m, n);
		
		///
		double KG = 0;
		double caliADC = 0;
		int csv_count = csv_id;
		//// 先將 ADC 投影到 KG 上		
		//int loc[2] = { 1,0 };		
		//KG = ADC2KG(in_matrix, 0, loc, csv_count, KG_ary, sensor_size); //int*** matrix, int ADC, int* loc, int csv_amount， matrix是csv平均後的資料(37*22*6)		
		//// 再將KG 投影回 校正斜線上(點斜式) 得到校正後的ADC值
		double slope = 1;
		////

		//tester
		org_matrix3D = (double***)malloc(sensor_size[0] * sizeof(double));
		for (int k = 0; k < sensor_size[0]; k++)
		{
			org_matrix3D[k] = (double **)malloc(sensor_size[1] * sizeof(double));
			for (int q = 0; q < sensor_size[1]; q++)
				org_matrix3D[k][q] = (double*)malloc(csv_count * sizeof(double)); // 6 代表 有6個不同的重量csv
		}
	
		for (int ii = 0; ii < sensor_size[0]; ii++)
			for (int jj = 0; jj < sensor_size[1]; jj++)
				for (int icsv = 0; icsv < csv_count;icsv++)
				{
					org_matrix3D[ii][jj][icsv] = matrix[ii][jj][icsv];
				}
		//




		return  slope;
	}
	

	int calibrationdll::parsedata(char* csv_dir, int* size, int csv_id, double*** matrix)
	{
		fstream file;
		file.open( csv_dir );
		string line;
		int i = 0;
		int j = 0;
		int frame = 0;	
		bool init_flag = true;
	
		while ( getline(file, line, '\n'))
		{
			if (line.length() < 45) // 代表 header
			{
				//std::cout << line << endl;
				getline(file, line, '\n'); // 讀下一行
				i = 0;
				j = 0;
				frame++;
				if (frame==2)
					init_flag = false;
			}
			istringstream templine(line); // string 轉換成 stream
			string data;
			while ( getline(templine, data, ',')) // 讀檔讀到逗號
			{
				int tmp = atof(data.c_str());
				if (init_flag)
				{
					matrix[i][j][csv_id] = tmp;  // string 轉換成 int
				}
				else
				{
					matrix[i][j][csv_id] += tmp;
				}
				//cout <<  *(*(*(matrix+i)+j)+csv_id)<<" nn ";
				j++;
				if (j == 22)
					j = 0;			
			}
			i++;	
		}
		file.close();



		for (int ii = 0; ii < size[0]; ii++)
			for (int jj = 0; jj < size[1]; jj++)
			{
				//cout << matrix[ii][jj][csv_id]<< ' ';
				matrix[ii][jj][csv_id] = matrix[ii][jj][csv_id] / frame;
			}

	

		return 0;
	}


	double calibrationdll::ADC2KG(double* matrix, int ADC, int* loc, int csv_amount, double* KG_ary, int* size)
	{
		int ADC_adj = 0;
		double* adc_ary;
		double KG = 0;
		// get sensor(x,y)  adc_ary
		adc_ary = new double[csv_amount];
		for (int i = 0; i < csv_amount; i++)
		{
			adc_ary[i] = matrix[i* size[0] * size[1] + loc[0] * size[1] + loc[1]];
			/*int arr[] = {  0,0,0, 18, 24,  24 };
			adc_ary[i] = arr[i];
			cout << " " << adc_ary[i]<<endl;*/
		}
		// 輸出新的adc_ary
		//
		int idx = Sort_Idx(adc_ary, ADC, csv_amount);

		double modi_slope = 0;
		double modi_slope_old = 0;
		double* slope_ary = new double[csv_amount];
		//修正斜率ary
		for (int idcc = 0; idcc < csv_amount; idcc++)
		{
			//ADC 連接原點
			if (idcc == 0 && adc_ary[0] != 0)
				modi_slope = (adc_ary[0]) / (KG_ary[0]);
			else if (idcc == 0 && adc_ary[0] == 0)
				modi_slope = 0;
			//中間線段  
			else
				modi_slope = (adc_ary[idcc] - adc_ary[idcc - 1]) / (KG_ary[idcc] - KG_ary[idcc - 1]);
			//輸出一個slope array	
			slope_ary[idcc] = modi_slope;
		}
		//
		if (idx == csv_amount) // when ADC is bigger than ADC_arr_max_value 
			KG = (1/slope_ary[idx - 1])*(ADC - adc_ary[idx - 1]) + KG_ary[idx - 1];
		else if (idx == 0)  // when ADC is smaller than ADC_arr_max_value，跟原點連線
			KG = (1/slope_ary[0])*(ADC);
		else
			KG = (1/slope_ary[idx])*(ADC - adc_ary[idx]) + KG_ary[idx];

		if (KG < 0)
			KG = 0;
		if (ADC == 0) // 例外狀況: ADC是0  輸出也必須是0 
			KG = 0;

	
		delete(adc_ary);

		return KG;
	}


	int calibrationdll::Sort_Idx(double* arr, int ADC, int csv_amount)
	{
		
		for (int t = 0; t < csv_amount; t++)
		{
			if (t == csv_amount - 1 & ADC >= arr[t])  //ADC大於 arr的最大一項
				return t+1;
			else if (t == 0 & ADC <= arr[0]) //ADC小於 arr的最小一項
				return 0;
			else if (ADC >= arr[t] & ADC < arr[t+1]) //return 後一項的id
				return t+1 ;
		}
			
		return 0;
	}


	double calibrationdll::KG2caliADC(int*xy, double slope, double KG)
	{
		// x : KG, y : ADC , xy經過原點
		double ADC = (KG - xy[0])*slope + (double)xy[1];

		return ADC;
	}


	double calibrationdll::Slope_calculation(double* matrix, int* sensor_size, int csv_count, double* KG_ary)
	{
		double slope = 0;
		double max_adc = 0;
		int size_h = sensor_size[0];
		int size_w = sensor_size[1];
		int csv_id = csv_count-1;
		for (int high = 0; high < size_h; high++)
			for (int wid = 0; wid < size_w; wid++)	
				max_adc += matrix[csv_id*size_h*size_w + high*size_w + wid ] ;

		max_adc = max_adc / (size_w * size_h);
		slope = max_adc / KG_ary[csv_id] ;
		
		
		return  slope ;
	}

	int *LUT; // 一維當三維用
	int* LUT_encrypt;
	int* LUT_decrypt;
	int *mat_size;
	double*  org_matrix;
	int save_KG_len;
	double* save_KG_ary;

	int calibrationdll::LUT_init( char* pw)
	{	
		if (strcmp(pw, "password") == 0)
			cout << "-------password correct--------" << endl;
		else
			cout << "///////password incorrect////////" << endl;

		//int init_h = 50;
		//int init_w = 50;
		int init_KG_len = 10;
		//for big map testing
		int init_h = 300;
		int init_w = 300;

		LUT = (int*)malloc(init_h*init_w * 256 * sizeof(int)); // 創建global變數
		mat_size = (int*)malloc(2 * sizeof(int));
		org_matrix = (double*)malloc(init_h*init_w*init_KG_len*sizeof(double));
		save_KG_len = (int)malloc(sizeof(int));
		save_KG_ary = (double*)malloc(init_KG_len*sizeof(double));
		
		//encrypt//decrpyt
		LUT_encrypt = (int*)malloc(init_h*init_w * 256 * sizeof(int));
		LUT_decrypt = (int*)malloc(init_h*init_w * 256 * sizeof(int));


		return 0;
	
	}

	int calibrationdll::Load_Frame_Info(int* input_size,int KG_len, double* KG_ary, double* matrix )
	{
		int h = input_size[0];
		int w = input_size[1];
		mat_size[0] = h;
		mat_size[1] = w;
		save_KG_len = KG_len;

		for (int idx = 0; idx < h*w*save_KG_len; idx++)
		{
			org_matrix[idx] = matrix[idx];
			//cout << matrix[idx] << " " << org_matrix[idx] << endl;
		}
		for (int i = 0; i < KG_len; i++)
			save_KG_ary[i] = KG_ary[i];	

		return 0;	
	}
	


	int calibrationdll::Generate_LUT( int* LUT_out, double* out_slope)
	{
		cout << "LUT_init" << endl;
		double caliADC = 0;
	
		int data_num = save_KG_len; //  is   6 
		int h = mat_size[0];
		int w = mat_size[1];
		
		//
		//get slope
		
		double slope = Slope_calculation( org_matrix, mat_size, data_num, save_KG_ary);
		// testing LUT 


		cout <<"slope: "<< slope << endl;
		int xy[2] = { 0, 0 }; // xy 代表點斜式的座標
		//ADC_2_KG
		//KG_2_ADC
		int loc[2];
		int ch_num = 256;
		for (int height = 0; height < h; height++)
			for (int wide = 0; wide < w; wide++)
				for (int channel = 0; channel < ch_num; channel++)
				{
					loc[0] = height;
					loc[1] = wide;				
					LUT[height*w*ch_num + wide*ch_num + channel] = (int)(KG2caliADC(xy, slope, ADC2KG(org_matrix, channel, loc, data_num, save_KG_ary, mat_size)) + 0.5);
					LUT_out[ height*w*ch_num + wide*ch_num + channel] = LUT[ height*w*ch_num + wide*ch_num + channel];
				}
		encrypt_LUTfile(LUT, LUT_encrypt);
		
		out_slope[0] = slope;
		return 0;
	}

	int calibrationdll::Load_LUT(int* _LUT)
	{
		for (int i = 0; i < mat_size[0] * mat_size[1] * 256; i++)
			LUT[i] = _LUT[i];

		return 0;
	
	}

	int calibrationdll::Do_Calibration(int* in_frame, int* out_frame) // input new frame.   using global var: LUT, mat size.
	{
		int h = mat_size[0];
		int w = mat_size[1];

		decrypt_LUTfile( LUT_encrypt, LUT_decrypt);

		for (int height = 0; height < h; height++)
			for (int wid = 0; wid < w; wid++)
			{
				out_frame[height*w + wid] = LUT_decrypt[height*w * 256 + wid * 256 + in_frame[height*w + wid]];
			}
		return 0;
	}

	int calibrationdll::Release_LUT_Memory()
	{
		free(LUT);
		free(mat_size);
		free(org_matrix);
		free(save_KG_ary);
		//free(main_slope);

		return 0;
	}
	// END of ORIGINAL

	//********************************************************************************************************************************************
	//********************************************************************************************************************************************
	
	//NEW with SLOPE PROCESS
	double* Slope_LUT;
	//double*  org_matrix;
	//int save_KG_len;
	double* main_slope;
	//double* save_KG_ary;
	
	
	double calibrationdll::surround_point(int* sensor_loc, int idx)
	{
		double avg_surround_value = 0;
		int h = mat_size[0];
		int w = mat_size[1];
		int boundary[4] = { 0, 0, h, w };
		//判斷目標點是否在邊界
		if (sensor_loc[0] == 0)
			boundary[0] = 1;
		if (sensor_loc[1] == 0)
			boundary[1] = 1;
		if (sensor_loc[0] == h)
			boundary[2] = 1;
		if (sensor_loc[1] == w)
			boundary[3] = 1;

		int sum = 0;
		for (int i = 0; i < 4; i++)
			if (boundary[i] == 1)
				sum++;
		if (sum == 0) //不是邊界 累加8個邊界
		{
			for (int i = -1; i < 2; i++)
				for (int j = -1; j < 2; j++)
					if ((i != 0) || (j != 0))
						avg_surround_value += org_matrix[idx* h * w + (sensor_loc[0] + i) * w + (sensor_loc[1] + j)];
		}
		else{ //共有八總狀況
			for (int i = -1; i < 2; i++)
				for (int j = -1; j < 2; j++)
					if ((i != 0) || (j != 0))
						if (((sensor_loc[0] + i) >= 0) && ((sensor_loc[1] + j) >= 0) && ((sensor_loc[0] + i) < h) && ((sensor_loc[1] + j) < w))
							avg_surround_value += org_matrix[idx* h * w + (sensor_loc[0] + i) * w + (sensor_loc[1] + j)];
		}

		double denominator;
		if (sum == 0)
			denominator = 8.0;
		else if (sum == 1)
			denominator = 5.0;
		else if (sum = 2)
			denominator = 3.0;

		//這裡是取平均 或者我可以取中位數，避免被極值影響過大
		//cout << "avg_sur " << avg_surround_value << ", sum " << sum << endl;
		return avg_surround_value / denominator;
	}


	int * calibrationdll::find_bad_point( int* sensor_loc, int* test_ary_in)
	{
		int h = mat_size[0];
		int w = mat_size[1];
		double avg_slope = 0;
		int slpoe_count = 0;
		int *reutrn_ary = new int[2];
			reutrn_ary[0] = -1; // case -1 :沒有壞點
			reutrn_ary[1] = -1;

		int *adc_ary = new int[save_KG_len];
		for (int i = 0; i < 6; i++)
			adc_ary[i] = (int)(org_matrix[i *h * w + sensor_loc[0] * w + sensor_loc[1]] + 0.5);
		
		bool dll_test = true;
		if (dll_test)
			adc_ary = test_ary_in;


		double sdpoint_threshold_precent = 0.6; // 單點過大或過小
		double sdpoint_sensitive_threshold_precent = 0.3;  //累積多點都超過此比例，則認為是敏感或遲鈍點
		double sensitive_point_count_threshold = save_KG_len - 2;
		bool sensitive_detection = false;
		int sensitive_point_count = 0;
		for (int i = 0; i < save_KG_len - 1; i++)
		{
			double sd_id_value = surround_point(sensor_loc, i);
			if (abs(adc_ary[i] - sd_id_value) > sd_id_value * sdpoint_sensitive_threshold_precent) // 目標點與周圍點的差距，超過周圍點的0.5倍
				sensitive_point_count += 1; // 多加一個過於敏感或是遲鈍的點
		}

		//判斷: 是否較敏感或遲鈍，部參考周圍的點
		if (sensitive_point_count > (sensitive_point_count_threshold )) //發現敏感或遲鈍sensor
			sensitive_detection = true;

		for (int idx = 0; idx < save_KG_len - 1; idx++)
		{
			if ((adc_ary[idx + 1] - adc_ary[idx]) < 0)
			{ // 條件: 目前的 idx 與下一個點的斜率，出現負斜率...

				//判斷: 是否較敏感或遲鈍，部參考周圍的點			
				if (sensitive_detection)
				{
					//針對問題: 先不用周圍的點，判斷當負斜率出現，要修正的前點還是後點
					double fp_s;	//bp => id
					double ep_s; 	//bp => id +1

					if (idx == 0){
						fp_s = adc_ary[0] / save_KG_ary[0];
						ep_s = adc_ary[1] / save_KG_ary[1];
					}
					else{
						fp_s = (adc_ary[idx] - adc_ary[idx - 1]) / (save_KG_ary[idx] - save_KG_ary[idx - 1]);
						ep_s = (adc_ary[idx + 1] - adc_ary[idx - 1]) / (save_KG_ary[idx + 1] - save_KG_ary[idx - 1]);
					}
					double avg_s = 0; //去除 前後點的平均		
					for (int i = 0; i < save_KG_len; i++){
						if (idx == 0) // 跳過 0, 1
						{
							if (i == 0 || i == 1)
								continue;
							else if (i == 2)
								avg_s += (adc_ary[i]) / (save_KG_ary[i]);
							else{
								avg_s += (adc_ary[i] - adc_ary[i - 1]) / (save_KG_ary[i] - save_KG_ary[i - 1]);
							}
						}
						else if (idx == 4) // 跳過 4, 5
						{
							if (i == 4 || i == 5)
								continue;
							else if (i == 0)
								avg_s += (adc_ary[0]) / (save_KG_ary[0]);
							else
								avg_s += (adc_ary[i] - adc_ary[i - 1]) / (save_KG_ary[i] - save_KG_ary[i - 1]);
						}
						else {
							if (i == idx || i == idx + 1)
								continue;
							else if (i == 0)
								avg_s += (adc_ary[0]) / (save_KG_ary[0]);
							else if (i == idx + 2)
								avg_s += (adc_ary[i] - adc_ary[i - 3]) / (save_KG_ary[i] - save_KG_ary[i - 3]);
							else
								avg_s += (adc_ary[i] - adc_ary[i - 1]) / (save_KG_ary[i] - save_KG_ary[i - 1]);
						}
					}
					avg_s = avg_s / 4.0;

					//選擇是前點錯，還是後點錯
					if (abs(avg_s - fp_s) > abs(avg_s - ep_s))  //成立代表 前點的落差較大
					{
						reutrn_ary[0] = 1; // case 1 : 敏感或遲鈍點，不考慮周圍點
						reutrn_ary[1] = idx;
						return reutrn_ary;
					}
					else
					{
						reutrn_ary[0] = 1; // 
						reutrn_ary[1] = idx+1;
						return reutrn_ary;
					}
				}
				else
				{
					double sd_id_value_ = surround_point(sensor_loc, idx);
					double sd_idp1_value_ = surround_point(sensor_loc, idx + 1);
					//只要太大或太小就是壞點

					if (abs(adc_ary[idx] - sd_id_value_) > sd_id_value_ * sdpoint_threshold_precent)
					{
						reutrn_ary[0] = 0; // case 0 : 單純是點 超過(周圍點 x threshold)的值
						reutrn_ary[1] = idx;
						return reutrn_ary;
					}
					else if (abs(adc_ary[idx + 1] - sd_idp1_value_) > sd_idp1_value_ * sdpoint_threshold_precent)
					{
						reutrn_ary[0] = 0;
						reutrn_ary[1] = idx + 1;
						return reutrn_ary;
					}

			
					if (sd_id_value_ < sd_idp1_value_){
						if (abs(sd_id_value_ - adc_ary[idx]) > abs(sd_idp1_value_ - adc_ary[idx + 1]))
						{
							reutrn_ary[0] = 2; // case 2: 正常狀態
							reutrn_ary[1] = idx;
							return reutrn_ary;
						}
						else{
							reutrn_ary[0] = 2; // 
							reutrn_ary[1] = idx + 1;
							return reutrn_ary;
						}
					}
				}
			}
		}


		return reutrn_ary; // 代表沒有bad_point

	}

	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD CursorPosition;
	void gotoXY(int x, int y)
	{
		CursorPosition.X = x;
		CursorPosition.Y = y;
		SetConsoleCursorPosition(console, CursorPosition);
	}

	//修正org_matrix中的資料
	int calibrationdll::fix_the_bad_point( int* sensor_loc, int* bp  , double* test_in_matix)
	{
		int h = mat_size[0];
		int w = mat_size[1];
		int bp_case = bp[0];
		int idx = bp[1];

		int *adc_ary = new int[save_KG_len];
		for (int i = 0; i < 6; i++)
			adc_ary[i] = (int)(org_matrix[i *h * w + sensor_loc[0] * w + sensor_loc[1]] + 0.5);
		
		// bad point range(0-5

		if(adc_ary[idx] == 0)
			adc_ary[idx] = 0;

		else if (bp_case == 0 ) // 與周圍相差太多的壞點，直接用周圍職取代
		{
			org_matrix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = surround_point(sensor_loc, idx);
			test_in_matix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = surround_point(sensor_loc, idx);

		}
		else if (bp_case == 1) // 敏感or遲鈍點
		{
			if (idx == save_KG_len - 1)
			{
				org_matrix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = adc_ary[idx - 1] + 
					(adc_ary[idx - 1] - adc_ary[idx - 2]) / (save_KG_ary[idx - 1] + save_KG_ary[idx - 2])*(save_KG_ary[idx] - save_KG_ary[idx - 1]);
				test_in_matix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = adc_ary[idx - 1] +
					(adc_ary[idx - 1] - adc_ary[idx - 2]) / (save_KG_ary[idx - 1] + save_KG_ary[idx - 2])*(save_KG_ary[idx] - save_KG_ary[idx - 1]);

			}
			else if (idx < save_KG_len - 1 && idx> 0)
			{
				org_matrix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = adc_ary[idx - 1] +
					(adc_ary[idx + 1] - adc_ary[idx - 1]) * ((save_KG_ary[idx] - save_KG_ary[idx - 1]) / (save_KG_ary[idx + 1] - save_KG_ary[idx - 1]));
				test_in_matix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = adc_ary[idx - 1] +
					(adc_ary[idx + 1] - adc_ary[idx - 1]) * ((save_KG_ary[idx] - save_KG_ary[idx - 1]) / (save_KG_ary[idx + 1] - save_KG_ary[idx - 1]));

			}
			else if (idx == 0)
			{
				org_matrix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = adc_ary[1] / (save_KG_ary[1]) * save_KG_ary[0];
				test_in_matix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = adc_ary[1] / (save_KG_ary[1]) * save_KG_ary[0];
			}
		
		}
		else if (bp_case == 2) // 正常情況
		{
				if (idx == save_KG_len - 1)
				{		
					org_matrix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = surround_point(sensor_loc, idx);
					test_in_matix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = surround_point(sensor_loc, idx);
				}
				else if (idx < save_KG_len - 1 && idx> 0)
				{				
					org_matrix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = surround_point(sensor_loc, idx);
					test_in_matix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = surround_point(sensor_loc, idx);
				}
				else if (idx == 0)
				{	
					org_matrix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = surround_point(sensor_loc, idx);
					test_in_matix[idx* h * w + sensor_loc[0] * w + sensor_loc[1]] = surround_point(sensor_loc, idx);
				}
		}
		
		return 0;
	}
	

	int calibrationdll::Get_mem_Matrix(double* in_matrix)
	{
		int h = mat_size[0];
		int w = mat_size[1];
		//for (int idx = 0; idx < h*w*save_KG_len; idx++)
		//{
		//	
		//	in_matrix[idx] = org_matrix[idx];
		//	//cout << matrix[idx] << " " << org_matrix[idx] << endl;
		//}
		copy(org_matrix, org_matrix + (h*w*save_KG_len), in_matrix);

		return 0;
	}

	int calibrationdll::Save_Slope_LUT_init(int* input_size, int KG_len, double* KG_ary, double *matrix)
	{
		int h = input_size[0];
		int w = input_size[1];
		// init memory space
		main_slope = (double*)malloc(sizeof(double));
		save_KG_len = (int)malloc(sizeof(int));
		org_matrix = (double*)malloc(h*w*KG_len*sizeof(double));
		Slope_LUT = (double*)malloc(h*w * KG_len * sizeof(double)); // 創建global變數
		mat_size = (int*)malloc(2 * sizeof(int));
		save_KG_ary = (double*)malloc(KG_len*sizeof(double));
		mat_size[0] = h;
		mat_size[1] = w;

		// init value
		for (int idx = 0; idx < h*w*KG_len; idx++)
		{
			org_matrix[idx] = matrix[idx];
			//cout << matrix[idx] << " " << org_matrix[idx] << endl;
		}
		for (int i = 0; i < KG_len; i++)
			save_KG_ary[i] = KG_ary[i];

		save_KG_len = KG_len;

		return 0;
	}

	double* slope_preprocess_save( double* adc_ary)
	{
		double* slope_ary = new double[save_KG_len ];
		for (int i = 0; i < save_KG_len ; i++) // 斜率 KG / ADC
			if (i == 0)
				slope_ary[i] = (adc_ary[i] - 0) / (save_KG_ary[i] - 0);
			else
				slope_ary[i] = ( (adc_ary[i] - adc_ary[i - 1]) / (save_KG_ary[i] - save_KG_ary[i - 1]) );

		return slope_ary;
	}


	int calibrationdll::Save_Slope_LUT_Generate(double* out_slope)
	{
		cout << "Save_Slope_Calibration_init" << endl;
		int h = mat_size[0];
		int w = mat_size[1];
		//double adc_ary;

		//get slope
		double dst_slope = Slope_calculation(org_matrix, mat_size, save_KG_len, save_KG_ary);
		out_slope[0] = dst_slope; // slope輸出

		double *adc_ary = new double[save_KG_len];
		int* sensor_loc = new int[2];
		for (int height = 0; height < h; height++)
			for (int wide = 0; wide < w; wide++)
			{
				for (int i = 0; i < save_KG_len; i++)
					adc_ary[i] = org_matrix[i* h * w + height * w + wide];
				sensor_loc[0] = height;
				sensor_loc[1] = wide;
							
				double* slope_ary = slope_preprocess_save(adc_ary);
				for (int sid = 0; sid < save_KG_len; sid++){
					Slope_LUT[ sid*h*w + height*w +wide ] = slope_ary[sid];
				}
			}
	
		main_slope[0] = dst_slope;
		return 0;
	}

	double *adc_ary_t = new double[save_KG_len]; //宣告從for迴圈中拿到這裡，省了非常多時間
	int xy[2] = { 0, 0 }; // xy 代表點斜式的座標

	int calibrationdll::Save_Slope_Do_calibration(int* in_frame, int* out_frame )
	{
		int h = mat_size[0];
		int w = mat_size[1];
		double KG;

		int ADC;
		int idx;
		for (int height = 0; height < h; height++)
			for (int wid = 0; wid < w; wid++)
			{
				ADC = in_frame[height*w + wid];	//target ADC			
				for (int i = 0; i < save_KG_len; i++)
					adc_ary_t[i] = org_matrix[i* h * w + height * w + wid];				
				//adc_ary_t = org_matrix3D[height][wid];
							
				for (int t = 0; t < save_KG_len; t++)
				{
					if (ADC >= adc_ary_t[save_KG_len - 1])  //ADC大於 arr的最大一項
					{
						idx = t + 1;
						break;
					}
					else if (ADC <= adc_ary_t[0]) //ADC小於 arr的最小一項
					{
						idx = 0;
						break;
					}
					else if (ADC >= adc_ary_t[t] & ADC < adc_ary_t[t + 1]) //return 後一項的id
					{
						idx = t + 1;
						break;
					}
				}


				if (idx == save_KG_len) // when ADC is bigger than ADC_arr_max_value 
					KG = ( 1/ Slope_LUT[(idx - 1)*h*w + height*w + wid] )*(ADC - adc_ary_t[idx - 1]) + save_KG_ary[idx - 1];
				else if (idx == 0)  // when ADC is smaller than ADC_arr_max_value，跟原點連線
					KG = ( 1/ Slope_LUT[0 * h*w + height*w + wid] )*(ADC);
				else
					KG = ( 1/ Slope_LUT[idx*h*w + height*w + wid] )*(ADC - adc_ary_t[idx - 1]) + save_KG_ary[idx - 1];

				if (KG < 0)
					KG = 0;
				if (ADC == 0) // 例外狀況: ADC是0  輸出也必須是0 
					KG = 0;
				
				
				out_frame[height*w + wid] = (int)((KG - xy[1])*( main_slope[0] ) + (double)xy[0] + 0.5);
				
		}

		//int height = 0;
		//int wid = 5;
		//cout << "tt2:"<<endl;
		//int ADC = in_frame[height*w + wid];
		//cout << "ADC " << ADC << endl;
		//double *adc_ary = new double[save_KG_len];
		//for (int i = 0; i < save_KG_len; i++)
		//{
		//	adc_ary[i] = org_matrix[i* h * w + height * w + wid];
		//	cout << adc_ary[i]<<",";
		//}
		//cout << endl;
		//int idx = Sort_Idx(adc_ary, ADC, save_KG_len);
		//
		//cout <<"idx "<<idx<< endl;
		//cout << "adc_ary " << adc_ary[idx-1] << endl;
		//cout << "save_KG_ary " << save_KG_ary[idx-1] << endl;
		//KG = (Slope_LUT[idx*h*w + height*w + wid])*(ADC - adc_ary[idx-1]) + save_KG_ary[idx-1];
		//cout << "slope " << Slope_LUT[idx*h*w + height*w + wid] << ",  " << ((save_KG_ary[idx] - save_KG_ary[idx - 1] )/ (adc_ary[idx] - adc_ary[idx - 1])) << endl;
		//cout << "KG " << KG << endl;
		//int xy[2] = { 0, 0 }; // xy 代表點斜式的座標
		//ADC_modified = (double)((KG - (double)xy[1])*(1 / main_slope[0]) + (double)xy[0] + 0.5);
		//cout <<"ADC_modified "<< ADC_modified << endl;	
		

		return 0;
	}


	int calibrationdll::Release_Slope_LUT_Memory()
	{
		free(Slope_LUT);
		free(mat_size);
		free(org_matrix);
		free(save_KG_ary);
		free(main_slope);
		return 0;
	}


	int calibrationdll::doub2int(double in)
	{
		int out =(int)( in+0.5) ;
		
		return out;
	}

	int key = 40;
	int calibrationdll::encrypt_LUTfile(int* LUT_org, int* LUT_encryption)
	{

		int num_of_lut_element = (mat_size[0] * mat_size[1] * 256);
		for (int i = 0; i < num_of_lut_element; i++)
		{
			LUT_encryption[(i + key) % num_of_lut_element] = LUT_org[i] + i;

		}
		return 0;
	}

	int calibrationdll::decrypt_LUTfile(int* LUT_encryption, int* LUT_org)
	{
		int num_of_lut_element = (mat_size[0] * mat_size[1] * 256);
		for (int i = 0; i < num_of_lut_element; i++)
		{
			LUT_org[i] = LUT_encryption[(i + key) % num_of_lut_element] - i;

		}
		return 0;
	}


	Mat init_mat,
		resize_mat,
		blur_mat,
		pass_mat;
	int calibrationdll::Frame_load(uint32_t* in_data_size, uint32_t* in_data_ary) // Initial:input( yes/no, 原始圖形的長寬) 初始化一個2000*2000 unsigned 16 bit 的整數
	{
		///pass_mat.create(in_data_size[0], in_data_size[1], CV_32F);
		float* float_frame;
		float_frame = (float*)malloc(in_data_size[0] * in_data_size[1] * sizeof(float));
		std::copy(in_data_ary, in_data_ary + (in_data_size[0] * in_data_size[1]), float_frame);
		cv::Mat read_mat(in_data_size[0], in_data_size[1], CV_32F, float_frame);
		
		pass_mat = read_mat.clone();
		for (int i = 5; i < 10; i++)
		{
			for (int j = 5; j < 10; j++)
			{
				cout << pass_mat.at<float>(i, j) << " ";
			}
			cout << endl;
		}cout << endl;
		return  0;
	}

	float* outtt;
	int calibrationdll::Frame_resize(uint32_t* resize_shape, float* out) //Interpolation : input(data array 1D, 原始長寬, 放大倍率, output array 1D)
	{
		cv::resize(pass_mat, pass_mat, cv::Size(resize_shape[0], resize_shape[1]), 0, 0, CV_INTER_LINEAR);
		
		for (int i = 0; i < 10; i++)
		{
		for (int j = 0; j < 10; j++)
		{
		cout << pass_mat.at<float>(i, j) << " ";
		}
		cout << endl;
		}cout << endl;
		outtt = (float*)pass_mat.data;
		memcpy(out, outtt, pass_mat.total()*sizeof(float));

		return 0;
	}


	int calibrationdll::Frame_blur(uint32_t kernal_size, uint32_t sigma, float* out)
	{

		cv::GaussianBlur(pass_mat, pass_mat, cv::Size(kernal_size, kernal_size), sigma);
		

		outtt = (float*)pass_mat.data;
		memcpy(out, outtt, pass_mat.total()*sizeof(float));

		for (int i = 0; i < 10; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				cout << out[i * 300 + j] << " ";
			}
			cout << endl;
		}cout << endl;

		return 0;
	}




	int calibrationdll::add(int a, int b, int c)
	{
		
			
		return a + b + c;
	}
	int calibrationdll::add(int a, int b)
	{
		return a + b ;
	}

}