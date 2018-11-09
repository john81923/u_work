#define FOO_DLL_EXPORTS
#ifdef FOO_DLL_EXPORTS
#define UNEO_API __declspec(dllexport)
#else
#define UNEO_API __declspec(dllimport)
#endif




#include <Windows.h>
#include <math.h> 

#include<opencv2/core/core.hpp>
//#include<opencv2/highgui/highgui.hpp>
//#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include "uncaleo_x86_h.h"
#include <iostream>
#include <stdint.h>
 
#pragma comment(lib, "opencv_core2413d.lib")
#pragma comment(lib, "opencv_contrib2413d.lib")
#pragma comment(lib, "opencv_imgproc2413d.lib")


using namespace cv;

uint32_t Sort_Idx(double* arr, uint32_t ADC, uint32_t csv_amount)
{

	for (uint32_t t = 0; t < csv_amount; t++)
	{
		if (t == csv_amount - 1 && ADC >= arr[t])  //ADC大於 arr的最大一項
			return t + 1;
		else if (t == 0 && ADC <= arr[0]) //ADC小於 arr的最小一項
			return 0;
		else if (ADC >= arr[t] && ADC < arr[t + 1]) //return 前一項的id
			return t + 1;
	}

	return 0;
}


double KG2caliADC(uint32_t*xy, double slope, double KG)
{
	// x : ADC, y : KG
	return  (KG - xy[1])*( slope) + (double)xy[0];
}

double ADC2KG(double* matrix, uint32_t ADC, uint32_t* loc, uint32_t csv_amount, double* KG_ary, uint32_t* size)
{
	uint32_t ADC_adj = 0;
	double* adc_ary;
	double KG = 0;
	// get sensor(x,y)  adc_ary
	adc_ary = new double[csv_amount];
	for (uint32_t i = 0; i < csv_amount; i++)
	{
		adc_ary[i] = matrix[i* size[0] * size[1] + loc[0] * size[1] + loc[1]];
		/*uint32_t arr[] = {  0,0,0, 18, 24,  24 };
		adc_ary[i] = arr[i];
		cout << " " << adc_ary[i]<<endl;*/
	}
	// 輸出新的adc_ary
	//
	uint32_t idx = Sort_Idx(adc_ary, ADC, csv_amount);

	double modi_slope = 0;
	double modi_slope_old = 0;
	double* slope_ary = new double[csv_amount];
	//修正斜率ary
	for (uint32_t idcc = 0; idcc < csv_amount; idcc++)
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
		KG = (1 / slope_ary[idx - 1])*(ADC - adc_ary[idx - 1]) + KG_ary[idx - 1];
	else if (idx == 0)  // when ADC is smaller than ADC_arr_max_value，跟原點連線
		KG = (1 / slope_ary[0])*(ADC);
	else
		KG = (1 / slope_ary[idx])*(ADC - adc_ary[idx]) + KG_ary[idx];

	if (KG < 0)
		KG = 0;
	if (ADC == 0) // 例外狀況: ADC是0  輸出也必須是0 
		KG = 0;


	delete(adc_ary);

	return KG;
}

double Slope_calculation(double* matrix, uint32_t* sensor_size, uint32_t csv_count, double* KG_ary)
{
	double slope = 0;
	double max_adc = 0;
	uint32_t size_h = sensor_size[0];
	uint32_t size_w = sensor_size[1];
	uint32_t csv_id = csv_count - 1;
	for (uint32_t high = 0; high < size_h; high++)
		for (uint32_t wid = 0; wid < size_w; wid++)
			max_adc += matrix[csv_id*size_h*size_w + high*size_w + wid];

	max_adc = max_adc / (size_w * size_h);
	slope = max_adc / KG_ary[csv_id];


	return  slope;

}


uint32_t *Parameters_og; // 一維當三維用
double *Parameters_slp;
uint32_t *mat_size;
double*  org_matrix;
uint32_t save_KG_len;
double* save_KG_ary;
bool Load_Parameter_only = false;


UNEO_API uint32_t 
Init(uint32_t* init_size, uint32_t init_KG_len, char* pw)
{
	if (strcmp(pw, "TechUneo") == 0)		
	{
		uint32_t init_h = init_size[0];
		uint32_t init_w = init_size[1];
		
		mat_size = (uint32_t*)malloc(2 * sizeof(uint32_t));
		org_matrix = (double*)malloc(init_h*init_w*init_KG_len*sizeof(double));
		save_KG_len = (uint32_t)malloc(sizeof(uint32_t));
		save_KG_ary = (double*)malloc(init_KG_len*sizeof(double));
		return 0;
	}else
		return 1;

}
 
UNEO_API uint32_t
Set_Frame_info_slope(uint32_t* input_size, uint32_t KG_len, double* KG_ary, double* matrix)
{
	uint32_t h = input_size[0];
	uint32_t w = input_size[1];
	mat_size[0] = h;
	mat_size[1] = w;
	save_KG_len = KG_len;

	Load_Parameter_only = false;
	Parameters_slp = (double*)malloc(h* w * save_KG_len * sizeof(double)); // 創建global變數

	uint32_t frame_len = h*w*save_KG_len;
	std::copy(matrix, matrix + frame_len, org_matrix);

	for (uint32_t i = 0; i < KG_len; i++)
		save_KG_ary[i] = KG_ary[i];

	return 0;
}


UNEO_API uint32_t
Set_Frame_size(uint32_t* input_size)
{
	uint32_t h = input_size[0];
	uint32_t w = input_size[1];
	mat_size[0] = h;
	mat_size[1] = w;
	Load_Parameter_only = true;


	return 0;
}

double* slope_preprocess_save(double* adc_ary)
{
	double* slope_ary = new double[save_KG_len];
	for (int i = 0; i < save_KG_len; i++) // 
		if (i == 0)
			slope_ary[i] = (adc_ary[i] - 0.0) / (save_KG_ary[i] - 0.0);
		else
			slope_ary[i] = ((adc_ary[i] - adc_ary[i - 1]) / (save_KG_ary[i] - save_KG_ary[i - 1]));

	return slope_ary;
}


UNEO_API uint32_t
Get_mem_Matrix(double* in_matrix)
{
	if (Load_Parameter_only == true)
		return 1;
	uint32_t h = mat_size[0];
	uint32_t w = mat_size[1];
	uint32_t idx = h*w*save_KG_len;
	std::copy(org_matrix, org_matrix + idx, in_matrix);


	return 0;
}


UNEO_API uint32_t
Encrypt_file_slope(uint32_t* Parameter_org, uint32_t* Parameter_encryption)
{


	return 0;
}

uint32_t Decrypt_file_slope(double* Parameter_encryption, uint32_t* Parameter_org)
{
	// no encrytion or derytption for slope version
	//
	//uint32_t num_of_Parameter_element = (mat_size[0] * mat_size[1] * save_KG_len);
	//for (uint32_t i = 0; i < num_of_Parameter_element; i++)
	//	Parameter_org[i] = Parameter_encryption[(i + key) % num_of_Parameter_element] ^ key_c;

	return 0;
}



double gb_slope;


UNEO_API uint32_t
Parameter_Setting_slope(double* Parameter_outslope, double* out_slope) // input KG 1D-array ,ADC 2D-array,  matrix size.
{
	double caliADC = 0;
	uint32_t data_num = save_KG_len;
	uint32_t h = mat_size[0];
	uint32_t w = mat_size[1];
	//get slope
	gb_slope = Slope_calculation(org_matrix, mat_size, data_num, save_KG_ary);
	out_slope[0] = gb_slope;

	if (Load_Parameter_only == true)
		return 1;
	
		double *adc_ary = new double[save_KG_len];
		int* sensor_loc = new int[2];
		double* slope_ary = new double[save_KG_len];
		for (int height = 0; height < h; height++)
			for (int wide = 0; wide < w; wide++)
			{
				for (int channel = 0; channel < save_KG_len; channel++)
					adc_ary[channel] = org_matrix[channel* h * w + height * w + wide];

				sensor_loc[0] = height;
				sensor_loc[1] = wide;

				slope_ary = slope_preprocess_save(adc_ary);
				for (int sid = 0; sid < save_KG_len; sid++){
					Parameters_slp[sid*h*w + height*w + wide] = slope_ary[sid];
				}
			}
		std::copy(Parameters_slp, Parameters_slp + (data_num*h*w), Parameter_outslope);
		// NO slope LUT encrytption for now
		//Encrypt_file(Parameters_slp, Parameter_out);

	return 0;
}



UNEO_API uint32_t
Load_Parameter_slope(double* new_Parameter) // Load Parameters.
{
	Decrypt_file_slope(new_Parameter, Parameters_og);
	//memcpy(Parameters, new_Parameter, sizeof(uint32_t) * mat_size[0] * mat_size[1] * 256);

	return 0;
}

double *adc_ary_t = new double[save_KG_len]; //宣告從for迴圈中拿到這裡，省了非常多時間
uint32_t xy[2] = { 0, 0 }; // xy 代表點斜式的座標

UNEO_API uint32_t
Frame_trans_slope(uint32_t* in_frame, uint32_t* out_frame) // input new frame.   using global var: Parameters, mat size.
{
	int h = mat_size[0];
	int w = mat_size[1];
	std::cout << "h: " << h << std::endl;
	std::cout << "w: " << w << std::endl;

	
	double KG;
	double ADC;
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
				else if ((ADC >= adc_ary_t[t]) & (ADC < adc_ary_t[t + 1])) //return 後一項的id
				{
					idx = t + 1;
					break;
				}
			}

			if (idx == save_KG_len) // when ADC is bigger than ADC_arr_max_value 
				KG = (1.0 / Parameters_slp[(idx - 1)*h*w + height*w + wid]) * (ADC - adc_ary_t[idx - 1]) + save_KG_ary[idx - 1];
			else if (idx == 0)  // when ADC is smaller than ADC_arr_max_value，跟原點連線
			{

				KG = (1.0 / Parameters_slp[0 * h*w + height*w + wid]) * (ADC);
			}
			else
			{
				KG = (1.0 / Parameters_slp[idx*h*w + height*w + wid]) * (ADC - adc_ary_t[idx - 1]) + save_KG_ary[idx - 1];
			}
			if (KG < 0)
				KG = 0;
			if (ADC == 0) // 例外狀況: ADC是0  輸出也必須是0 
				KG = 0;

			out_frame[height*w + wid] = (uint32_t)((KG - xy[1])*(gb_slope)+(double)xy[0] + 0.5);
		}
	

	return 0;
}


UNEO_API uint32_t
Release_Memory_slope(uint32_t* out)
{
	
	free(Parameters_slp);
	free(mat_size);

	return 0;
}





