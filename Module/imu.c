#include "imu.h"
#include "mpu6050.h"
#include "param_save.h"
#include <string.h>
#include "Filter.h"
#include <math.h>
#include "ahrs.h"
#include "time_cnt.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

//������˹�˲�����
static Butter_Parameter Gyro_Parameter;
static Butter_Parameter Accel_Parameter;
static Butter_Parameter Acce_Correct_Parameter;
//������˹�˲��ڲ�����
static Butter_BufferData Gyro_BufferData[3];
static Butter_BufferData Accel_BufferData[3];
static Butter_BufferData Butter_Buffer_Correct[3];
//������ԭʼ����
Vector3i_t accDataFilter;
Vector3i_t gyroDataFilter;
Vector3i_t acceCorrectFilter;
Vector3i_t gyroCorrectFilter;
float tempDataFilter;

/**********************************************************************************************************
*�� �� ��: imu_init
*����˵��: IMU��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void imu_init()
{
	//���ô������˲�����
	Set_Cutoff_Frequency(Sampling_Freq, 50,&Gyro_Parameter);
	Set_Cutoff_Frequency(Sampling_Freq, 60,&Accel_Parameter);
	Set_Cutoff_Frequency(Sampling_Freq, 1,&Acce_Correct_Parameter);
	//MPU6050��ʼ��
	MPU6050_Detect();
    MPU6050_Init();
}

/**********************************************************************************************************
*�� �� ��: get_imu_data
*����˵��: ��ȡIMU���ݲ��˲�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void get_imu_data()
{
	Vector3i_t accRawData;
	Vector3i_t gyroRawData;
	float tempRawData;
    
	//��ȡ���ٶȴ�����
	MPU6050_ReadAcc(&accRawData);
	//��ȡ�����Ǵ�����
	MPU6050_ReadGyro(&gyroRawData);
	//��ȡ�¶ȴ�����
	MPU6050_ReadTemp(&tempRawData);
	
	//����У׼
	accRawData.x = paramer_save_data.accel_x_scale * accRawData.x - paramer_save_data.accel_x_offset / ACCEL_SCALE;
	accRawData.y = paramer_save_data.accel_y_scale * accRawData.y - paramer_save_data.accel_y_offset / ACCEL_SCALE;
	accRawData.z = paramer_save_data.accel_z_scale * accRawData.z - paramer_save_data.accel_z_offset / ACCEL_SCALE;
    
	gyroRawData.x = gyroRawData.x - paramer_save_data.gyro_x_offset;
	gyroRawData.y = gyroRawData.y - paramer_save_data.gyro_y_offset;
	gyroRawData.z = gyroRawData.z - paramer_save_data.gyro_z_offset;
	
	//���������ݵ�ͨ�˲�
	gyroDataFilter.x = Butterworth_Filter(gyroRawData.x, &Gyro_BufferData[0], &Gyro_Parameter);
	gyroDataFilter.y = Butterworth_Filter(gyroRawData.y, &Gyro_BufferData[1], &Gyro_Parameter);
	gyroDataFilter.z = Butterworth_Filter(gyroRawData.z, &Gyro_BufferData[2], &Gyro_Parameter);
    
	//�����ǽ������ݲ��˲�
    gyroCorrectFilter.x = gyroRawData.x;
    gyroCorrectFilter.y = gyroRawData.y;
    gyroCorrectFilter.z = gyroRawData.z;
	
	//���ټƽ������ݵ�ͨ�˲�
	acceCorrectFilter.x = Butterworth_Filter(accRawData.x, &Butter_Buffer_Correct[0], &Acce_Correct_Parameter);
	acceCorrectFilter.y = Butterworth_Filter(accRawData.y, &Butter_Buffer_Correct[1], &Acce_Correct_Parameter);
	acceCorrectFilter.z = Butterworth_Filter(accRawData.z, &Butter_Buffer_Correct[2], &Acce_Correct_Parameter);
	
	//���ټ����ݵ�ͨ�˲�
	accDataFilter.x = Butterworth_Filter(accRawData.x, &Accel_BufferData[0], &Accel_Parameter);
	accDataFilter.y = Butterworth_Filter(accRawData.y, &Accel_BufferData[1], &Accel_Parameter);
	accDataFilter.z = Butterworth_Filter(accRawData.z, &Accel_BufferData[2], &Accel_Parameter);
	
	//�¶����ݲ��˲�
	tempDataFilter = tempRawData;
}

/**********************************************************************************************************
*�� �� ��: Calibrate_Reset_Matrices
*����˵��: ��ʼ���������
*��    ��: �ݶȾ��� Hessian����
*�� �� ֵ: ��
**********************************************************************************************************/
static void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
	int16_t j, k;
	for (j = 0; j < 6; j++) {
		dS[j] = 0.0f;
		for (k = 0; k < 6; k++) {
			JS[j][k] = 0.0f;
		}
	}
}

/**********************************************************************************************************
*�� �� ��: Calibrate_Find_Delta
*����˵��: ʹ�ø�˹��Ԫ������
*��    ��: �ݶȾ��� Hessian���� ��������
*�� �� ֵ: ��
**********************************************************************************************************/
static void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
	int16_t i, j, k;
	float mu;
	//�����Ԫ�������Է�����ת��Ϊ�����Ƿ�����
	for (i = 0; i < 6; i++) {
		//��JtJ[i][i]��Ϊ0����������JtJ[i][i]���µ�Ԫ����Ϊ0
		for (j = i + 1; j < 6; j++) {
			mu = JS[i][j] / JS[i][i];
			if (mu != 0.0f) {
				dS[j] -= mu * dS[i];
				for (k = j; k < 6; k++) {
					JS[k][j] -= mu * JS[k][i];
				}
			}
		}
	}
	//�ش��õ�������Ľ�
	for (i = 5; i >= 0; i--)
	{
		dS[i] /= JS[i][i];
		JS[i][i] = 1.0f;

		for (j = 0; j < i; j++) {
			mu = JS[i][j];
			dS[j] -= mu * dS[i];
			JS[i][j] = 0.0f;
		}
	}
	for (i = 0; i < 6; i++) {
		delta[i] = dS[i];
	}
}

/**********************************************************************************************************
*�� �� ��: Calibrate_Update_Matrices
*����˵��: ���������õ��ľ���
*��    ��: �ݶȾ��� Hessian���� ���̽� ����
*�� �� ֵ: ��
**********************************************************************************************************/
static void Calibrate_Update_Matrices(float dS[6], float JS[6][6], float beta[6], float data[3])
{
	int16_t j, k;
	float dx, b;
	float residual = 1.0;
	float jacobian[6];
	for (j = 0; j < 3; j++) {
		b = beta[3 + j];
		dx = (float)data[j] - beta[j];
		//����в� (���������̵����)
		residual -= b * b * dx * dx;
		//�����ſɱȾ���
		jacobian[j] = 2.0f * b * b * dx;
		jacobian[3 + j] = -2.0f * b * dx * dx;
	}

	for (j = 0; j < 6; j++) {
		//���㺯���ݶ�
		dS[j] += jacobian[j] * residual;
		for (k = 0; k < 6; k++) {
			//����Hessian���󣨼���ʽ��ʡ�Զ���ƫ���������ſɱȾ�������ת�õĳ˻�
			JS[j][k] += jacobian[j] * jacobian[k];
		}
	}
}

/**********************************************************************************************************
*�� �� ��: Calibrate_accel
*����˵��: ��˹ţ�ٷ���⴫�������̣��õ�У׼����
*��    ��: �������������ݣ�6�飩 ��ƫ���ָ�� �������ָ��
*�� �� ֵ: 0��ʧ�� 1���ɹ�
**********************************************************************************************************/
static uint8_t Calibrate_accel(Vector3f_t accel_sample[6], Vector3f_t *accel_offsets, Vector3f_t *accel_scale)
{
	int16_t i;
	int16_t num_iterations = 0;
	float eps = 0.000000001;
	float change = 100.0;
	float data[3] = {0};
	//���̽�
	float beta[6] = {0};
	//��������
	float delta[6] = {0};
	//�ݶȾ���
	float ds[6] = {0};
	//Hessian����
	float JS[6][6] = {0};
	//�趨���̽��ֵ
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1.0f / GRAVITY_MSS;
	//��ʼ����������������С��epsʱ�������㣬�õ����̽������Ž�
	while (num_iterations < 20 && change > eps)
	{
		num_iterations++;
		//�����ʼ��
		Calibrate_Reset_Matrices(ds, JS);

		//�������̺������ݶ�JtR��Hessian����JtJ
		for (i = 0; i < 6; i++) {
			data[0] = accel_sample[i].x;
			data[1] = accel_sample[i].y;
			data[2] = accel_sample[i].z;
			Calibrate_Update_Matrices(ds, JS, beta, data);
		}
		//��˹��Ԫ����ⷽ�̣�JtJ * delta = JtR���õ�delta
		Calibrate_Find_Delta(ds, JS, delta);
		//�����������
		change = delta[0] * delta[0] +
			delta[0] * delta[0] +
			delta[1] * delta[1] +
			delta[2] * delta[2] +
			delta[3] * delta[3] / (beta[3] * beta[3]) +
			delta[4] * delta[4] / (beta[4] * beta[4]) +
			delta[5] * delta[5] / (beta[5] * beta[5]);
		//���·��̽�
		for (i = 0; i < 6; i++) {
			beta[i] -= delta[i];
		}
	}
	//����У׼����
	accel_scale->x = beta[3] * GRAVITY_MSS;
	accel_scale->y = beta[4] * GRAVITY_MSS;
	accel_scale->z = beta[5] * GRAVITY_MSS;
	accel_offsets->x = beta[0] * accel_scale->x;
	accel_offsets->y = beta[1] * accel_scale->y;
	accel_offsets->z = beta[2] * accel_scale->z;

	//��׼�������
	if (fabsf(accel_scale->x - 1.0f) > 0.5f || fabsf(accel_scale->y - 1.0f) > 0.5f || fabsf(accel_scale->z - 1.0f) > 0.5f) {
		return 0;
	}
	if (fabsf(accel_offsets->x) > 5.0f || fabsf(accel_offsets->y) > 5.0f || fabsf(accel_offsets->z) > 5.0f) {
		return 0;
	}
	return 1;
}

/**********************************************************************************************************
*�� �� ��: accel_calibration
*����˵��: ���ټ�У׼
*��    ��: У׼��
*�� �� ֵ: ��
**********************************************************************************************************/
//��һ��ɿ�ƽ�ţ�Z�����������Ϸ�
//�ڶ���ɿ�ƽ�ţ�X�����������Ϸ�
//������ɿ�ƽ�ţ�X�����������·�
//������ɿ�ƽ�ţ�Y�����������·�
//������ɿ�ƽ�ţ�Y�����������Ϸ�
//������ɿ�ƽ�ţ�Z�����������·�
void accel_calibration(uint8_t where)
{
    //���ټ�У׼������6�����������
    static Vector3f_t acce_calibration_data[6];
    //���ټ�У׼״̬
    static uint8_t acce_calibration_flag;
    uint16_t num_samples;
    portTickType xLastWakeTime;
	UBaseType_t this_task_priority;
	Vector3f_t acce_sample_sum;
	Vector3f_t new_offset;
	Vector3f_t new_scales;
	
    //�������
    if (where > 6 || where < 1) {
        printf("��������\r\n");
        return;
    }
    
	//�����ز���
    acce_sample_sum.x = 0;
    acce_sample_sum.y = 0;
    acce_sample_sum.z = 0;
	paramer_save_data.accel_x_offset = 0;
	paramer_save_data.accel_y_offset = 0;
	paramer_save_data.accel_z_offset = 0;
	paramer_save_data.accel_x_scale = 1;
	paramer_save_data.accel_y_scale = 1;
	paramer_save_data.accel_z_scale = 1;
	
    //��߱��������ȼ�
    this_task_priority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    //��Ϊ��λ��У׼���������Ҫ�ȴ����ݸ���
    vTaskDelay(10 / portTICK_RATE_MS);
    //��ʼ��ʱ��
    xLastWakeTime = xTaskGetTickCount();
    for (num_samples = 0; num_samples < 1000; num_samples++) {
        //�����ټ������ۼ�
        acce_sample_sum.x += acceCorrectFilter.x * ACCEL_SCALE;
        acce_sample_sum.y += acceCorrectFilter.y * ACCEL_SCALE;
        acce_sample_sum.z += acceCorrectFilter.z * ACCEL_SCALE;
        //5ms���ڶ�ʱ
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
    //�����Ӧ��ļ��ٶȼ���
    acce_calibration_data[where - 1].x = acce_sample_sum.x / num_samples;
    acce_calibration_data[where - 1].y = acce_sample_sum.y / num_samples;
    acce_calibration_data[where - 1].z = acce_sample_sum.z / num_samples;
    acce_calibration_flag |= 1 << where;
    //�ָ����������ȼ�
    vTaskPrioritySet(NULL, this_task_priority);
    printf("x:%0.3f y:%0.3f z:%0.3f\r\n", acce_calibration_data[where - 1].x, acce_calibration_data[where - 1].y, acce_calibration_data[where - 1].z);
    printf("���ټƵ�%d������ɹ�\r\n", where);
    
    //ȫ����У׼���
    if (acce_calibration_flag == 0x7E) {
        //������6�����ݼ�����ٶ�У׼����
        if(Calibrate_accel(acce_calibration_data, &new_offset, &new_scales)) {
            //��������
            paramer_save_data.accel_x_offset = new_offset.x;
            paramer_save_data.accel_y_offset = new_offset.y;
            paramer_save_data.accel_z_offset = new_offset.z;
            paramer_save_data.accel_x_scale = new_scales.x;
            paramer_save_data.accel_y_scale = new_scales.y;
            paramer_save_data.accel_z_scale = new_scales.z;
            write_save_paramer();
            printf("���ټƽ����ɹ�\r\n");
        }
    }
}

/**********************************************************************************************************
*�� �� ��: gyro_calibration
*����˵��: ������У׼У׼
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void gyro_calibration()
{
    uint16_t gyro_calibration_flag;
	Vector3l_t gyroSumData;
	UBaseType_t this_task_priority;
    portTickType xLastWakeTime;
	
    //��λУ׼����
    gyroSumData.x = 0;
    gyroSumData.y = 0;
    gyroSumData.z = 0;
	paramer_save_data.gyro_x_offset = 0;
	paramer_save_data.gyro_y_offset = 0;
	paramer_save_data.gyro_z_offset = 0;
    
	//��߱��������ȼ�
	this_task_priority = uxTaskPriorityGet(NULL);
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    //��Ϊ��λ��У׼���������Ҫ�ȴ����ݸ���
    vTaskDelay(10 / portTICK_RATE_MS);
	//�õ���ʼʱ��
	xLastWakeTime = xTaskGetTickCount();
	for (gyro_calibration_flag = 0; gyro_calibration_flag < 400; gyro_calibration_flag++) {
		//�������������ۼ�
		gyroSumData.x += gyroCorrectFilter.x;
		gyroSumData.y += gyroCorrectFilter.y;
		gyroSumData.z += gyroCorrectFilter.z;
		//5ms���ڶ�ʱ
		vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
	}
	//ȡƽ��
	gyroSumData.x = gyroSumData.x / 400;
	gyroSumData.y = gyroSumData.y / 400;
	gyroSumData.z = gyroSumData.z / 400;
	//�ָ����������ȼ�
	vTaskPrioritySet(NULL, this_task_priority);
	//�������
	paramer_save_data.gyro_x_offset = gyroSumData.x;
	paramer_save_data.gyro_y_offset = gyroSumData.y;
	paramer_save_data.gyro_z_offset = gyroSumData.z;
	write_save_paramer();
    printf("\r\nx:%d y:%d z:%d\r\n", paramer_save_data.gyro_x_offset, paramer_save_data.gyro_y_offset, paramer_save_data.gyro_z_offset);
    printf("�����ǽ����ɹ�\r\n");
}