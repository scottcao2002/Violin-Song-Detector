/**
 * IMPLEMENTATION OF EMBEDDEDML IN LEARNING DEVICE ORIENTATION
 **/

/**
 ******************************************************************************
 * @file    DataLog/Src/main.c
 * @author  Central Labs
 * @version V1.1.1
 * @date    06-Dec-2016
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <string.h> /* strlen */
#include <stdio.h>  /* sprintf */
#include <math.h>   /* trunc */
#include "embeddedML.h"
#include "main.h"

#include "datalog_application.h"
#include "usbd_cdc_interface.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Data acquisition period [ms] */
#define DATA_PERIOD_MS 10

#define NUMBER_TEST_CYCLES 20
#define CLASSIFICATION_ACC_THRESHOLD 1
#define CLASSIFICATION_DISC_THRESHOLD 1.05
#define START_POSITION_INTERVAL 3000
#define TRAINING_CYCLES 2000
#define LED_BLINK_INTERVAL 200
#define ANGLE_MAG_MAX_THRESHOLD 30
#define MOTION_TIME_LIMIT 1000
#define MAX_ROTATION_ACQUIRE_CYCLES 2000

//#define NOT_DEBUGGING

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* SendOverUSB = 0  --> Save sensors data on SDCard (enable with double click) */
/* SendOverUSB = 1  --> Send sensors data via USB */
uint8_t SendOverUSB = 1;

USBD_HandleTypeDef USBD_Device;
static volatile uint8_t MEMSInterrupt = 0;
static volatile uint8_t no_H_HTS221 = 0;
static volatile uint8_t no_T_HTS221 = 0;
static volatile uint8_t no_GG = 0;

static RTC_HandleTypeDef RtcHandle;
static void *LSM6DSM_X_0_handle = NULL;
static void *LSM6DSM_G_0_handle = NULL;
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *LPS22HB_P_0_handle = NULL;
static void *LPS22HB_T_0_handle = NULL;
static void *HTS221_H_0_handle = NULL;
static void *HTS221_T_0_handle = NULL;
static void *GG_handle = NULL;

int xyz_initial[3], xyz_initial_prev[3];
int xyz_initial_filter[3], xyz_initial_filter_prev[3];
int xyz_initial_HP[3], xyz_initial_prev_HP[3];
int xyz_initial_filter_HP[3], xyz_initial_filter_prev_HP[3];

float angular_velocity_x_direct;
float angular_velocity_x_direct_filter = 0;
float angular_velocity_x_direct_prev = 0;
float angular_velocity_x_direct_prev_t = 0;
float angular_velocity_x_direct_filter_prev = 0;
float angular_displacement_filter, angular_displacement_filter_prev,
		angular_displacement_prev;

float angular_velocity_x_filter, angular_velocity_x_prev = 0,
		angular_velocity_x_filter_prev = 0;
float angular_displacement = 0;
/* Private function prototypes -----------------------------------------------*/

static void Error_Handler(void);
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void initializeAllSensors(void);

/* Private functions ---------------------------------------------------------*/

static volatile uint8_t hasTrained = 0;
unsigned int training_cycles = TRAINING_CYCLES;

void stable_softmax(float *x, float *y) {
	int size = 3;
	float multiplier = 1.0;

	int i;

	//softmax implemented as square law algorithm to be accommodate numerical precision requirements

	y[0] = (x[0] * x[0] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));
	y[1] = (x[1] * x[1] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));
	y[2] = (x[2] * x[2] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));

	for (i = 0; i < size; i++) {
		if (x[i] < 0.0)
			y[i] = y[i] * -1.0;
	}
}

void motion_softmax(int size, float *x, float *y) {
	y[0] = x[0] / 110;
	y[1] = x[1] / 110;
	y[2] = x[2] / 50;

	int i;
	for (i = 0; i < size; i++) {
		if (x[i] < 0.0)
			y[i] = y[i] * -1.0;
	}
}

void LED_Code_Blink(int count) {

	int i;

	for (i = 0; i < 7; i++) {
		BSP_LED_On(LED1);
		HAL_Delay(30);
		BSP_LED_Off(LED1);
		HAL_Delay(30);
	}

	if (count != 0) {
		HAL_Delay(1000);
		for (i = 0; i < count; i++) {
			BSP_LED_On(LED1);
			HAL_Delay(400);
			BSP_LED_Off(LED1);
			HAL_Delay(400);
		}
	}

	for (i = 0; i < 7; i++) {
		BSP_LED_On(LED1);
		HAL_Delay(30);
		BSP_LED_Off(LED1);
		HAL_Delay(30);
	}

}

void getAccel(void *handle, int *xyz) {
	uint8_t id;
	SensorAxes_t acceleration;
	uint8_t status;

	BSP_ACCELERO_Get_Instance(handle, &id);

	BSP_ACCELERO_IsInitialized(handle, &status);

	if (status == 1) {
		if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR) {
			acceleration.AXIS_X = 0;
			acceleration.AXIS_Y = 0;
			acceleration.AXIS_Z = 0;
		}

		xyz[0] = (int) acceleration.AXIS_X;
		xyz[1] = (int) acceleration.AXIS_Y;
		xyz[2] = (int) acceleration.AXIS_Z;
	}
}

void getAngularVelocity(void *handle, int *xyz) {
	uint8_t id;
	SensorAxes_t angular_velocity;
	uint8_t status;

	BSP_GYRO_Get_Instance(handle, &id);
	BSP_GYRO_IsInitialized(handle, &status);

	if (status == 1) {
		if (BSP_GYRO_Get_Axes(handle, &angular_velocity) == COMPONENT_ERROR) {
			angular_velocity.AXIS_X = 0;
			angular_velocity.AXIS_Y = 0;
			angular_velocity.AXIS_Z = 0;
		}
		xyz[0] = (int) angular_velocity.AXIS_X;
		xyz[1] = (int) angular_velocity.AXIS_Y;
		xyz[2] = (int) angular_velocity.AXIS_Z;
	}
}

void Feature_Extraction_Gyro(void *handle, int * ttt_1, int * ttt_2,
		int * ttt_3, int * ttt_mag_scale) {

	int ttt[3], ttt_initial[3], ttt_offset[3];
	int axis_index, sample_index;
	float rotate_angle[3];
	float Tsample;
	char msg1[256];
	uint32_t start_tick1, start_tick2;
	uint32_t final_tick1, final_tick2;


	/*
	 * Compute sample period with scaling from milliseconds
	 * to seconds
	 */

	Tsample = (float)(DATA_PERIOD_MS)/1000;

	/*
	 * Initialize rotation angle values
	 */

	for (axis_index = 0; axis_index < 3; axis_index++) {
		ttt[axis_index] = 0;
		rotate_angle[axis_index] = 0;
	}

	/*
	 * Rotation Rate Signal integration loop
	 *
	 * Note that loop cycle time is DATA_PERIOD_MS matching the SensorTile
	 * sensor sampling period
	 *
	 */

	/*
	 * Acquire Rotation Rate values prior to motion
	 *
	 * This includes the initial sensor offset value to be subtracted
	 * from all subsequent samples
	 */

	getAngularVelocity(handle, ttt_offset);


	start_tick1 = HAL_GetTick();
	final_tick1 = 5000;
	/*
	 * Notify user to initiate song
	 */


	BSP_LED_On(LED1);
	sprintf(msg1, "\r\nGet ready to play a song");
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

	/*
	 * Obtain initial angle right before starting song
	 */


	while ((HAL_GetTick()-start_tick1)<final_tick1) {

		/*
		 * Acquire initial sample value of rotation rate
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt_initial[axis_index] = ttt[axis_index];
		}

		/*
		 * Introduce integration time period delay
		 */

		HAL_Delay(DATA_PERIOD_MS);

		/*
		 * Acquire current sample value of rotation rate and remove
		 * offset value
		 */

		getAngularVelocity(handle, ttt);
		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt[axis_index] = ttt[axis_index] - ttt_offset[axis_index];
		}


		/*
		 * Suppress value of Y-Axis and Z-Axis rotation signals
		 */

		ttt_initial[2] = 0;
		ttt[2] = 0;
		ttt_initial[1] = 0;
		ttt[1] = 0;

		/*
		 * Compute rotation angles by integration
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			rotate_angle[axis_index] = rotate_angle[axis_index]
					+ (float)((ttt_initial[axis_index] + ttt[axis_index]) * Tsample / 2);
		}




	}
	/*
	 * Store initial angle as first neuron
	 */
	*ttt_1 = rotate_angle[0] / (1000);
	start_tick2 = HAL_GetTick();
	final_tick2 = 10000;
	sprintf(msg1, "\r\nBegin playing the song. Go back to rest position when finished:");
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
	/*
	 * Obtain intermediate angle 20 seconds into the song
	 */
	while ((HAL_GetTick()-start_tick2)<final_tick2) {

		/*
		 * Acquire initial sample value of rotation rate
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt_initial[axis_index] = ttt[axis_index];
		}

		/*
		 * Introduce integration time period delay
		 */

		HAL_Delay(DATA_PERIOD_MS);

		/*
		 * Acquire current sample value of rotation rate and remove
		 * offset value
		 */

		getAngularVelocity(handle, ttt);
		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt[axis_index] = ttt[axis_index] - ttt_offset[axis_index];
		}


		/*
		 * Suppress value of Y-Axis and Z-Axis rotation signals
		 */

		ttt_initial[2] = 0;
		ttt[2] = 0;
		ttt_initial[1] = 0;
		ttt[1] = 0;

		/*
		 * Compute rotation angles by integration
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			rotate_angle[axis_index] = rotate_angle[axis_index]
					+ (float)((ttt_initial[axis_index] + ttt[axis_index]) * Tsample / 2);
		}




	}
	/*
	 * Store intermediate angle as second neuron
	 */
	*ttt_2 = rotate_angle[0] / (1000);
	sprintf(msg1, "\r\nAcquired intermediate angle data");
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

	/*
	 * Obtain time it takes to finish song
	 */

	while (1) {

		/*
		 * Acquire initial sample value of rotation rate
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt_initial[axis_index] = ttt[axis_index];
		}

		/*
		 * Introduce integration time period delay
		 */

		HAL_Delay(DATA_PERIOD_MS);

		/*
		 * Acquire current sample value of rotation rate and remove
		 * offset value
		 */

		getAngularVelocity(handle, ttt);
		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt[axis_index] = ttt[axis_index] - ttt_offset[axis_index];
		}


		/*
		 * Suppress value of Y-Axis and Z-Axis rotation signals
		 */

		ttt_initial[2] = 0;
		ttt[2] = 0;
		ttt_initial[1] = 0;
		ttt[1] = 0;

		/*
		 * Compute rotation angles by integration
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			rotate_angle[axis_index] = rotate_angle[axis_index]
					+ (float)((ttt_initial[axis_index] + ttt[axis_index]) * Tsample / 2);
		}
		if (abs((int)rotate_angle[0]/1000) < 5){
			break;
		}






	}
	/*
	 * Store time it takes to finish song as 3rd neuron
	 */

	*ttt_3 = (HAL_GetTick()-start_tick2)/1000;



	BSP_LED_Off(LED1);
	return;
}

void printOutput_ANN(ANN *net, int input_state, int * error) {

	char dataOut[256] = { };
	int i, loc, count;
	float point = 0.0;
	float rms_output, mean_output, mean_output_rem, next_max;
	float classification_metric;

	/*
	 * Initialize error state
	 */

	*error = 0;

	count = 0;
	mean_output = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		mean_output = mean_output + (net->output[i]);
		if (net->output[i] > point && net->output[i] > 0.1) {
			point = net->output[i];
			loc = i;
		}
		count++;
	}

	next_max = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		if (i == loc) {
			continue;
		}
		if (net->output[i] > next_max && net->output[i] > 0.1) {
			next_max = net->output[i];
		}
	}

	mean_output = (mean_output) / (count);

	count = 0;
	mean_output_rem = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		mean_output_rem = mean_output_rem + (net->output[i]);
		if (i == loc) {
			continue;
		}
		count++;
	}

	mean_output_rem = (mean_output_rem) / (count);

	rms_output = 0;

	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		rms_output = rms_output + pow((net->output[i] - mean_output), 2);
	}

	rms_output = sqrt(rms_output / count);
	if (rms_output != 0) {
		classification_metric = (point - mean_output) / rms_output;
	} else {
		classification_metric = 0;
	}

	if (loc != input_state) {
		rms_output = 0;
		classification_metric = 0;
		point = 0;
		mean_output = 0;
		mean_output_rem = 0;
	}

	sprintf(dataOut, "\r\nState %i\tMax %i\tMean %i\t\tZ-score %i\tOutputs",
			loc, (int) (100 * point), (int) (100 * mean_output),
			(int) (100 * classification_metric));
	CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));

	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		sprintf(dataOut, "\t%i", (int) (100 * net->output[i]));
		CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));
	}

	if (loc != input_state) {
		*error = 1;
		sprintf(dataOut, "\t Classification Error");
		CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));
	}

	if ((loc == input_state)
			&& ((classification_metric < CLASSIFICATION_ACC_THRESHOLD)
					|| ((point / next_max) < CLASSIFICATION_DISC_THRESHOLD))) {
		*error = 1;
		sprintf(dataOut, "\t Classification Accuracy Limit");
		CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));
	}

}

void TrainRotation(void *handle, ANN *net) {

	uint8_t id;
	SensorAxes_t angular_velocity;
	uint8_t status;
	float training_dataset[3][8][3];
	float training_data_init[3];
	float * init_state[3];
	float XYZ[3];
	float xyz[3];
	char msg1[256];
	int num_train_data_cycles;
	int i, j, k, m, n, r, index, error, net_error;
	int ttt_1, ttt_2, ttt_3, ttt_mag_scale;

	BSP_GYRO_Get_Instance(handle, &id);
	BSP_GYRO_IsInitialized(handle, &status);
	if (status == 1) {
		if (BSP_GYRO_Get_Axes(handle, &angular_velocity) == COMPONENT_ERROR) {
			angular_velocity.AXIS_X = 0;
			angular_velocity.AXIS_Y = 0;
			angular_velocity.AXIS_Z = 0;
		}

		sprintf(msg1, "\r\n\r\n\r\nTraining Start in 5 seconds ..");
		CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
		BSP_LED_Off(LED1);
		HAL_Delay(5000);

		/*
		 * Maximum of 8 cycles
		 */
		num_train_data_cycles = 1;

		for (k = 0; k < num_train_data_cycles; k++) {
			for (i = 0; i < 3; i++) {

				sprintf(msg1, "\r\nGo to rest position");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				HAL_Delay(START_POSITION_INTERVAL);

				switch (i) {
				HAL_Delay(1000);
			case 0:

				sprintf(msg1, "\r\nSong 1:");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

				Feature_Extraction_Gyro(handle, &ttt_1, &ttt_2, &ttt_3,
						&ttt_mag_scale);

				break;

			case 1:

				sprintf(msg1, "\r\nSong 2:");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

				Feature_Extraction_Gyro(handle, &ttt_1, &ttt_2, &ttt_3,
						&ttt_mag_scale);

				break;

			case 2:
				sprintf(msg1, "\r\nSong 3:");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

				Feature_Extraction_Gyro(handle, &ttt_1, &ttt_2, &ttt_3,
						&ttt_mag_scale);

				break;



				}



                XYZ[0] = (float) ttt_1;
                XYZ[1] = (float) ttt_2;
                XYZ[2] = (float) ttt_3;

				motion_softmax(net->topology[0], XYZ, xyz);

				training_dataset[i][k][0] = xyz[0];
				training_dataset[i][k][1] = xyz[1];
				training_dataset[i][k][2] = xyz[2];

				sprintf(msg1, "\r\n Softmax Input \t");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				for (r = 0; r < 3; r++) {
					sprintf(msg1, "\t%i", (int) XYZ[r]);
					CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				}
				sprintf(msg1, "\r\n Softmax Output\t");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				for (r = 0; r < 3; r++) {
					sprintf(msg1, "\t%i", (int) (100 * xyz[r]));
					CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				}

				sprintf(msg1, "\r\n\r\n");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

			}

		}

		/*
		 * Enter NN training
		 */

		float _Motion_1[3] = { 1.0, 0.0, 0.0};
		float _Motion_2[3] = { 0.0, 1.0, 0.0};
		float _Motion_3[3] = { 0.0, 0.0, 1.0};

		sprintf(msg1, "\r\n\r\nTraining Start\r\n");
		CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

		/*
		 * Initialize weights with multistart cycle
		 */

		init_state[0] = _Motion_1;
		init_state[1] = _Motion_2;
		init_state[2] = _Motion_3;

		init_ann(net);
		/*
		 * Initialize training with default input
		 */

		index = 0;

		training_data_init[0] = 1;
		training_data_init[1] = 1;
		training_data_init[2] = 1;
		train_ann(net, training_data_init, init_state[index]);

		for (k = 0; k < num_train_data_cycles; k++) {

			i = 0;
			while (i < training_cycles) {
				for (j = 0; j < 3; j++) {

					if ((i % 20 == 0 && i < 100) || i % 100 == 0) {
						char print_train_time[128];
						sprintf(print_train_time,
								"\r\n\r\nTraining Epochs: %d\r\n", i);
						CDC_Fill_Buffer((uint8_t *) print_train_time,
								strlen(print_train_time));

						LED_Code_Blink(0);

						net_error = 0;
						for (m = 0; m < 3; m++) {
							run_ann(net, training_dataset[m][k]);
							printOutput_ANN(net, m, &error);
							if (error == 1) {
								net_error = 1;
							}
						}
						sprintf(msg1, "\r\nIndex %i Error State: %i\r\n", index,
								net_error);
						CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
						if (net_error == 0) {
							return;
						}

					}

					switch (j) {

                    case 0:
                            train_ann(net, training_dataset[j][k], _Motion_1);
                            break;
                    case 1:
                            train_ann(net, training_dataset[j][k], _Motion_2);
                            break;
                    case 2:
                            train_ann(net, training_dataset[j][k], _Motion_3);
                            break;

                    default:
                            break;
					}
					i++;
					HAL_Delay(5);
				}

			}

		}
	}

	if (SendOverUSB) /* Write data on the USB */
	{
		//sprintf( dataOut, "\n\rAX: %d, AY: %d, AZ: %d", (int)acceleration.AXIS_X, (int)acceleration.AXIS_Y, (int)acceleration.AXIS_Z );
		//CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
	}

	if (net_error == 0) {
		LED_Code_Blink(0);
		LED_Code_Blink(0);
	} else {
		LED_Code_Blink(1);
		LED_Code_Blink(1);
	}

	sprintf(msg1, "\r\n\r\nTraining Complete, Now Start Test Motions\r\n");
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
	return;
}

int Gyro_Sensor_Handler_Rotation(void *handle, ANN *net, int prev_loc) {
	uint8_t id;
	SensorAxes_t angular_velocity;
	uint8_t status;
	float xyz[3];
	float XYZ[3];
	float point;
	int ttt_1, ttt_2, ttt_3, ttt_mag_scale;
	char msg1[128];
	int i, j, k, loc;

	BSP_GYRO_Get_Instance(handle, &id);

	BSP_GYRO_IsInitialized(handle, &status);

	if (status == 1) {
		if (BSP_GYRO_Get_Axes(handle, &angular_velocity) == COMPONENT_ERROR) {
			angular_velocity.AXIS_X = 0;
			angular_velocity.AXIS_Y = 0;
			angular_velocity.AXIS_Z = 0;
		}

		/*
		 * Perform limited number of NN execution and prediction cycles.
		 * Upon return, training will be repeased
		 */

		k = 0;

		while (k < NUMBER_TEST_CYCLES) {

			BSP_LED_Off(LED1);

			sprintf(msg1, "\n\rGo to rest position");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			HAL_Delay(START_POSITION_INTERVAL);

			Feature_Extraction_Gyro(handle, &ttt_1, &ttt_2, &ttt_3,
					&ttt_mag_scale);

            XYZ[0] = (float) ttt_1;
            XYZ[1] = (float) ttt_2;
            XYZ[2] = (float) ttt_3;

			motion_softmax(net->topology[0], XYZ, xyz);

			sprintf(msg1, "\r\n Softmax Input: \t");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			for (j = 0; j < 3; j++) {
				sprintf(msg1, "%i\t", (int) XYZ[j]);
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			}
			sprintf(msg1, "\r\n Softmax Output: \t");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			for (j = 0; j < 3; j++) {
				sprintf(msg1, "%i\t", (int) (100 * xyz[j]));
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			}
			sprintf(msg1, "\r\n");

			run_ann(net, xyz);

			point = 0.0;
			loc = -1;

			for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
				if (net->output[i] > point && net->output[i] > 0.1) {
					point = net->output[i];
					loc = i;
				}
			}

			if (loc == -1) {
				LED_Code_Blink(0);
			} else {
				LED_Code_Blink(loc + 1);
			}

			switch (loc) {
			case 0:
				sprintf(msg1, "\n\rYou played song 1!");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				break;
			case 1:
				sprintf(msg1, "\n\rYou played song 2!");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				break;
			case 2:
				sprintf(msg1, "\n\rYou played song 3!");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				break;

			case -1:
				sprintf(msg1, "\n\rNeural Network Classification - ERROR");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				break;
			default:
				sprintf(msg1, "\n\rNeural Network Classification - NULL");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				break;
			}
			k = k + 1;
		}
	}
	return prev_loc;
}

int main(void) {
	uint32_t msTick, msTickPrev = 0;
	uint8_t doubleTap = 0;
	char msg2[128];

	/* STM32L4xx HAL library initialization:
	 - Configure the Flash prefetch, instruction and Data caches
	 - Configure the Systick to generate an interrupt each 1 msec
	 - Set NVIC Group Priority to 4
	 - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	if (SendOverUSB) {
		/* Initialize LED */
		BSP_LED_Init(LED1);
	}
#ifdef NOT_DEBUGGING
	else
	{
		/* Initialize LEDSWD: Cannot be used during debug because it overrides SWDCLK pin configuration */
		BSP_LED_Init(LEDSWD);
		BSP_LED_Off(LEDSWD);
	}
#endif

	/* Initialize RTC */
	RTC_Config();
	RTC_TimeStampConfig();

	/* enable USB power on Pwrctrl CR2 register */
	HAL_PWREx_EnableVddUSB();

	if (SendOverUSB) /* Configure the USB */
	{
		/*** USB CDC Configuration ***/
		/* Init Device Library */
		USBD_Init(&USBD_Device, &VCP_Desc, 0);
		/* Add Supported Class */
		USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
		/* Add Interface callbacks for AUDIO and CDC Class */
		USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
		/* Start Device Process */
		USBD_Start(&USBD_Device);
	} else /* Configure the SDCard */
	{
		DATALOG_SD_Init();
	}
	HAL_Delay(200);

	/* Configure and disable all the Chip Select pins */
	Sensor_IO_SPI_CS_Init_All();

	/* Initialize and Enable the available sensors */
	initializeAllSensors();
	enableAllSensors();

	sprintf(msg2, "\n\rEmbeddedML Violin Song Classification");
	CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));
	sprintf(msg2, "\n\r\n\r\nViolin song detector using STM32 SensorTile");
	CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));
	sprintf(msg2, "\n\rTrains neural network using arm angles and song length to identify violin piece");
	CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));

	/* Notify user */

	sprintf(msg2, "\n\r\n\rDouble Tap to start training");
	CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));


	//---EMBEDDED ANN---
	float weights[54] = { 0.680700, 0.324900, 0.607300, 0.365800, 0.693000,
			0.527200, 0.754400, 0.287800, 0.592300, 0.570900, 0.644000,
			0.416500, 0.249200, 0.704200, 0.598700, 0.250300, 0.632700,
			0.372900, 0.684000, 0.661200, 0.230300, 0.516900, 0.770900,
			0.315700, 0.756000, 0.293300, 0.509900, 0.627800, 0.781600,
			0.733500, 0.509700, 0.382600, 0.551200, 0.326700, 0.781000,
			0.563300, 0.297900, 0.714900, 0.257900, 0.682100, 0.596700,
			0.467200, 0.339300, 0.533600, 0.548500, 0.374500, 0.722800,
			0.209100, 0.619400, 0.635700, 0.300100, 0.715300, 0.670800,
			0.794400
			};
	float dedw[54];
	float bias[12];
	unsigned int network_topology[3] = { 3, 9, 3 };
	float output[3];

	ANN net;
	net.weights = weights;
	net.dedw = dedw;
	net.bias = bias;
	net.topology = network_topology;
	net.n_layers = 3;
	net.n_weights = 54;
	net.n_bias = 12;
	net.output = output;

	//OPTIONS
	net.eta = 0.13;     //Learning Rate
	net.beta = 0.01;    //Bias Learning Rate
	net.alpha = 0.25;   //Momentum Coefficient
	net.output_activation_function = &relu2;
	net.hidden_activation_function = &relu2;

	init_ann(&net);
	//---------------------

	int loc = -1;
	while (1) {
		/*Get sysTick value and check if it's time to execute the task*/
		msTick = HAL_GetTick();
		if (msTick % DATA_PERIOD_MS == 0 && msTickPrev != msTick) {
			msTickPrev = msTick;

			if (SendOverUSB) {
				BSP_LED_On(LED1);
			}

			//RTC_Handler( &RtcHandle );

			if (hasTrained) {
				loc = Gyro_Sensor_Handler_Rotation(LSM6DSM_G_0_handle, &net,
						loc);
				/*
				 * Upon return from Accel_Sensor_Handler, initiate retraining.
				 */
				hasTrained = 0;

				sprintf(msg2,
						"\n\r\n\rDouble Tap to start a new training session");
				CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));
			}

			if (SendOverUSB) {
				BSP_LED_Off(LED1);

			}

		}

		/* Check LSM6DSM Double Tap Event  */
		if (!hasTrained) {
			BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(LSM6DSM_X_0_handle,&doubleTap);
		if (doubleTap) {
			LED_Code_Blink(0);
			TrainRotation(LSM6DSM_G_0_handle, &net);
			hasTrained = 1;
			}
		}

		/* Go to Sleep */
		__WFI();
	}
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors(void) {
	if (BSP_ACCELERO_Init(LSM6DSM_X_0, &LSM6DSM_X_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_GYRO_Init(LSM6DSM_G_0, &LSM6DSM_G_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_ACCELERO_Init(LSM303AGR_X_0, &LSM303AGR_X_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_MAGNETO_Init(LSM303AGR_M_0, &LSM303AGR_M_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_PRESSURE_Init(LPS22HB_P_0, &LPS22HB_P_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_TEMPERATURE_Init(LPS22HB_T_0, &LPS22HB_T_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_TEMPERATURE_Init(HTS221_T_0, &HTS221_T_0_handle)
			== COMPONENT_ERROR) {
		no_T_HTS221 = 1;
	}

	if (BSP_HUMIDITY_Init(HTS221_H_0, &HTS221_H_0_handle) == COMPONENT_ERROR) {
		no_H_HTS221 = 1;
	}

	/* Inialize the Gas Gauge if the battery is present */
	if (BSP_GG_Init(&GG_handle) == COMPONENT_ERROR) {
		no_GG = 1;
	}

	//if(!SendOverUSB)
	//{
	/* Enable HW Double Tap detection */
	BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(LSM6DSM_X_0_handle);
	BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DSM_X_0_handle,
	LSM6DSM_TAP_THRESHOLD_MID);
	//}

}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void enableAllSensors(void) {
	BSP_ACCELERO_Sensor_Enable(LSM6DSM_X_0_handle);
	BSP_GYRO_Sensor_Enable(LSM6DSM_G_0_handle);
	BSP_ACCELERO_Sensor_Enable(LSM303AGR_X_0_handle);
	BSP_MAGNETO_Sensor_Enable(LSM303AGR_M_0_handle);
	BSP_PRESSURE_Sensor_Enable(LPS22HB_P_0_handle);
	BSP_TEMPERATURE_Sensor_Enable(LPS22HB_T_0_handle);
	if (!no_T_HTS221) {
		BSP_TEMPERATURE_Sensor_Enable(HTS221_T_0_handle);
		BSP_HUMIDITY_Sensor_Enable(HTS221_H_0_handle);
	}

}

/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
void disableAllSensors(void) {
	BSP_ACCELERO_Sensor_Disable(LSM6DSM_X_0_handle);
	BSP_ACCELERO_Sensor_Disable(LSM303AGR_X_0_handle);
	BSP_GYRO_Sensor_Disable(LSM6DSM_G_0_handle);
	BSP_MAGNETO_Sensor_Disable(LSM303AGR_M_0_handle);
	BSP_HUMIDITY_Sensor_Disable(HTS221_H_0_handle);
	BSP_TEMPERATURE_Sensor_Disable(HTS221_T_0_handle);
	BSP_TEMPERATURE_Sensor_Disable(LPS22HB_T_0_handle);
	BSP_PRESSURE_Sensor_Disable(LPS22HB_P_0_handle);
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void) {
	/*##-1- Configure the RTC peripheral #######################################*/
	RtcHandle.Instance = RTC;

	/* Configure RTC prescaler and RTC data registers */
	/* RTC configured as follow:
	 - Hour Format    = Format 12
	 - Asynch Prediv  = Value according to source clock
	 - Synch Prediv   = Value according to source clock
	 - OutPut         = Output Disable
	 - OutPutPolarity = High Polarity
	 - OutPutType     = Open Drain */
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
	RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
	RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK) {

		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void) {

	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;

	/*##-3- Configure the Date using BCD format ################################*/
	/* Set Date: Monday January 1st 2000 */
	sdatestructure.Year = 0x00;
	sdatestructure.Month = RTC_MONTH_JANUARY;
	sdatestructure.Date = 0x01;
	sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

	if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK) {

		/* Initialization Error */
		Error_Handler();
	}

	/*##-4- Configure the Time using BCD format#################################*/
	/* Set Time: 00:00:00 */
	stimestructure.Hours = 0x00;
	stimestructure.Minutes = 0x00;
	stimestructure.Seconds = 0x00;
	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss) {

	RTC_TimeTypeDef stimestructure;

	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.Hours = hh;
	stimestructure.Minutes = mm;
	stimestructure.Seconds = ss;
	stimestructure.SubSeconds = 0;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	MEMSInterrupt = 1;
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {

	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	while (1)
	{}
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

