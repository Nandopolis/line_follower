/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "stm32f0xx.h"
#include "snippets.h"
#include "LowPass.h"

enum CONTROL_CONSTANTS
{
	SAMPLING_PERIOD = 2,
	PID_PERIOD = 4,
	ROBOT_SPEED = 767,
	FORWARD_WORKSPACE = 4095 - ROBOT_SPEED,
};

enum MODES
{
	STANDBY = 0,
	CALIBRATE,
	NORMAL,
	SAVE,
	INITIAL = 255,
};

enum RTOS_SIGNALS
{
	ADC_READY = 1,
};


typedef struct  
{
	float32_t filter_output[NUMBER_OF_ADC_CHANNEL];
} filter_array;

osThreadId main_thread, mode_thread, filter_thread, \
			centroid_thread, pid_thread, cal_thread;
osMailQId filter_mailQ, centroid_mailQ;
osTimerId adc_timer;

float32_t max_array[NUMBER_OF_ADC_CHANNEL], min_array[NUMBER_OF_ADC_CHANNEL];
LowPassType *low_pass[NUMBER_OF_ADC_CHANNEL];

void mode(void const *argument);
void filter(void const *argument);
void centroid(void const *argument);
void pid(void const *argument);
void cal(void const *argument);
void adc(void const *argument);

osThreadDef(mode, osPriorityLow, 1, 144);
osThreadDef(cal, osPriorityAboveNormal, 1, 0);
osThreadDef(filter, osPriorityHigh, 1, 144);
osThreadDef(centroid, osPriorityAboveNormal, 1, 144);
osThreadDef(pid, osPriorityAboveNormal, 1, 144);
osMailQDef(filter_mailQ, 2, filter_array);
osMailQDef(centroid_mailQ, 2, float32_t);
osTimerDef(adc, adc);

/*
 * main: initialize and start the system
 */
int main (void)
{
	uint8_t i;
	
	adc_timer = osTimerCreate(osTimer(adc), osTimerPeriodic, NULL);
	osKernelInitialize ();                    // initialize CMSIS-RTOS
	//osTimerStart(adc_timer, SAMPLING_PERIOD);

	// initialize peripherals here
	SetSysClock();
	ConfigureGPIO();
	SetClockForADC();
	CalibrateADC(); 
	ConfigureGPIOforADC();
	EnableADC();
	ConfigureADC();
	ConfigureDMA();
	ConfigureTIMsPWM();
	
	for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
	{
		low_pass[i] = LowPass_create();
	}
	
	// create 'thread' functions that start executing,
	// example: tid_name = osThreadCreate (osThread(name), NULL);
	mode_thread = osThreadCreate(osThread(mode), NULL);
	main_thread = osThreadGetId();
	
	filter_mailQ = osMailCreate(osMailQ(filter_mailQ), NULL);
	centroid_mailQ = osMailCreate(osMailQ(centroid_mailQ), NULL);
	
	osKernelStart ();                         // start thread execution
	osThreadTerminate(main_thread);
}

void stopMotors(void)
{
	M_RIGHT_BW = 4095;
	M_RIGHT_FW = 4095;
	M_LEFT_BW = 4095;
	M_LEFT_FW = 4095;
}

void adc(void const *argument)
{
	GPIOB->BSRR = GPIO_BSRR_BS_1;
	__nop();
	__nop();
	ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
}

__inline void closeThreads(uint8_t prev_mode)
{
	switch (prev_mode)
	{
		case STANDBY:
			break;
		case CALIBRATE:
			osTimerStop(adc_timer);
			osThreadTerminate(filter_thread);
			osThreadTerminate(cal_thread);
			break;
		case NORMAL:
			osTimerStop(adc_timer);
			osThreadTerminate(filter_thread);
			osThreadTerminate(centroid_thread);
			osThreadTerminate(pid_thread);
			break;
		case SAVE:
			break;
		default:
			break;
	}
}

__inline void dummy(void)
{
	M_RIGHT_BW = 4095;
	M_RIGHT_FW = 3072;
	M_LEFT_BW = 3072;
	M_LEFT_FW = 4095;
	osDelay(125);
	stopMotors();
	osDelay(200);
	M_RIGHT_BW = 3072;
	M_RIGHT_FW = 4095;
	M_LEFT_BW = 4095;
	M_LEFT_FW = 3072;
	osDelay(188);
	stopMotors();
	osDelay(200);
	M_RIGHT_BW = 4095;
	M_RIGHT_FW = 3072;
	M_LEFT_BW = 3072;
	M_LEFT_FW = 4095;
	osDelay(125);
	stopMotors();
}

__inline void openThreads(uint8_t new_mode)
{
	switch (new_mode)
	{
		case STANDBY:
			stopMotors();
			break;
		case CALIBRATE:
			cal_thread = osThreadCreate(osThread(cal), NULL);
			filter_thread = osThreadCreate (osThread(filter), NULL);
			osTimerStart(adc_timer, SAMPLING_PERIOD);
			osDelay(500);
			dummy();
			break;
		case NORMAL:
			pid_thread = osThreadCreate (osThread(pid), NULL);
			centroid_thread = osThreadCreate (osThread(centroid), NULL);
			filter_thread = osThreadCreate (osThread(filter), NULL);
			osTimerStart(adc_timer, SAMPLING_PERIOD);
			break;
		case SAVE:
			break;
		default:
			break;
	}
}

void mode(void const *argument)
{
	uint32_t cur_mode = INITIAL;
	uint32_t new_mode;
	
	for(;;)
	{
		if ((GPIOF->IDR & 3) != cur_mode)
		{
			new_mode = GPIOF->IDR & 3;
			osDelay(10);
			if ((GPIOF->IDR & 3) == new_mode)
			{
				closeThreads(cur_mode);
				cur_mode = new_mode;
				openThreads(cur_mode);
			}
		}
		osDelay(100);
	}
}


void filter(void const *argument)
{
	filter_array *filter_tx;
	float32_t input;
	uint32_t count = 0, i;
	
	for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
	{
		LowPass_reset(low_pass[i]);
	}
	for(;;)
	{
		osSignalWait(ADC_READY, osWaitForever);
		GPIOB->BSRR = GPIO_BSRR_BS_1;
		for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
		{
			input = (float32_t)ADC_array[i];
			LowPass_writeInput(low_pass[i], input);
		}
		if (++count == PID_PERIOD / SAMPLING_PERIOD) {
			filter_tx = (filter_array *)osMailAlloc(filter_mailQ, 0);
			if (filter_tx != NULL)
			{
				for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
				{
					filter_tx->filter_output[i] = low_pass[i]->output;
				}
				osMailPut(filter_mailQ, filter_tx);
			}
			count = 0;
		}
		GPIOB->BRR = GPIO_BRR_BR_1;
	}
}

__inline void calibrate(float32_t *data, float32_t *cal_data)
{
	uint32_t i;
	
	for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
	{
		if (data[i] > min_array[i])
		{
			if (data[i] < max_array[i])
			{
				cal_data[i] = (data[i] - min_array[i]) \
							/ (max_array[i] - min_array[i]);
			}
			else
			{
				cal_data[i] = 1.0;
			}
		}
		else
		{
			cal_data[i] = 0.0;
		}
	}
}

void centroid(void const *argument)
{
	osEvent mail;
	filter_array *filter_rx;
	float32_t *line_pos_tx;
	float32_t cal_data[NUMBER_OF_ADC_CHANNEL];
	float32_t num, div;
	float32_t static const offset = ((float32_t)NUMBER_OF_ADC_CHANNEL - 1) / 2;
	uint32_t i;
	
	for(;;)
	{
		mail = osMailGet(filter_mailQ, osWaitForever);
		GPIOB->BSRR = GPIO_BSRR_BS_1;
		if (mail.status == osEventMail)
		{
			filter_rx = (filter_array *)mail.value.p;
			calibrate(filter_rx->filter_output, cal_data);
			num = 0.0;
			div = 0.0;
			for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
			{
				num += cal_data[i] * i;
				div += cal_data[i];
			}
			osMailFree(filter_mailQ, filter_rx);
			line_pos_tx = (float32_t *)osMailAlloc(centroid_mailQ, 0);
			if (line_pos_tx != NULL)
			{
				*line_pos_tx = num/div - offset;
				osMailPut(centroid_mailQ, line_pos_tx);
			}
		}
		GPIOB->BRR = GPIO_BRR_BR_1;
	}
}

void setMotorPWM(int32_t u)
{
	uint32_t aux;
	__IO uint32_t *m1_fw, *m1_bw, *m2_fw, *m2_bw;

	if (u > 0)
	{
		m2_fw = &M_RIGHT_FW;
		m2_bw = &M_RIGHT_BW;
		m1_fw = &M_LEFT_FW;
		m1_bw = &M_LEFT_BW;
	}
	else
	{
		u *= -1;
		m1_fw = &M_RIGHT_FW;
		m1_bw = &M_RIGHT_BW;
		m2_fw = &M_LEFT_FW;
		m2_bw = &M_LEFT_BW;
	}
	
	if (u > FORWARD_WORKSPACE)
	{
		aux = 2 * u - FORWARD_WORKSPACE;
		*m1_bw = 0;
		*m1_fw = 4095;
	}
	else
	{
		aux = u;
		*m1_bw = 4095 - (ROBOT_SPEED + u);
		*m1_fw = 4095;
	}
	if (aux > ROBOT_SPEED)
	{
		aux -= ROBOT_SPEED;
		*m2_bw = 4095;
		*m2_fw = 4095 - aux;
	}
	else
	{
		*m2_bw = 4095 - (ROBOT_SPEED - aux);
		*m2_fw = 4095;
	}
}

void pid (void const *argument)
{
	osEvent mail;
	float32_t *line_pos_rx;
	float32_t kp = 1024.0, ki = 64.0, kd = 32.0;
	float32_t static const T = (float32_t)PID_PERIOD / 1000.0;
	float32_t e, u, aux;
	float32_t sum_e = 0.0, e_1 = 0.0;
	
	for(;;)
	{
		mail = osMailGet(centroid_mailQ, osWaitForever);
		GPIOB->BSRR = GPIO_BSRR_BS_1;
		if (mail.status == osEventMail)
		{
			line_pos_rx = (float32_t *)mail.value.p;
			e = 0.0 - *line_pos_rx;
			u = kp * e;
			aux = (sum_e + e) * T;
			if ((aux > -ROBOT_SPEED) && (aux < ROBOT_SPEED)) {
				sum_e += e;
			}
			u += ki * T * sum_e;
			u += kd / T * (e - e_1);
			if (u > ROBOT_SPEED)
			{
				u = ROBOT_SPEED;
			}
			else if (u < -ROBOT_SPEED)
			{
				u = -ROBOT_SPEED;
			}
			setMotorPWM((int32_t)u);
			e_1 = e;
			osMailFree(centroid_mailQ, line_pos_rx);
		}
		GPIOB->BRR = GPIO_BRR_BR_1;
	}
}

void cal(void const *argument)
{
	uint32_t i;
	osEvent mail;
	filter_array *filter_rx;
	
	for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
	{
		max_array[i] = 0.0;
		min_array[i] = 4095.0;
	}
	
	for(;;)
	{
		mail = osMailGet(filter_mailQ, osWaitForever);
		if (mail.status == osEventMail)
		{
			filter_rx = (filter_array *)mail.value.p;
			for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++)
			{
				if (filter_rx->filter_output[i] > max_array[i])
				{
					max_array[i] = filter_rx->filter_output[i];
				}
				else if (filter_rx->filter_output[i] < min_array[i])
				{
					min_array[i] = filter_rx->filter_output[i];
				}
			}
			osMailFree(filter_mailQ, filter_rx);
		}
	}
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f072xb.s).                                             */
/******************************************************************************/

/**
  * @brief  This function handles DMA Channel1 interrupt request.
  *         It manages the ADC and DMA 
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
	if ((DMA1->ISR & DMA_ISR_TCIF1) != 0) /* Test if transfer completed on DMA channel 1 */
	{
		DMA1_Channel1->CCR &= (uint32_t)(~DMA_CCR_EN); /* Disable DMA Channel 1 to write in CNDTR*/
		DMA1_Channel1->CNDTR = NUMBER_OF_ADC_CHANNEL; /* Reload the number of DMA tranfer to be performs on DMA channel 1 */
		DMA1_Channel1->CCR |= DMA_CCR_EN; /* Enable again DMA Channel 1 */
		DMA1->IFCR |= DMA_IFCR_CTCIF1; /* Clear the flag */
		GPIOB->BRR = GPIO_BRR_BR_1;
		osSignalSet(filter_thread, ADC_READY);
	}
	else if ((DMA1->ISR & DMA_ISR_TEIF1) != 0) /* Test if transfer error on DMA channel 1 */
	{
		DMA1->IFCR |= DMA_IFCR_CTEIF1; /* Clear the flag */
		while(1);
	}
}


/**
  * @brief  This function handles ADC interrupt request.
  *         It manages the ADC and DMA in case of overrun
  *         the ADC is stopped but not disabled,
  *         the DMA is reinitialized,
  *         The AD conversion is reume till the USER button is pressed
  * @param  None
  * @retval None
  */
void ADC1_COMP_IRQHandler(void)
{
	if ((ADC1->ISR & ADC_ISR_OVR) != 0)  /* Check OVR has triggered the IT */
	{
		ADC1->ISR |= ADC_ISR_OVR; /* Clear the pending bit */
		while(1);
	}
}
