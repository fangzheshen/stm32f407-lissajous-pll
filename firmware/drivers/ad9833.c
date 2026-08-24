#include "ad9833.h"
#include "main.h"

#define		FSYNC_1()     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
#define		FSYNC_0()   	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
#define     SCK_1()			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET);
#define 	SCK_0()			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET);
#define 	DAT_1()			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
#define 	DAT_0()			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_RESET);
#define 	CS_1()			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
#define 	CS_0()			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);

#define AD9833_MCLK_HZ       25000000.0
#define AD9833_TUNING_SCALE  268435456.0

void AD9833_Init(void)
{

 //GPIO_InitTypeDef  GPIO_InitStructure;

// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PB,PE�˿�ʱ��

// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;				 //LED0-->PB.5 �˿�����
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
// GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
// GPIO_SetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);						 //PB.5 �����
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
}

/*
*********************************************************************************************************
*	�� �� ��: AD9833_Delay
*	����˵��: ʱ����ʱ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AD9833_Delay(void)
{
	unsigned int i;
	for (i = 0; i < 1; i++);
}

/*
 * AD9833 phase registers use a 12-bit phase word:
 * phase_word = phase_degree * 4096 / 360.
 *
 * Degrees are used at the public API so 90.0 means 90 degrees.
 */
static unsigned int AD9833_PhaseDegreeToWord(double phase_degree)
{
	while (phase_degree >= 360.0)
	{
		phase_degree -= 360.0;
	}
	while (phase_degree < 0.0)
	{
		phase_degree += 360.0;
	}

	return ((unsigned int)(phase_degree * 4096.0 / 360.0 + 0.5)) & 0x0FFFU;
}



/*
*********************************************************************************************************
*	�� �� ��: AD9833_Write
*	����˵��: ��SPI���߷���16��bit����
*	��    ��: TxData : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD9833_Write(unsigned int TxData)
{
	unsigned char i;

	SCK_1();
	//AD9833_Delay();
	FSYNC_1();
	//AD9833_Delay();
	FSYNC_0();
	//AD9833_Delay();
	for(i = 0; i < 16; i++)
	{
		if(TxData&0x8000)
		{DAT_1();}
		else
		{DAT_0();}




		AD9833_Delay();
		SCK_0();
		AD9833_Delay();
		SCK_1();

		TxData <<= 1;
	}
	FSYNC_1();

}



/*
*********************************************************************************************************
*	�� �� ��: AD9833_AmpSet
*	����˵��: �ı�����źŷ���ֵ
*	��    ��: 1.amp ������ֵ  0- 255
*	�� �� ֵ: ��
*********************************************************************************************************
*/


void AD9833_AmpSet(unsigned char amp)
{
	unsigned char i;
	unsigned int temp;

	CS_0();
	temp =0x1100|amp;
	for(i=0;i<16;i++)
	{
	    SCK_0();
	   if(temp&0x8000)
	   {	DAT_1();}
	   else
		 DAT_0();
		 temp<<=1;
	    SCK_1();
	    AD9833_Delay();
	}

   	CS_1();
}



/*
*********************************************************************************************************
*	�� �� ��: AD9833_WaveSeting
*	����˵��: ��SPI���߷���16��bit����
*	��    ��: 1.Freq: Ƶ��ֵ, 0.1 hz - 12Mhz
			  2.Freq_SFR: 0 �� 1
			  3.WaveMode: TRI_WAVE(���ǲ�),SIN_WAVE(���Ҳ�),SQU_WAVE(����)
			  4.Phase : ���εĳ���λ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD9833_WaveSeting(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,double PhaseDegree)
{

		int frequence_LSB,frequence_MSB,Phs_data;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;

		/*********************************����Ƶ�ʵ�16����ֵ***********************************/
		frequence_mid=268435456/25;//�ʺ�25M����
		//���ʱ��Ƶ�ʲ�Ϊ25MHZ���޸ĸô���Ƶ��ֵ����λMHz ��AD9833���֧��25MHz
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1000000;
		frequence_DATA=frequence_DATA*frequence_mid;
		frequence_hex=frequence_DATA;  //���frequence_hex��ֵ��32λ��һ���ܴ�����֣���Ҫ��ֳ�����14λ���д�����
		frequence_LSB=frequence_hex; //frequence_hex��16λ�͸�frequence_LSB
		frequence_LSB=frequence_LSB&0x3fff;//ȥ�������λ��16λ����ȥ����λ������14λ
		frequence_MSB=frequence_hex>>14; //frequence_hex��16λ�͸�frequence_HSB
		frequence_MSB=frequence_MSB&0x3fff;//ȥ�������λ��16λ����ȥ����λ������14λ

		Phs_data=(int)(AD9833_PhaseDegreeToWord(PhaseDegree)|0xC000U);
		AD9833_Write(0x0100); //��λAD9833,��RESETλΪ1
		AD9833_Write(0x2100); //ѡ������һ��д�룬B28λ��RESETλΪ1

		if(Freq_SFR==0)				  //���������õ�����Ƶ�ʼĴ���0
		{
		 	frequence_LSB=frequence_LSB|0x4000;
		 	frequence_MSB=frequence_MSB|0x4000;
			 //ʹ��Ƶ�ʼĴ���0�������
			AD9833_Write(frequence_LSB); //L14��ѡ��Ƶ�ʼĴ���0�ĵ�14λ��������
			AD9833_Write(frequence_MSB); //H14 Ƶ�ʼĴ����ĸ�14λ��������
			AD9833_Write(Phs_data);	//������λ
			//AD9833_Write(0x2000); /**����FSELECTλΪ0��оƬ���빤��״̬,Ƶ�ʼĴ���0�������**/
	    }
		if(Freq_SFR==1)				//���������õ�����Ƶ�ʼĴ���1
		{
			 frequence_LSB=frequence_LSB|0x8000;
			 frequence_MSB=frequence_MSB|0x8000;
			//ʹ��Ƶ�ʼĴ���1�������
			AD9833_Write(frequence_LSB); //L14��ѡ��Ƶ�ʼĴ���1�ĵ�14λ����
			AD9833_Write(frequence_MSB); //H14 Ƶ�ʼĴ���1Ϊ
			AD9833_Write(Phs_data);	//������λ
			//AD9833_Write(0x2800); /**����FSELECTλΪ0������FSELECTλΪ1����ʹ��Ƶ�ʼĴ���1��ֵ��оƬ���빤��״̬,Ƶ�ʼĴ���1�������**/
		}

		if(WaveMode==TRI_WAVE) //������ǲ�����
		 	AD9833_Write(0x2002);
		if(WaveMode==SQU_WAVE)	//�����������
			AD9833_Write(0x2028);
		if(WaveMode==SIN_WAVE)	//������Ҳ���
			AD9833_Write(0x2000);

}

/*
 * Update PHASE0 without resetting the DDS accumulator.  Use this function
 * for phase feedback after the output frequency has been configured.
 */
void AD9833_SetPhase(double PhaseDegree)
{
	unsigned int phase_word = AD9833_PhaseDegreeToWord(PhaseDegree);
	AD9833_Write(0xC000U | phase_word);
}

/*
 * Update FREQ0 without asserting RESET.  The phase accumulator keeps
 * running, so this function is suitable for the phase-locked loop.
 * Call AD9833_WaveSeting() or AD9833_OUT_Setting() once before using it.
 */
void AD9833_SetFrequency(double Freq)
{
	unsigned long frequency_word;
	unsigned int frequency_lsb;
	unsigned int frequency_msb;

	if (Freq < 0.0)
	{
		Freq = 0.0;
	}
	else if (Freq > AD9833_MCLK_HZ / 2.0)
	{
		Freq = AD9833_MCLK_HZ / 2.0;
	}

	frequency_word = (unsigned long)(
		Freq * AD9833_TUNING_SCALE / AD9833_MCLK_HZ + 0.5);
	frequency_lsb = (unsigned int)(frequency_word & 0x3FFFUL);
	frequency_msb = (unsigned int)((frequency_word >> 14U) & 0x3FFFUL);

	AD9833_Write(0x4000U | frequency_lsb);
	AD9833_Write(0x4000U | frequency_msb);
}

/*
 * Configure frequency, waveform, raw MCP41010 amplitude code and phase.
 * amp is a 0-to-255 wiper code, not an output-voltage value.
 */
void AD9833_OUT_Setting(double Freq,
                        unsigned int WaveMode,
                        unsigned char amp,
                        double PhaseDegree)
{
	AD9833_WaveSeting(Freq, 0U, WaveMode, PhaseDegree);
	AD9833_AmpSet(amp);
}
