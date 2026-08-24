#ifndef __AD9833_H
#define __AD9833_H

#define TRI_WAVE 	0  		//三角波
#define SIN_WAVE 	1		//正弦波
#define SQU_WAVE 	2		//方波

void AD9833_Init(void);
void AD9833_WaveSeting(double frequence,unsigned int frequence_SFR,unsigned int WaveMode,double PhaseDegree);
void AD9833_SetFrequency(double Freq);
void AD9833_SetPhase(double PhaseDegree);
void AD9833_AmpSet(unsigned char amp);
void AD9833_OUT_Setting(double Freq,unsigned int WaveMode,unsigned char amp,double PhaseDegree);
#endif
