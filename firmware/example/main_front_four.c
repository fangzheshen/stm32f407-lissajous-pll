/*
 * Minimal integration example for Questions 1-4.
 * Copy the relevant parts into the USER CODE sections of CubeMX main.c.
 */
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "ad9833.h"
#include "question.h"

typedef enum
{
    APP_QUESTION_1 = 1U,
    APP_QUESTION_2,
    APP_QUESTION_3,
    APP_QUESTION_4
} AppQuestion;

static AppQuestion selected_question = APP_QUESTION_1;
static Question4Shape q4_shape = QUESTION_4_SHAPE_DIAGONAL;
static uint8_t q4_vpp = 4U;

static void App_Run(void)
{
    switch (selected_question)
    {
        case APP_QUESTION_2:
            Question_2();
            break;
        case APP_QUESTION_3:
            Question_3();
            break;
        case APP_QUESTION_4:
            Question_4(q4_shape, q4_vpp);
            break;
        case APP_QUESTION_1:
        default:
            Question_1();
            break;
    }
}

/*
 * Required startup order after CubeMX peripheral initialization:
 *
 *   MX_GPIO_Init();
 *   MX_DMA_Init();
 *   MX_ADC1_Init();
 *   MX_ADC2_Init();
 *   MX_TIM8_Init();
 *   MX_USART2_UART_Init();
 *   MX_SPI2_Init();
 *   AD9833_Init();
 *
 * Then call App_Run() continuously in while (1).
 */
