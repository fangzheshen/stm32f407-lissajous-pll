#include "screen.h"

#include "question.h"
#include "tjc_usart_hmi.h"

typedef enum
{
    SCREEN_RUN_QUESTION_1 = 1U,
    SCREEN_RUN_QUESTION_2,
    SCREEN_RUN_QUESTION_3,
    SCREEN_RUN_QUESTION_4
} ScreenRunMode;

static ScreenRunMode screen_run_mode = SCREEN_RUN_QUESTION_1;
static Question4Shape screen_q4_shape = QUESTION_4_SHAPE_DIAGONAL;
static uint8_t screen_q4_vpp = 4U;

static uint8_t Screen_IsPressEvent(uint8_t component, uint8_t event)
{
    return (event == 1U || event == component) ? 1U : 0U;
}

static void Screen_HandleTouch(uint8_t page,
                               uint8_t component,
                               uint8_t event)
{
    if (Screen_IsPressEvent(component, event) == 0U)
    {
        return;
    }

    if (page == 0U)
    {
        if (component == 1U)
        {
            screen_run_mode = SCREEN_RUN_QUESTION_1;
        }
        else if (component == 2U)
        {
            screen_run_mode = SCREEN_RUN_QUESTION_2;
        }
        else if (component == 3U)
        {
            screen_run_mode = SCREEN_RUN_QUESTION_3;
        }
        return;
    }

    if (component < 1U || component > 4U)
    {
        return;
    }

    if (page == 2U)
    {
        screen_q4_shape = QUESTION_4_SHAPE_DIAGONAL;
    }
    else if (page == 3U)
    {
        screen_q4_shape = QUESTION_4_SHAPE_CIRCLE;
    }
    else if (page == 4U)
    {
        screen_q4_shape = QUESTION_4_SHAPE_INFINITY;
    }
    else
    {
        return;
    }

    screen_q4_vpp = component;
    screen_run_mode = SCREEN_RUN_QUESTION_4;
}

void Screen_Init(void)
{
    initRingBuffer();
}

void Screen_ProcessEvents(void)
{
    while (usize >= FRAME_LENGTH)
    {
        uint8_t frame_header = u(0);
        uint8_t page = u(1);
        uint8_t component = u(2);
        uint8_t event = u(3);
        uint8_t end_0 = u(4);
        uint8_t end_1 = u(5);
        uint8_t end_2 = u(6);

        if (frame_header == 0x65U
            && end_0 == 0xFFU
            && end_1 == 0xFFU
            && end_2 == 0xFFU)
        {
            Screen_HandleTouch(page, component, event);
            udelete(FRAME_LENGTH);
        }
        else
        {
            udelete(1U);
        }
    }
}

void Screen_RunSelectedQuestion(void)
{
    switch (screen_run_mode)
    {
        case SCREEN_RUN_QUESTION_2:
            Question_2();
            break;
        case SCREEN_RUN_QUESTION_3:
            Question_3();
            break;
        case SCREEN_RUN_QUESTION_4:
            Question_4(screen_q4_shape, screen_q4_vpp);
            break;
        case SCREEN_RUN_QUESTION_1:
        default:
            Question_1();
            break;
    }
}
