#ifndef _QUESTION_H_
#define _QUESTION_H_

#include "fft.h"

typedef enum
{
    QUESTION_4_SHAPE_DIAGONAL = 1,
    QUESTION_4_SHAPE_CIRCLE = 2,
    QUESTION_4_SHAPE_INFINITY = 3
} Question4Shape;

void Question_1(void);
void Question_2(void);
void Question_3(void);
void Question_4(Question4Shape shape, uint8_t vpp);

#endif
