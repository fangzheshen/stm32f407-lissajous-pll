#include "fft.h"

#include "adc.h"
#include "tim.h"

#include <math.h>
#include <string.h>

#define FFT_PI             3.14159265358979323846f
#define FFT_TWO_PI         (2.0f * FFT_PI)
#define FFT_CAPTURE_TIMEOUT_MS 100U
#define FFT_DEFAULT_PHASE_CAPTURE_SIZE 1024U
#define FFT_ZERO_CROSS_MAX_USE_HZ 60000.0f
#define FFT_Y_SEARCH_MIN_HZ       500.0f
#define FFT_Y_SEARCH_MAX_HZ       220000.0f
/* User-visible phase compensation is kept in question.c. */
#define FFT_INTERNAL_XY_PHASE_OFFSET_DEG 0.0f

static volatile uint8_t adc_dma_done;
volatile uint8_t fft_capture_error;
static float fft_sample_clock_calibration = 0.99310f;

uint32_t adc_xy_buf[FFT_SIZE];

/*
 * Static work buffers keep the large arrays off the small C stack.
 * 8192 complex float samples use 64 KiB; simultaneous packed X/Y
 * samples use another 32 KiB.
 */
static float fft_real[FFT_SIZE];
static float fft_imag[FFT_SIZE];

static uint16_t FFT_GetXSample(uint32_t index)
{
    return (uint16_t)(adc_xy_buf[index] >> 16U);
}

static uint16_t FFT_GetYSample(uint32_t index)
{
    return (uint16_t)(adc_xy_buf[index] & 0xFFFFU);
}

/*
 * Estimate the X frequency from many linearly interpolated rising
 * zero crossings.  For a clean competition sine this is much less
 * likely than one FFT frame to jump between adjacent frequency bins.
 * The FFT peak is still used later as the signal/validity check.
 */
static uint8_t FFT_EstimateXFrequencyByZeroCrossings(
    uint32_t sample_count,
    float x_mean_counts,
    float sample_rate_hz,
    float expected_frequency_hz,
    float *frequency_hz)
{
    float first_crossing = 0.0f;
    float last_crossing = 0.0f;
    float minimum_spacing;
    float hysteresis_counts;
    uint16_t minimum_sample = 4095U;
    uint16_t maximum_sample = 0U;
    uint32_t crossing_count = 0U;
    uint32_t i;
    uint8_t crossing_armed = 0U;

    if (frequency_hz == NULL
        || sample_count < 3U
        || sample_rate_hz <= 0.0f)
    {
        return 0U;
    }

    for (i = 0U; i < sample_count; ++i)
    {
        uint16_t sample = FFT_GetXSample(i);
        if (sample < minimum_sample)
        {
            minimum_sample = sample;
        }
        if (sample > maximum_sample)
        {
            maximum_sample = sample;
        }
    }

    hysteresis_counts =
        0.05f * (float)(maximum_sample - minimum_sample);
    if (hysteresis_counts < 4.0f)
    {
        hysteresis_counts = 4.0f;
    }

    if (expected_frequency_hz >= FFT_SEARCH_MIN_HZ
        && expected_frequency_hz <= FFT_SEARCH_MAX_HZ)
    {
        minimum_spacing =
            sample_rate_hz / (1.6f * expected_frequency_hz);
    }
    else
    {
        minimum_spacing =
            sample_rate_hz / (2.0f * FFT_SEARCH_MAX_HZ);
    }
    if (minimum_spacing < 2.0f)
    {
        minimum_spacing = 2.0f;
    }

    for (i = 1U; i < sample_count; ++i)
    {
        float previous_sample =
            (float)FFT_GetXSample(i - 1U) - x_mean_counts;
        float current_sample =
            (float)FFT_GetXSample(i) - x_mean_counts;

        if (current_sample <= -hysteresis_counts)
        {
            crossing_armed = 1U;
        }

        if (crossing_armed != 0U
            && previous_sample < hysteresis_counts
            && current_sample >= hysteresis_counts)
        {
            float denominator = current_sample - previous_sample;
            float crossing_position;

            crossing_armed = 0U;
            if (denominator <= 0.0f)
            {
                continue;
            }

            crossing_position =
                (float)(i - 1U)
              + (hysteresis_counts - previous_sample)
                / denominator;

            /*
             * Ignore a second noise crossing too close to the previous
             * one.  A real rising crossing is one full period later.
             */
            if (crossing_count != 0U
                && (crossing_position - last_crossing)
                    < minimum_spacing)
            {
                continue;
            }

            if (crossing_count == 0U)
            {
                first_crossing = crossing_position;
            }
            last_crossing = crossing_position;
            ++crossing_count;
        }
    }

    if (crossing_count < 2U
        || last_crossing <= first_crossing)
    {
        return 0U;
    }

    *frequency_hz =
        (float)(crossing_count - 1U) * sample_rate_hz
        / (last_crossing - first_crossing);

    return (*frequency_hz >= FFT_SEARCH_MIN_HZ
         && *frequency_hz <= FFT_SEARCH_MAX_HZ) ? 1U : 0U;
}

/*
 * The harmonic modes need the real Y frequency, not a frequency inferred
 * from wrapped phase samples.  At an 8 ms control interval, phase-slope
 * frequency has aliases 125 Hz apart; direct Y zero crossings do not.
 */
static uint8_t FFT_EstimateYFrequencyByZeroCrossings(
    uint32_t sample_count,
    float y_mean_counts,
    float sample_rate_hz,
    float expected_frequency_hz,
    float *frequency_hz)
{
    float first_crossing = 0.0f;
    float last_crossing = 0.0f;
    float minimum_spacing;
    float hysteresis_counts;
    uint16_t minimum_sample = 4095U;
    uint16_t maximum_sample = 0U;
    uint32_t crossing_count = 0U;
    uint32_t i;
    uint8_t crossing_armed = 0U;

    if (frequency_hz == NULL
        || sample_count < 3U
        || sample_rate_hz <= 0.0f)
    {
        return 0U;
    }

    for (i = 0U; i < sample_count; ++i)
    {
        uint16_t sample = FFT_GetYSample(i);
        if (sample < minimum_sample)
        {
            minimum_sample = sample;
        }
        if (sample > maximum_sample)
        {
            maximum_sample = sample;
        }
    }

    hysteresis_counts =
        0.05f * (float)(maximum_sample - minimum_sample);
    if (hysteresis_counts < 4.0f)
    {
        hysteresis_counts = 4.0f;
    }

    if (expected_frequency_hz >= FFT_Y_SEARCH_MIN_HZ
        && expected_frequency_hz <= FFT_Y_SEARCH_MAX_HZ)
    {
        minimum_spacing =
            sample_rate_hz / (1.4f * expected_frequency_hz);
    }
    else
    {
        minimum_spacing =
            sample_rate_hz / (2.0f * FFT_Y_SEARCH_MAX_HZ);
    }
    if (minimum_spacing < 2.0f)
    {
        minimum_spacing = 2.0f;
    }

    for (i = 1U; i < sample_count; ++i)
    {
        float previous_sample =
            (float)FFT_GetYSample(i - 1U) - y_mean_counts;
        float current_sample =
            (float)FFT_GetYSample(i) - y_mean_counts;

        if (current_sample <= -hysteresis_counts)
        {
            crossing_armed = 1U;
        }

        if (crossing_armed != 0U
            && previous_sample < hysteresis_counts
            && current_sample >= hysteresis_counts)
        {
            float denominator = current_sample - previous_sample;
            float crossing_position;

            crossing_armed = 0U;
            if (denominator <= 0.0f)
            {
                continue;
            }

            crossing_position =
                (float)(i - 1U)
              + (hysteresis_counts - previous_sample)
                / denominator;

            if (crossing_count != 0U
                && (crossing_position - last_crossing)
                    < minimum_spacing)
            {
                continue;
            }

            if (crossing_count == 0U)
            {
                first_crossing = crossing_position;
            }
            last_crossing = crossing_position;
            ++crossing_count;
        }
    }

    if (crossing_count < 2U
        || last_crossing <= first_crossing)
    {
        return 0U;
    }

    *frequency_hz =
        (float)(crossing_count - 1U) * sample_rate_hz
        / (last_crossing - first_crossing);

    return (*frequency_hz >= FFT_Y_SEARCH_MIN_HZ
         && *frequency_hz <= FFT_Y_SEARCH_MAX_HZ) ? 1U : 0U;
}

static float FFT_WrapPhaseDeg(float phase_deg)
{
    while (phase_deg > 180.0f)
    {
        phase_deg -= 360.0f;
    }
    while (phase_deg <= -180.0f)
    {
        phase_deg += 360.0f;
    }

    return phase_deg;
}

static uint32_t FFT_GetTim8ClockHz(void)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();

    /*
     * On STM32F4, a divided APB clock drives the timer at twice PCLK.
     * PPRE2 = 0 means APB2 is not divided.
     */
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U)
    {
        timer_clock *= 2U;
    }

    if (timer_clock == 0U)
    {
        timer_clock = FFT_TIMER_CLOCK_FALLBACK_HZ;
    }

    return timer_clock;
}

static uint8_t FFT_Capture(float *actual_sample_rate_hz,
                           uint32_t sample_count)
{
    uint32_t timer_clock;
    uint32_t timer_counts;
    uint32_t start_tick;
    HAL_StatusTypeDef status;

    fft_capture_error = 0U;
    if (sample_count == 0U || sample_count > FFT_SIZE)
    {
        fft_capture_error = 5U;
        return 0U;
    }
    timer_clock = FFT_GetTim8ClockHz();
    timer_counts = (timer_clock + FFT_SAMPLE_RATE_HZ / 2U)
                 / FFT_SAMPLE_RATE_HZ;

    if (timer_counts < 2U)
    {
        timer_counts = 2U;
    }
    if (timer_counts > 65536U)
    {
        timer_counts = 65536U;
    }

    *actual_sample_rate_hz =
        (float)timer_clock / (float)timer_counts;

    HAL_TIM_Base_Stop(&htim8);
    HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    HAL_ADC_Stop(&hadc2);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC | ADC_FLAG_OVR);

    __HAL_TIM_SET_AUTORELOAD(&htim8, timer_counts - 1U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    htim8.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);

    adc_dma_done = 0U;
    status = HAL_ADC_Start(&hadc2);
    if (status != HAL_OK)
    {
        fft_capture_error = 1U;
        return 0U;
    }

    status = HAL_ADCEx_MultiModeStart_DMA(
        &hadc1,
        adc_xy_buf,
        sample_count);
    if (status != HAL_OK)
    {
        fft_capture_error = 2U;
        HAL_ADC_Stop(&hadc2);
        return 0U;
    }

    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    status = HAL_TIM_Base_Start(&htim8);
    if (status != HAL_OK)
    {
        fft_capture_error = 3U;
        HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        HAL_ADC_Stop(&hadc2);
        return 0U;
    }

    start_tick = HAL_GetTick();
    while (adc_dma_done == 0U)
    {
        if ((HAL_GetTick() - start_tick) > FFT_CAPTURE_TIMEOUT_MS)
        {
            fft_capture_error = 4U;
            HAL_TIM_Base_Stop(&htim8);
            HAL_ADCEx_MultiModeStop_DMA(&hadc1);
            HAL_ADC_Stop(&hadc2);
            __HAL_ADC_CLEAR_FLAG(
                &hadc1,
                ADC_FLAG_EOC | ADC_FLAG_OVR);
            __HAL_ADC_CLEAR_FLAG(
                &hadc2,
                ADC_FLAG_EOC | ADC_FLAG_OVR);
            return 0U;
        }
    }

    HAL_TIM_Base_Stop(&htim8);
    HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    HAL_ADC_Stop(&hadc2);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC | ADC_FLAG_OVR);
    return 1U;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /*
         * Stop the trigger source as soon as the normal-mode DMA frame is
         * complete.  Otherwise extra conversions can set ADC1 OVR before
         * the foreground code gets a chance to stop TIM8.
         */
        HAL_TIM_Base_Stop(&htim8);
        adc_dma_done = 1U;
    }
}

static void FFT_ComplexForward(float *real, float *imag, uint32_t length)
{
    uint32_t i;
    uint32_t j;
    uint32_t block_length;

    /* In-place bit reversal. */
    j = 0U;
    for (i = 1U; i < length; ++i)
    {
        uint32_t bit = length >> 1U;

        while ((j & bit) != 0U)
        {
            j ^= bit;
            bit >>= 1U;
        }
        j ^= bit;

        if (i < j)
        {
            float temp;

            temp = real[i];
            real[i] = real[j];
            real[j] = temp;

            temp = imag[i];
            imag[i] = imag[j];
            imag[j] = temp;
        }
    }

    /* Iterative radix-2 Cooley-Tukey FFT. */
    for (block_length = 2U;
         block_length <= length;
         block_length <<= 1U)
    {
        uint32_t half_length = block_length >> 1U;
        float angle = -FFT_TWO_PI / (float)block_length;
        float step_real = cosf(angle);
        float step_imag = sinf(angle);
        uint32_t block;

        for (block = 0U; block < length; block += block_length)
        {
            float twiddle_real = 1.0f;
            float twiddle_imag = 0.0f;
            uint32_t k;

            for (k = 0U; k < half_length; ++k)
            {
                uint32_t even_index = block + k;
                uint32_t odd_index = even_index + half_length;
                float odd_real =
                    twiddle_real * real[odd_index]
                  - twiddle_imag * imag[odd_index];
                float odd_imag =
                    twiddle_real * imag[odd_index]
                  + twiddle_imag * real[odd_index];
                float even_real = real[even_index];
                float even_imag = imag[even_index];
                float next_twiddle_real;

                real[even_index] = even_real + odd_real;
                imag[even_index] = even_imag + odd_imag;
                real[odd_index] = even_real - odd_real;
                imag[odd_index] = even_imag - odd_imag;

                next_twiddle_real =
                    twiddle_real * step_real
                  - twiddle_imag * step_imag;
                twiddle_imag =
                    twiddle_real * step_imag
                  + twiddle_imag * step_real;
                twiddle_real = next_twiddle_real;
            }
        }
    }
}

static float FFT_MagnitudeSquared(uint32_t index)
{
    return fft_real[index] * fft_real[index]
         + fft_imag[index] * fft_imag[index];
}

static float FFT_RefinePeakFrequency(uint32_t peak_bin,
                                     float sample_rate_hz)
{
    float left_log;
    float center_log;
    float right_log;
    float denominator;
    float bin_offset = 0.0f;

    /*
     * Quadratic interpolation of the log spectrum is accurate for a
     * Hann-windowed single sine and avoids a 125 Hz-only result.
     */
    left_log = logf(FFT_MagnitudeSquared(peak_bin - 1U) + 1.0e-20f);
    center_log = logf(FFT_MagnitudeSquared(peak_bin) + 1.0e-20f);
    right_log = logf(FFT_MagnitudeSquared(peak_bin + 1U) + 1.0e-20f);

    denominator = left_log - 2.0f * center_log + right_log;
    if (fabsf(denominator) > 1.0e-12f)
    {
        bin_offset = 0.5f * (left_log - right_log) / denominator;
        if (bin_offset > 0.5f)
        {
            bin_offset = 0.5f;
        }
        else if (bin_offset < -0.5f)
        {
            bin_offset = -0.5f;
        }
    }

    return ((float)peak_bin + bin_offset)
         * sample_rate_hz / (float)FFT_SIZE;
}

static void FFT_FitXY(float frequency_hz,
                      float sample_rate_hz,
                      uint32_t sample_count,
                      float x_mean_counts,
                      float y_mean_counts,
                      float *x_peak_v,
                      float *y_peak_v,
                      float *x_phase_deg,
                      float *y_phase_deg)
{
    float phase_step = FFT_TWO_PI * frequency_hz / sample_rate_hz;
    float step_cos = cosf(phase_step);
    float step_sin = sinf(phase_step);
    float oscillator_cos = 1.0f;
    float oscillator_sin = 0.0f;
    float sum_cos = 0.0f;
    float sum_sin = 0.0f;
    float sum_cos_cos = 0.0f;
    float sum_sin_sin = 0.0f;
    float sum_cos_sin = 0.0f;
    float sum_x_cos = 0.0f;
    float sum_x_sin = 0.0f;
    float sum_y_cos = 0.0f;
    float sum_y_sin = 0.0f;
    float adc_scale = FFT_ADC_VREF_V / FFT_ADC_FULL_SCALE;
    float determinant;
    float x_cosine_coefficient;
    float x_sine_coefficient;
    float y_cosine_coefficient;
    float y_sine_coefficient;
    uint32_t i;

    *x_peak_v = 0.0f;
    *y_peak_v = 0.0f;
    *x_phase_deg = 0.0f;
    *y_phase_deg = 0.0f;

    /*
     * Remove the means of sine and cosine as well as the signal mean.
     * This makes the fit accurate when the capture contains a
     * non-integer number of periods.
     */
    for (i = 0U; i < sample_count; ++i)
    {
        sum_cos += oscillator_cos;
        sum_sin += oscillator_sin;

        {
            float next_cos =
                oscillator_cos * step_cos - oscillator_sin * step_sin;
            oscillator_sin =
                oscillator_cos * step_sin + oscillator_sin * step_cos;
            oscillator_cos = next_cos;
        }
    }

    sum_cos /= (float)sample_count;
    sum_sin /= (float)sample_count;
    oscillator_cos = 1.0f;
    oscillator_sin = 0.0f;

    for (i = 0U; i < sample_count; ++i)
    {
        float centered_cos = oscillator_cos - sum_cos;
        float centered_sin = oscillator_sin - sum_sin;
        float x_sample_v =
            ((float)FFT_GetXSample(i) - x_mean_counts) * adc_scale;
        float y_sample_v =
            ((float)FFT_GetYSample(i) - y_mean_counts) * adc_scale;
        float next_cos;

        sum_cos_cos += centered_cos * centered_cos;
        sum_sin_sin += centered_sin * centered_sin;
        sum_cos_sin += centered_cos * centered_sin;
        sum_x_cos += x_sample_v * centered_cos;
        sum_x_sin += x_sample_v * centered_sin;
        sum_y_cos += y_sample_v * centered_cos;
        sum_y_sin += y_sample_v * centered_sin;

        next_cos =
            oscillator_cos * step_cos - oscillator_sin * step_sin;
        oscillator_sin =
            oscillator_cos * step_sin + oscillator_sin * step_cos;
        oscillator_cos = next_cos;
    }

    determinant =
        sum_cos_cos * sum_sin_sin - sum_cos_sin * sum_cos_sin;
    if (fabsf(determinant) < 1.0e-12f)
    {
        return;
    }

    x_cosine_coefficient =
        (sum_x_cos * sum_sin_sin - sum_x_sin * sum_cos_sin)
        / determinant;
    x_sine_coefficient =
        (sum_x_sin * sum_cos_cos - sum_x_cos * sum_cos_sin)
        / determinant;

    y_cosine_coefficient =
        (sum_y_cos * sum_sin_sin - sum_y_sin * sum_cos_sin)
        / determinant;
    y_sine_coefficient =
        (sum_y_sin * sum_cos_cos - sum_y_cos * sum_cos_sin)
        / determinant;

    /*
     * For v = A*sin(wt + phi):
     *   sine coefficient   = A*cos(phi)
     *   cosine coefficient = A*sin(phi)
     */
    *x_peak_v = sqrtf(
        x_cosine_coefficient * x_cosine_coefficient
      + x_sine_coefficient * x_sine_coefficient);
    *y_peak_v = sqrtf(
        y_cosine_coefficient * y_cosine_coefficient
      + y_sine_coefficient * y_sine_coefficient);

    *x_phase_deg =
        atan2f(x_cosine_coefficient, x_sine_coefficient)
        * 180.0f / FFT_PI;
    *y_phase_deg =
        atan2f(y_cosine_coefficient, y_sine_coefficient)
        * 180.0f / FFT_PI;
}

void FFT_Init(void)
{
    adc_dma_done = 0U;
    memset(adc_xy_buf, 0, sizeof(adc_xy_buf));
}

void FFT_SetSampleClockCalibration(float calibration)
{
    if (calibration >= 0.90f && calibration <= 1.10f)
    {
        fft_sample_clock_calibration = calibration;
    }
}

uint8_t FFT_MeasureInput(FFT_InputResult *result)
{
    float sample_rate_hz;
    float x_mean_counts = 0.0f;
    float y_mean_counts = 0.0f;
    float adc_scale = FFT_ADC_VREF_V / FFT_ADC_FULL_SCALE;
    float window_cos = 1.0f;
    float window_sin = 0.0f;
    float window_angle = FFT_TWO_PI / (float)(FFT_SIZE - 1U);
    float window_step_cos = cosf(window_angle);
    float window_step_sin = sinf(window_angle);
    uint16_t x_minimum = 4095U;
    uint16_t x_maximum = 0U;
    uint16_t y_minimum = 4095U;
    uint16_t y_maximum = 0U;
    uint32_t minimum_bin;
    uint32_t maximum_bin;
    uint32_t peak_bin;
    float peak_power = 0.0f;
    float fft_peak_frequency_hz;
    float zero_cross_frequency_hz;
    float x_fitted_peak_v;
    float y_fitted_peak_v;
    uint32_t i;

    if (result == NULL)
    {
        return 0U;
    }

    memset(result, 0, sizeof(*result));

    if (FFT_Capture(&sample_rate_hz, FFT_SIZE) == 0U)
    {
        return 0U;
    }

    /*
     * The timer rate calculated from RCC registers is nominal.  The
     * current project runs from HSI, so use the measured clock scale
     * until an accurate HSE source is enabled.
     */
    sample_rate_hz *= fft_sample_clock_calibration;

    for (i = 0U; i < FFT_SIZE; ++i)
    {
        uint16_t x_sample = FFT_GetXSample(i);
        uint16_t y_sample = FFT_GetYSample(i);
        x_mean_counts += (float)x_sample;
        y_mean_counts += (float)y_sample;

        if (x_sample < x_minimum)
        {
            x_minimum = x_sample;
        }
        if (x_sample > x_maximum)
        {
            x_maximum = x_sample;
        }
        if (y_sample < y_minimum)
        {
            y_minimum = y_sample;
        }
        if (y_sample > y_maximum)
        {
            y_maximum = y_sample;
        }
    }
    x_mean_counts /= (float)FFT_SIZE;
    y_mean_counts /= (float)FFT_SIZE;

    /*
     * Remove measured DC and apply a Hann window.  A recursive
     * oscillator avoids thousands of slow cosf() calls.
     */
    for (i = 0U; i < FFT_SIZE; ++i)
    {
        float window = 0.5f - 0.5f * window_cos;
        float centered_sample =
            ((float)FFT_GetXSample(i) - x_mean_counts) * adc_scale;
        float next_window_cos;

        fft_real[i] = centered_sample * window;
        fft_imag[i] = 0.0f;

        next_window_cos =
            window_cos * window_step_cos - window_sin * window_step_sin;
        window_sin =
            window_cos * window_step_sin + window_sin * window_step_cos;
        window_cos = next_window_cos;
    }

    FFT_ComplexForward(fft_real, fft_imag, FFT_SIZE);

    minimum_bin = (uint32_t)ceilf(
        FFT_SEARCH_MIN_HZ * (float)FFT_SIZE / sample_rate_hz);
    maximum_bin = (uint32_t)floorf(
        FFT_SEARCH_MAX_HZ * (float)FFT_SIZE / sample_rate_hz);

    if (minimum_bin < 2U)
    {
        minimum_bin = 2U;
    }
    if (maximum_bin > FFT_SIZE / 2U - 2U)
    {
        maximum_bin = FFT_SIZE / 2U - 2U;
    }

    peak_bin = minimum_bin;
    for (i = minimum_bin; i <= maximum_bin; ++i)
    {
        float power = FFT_MagnitudeSquared(i);
        if (power > peak_power)
        {
            peak_power = power;
            peak_bin = i;
        }
    }

    result->sample_rate_hz = sample_rate_hz;
    fft_peak_frequency_hz =
        FFT_RefinePeakFrequency(peak_bin, sample_rate_hz);
    result->frequency_raw_hz = fft_peak_frequency_hz;

    /*
     * Accept the zero-crossing estimate only when it agrees with the
     * spectral peak.  This combines the FFT's noise immunity with the
     * much finer multi-period timing estimate.
     */
    if (fft_peak_frequency_hz <= FFT_ZERO_CROSS_MAX_USE_HZ
        && FFT_EstimateXFrequencyByZeroCrossings(
            FFT_SIZE,
            x_mean_counts,
            sample_rate_hz,
            fft_peak_frequency_hz,
            &zero_cross_frequency_hz) != 0U
        && fabsf(zero_cross_frequency_hz
               - fft_peak_frequency_hz)
            <= FFT_INPUT_VALID_MARGIN_HZ)
    {
        result->frequency_raw_hz = zero_cross_frequency_hz;
    }

    /* Keep the measured frequency; the input is not restricted to a
     * software-defined 100 Hz grid. */
    result->frequency_hz = result->frequency_raw_hz;
    if (result->frequency_hz < FFT_INPUT_MIN_HZ)
    {
        result->frequency_hz = FFT_INPUT_MIN_HZ;
    }
    else if (result->frequency_hz > FFT_INPUT_MAX_HZ)
    {
        result->frequency_hz = FFT_INPUT_MAX_HZ;
    }

    /*
     * Use the measured raw frequency for amplitude fitting.  This
     * avoids attenuation caused by fitting at an artificially rounded
     * frequency.
     */
    FFT_FitXY(
        result->frequency_raw_hz,
        sample_rate_hz,
        FFT_SIZE,
        x_mean_counts,
        y_mean_counts,
        &x_fitted_peak_v,
        &y_fitted_peak_v,
        &result->x_phase_deg,
        &result->y_phase_deg);

    result->adc_bias_v = x_mean_counts * adc_scale;
    result->adc_vpp = 2.0f * x_fitted_peak_v;
    result->input_vpp =
        result->adc_vpp / FFT_INPUT_FRONTEND_GAIN;
    result->y_adc_bias_v = y_mean_counts * adc_scale;
    result->y_adc_vpp = 2.0f * y_fitted_peak_v;
    result->y_input_vpp =
        result->y_adc_vpp / FFT_INPUT_FRONTEND_GAIN;

    result->phase_diff_deg = FFT_WrapPhaseDeg(
        result->y_phase_deg - result->x_phase_deg);
    result->phase_diff_calibrated_deg = FFT_WrapPhaseDeg(
        result->phase_diff_deg - FFT_INTERNAL_XY_PHASE_OFFSET_DEG);

    result->adc_min = x_minimum;
    result->adc_max = x_maximum;
    result->y_adc_min = y_minimum;
    result->y_adc_max = y_maximum;
    result->clipped =
        (x_minimum <= 8U || x_maximum >= 4087U) ? 1U : 0U;
    result->y_clipped =
        (y_minimum <= 8U || y_maximum >= 4087U) ? 1U : 0U;

    result->valid =
        (peak_power > 1.0e-6f
      && result->frequency_raw_hz
            >= FFT_INPUT_MIN_HZ - FFT_INPUT_VALID_MARGIN_HZ
      && result->frequency_raw_hz
            <= FFT_INPUT_MAX_HZ + FFT_INPUT_VALID_MARGIN_HZ
      && result->adc_vpp > 0.05f
      && result->clipped == 0U) ? 1U : 0U;

    result->phase_valid =
        (result->valid != 0U
      && result->y_adc_vpp > 0.05f
      && result->y_clipped == 0U) ? 1U : 0U;

    return result->valid;
}

uint8_t FFT_MeasureSecondHarmonicPhase(
    FFT_InputResult *result,
    float input_frequency_hz)
{
    return FFT_MeasureSecondHarmonicPhaseSamples(
        result,
        input_frequency_hz,
        FFT_SIZE);
}

uint8_t FFT_MeasurePhaseRelationshipSamples(
    FFT_InputResult *result,
    float input_frequency_hz,
    float output_frequency_hz,
    float input_phase_multiplier,
    uint32_t sample_count)
{
    float sample_rate_hz;
    float x_mean_counts = 0.0f;
    float y_mean_counts = 0.0f;
    float adc_scale = FFT_ADC_VREF_V / FFT_ADC_FULL_SCALE;
    float x_fitted_peak_v;
    float y_fitted_peak_v;
    float unused_peak_v;
    float unused_phase_deg;
    float zero_cross_frequency_hz;
    float y_zero_cross_frequency_hz;
    uint8_t zero_cross_valid;
    uint8_t y_zero_cross_valid;
    uint16_t x_minimum = 4095U;
    uint16_t x_maximum = 0U;
    uint16_t y_minimum = 4095U;
    uint16_t y_maximum = 0U;
    uint32_t i;

    if (result == NULL
        || input_frequency_hz
            < FFT_INPUT_MIN_HZ - FFT_INPUT_VALID_MARGIN_HZ
        || input_frequency_hz
            > FFT_INPUT_MAX_HZ + FFT_INPUT_VALID_MARGIN_HZ
        || output_frequency_hz <= 0.0f
        || input_phase_multiplier <= 0.0f
        || sample_count == 0U
        || sample_count > FFT_SIZE)
    {
        return 0U;
    }

    memset(result, 0, sizeof(*result));

    if (FFT_Capture(
            &sample_rate_hz,
            sample_count) == 0U)
    {
        return 0U;
    }

    sample_rate_hz *= fft_sample_clock_calibration;
    if (output_frequency_hz >= 0.45f * sample_rate_hz)
    {
        return 0U;
    }

    for (i = 0U; i < sample_count; ++i)
    {
        uint16_t x_sample = FFT_GetXSample(i);
        uint16_t y_sample = FFT_GetYSample(i);

        x_mean_counts += (float)x_sample;
        y_mean_counts += (float)y_sample;

        if (x_sample < x_minimum)
        {
            x_minimum = x_sample;
        }
        if (x_sample > x_maximum)
        {
            x_maximum = x_sample;
        }
        if (y_sample < y_minimum)
        {
            y_minimum = y_sample;
        }
        if (y_sample > y_maximum)
        {
            y_maximum = y_sample;
        }
    }

    x_mean_counts /= (float)sample_count;
    y_mean_counts /= (float)sample_count;
    zero_cross_valid =
        FFT_EstimateXFrequencyByZeroCrossings(
            sample_count,
            x_mean_counts,
            sample_rate_hz,
            input_frequency_hz,
            &zero_cross_frequency_hz);
    y_zero_cross_valid =
        FFT_EstimateYFrequencyByZeroCrossings(
            sample_count,
            y_mean_counts,
            sample_rate_hz,
            output_frequency_hz,
            &y_zero_cross_frequency_hz);

    if (fabsf(output_frequency_hz - input_frequency_hz) < 0.01f)
    {
        FFT_FitXY(
            input_frequency_hz,
            sample_rate_hz,
            sample_count,
            x_mean_counts,
            y_mean_counts,
            &x_fitted_peak_v,
            &y_fitted_peak_v,
            &result->x_phase_deg,
            &result->y_phase_deg);
    }
    else
    {
        FFT_FitXY(
            input_frequency_hz,
            sample_rate_hz,
            sample_count,
            x_mean_counts,
            y_mean_counts,
            &x_fitted_peak_v,
            &unused_peak_v,
            &result->x_phase_deg,
            &unused_phase_deg);
        FFT_FitXY(
            output_frequency_hz,
            sample_rate_hz,
            sample_count,
            x_mean_counts,
            y_mean_counts,
            &unused_peak_v,
            &y_fitted_peak_v,
            &unused_phase_deg,
            &result->y_phase_deg);
    }

    result->sample_rate_hz = sample_rate_hz;
    result->frequency_raw_hz =
        (zero_cross_valid != 0U)
      ? zero_cross_frequency_hz
      : input_frequency_hz;
    result->frequency_hz = result->frequency_raw_hz;
    result->y_frequency_raw_hz =
        (y_zero_cross_valid != 0U)
      ? y_zero_cross_frequency_hz
      : 0.0f;

    result->adc_bias_v = x_mean_counts * adc_scale;
    result->adc_vpp = 2.0f * x_fitted_peak_v;
    result->input_vpp =
        result->adc_vpp / FFT_INPUT_FRONTEND_GAIN;
    result->y_adc_bias_v = y_mean_counts * adc_scale;
    result->y_adc_vpp = 2.0f * y_fitted_peak_v;
    result->y_input_vpp =
        result->y_adc_vpp / FFT_INPUT_FRONTEND_GAIN;

    result->phase_diff_deg = FFT_WrapPhaseDeg(
        result->y_phase_deg
        - input_phase_multiplier * result->x_phase_deg);
    result->phase_diff_calibrated_deg = FFT_WrapPhaseDeg(
        result->phase_diff_deg - FFT_INTERNAL_XY_PHASE_OFFSET_DEG);

    result->adc_min = x_minimum;
    result->adc_max = x_maximum;
    result->y_adc_min = y_minimum;
    result->y_adc_max = y_maximum;
    result->clipped =
        (x_minimum <= 8U || x_maximum >= 4087U) ? 1U : 0U;
    result->y_clipped =
        (y_minimum <= 8U || y_maximum >= 4087U) ? 1U : 0U;

    result->valid =
        (result->adc_vpp > 0.05f
      && result->clipped == 0U) ? 1U : 0U;
    result->phase_valid =
        (result->valid != 0U
      && result->y_adc_vpp > 0.05f
      && result->y_clipped == 0U) ? 1U : 0U;

    return result->valid;
}

uint8_t FFT_MeasurePhaseRelationship(
    FFT_InputResult *result,
    float input_frequency_hz,
    float output_frequency_hz,
    float input_phase_multiplier)
{
    return FFT_MeasurePhaseRelationshipSamples(
        result,
        input_frequency_hz,
        output_frequency_hz,
        input_phase_multiplier,
        FFT_DEFAULT_PHASE_CAPTURE_SIZE);
}

uint8_t FFT_MeasurePhaseAtFrequency(FFT_InputResult *result,
                                    float known_frequency_hz)
{
    return FFT_MeasurePhaseRelationship(
        result,
        known_frequency_hz,
        known_frequency_hz,
        1.0f);
}

uint8_t FFT_MeasureSecondHarmonicPhaseSamples(
    FFT_InputResult *result,
    float input_frequency_hz,
    uint32_t capture_sample_count)
{
    float sample_rate_hz;
    float x_mean_counts = 0.0f;
    float y_mean_counts = 0.0f;
    float x_energy_counts = 0.0f;
    float x_peak_counts;
    float adc_scale = FFT_ADC_VREF_V / FFT_ADC_FULL_SCALE;
    float phase_step;
    float lag_sine;
    float cosine_denominator;
    float sum_sin2_sin2 = 0.0f;
    float sum_cos2_cos2 = 0.0f;
    float sum_sin2_cos2 = 0.0f;
    float sum_y_sin2 = 0.0f;
    float sum_y_cos2 = 0.0f;
    float determinant;
    float y_sine_coefficient;
    float y_cosine_coefficient;
    float zero_cross_frequency_hz;
    uint8_t zero_cross_valid;
    uint16_t x_minimum = 4095U;
    uint16_t x_maximum = 0U;
    uint16_t y_minimum = 4095U;
    uint16_t y_maximum = 0U;
    uint32_t lag;
    uint32_t fit_sample_count;
    uint32_t i;

    if (result == NULL
        || input_frequency_hz < FFT_INPUT_MIN_HZ
        || input_frequency_hz > FFT_INPUT_MAX_HZ
        || capture_sample_count < 64U
        || capture_sample_count > FFT_SIZE)
    {
        return 0U;
    }

    memset(result, 0, sizeof(*result));
    if (FFT_Capture(
            &sample_rate_hz,
            capture_sample_count) == 0U)
    {
        return 0U;
    }

    sample_rate_hz *= fft_sample_clock_calibration;

    for (i = 0U; i < capture_sample_count; ++i)
    {
        uint16_t x_sample = FFT_GetXSample(i);
        uint16_t y_sample = FFT_GetYSample(i);

        x_mean_counts += (float)x_sample;
        y_mean_counts += (float)y_sample;

        if (x_sample < x_minimum)
        {
            x_minimum = x_sample;
        }
        if (x_sample > x_maximum)
        {
            x_maximum = x_sample;
        }
        if (y_sample < y_minimum)
        {
            y_minimum = y_sample;
        }
        if (y_sample > y_maximum)
        {
            y_maximum = y_sample;
        }
    }

    x_mean_counts /= (float)capture_sample_count;
    y_mean_counts /= (float)capture_sample_count;
    zero_cross_valid =
        FFT_EstimateXFrequencyByZeroCrossings(
            capture_sample_count,
            x_mean_counts,
            sample_rate_hz,
            input_frequency_hz,
            &zero_cross_frequency_hz);

    for (i = 0U; i < capture_sample_count; ++i)
    {
        float centered_x =
            (float)FFT_GetXSample(i) - x_mean_counts;
        x_energy_counts += centered_x * centered_x;
    }
    x_peak_counts = sqrtf(
        2.0f * x_energy_counts
        / (float)capture_sample_count);
    if (x_peak_counts < 1.0f)
    {
        return 0U;
    }

    /*
     * Use a delay close to one quarter of the X period.  Compared with
     * a one-sample derivative this keeps the cosine estimate quiet even
     * at the 1 kHz lower limit:
     *
     * x[n+L]-x[n-L] = 2*A*cos(theta)*sin(w*L*Ts).
     */
    phase_step = FFT_TWO_PI * input_frequency_hz / sample_rate_hz;
    lag = (uint32_t)(
        sample_rate_hz / (4.0f * input_frequency_hz) + 0.5f);
    if (lag < 1U)
    {
        lag = 1U;
    }
    else if (lag > capture_sample_count / 4U)
    {
        lag = capture_sample_count / 4U;
    }

    lag_sine = sinf(phase_step * (float)lag);
    cosine_denominator = 2.0f * x_peak_counts * lag_sine;
    if (fabsf(cosine_denominator) < 1.0e-6f)
    {
        return 0U;
    }

    fit_sample_count = capture_sample_count - 2U * lag;
    for (i = lag; i < capture_sample_count - lag; ++i)
    {
        float sine_theta =
            ((float)FFT_GetXSample(i) - x_mean_counts)
            / x_peak_counts;
        float cosine_theta =
            ((float)FFT_GetXSample(i + lag)
             - (float)FFT_GetXSample(i - lag))
            / cosine_denominator;
        float sine_2theta = 2.0f * sine_theta * cosine_theta;
        float cosine_2theta =
            1.0f - 2.0f * sine_theta * sine_theta;
        float y_sample_v =
            ((float)FFT_GetYSample(i) - y_mean_counts)
            * adc_scale;

        sum_sin2_sin2 += sine_2theta * sine_2theta;
        sum_cos2_cos2 += cosine_2theta * cosine_2theta;
        sum_sin2_cos2 += sine_2theta * cosine_2theta;
        sum_y_sin2 += y_sample_v * sine_2theta;
        sum_y_cos2 += y_sample_v * cosine_2theta;
    }

    determinant =
        sum_sin2_sin2 * sum_cos2_cos2
      - sum_sin2_cos2 * sum_sin2_cos2;
    if (fit_sample_count == 0U
        || fabsf(determinant) < 1.0e-12f)
    {
        return 0U;
    }

    y_sine_coefficient =
        (sum_y_sin2 * sum_cos2_cos2
         - sum_y_cos2 * sum_sin2_cos2)
        / determinant;
    y_cosine_coefficient =
        (sum_y_cos2 * sum_sin2_sin2
         - sum_y_sin2 * sum_sin2_cos2)
        / determinant;

    result->sample_rate_hz = sample_rate_hz;
    result->frequency_raw_hz =
        (zero_cross_valid != 0U)
      ? zero_cross_frequency_hz
      : input_frequency_hz;
    result->frequency_hz = result->frequency_raw_hz;
    result->adc_bias_v = x_mean_counts * adc_scale;
    result->adc_vpp = 2.0f * x_peak_counts * adc_scale;
    result->input_vpp =
        result->adc_vpp / FFT_INPUT_FRONTEND_GAIN;
    result->y_adc_bias_v = y_mean_counts * adc_scale;
    result->y_adc_vpp = 2.0f * sqrtf(
        y_sine_coefficient * y_sine_coefficient
      + y_cosine_coefficient * y_cosine_coefficient);
    result->y_input_vpp =
        result->y_adc_vpp / FFT_INPUT_FRONTEND_GAIN;

    result->x_phase_deg = 0.0f;
    result->y_phase_deg =
        atan2f(y_cosine_coefficient, y_sine_coefficient)
        * 180.0f / FFT_PI;
    result->phase_diff_deg =
        FFT_WrapPhaseDeg(result->y_phase_deg);
    result->phase_diff_calibrated_deg = FFT_WrapPhaseDeg(
        result->phase_diff_deg - FFT_INTERNAL_XY_PHASE_OFFSET_DEG);

    result->adc_min = x_minimum;
    result->adc_max = x_maximum;
    result->y_adc_min = y_minimum;
    result->y_adc_max = y_maximum;
    result->clipped =
        (x_minimum <= 8U || x_maximum >= 4087U) ? 1U : 0U;
    result->y_clipped =
        (y_minimum <= 8U || y_maximum >= 4087U) ? 1U : 0U;
    result->valid =
        (result->adc_vpp > 0.05f
      && result->clipped == 0U) ? 1U : 0U;
    result->phase_valid =
        (result->valid != 0U
      && result->y_adc_vpp > 0.05f
      && result->y_clipped == 0U) ? 1U : 0U;

    return result->valid;
}
