#ifndef __FFT_H
#define __FFT_H

#include <stdint.h>

/*
 * Input acquisition for the 2026 Lissajous display controller.
 *
 * The oscilloscope input is 4 Vpp after item 1 is calibrated:
 *     x(t) = 2 * sin(2*pi*f*t) V
 *
 * Two identical input front ends are used:
 *   X -> 0.5 gain + 1.65 V bias -> PA1 / ADC2
 *   Y -> 0.5 gain + 1.65 V bias -> PC5 / ADC1
 */
#define FFT_SIZE                    8192U
#define FFT_SAMPLE_RATE_HZ          1024000U
#define FFT_TIMER_CLOCK_FALLBACK_HZ 128000000U

#define FFT_ADC_VREF_V              3.3f
#define FFT_ADC_FULL_SCALE          4095.0f
#define FFT_INPUT_FRONTEND_GAIN     0.5f

#define FFT_INPUT_MIN_HZ            1000.0f
#define FFT_INPUT_MAX_HZ            100000.0f
#define FFT_INPUT_VALID_MARGIN_HZ    500.0f

/*
 * Search slightly beyond the legal input range so tones at exactly
 * 1 kHz or 100 kHz are not forced onto an FFT search boundary.
 * The final validity check still enforces the 1 kHz to 100 kHz range.
 */
#define FFT_SEARCH_MIN_HZ           500.0f
#define FFT_SEARCH_MAX_HZ           110000.0f

typedef struct
{
    float sample_rate_hz;
    float frequency_raw_hz;
    float frequency_hz;
    /* Raw Y-channel frequency, measured independently of the phase fit. */
    float y_frequency_raw_hz;

    float adc_bias_v;
    float adc_vpp;
    float input_vpp;
    float y_adc_bias_v;
    float y_adc_vpp;
    float y_input_vpp;

    float x_phase_deg;
    float y_phase_deg;
    float phase_diff_deg;
    float phase_diff_calibrated_deg;

    uint16_t adc_min;
    uint16_t adc_max;
    uint16_t y_adc_min;
    uint16_t y_adc_max;
    uint8_t clipped;
    uint8_t y_clipped;
    uint8_t phase_valid;
    uint8_t valid;
} FFT_InputResult;

/*
 * ADC dual-mode packed word:
 *   bits 31:16 = ADC2 / PA1 / X
 *   bits 15:0  = ADC1 / PC5 / Y
 */
extern uint32_t adc_xy_buf[FFT_SIZE];
extern volatile uint8_t fft_capture_error;

void FFT_Init(void);
void FFT_SetSampleClockCalibration(float calibration);

/*
 * Capture one simultaneous ADC1/ADC2 frame with TIM8 TRGO and DMA,
 * then estimate X frequency, X/Y amplitudes and X/Y phase difference.
 *
 * Return value:
 *   1: capture and analysis succeeded
 *   0: ADC/DMA timeout or no valid 1 kHz to 100 kHz sine was found
 *
 * result->valid indicates a valid X frequency/amplitude measurement.
 * result->phase_valid additionally requires a valid unclipped Y.
 */
uint8_t FFT_MeasureInput(FFT_InputResult *result);

/*
 * Fast tracking path used after the input frequency has been identified.
 * It captures 1024 simultaneous X/Y samples and fits both channels
 * directly at known_frequency_hz, avoiding another full 8192-point FFT.
 */
uint8_t FFT_MeasurePhaseAtFrequency(FFT_InputResult *result,
                                    float known_frequency_hz);

/*
 * General phase relationship measurement:
 *   phase_diff = phase(output_frequency_hz)
 *              - input_phase_multiplier * phase(input_frequency_hz)
 *
 * For item 3 use output_frequency_hz = 2 * input_frequency_hz and
 * input_phase_multiplier = 2.
 */
uint8_t FFT_MeasurePhaseRelationship(
    FFT_InputResult *result,
    float input_frequency_hz,
    float output_frequency_hz,
    float input_phase_multiplier);

/*
 * Same measurement with a caller-selected capture length.  The
 * competition controller exposes this value in question.c so all
 * field-adjustable control parameters stay in one place.
 */
uint8_t FFT_MeasurePhaseRelationshipSamples(
    FFT_InputResult *result,
    float input_frequency_hz,
    float output_frequency_hz,
    float input_phase_multiplier,
    uint32_t sample_count);

/*
 * Item-3-specific measurement.  The captured X waveform itself is used
 * to construct sin(2*theta) and cos(2*theta), then Y is fitted against
 * those bases.  The returned phase difference is phase(Y)-2*phase(X).
 */
uint8_t FFT_MeasureSecondHarmonicPhase(
    FFT_InputResult *result,
    float input_frequency_hz);

/*
 * Same second-harmonic measurement with a caller-selected capture length.
 * Shorter captures reduce loop delay and prevent phase-slope aliasing
 * during acquisition at medium and high input frequencies.
 */
uint8_t FFT_MeasureSecondHarmonicPhaseSamples(
    FFT_InputResult *result,
    float input_frequency_hz,
    uint32_t capture_sample_count);

#endif
