#include "question.h"
#include "ad9833.h"
#include "main.h"
#include "usart.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static FFT_InputResult input_signal;

/* =====================================================================
 * 用户现场调参区
 *
 * 一般只修改这一段，不要修改下面的控制状态和算法。
 * 相位正方向：正数让 Y 更超前，负数让 Y 更滞后。
 * 如果示波器看到 Y 超前目标角度，应把对应 PHASE_TRIM 设为负数；
 * 如果示波器看到 Y 滞后目标角度，应把它设为正数。
 * ===================================================================*/

/* 题目模式编号，不需要修改。 */
#define QUESTION_MODE_1                  1U
#define QUESTION_MODE_2                  2U
#define QUESTION_MODE_3                  3U
#define QUESTION_MODE_4_DIAGONAL         4U
#define QUESTION_MODE_4_CIRCLE           5U
#define QUESTION_MODE_4_INFINITY         6U

/* 第一、二、三问固定为4 Vpp。 */
#define QUESTION_Q123_AMP_CODE           141U

/*
 * 第四问幅度档位：
 * 示波器为0.5 V/div，因此1/2/3/4 Vpp对应Y方向2/4/6/8 div。
 * 这里只保存实测映射，不包含串口屏、按键或档位切换逻辑。
 */
#define QUESTION_4_1VPP_AMP_CODE          35U
#define QUESTION_4_2VPP_AMP_CODE          70U
#define QUESTION_4_3VPP_AMP_CODE         105U
#define QUESTION_4_4VPP_AMP_CODE         141U

/*
 * 示波器显示点相对于 ADC 反馈点的固定相位补偿，单位为度。
 * 现在全部先设为 0.0f。第一问试相位时只改 QUESTION_1_PHASE_TRIM_DEG。
 * 第二问的理论目标仍是 +90 度；第三问控制的是 phiY - 2*phiX。
 */
#define QUESTION_1_PHASE_TRIM_DEG         0.0f
#define QUESTION_2_PHASE_TRIM_DEG         0.0f
#define QUESTION_3_PHASE_TRIM_DEG         0.0f
#define QUESTION_4_PHASE_TRIM_DEG         0.0f

/*
 * Measured same-frequency phase error at the oscilloscope:
 *     1 kHz   -> 0.00 deg
 *     100 kHz -> +2.50 deg
 *
 * Positive means the physical Y waveform leads the requested target, so
 * the controller applies the opposite (negative) compensation.  If the
 * observed ellipse opens in the opposite phase direction, change the
 * 100 kHz value to -2.50f.  This calibration is only for the 1:1 modes;
 * the 2:1 figure-eight relationship has its own phase trim.
 */
#define QUESTION_SCOPE_ERROR_AT_1KHZ_DEG    0.00f
#define QUESTION_SCOPE_ERROR_AT_100KHZ_DEG  2.50f

/*
 * 第一、二问锁相阶段每次使用的同步 ADC 样本数。
 * 1024：约 5 ms 更新一次，当前推荐值。
 * 增大可降低测量噪声但反应变慢；不建议小于 1024。
 * 第三问仍使用专门的 8192 点二倍频相位算法，不受此项影响。
 */
#define QUESTION_PHASE_CAPTURE_SAMPLES   1024U

/*
 * TIM8/ADC sample-clock calibration.  The old 0.99380 value measured
 * a real 60 kHz input as 60042.18 Hz on average.  0.99310 removes that
 * proportional error.  Recalibrate this value if the MCU clock changes.
 */
#define QUESTION_SAMPLE_CLOCK_CALIBRATION 0.99310f

/* 共用相位环：增益、死区、锁定后最大步进、捕获时最大步进。 */
#define CTRL_PHASE_REGISTER_GAIN          1.00f
#define CTRL_PHASE_STEP_DEADBAND_DEG      0.05f
#define CTRL_PHASE_FINE_MAX_STEP_DEG      1.5f
#define CTRL_PHASE_ACQUIRE_MAX_STEP_DEG  12.0f

/*
 * 用测得的残余频差预测下一帧相位漂移。
 * 0.5 表示补偿下一周期漂移的一半，使误差围绕 0 度对称。
 */
#define CTRL_PHASE_DRIFT_FEEDFORWARD      0.50f

/* 频率环增益：第一/二问共用一项，第三问可单独调整。 */
#define CTRL_Q12_FREQUENCY_GAIN           0.50f
#define CTRL_Q3_FREQUENCY_GAIN            0.50f
#define CTRL_DIRECT_COARSE_GAIN            0.75f
#define CTRL_DIRECT_COARSE_SWITCH_HZ       5.0f
#define CTRL_DIRECT_COARSE_MIN_FRAMES      4U
#define CTRL_FINE_FREQUENCY_GAIN          0.15f
#define CTRL_FINE_FREQUENCY_DEADBAND_HZ   0.04f

/* AD9833 相对基础频率允许修正的最大范围。 */
#define CTRL_FUNDAMENTAL_TRIM_LIMIT_HZ  250.0f
#define CTRL_HARMONIC_TRIM_LIMIT_HZ     500.0f

/* 未锁定/锁定后的频差观察窗口。越短越快，越长越稳。 */
#define CTRL_FREQUENCY_ACQUIRE_WINDOW_MS 125U
#define CTRL_FREQUENCY_FINE_WINDOW_MS    250U

/* 无效数据间隔超过此值时重新开始相位跟踪。 */
#define CTRL_MAX_PHASE_INTERVAL_MS       500U

/* 锁定判据以及连续满足判据的窗口数。 */
#define CTRL_LOCK_FREQUENCY_ERROR_HZ      0.15f
#define CTRL_HARMONIC_LOCK_FREQUENCY_HZ   0.80f
#define CTRL_LOCK_PHASE_ERROR_DEG         3.0f
#define CTRL_LOCK_CONFIRM_WINDOWS         3U
#define CTRL_HARMONIC_LOCK_CONFIRM_WINDOWS 2U

/*
 * Hold the selected DDS word after lock, but release it after a sustained
 * large error.  Without this release the state can report "unlocked"
 * while the frequency loop remains permanently frozen.
 */
#define CTRL_HOLD_RELEASE_FREQUENCY_HZ     1.50f
#define CTRL_HOLD_RELEASE_PHASE_DEG        8.0f
#define CTRL_HOLD_RELEASE_WINDOWS          2U

/* 启动时用于确定输入频率的完整 FFT 帧数。 */
#define CTRL_STARTUP_FREQUENCY_FRAMES     3U

/*
 * Runtime input-frequency reacquisition:
 * - Run a full FFT every 1000 ms while the output is active.
 * - Once locked, ignore measurement movement smaller than 500 Hz.
 * - If lock has already been lost, use a 150 Hz threshold to recover.
 * - Force an early FFT after several invalid fast phase captures.
 */
#define CTRL_INPUT_RECHECK_PERIOD_MS    1000U
#define CTRL_INPUT_CHANGE_HZ              500.0f
#define CTRL_TRIM_RECHECK_MARGIN_HZ         1.0f
#define CTRL_TRIM_RECHECK_ERROR_HZ          2.0f
#define CTRL_INVALID_PHASE_FRAMES         3U
#define QUESTION_UART_PERIOD_MS          500U

/* AD9833 PHASE0 为 12 位，这是硬件常数，不要修改。 */
#define AD9833_PHASE_LSB_DEG             (360.0f / 4096.0f)

static uint8_t q1_output_initialized;
static uint8_t q1_phase_tracking;
static uint8_t q1_locked;
static uint8_t q1_frequency_hold;
static uint8_t q1_lock_good_windows;
static uint8_t q1_hold_bad_windows;
static uint8_t q1_startup_frequency_count;
static uint8_t q1_fft_initialized;
static uint8_t q1_active_mode;
static uint8_t q1_active_amp_code;
static uint8_t q1_invalid_phase_frames;
static uint8_t q1_force_frequency_check;
static uint32_t q1_previous_phase_tick;
static uint32_t q1_phase_window_start_tick;
static uint32_t q1_last_frequency_check_tick;
static float q1_input_frequency_hz;
static float q1_base_frequency_hz;
static float q1_startup_frequency_samples[
    CTRL_STARTUP_FREQUENCY_FRAMES];
static float q1_command_frequency_hz;
static float q1_last_programmed_frequency_hz;
static float q1_frequency_trim_hz;
static float q1_frequency_error_hz;
static float q1_direct_frequency_error_sum_hz;
static float q1_direct_frequency_error_hz;
static uint16_t q1_direct_frequency_error_count;
static float q1_phase_error_deg;
static float q1_phase_command_deg;
static float q1_previous_phase_diff_deg;
static float q1_phase_window_change_deg;
static uint32_t q1_last_phase_window_ms;
static uint32_t q1_last_phase_interval_ms;
static uint32_t q1_last_capture_samples;
static uint8_t q1_last_capture_was_frequency_check;
static char q1_uart_buffer[448];
static volatile uint8_t q1_uart_busy;
static uint32_t q1_uart_last_tick;

static float Question_WrapPhaseDeg(float phase_deg)
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

static void Question_SendDebug(void)
{
    const char *prefix;
    uint32_t now_tick = HAL_GetTick();
    int length;

    if (q1_uart_busy != 0U
        || (now_tick - q1_uart_last_tick)
            < QUESTION_UART_PERIOD_MS)
    {
        return;
    }

    if (q1_active_mode == QUESTION_MODE_4_INFINITY)
    {
        prefix = "QINF";
    }
    else if (q1_active_mode == QUESTION_MODE_3)
    {
        prefix = "Q3D";
    }
    else
    {
        prefix = "QF";
    }

    length = snprintf(
        q1_uart_buffer,
        sizeof(q1_uart_buffer),
        "%s,mode=%u,cap=%u,scan=%u,samples=%lu,"
        "valid=%u,pv=%u,lock=%u,hold=%u,"
        "fraw=%.2f,fy=%.2f,fin=%.2f,fout=%.2f,fcmd=%.3f,"
        "trim=%.3f,df=%.3f,cdf=%.3f,win=%lu,step=%lu,"
        "xvpp=%.3f,yvpp=%.3f,xph=%.2f,yph=%.2f,"
        "rel=%.2f,err=%.2f,pcmd=%.2f\r\n",
        prefix,
        q1_active_mode,
        fft_capture_error,
        q1_last_capture_was_frequency_check,
        (unsigned long)q1_last_capture_samples,
        input_signal.valid,
        input_signal.phase_valid,
        q1_locked,
        q1_frequency_hold,
        input_signal.frequency_raw_hz,
        input_signal.y_frequency_raw_hz,
        q1_input_frequency_hz,
        q1_base_frequency_hz,
        q1_command_frequency_hz,
        q1_frequency_trim_hz,
        q1_frequency_error_hz,
        q1_direct_frequency_error_hz,
        (unsigned long)q1_last_phase_window_ms,
        (unsigned long)q1_last_phase_interval_ms,
        input_signal.input_vpp,
        input_signal.y_input_vpp,
        input_signal.x_phase_deg,
        input_signal.y_phase_deg,
        input_signal.phase_diff_calibrated_deg,
        q1_phase_error_deg,
        q1_phase_command_deg);

    if (length <= 0)
    {
        return;
    }
    if ((size_t)length >= sizeof(q1_uart_buffer))
    {
        length = (int)sizeof(q1_uart_buffer) - 1;
    }

    if (HAL_UART_Transmit_IT(
            &huart2,
            (uint8_t *)q1_uart_buffer,
            (uint16_t)length) == HAL_OK)
    {
        q1_uart_busy = 1U;
        q1_uart_last_tick = now_tick;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        q1_uart_busy = 0U;
    }
}

static uint8_t Question_InputFrequencyChanged(void)
{
    return (fabsf(input_signal.frequency_raw_hz
                - q1_input_frequency_hz)
            >= CTRL_INPUT_CHANGE_HZ) ? 1U : 0U;
}

static uint8_t Question_IsHarmonicMode(void)
{
    return (q1_active_mode == QUESTION_MODE_3
         || q1_active_mode == QUESTION_MODE_4_INFINITY) ? 1U : 0U;
}

static float Question_GetFundamentalPhaseCompensationDeg(
    float frequency_hz)
{
    float normalized_frequency;
    float measured_scope_error_deg;

    if (frequency_hz <= FFT_INPUT_MIN_HZ)
    {
        normalized_frequency = 0.0f;
    }
    else if (frequency_hz >= FFT_INPUT_MAX_HZ)
    {
        normalized_frequency = 1.0f;
    }
    else
    {
        normalized_frequency =
            (frequency_hz - FFT_INPUT_MIN_HZ)
            / (FFT_INPUT_MAX_HZ - FFT_INPUT_MIN_HZ);
    }

    measured_scope_error_deg =
        QUESTION_SCOPE_ERROR_AT_1KHZ_DEG
      + normalized_frequency
        * (QUESTION_SCOPE_ERROR_AT_100KHZ_DEG
           - QUESTION_SCOPE_ERROR_AT_1KHZ_DEG);

    return -measured_scope_error_deg;
}

static float Question_GetFrequencyTrimLimitHz(void)
{
    return (Question_IsHarmonicMode() != 0U)
         ? CTRL_HARMONIC_TRIM_LIMIT_HZ
         : CTRL_FUNDAMENTAL_TRIM_LIMIT_HZ;
}

static float Question_Median3(float a, float b, float c)
{
    if (a > b)
    {
        float temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c)
    {
        float temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b)
    {
        b = a;
    }

    return b;
}

static void Question_ResetController(uint8_t mode)
{
    memset(&input_signal, 0, sizeof(input_signal));

    q1_active_mode = mode;
    q1_output_initialized = 0U;
    q1_phase_tracking = 0U;
    q1_locked = 0U;
    q1_frequency_hold = 0U;
    q1_lock_good_windows = 0U;
    q1_hold_bad_windows = 0U;
    q1_startup_frequency_count = 0U;
    q1_active_amp_code = 0U;
    q1_invalid_phase_frames = 0U;
    q1_force_frequency_check = 0U;
    q1_input_frequency_hz = 0.0f;
    q1_base_frequency_hz = 0.0f;
    memset(
        q1_startup_frequency_samples,
        0,
        sizeof(q1_startup_frequency_samples));
    q1_command_frequency_hz = 0.0f;
    q1_last_programmed_frequency_hz = 0.0f;
    q1_frequency_trim_hz = 0.0f;
    q1_frequency_error_hz = 0.0f;
    q1_direct_frequency_error_sum_hz = 0.0f;
    q1_direct_frequency_error_hz = 0.0f;
    q1_direct_frequency_error_count = 0U;
    q1_phase_error_deg = 0.0f;
    q1_phase_command_deg = 0.0f;
    q1_previous_phase_diff_deg = 0.0f;
    q1_phase_window_change_deg = 0.0f;
    q1_last_phase_window_ms = 0U;
    q1_last_phase_interval_ms = 0U;
    q1_last_capture_samples = 0U;
    q1_last_capture_was_frequency_check = 0U;
    q1_previous_phase_tick = HAL_GetTick();
    q1_phase_window_start_tick = q1_previous_phase_tick;
    q1_last_frequency_check_tick = q1_previous_phase_tick;
}

static void Question_1_StartOutput(float input_frequency_hz,
                                   float output_frequency_ratio,
                                   uint8_t amp_code,
                                   uint32_t now_tick)
{
    q1_input_frequency_hz = input_frequency_hz;
    q1_base_frequency_hz =
        input_frequency_hz * output_frequency_ratio;
    q1_command_frequency_hz = q1_base_frequency_hz;
    q1_last_programmed_frequency_hz = q1_base_frequency_hz;
    q1_frequency_trim_hz = 0.0f;
    q1_frequency_error_hz = 0.0f;
    q1_direct_frequency_error_sum_hz = 0.0f;
    q1_direct_frequency_error_hz = 0.0f;
    q1_direct_frequency_error_count = 0U;
    q1_phase_error_deg = 0.0f;
    q1_phase_command_deg = 0.0f;
    q1_previous_phase_diff_deg = 0.0f;
    q1_phase_window_change_deg = 0.0f;
    q1_last_phase_window_ms = 0U;
    q1_last_phase_interval_ms = 0U;
    q1_previous_phase_tick = now_tick;
    q1_phase_window_start_tick = now_tick;
    q1_phase_tracking = 0U;
    q1_locked = 0U;
    q1_frequency_hold = 0U;
    q1_lock_good_windows = 0U;
    q1_hold_bad_windows = 0U;
    q1_startup_frequency_count = 0U;
    memset(
        q1_startup_frequency_samples,
        0,
        sizeof(q1_startup_frequency_samples));
    q1_invalid_phase_frames = 0U;
    q1_force_frequency_check = 0U;
    q1_last_frequency_check_tick = now_tick;

    AD9833_OUT_Setting(
        (double)q1_base_frequency_hz,
        SIN_WAVE,
        amp_code,
        0.0);

    q1_active_amp_code = amp_code;
    q1_output_initialized = 1U;
}

static float Question_1_CalculatePhaseStep(float phase_error_deg,
                                           uint32_t interval_ms)
{
    float maximum_step_deg =
        (q1_locked == 0U)
      ? CTRL_PHASE_ACQUIRE_MAX_STEP_DEG
      : CTRL_PHASE_FINE_MAX_STEP_DEG;
    float predicted_drift_deg = Question_WrapPhaseDeg(
        0.360f * q1_frequency_error_hz * (float)interval_ms);
    float phase_step_deg =
        CTRL_PHASE_REGISTER_GAIN * phase_error_deg
      - CTRL_PHASE_DRIFT_FEEDFORWARD * predicted_drift_deg;

    if (phase_step_deg > maximum_step_deg)
    {
        phase_step_deg = maximum_step_deg;
    }
    else if (phase_step_deg < -maximum_step_deg)
    {
        phase_step_deg = -maximum_step_deg;
    }

    if (fabsf(phase_step_deg) <= CTRL_PHASE_STEP_DEADBAND_DEG)
    {
        phase_step_deg = 0.0f;
    }

    return phase_step_deg;
}

/*
 * AD9833 PHASE0 is a 12-bit register.  Return the phase movement that
 * the chip can actually make, rather than the unquantized request.
 * This prevents sub-LSB rounding error from leaking into the measured
 * frequency slope when the fine loop runs every few milliseconds.
 */
static float Question_1_ApplyPhaseStep(float requested_step_deg)
{
    float desired_phase_deg = Question_WrapPhaseDeg(
        q1_phase_command_deg + requested_step_deg);
    float normalized_phase_deg = desired_phase_deg;
    float quantized_phase_deg;
    float applied_step_deg;
    uint32_t phase_word;

    if (normalized_phase_deg < 0.0f)
    {
        normalized_phase_deg += 360.0f;
    }

    phase_word = (uint32_t)(
        normalized_phase_deg / AD9833_PHASE_LSB_DEG + 0.5f);
    phase_word &= 0x0FFFU;
    quantized_phase_deg =
        (float)phase_word * AD9833_PHASE_LSB_DEG;
    if (quantized_phase_deg > 180.0f)
    {
        quantized_phase_deg -= 360.0f;
    }

    applied_step_deg = Question_WrapPhaseDeg(
        quantized_phase_deg - q1_phase_command_deg);
    if (applied_step_deg != 0.0f)
    {
        q1_phase_command_deg = quantized_phase_deg;
        AD9833_SetPhase((double)q1_phase_command_deg);
    }

    return applied_step_deg;
}

static void Question_1_UpdatePhaseLoop(uint32_t now_tick,
                                       float target_phase_deg)
{
    uint32_t phase_interval_ms;
    uint32_t phase_window_ms;
    uint32_t required_frequency_window_ms;
    float phase_change_deg;
    float phase_register_step_deg;
    float phase_error_for_lock_deg;
    float phase_slope_frequency_error_hz;
    float frequency_correction_gain;
    float new_frequency_trim_hz;
    float frequency_trim_limit_hz;
    float lock_frequency_error_hz;
    uint8_t required_lock_windows;
    uint8_t use_direct_coarse_error = 0U;

    q1_phase_error_deg = Question_WrapPhaseDeg(
        target_phase_deg - input_signal.phase_diff_calibrated_deg);

    /*
     * Do not differentiate the wrapped absolute phase directly.  Save
     * each small frame-to-frame movement and unwrap that movement before
     * adding it to the frequency observation window.
     */
    if (q1_phase_tracking == 0U)
    {
        phase_register_step_deg =
            Question_1_CalculatePhaseStep(
                q1_phase_error_deg,
                0U);
        phase_register_step_deg =
            Question_1_ApplyPhaseStep(phase_register_step_deg);

        q1_previous_phase_diff_deg = Question_WrapPhaseDeg(
            input_signal.phase_diff_calibrated_deg
            + phase_register_step_deg);
        q1_previous_phase_tick = now_tick;
        q1_phase_window_start_tick = now_tick;
        q1_phase_window_change_deg = 0.0f;
        q1_phase_tracking = 1U;
        return;
    }

    phase_interval_ms = now_tick - q1_previous_phase_tick;
    q1_last_phase_interval_ms = phase_interval_ms;
    if (phase_interval_ms == 0U)
    {
        return;
    }

    /*
     * A long invalid-data gap can contain more than half a turn and is
     * therefore ambiguous.  Restart the observation window instead of
     * feeding a possibly wrong turn direction into the controller.
     */
    if (phase_interval_ms > CTRL_MAX_PHASE_INTERVAL_MS)
    {
        q1_previous_phase_diff_deg =
            input_signal.phase_diff_calibrated_deg;
        q1_previous_phase_tick = now_tick;
        q1_phase_window_start_tick = now_tick;
        q1_phase_window_change_deg = 0.0f;
        q1_locked = 0U;
        q1_lock_good_windows = 0U;
        return;
    }

    phase_change_deg = Question_WrapPhaseDeg(
        input_signal.phase_diff_calibrated_deg
        - q1_previous_phase_diff_deg);
    q1_phase_window_change_deg += phase_change_deg;
    q1_previous_phase_tick = now_tick;

    /*
     * During acquisition PHASE0 may move by up to 12 degrees per frame,
     * so a large initial phase error does not take several seconds to
     * unwind.  After lock the limit automatically returns to 1.5
     * degrees for quiet fine tracking.
     */
    phase_register_step_deg =
        Question_1_CalculatePhaseStep(
            q1_phase_error_deg,
            phase_interval_ms);
    phase_register_step_deg =
        Question_1_ApplyPhaseStep(phase_register_step_deg);

    /*
     * Use the expected phase after our own PHASE0 step as the next
     * reference.  Therefore phase corrections do not pollute df.
     */
    q1_previous_phase_diff_deg = Question_WrapPhaseDeg(
        input_signal.phase_diff_calibrated_deg
        + phase_register_step_deg);

    phase_window_ms = now_tick - q1_phase_window_start_tick;
    required_frequency_window_ms =
        (q1_frequency_hold == 0U)
      ? CTRL_FREQUENCY_ACQUIRE_WINDOW_MS
      : CTRL_FREQUENCY_FINE_WINDOW_MS;
    if (phase_window_ms < required_frequency_window_ms)
    {
        return;
    }

    /*
     * d(phiY-phiX)/dt = 360 * (fY-fX).
     *
     * Only the measured phase slope is allowed to change the frequency.
     * Keeping phase error out of the frequency command prevents a large
     * phase error from creating another fast phase rotation.
     */
    phase_slope_frequency_error_hz =
        q1_phase_window_change_deg
        / (0.360f * (float)phase_window_ms);
    q1_frequency_error_hz = phase_slope_frequency_error_hz;

    /*
     * Wrapped phase samples cannot distinguish frequency errors separated
     * by 1000/interval_ms hertz.  This is about 125 Hz for the present
     * 8 ms loop and can make a visibly wrong output look perfectly locked
     * at each ADC capture.  Average the independently measured X/Y
     * zero-crossing frequencies over the complete window:
     *     1:1 modes: direct error = fy - fx
     *     2:1 modes: direct error = fy - 2*fx
     * Use that unambiguous error for coarse acquisition, then return to
     * the quieter phase slope near lock.
     */
    if (q1_direct_frequency_error_count
            >= CTRL_DIRECT_COARSE_MIN_FRAMES)
    {
        q1_direct_frequency_error_hz =
            q1_direct_frequency_error_sum_hz
            / (float)q1_direct_frequency_error_count;

        if (fabsf(q1_direct_frequency_error_hz)
                > CTRL_DIRECT_COARSE_SWITCH_HZ)
        {
            q1_frequency_error_hz =
                q1_direct_frequency_error_hz;
            use_direct_coarse_error = 1U;
        }
    }
    q1_direct_frequency_error_sum_hz = 0.0f;
    q1_direct_frequency_error_count = 0U;
    q1_last_phase_window_ms = phase_window_ms;

    if (use_direct_coarse_error != 0U)
    {
        /*
         * A coarse error means the old "lock" was a phase-sampling
         * alias.  Leave hold immediately and move most of the measured
         * frequency error in one update.
         */
        q1_locked = 0U;
        q1_frequency_hold = 0U;
        q1_lock_good_windows = 0U;
        q1_hold_bad_windows = 0U;
        new_frequency_trim_hz =
            q1_frequency_trim_hz
          - CTRL_DIRECT_COARSE_GAIN
            * q1_frequency_error_hz;
    }
    else if (q1_frequency_hold == 0U)
    {
        frequency_correction_gain =
            (Question_IsHarmonicMode() != 0U)
          ? CTRL_Q3_FREQUENCY_GAIN
          : CTRL_Q12_FREQUENCY_GAIN;
        new_frequency_trim_hz =
            q1_frequency_trim_hz
          - frequency_correction_gain * q1_frequency_error_hz;
    }
    else
    {
        /*
         * After lock, keep a slow frequency servo instead of freezing
         * the DDS word.  Otherwise a 0.2-to-0.4 Hz residual forces the
         * phase register to rotate forever and the figure never looks
         * truly stationary.
         */
        if (fabsf(q1_frequency_error_hz)
                > CTRL_FINE_FREQUENCY_DEADBAND_HZ)
        {
            new_frequency_trim_hz =
                q1_frequency_trim_hz
              - CTRL_FINE_FREQUENCY_GAIN
                * q1_frequency_error_hz;
        }
        else
        {
            new_frequency_trim_hz = q1_frequency_trim_hz;
        }
    }

    frequency_trim_limit_hz =
        Question_GetFrequencyTrimLimitHz();
    if (new_frequency_trim_hz > frequency_trim_limit_hz)
    {
        new_frequency_trim_hz = frequency_trim_limit_hz;
    }
    else if (new_frequency_trim_hz < -frequency_trim_limit_hz)
    {
        new_frequency_trim_hz = -frequency_trim_limit_hz;
    }

    q1_frequency_trim_hz = new_frequency_trim_hz;
    q1_command_frequency_hz =
        q1_base_frequency_hz + new_frequency_trim_hz;

    /*
     * One AD9833 tuning LSB at 25 MHz is about 0.093 Hz.  Avoid writing
     * the same effective frequency word on every measurement frame.
     */
    if (fabsf(q1_command_frequency_hz
            - q1_last_programmed_frequency_hz) >= 0.04f)
    {
        AD9833_SetFrequency((double)q1_command_frequency_hz);
        q1_last_programmed_frequency_hz =
            q1_command_frequency_hz;
    }

    /*
     * If the frequency trim is already at its rail while a clear
     * residual slope remains, the stored input base frequency is stale.
     * Request a full FFT immediately instead of waiting for the normal
     * one-second scan period.
     */
    if (q1_frequency_hold == 0U
        && fabsf(q1_frequency_trim_hz)
            >= frequency_trim_limit_hz
             - CTRL_TRIM_RECHECK_MARGIN_HZ
        && fabsf(q1_frequency_error_hz)
            >= CTRL_TRIM_RECHECK_ERROR_HZ)
    {
        q1_force_frequency_check = 1U;
    }

    phase_error_for_lock_deg = Question_WrapPhaseDeg(
        q1_phase_error_deg - phase_register_step_deg);

    lock_frequency_error_hz =
        (q1_active_mode == QUESTION_MODE_3
         || q1_active_mode == QUESTION_MODE_4_INFINITY)
      ? CTRL_HARMONIC_LOCK_FREQUENCY_HZ
      : CTRL_LOCK_FREQUENCY_ERROR_HZ;
    required_lock_windows =
        (q1_active_mode == QUESTION_MODE_3
         || q1_active_mode == QUESTION_MODE_4_INFINITY)
      ? CTRL_HARMONIC_LOCK_CONFIRM_WINDOWS
      : CTRL_LOCK_CONFIRM_WINDOWS;

    if (fabsf(q1_frequency_error_hz)
            <= lock_frequency_error_hz
        && fabsf(phase_error_for_lock_deg)
            <= CTRL_LOCK_PHASE_ERROR_DEG)
    {
        if (q1_lock_good_windows < required_lock_windows)
        {
            ++q1_lock_good_windows;
        }
        if (q1_lock_good_windows >= required_lock_windows)
        {
            q1_locked = 1U;
            q1_frequency_hold = 1U;
            q1_hold_bad_windows = 0U;
        }
    }
    else
    {
        q1_lock_good_windows = 0U;

        if (q1_frequency_hold != 0U
            && (fabsf(q1_frequency_error_hz)
                    >= CTRL_HOLD_RELEASE_FREQUENCY_HZ
                || fabsf(phase_error_for_lock_deg)
                    >= CTRL_HOLD_RELEASE_PHASE_DEG))
        {
            if (q1_hold_bad_windows < CTRL_HOLD_RELEASE_WINDOWS)
            {
                ++q1_hold_bad_windows;
            }
            if (q1_hold_bad_windows >= CTRL_HOLD_RELEASE_WINDOWS)
            {
                q1_locked = 0U;
                q1_frequency_hold = 0U;
                q1_hold_bad_windows = 0U;
            }
            else
            {
                q1_locked = 1U;
            }
        }
        else if (q1_frequency_hold != 0U)
        {
            q1_hold_bad_windows = 0U;
            q1_locked = 1U;
        }
        else
        {
            q1_locked = 0U;
        }
    }

    q1_phase_window_start_tick = now_tick;
    q1_phase_window_change_deg = 0.0f;
}

/*
 * Item 1 - same-frequency, zero-degree phase lock.
 *
 * Simultaneous verification paths:
 *     X -> 0.5 gain + 1.65 V bias -> PA1 / ADC2
 *     Y -> 0.5 gain + 1.65 V bias -> PC5 / ADC1
 *
 * Average four valid X frames before starting AD9833.  During this
 * fixed-frequency test AD9833 is initialized only once; small estimator
 * jitter cannot reset the DDS or clear the PI integrator.
 * The full FFT is used only during startup.  Once the frequency is known,
 * a direct X/Y fit provides fast phase tracking.  PHASE0 then moves by at
 * most 1.5 degrees per tracking frame, while a slower phase-slope loop
 * corrects only the genuine frequency drift.  Acquisition allows
 * larger phase steps and a shorter frequency window; after lock it
 * returns to small steps and the selected DDS frequency word is held
 * to prevent adjacent-word chatter.
 */
static void Question_RunMode(uint8_t mode,
                             float output_frequency_ratio,
                             float input_phase_multiplier,
                             float target_phase_deg,
                             uint8_t amp_code)
{
    uint32_t now_tick;
    uint8_t measurement_ok;
    uint8_t frequency_check = 0U;

    if (q1_active_mode != mode)
    {
        Question_ResetController(mode);
    }

    if (q1_fft_initialized == 0U)
    {
        FFT_SetSampleClockCalibration(
            QUESTION_SAMPLE_CLOCK_CALIBRATION);
        FFT_Init();
        q1_fft_initialized = 1U;
    }

    /*
     * Changing a Q4 amplitude wrapper only updates the digital
     * potentiometer.  It does not reset frequency or phase lock.
     */
    if (q1_output_initialized != 0U
        && q1_active_amp_code != amp_code)
    {
        AD9833_AmpSet(amp_code);
        q1_active_amp_code = amp_code;
    }

    now_tick = HAL_GetTick();
    if (q1_output_initialized == 0U
        || q1_force_frequency_check != 0U
        || (now_tick - q1_last_frequency_check_tick)
            >= CTRL_INPUT_RECHECK_PERIOD_MS)
    {
        /*
         * The old controller measured frequency only at startup.  A
         * generator frequency change therefore left the phase fit
         * running forever at the old frequency.  Periodic full-FFT
         * checks make the active mode follow a changed input.
         */
        frequency_check = 1U;
        q1_last_capture_was_frequency_check = 1U;
        q1_last_capture_samples = FFT_SIZE;
        q1_force_frequency_check = 0U;
        measurement_ok = FFT_MeasureInput(&input_signal);
        q1_last_frequency_check_tick = HAL_GetTick();
    }
    else
    {
        q1_last_capture_was_frequency_check = 0U;
        measurement_ok = FFT_MeasurePhaseRelationshipSamples(
            &input_signal,
            q1_input_frequency_hz,
            q1_base_frequency_hz,
            input_phase_multiplier,
            QUESTION_PHASE_CAPTURE_SAMPLES);
        q1_last_capture_samples =
            QUESTION_PHASE_CAPTURE_SAMPLES;
    }

    now_tick = HAL_GetTick();

    if (frequency_check != 0U)
    {
        if (measurement_ok != 0U)
        {
            if (q1_output_initialized == 0U)
            {
                float startup_frequency_hz;

                if (q1_startup_frequency_count
                    < CTRL_STARTUP_FREQUENCY_FRAMES)
                {
                    q1_startup_frequency_samples[
                        q1_startup_frequency_count] =
                            input_signal.frequency_raw_hz;
                    ++q1_startup_frequency_count;
                }

                if (q1_startup_frequency_count
                    >= CTRL_STARTUP_FREQUENCY_FRAMES)
                {
                    /*
                     * Three captures are enough for a robust startup:
                     * the median rejects one high-frequency outlier
                     * without waiting for a tight spread condition.
                     */
                    startup_frequency_hz = Question_Median3(
                        q1_startup_frequency_samples[0],
                        q1_startup_frequency_samples[1],
                        q1_startup_frequency_samples[2]);

                    if (startup_frequency_hz < FFT_INPUT_MIN_HZ)
                    {
                        startup_frequency_hz = FFT_INPUT_MIN_HZ;
                    }
                    else if (startup_frequency_hz > FFT_INPUT_MAX_HZ)
                    {
                        startup_frequency_hz = FFT_INPUT_MAX_HZ;
                    }

                    Question_1_StartOutput(
                        startup_frequency_hz,
                        output_frequency_ratio,
                        amp_code,
                        now_tick);
                }
            }
            else
            {
                q1_invalid_phase_frames = 0U;

                if (Question_InputFrequencyChanged() != 0U
                    || (fabsf(q1_frequency_trim_hz)
                            >= Question_GetFrequencyTrimLimitHz()
                             - CTRL_TRIM_RECHECK_MARGIN_HZ
                        && fabsf(q1_frequency_error_hz)
                            >= CTRL_TRIM_RECHECK_ERROR_HZ))
                {
                    Question_1_StartOutput(
                        input_signal.frequency_raw_hz,
                        output_frequency_ratio,
                        amp_code,
                        now_tick);
                }
            }
        }
        else if (q1_output_initialized != 0U)
        {
            /* Keep scanning until a valid input signal returns. */
            q1_force_frequency_check = 1U;
        }
    }
    else if (measurement_ok != 0U
             && input_signal.phase_valid != 0U)
    {
        q1_invalid_phase_frames = 0U;

        /*
         * The short capture also estimates X frequency from its rising
         * zero crossings.  If it sees a significant input change, skip
         * the stale phase update and request a confirming full FFT now.
         */
        if (mode != QUESTION_MODE_3
            && mode != QUESTION_MODE_4_INFINITY
            && Question_InputFrequencyChanged() != 0U)
        {
            q1_force_frequency_check = 1U;
        }
        else
        {
            float effective_target_phase_deg = target_phase_deg;

            if (input_signal.y_frequency_raw_hz > 0.0f)
            {
                float direct_frequency_error_hz =
                    input_signal.y_frequency_raw_hz
                  - input_phase_multiplier
                    * input_signal.frequency_raw_hz;

                /*
                 * Reject only an obviously corrupted crossing count.
                 * Normal startup/DDS-clock error is far below 2 kHz.
                 */
                if (fabsf(direct_frequency_error_hz) < 2000.0f)
                {
                    q1_direct_frequency_error_sum_hz +=
                        direct_frequency_error_hz;
                    if (q1_direct_frequency_error_count < 65535U)
                    {
                        ++q1_direct_frequency_error_count;
                    }
                }
            }

            if (Question_IsHarmonicMode() == 0U)
            {
                effective_target_phase_deg +=
                    Question_GetFundamentalPhaseCompensationDeg(
                        q1_input_frequency_hz);
            }

            Question_1_UpdatePhaseLoop(
                now_tick,
                effective_target_phase_deg);
        }
    }
    else
    {
        if (q1_invalid_phase_frames < CTRL_INVALID_PHASE_FRAMES)
        {
            ++q1_invalid_phase_frames;
        }
        if (q1_invalid_phase_frames >= CTRL_INVALID_PHASE_FRAMES)
        {
            q1_invalid_phase_frames = 0U;
            q1_force_frequency_check = 1U;
        }
    }

    Question_SendDebug();
}

void Question_1(void)
{
    Question_RunMode(
        QUESTION_MODE_1,
        1.0f,
        1.0f,
        QUESTION_1_PHASE_TRIM_DEG,
        QUESTION_Q123_AMP_CODE);
}

void Question_2(void)
{
    /*
     * Same frequency and amplitude as X, with Y leading X by 90 degrees.
     * The sign of 90 degrees only changes the traversal direction of the
     * circle, not its shape.
     */
    Question_RunMode(
        QUESTION_MODE_2,
        1.0f,
        1.0f,
        90.0f + QUESTION_2_PHASE_TRIM_DEG,
        QUESTION_Q123_AMP_CODE);
}

void Question_3(void)
{
    /*
     * fY = 2*fX.  The time-origin-independent symmetry phase is
     * phase(Y) - 2*phase(X), which is driven to zero for a horizontal
     * and vertically/horizontally symmetric figure-eight.
     */
    Question_RunMode(
        QUESTION_MODE_3,
        2.0f,
        2.0f,
        QUESTION_3_PHASE_TRIM_DEG,
        QUESTION_Q123_AMP_CODE);
}

static uint8_t Question_4_GetAmpCode(uint8_t vpp)
{
    switch (vpp)
    {
        case 1U:
            return QUESTION_4_1VPP_AMP_CODE;
        case 2U:
            return QUESTION_4_2VPP_AMP_CODE;
        case 3U:
            return QUESTION_4_3VPP_AMP_CODE;
        case 4U:
        default:
            return QUESTION_4_4VPP_AMP_CODE;
    }
}

void Question_4(Question4Shape shape, uint8_t vpp)
{
    uint8_t amp_code = Question_4_GetAmpCode(vpp);

    switch (shape)
    {
        case QUESTION_4_SHAPE_CIRCLE:
            /*
             * Same frequency, Y leads X by 90 degrees.  It is a true
             * circle only at 4 Vpp; lower Y amplitudes form ellipses,
             * while their Y extents remain 2/4/6 div as required.
             */
            Question_RunMode(
                QUESTION_MODE_4_CIRCLE,
                1.0f,
                1.0f,
                90.0f + QUESTION_4_PHASE_TRIM_DEG,
                amp_code);
            break;

        case QUESTION_4_SHAPE_INFINITY:
            /* fY=2*fX and phiY-2*phiX=0. */
            Question_RunMode(
                QUESTION_MODE_4_INFINITY,
                2.0f,
                2.0f,
                QUESTION_4_PHASE_TRIM_DEG,
                amp_code);
            break;

        case QUESTION_4_SHAPE_DIAGONAL:
        default:
            /* Same frequency and phase. */
            Question_RunMode(
                QUESTION_MODE_4_DIAGONAL,
                1.0f,
                1.0f,
                QUESTION_4_PHASE_TRIM_DEG,
                amp_code);
            break;
    }
}
