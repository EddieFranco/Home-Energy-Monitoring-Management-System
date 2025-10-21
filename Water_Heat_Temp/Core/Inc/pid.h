#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

// A PID controller remembers 3 things: P (now), I (past), D (trend).
// We keep gains and "memory" in this struct.
typedef struct {
  // --- Gains (tuning knobs) ---
  float kp;  // Proportional gain: reacts to "how far" we are from target now
  float ki;  // Integral gain (1/second): reacts to "how long/how much" we've been off
  float kd;  // Derivative gain (seconds): reacts to "how fast" temperature is changing

  // --- Output limits (safety) ---
  float out_min;  // e.g., 0% PWM
  float out_max;  // e.g., 100% PWM

  // --- Timing & memory ---
  float dt_s;       // how often PID runs, in seconds (e.g., 0.2 s)
  float integrator; // "I-term memory": running sum of error
  float prev_meas;  // last measurement (for derivative)
  unsigned char first_run; // true only on the very first compute()

  // --- Derivative smoothing (reduces noise) ---
  float d_alpha; // 0..1, 0 = heavy smoothing, 1 = no smoothing
  float d_filt;  // filtered derivative state
} PID_t;

// Fill in the struct once at startup (or when you change sample time/limits).
void PID_Init(PID_t *pid,float kp, float ki, float kd,float out_min, float out_max,float dt_s, float d_alpha);

// Clear I/D memory (useful after faults or big setpoint changes)
void PID_Reset(PID_t *pid);

// Change gains at runtime (e.g., from UI)
static inline void PID_SetTunings(PID_t *pid, float kp, float ki, float kd) {
  pid->kp = kp; pid->ki = ki; pid->kd = kd;
}

// The main step: give it target (setpoint) and actual value (measurement).
// It returns the control output (e.g., PWM %). Internally limited to [out_min, out_max].
float PID_Compute(PID_t *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif // PID_H
