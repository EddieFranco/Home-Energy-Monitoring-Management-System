#include "pid.h"

// Helper: keep a value inside [a, b]
static inline float clampf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

void PID_Init(PID_t *pid,float kp, float ki, float kd,float out_min, float out_max,float dt_s, float d_alpha)
{
  // Store gains and basic limits
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->out_min = out_min;
  pid->out_max = out_max;

  // Sample time (how often PID_Compute will be called)
  if (dt_s > 0.0f) {
    pid->dt_s = dt_s;       // use the given sample time
  } else {
    pid->dt_s = 0.1f;       // fallback default: 0.1 s
  }


  // Reset memories
  pid->integrator = 0.0f;  // I-term memory starts at 0
  pid->prev_meas  = 0.0f;  // no last measurement yet
  pid->first_run  = 1;     // first call gets special handling

  // Derivative smoothing factor [0..1]
  if (d_alpha < 0.0f)
	  d_alpha = 0.0f;
  if (d_alpha > 1.0f)
	  d_alpha = 1.0f;

  pid->d_alpha = d_alpha;
  pid->d_filt  = 0.0f;
}

void PID_Reset(PID_t *pid)
{
  pid->integrator = 0.0f;
  pid->prev_meas  = 0.0f;
  pid->first_run  = 1;
  pid->d_filt     = 0.0f;
}


//  error = setpoint - measurement
//  -> positive error means "we are too cold, add heat"
float PID_Compute(PID_t *pid, float setpoint, float measurement)
{
  const float dt = pid->dt_s;

  // 1) ERROR: how far from target (use set - meas so positive = need more heat)
  float error = setpoint - measurement;

  // 2) P-TERM: immediate push proportional to the error "right now"
  float P = pid->kp * error;

  // 3) D-TERM on MEASUREMENT (to avoid setpoint "kick"):
  //    Approximate rate of change of the temperature.
  float d_raw;
  if (pid->first_run) {
    d_raw = 0.0f;             // no previous measurement on very first call
    pid->first_run = 0;
  } else {
    d_raw = (measurement - pid->prev_meas) / dt;
  }

  pid->prev_meas = measurement;

  // Smooth D (helps with sensor noise)
  pid->d_filt = (1.0f - pid->d_alpha) * pid->d_filt + pid->d_alpha * d_raw;

  // NOTE the minus sign:
  //  We took derivative of MEASUREMENT; derivative of ERROR would be (-d_meas).
  //  So D = -kd * d(meas)/dt.
  float D = -pid->kd * pid->d_filt;

  // 4) I-TERM (provisional): accumulate error over time (area)
  float I_ = pid->integrator + pid->ki * error * dt;

  // 5) Sum P + I + D (unclamped)
  float u_unclamped = P + I_ + D;

  // 6) Clamp to safe range (e.g., 0..100 %)
  float u = clampf(u_unclamped, pid->out_min, pid->out_max);

  // 7) ANTI-WINDUP: Only accept the new I if we are NOT pushing deeper into a limit.
  //    Example: already at 100%, error still positive → don't integrate more upward.
  int at_upper = (u >= pid->out_max - 1e-6f);
  int at_lower = (u <= pid->out_min + 1e-6f);

  if ((at_upper && error > 0.0f) || (at_lower && error < 0.0f)) {
    // hold integrator (do not update)
  } else {
    pid->integrator = I_;
  }

  // Final control output
  return u;
}
