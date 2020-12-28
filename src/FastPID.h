#ifndef FastPID_H
#define FastPID_H

#include <stdint.h>

#define INTEG_MAX    (INT32_MAX)
#define INTEG_MIN    (INT32_MIN)
#define DERIV_MAX    (INT16_MAX)
#define DERIV_MIN    (INT16_MIN)

#define PARAM_SHIFT  8
#define PARAM_BITS   16
#define PARAM_MAX    (((0x1ULL << PARAM_BITS)-1) >> PARAM_SHIFT) 
#define PARAM_MULT   (((0x1ULL << PARAM_BITS)) >> (PARAM_BITS - PARAM_SHIFT)) 

/*
  A fixed point PID controller with a 32-bit internal calculation pipeline.
*/
class FastPID {

public:
  FastPID() 
  {
    clear();
  }

  FastPID(const float kp, const float ki, const float kd, const float hz, const int bits=16, const bool sign=false)
  {
    configure(kp, ki, kd, hz, bits, sign);
  }

  ~FastPID();

  bool setCoefficients(const float kp, const float ki, const float kd, const float hz);
  bool setOutputConfig(const int bits, const bool sign);
  bool setOutputRange(const int16_t min, const int16_t max);
  void clear();
  bool configure(const float kp, const float ki, const float kd, const float hz, const int bits=16, const bool sign=false);
  int16_t step(const int16_t sp, const int16_t fb);

  bool err() {
    return _cfg_err;
  }

private:

  uint32_t floatToParam(const float); 
  void setCfgErr(); 

private:

  // Configuration
  uint32_t _p, _i, _d;
  int64_t _outmax, _outmin; 
  bool _cfg_err; 
  
  // State
  int16_t _last_sp, _last_out;
  int64_t _sum;
  int32_t _last_err;
};

#endif
