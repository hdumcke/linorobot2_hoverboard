
struct pid_ff_controller
{
  pid_ff_controller(
    float * kp,
    float * kd,
    float * kff,
    float min_bound,
    float max_bound,
    float d_alpha = 1.0f,
    float out_alpha = 1.0f
  ) :
  _kp(kp),
  _kd(kd),
  _kff(kff),
  _min_bound(min_bound),
  _max_bound(max_bound),
  _d_alpha(d_alpha),
  _out_alpha(out_alpha)
  {

  }

  void reset()
  {
    _last_error = 0.0f;
    _last_derivative = 0.0f;
    _last_ouput = 0.0f;
  }

  float process(float error, float ff_input = 0.0f)
  {
    float const derivative = (error-_last_error)*_d_alpha + (1.0f-_d_alpha)*_last_derivative;
    _last_derivative = derivative;
    _last_error = error;
    float output = 0.0f;
    if(_kp!=nullptr) output += (*_kp)*error;
    if(_kd!=nullptr) output += (*_kd)*derivative;
    if(_kff!=nullptr) output += (*_kff)*ff_input;
    output =_out_alpha*output + (1.0f-_out_alpha)*_last_ouput;
    _last_ouput = output;
    return std::clamp(output,_min_bound,_max_bound);
  }

  //private:
    float * _kp;
    float * _kd;
    float * _kff;
    float _min_bound;
    float _max_bound;
    float _d_alpha;
    float _out_alpha;

    float _last_error = 0.0f;
    float _last_derivative = 0.0f;
    float _last_ouput = 0.0f;
};

