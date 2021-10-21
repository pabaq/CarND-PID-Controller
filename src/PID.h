#ifndef PID_H
#define PID_H

class PID {
 public:
  PID(double kp, double ki, double kd);
  virtual ~PID();
  void update_error(double cte);
  double total_error();

 private:
  double p_error, i_error, d_error;
  double kp, ki, kd;
};

#endif  // PID_H