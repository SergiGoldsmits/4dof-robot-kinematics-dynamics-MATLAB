function  [T_lim,dq_lim] = limit_case(motor)

T_lim = motor.V * (1/motor.A+1/motor.D);
dq_lim = 0.5 * motor.V^2 * (1/motor.A+1/motor.D);

end