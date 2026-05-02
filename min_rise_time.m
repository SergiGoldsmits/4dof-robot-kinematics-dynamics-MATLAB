function [T_rise] = min_rise_time(motor,dq)

[T_lim,dq_lim] = limit_case(motor)

if dq < dq_lim %case1
    
   T_rise = ((sqrt(motor.A/motor.D)+sqrt(motor.D/motor.A)))*...
       sqrt(2*dq*1/(motor.A+motor.D));
else
    T_rise = dq/motor.V+motor.V*(motor.A+motor.D)/(2*motor.A*motor.D); %case2
end

end