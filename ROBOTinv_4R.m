% --- Inverse Kinematics ---

function Q = ROBOTinv_4R(S, L, sol)

% Links lengths
l0=L(1);
l1=L(2);
l2=L(3);
l3=L(4);

% Pose of the EE
p_x = S(1);
p_y = S(2);
p_z = S(3);
teta = S(4);

Q = zeros(4,1);

%  Joint q1 coordinate
Q(1) = atan2(p_y, p_x);


R_target = sqrt(p_x^2 + p_y^2);      % horizontal distance of the EE from the base axis
R_wrist = R_target - l3 * cos(teta); % horizontal distance of the wrist from the base axis

pw_z = p_z - l0 - l3*sin(teta); % vertical position of the wrist w.r.t. joint-2 frame

% elbow joint q3 from the law of cosines
c3 = (R_wrist^2 + pw_z^2 - l1^2 - l2^2)/(2*l1*l2);
s3_sgn = sqrt(1-c3^2);


if sol > 0
    s3 = s3_sgn;    % elbow-up solution
else
    s3 = -s3_sgn;   % elbow-down solution
end

q2_z = atan2(pw_z, R_wrist);         % angle between horizontal axis and wrist vector

q2_R = atan2(l2 * s3, l1 + l2 * c3); % internal angle of triangle

% Joint q2 coordinate
Q(2) = q2_z-q2_R;

% Joint q3 coordinate
Q(3) = atan2(s3,c3);

% Joint q4 coordinate
Q(4) = teta-Q(2)-Q(3);

end