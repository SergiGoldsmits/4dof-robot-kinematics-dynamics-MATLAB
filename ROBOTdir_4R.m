% --- Direct Kinematics ---

function S = ROBOTdir_4R(Q,L)

% Links lengths
l0=L(1);
l1=L(2);
l2=L(3);
l3=L(4);

S= zeros(4,1);

% Abbreviations
c1 = cos(Q(1)); s1 = sin(Q(1));
c2 = cos(Q(2)); s2 = sin(Q(2));
c3 = cos(Q(3)); s3 = sin(Q(3));
c4 = cos(Q(4)); s4 = sin(Q(4));
Q23  = Q(2) + Q(3);
Q234 = Q(2) + Q(3) + Q(4);
c23  = cos(Q23);   s23  = sin(Q23);
c234 = cos(Q234);  s234 = sin(Q234);

% Pose of the EE

R = l1 * c2 + l2 * c23 + l3 * c234; %radial distance

S(1) = c1 * R;                              % X-coordinate
S(2) = s1 * R;                              % Y-coordinate
S(3) = l0 + l1 * s2 + l2 * s23 + l3 * s234; % Z-coordinate
S(4) = Q234;                                % Theta (EE orientation)

end 