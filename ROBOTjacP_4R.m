function Jdot = ROBOTjacP_4R(Q,Q_dot,L)

l0 = L(1); l1=L(2); l2=L(3); l3=L(4);
q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);
q1_dot = Q_dot(1); q2_dot = Q_dot(2); q3_dot = Q_dot(3); q4_dot = Q_dot(4);

% Trigonometric Abbreviations
c1 = cos(q1); s1 = sin(q1);
Q23 = q2 + q3; Q234 = q2 + q3 + q4;
c2 = cos(q2); s2 = sin(q2);
c23 = cos(Q23); s23 = sin(Q23);
c234 = cos(Q234); s234 = sin(Q234);

% Partial computations
R = l1 * c2 + l2 * c23 + l3 * c234; 

% Derivate Prime (Coefficienti in J)
dR_dq2 = -l1 * s2 - l2 * s23 - l3 * s234;
dR_dq3 = -l2 * s23 - l3 * s234;
dR_dq4 = -l3 * s234;

% Second order partial derivatives

d2R_dq2_dqi = [-l1*c2 - l2*c23 - l3*c234, -l2*c23 - l3*c234, -l3*c234];

d2R_dq3_dqi = [-l2*c23 - l3*c234, -l2*c23 - l3*c234, -l3*c234];

d2R_dq4_dqi = [-l3*c234, -l3*c234, -l3*c234];

d2pz_dq2_dqi = [dR_dq2, dR_dq3, dR_dq4];

d2pz_dq3_dqi = [-l2*s23 - l3*s234, -l2*s23 - l3*s234, -l3*s234]; 
d2pz_dq4_dqi = [-l3*s234, -l3*s234, -l3*s234];

% JacobianP matrix construction

Jdot= zeros(4,4);

J_dot(1, 1) = (-c1*R)*q1_dot + (-s1*dR_dq2)*q2_dot + (-s1*dR_dq3)*q3_dot + (-s1*dR_dq4)*q4_dot;
J_dot(2, 1) = (-s1*R)*q1_dot + (c1*dR_dq2)*q2_dot + (c1*dR_dq3)*q3_dot + (c1*dR_dq4)*q4_dot;
J_dot(3, 1) = 0;
J_dot(4, 1) = 0;

J_dot(1, 2) = (-s1*dR_dq2)*q1_dot + c1*(d2R_dq2_dqi(1)*q2_dot + d2R_dq2_dqi(2)*q3_dot + d2R_dq2_dqi(3)*q4_dot);
J_dot(2, 2) = (c1*dR_dq2)*q1_dot + s1*(d2R_dq2_dqi(1)*q2_dot + d2R_dq2_dqi(2)*q3_dot + d2R_dq2_dqi(3)*q4_dot);
J_dot(3, 2) = d2pz_dq2_dqi(1)*q2_dot + d2pz_dq2_dqi(2)*q3_dot + d2pz_dq2_dqi(3)*q4_dot;
J_dot(4, 2) = 0;

J_dot(1, 3) = (-s1*dR_dq3)*q1_dot + c1*(d2R_dq3_dqi(1)*q2_dot + d2R_dq3_dqi(2)*q3_dot + d2R_dq3_dqi(3)*q4_dot);
J_dot(2, 3) = (c1*dR_dq3)*q1_dot + s1*(d2R_dq3_dqi(1)*q2_dot + d2R_dq3_dqi(2)*q3_dot + d2R_dq3_dqi(3)*q4_dot);
J_dot(3, 3) = d2pz_dq3_dqi(1)*q2_dot + d2pz_dq3_dqi(2)*q3_dot + d2pz_dq3_dqi(3)*q4_dot;
J_dot(4, 3) = 0;

J_dot(1, 4) = (-s1*dR_dq4)*q1_dot + c1*(d2R_dq4_dqi(1)*q2_dot + d2R_dq4_dqi(2)*q3_dot + d2R_dq4_dqi(3)*q4_dot);
J_dot(2, 4) = (c1*dR_dq4)*q1_dot + s1*(d2R_dq4_dqi(1)*q2_dot + d2R_dq4_dqi(2)*q3_dot + d2R_dq4_dqi(3)*q4_dot);
J_dot(3, 4) = d2pz_dq4_dqi(1)*q2_dot + d2pz_dq4_dqi(2)*q3_dot + d2pz_dq4_dqi(3)*q4_dot;
J_dot(4, 4) = 0;

end

