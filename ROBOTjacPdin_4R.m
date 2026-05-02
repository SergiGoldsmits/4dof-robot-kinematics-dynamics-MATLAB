function J_dot_din = ROBOTjacPdin_4R(Q, Q_dot, L)

% Lenghts
l0 = L(1); l1 = L(2); l2 = L(3); l3 = L(4);
g1 = L(5); g2 = L(6); g3 = L(7);
q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);
q_dot1 = Q_dot(1); q_dot2 = Q_dot(2); q_dot3 = Q_dot(3); q_dot4 = Q_dot(4);

% Abbreviations
c1 = cos(q1); s1 = sin(q1); 
c2 = cos(q2); s2 = sin(q2);
Q23 = q2 + q3; Q234 = q2 + q3 + q4;
c23 = cos(Q23); s23 = sin(Q23);
c234 = cos(Q234); s234 = sin(Q234);

c1_dot = -s1 * q_dot1; s1_dot = c1 * q_dot1;
c2_dot = -s2 * q_dot2; s2_dot = c2 * q_dot2;
Q23_dot = q_dot2 + q_dot3;
c23_dot = -s23 * Q23_dot; s23_dot = c23 * Q23_dot;
Q234_dot = Q23_dot + q_dot4;
c234_dot = -s234 * Q234_dot; s234_dot = c234 * Q234_dot;
R = l1 * c2 + l2 * c23 + l3 * c234;
R_prime = R - (g3 * c234);
R_double_prime = l1 * c2 + g2 * c23;
R_triple_prime = g1 * c2;
R_dot = l1 * c2_dot + l2 * c23_dot + l3 * c234_dot;
R_prime_dot = R_dot - g3 * c234_dot;
R_double_prime_dot = l1 * c2_dot + g2 * c23_dot;
R_triple_prime_dot = g1 * c2_dot;
dR_dQ2 = -l1 * s2 - l2 * s23 - l3 * s234;
dR_dQ3 = -l2 * s23 - l3 * s234;
dR_dQ4 = -l3 * s234;
dR_prime_dQ2 = -l1*s2 - l2*s23 - g3*s234;%%
dR_prime_dQ3 = dR_dQ3 + g3 * s234;
dR_prime_dQ4 = dR_dQ4 + g3 * s234;
dR_double_prime_dQ2 = -l1 * s2 - g2 * s23;
dR_double_prime_dQ3 = -g2 * s23;
dR_triple_prime_dQ2 = -g1 * s2;

dR_dQ2_dot = -l1 * s2_dot - l2 * s23_dot - l3 * s234_dot;
dR_dQ3_dot = -l2 * s23_dot - l3 * s234_dot;
dR_dQ4_dot = -l3 * s234_dot;

dR_prime_dQ2_dot = -l1*s2_dot - l2*s23_dot - g3*s234_dot; %%
dR_prime_dQ3_dot = dR_dQ3_dot + g3 * s234_dot;
dR_prime_dQ4_dot = dR_dQ4_dot + g3 * s234_dot;

dR_double_prime_dQ2_dot = -l1 * s2_dot - g2 * s23_dot;
dR_double_prime_dQ3_dot = -g2 * s23_dot;

dR_triple_prime_dQ2_dot = -g1 * s2_dot;

% Jacobian derivative 
J_dot_din = zeros(19, 4);

J_dot_din(1, 1) = -s1_dot * R - s1 * R_dot;
J_dot_din(1, 2) = c1_dot * dR_dQ2 + c1 * dR_dQ2_dot;
J_dot_din(1, 3) = c1_dot * dR_dQ3 + c1 * dR_dQ3_dot;
J_dot_din(1, 4) = c1_dot * dR_dQ4 + c1 * dR_dQ4_dot;

J_dot_din(2, 1) = 0;
J_dot_din(2, 2) = R_dot;
J_dot_din(2, 3) = l2 * c23_dot + l3 * c234_dot;
J_dot_din(2, 4) = l3 * c234_dot;

J_dot_din(3, 1) = c1_dot * R + c1 * R_dot;
J_dot_din(3, 2) = s1_dot * dR_dQ2 + s1 * dR_dQ2_dot;
J_dot_din(3, 3) = s1_dot * dR_dQ3 + s1 * dR_dQ3_dot;
J_dot_din(3, 4) = s1_dot * dR_dQ4 + s1 * dR_dQ4_dot;

J_dot_din(4, 1) = -s1_dot * R_prime - s1 * R_prime_dot;
% J_dot_din(4, 2) = c1_dot * dR_dQ2 + c1 * dR_dQ2_dot;
J_dot_din(4, 2) = c1_dot * dR_prime_dQ2 + c1 * dR_prime_dQ2_dot;%%
J_dot_din(4, 3) = c1_dot * dR_prime_dQ3 + c1 * dR_prime_dQ3_dot;
J_dot_din(4, 4) = c1_dot * dR_prime_dQ4 + c1 * dR_prime_dQ4_dot;

J_dot_din(5, 1) = 0;
J_dot_din(5, 2) = l1 * c2_dot + l2 * c23_dot + g3 * c234_dot;
J_dot_din(5, 3) = l2 * c23_dot + g3 * c234_dot;
J_dot_din(5, 4) = g3 * c234_dot;

J_dot_din(6, 1) = c1_dot * R_prime + c1 * R_prime_dot;
% J_dot_din(6, 2) = s1_dot * dR_dQ2 + s1 * dR_dQ2_dot;
J_dot_din(6, 2) = s1_dot * dR_prime_dQ2 + s1 * dR_prime_dQ2_dot;
J_dot_din(6, 3) = s1_dot * dR_prime_dQ3 + s1 * dR_prime_dQ3_dot;
J_dot_din(6, 4) = s1_dot * dR_prime_dQ4 + s1 * dR_prime_dQ4_dot;

J_dot_din(7, :) = [0, 0, 0, 0];

J_dot_din(8, 1) = -s1_dot * R_double_prime - s1 * R_double_prime_dot;
J_dot_din(8, 2) = c1_dot * dR_double_prime_dQ2 + c1 * dR_double_prime_dQ2_dot;
J_dot_din(8, 3) = c1_dot * dR_double_prime_dQ3 + c1 * dR_double_prime_dQ3_dot;
J_dot_din(8, 4) = 0;

J_dot_din(9, 1) = 0;
J_dot_din(9, 2) = l1 * c2_dot + g2 * c23_dot;
J_dot_din(9, 3) = g2 * c23_dot;
J_dot_din(9, 4) = 0;

J_dot_din(10, 1) = c1_dot * R_double_prime + c1 * R_double_prime_dot;
J_dot_din(10, 2) = s1_dot * dR_double_prime_dQ2 + s1 * dR_double_prime_dQ2_dot;
J_dot_din(10, 3) = s1_dot * dR_double_prime_dQ3 + s1 * dR_double_prime_dQ3_dot;
J_dot_din(10, 4) = 0;

J_dot_din(11, :) = [0, 0, 0, 0];

J_dot_din(12, 1) = -s1_dot * R_triple_prime - s1 * R_triple_prime_dot;
J_dot_din(12, 2) = c1_dot * dR_triple_prime_dQ2 + c1 * dR_triple_prime_dQ2_dot;
J_dot_din(12, 3) = 0;
J_dot_din(12, 4) = 0;

J_dot_din(13, 1) = 0;
J_dot_din(13, 2) = g1 * c2_dot;
J_dot_din(13, 3) = 0;
J_dot_din(13, 4) = 0;

J_dot_din(14, 1) = c1_dot * R_triple_prime + c1 * R_triple_prime_dot;
J_dot_din(14, 2) = s1_dot * dR_triple_prime_dQ2 + s1 * dR_triple_prime_dQ2_dot;
J_dot_din(14, 3) = 0;
J_dot_din(14, 4) = 0;

J_dot_din(15, :) = [0, 0, 0, 0];

J_dot_din(16, :) = [0, 0, 0, 0];
J_dot_din(17, :) = [0, 0, 0, 0];
J_dot_din(18, :) = [0, 0, 0, 0];

J_dot_din(19, :) = [0, 0, 0, 0]; 
end