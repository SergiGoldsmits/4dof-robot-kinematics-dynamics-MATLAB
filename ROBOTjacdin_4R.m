function J_din = ROBOTjacdin_4R(Q,L)

% Lenghts
l0 = L(1); l1=L(2); l2=L(3); l3=L(4);
g1 = L(5); g2 = L(6); g3 = L(7);
q1 = Q(1); q2 = Q(2); q3 = Q(3); q4 = Q(4);

% Abbreviations
c1 = cos(q1); s1 = sin(q1);
c2 = cos(q2); s2 = sin(q2);
Q23  = q2 + q3;
Q234 = q2 + q3 + q4;
c23  = cos(Q23);   s23  = sin(Q23);
c234 = cos(Q234);  s234 = sin(Q234);
R = l1 * c2 + l2 * c23 + l3 * c234;
R_prime = R - (g3 * c234);
R_double_prime = l1 * c2 + g2 * c23;
R_triple_prime = g1 * c2;

dR_dQ2 = -l1 * s2 - l2 * s23 - l3 * s234;
dR_dQ3 = -l2 * s23 - l3 * s234;
dR_dQ4 = -l3 * s234;

dR_prime_dQ2 = -l1 * s2 - l2 * s23 - g3 * s234;   % <-- CoM3, NON l3

dR_prime_dQ3 = dR_dQ3 + g3 * s234; 
dR_prime_dQ4 = dR_dQ4 + g3 * s234;

dR_double_prime_dQ2 = -l1 * s2 - g2 * s23;
dR_double_prime_dQ3 = -g2 * s23;

dR_triple_prime_dQ2 = -g1 * s2;

% --- Jacobian (19x4) ---
J_din = zeros(19, 4);

J_din(1, 1) = -s1 * R;
J_din(1, 2) = c1 * dR_dQ2;
J_din(1, 3) = c1 * dR_dQ3;
J_din(1, 4) = c1 * dR_dQ4;

J_din(2, 1) = 0;
J_din(2, 2) = l1 * c2 + l2 * c23 + l3 * c234; % = R
J_din(2, 3) = l2 * c23 + l3 * c234;
J_din(2, 4) = l3 * c234;

J_din(3, 1) = c1 * R;
J_din(3, 2) = s1 * dR_dQ2;
J_din(3, 3) = s1 * dR_dQ3;
J_din(3, 4) = s1 * dR_dQ4;

J_din(4, 1) = -s1 * R_prime;
% J_din(4, 2) = c1 * dR_dQ2; 
J_din(4, 2) = c1 * dR_prime_dQ2;%%
J_din(4, 3) = c1 * dR_prime_dQ3;
J_din(4, 4) = c1 * dR_prime_dQ4;

J_din(5, 1) = 0;
J_din(5, 2) = l1 * c2 + l2 * c23 + g3 * c234;
J_din(5, 3) = l2 * c23 + g3 * c234;
J_din(5, 4) = g3 * c234;

J_din(6, 1) = c1 * R_prime;
% J_din(6, 2) = s1 * dR_dQ2; 
J_din(6, 2) = s1 * dR_prime_dQ2;%%
J_din(6, 3) = s1 * dR_prime_dQ3;
J_din(6, 4) = s1 * dR_prime_dQ4;

J_din(7, :) = [0, 1, 1, 1];

J_din(8, 1) = -s1 * R_double_prime;
J_din(8, 2) = c1 * dR_double_prime_dQ2;
J_din(8, 3) = c1 * dR_double_prime_dQ3;
J_din(8, 4) = 0;

J_din(9, 1) = 0;
J_din(9, 2) = l1 * c2 + g2 * c23;
J_din(9, 3) = g2 * c23;
J_din(9, 4) = 0;

J_din(10, 1) = c1 * R_double_prime;
J_din(10, 2) = s1 * dR_double_prime_dQ2;
J_din(10, 3) = s1 * dR_double_prime_dQ3;
J_din(10, 4) = 0;

J_din(11, :) = [0, 1, 1, 0];

J_din(12, 1) = -s1 * R_triple_prime;
J_din(12, 2) = c1 * dR_triple_prime_dQ2;
J_din(12, 3) = 0;
J_din(12, 4) = 0;

J_din(13, 1) = 0;
J_din(13, 2) = g1 * c2;
J_din(13, 3) = 0;
J_din(13, 4) = 0;

J_din(14, 1) = c1 * R_triple_prime;
J_din(14, 2) = s1 * dR_triple_prime_dQ2;
J_din(14, 3) = 0;
J_din(14, 4) = 0;

J_din(15, :) = [0, 1, 0, 0];

J_din(16, :) = [0, 0, 0, 0];

J_din(17, :) = [0, 0, 0, 0];

J_din(18, :) = [0, 0, 0, 0];

J_din(19, :) = [1, 0, 0, 0];

end 