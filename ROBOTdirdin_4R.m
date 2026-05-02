function S_din = ROBOTdirdin_4R(Q,L)

% direct kinematics
l0 = L(1); l1=L(2); l2=L(3); l3=L(4);
g1 = L(5); g2 = L(6); g3 = L(7);
g0=l0/2;

S= zeros(19,1);

%abbreviations
c1 = cos(Q(1)); s1 = sin(Q(1));
c2 = cos(Q(2)); s2 = sin(Q(2));
c3 = cos(Q(3)); s3 = sin(Q(3));
c4 = cos(Q(4)); s4 = sin(Q(4));
Q23  = Q(2) + Q(3);
Q234 = Q(2) + Q(3) + Q(4);
c23  = cos(Q23);   s23  = sin(Q23);
c234 = cos(Q234);  s234 = sin(Q234);

%pose of the EE
R = l1 * c2 + l2 * c23 + l3 * c234;

S(1) = c1 * R; 
S(2) = l0 + l1 * s2 + l2 * s23 + l3 * s234; %z
S(3) = s1 * R; 

S(4) = c1 * (R - (g3 * c234));
S(5) = l0 + l1 * s2 + l2 * s23 + g3 * s234;
S(6) = s1 * (R - (g3 * c234));
S(7) = Q234;

S(8) = c1 * (l1 *c2 + g2 *c23);
S(9) = l0 + l1 * s2 + g2 * s23;
S(10) = s1 * (l1 *c2 + g2 *c23);
S(11) = Q23;

S(12) = c1 * (g1 *c2);
S(13) = l0 + g1 * s2;
S(14) = s1 * (g1 *c2);
S(15) = Q2;

S(16) = 0;
S(17) = g0;
S(18) = 0;
S(19) = Q(1);


end