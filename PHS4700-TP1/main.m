clear all; clc;


%cas 1
disp ('cas 1');
posA = [22.95+3.82;0;1.345+0.25];
ar = 0;
va = [0; 0; 0];
Forces = [11000;11000;260000];

[pcm1, MI1, aa1] = Devoir1(posA,ar,va,Forces);

%cas 2
disp ('cas 2');
posA = [4198;0;618];

ar = 0.15;
va = [0.0; -0.003; -0.01];
Forces = [8000;11000;260000];

[pcm2, MI2, aa2] = Devoir1(posA,ar,va,Forces);
           