function [MI] = CalculateMI(plane,pcm)
%CalculateMI is the function that will calculate the moment of inertia 

cabin = plane('cabin');
body = plane('body');
wingL = plane('wingL');
wingR = plane('wingR');
fin = plane('fin');
motorL = plane('motorL');
motorR = plane('motorR');

%Calculate MI of cabin
valueCalculated1 = ((12 * (cabin.r) ^ 2 + 3 * (cabin.h) ^ 2)) / 80;
valueCalculated2 = (3 * (cabin.r) ^ 2) / 10;
MI_cabin = [valueCalculated1 0 0; 0 valueCalculated1 0; 0 0 valueCalculated2];

d = pcm - cabin.vpos;
matrix = [(d(2)^2 + d(3)^2) (-1 * d(1) * d(2)) (-1 * d(1) * d(3)) ;
           (-1 * d(1) * d(2)) (d(1)^2 + d(3)^2) (-1 * d(2) * d(3)) ;
           (-1 * d(1) * d(3)) (-1 * d(2) * d(3)) (d(1)^2 + d(2)^2)];
MI_cabin_pcm = MI_cabin + cabin.m * matrix;

%Calculate MI of body

Ixx_body = (body.m) / 2 * (body.r) ^ 2;
Iyy_body = ((body.m) / 4) * (body.r) ^ 2 + ((body.m) / 12) * (body.h) ^ 2; 
Izz_body = ((body.m) / 4) * (body.r) ^ 2 + ((body.m) / 12) * (body.h) ^ 2;
MI_body = [Ixx_body 0 0; 0 Iyy_body 0; 0 0 Izz_body];

d = pcm - body.vpos;
matrix = [(d(2)^2 + d(3)^2) (-1 * d(1) * d(2)) (-1 * d(1) * d(3)) ;
           (-1 * d(1) * d(2)) (d(1)^2 + d(3)^2) (-1 * d(2) * d(3)) ;
           (-1 * d(1) * d(3)) (-1 * d(2) * d(3)) (d(1)^2 + d(2)^2)];
MI_body_pcm = MI_body + body.m * matrix;

%Calculate MI of wingL
Ixx_wingL = (wingL.m / 12) * (wingL.L ^ 2 + wingL.e ^ 2);
Iyy_wingL = (wingL.m / 12) * (wingL.l ^ 2 + wingL.e ^ 2);
Izz_wingL = (wingL.m / 12) * (wingL.l ^ 2 + wingL.L ^ 2);
MI_wingL = [Ixx_wingL 0 0; 0 Iyy_wingL 0; 0 0 Izz_wingL];

d = pcm - wingL.vpos;
matrix = [(d(2)^2 + d(3)^2) (-1 * d(1) * d(2)) (-1 * d(1) * d(3)) ;
           (-1 * d(1) * d(2)) (d(1)^2 + d(3)^2) (-1 * d(2) * d(3)) ;
           (-1 * d(1) * d(3)) (-1 * d(2) * d(3)) (d(1)^2 + d(2)^2)];
MI_wingL_pcm = MI_wingL + wingL.m * matrix;

%Calculate MI of wingR
Ixx_wingR = (wingR.m / 12) * (wingR.L ^ 2 + wingR.e ^ 2);
Iyy_wingR = (wingR.m / 12) * (wingR.l ^ 2 + wingR.e ^ 2);
Izz_wingR = (wingR.m / 12) * (wingR.l ^ 2 + wingR.L ^ 2);
MI_wingR = [Ixx_wingR 0 0; 0 Iyy_wingR 0; 0 0 Izz_wingR];

d = pcm - wingR.vpos;
matrix = [(d(2)^2 + d(3)^2) (-1 * d(1) * d(2)) (-1 * d(1) * d(3)) ;
           (-1 * d(1) * d(2)) (d(1)^2 + d(3)^2) (-1 * d(2) * d(3)) ;
           (-1 * d(1) * d(3)) (-1 * d(2) * d(3)) (d(1)^2 + d(2)^2)];
MI_wingR_pcm = MI_wingR + wingR.m * matrix;

%Calculate MI of fin
Ixx_fin = (fin.m / 12) * (fin.h ^ 2 + fin.e ^ 2);
Iyy_fin = (fin.m / 12) * (fin.h ^ 2 + fin.l ^ 2);
Izz_fin = (fin.m / 12) * (fin.l ^ 2 + fin.e ^ 2);
MI_fin = [Ixx_fin 0 0; 0 Iyy_fin 0; 0 0 Izz_fin];

d = pcm - fin.vpos;
matrix = [(d(2)^2 + d(3)^2) (-1 * d(1) * d(2)) (-1 * d(1) * d(3)) ;
           (-1 * d(1) * d(2)) (d(1)^2 + d(3)^2) (-1 * d(2) * d(3)) ;
           (-1 * d(1) * d(3)) (-1 * d(2) * d(3)) (d(1)^2 + d(2)^2)];
MI_fin_pcm = MI_fin + fin.m * matrix;

%Calculate MI of motorL
Ixx_motorL = (motorL.m) / 2 * (motorL.r) ^ 2;
Iyy_motorL = ((motorL.m) / 4) * (motorL.r) ^ 2 + ((motorL.m) / 12) * (motorL.L) ^ 2; 
Izz_motorL = ((motorL.m) / 4) * (motorL.r) ^ 2 + ((motorL.m) / 12) * (motorL.L) ^ 2;
MI_motorL = [Ixx_motorL 0 0; 0 Iyy_motorL 0; 0 0 Izz_motorL];

d = pcm - motorL.vpos;
matrix = [(d(2)^2 + d(3)^2) (-1 * d(1) * d(2)) (-1 * d(1) * d(3)) ;
           (-1 * d(1) * d(2)) (d(1)^2 + d(3)^2) (-1 * d(2) * d(3)) ;
           (-1 * d(1) * d(3)) (-1 * d(2) * d(3)) (d(1)^2 + d(2)^2)];
MI_motorL_pcm = MI_motorL + motorL.m * matrix;

%Calculate MI of motorR
Ixx_motorR = (motorR.m) / 2 * (motorR.r) ^ 2;
Iyy_motorR = ((motorR.m) / 4) * (motorR.r) ^ 2 + ((motorR.m) / 12) * (motorR.L) ^ 2; 
Izz_motorR = ((motorR.m) / 4) * (motorR.r) ^ 2 + ((motorR.m) / 12) * (motorR.L) ^ 2;
MI_motorR = [Ixx_motorR 0 0; 0 Iyy_motorR 0; 0 0 Izz_motorR];

d = pcm - motorR.vpos;
matrix = [(d(2)^2 + d(3)^2) (-1 * d(1) * d(2)) (-1 * d(1) * d(3)) ;
           (-1 * d(1) * d(2)) (d(1)^2 + d(3)^2) (-1 * d(2) * d(3)) ;
           (-1 * d(1) * d(3)) (-1 * d(2) * d(3)) (d(1)^2 + d(2)^2)];
MI_motorR_pcm = MI_motorR + motorR.m * matrix;

%Calculate MI of whole plane

MI = MI_cabin_pcm + MI_body_pcm + MI_wingL_pcm + MI_wingR_pcm + MI_fin_pcm + MI_motorL_pcm + MI_motorR_pcm;

end  