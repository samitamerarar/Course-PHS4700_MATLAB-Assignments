function [pcm, MI, aa] = Devoir1(posA,ar,va,Forces)
%DEVOIR1 Summary of this function goes here
%   Detailed explanation goes here
posA = transpose(posA);

va = transpose(va);
Forces = transpose(Forces);

tetax= 0;
tetay = ar;
tetaz = 0;

R2_1 = RotationMat(tetaz,'z')*RotationMat(tetay,'y')*RotationMat(tetax,'x');
R2_1T = RotationMat(-tetaz,'z')*RotationMat(-tetay,'y')*RotationMat(-tetax,'x');

%cabin is a cone
cabin.h = 3.82; %m
cabin.r = 1.345; %m
cabin.m = 700; %kg
cabin.posy = 0; %m
cabin.posx = 0; %m
cabin.posz = 0; %m
cabin.vpos = [cabin.posy,cabin.posx,cabin.posz];

%body is a cylendar
body.h = 22.95; %m
body.r = cabin.r; %m
body.m = 1510; %kg
body.posy = 0;
body.posx = 0;
body.posz = 0;
body.vpos = [body.posy,body.posx,body.posz];

%wingL is a paralapaped representing the left wing
wingL.L = 10.6; %m
wingL.l = 1.14; %m
wingL.e = 0.25; %m
wingL.m = 3250; %kg
wingL.posy = 0; %m
wingL.posx = 0; %m
wingL.posz = 0; %m
wingL.vpos = [wingL.posy,wingL.posx,wingL.posz];

%wingR is a paralapaped representing the right wing
wingR.L = 10.6; %m
wingR.l = 1.14; %m
wingR.e = 0.25; %m
wingR.m = 3250; %kg
wingR.posy = 0; %m
wingR.posx = 0; %m
wingR.posz = 0; %m
wingR.vpos = [wingL.posy,wingL.posx,wingL.posz];

%fin is a solid with a triangle base
fin.h = 2.1; %m
fin.l = 1.28; %m
fin.e = 0.07; %m
fin.m = 500; %kg
fin.posy = 0; %m
fin.posx = 0; %m
fin.posz = 0; %m
fin.vpos = [fin.posy,fin.posx,fin.posz];

%motorL is a cylendar representing the left Motor
motorL.L = 3.68; %m
motorL.r = 0.724; %m
motorL.m = 1750; %kg
motorL.posy = 0; %m
motorL.posx = 0; %m
motorL.posz = 0; %m
motorL.vpos = [motorL.posy,motorL.posx,motorL.posz];

%motorR is a cylendar representing the right Motor
motorR.L = 3.68; %m
motorR.r = 0.724; %m
motorR.m = 1750; %kg
motorR.posy = 0; %m
motorR.posx = 0; %m
motorR.posz = 0; %m
motorR.vpos = [motorR.posy,motorR.posx,motorR.posz];

%Plane is a value containing all the different forms of the plane 
keySet = {'cabin', 'body', 'wingL', 'wingR', 'fin', 'motorL', 'motorR'};
valueSet = {cabin, body, wingL, wingR, fin, motorL, motorR};
plane = containers.Map(keySet,valueSet);

[pcm, planeNewValues] = calculate_pcm(plane);

 cabin = planeNewValues('cabin');
 body = planeNewValues('body');
 wingL = planeNewValues('wingL');
 wingR = planeNewValues('wingR');
 fin = planeNewValues('fin');
 motorL = planeNewValues('motorL');
 motorR = planeNewValues('motorR');

%MI Function
MI = CalculateMI(planeNewValues,pcm);

%aa Function
aa = Calculateaa(body,wingL,motorR,va,pcm,Forces,MI,R2_1,R2_1T);

%Rotate the PCM matrix to put it in the observer axis
pcm = R2_1*transpose(pcm);

%Move the pcm matrix to put it in the observer axis
posAOrigin = [body.h+cabin.h,0,body.r+wingL.e];
pcm = posA-posAOrigin + transpose(pcm);

%Rotate the MI matrix to put it in the observer axis
MI = R2_1*MI*R2_1T;

%Display the variable
disp ('PCM :');
disp(pcm);

disp('MI :');
disp(MI);

disp('aa :');
disp(aa);

end