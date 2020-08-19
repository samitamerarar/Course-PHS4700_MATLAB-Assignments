function [acc] = Calculateaa(body,wingL,motorR,va,pcm,Force,MI,R2_1,R2_1T)
%aa is the function that will pull out the angular accelaration 
%it takes in account all the other factors

%Create wWave matrix
wWave = [0,-va(3),va(2);
    va(3),0,-va(1);
    -va(2),va(1),0];

%identify the position and the direction of the native force
posForce_md = [5,body.r+motorR.r,body.r+wingL.e];
dirForce_md = [1,0,0];
posForce_mg = [5,-(motorR.r+body.r),body.r+wingL.e];
dirForce_mg = [1,0,0];
posForce_p =  [10.54,0,0];
dirForce_p = [0,0,1];

%Create the force matrix from the force value and the rotated position
posForce_mg = R2_1*transpose(posForce_mg-pcm);
posForce_md = R2_1*transpose(posForce_md-pcm);
posForce_p = R2_1*transpose(posForce_p-pcm);

%Calculate the matrix of the force with it's direction
matForce_md = R2_1*transpose((dirForce_md*Force(1)));
matForce_mg = R2_1*transpose((dirForce_mg*Force(2)));
matForce_p = transpose((dirForce_p*Force(3)));

%Calculate the torque of the force on the mass center
toMotorL = torqueTo(matForce_mg,posForce_mg);
toMotorR = torqueTo(matForce_md,posForce_md);
toPush = torqueTo(matForce_p,posForce_p);

%Sum the torque on the mass center
totalTo = toMotorR + toMotorL + toPush;

%Retrieve the angular acceleration matrix
acc = (R2_1*MI*R2_1T)\(transpose(totalTo)-wWave*R2_1*MI*R2_1T*transpose(va));

end


function to = torqueTo(matForce,posForce)
%calculate the torque from the force, the foce position and mass center
%
to = [posForce(2)*matForce(3)-posForce(3)*matForce(2),posForce(1)*matForce(3)-posForce(3)*matForce(1),posForce(1)*matForce(2)-posForce(2)*matForce(1)];

end