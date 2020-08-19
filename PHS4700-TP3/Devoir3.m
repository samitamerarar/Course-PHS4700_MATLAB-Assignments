function [Coup, tf, vbaf, vbof, wbof, rbaf, rbof ]= Devoir3(vbal,wboi,tl)

%Set initial value
epsi=0.001; %error margin
epsivec = [epsi,epsi,epsi,10,10,10,10,10,10,10,10,10]; %10 valeur arbitraire puisque la precision n'est pas necessaire
e = 0.5; %coefficient of collision
grav = -9.8;
%can vector for all the dynamics value
canVec = [3,0,10,0,0,0,0,0,0,transpose(wboi)]; % [rx, ry, rz, vx, vy, vz, tetx, tety, tetz, wx, wy, wz]
%ball vector for all the dynamics value
ballVec = [0, 0, 2, transpose(vbal), 0, 0, 0, 0, 0, 0]; % [rx, ry, rz, vx, vy, vz, tetx, tety, tetz, wx, wy, wz]
ballm = 0.058; %ball mass
canm = 0.075; %can mass
canR = 0.15/sqrt(6); %can radius
canH = 0.15; %can height
ballR = 0.0335; %ball radius
isFinish = false; %if the simulation is over
vTrajectoryBall = []; %trajectory vector of the ball
vTrajectoryCan = []; %trajectory vector of the can
time = 0;       % time
itTimew = 0.00005;
itTime = itTimew;% iteration time
Coup = 0; %If hit

tic;
while(~isFinish)
   %get the m
   m1=0;
   m2=0;
   [nouse1,m1,nouse2]=SEDRK4t0E(ballVec,time,time+itTimew,epsivec,'kutta_Ball');
   [nouse1,m2,nouse2]=SEDRK4t0E(canVec,time,time+itTimew,epsivec,'kutta_Can');
   m=max(m1,m2);
   itTime = itTimew/2^(m-1);
   
   %look if the can has hit the ground
   if (canVec(3) >= canH/2)
        canVec = canVec + movementInterval_Can(canVec,itTime);%SEDRK4t0(canVec,time,itTime,'movementInterval_Can');
   end
   %look if the time is past the departure time of the ball
   if time > tl
        ballVec = ballVec + movementInterval_Ball(ballVec,itTime);%SEDRK4t0(ballVec,time,itTime,'movementInterval_Ball');        
   end
   %save the trajectory of the ball and the can
   vTrajectoryCan = [vTrajectoryCan; canVec(1:3)];
   vTrajectoryBall = [vTrajectoryBall; ballVec(1:3)];
   %look if there is a collision
   [Coup, isFinish, colpoint] = verifyCollision(ballVec,canVec,epsi);
   %iterate the time
   time = time +itTime;
end
toc;

% GET THE ROTATION MATRIX
[ Rx,Ry,Rz ] = rotationMatrix( canVec(7), canVec(8), canVec(9) );

% MAKE A GRAPH FOR THE TRAJECTORY AND THE COLLISION

figure;
hold on;
%Position of the center of mass at the end for the can
x0=vTrajectoryCan(end,1);y0=vTrajectoryCan(end,2);z0=vTrajectoryCan(end,3);
%Position of the center of mass at the end for the ball
xs0=vTrajectoryBall(end,1);ys0=vTrajectoryBall(end,2);zs0=vTrajectoryBall(end,3);
%Create a cylinder vector
[x,y,z]=cylinder(canR);
%create a sphere vector
[xs,ys,zs]=sphere;
%adjust the size of the sphere and move it to the good position
xs=xs*ballR+xs0;
ys=ys*ballR+ys0;
zs=zs*ballR+zs0;

%adjust the height of the cylinder and the position of the center of mass
z=z*canH-canH/2;
% get points at the two rings and rotate them separately:
positionOld1 = [x(1,:)',y(1,:)',z(1,:)'];
positionOld2 = [x(2,:)',y(2,:)',z(2,:)'];
positionNew1 = transpose(Rz*Ry*Rx*transpose(positionOld1));
positionNew2 = transpose(Rz*Ry*Rx*transpose(positionOld2));

% reassemble the two sets of points into X Y Z format:
x = [positionNew1(:,1),positionNew2(:,1)];
y = [positionNew1(:,2),positionNew2(:,2)];
z = [positionNew1(:,3),positionNew2(:,3)];

%Move the cylinder to the good position
x=x+x0;
y=y+y0;
z=z+z0;

%add these figure to the graph
surf(x,y,z);
surf(xs,ys,zs);

%draw the trajectory
plot3(vTrajectoryCan(:,1),vTrajectoryCan(:,2),vTrajectoryCan(:,3));
plot3(vTrajectoryBall(:,1),vTrajectoryBall(:,2),vTrajectoryBall(:,3));
if Coup == 1
    plot3(colpoint(1),colpoint(2),colpoint(3),'o');
    xlim([colpoint(1)-0.25,colpoint(1)+0.25]);
    ylim([colpoint(2)-0.25,colpoint(2)+0.25]);
    zlim([colpoint(3)-0.25,colpoint(3)+0.25]);
else
    xlim([0,6]);
    ylim([-3,3]);
    zlim([-1,10]);
end

grid on;
xlabel('x');
ylabel('y');
zlabel('z');
hold off;
view(3);

%set final time
tf=time;
%set collision position
rbaf=ballVec(1:3);
rbof=canVec(1:3);
%set speed before collision
vbaf(1:3,1)=ballVec(4:6);
vbof(1:3,1)=canVec(4:6);
%set speed after collision
vbaf(1:3,2)=vbaf(1:3,1);
vbof(1:3,2)=vbof(1:3,1);
%set rotation speed after collision
wbof=wboi;

%If there is a collision, recalculate the speed and rotation speed
if Coup == 1
    %r can collision
    rcolcan = (colpoint-transpose(canVec(1:3)));
    %I can
    Ic = [canm/2*canR^2+canm/12*canH^2,0,0;0,canm/2*canR^2+canm/12*canH^2,0;0,0,canm*canR^2];
    Ic = Rz*Ry*Rx*transpose(Ic);
    Ic = Ic + canm * [rcolcan(2)^2+rcolcan(3)^2,-rcolcan(1)*rcolcan(2),-rcolcan(1)*rcolcan(3);
        -rcolcan(1)*rcolcan(2),rcolcan(1)^2+rcolcan(3)^2, -rcolcan(2)*rcolcan(3);
        -rcolcan(1)*rcolcan(3),-rcolcan(2)*rcolcan(3),rcolcan(1)^2+rcolcan(2)^2];
    %r ball collision
    rcolball = (colpoint - transpose(ballVec(1:3)));
    %I ball
    Ib = [1,0,0;0,1,0;0,0,1]*2/5*ballm*ballR^2;
    Ib = Ib + ballm * [rcolball(2)^2+rcolball(3)^2,-rcolball(1)*rcolball(2),-rcolball(1)*rcolball(3);
        -rcolball(1)*rcolball(2),rcolball(1)^2+rcolball(3)^2, -rcolball(2)*rcolball(3);
        -rcolball(1)*rcolball(3),-rcolball(2)*rcolball(3),rcolball(1)^2+rcolball(2)^2];
    %v can on collision point
    vcanp = transpose(canVec(4:6)) + cross(wboi,rcolcan);
    %v ball on collision point
    vballp = transpose(ballVec(4:6));
    %n_chap vector
    n = (transpose(ballVec(1:3)) - colpoint);
    n_chap = n/norm(n);
    %vr- value
    vr = vcanp-vballp;
    v = transpose(n_chap)*vr;
    %G can
    Gc=transpose(n_chap)*cross(((Ic)\(cross(rcolcan,n_chap))),rcolcan);
    %G ball
    Gb=transpose(n_chap)*cross(((Ib)\(cross(rcolball,n_chap))),rcolball);
    %alpha
    alpha_ = 1/((1/ballm)+(1/canm)+Gc+Gb);
    %j
    j = -alpha_*(1+e)*v;
    %J
    J = n_chap*j;
    %Ball after collision velocity
    vbaf(1:3,2) = vbaf(1:3,1) - J/ballm;
    %Can after collision velocity
    vbof(1:3,2) = vbof(1:3,1) + J/canm;
    %Can after collision angular velocity
    wbof = wboi + (Ic)\(cross(rcolcan,J));
end

end
