function [coup, isFinish, colPos] = verifyCollision(ball,can,e)
%VERIFYPOSITION Check if ball and can are in collision
%
% Syntax:  [isFinish ] = verifyPosition(ball,can)
%
% -Inputs-
%       ball        :   Ball values containing :
%                       position, velocity, rotation velocity, 
%                       accelaration, angular acceleration, its radius
%       can         :   Can values containing :
%                       position, velocity, rotation velocity, 
%                       accelaration, angular acceleration, its size
%       e           :   Error margin in meters
%
% -Outputs-
%       isFinish	:   Boolean for end of simulation
%
% -Possible Outputs-
%       isFinish = false (simulation not ended yet)
%       isFinish = The ball and the can colide

ballmc = ball(1:3);
canmc = can(1:3);
ballR = 0.0335;
canR = 0.15/sqrt(6);
canH = 0.15;
coup = 0;
colPos =[0,0,0];
isFinish = false;
%ROTATE
alpha = can(7); beta = can(8); gamma = can(9);
[ Rx,Ry,Rz ] = rotationMatrix( -alpha, -beta, -gamma );

canmc = Rz*Ry*Rx*transpose(canmc);
ballmc = Rz*Ry*Rx*transpose(ballmc);

%RECENTER
temp = transpose(canmc);
ballmc = ballmc-canmc;
canmc = [0;0;0];

ballmc = transpose(ballmc);
canmc = transpose(canmc);
Rcont1 = sqrt(ballR^2-(ballmc(3)-canH/2)^2);
Rcont2 = sqrt(ballR^2-(ballmc(3)+canH/2)^2);
%LOOK AT COLLISION

%INTERSECTION WITH TOP OR BOTOM
if (ballmc(1)^2+ballmc(2)^2 < (canR)^2 && ~isFinish)
    if (ballmc(3)-canH/2 <= ballR && ballmc(3)-canH/2 >= 0)
        colPos = [ballmc(1),ballmc(2),canH/2];
        coup = 1;
        isFinish = true;
    elseif (ballmc(3)+canH/2 >= -ballR && ballmc(3)+canH/2 <= 0)
        colPos = [ballmc(1),ballmc(2),-canH/2];
        coup = 1;
        isFinish = true;
    end
end
%collision on side
if (ballmc(3) < canH/2 && ballmc(3) > -canH/2 && ~isFinish)
    if (sqrt(ballmc(1)^2+ballmc(2)^2)<= canR + ballR)
        colPos(1:2) = ballmc(1:2)*canR/sqrt(ballmc(1)^2+ballmc(2)^2);
        colPos(3) = ballmc(3);
        isFinish = true;
        coup = 1;
    end
end
%collision on arrette
if ((ballmc(3)-canH/2 <= ballR && ballmc(3)-canH/2 >= 0) && canR < sqrt(ballmc(1)^2+ballmc(2)^2) && sqrt(ballmc(1)^2+ballmc(2)^2) <= (ballR + canR) && ~isFinish)
    if (sqrt(ballmc(1)^2+ballmc(2)^2) <= canR + Rcont1)
        colPos(1:2) = ballmc(1:2)*canR/sqrt(ballmc(1)^2+ballmc(2)^2);
        colPos(3) = canH/2;
        isFinish = true;
        coup = 1;
    end
end
if ((ballmc(3)+canH/2 < 0 && ballmc(3)+canH/2 >= -ballR) && canR < sqrt(ballmc(1)^2+ballmc(2)^2) && sqrt(ballmc(1)^2+ballmc(2)^2) <= (ballR + canR) && ~isFinish)
    if (sqrt(ballmc(1)^2+ballmc(2)^2) <= canR + Rcont2)
        colPos(1:2) = ballmc(1:2)*canR/sqrt(ballmc(1)^2+ballmc(2)^2);
        colPos(3) = -canH/2;
        isFinish = true;
        coup = 1;
    end  
end
%NO POSSIBLE COLLISION

%COL POS
if (isFinish)
    colPos = colPos + temp;
    [ Rx,Ry,Rz ] = rotationMatrix( alpha, beta, gamma );
    colPos = Rz*Ry*Rx*transpose(colPos);
end

%BALL OR CAN HITTING GROUND
if (ball(3)<= ballR)
    isFinish = true;
    colPos = ball(1:3);
end

end
