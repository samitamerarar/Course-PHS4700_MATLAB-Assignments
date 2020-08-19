function [pcm, planeNewValues] = calculate_pcm(plane) 
        
    cabin = plane('cabin');
    body = plane('body');
    wingL = plane('wingL');
    wingR = plane('wingR');
    fin = plane('fin');
    motorL = plane('motorL');
    motorR = plane('motorR');
        
        %cone 
        center_Cone = [body.h+(cabin.h/2),0,cabin.r];  
        cabin.posx = center_Cone(1);
        cabin.posy = center_Cone(2);
        cabin.posz = center_Cone(3);
        cabin.vpos = center_Cone; 
           
        %cylindre
        center_Cylinder = [body.h/2,0,body.r];
        body.posx = center_Cylinder(1);
        body.posy = center_Cylinder(2);
        body.posz = center_Cylinder(3);
        body.vpos = center_Cylinder;
 
        %Aile
        center_WingL = [10.54,wingL.L/2,wingL.e/2];  %Left
        wingL.posx = center_WingL(1);
        wingL.posy = center_WingL(2);
        wingL.posz = center_WingL(3);
        wingL.vpos = center_WingL;
        
        center_WingR = [10.54,-wingR.L/2,wingR.e/2];  %Right
        wingR.posx = center_WingR(1);
        wingR.posy = center_WingR(2);
        wingR.posz = center_WingR(3);
        wingR.vpos = center_WingR;
        
        %fin 
        center_Fin = [fin.l/2,0,(2*cabin.r+wingL.e)];
        fin.posx = center_Fin(1);
        fin.posy = center_Fin(2);
        fin.posz = center_Fin(3);
        fin.vpos = center_Fin;
        
        %moteur 
        center_MotorR = [5,(body.r + motorR.r),body.r+wingR.e];   %Right
        motorR.posx = center_MotorR(1);
        motorR.posy = center_MotorR(2);
        motorR.posz = center_MotorR(3);
        motorR.vpos = center_MotorR;
        
        center_MotorL = [5,-(body.r + motorR.r), body.r + wingL.e];   %Left
        motorL.posx = center_MotorL(1);
        motorL.posy = center_MotorL(2);
        motorL.posz = center_MotorL(3);  
        motorL.vpos = center_MotorL;
            
           
        
        centerObjects = [center_Cone; center_Cylinder; center_WingL; center_WingR; center_Fin; center_MotorR; center_MotorL];
        masssObjects = [cabin.m; body.m; wingL.m; wingR.m; fin.m; motorR.m ; motorL.m];
        pcm = CenterTot(centerObjects, masssObjects);        
        
        keySet = {'cabin', 'body', 'wingL', 'wingR', 'fin', 'motorL', 'motorR'};
        valueSet = {cabin, body, wingL, wingR, fin, motorL, motorR};
        planeNewValues = containers.Map(keySet,valueSet);

end

 %Center of Mass
            function [centerTot] = CenterTot(center_Objects, massObjects)
			masseTot = sum(massObjects);
            center = [0,0,0];
            %Loop on all the diffrents parts of the plane
            for i = 1:numel(massObjects)
				centerObjectP = [center_Objects(i,1) , center_Objects(i,2) , center_Objects(i,3)] * massObjects(i);
                center = center + centerObjectP;
            end 
			
            centerTot = center / masseTot;
		
            end
            
        