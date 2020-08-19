function R = RotationMat(teta,axe)
%ROTATIONMAT Create the rotation matrice
%
    switch axe
        case 'x'
            R = [1,0,0;
                0,cos(teta),-sin(teta);
                0,sin(teta),cos(teta)];
        case 'y'
            R = [cos(teta),0,sin(teta);
                0,1,0;
                -sin(teta),0,cos(teta)];

        case 'z'
            R = [cos(teta),-sin(teta),0;
                sin(teta),cos(teta),0;
                0,0,1];       
    end

end