%Function that permits to evaluate the rotation matrix from the
%denavit-hartenberg convention and returns the value of the end effector
%position from the motor angles.

%Model evaluated for an antropomorphic manipulator
function [x3,y3,z3] = DHmatrixGenerator(theta,d,a,alpha,outputDim)    

    if nargin == 4
        outputDim = 1;
    end

    %% Dimension check
    inputs = {theta,d,a,alpha};
    for i = 1:length(inputs)
        if (~isvector(inputs{i}) || length(inputs{i}) ~= 3)
            ME = MException('MyComponent:inputError','Dimension of input variable %d not valid', i);
            throw(ME)
        end
    end
    clear inputs

    if outputDim == 1
        %% Function execution for single values
        %Generation of the T matrix with the aim of DH convention formula
        T = eye(4);

        z = zeros(3,4);
        z(:,1) = [0,0,1]';
        p = zeros(3,4);

        for i = 1:3
            T = T * [cos(theta{i}),-sin(theta{i})*cos(alpha(i)),sin(theta{i})*sin(alpha(i)),a(i)*cos(theta{i});
                     sin(theta{i}),cos(theta{i})*cos(alpha(i)),-cos(theta{i})*sin(alpha(i)),a(i)*sin(theta{i});
                     0,sin(alpha(i)),cos(alpha(i)),d(i);
                     0,0,0,1];
            z(i+1) = T(1:3,3);
            p(i+1) = T(1:3,4);
        end
        
        x3 = T(1,4);
        y3 = T(2,4);
        z3 = T(3,4);

        %% Jacobian evaluation
        J1 = [cross(z(:,1),p(:,4)-p(:,1));z(:,1)];
        J2 = [cross(z(:,2),p(:,4)-p(:,2));z(:,2)];
        J3 = [cross(z(:,3),p(:,4)-p(:,3));z(:,3)];
        J = [J1,J2,J3];
        sigJ = J(1:3,1:3);
        
    else 
        %% Function execution for a whole set of data
        if outputDim > 1
            [Q1,Q2,Q3] = ndgrid(theta{1},theta{2},theta{3}); %This create matrices of 180x180x180 for each variable
            x3 = cos(Q1).*(a(2).*cos(Q2)+a(3).*cos(Q2+Q3));
            y3 = sin(Q1).*(a(2).*cos(Q2)+a(3).*cos(Q2+Q3));
            z3 = a(2)*sin(Q2)+a(3)*sin(Q2+Q3);
        else
            ME = MException('MyComponent:inputError','Dimension %d not valid',outputDim);
            throw(ME)
        end
    end


end

