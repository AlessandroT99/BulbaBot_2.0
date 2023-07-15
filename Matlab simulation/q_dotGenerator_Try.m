% DEFINITION:    The evaluation of the Jacobian permits to get the value of
%                q_dot, which after has been integrated begin the needed q

% INPUT:         q = quantized rounded expected angle
%                ke = results of the multiplication of the error with the gain correction
%                a = values of the links length

% OUTPUT:        q_dot = angular velocity in rads/s of the joints

xd = [1;1;1];
a = [10,10,10];
A = [1,0,0;0,1,0;0,0,1];
ke = A*xd;
q = ke';

%% Constant declaration
    s1 = sin(q(1));
    c1 = cos(q(1));
    s2 = sin(q(2));
    c2 = cos(q(2));
    s3 = sin(q(3));
    c3 = cos(q(3));
    
%% Jacobian computation
%     Transformation matrix T03
%     T03 = [c1c2c3-c1s2s3    -c1c2s3-c1s2c3      s1      a3c1c2c3-a3c1s2s3+a2c1c2+a1c1
%            s1c2c3-s1s2s3    -s1c2s3-s1s2c3      -c1     a3s1c2c3-a3s1s2s3+a2s1c2+a1s1
%            s2c3+c2s3        -s2s3+c2c3          0       a3s2c3+a3c2s3+a2s2
%            0                0                   0       1]

%     Jacobian computation 
%     where J(i,k) = dk(i)/dq(k)
%     where k(i) = O03(i)
%     notice that Tab = [Rab,Oab;0,0,0,1]

    J11 = -a(3)*s1*c2*c3+a(3)*s1*s2*s3-a(2)*s1*c2-a(1)*s1;
    J12 = -a(3)*c1*s2*c3-a(3)*c1*c2*s3-a(2)*c1*s2;
    J13 = -a(3)*c1*c2*s3-a(3)*c1*s2*c3;
    J21 = a(3)*c1*c2*c3-a(3)*c1*s2*s3+a(2)*c1*c2+a(1)*c1;
    J22 = -a(3)*s1*s2*c3-a(3)*s1*c2*s3-a(2)*c1*s2;
    J23 = -a(3)*s1*c2*s3-a(3)*s1*s2*c3;
    J31 = 0;
    J32 = a(3)*c2*c3-a(3)*s2*s3+a(2)*c2;
    J33 = -a(3)*s2*s3+a(3)*c2*c3;

%% Result evaluation
%     q_dot = J'*ke*e
    q1_dot = J11*ke(1)+J21*ke(2)+J31*ke(3);
    q2_dot = J12*ke(1)+J22*ke(2)+J32*ke(3);
    q3_dot = J13*ke(1)+J23*ke(2)+J33*ke(3);
    q_dot = [q1_dot,q2_dot,q3_dot];
