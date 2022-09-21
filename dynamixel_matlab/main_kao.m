global motor1;
global motor2;
global motor3;

motor1=0;
motor2=0;
motor3=0;

%run("init_3_dynamixels.m");


global a1
global a2
global a3

global motor1;
global motor2;
global motor3;


a1=0.09;
a2=0.19;
a3=0.18;

pd = inverseVector(0.3, 0, 0.3) %kutevi rotacije motora za (x, y, z)
ad = directMatrix(0,pi/2,0);      %(x, y, z) za rotaciju motora
ad_fancy = ad([1:3],4)  

sendToMotor(pi/4, 0, pi/4)


function sendToMotor(fi1,fi2,fi3)
    global motor1;
    global motor2;
    global motor3;

    motor1=rad2deg(fi1);
    motor2=rad2deg(fi2);
    motor3=rad2deg(fi3);
end

%DIREKTNA KINEMATIKA
function T0T = directMatrix(p0, p1, p2)

    global a1
    global a2
    global a3

    T2T = [-sin(p2) 0 cos(p2) a3*cos(p2); cos(p2) 0 sin(p2) a3*sin(p2); 0 1 0 0; 0 0 0 1];
    T12 = [cos(p1) -sin(p1) 0 a2*cos(p1); sin(p1) cos(p1) 0 a2*sin(p1); 0 0 1 0; 0 0 0 1];
    T01 = [0 cos(p0) -sin(p0) 0; 0 sin(p0) cos(p0) 0; 1 0 0 a1; 0 0 0 1];

    T0T = T01*T12*T2T;
    xyz = T0T([1:3],4) 
    return
end

%INVERZNA KINEMATIKA
function pd = inverseVector(x,y,z)
    global a1
    global a2
    global a3

    q1 = atan2(y, x);
    r1 = sqrt(x^2+y^2);
    r2 = z - a1;
    fi2 = atan2(r2, r1);
    r3 = sqrt(r2^2+r1^2);
    fi1 = acos((a3^2 - a2^2 - r3^2)/(-2*a2*r3));
    q2 = pi/2 - (fi1+fi2);
    fi3 = acos((r3^2-a2^2 -a3^2)/(-2*a2*a3));
    q3=pi-fi3;

    p = [q1, , angle(q3)]
    pd = [rad2deg(p(1)), rad2deg(p(2)), rad2deg(p(3))];

    q1 = pd(1);
    q2 = pd(2);
    q3 = pd(3);
    return
end
