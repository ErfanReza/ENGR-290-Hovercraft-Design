clc;
clear;
close all;
mass1 = 0.551635;
mass2 = 0.5486205;
mass3 = 0.387919;
t=[0:10];
f = 0.299335;
f3 = f/2;
a1 = [f/mass1,f/mass1,f/mass1,f/mass1,f/mass1,f/mass1,f/mass1,f/mass1,f/mass1,f/mass1,f/mass1];
a2 = [f/mass2,f/mass2,f/mass2,f/mass2,f/mass2,f/mass2,f/mass2,f/mass2,f/mass2,f/mass2,f/mass2];
a3 = [f3/mass3,f3/mass3,f3/mass3,f3/mass3,f3/mass3,f3/mass3,f3/mass3,f3/mass3,f3/mass3,f3/mass3,f3/mass3];
figure(1);
subplot(3,3,1);
plot(t,a1);
xlabel('Seconds');
ylabel('m/s^2');
title('Design 1 Acceleration')
subplot(3,3,2);
plot(t,a2);
xlabel('Seconds');
ylabel('m/s^2');
title('Design 2 Acceleration')
subplot(3,3,3);
plot(t,a3);
xlabel('Seconds');
ylabel('m/s^2');
title('Design 3 Acceleration')
v1 = a1.*t;
v2 = a2.*t;
v3 = a3.*t;

subplot(3,3,4);
plot(t,v1);
xlabel('Seconds');
ylabel('m/s');
title('Design 1 Velocity ');
subplot(3,3,5);
plot(t,v2);
xlabel('Seconds');
ylabel('m/s');
title('Design 2 Velocity ');
subplot(3,3,6);
plot(t,v3);
xlabel('Seconds');
ylabel('m/s');
title('Design 3 Velocity ');
d1=(v1/2).*t;
d2=(v2/2).*t;
d3=(v3/2).*t;
subplot(3,3,7);
plot(t,d1);
xlabel('Seconds');
ylabel('m');
title('Design 1 Linear Displacement');
subplot(3,3,8);
plot(t,d2);
xlabel('Seconds');
ylabel('m');
title('Design 2 Linear Displacement');
subplot(3,3,9);
plot(t,d3);
xlabel('Seconds');
ylabel('m');
title('Design 3 Linear Displacement');
%t for torque
t1 = 7.25*f*sqrt(2)/2;
t2 = 8.36*f;
t3 = 2.5*f3*sqrt(2)/2;

%l for alpha
l1 = [t1/1.359,t1/1.359,t1/1.359,t1/1.359,t1/1.359,t1/1.359,t1/1.359,t1/1.359,t1/1.359,t1/1.359,t1/1.359];
l2 = [t2/2.216,t2/2.216,t2/2.216,t2/2.216,t2/2.216,t2/2.216,t2/2.216,t2/2.216,t2/2.216,t2/2.216,t2/2.216];
l3 = [t3/3.857,t3/3.857,t3/3.857,t3/3.857,t3/3.857,t3/3.857,t3/3.857,t3/3.857,t3/3.857,t3/3.857,t3/3.857];
figure(2);
subplot(3,3,1);
plot(t,l1);
xlabel('seconds');
ylabel('rads/s^2');
title('Design 1 Angular Acceleration');
subplot(3,3,2);
plot(t,l2);
xlabel('seconds');
ylabel('rads/s^2');
title('Design 2 Angular Acceleration');
subplot(3,3,3);
plot(t,l3);
xlabel('seconds');
ylabel('rads/s^2');
title('Design 3 Angular Acceleration');
av1 = l1.*t;
av2 = l2.*t;
av3 = l3.*t;
subplot(3,3,4);
plot(t,av1);
xlabel('seconds');
ylabel('rads/s');
title('Design 1 Angular Velocity');
subplot(3,3,5);
plot(t,av2);
xlabel('seconds');
ylabel('rads/s');

title('Design 2 Angular Velocity');
subplot(3,3,6);
plot(t,av3);
xlabel('seconds');
ylabel('rads/s');
title('Design 3 Angular Velocity');
ad1 = (av1/2).*t;
ad2 = (av2/2).*t;
ad3 = (av3/2).*t;
subplot(3,3,7);
plot(t,av1);
xlabel('seconds');
ylabel('rads');
title('Design 1 Angular Displacement');
subplot(3,3,8);
plot(t,av2);
xlabel('seconds');
ylabel('rads');
title('Design 2 Angular Displacement');
subplot(3,3,9);
plot(t,av3);
xlabel('seconds');
ylabel('rads');
title('Design 3 Angular Displacement');
