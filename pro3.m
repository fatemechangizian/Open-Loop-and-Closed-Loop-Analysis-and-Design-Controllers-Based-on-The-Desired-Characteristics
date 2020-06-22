clc
K = 12;
b = 10;
a = 70;
s = tf('s');
P = K/(s*(s+b)*(s+a))
poles=pole(P)

%part1
t = 0:0.001:2;
step(P,t)
title('Step Response - P Control')
figure;
%part 1)a
stepinfo(P)

%part 1)b
bode(P), grid
figure;
H = tf([12],[1 80 700 0])
nyquist(H)
figure;
%part 1)c
[Gm,Pm] = margin(P);
GmdB = 20*log10(Gm)   % gain margin in dB
Pm  % phase margin in degrees
[Gm,Pm,Wgm,Wpm] = margin(P)
%part 1)d
rlocus(P)
title('Root Locus - P Control')
figure;

%part 2

%part 2)a
%lead controller 
%in time domain
C1=(385*(s+10))/(s+15)
T1 = feedback(C1*P,1);
step(T1)
figure;
stepinfo(T1)
%in frequency domain
C3=443*(1+0.13*s)
T3 = feedback(C3*P,1);
step(T3)
figure;
stepinfo(T3)
bode(T3),grid
[Gm3,Pm3] = margin(T3)
Gm3dB = 20*log10(Gm3)
figure;
%part 2)b
%pid controller 
Kp=1424.1955;
Ki=503.0944;
Kd=124.1902;
C2= pid(Kp,Ki,Kd)
%C2=(244.99*(1+0.052*s)*(1+4.3*s))/s
T2 = feedback(C2*P,1);
step(T2)
figure;
nyquist(T2)
figure;
stepinfo(T2)
%part 2)c
%State feedback controller
b = [12];
a = [1 80 700 0];
[A,B,C,D] = tf2ss(b,a)
P_ss = ss(A,B,C,D);
sys_order = order(P_ss);
determinant = det(ctrb(A,B))
p1 = -10 + 10i;
p2 = -10 - 10i;
p3 = -60;
Kc = place(A,B,[p1, p2, p3])
t = 0:0.001:2;
sys_cl = ss(A-B*Kc,B,C,D);
[b,a] = ss2tf(A-B*Kc,B,C,D)
W=tf(b,a)
step(sys_cl,t)
stepinfo(sys_cl)
%%
% €ÌÌ— œÂ œ—’œÌ œ— Å«—«„ —Â«:

PP= tf([13.2],[1 88 770 0])
poles=pole(PP)

%lead controller 
%in time domain
C1=(385*(s+10))/(s+15)
T11 = feedback(C1*PP,1);
step(T11)
figure;
stepinfo(T11)
%in frequency domain
C3=443*(1+0.13*s)
T33 = feedback(C3*PP,1);
step(T33)
figure;
stepinfo(T33)
bode(T33),grid
[Gm33,Pm33] = margin(T33)
Gm33dB = 20*log10(Gm33)
figure;

%pid controller 
Kp=1424.1955;
Ki=503.0944;
Kd=124.1902;
C2= pid(Kp,Ki,Kd)
%C2=(244.99*(1+0.052*s)*(1+4.3*s))/s
T22 = feedback(C2*PP,1);
step(T22)
figure;
stepinfo(T22)

%State feedback controller
b = [13.2];
a = [1 88 770 0];
[A,B,C,D] = tf2ss(b,a)
P_ss = ss(A,B,C,D);
sys_order = order(P_ss);
determinant = det(ctrb(A,B))
p1 = -10 + 10i;
p2 = -10 - 10i;
p3 = -60;
Kc = place(A,B,[p1, p2, p3])
t = 0:0.001:2;
sys_cl = ss(A-B*Kc,B,C,D);
[b,a] = ss2tf(A-B*Kc,B,C,D)
W=tf(b,a)
step(sys_cl,t)
stepinfo(sys_cl)