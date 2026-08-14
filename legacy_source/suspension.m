clear
close all
clc

for i=1:2000
    
    if  i<400
        d(i)=0.1*sin(.5*pi*i/200);
    elseif i>800 && i<1200
        d(i)=0.06*sin(.5*pi*i/200);
    else
        d(i)=0;
    end
    i=i+1;
end
T=linspace(0,100,2000);
plot(T,d,'k','linewidth',2)

%% Parameters
m=100;
m_prime=10;
k1=200;
k2=k1;
c=500;

A=[0 1 0 0;
    -k1/m -c/m k1/m c/m;
    0 0 0 1;
    k1/m_prime c/m_prime -(k1+k2)/m_prime -c/m_prime];
B=[0 0;
    1/m 0;
    0 0;
    -1/m_prime k2/m_prime];
C=[1 0 0 0];
D=[0 0];


G=ss(A,B,C,D);


%% simulation
% input 
u_c=zeros(size(T));
u=[u_c;d];

% initial conditions
x0=[0;0;0;0];

% output
y=lsim(G,u,T,x0);

plot(T,y,'b',T,d,'r:','linewidth',2)
legend('System Response','Disturbance')

xlabel('Time (s)')
ylabel('Displacement (m)')