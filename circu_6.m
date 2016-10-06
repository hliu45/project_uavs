%% parameter
clear all;
global u;
global v;
global rMesP;

c1=2.4;     c3=0.5;
c2=11;      c4=0;
d=10;       dmin=4;
rd=d;       % for calculate average error

% initial condition
aveR=[];  % used for collect average value of error
Kaccum=[];% used for collect gain k

K=[-0.1635 -1.1519];
for i=1:16
% starting point
o_x= -15;      
o_y= -15;    
o_theta=3*pi/4;   % theta of vehicle at starting point 
X0=[o_x;o_y;o_theta]; 
rMesP=sqrt(o_x^2+o_y^2); % r at starting point(r: distance between vehicle and center)


% initial the data x, y, theta, time and r to empty
xRec=[];
yRec=[]; 
thRec=[];
tRec=[];
rRec=[];

% velocity
v=1;        % velocity
u=v/15;      % initial control u

% configuration
n=2000;         % n:# of loop, 
dT=0.4;         % dT: sample time,  
i=0;

uRec=[];    % control u
x2Rec=[];   % x2 

tRec=[];    % time
tAccu=[0];  % tAccu used for accumulating time

%% LQR configuration
%  A=[0 1; -v^2/d^2 0];
%   B=[0;-v];
%   C=[1 1];
%   D=[0 0];
%   Q=[0.03 0;0 1];
%   R=1;
%   [K,S,e]=lqr(A,B,Q,R)

%%

while i<n,
  i=i+1;
  
  [t X]=ode45(@uavModel,[0 dT],X0);    
 
    xRec=[xRec;X(:,1)];     % collecting x value from ode45
    yRec=[yRec;X(:,2)];     % collecting y value
    thRec=[thRec;X(:,3)];   % for theta value
    tRec=[tRec;t+tAccu(end)]; %time
    xMes=X(end,1);          
    yMes=X(end,2);
    rMes=sqrt(xMes^2+yMes^2);%+normrnd(0,0.5);
    x2hat= 1/dT*(rMes-rMesP)+normrnd(0,0.5); % create noise with sigma=0.5 and mean value =0
    
    x2Rec=[x2Rec;ones(size(t))*x2hat];

    % control u
%         u=(1/sqrt(v^2-x2hat^2))*(-K(1)*(rMes-d) + (v^2-x2hat^2)/rMes...
%          -K(2)*x2hat);  
        u=(1/sqrt(v^2-x2hat^2))*(c1*(rMes-d)/c2 + (v^2-x2hat^2)/rMes...
         +c3*x2hat);
         u=real(u);
     
    if u>1.5,
         u=1.5;
    elseif u< -1.5,
         u=-1.5;
    end
        
        
    uRec=[uRec;ones(size(t))*u]; % collecting u
    tAccu=[tAccu;tAccu(end)+t];  %update time
  
 % record the last point
  X0(1,1)=X(end,1);         
  X0(2,1)=X(end,2);     
  X0(3,1)=X(end,3);
  % update r value
  rMesP=rMes;               
 
end

%% collect data of r(distance between vehicle to the center)
for i=1:size(xRec),
    rRec=[rRec;sqrt(xRec(i)^2+yRec(i)^2)];
end

%% calculate average value of error 
sumr=0;
aver=0;
for j=20555:size(rRec),
    sumr=sumr+(rRec(j)-rd)^2;
end
nu=size(rRec)-20555;
aver=sumr/nu(1);
aveR=[aveR;aver];

end
disp(aveR);
%% plot the data
% figure(1)
% plot(xRec,yRec);
% axis([-20 20 -20 20]);
% title(' y/x plot');
% xlabel('X');
% ylabel('Y');
% hold on,
% ang=0:0.01:2*pi;
% xp=rd*cos(ang)+0;
% yp=rd*sin(ang)+0;
% plot(yp,xp);
% hold off

% figure(2)
% plot(tRec,uRec);
% axis([0 600 -1.5 2]);
% xlabel('time');
% ylabel('control u');

% figure(3)
% plot(tRec,x2Rec);
% xlabel('time');
% ylabel('x2');
% 
% figure(4)
% plot(tRec,rRec);
% title('r vs time plot')
% xlabel('time');
% ylabel('rRec');



