%% parameter
clear all;
global u;
global v;
global rMesP;
global xMesP;
global yMesP;
global wx;
global wy;
global w;

d=10;       dmin=3;
%wx=0.0;     wy=0.0;    
%w=(wx^2+wy^2)^0.5;  % the velocity of wind
% initial condition
o_x= 0;    % starting x  
o_y= 0;    % starting y
dr=15;
xRec=[];
yRec=[]; 
thRec=[];
tRec=[];
rRec=[];
v=1;      % velocity
u=v/dr;      % control u
o_theta=[atan2(o_y,o_x)+pi];   % theta of vehicle at starting point  
X0=[o_x;o_y;o_theta]; 
rMesP=sqrt(o_x^2+o_y^2); % r at starting point(r: distance between vehicle and center)
xMesp=o_x;
yMesp=o_y;

% configuration
n=700;         % n:# of loop, 
dT=0.5;         % dT: time duration,  
k=0.1;
lambda=1;
rs=0.2;
i=0;
mode = 0; %(control mode 0: original law; 1: control candidate 1)

uRec=[];    % control u
x2Rec=[];   % x2 

flagFirst=1;
tRec=[];    % time
tAccu=[0];  % tAccu used for accumulating time


%%

while i<n,
  i=i+1;
  
  [t X]=ode45(@uavModel,[0 dT],X0);    
 
    xRec=[xRec;X(:,1)];
    yRec=[yRec;X(:,2)];
    thRec=[thRec;X(:,3)];
    tRec=[tRec;t+tAccu(end)];
    xMes=X(end,1);
    yMes=X(end,2);
    beta=atan2(yMes,xMes);
    alpha=pi+X(end,3)-beta;
    
    
    rMes=sqrt(xMes^2+yMes^2);
    x2hat= 1/dT*(rMes-rMesP);
    
    x2Rec=[x2Rec;ones(size(t))*x2hat];
    
    if rMes>rs
        u=lambda*k*v*cos(alpha) - lambda*k*v*sqrt(rMes^2-rs^2)/rMes;
    else
        u=lambda*k*v*cos(alpha) + lambda*k*v*sqrt(rs^2-rMes^2)/rs;
    end
    
     u=real(u);
     
     
     % u is bounded between -v/dmin to v/dmin
%      if u>(v/dmin)
%          u=v/dmin;
%      elseif u<-(v/dmin)
%          u=-v/dmin;
%      end
% %      %
     
    uRec=[uRec;ones(size(t))*u];
    flagFirst=0;

    tAccu=[tAccu;tAccu(end)+t]; %update time
  
  X0(1,1)=X(end,1);         % record the last point
  X0(2,1)=X(end,2);     
  X0(3,1)=X(end,3);
  rMesP=rMes;               % update r (r: distance to the center)
  xMesP=xMes;
  yMesP=yMes;
 
end

%% collect data of r(distance between vehicle to the center)

for i=1:size(xRec),
    rRec=[rRec;sqrt(xRec(i)^2+yRec(i)^2)];
end



%% plot the data
figure(1)
plot(xRec,yRec);
title(' y/x plot');
xlabel('X');
ylabel('Y');

figure(2)
plot(tRec,uRec);
xlabel('time');
ylabel('control u');

% figure(3)
% plot(tRec,x2Rec);
% xlabel('time');
% ylabel('x2');
% 
figure(4)
plot(tRec,rRec);
title('r vs time plot')
xlabel('time');
ylabel('rRec');

