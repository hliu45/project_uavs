%% parameter
clear all;
global u;
global v;
global rMesP;
d=10;       dmin=4;
% initial condition

delta=0.83;
gamma=0.3;
d0=2;
    
o_x= 120;    % starting x  
o_y= 100;    % starting y

xRec=[];
yRec=[]; 
thRec=[];
tRec=[];
rRec=[];
v=0.5;        % velocity
u=1;          % control u
%o_theta=[atan2(o_y,o_x)+pi];   % theta of vehicle at starting point  
o_theta=0;
X0=[o_x;o_y;o_theta]; 
rMesP=sqrt(o_x^2+o_y^2); % r at starting point(r: distance between vehicle and center)

% configuration
n=3000;         % n:# of loop, 
dT=0.2;         % dT: time duration,  
%k=1.3;
lambda=1;
rd=10;
i=0;

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
    rMes=sqrt(xMes^2+yMes^2);
    x2hat= 1/dT*(rMes-rMesP);
    
    x2Rec=[x2Rec;ones(size(t))*x2hat];

%%%%%%%%%%%% (7)
    r=rMes-d0;    
    if abs(r)<=delta,
        xx=gamma*r;
    elseif r>delta,
        xx=gamma*delta;
    elseif r<-delta,
        xx=-gamma*delta;
    end
    
    u=u*sign(x2hat+xx);
%%%%%%%%%%%%%%    
    uRec=[uRec;ones(size(t))*u];
    flagFirst=0;

    tAccu=[tAccu;tAccu(end)+t]; %update time
  
  X0(1,1)=X(end,1);         % record the last point
  X0(2,1)=X(end,2);     
  X0(3,1)=X(end,3);
  rMesP=rMes;               % update r (r: distance to the center)
 
end

%% collect data of r(distance between vehicle to the center)

for i=1:size(xRec),
    rRec=[rRec;sqrt(xRec(i)^2+yRec(i)^2)];
end

%% plot the data
figure(1)
plot(tRec,uRec);
xlabel('time');
ylabel('control u');

figure(2)
plot(tRec,x2Rec);
xlabel('time');
ylabel('x2');
% 
figure(3)
plot(tRec,rRec);
title('r vs time plot')
xlabel('time');
ylabel('rRec');

figure(4)
plot(xRec,yRec);
title(' y/x plot');
xlabel('X');
ylabel('Y');
hold on,
ang=0:0.01:2*pi;
xp=d0*cos(ang)+0;
yp=d0*sin(ang)+0;
plot(yp,xp);
hold off



