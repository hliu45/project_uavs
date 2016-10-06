%% parameter
clear all;
global u;
global v;
global rMesP;
global w;

d=10;       dmin=4.5;
wx=0.5;     wy=0.0;    
w=(wx^2+wy^2)^0.5;      % the velocity of wind
% initial condition
aveR=[];                % used for collect average value of error
Kaccum=[];              % used for collect gain k
for i=1:1,
k=3;    
Kaccum=[Kaccum;k];    

%% starting point
o_x= -15;    % starting x  
o_y= -15;    % starting y
o_theta=3*pi/4; %atan2(o_y,o_x)+pi;   % theta of vehicle at starting point  
X0=[o_x;o_y;o_theta]; 
rMesP=sqrt(o_x^2+o_y^2); % r at starting point(r: distance between vehicle and center)

dr=15; %desired radis of orbit

% initial data o x,y,theta,time and r to empty
xRec=[];
yRec=[]; 
thRec=[];
tRec=[];
rRec=[];

v=1;        % velocity
u=v/dr;     % control u


% configuration
n=2000;         % n:# of loop, 
dT=0.4;         % dT: time duration,  
%k=1.3;
lambda=1;
rd=10;
rs=sqrt(rd^2-1/k^2);
%rs=9.95;
%rd=sqrt(rs^2+1/k^2);
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
    rMes=sqrt(xMes^2+yMes^2);
    rMes=rMes;% +normrnd(0,0.5);  
    x2hat= 1/dT*(rMes-rMesP)+normrnd(0,0.5);% create noise
    
    x2Rec=[x2Rec;ones(size(t))*x2hat];
    
    if rMes>rs
        u=-lambda*k*x2hat - lambda*k*v*cos(asin(rs/rMes));
    else
        u=-lambda*k*x2hat +lambda*k*v*cos(asin(rMes/rs));
    end
    
    u=-u;
    
     u=real(u);
     
     if u>1.5,
         u=1.5;
    elseif u< -1.5,
         u=-1.5;
    end

     
    uRec=[uRec;ones(size(t))*u];
    flagFirst=0;

    tAccu=[tAccu;tAccu(end)+t]; %update time
  
    X0(1,1)=X(end,1);   % record the last point
    X0(2,1)=X(end,2);     
    X0(3,1)=X(end,3);
    rMesP=rMes;         % update r (r: distance to the center)
 
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

% %% plot the data
figure(1)
plot(tRec,uRec);
axis([0 600 -1.5 2]);
xlabel('time');
ylabel('control u');
% 
% % figure(2)
% % plot(tRec,x2Rec);
% % xlabel('time');
% % ylabel('x2');
% % 
figure(3)
plot(tRec,rRec);
title('r vs time plot')
xlabel('time');
ylabel('rRec');
% 
figure(4)
plot(xRec,yRec);
axis([-20 20 -20 20]);
title(' y/x plot');
xlabel('X');
ylabel('Y');
hold on,
ang=0:0.01:2*pi;
xp=rd*cos(ang)+0;
yp=rd*sin(ang)+0;
plot(yp,xp);
hold off

% figure(5)
% plot(Kaccum,aveR);
% axis([0 3 0 2.4]);
% xlabel('k');
% ylabel('average value of error');

