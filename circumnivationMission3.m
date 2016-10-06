function [xRec, yRec, thRec,tAccu]=circumnivationMission3(x,y,theta,safe,d,next,lambda)

global u;
global v;
global rMesP;
global rnMesp;
global w;

dmin=4.5;

% initial condition
aveR=[]; 
k=0.5;  

%% starting point
o_x= x;    % starting x  
o_y= y;    % starting y
o_theta=theta; %atan2(o_y,o_x)+pi;   % theta of vehicle at starting point  
X0=[o_x;o_y;o_theta]; 
rMesP=sqrt((o_x-safe(1))^2+(o_y-safe(2))^2); % r at starting point(r: distance between vehicle and center)
rnMesP=sqrt((o_x-next(1))^2+(o_y-next(2))^2);

dr=15; %desired radis of orbit

% initial data o x,y,theta,time and r to empty
xRec=[];
yRec=[]; 
thRec=[];
tRec=[];
rRec=[];

v=5;        % velocity
u=v/dr;     % control u

% configuration
n=680;         % n:# of loop, 
dT=0.1;         % dT: time duration,  
%k=1.3;
rd=d;
rs=sqrt(rd^2-1/k^2);
%rs=9.95;
%rd=sqrt(rs^2+1/k^2);
i=0;
%mode = 0; %(control mode 0: original law; 1: control candidate 1)

uRec=[];    % control u
x2Rec=[];   % x2 

flagFirst=1;
tRec=[];    % time
tAccu=[0];  % tAccu used for accumulating time


%%

while i<n,
  i=i+1;
  
  figure(1)
  
  [t X]=ode45(@uavModel,[0 dT],X0);
  pause(0.1);
 
  plot(X(:,1),X(:,2),'r','linewidth',2); % plot the roule 
    xRec=[xRec;X(:,1)];
    yRec=[yRec;X(:,2)];
    thRec=[thRec;X(:,3)];
    tRec=[tRec;t+tAccu(end)];
    xMes=X(end,1);
    yMes=X(end,2);
    rMes=sqrt((xMes-safe(1))^2+(yMes-safe(2))^2);
    rMes=rMes;
    rnMes=sqrt((xMes-next(1))^2+(yMes-next(2))^2);
    x2hat= 1/dT*(rMes-rMesP);
    x2nhat= 1/dT*(rnMes-rnMesp); % radius changing rate to the next point
    
    x2Rec=[x2Rec;ones(size(t))*x2hat];
    
    if rMes>rs
        u=-lambda*k*x2hat - lambda*k*v*cos(asin(rs/rMes));
    else
        u=-lambda*k*x2hat +lambda*k*v*cos(asin(rMes/rs));
    end
    
    u=-u;
    u=real(u);
    uRec=[uRec;ones(size(t))*u];
    tAccu=[tAccu;tAccu(end)+t]; %update time
    X0(1,1)=X(end,1);   % record the last point
    X0(2,1)=X(end,2);     
    X0(3,1)=X(end,3);
    rMesP=rMes;         % update r (r: distance to the center)
    rnMesp=rnMes;
 
    disp(x2nhat);
    if x2nhat < -v*0.9
        break;
    end
    
end

%% collect data of r(distance between vehicle to the center)

for i=1:size(xRec),
    rRec=[rRec;sqrt((xRec(i)-safe(1))^2+(yRec(i)-safe(2))^2)];
end