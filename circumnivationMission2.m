function [xRec, yRec, thRec, tAccu]=circumnivationMission1(x,y,theta,safe,safe2,lambda,g)

global u;
global v;
global rMesP;
global rMesP2;
global w;

d=10;       dmin=4.5;

% initial condition
aveR=[]; 
k=0.5;  

%% starting point
o_x= x;    % starting x  
o_y= y;    % starting y
o_theta=theta; %atan2(o_y,o_x)+pi;   % theta of vehicle at starting point  
X0=[o_x;o_y;o_theta]; 
rMesP=sqrt((o_x-safe(1))^2+(o_y-safe(2))^2); % r at starting point(r: distance between vehicle and center)
rMesP2=sqrt((o_x-safe2(1))^2+(o_y-safe2(2))^2);


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
n=700;         % n:# of loop, 
dT=0.1;         % dT: time duration,  

rd=10;
rs=sqrt(rd^2-1/k^2);

i=0;


uRec=[];    % control u
x2Rec=[];   % x2 

flag=0;
tRec=[];    % time
tAccu=[0];  % tAccu used for accumulating time


%%

while i<n,
  i=i+1;
  
  figure(1)
  [t X]=ode45(@uavModel,[0 dT],X0);
  pause(0.1);
  
  figure(1)
  plot(X(:,1),X(:,2),'r','linewidth',2); hold on,% plot the roule
  axis([0, 200, 0, 200]);
  
    xRec=[xRec;X(:,1)];
    yRec=[yRec;X(:,2)];
    thRec=[thRec;X(:,3)];
    tRec=[tRec;t+tAccu(end)];
    xMes=X(end,1);
    yMes=X(end,2);
    rMes=sqrt((xMes-safe(1))^2+(yMes-safe(2))^2)+normrnd(0,0.0001);
      
    x2hat= 1/dT*(rMes-rMesP);% create noise
    
    
    rMes2=sqrt((xMes-safe2(1))^2+(yMes-safe2(2))^2)+normrnd(0,0.0001);
    x2hat2=1/dT*(rMes2-rMesP2);
    
    dis=norm([xMes-safe2(1),yMes-safe2(2)]);
    if x2hat2 <= -v*sqrt(dis^2-(0.9*dr)^2)/dis,
        flag=1;
    end
    
    if (x2hat2 > -v*sqrt(dis^2-(0.8*dr)^2)/dis) && (flag==1),
        flag=2;
    end
    
    if flag==g,
        break;
    end
        
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
    rMesP2=rMes2;
 
end

%% collect data of r(distance between vehicle to the center)

for i=1:size(xRec),
    rRec=[rRec;sqrt((xRec(i)-safe(1))^2+(yRec(i)-safe(2))^2)];
end