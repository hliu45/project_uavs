%% parameter
clear all;
global u;
global v;
global rMesP;

c1=0.6;     c3=5;
c2=15;      c4=0;
d=10;       dmin=4;

% initial condition
o_x=    100;    % starting x  
o_y=    250;    % starting y
delta=0.5;
xRec=[];
yRec=[]; 
thRec=[];
tRec=[];
rRec=[];
v=1.2;      % velocity
u=0.5;
o_theta=[atan2(o_y,o_x)+pi];   % theta of vehicle at starting point  
X0=[o_x;o_y;o_theta]; 
rMesP=sqrt(o_x^2+o_y^2); % r at starting point(r: distance between vehicle and center)

% configuration
n=1000;         % n:# of loop, 
dT=0.2;         % dT: time duration,  
i=0;
mode = 0; %(control mode 0: original law; 1: control candidate 1)

uRec=[];    % control u
x2Rec=[];   % x2 

flagFirst=1;
tRec=[];    % time
tAccu=[0];  % tAccu used for accumulating time


%%


j=0;
while rMesP>5,
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
    
    fprintf('r:%d, x2:%d, i:%d\n',rMes,x2hat,j);
    
        
            u=delta/(rMes - 1.5*x2hat);
        
        
    
    uRec=[uRec;ones(size(t))*u];
    
    tAccu=[tAccu;tAccu(end)+t]; %update time
    X0(1,1)=X(end,1);         % record the last point
    X0(2,1)=X(end,2);     
    X0(3,1)=X(end,3);
    rMesP=rMes;
    
    j=j+1;
    if j> 6000,
        break;
    end
end

% while i<n,
%   i=i+1;
%   
%   [t X]=ode45(@uavModel,[0 dT],X0);    
%  
%     xRec=[xRec;X(:,1)];
%     yRec=[yRec;X(:,2)];
%     thRec=[thRec;X(:,3)];
%     tRec=[tRec;t+tAccu(end)];
%     xMes=X(end,1);
%     yMes=X(end,2);
%     rMes=sqrt(xMes^2+yMes^2);
%     x2hat= 1/dT*(rMes-rMesP);
    
%     x2Rec=[x2Rec;ones(size(t))*x2hat];
% 
%     if mode==0,
%     % original control law
%         u=(1/sqrt(v^2-x2hat^2))*(c1*(rMes-d)/c2 + (v^2-x2hat^2)/rMes...
%          +c3*x2hat); 
%      % control law candidate 1
%     elseif mode ==1,
%         u=(1/sqrt(v^2-(x2hat+w)^2))*(c1*(rMes-d)/c2 + (v^2-x2hat^2-w^2)/rMes...
%           + 2*w^2/rMes + 2*w*sqrt(v^2-(x2hat-w)^2)/rMes + c3*x2hat);
%     else
%         u=0.2;
%     end
%     
%      u=real(u);
     
%      % u is bounded between -v/dmin to v/dmin
%      if u>(v/dmin)
%          u=v/dmin;
%      elseif u<-(v/dmin)
%          u=-v/dmin;
%      end
% %      %
%      
%     uRec=[uRec;ones(size(t))*u];
%     flagFirst=0;
% 
%     tAccu=[tAccu;tAccu(end)+t]; %update time
%   
%   X0(1,1)=X(end,1);         % record the last point
%   X0(2,1)=X(end,2);     
%   X0(3,1)=X(end,3);
%   rMesP=rMes;               % update r (r: distance to the center)
%  
% end

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

figure(3)
plot(tRec,x2Rec);
xlabel('time');
ylabel('x2');

figure(4)
plot(tRec,rRec);
title('r vs time plot')
xlabel('time');
ylabel('rRec');

