global u;
global v;
v=5;
envir=[0, 200, 0, 200]; % the boundary of the environment

%% location of each radio tower
radiotower=[30,35,10;
            55,60,10;
            75,140,10;
            155,110,10;
            180,180,10]; % (x,y,r)
        
%% initial position
s_x=20;
s_y=20;
s_th=0;
s_t=0;
        

Xcollect=[s_x];
Ycollect=[s_y];
Thcollect=[s_th];
TMEcollect=[s_t];

for i=1:length(radiotower(:,1))
   circle(radiotower(i,1),radiotower(i,2),radiotower(i,3)); 
end





%% lambda signal where it do CW or CCW
lambda=[];
%% g is used for dubins paths, g=1 meaning it do either L-S-L or R-S-R, g=2, meaing it do R-S-L or L-S-R
g=[];

if radiotower(2,1)*(radiotower(1,2)-s_y)-radiotower(2,2)*(radiotower(1,1)-s_x) >=...
    radiotower(1,1)*(radiotower(1,2)-s_y)-radiotower(1,2)*(radiotower(1,1)-s_x),
    
    lambdaC=1;
else
    lambdaC=-1;
end
lambda=[lambda;lambdaC];

for i=2:length(radiotower(:,1))-1,
    if radiotower(i+1,1)*(radiotower(i,2)-radiotower(i-1,2))-radiotower(i+1,2)*(radiotower(i,1)-radiotower(i-1,1)) >=...
    radiotower(i,1)*(radiotower(i,2)-radiotower(i-1,2))-radiotower(i,2)*(radiotower(i,1)-radiotower(i-1,1)),
    
        lambdaC=1;
    else
        lambdaC=-1;
    end
    lambda=[lambda;lambdaC];
end

lambda=[lambda;1];
lambda=[-1;-1;-1;-1;-1];


for i=1:length(lambda)-1,
   if lambda(i+1)==lambda(i)
       gC=1;
   else
       gC=2;
   end
   g=[g;gC];
end
g=[g;1];
%g=[2;1;2;2;1];

radiotower=[radiotower,lambda,g];

%% desired orbit
d=10;
%% sample time
dT=0.1;

% figure(1)
% for i=1:length(radiotower(:,1))
%     circle(radiotower(i,1),radiotower(i,2),radiotower(i,3));
% end

for i=1:length(radiotower(:,1))-1
    [xRec,yRec,thRec,tAccu]=circumnivationMission1(Xcollect(end),Ycollect(end),Thcollect(end),...
        radiotower(i,1:3),radiotower(i+1,1:3),radiotower(i,4),radiotower(i,5));
    
    Xcollect=[Xcollect;xRec];
    Ycollect=[Ycollect;yRec];
    Thcollect=[Thcollect;thRec];
    TMEcollect=[TMEcollect;tAccu];
    
    if i~=length(radiotower)-1
        while norm([Xcollect(end)-radiotower(i+1,1),Ycollect(end)-radiotower(i+1,2)])> 1.3*d+6 
            u=0;
            X0=[Xcollect(end);Ycollect(end);Thcollect(end)];
            [t X]=ode45(@uavModel,[0 dT],X0);
        
            pause(0.1);
            figure(1)
            plot(X(:,1),X(:,2),'r','linewidth',2); hold on,% plot the roule
            axis(envir);
        
            Xcollect=[Xcollect;X(:,1)];
            Ycollect=[Ycollect;X(:,2)];
            Thcollect=[Thcollect;X(:,3)];
            TMEcollect=[TMEcollect;t+TMEcollect(end)];
        end
    end
end


[xRec, yRec, thRec]=circumnivation2(Xcollect(end),Ycollect(end),Thcollect(end),radiotower(end,1:3),radiotower(end,5));
Xcollect=[Xcollect;xRec];
Ycollect=[Ycollect;yRec];
Thcollect=[Thcollect;thRec];
