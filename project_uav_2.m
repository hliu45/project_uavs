global u;
global v;
v=5;
envir=[0, 200, 0, 200];

radiotower=[30,35,10;
            55,60,10;
            75,140,10;
            155,110,10;
            180,180,10]; % (x,y,r,lambda,k)
        
s_x=20;
s_y=20;
s_th=0;
s_t=0;
        

Xcollect=[s_x];
Ycollect=[s_y];
Thcollect=[s_th];
TMEcollect=[s_t];

lambda=[];
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


for i=1:length(lambda)-1,
   if lambda(i+1)==lambda(i)
       gC=1;
   else
       gC=2;
   end
   g=[g;gC];
end
g=[g;1];
lambda=[1;1;1;1;1];
g=[1;1;1;1;1];

radiotower=[radiotower,lambda,g];

d=10;
dT=0.1;

figure(1)
for i=1:length(radiotower(:,1))
    circle(radiotower(i,1),radiotower(i,2),radiotower(i,3));
end

for i=1:length(radiotower(:,1))
    [xRec,yRec,thRec]=circumnivationMission(Xcollect(end),Ycollect(end),Thcollect(end),...
        radiotower(i,1:3),radiotower(i,4));
    
    Xcollect=[Xcollect;xRec];
    Ycollect=[Ycollect;yRec];
    Thcollect=[Thcollect;thRec];
    TMEcollect=[TMEcollect;tAccu];
    
end

