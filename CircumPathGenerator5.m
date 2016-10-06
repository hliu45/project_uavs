global idx;
global ridx;
global fidx;

start_node = [10,10,0];
end_node   = [160,160,0];
radiotower=[30,80;
            50,130;
            70,90;
            90,30;
            100,70;
            120,40;
            130,150]; 
Rl=length(radiotower(:,1));

height=[0,200];
width=[0,200];
segment=30;
impossible = 1;
flag=0;

figure(1)
axis([width height]); hold on,
for i=1:Rl
    plot(radiotower(i,1),radiotower(i,2),'*');
end
plot(start_node(1),start_node(2),'o');
plot(end_node(1),end_node(2),'o');
hold on,
tree=[start_node];

while impossible > 0 
    randpoint=[(width(2)-width(1))*rand+width(1),(height(2)-height(1))*rand+height(1)];
    minimum=norm([width(2),height(2)]-[width(1),height(2)]);
    for i=1:length(tree(:,1))
        if norm(randpoint-tree(i,1:2))<minimum,
            minimum=norm(randpoint-tree(i,1:2));
            idx=i;
        end
    end
    ang=atan2(randpoint(2)-tree(idx,2),randpoint(1)-tree(idx,1));
    newx=tree(idx,1)+segment*cos(ang);
    newy=tree(idx,2)+segment*sin(ang);
    tree=[tree;newx,newy,idx];
    plot([tree(idx,1),newx],[tree(idx,2),newy],'y');
    
        if norm([newx,newy]-end_node(1:2))<segment+10,
            end_node(3)=length(tree(:,1));
            tree=[tree;end_node];
            flag=1;
            break;
        end
    
    if flag==1,
        break;
    end
end

path=[tree(end,1:2)];
parent=tree(end,3);

while parent > 1
    path=[tree(parent,1:2);path];
    parent=tree(parent,3);
end
path=[start_node(1:2);path];

for i=1:length(path(:,1))-1,
    figure(1)
    plot([path(i,1),path(i+1,1)],[path(i,2),path(i+1,2)],'b','linewidth',2); hold on,
end

circle=[];
for i=1:length(path(:,1)),
   min=norm([width(2),height(2)]-[width(1),height(2)]);
   for j=1:length(radiotower(:,1))
      if norm(radiotower(j,:)-path(i,:))<min 
        min=norm(radiotower(j,:)-path(i,:));
        ridx=j;
      end    
   end
   circle=[circle;ridx,min];
end

%% pruning circle
i=1; j=1;
use_circle=[];
while j~=length(circle(:,1)),
   while circle(i,1)==circle(j,1)
       j=j+1;
       if j==length(circle(:,1))
            break;
       end
   end
   use_circle=[use_circle;circle(j,:)];
   i=j;
end
f_circle1=[]; % first pruning
if use_circle(end,1)==use_circle(end-1,1)
    f_circle1=[use_circle(1:end-2,:)];
    f_circle1=[f_circle1;use_circle(end,:)];
end


f_circle2=f_circle1; % second pruning
for i=1:length(f_circle1(:,1))-1,
    for j=i+1:length(f_circle1(:,1)),
        if f_circle1(i,1)==f_circle1(j,1)
            f_circle2(i,1)=0;
        end
    end
end

f_circle3=[];
for i=1:length(f_circle2(:,1))
    if f_circle2(i,1)~=0;
        f_circle3=[f_circle3;f_circle2(i,:)];
    end
end

f_circle4=f_circle3;
for i=1:length(f_circle3(:,1))-1,
    for j=i+1:length(f_circle3(:,1)),
        if norm(radiotower(f_circle3(i,1),:)- radiotower(f_circle3(j,1),:))<f_circle3(i,2)+f_circle3(j,2)
            f_circle4(i,:)=0;
        end
    end
end

f_circle=[];
for i=1:length(f_circle4(:,1))
    if f_circle4(i,1)~=0;
        f_circle=[f_circle;f_circle4(i,:)];
    end
end


%% plot circle
for i=1:length(f_circle(:,1)),
    angle=0:0.01:2*pi; 
    xp=f_circle(i,2)*cos(angle);
    yp=f_circle(i,2)*sin(angle);
    plot(radiotower(f_circle(i,1),1)+xp,radiotower(f_circle(i,1),2)+yp,'c');
    hold on,
end

lambdacollect=[-1];
for i=1:length(f_circle(:,1))-1,
    lambdacollect=[lambdacollect;-1];
end

%% simulation
s_th=0;
s_t=0;
        
Xcollect=[start_node(1)];
Ycollect=[start_node(2)];
Thcollect=[s_th];
TMEcollect=[s_t];

for i=1:length(f_circle(:,1))-1
    while norm([Xcollect(end)-radiotower(f_circle(i,1),1),Ycollect(end)-radiotower(f_circle(i,1),2)])> f_circle(i,2)+8
        [xRec,yRec,thRec,tAccu]=circumnivationMission3(Xcollect(end),Ycollect(end),Thcollect(end),...
            radiotower(f_circle(i,1),1:2),f_circle(i,2),radiotower(f_circle(i+1,1),:),lambdacollect(i));
    
        Xcollect=[Xcollect;xRec];
        Ycollect=[Ycollect;yRec];
        Thcollect=[Thcollect;thRec];
        TMEcollect=[TMEcollect;tAccu];
    end
end

[xRec, yRec, thRec, tAccu]=circumnivationMission6(Xcollect(end),Ycollect(end),Thcollect(end),...
    radiotower(f_circle(end,1),:),f_circle(end,2),end_node(1:2),lambdacollect(end));
Xcollect=[Xcollect;xRec];
Ycollect=[Ycollect;yRec];
Thcollect=[Thcollect;thRec];
TMEcollect=[TMEcollect;tAccu];







