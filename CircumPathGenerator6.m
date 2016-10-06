global idx;
global ridx;
global fidx;

start_node = [10,10,0,0,0,0,0,0];
end_node   = [160,160,0,0,0,0,0,0];
radiotower=[30,80;
            50,130;
            70,90;
            90,30;
            100,70;
            120,40;
            130,130;
            130,110]; 
Rl=length(radiotower(:,1));

height=[-30,200];
width=[1-30,200];
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

Ctrand=[];
Clrand=[];
Cdrand=[];

Xcollect=[];
Ycollect=[];
Thcollect=[];
TMEcollect=[];


while norm([tree(end,1),tree(end,2)]-[end_node(1),end_node(2)])>28
%%
trand=round(length(radiotower(:,1))*rand());
if trand==0,
    trand=8;
end

if round(rand())==0
    lrand=-1;
else
    lrand=1;
end

drand=round(210*rand())+10;

Ctrand=[Ctrand;trand];
Clrand=[Clrand;lrand];
Cdrand=[Cdrand;drand];
%%
idx=1;
min=norm([end_node(1),end_node(2)]-[start_node(1),start_node(2)]);
for i=1:length(tree(:,1)),
   if norm([end_node(1),end_node(2)]-[tree(i,1),tree(i,2)]) < min,
      min=norm([end_node(1),end_node(2)]-[tree(i,1),tree(i,2)]);
      idx=i;
   end
end

[xRec, yRec, thRec, tAccu]=circumnivationMission7(tree(idx,1),tree(idx,2),tree(idx,3),...
    radiotower(trand,:),drand,lrand);
% Xcollect=[Xcollect;xRec];
% Ycollect=[Ycollect;yRec];
% Thcollect=[Thcollect;thRec];
% TMEcollect=[TMEcollect;tAccu];

%tree(end,6:8)=[trand,lrand,drand]
tree=[tree;xRec(end),yRec(end),thRec(end),tAccu(end),idx,trand,lrand,drand];

plot(xRec(end),yRec(end),'o');

end
[xRec, yRec, thRec, tAccu]=circumnivationMission10(tree(end,1),tree(end,2),tree(end,3),...
    radiotower(end-1,:),42.4624,-1);


path=[tree(end,:)];
parent=tree(end,5);
while parent > 0,
    path=[tree(parent,:);path];
    parent=tree(parent,5);
end
%path=[tree(1,:);path];

for i=1:length(path(:,1))-1
    [xRec, yRec, thRec, tAccu]=circumnivationMission9(path(i,1),path(i,2),path(i,3),...
    radiotower(path(i+1,6),:),path(i+1,8),path(i+1,7));
end

[xRec, yRec, thRec, tAccu]=circumnivationMission9(tree(end,1),tree(end,2),tree(end,3),...
    radiotower(end-1,:),42.4624,-1);
[xRec, yRec, thRec, tAccu]=circumnivationMission9(xRec(end),yRec(end),thRec(end),...
    radiotower(end-1,:),42.4624,-1);
[xRec, yRec, thRec, tAccu]=circumnivationMission9(xRec(end),yRec(end),thRec(end),...
    radiotower(end-1,:),42.4624,-1);
[xRec, yRec, thRec, tAccu]=circumnivationMission9(xRec(end),yRec(end),thRec(end),...
    radiotower(end-1,:),42.4624,-1);
[xRec, yRec, thRec, tAccu]=circumnivationMission9(xRec(end),yRec(end),thRec(end),...
    radiotower(end-1,:),42.4624,-1);
[xRec, yRec, thRec, tAccu]=circumnivationMission9(xRec(end),yRec(end),thRec(end),...
    radiotower(end-1,:),42.4624,-1);

