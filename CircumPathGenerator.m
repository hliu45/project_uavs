%% pathRRT
%%  - create a path from a start node to an end node
%%    using the RRT algorithm.
%%  - RRT = Rapidly-exploring Random Tree

%function CircumPathGenerator
 
global idx;
% randomly select start and end nodes
start_node = [10,10,0,0,0];
end_node   = [160,160,0,0,0];
radiotower=[25,40;
            50,75;
            10,120;
            40,110;
            130,140]; 
Rl=length(radiotower(:,1));
        
DelT = 15;
v=1;

tree = [start_node(1:2),0,0,0,0];
%%
figure(1)
axis([-20 180 -20 180]);
hold on,
for i=1:Rl
    plot(radiotower(i,1),radiotower(i,2),'*');
end
plot(start_node(1),start_node(2),'o');
plot(end_node(1),end_node(2),'o');
hold on,

%%
minimum = norm(end_node(1:2)-start_node(1:2)); 
while minimum >38
rn=round(rand(1)*Rl);
if rn==0,
    rn=1;
end
if rand(1)>0.5
    lambda=1;
else
    lambda=-1;
end

idx=1;
minimum=norm(end_node(1:2)-start_node(1:2));
for i=1:length(tree(:,1))
    if norm(end_node(1:2)-tree(i,1:2))< minimum;
        minimum=norm(end_node(1:2)-tree(i,1:2));
        idx=i;
    end
end

 r_o=norm(radiotower(rn,1:2)-tree(idx,1:2));

 del_theta = v*DelT/r_o;
if length(tree(:,1))<3 || idx==1, 
  if lambda==1,
    theta=atan2(tree(idx,2)-radiotower(rn,2),tree(idx,1)-radiotower(rn,1));
    newx=r_o*cos(theta-del_theta)+radiotower(rn,1);
    newy=r_o*sin(theta-del_theta)+radiotower(rn,2);
        
    tree=[tree;newx,newy,tree(idx,1:2),rn,lambda];
    
    
    cir_x=[];
    cir_y=[];
    for ang=theta:-0.005:theta-del_theta,
        cirx=r_o*cos(ang)+radiotower(rn,1);
        ciry=r_o*sin(ang)+radiotower(rn,2);
        cir_x=[cir_x;cirx];
        cir_y=[cir_y;ciry];      
    end
    figure(1),
    plot(cir_x,cir_y,'r'); hold on,

 else
    theta=atan2(tree(idx,2)-radiotower(rn,2),tree(idx,1)-radiotower(rn,1));
    newx=r_o*cos(theta+del_theta)+radiotower(rn,1);
    newy=r_o*sin(theta+del_theta)+radiotower(rn,2);
    
    tree=[tree;newx,newy,tree(idx,1:2),rn,-1];
    
    cir_x=[];
    cir_y=[];
    for ang=theta:0.005:theta+del_theta,
        cirx=r_o*cos(ang)+radiotower(rn,1);
        ciry=r_o*sin(ang)+radiotower(rn,2);
        cir_x=[cir_x;cirx];
        cir_y=[cir_y;ciry];
    end
    figure(1),
    plot(cir_x,cir_y,'g'); hold on,

 end
else
 if lambda==1,
    theta=atan2(tree(idx,2)-radiotower(rn,2),tree(idx,1)-radiotower(rn,1));
    newx=r_o*cos(theta-del_theta)+radiotower(rn,1);
    newy=r_o*sin(theta-del_theta)+radiotower(rn,2);
    
    test1=(tree(idx,3)*(tree(idx,2)-radiotower(tree(idx,5),2))-tree(idx,4)*(tree(idx,1)-radiotower(tree(idx,5),1))-...
        tree(idx,1)*(tree(idx,2)-radiotower(tree(idx,5),2))+tree(idx,2)*(tree(idx,1)-radiotower(tree(idx,5),1)));
    test2=(newx*(tree(idx,2)-radiotower(tree(idx,5),2))-newy*(tree(idx,1)-radiotower(tree(idx,5),1))-...
        tree(idx,1)*(tree(idx,2)-radiotower(tree(idx,5),2))+tree(idx,2)*(tree(idx,1)-radiotower(tree(idx,5),1))); 
        
    if test1<0,
        test2=test2-5;
    else
        test2=test2+5;
    end
    
    if test1*test2<0,
    tree=[tree;newx,newy,tree(idx,1:2),rn,lambda];
    
    
    cir_x=[];
    cir_y=[];
    for ang=theta:-0.005:theta-del_theta,
        cirx=r_o*cos(ang)+radiotower(rn,1);
        ciry=r_o*sin(ang)+radiotower(rn,2);
        cir_x=[cir_x;cirx];
        cir_y=[cir_y;ciry];      
    end
    figure(1),
    plot(cir_x,cir_y,'r'); hold on,
    end
    
 else
    theta=atan2(tree(idx,2)-radiotower(rn,2),tree(idx,1)-radiotower(rn,1));
    newx=r_o*cos(theta+del_theta)+radiotower(rn,1);
    newy=r_o*sin(theta+del_theta)+radiotower(rn,2);
    
    test1=(tree(idx,3)*(tree(idx,2)-radiotower(tree(idx,5),2))-tree(idx,4)*(tree(idx,1)-radiotower(tree(idx,5),1))-...
        tree(idx,1)*(tree(idx,2)-radiotower(tree(idx,5),2))+tree(idx,2)*(tree(idx,1)-radiotower(tree(idx,5),1)));
    test2=(newx*(tree(idx,2)-radiotower(tree(idx,5),2))-newy*(tree(idx,1)-radiotower(tree(idx,5),1))-...
        tree(idx,1)*(tree(idx,2)-radiotower(tree(idx,5),2))+tree(idx,2)*(tree(idx,1)-radiotower(tree(idx,5),1)));   
     
    if test1<0,
        test2=test2-5;
    else
        test2=test2+5;
    end
    
    if test1*test2<0,
    tree=[tree;newx,newy,tree(idx,1:2),rn,-1];
    
    cir_x=[];
    cir_y=[];
    for ang=theta:0.005:theta+del_theta,
        cirx=r_o*cos(ang)+radiotower(rn,1);
        ciry=r_o*sin(ang)+radiotower(rn,2);
        cir_x=[cir_x;cirx];
        cir_y=[cir_y;ciry];
    end
    figure(1),
    plot(cir_x,cir_y,'g'); hold on,
    end
 end
end
end

path=[];
parent=tree(end,3:6);
path=[parent];
while parent(1:2) ~= start_node(1:2)
for i=1:length(tree(:,1)),
   if parent(1:2)==tree(i,1:2),
      parent=tree(i,3:6);
      path=[parent;path];
   end
end
end


hold off,
disp(tree)




