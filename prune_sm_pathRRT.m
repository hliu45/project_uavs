%% pathRRT
%%  - create a path from a start node to an end node
%%    using the RRT algorithm.
%%  - RRT = Rapidly-exploring Random Tree
%% 

 
function wayPoint= prune_sm_pathRRT
 
% create random world
Size = 200;
NumObstacles = 10;

s=[20,20];
e=[180,180];

% randomly select start and end nodes
start_node = [s(1),s(2),0,0,0];
end_node   = [e(1),e(2),0,0,0];

world = createWorld(NumObstacles,[Size; Size],[0;0],start_node,end_node);
 
% standard length of path segments
segmentLength = 5;

% establish tree starting with the start node
tree = start_node;
 
% check to see if start_node connects directly to end_node
if ( (norm(start_node(1:2)-end_node(1:2))<segmentLength )...
    &(collision(start_node,end_node,world)==0) )
  path = [start_node; end_node];
else
  numPaths = 0;
  while numPaths<1,
      [tree,flag] = extendTree(tree,end_node,segmentLength,world);
      %canEndConnectToTree(tree,end_node,1,world);
      numPaths = numPaths + flag;
  end
end
 
% find path with minimum cost to end_node
path = findMinimumPath(tree,end_node);
WP=plotWorld(world,path,tree,start_node,end_node);
wayPoint=WP;
disp('wayPoint:');
disp(wayPoint);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% createWorld
%%  - create random world with obstacles
%%  the first element is the north coordinate
%%  the second element is the south coordinate
function world = createWorld(NumObstacles, NEcorner, SWcorner,start_node,end_node);
  fprintf('Ready to creatWorld\n');
  % check to make sure that the region is nonempty
  if (NEcorner(1) <= SWcorner(1)) | (NEcorner(2) <= SWcorner(2)),
      disp('Not valid corner specifications!')
      world=[];
       
  % create world data structure
  else
    world.NumObstacles = NumObstacles;
    world.NEcorner = NEcorner;
    world.SWcorner = SWcorner;
                           
    % create NumObstacles
    maxRadius = min(NEcorner(1)- SWcorner(1), NEcorner(2)-SWcorner(2));
    maxRadius = (maxRadius/NumObstacles)/5;
    for i=1:NumObstacles-2,
        % randomly pick radius
            world.radius(i) = maxRadius*rand;
            world.radius(NumObstacles)=5;
            world.radius(NumObstacles-1)=3;
            % randomly pick center of obstacles
            cn = SWcorner(1) + world.radius(i)...
                + (NEcorner(1)-SWcorner(1)-2*world.radius(i))*rand;
            ce = SWcorner(2) + world.radius(i)...
                + (NEcorner(2)-SWcorner(2)-2*world.radius(i))*rand;
            %world.cn(i) = cn;
            %world.ce(i) = ce;
            if ((norm([cn,ce]-[start_node(1),start_node(2)])>(world.radius(i)+5))...
                & (norm([cn,ce]-[end_node(1),end_node(2)])>(world.radius(i)+5))),
                world.cn(i) = cn;
                world.ce(i) = ce;
                world.cn(NumObstacles)=80;
                world.ce(NumObstacles)=50;
                world.cn(NumObstacles-1)=150;
                world.ce(NumObstacles-1)=160;
            else
                world.radius(i) = 0;
                world.cn(i) = 0;
                world.ce(i) = 0;
                fprintf('obstacle invalid');
            end
            
    end
  end
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% generateRandomNode
%%   create a random node (initialize)
function node=generateRandomNode(world);
 
% randomly pick configuration
pn       = (world.NEcorner(1)-world.SWcorner(1))*rand;
pe       = (world.NEcorner(2)-world.SWcorner(2))*rand;
chi      = 0;
cost     = 0;
node     = [pn, pe, chi, cost, 0];
 
% check collision with obstacle
while collision(node, node, world),
  pn       = (world.NEcorner(1)-world.SWcorner(1))*rand;
  pe       = (world.NEcorner(2)-world.SWcorner(2))*rand;
  chi      = 0;
  cost     = 0;
  node     = [pn, pe, chi, cost, 0];
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% collision
%%   check to see if a node is in collsion with obstacles
function collision_flag = collision(node, parent, world);
fprintf('checking collision\n');

collision_flag = 0;
 
if ((node(1)>world.NEcorner(1))...
    | (node(1)<world.SWcorner(1))...
    | (node(2)>world.NEcorner(2))...
    | (node(2)<world.SWcorner(2))),
        collision_flag = 1;
else
    for sigma = 0:.2:1,
        p = sigma*node(1:2) + (1-sigma)*parent(1:2);
        % check each obstacle
        for i=1:world.NumObstacles,
         if (norm([p(1);p(2)]-[world.cn(i); world.ce(i)])<=1.5*world.radius(i)),
            collision_flag = 1;
            break;
         end
      end
    end
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% collision_pru
%%   check to see if a node is in collsion with obstacles
function collision_flag = collision_pru(node, parent, world);
collision_flag=0;
% check each obstacle
a=parent(2)-node(2);
b=node(1)-parent(1);
c=node(2)*parent(1)-parent(2)*node(1);
r=a^2+b^2;
    for i=1:world.NumObstacles,
          x=(b*(b*world.cn(i)-a*world.ce(i))-a*c)/r; % the closest point(x,y) to the obstacle on the line
          y=(a*(-b*world.cn(i)+a*world.ce(i))-b*c)/r;% the closest point(x,y) to the obstacle on the line
          d=abs(a*world.cn(i)+b*world.ce(i)+c)/r^0.5; % distance between the center of obstacle and the line
        if parent(1)>=node(1)
            if (x>=node(1) && x<=parent(1)),
                if d<=((world.radius(i))+3),
                    collision_flag = 1;
                break;
                end
            end
        else
            if (x>=parent(1) && x<=node(1)),
                if d<=((world.radius(i))+3),
                    collision_flag = 1;
                break;
                end
            end
        end
      end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% canEndConnectToTree
%%   check to see if the end node can connect to the tree
%function flag = canEndConnectToTree(tree,end_node,minDist,world);
%  flag = 0;
  % check only last node added to tree since others have been checked
%  if ( (norm(tree(end,1:2)-end_node(1:2))<minDist)...
%     & (collision(tree(end,1:2), end_node(1:2), world)==0) ),
%    flag = 1;
%  end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% extendTree
%%   extend tree by randomly selecting point and growing tree toward that
%%   point
function [new_tree,flag] = extendTree(tree,end_node,segmentLength,world);
  fprintf('extendTree...\n');
  flag1 = 0;
  while flag1==0,
    % select a random point
    if (rand>0.01 & rand<0.99)
        randomPoint = [...
        (world.NEcorner(1)-world.SWcorner(1))*rand,...
        (world.NEcorner(2)-world.SWcorner(2))*rand];
    else
        randomPoint=[end_node(1),end_node(2)];
    end
    
    % find leaf on node that is closest to randomPoint
    tmp = tree(:,1:2)-ones(size(tree,1),1)*randomPoint; % times ones is to make sure the same dimention, because tree maybe has n rows
    [dist,idx] = min(diag(tmp*tmp')); 
    cost     = tree(idx,4) + segmentLength;
    new_point = (randomPoint-tree(idx,1:2));
    new_point = tree(idx,1:2)+new_point/norm(new_point)*segmentLength;
    new_node = [new_point, 0, cost, idx];
    if collision(new_node, tree(idx,:), world)==0,
      new_tree = [tree; new_node];
      flag1=1;
    end
  end
   
  % check to see if new node connects directly to end_node
  if ( (norm(new_node(1:2)-end_node(1:2))<segmentLength )...
      &(collision(new_node,end_node,world)==0) )
    flag = 1;
    new_tree(end,3)=1;  % mark node as connecting to end.
  else
    flag = 0;
  end
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% findMinimumPath
%%   find the lowest cost path to the end node
function path = findMinimumPath(tree,end_node);
    fprintf('find minimum path...\n'); 
    % find nodes that connect to end_node
    connectingNodes = [];
    for i=1:size(tree,1),
        if tree(i,3)==1,
            connectingNodes = [connectingNodes; tree(i,:)];
            disp(connectingNodes);
            disp(tree);
        end
    end
 
    % find minimum cost last node
    %disp(connectingNodes);
    [tmp,idx] = min(connectingNodes(:,4));
    fprintf('tmp=%d,idx=%d\n',tmp,idx);
    
    % construct lowest cost path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,5);
    while parent_node>=1,
        path = [tree(parent_node,:); path];
        parent_node = tree(parent_node,5);
    end
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotWorld
%%   plot obstacles and path
function WP=plotWorld(world,path,tree,start_node,end_node)
  fprintf('ploting...\n');
  % the first element is the north coordinate
  % the second element is the south coordinate
  disp(start_node);
  disp(end_node);
  N = 10;
  th = 0:2*pi/N:2*pi;
  figure(1), clf
  axis([world.SWcorner(1),world.NEcorner(1),...
      world.SWcorner(2), world.NEcorner(2)]);
  hold on
   
  %for i=1:world.NumObstacles,
  %    X = world.radius(i)*cos(th) + world.cn(i);
  %    Y = world.radius(i)*sin(th) + world.ce(i);
  %    fill(X,Y,'k');
  %end
  %for i=2:size(tree,1); % tree plotting
  %    plot([tree(tree(i,5),1), tree(i,1)], [tree(tree(i,5),2), tree(i,2)])
  %end
  X = path(:,1);
  Y = path(:,2);
  [n,m]=size(path);
  cp=[,];  %control point 
  for i=1:n-1,
    cp0=[path(i,1)+(path(i+1,1)-path(i,1))/4,path(i,2)+(path(i+1,2)-path(i,2))/4];
    cp1=[path(i,1)+(path(i+1,1)-path(i,1))/2,path(i,2)+(path(i+1,2)-path(i,2))/2];
    cp2=[path(i,1)+(path(i+1,1)-path(i,1))*3/4,path(i,2)+(path(i+1,2)-path(i,2))*3/4];
    cp=[cp;cp0;cp1;cp2];
  end
  B=[,]; %Cubic Bezier curve point
  [nc,mc]=size(cp);
  for j=2:3:nc-2,
    for t=0:0.01:1,
        B1=[(1-t)^3*cp(j,1)+3*(1-t)^2*t*cp(j+1,1)+3*(1-t)*t^2*cp(j+2,1)+t^3*cp(j+3,1),...
            (1-t)^3*cp(j,2)+3*(1-t)^2*t*cp(j+1,2)+3*(1-t)*t^2*cp(j+2,2)+t^3*cp(j+3,2)];
        B=[B;B1];
    end
  end
  %% path pruning
    pru=path(n,:);  %pru represent the pruned path
    j=n;
    p=n-1;
    while j>2
        if (collision_pru(path(p,:),path(j,:),world)==1 | p==1),
            if p > 1,
                pru=[pru;path(p+1,:)];
            end
            j=p+1;
        end
        if p>1,
            p=p-1;
        end
    end 
    pru=[pru;path(1,:)];
    fprintf('pru:');
    disp(pru);
  %% pruning path Cubic Bezier curve point
  [pn,pm]=size(pru);
  pcp=[,];  %control point 
  for i=1:pn-1,
    pcp0=[pru(i,1)+(pru(i+1,1)-pru(i,1))/4,pru(i,2)+(pru(i+1,2)-pru(i,2))/4];
    pcp1=[pru(i,1)+(pru(i+1,1)-pru(i,1))/2,pru(i,2)+(pru(i+1,2)-pru(i,2))/2];
    pcp2=[pru(i,1)+(pru(i+1,1)-pru(i,1))*3/4,pru(i,2)+(pru(i+1,2)-pru(i,2))*3/4];
    pcp=[pcp;pcp0;pcp1;pcp2];
  end
  pB=[,]; %Cubic Bezier curve point
  WP=[end_node(1),end_node(2)];
  [pnc,pmc]=size(pcp);
  if pnc>4
    for j=2:3:pnc-2,
        for t=0:0.01:1,
            pB1=[(1-t)^3*pcp(j,1)+3*(1-t)^2*t*pcp(j+1,1)+3*(1-t)*t^2*pcp(j+2,1)+t^3*pcp(j+3,1),...
                (1-t)^3*pcp(j,2)+3*(1-t)^2*t*pcp(j+1,2)+3*(1-t)*t^2*pcp(j+2,2)+t^3*pcp(j+3,2)];
            if t==0.25 | t==0.5 | t==0.75,
                WP=[WP;pB1]; %WP: waypoint
            end
            pB=[pB;pB1];
        end
    end
  else
      for t=0:0.1:1,
          pB1=[pru(2,1)-pru(1,1),pru(2,2)-pru(1,2)]*t+[pru(1,1),pru(1,2)];
          pB=[pB;pB1];
           if t==0.5,
                WP=[WP;pB1]; %WP: waypoint
           end
      end
  end
  WP=[WP;[start_node(1),start_node(2)]];
  %%
  cpx=cp(:,1);  %control point x coordinate
  cpy=cp(:,2);  %control point y coordinate
  BX=B(:,1);    %cubic bezier curve x coordinate
  BY=B(:,2);    %cubic bezier curve y coordinate
  pBX=pB(:,1);
  pBY=pB(:,2);
  disp('pB:');
  disp(pB);
  
  PX=pru(:,1);
  PY=pru(:,2);
  plot(PX,PY,'b','linewidth',2);
  plot(BX,BY,'b','linewidth',2);
  plot(pBX,pBY,'m','linewidth',2);
  %
  plot([pcp(pnc-1,1),start_node(1)],[pcp(pnc-1,2),start_node(2)],'b','linewidth',2);
  plot([pcp(2,1),end_node(1)],[pcp(2,2),end_node(2)],'b','linewidth',2);
  %
  %plot(cpx,cpy,'o');
  plot(X,Y,'r','linewidth',1.5);
  plot(start_node(1),start_node(2),'s');
  plot(end_node(1),end_node(2),'o');
  %display(pru);
  
  nwaypoint=length(WP);
  for i=1:nwaypoint,
        plot(WP(i,1),WP(i,2),'r*');
  end
  WP=WP;
  
  