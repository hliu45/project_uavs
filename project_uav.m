%   The mission is to go from point s(start node) to e(end node), but if it enter the GPS denied region,
%   it has to escape to pre-defined area
%%
s=[20,20];    % start node
e=[180,180];  % end node
envir=[0, 200, 0, 200]; % define the area
safe=[160,40,25];       % define the safe region with center at (160,40)
GPSdenied=[100,120,10]; % define the region, where no GPS signal
NumObstacles = 60;  % number of obstacles

obstacle = ObstacleCreator(NumObstacles,envir,s,e,safe);  % This randomly create obstacle and it's distributed in the environment 

[waypoint,obstacle]= WaypointGenerator(e,s,envir,safe,obstacle); % generate waypoints

[X_path,Y_path]=path_following(waypoint,GPSdenied,envir,safe); % path following algorithm


%% ---------------------------- Plot -----------------
figure(2)
  for i=1:length(waypoint),
        plot(waypoint(i,1),waypoint(i,2),'r*'); hold on,
  end
  
  for i=1:length(waypoint)-1,
    plot([waypoint(i,1);waypoint(i+1,1)],[waypoint(i,2);waypoint(i+1,2)],'b');  %draw the line connecting waypoints
    hold on,
  end
  
  %% plot obstacle
  N=10;
  th = 0:2*pi/N:2*pi;
  for i=1:60,
     X = obstacle.radius(i)*cos(th) + obstacle.cn(i);
     Y = obstacle.radius(i)*sin(th) + obstacle.ce(i);
     fill(X,Y,'k'); hold on,
  end
  
  %% plot trajectory
  plot(X_path,Y_path,'r','linewidth',2);
  
  %% safe region
  ang=0:0.01:2*pi;
  xp=safe(3)*cos(ang)+safe(2);
  yp=safe(3)*sin(ang)+safe(1);
  plot(yp,xp);
  %legend('safe');
    
  %% GPS denied region
  ang=0:0.01:2*pi;
  xde=GPSdenied(3)*cos(ang)+GPSdenied(2);
  yde=GPSdenied(3)*sin(ang)+GPSdenied(1);
  plot(yde,xde);
    
  hold off,