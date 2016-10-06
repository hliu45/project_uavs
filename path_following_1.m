function path;
%clear
%r=[10,0];
%%
xlimit=1.2;
kpath=0.7;
%korbit=1;
tspan=[0,0.1];
x0=0;
v0=0;
xdot0=0;
figSize = [25,185,20,170];
%%
p=[40,60;  % waypoint position
   50,75;
   55,80];
[nw,mw]=size(p)  % nw represent the number of waypoint

nvec=followW(p,nw) %calculate n for each waypoint
display(nvec)

xq=[];  % preparing for xq

for i=1:nw-1,
    xq=[xq;atan2((p(i+1,1)-p(i,1)),(p(i+1,2)-p(i,2)))];  % Calculating xq for each for each waypoint
end


xc= straight_line_following(p(1,:),xq(1),p(1,:),x0,kpath,xlimit);  % calculatingt the first course angle reference

figure(1)
axis(figSize);
title('UAV simulation');
xlabel('x');
ylabel('y');
hold on,

for i=1:nw-1
plot([p(i,1);p(i+1,1)],[p(i,2);p(i+1,2)],'b');  %draw the line connecting waypoints
hold on,
end

hold on,



[t,v]=ode45(@(t,v) twoduav(t,v,xc),tspan,[p(1,1),p(1,2),v0,x0,xdot0]);  % execute the position,course angle according to guidance rule
X=v(:,1); 
Y=v(:,2);
plot(X,Y); % plot the roule 
[n,m]=size(v)

for i=1:nw-1,
    while norm([v(n,1)-p(i+1,1),v(n,2)-p(i+1,2)])>0.8,
        xc= straight_line_following(p(i,:),xq(i),[v(n,1),v(n,2)],v(n,4),kpath,xlimit);
        [t,v]=ode45(@(t,v) twoduav(t,v,xc),tspan,[v(n,1),v(n,2),v(n,3),v(n,4),v(n,5)]);
        X=v(:,1);
        Y=v(:,2);
        Z=v(:,4)
        plot(X,Y,'r','linewidth',2);
        hold on,
        pause(0.1);
        
        [n,m]=size(v)
        display(v(n,:));
    end
end


%%
function xc = straight_line_following(r,xq,p,x,kpath,xlimit);
    while xq - x < (-pi),
        xq = xq + 2*pi;
    end
    while xq - x > pi,
        xq = xq - 2*pi;
    end
    epy=-(p(2)-r(2))*sin(xq)+(p(1)-r(1))*cos(xq);
    xc=xq-xlimit * (2/pi) * atan(kpath * epy);

%%
function d = twoduav(t,v,xc)
syms dotbx bx bva vac;

d = zeros(5,1);
dotbx=15;
bx=90;
bva=1;
vac=25;

d(1)=v(3)*sin(v(4));
d(2)=v(3)*cos(v(4));
d(3)=bva*(vac-v(3));
d(4)=v(5);
d(5)=-dotbx*v(5)+bx*(xc-v(4));
%%

function n=followW(w,N)
n=[0,0];
for i=2:N-1,
    q_i_1=  (w(i,:)-w(i-1,:))/norm(w(i,:)-w(i-1,:));
    q_i=    (w(i+1,:)-w(i,:))/norm(w(i+1,:)-w(i,:));
    nv=(q_i_1+q_i)/norm(q_i_1+q_i);
    n=[n;nv];
    display(i);
end
n=[n;0,0];
%%

