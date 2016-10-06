function path;
%clear
%
dT=0.5;
xlimit=1.2;       %maximum turning rate
kpath=0.7;      %kpath value
korbit=1.2;     %korbit value
tspan=[0,dT];  %ode45 function time span
v0=0;           %initial velocity
x0=0;           %initial x course angle
xdot0=0;        %initial x rate
figSize = [0, 200, 0, 200];
R=8; % radius of orbit and fillets
final_point_radius=8;

%
p=[40,60;  % waypoint position
   50,75;
   55,80];
[nw,mw]=size(p)  % nw represent the number of waypoint


[nvec,rho]=followW(p,nw) %calculate n for each waypoint

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
plot(X,Y,'r','linewidth',2); % plot the roule 
[n,m]=size(v)
    
    for i=1:nw-2,
        qi=(p(i+1,:)-p(i,:))/norm(p(i+1,:)-p(i,:));
        qi1=(p(i+2,:)-p(i+1,:))/norm(p(i+2,:)-p(i+1,:));
        display(qi);
        display(qi1);
        z=p(i+1,:)-(R/tan(rho(i+1)/2))*qi;
        plot(z(1),z(2),'o');
        
        while H(v(n,:),z,qi)<1,
            xc= straight_line_following(p(i,:),xq(i),v(n,1:2),v(n,4),kpath,xlimit);
            [t,v]=ode45(@(t,v) twoduav(t,v,xc),tspan,v(n,:));
            X=v(:,1);
            Y=v(:,2);
            plot(X,Y,'r','linewidth',2);
            hold on,
            pause(0.1);
        
            [n,m]=size(v)
            display(v(n,:));
        end
        
        c=p(i+1,:)-(R/sin(rho(i+1)/2))*((qi-qi1)/norm(qi-qi1));
        lambda = sign(qi(2)*qi1(1)-qi(1)*qi1(2));
        z=p(i+1,:)+(R/tan(rho(i+1)/2))*qi1;
        plot(z(1),z(2),'o');
        circle(c(1),c(2),R);
        circle(c(1),c(2),R-2);
       
        while H(v(n,:),z,qi1)<1,
            xc = followOrbit(c,R-2,lambda,v(n,1:2),v(n,4),korbit);
            [t,v] = ode45(@(t,v)twoduav(t,v,xc),tspan,v(n,:));
            X=v(:,1);
            Y=v(:,2);
            plot(X,Y,'r','linewidth',2);
            hold on,
            pause(0.1);
            
            [n,m]=size(v)
            display(v(n,:));
        end
    end

%%
    while norm([v(n,1)-p(nw,1),v(n,2)-p(nw,2)])>final_point_radius,
        xc= straight_line_following(p(nw-1,:),xq(nw-1),[v(n,1),v(n,2)],v(n,4),kpath,xlimit);
        [t,v]=ode45(@(t,v) twoduav(t,v,xc),tspan,[v(n,1),v(n,2),v(n,3),v(n,4),v(n,5)]);
        X=v(:,1);
        Y=v(:,2);
        plot(X,Y,'r','linewidth',2);
        hold on,
        pause(0.1);
        
        [n,m]=size(v)
        display(v(n,:));
    end
%%

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
vac=20;

d(1)=v(3)*sin(v(4));
d(2)=v(3)*cos(v(4));
d(3)=bva*(vac-v(3));
d(4)=v(5);
d(5)=-dotbx*v(5)+bx*(xc-v(4));
%%

function [n,rho]=followW(w,N)
n=[0,0];
rho=[0];
for i=2:N-1,
    q_i_1=  (w(i,:)-w(i-1,:))/norm(w(i,:)-w(i-1,:));
    q_i=    (w(i+1,:)-w(i,:))/norm(w(i+1,:)-w(i,:));
    nv=(q_i_1+q_i)/norm(q_i_1+q_i);
    n=[n;nv];
    rhov=acos(dot(-transpose(q_i_1),q_i));
    rho=[rho;rhov];
end
n=[n;0,0];
rho=[rho;0];
%%

%%
function xc = followOrbit(c,R,lambda,v,x,korbit)

d = norm([v(1)-c(1),v(2)-c(2)]);
psi = atan2(v(1)-c(1),v(2)-c(2));
while psi - x < -pi,
    psi = psi + 2*pi;
end
while psi -x > pi,
    psi = psi - 2*pi;
end

xc = psi + lambda*((pi/2)+atan(korbit*((d - R)/R)));


%%
function circle(x,y,r)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp);
hold on,

%%

%%

function hflag=H(v,p,n)
    hflag = 0;
    if (v(1)*n(1)+v(2)*n(2))>=(p(1)*n(1)+p(2)*n(2)),
        hflag = 1;
    end
 %%