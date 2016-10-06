function path;
clear
center= [10,10];
R=8;
lambda = 1;
korbit = 1;
tspan=[0,0.1];
kpath = 0.02;
xlimit =1.1;
r=[10,-10];
xq=0;%atan2(1.2,1);
x =1;
p = [-10,30];
q = [1,1]
n = [3,5]
%flag= H(p,q,n)
%display(flag)
%circle(center(1),center(2),R);

figure(1)
axis([-20 40 -10 30])
hold on,
plot([r(1);r(1)+60*sin(xq)],[r(2);r(2)+60*cos(xq)],'linewidth',3);

for i=0:30,
    %for j=0:20,
    p=[-20+i*2,23]
    %xc = followOrbit(center,R,lambda,p,x,korbit);  
    xc = straight_line_following(r,xq,p,x,kpath,xlimit);
    plot([p(1);p(1)+2.2*sin(xc)],[p(2);p(2)+2.2*cos(xc)],'r');
    hold on,
    plot([p(1)+2*sin(xc);p(1)+2*sin(xc)+0.5*sin(xc-pi+0.2)],[p(2)+2*cos(xc);p(2)+2*cos(xc)+0.5*cos(xc-pi-0.2)],'r');
    hold on,
    plot([p(1)+2*sin(xc);p(1)+2*sin(xc)+0.5*sin(xc-pi+0.2)],[p(2)+2*cos(xc);p(2)+2*cos(xc)+0.5*cos(xc-pi+0.3)],'r');
    hold on,
    %end
end


%%
function xc = followOrbit(c,R,lambda,p,x,korbit)

d = norm([p(1)-c(1),p(2)-c(2)]);
psi = atan2(p(1)-c(1),p(2)-c(2));
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

function hflag=H(p,q,n)
    hflag = 0;
    if (p(1)*n(1)+p(2)*n(2))>=(q(1)*n(1)+q(2)*n(2)),
        hflag = 1;
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