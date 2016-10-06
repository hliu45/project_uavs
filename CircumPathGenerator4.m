
start_node = [10,10];
middle_node=[75,75];
end_node   = [160,160];
radiotower=[70,90;
            120,40]; 
Rl=length(radiotower(:,1));
dline=norm(radiotower(1,:)-radiotower(2,:));
d=[];
d=[norm(radiotower(1,:)-start_node)];
d=[d;d(end)-dline];
d=[d;dline-d(end)];
d=[d;norm(radiotower(2,:)-end_node)];

figure(1)
axis([-80 260 -120 260]); hold on,
for i=1:Rl
    plot(radiotower(i,1),radiotower(i,2),'*');
end
plot(start_node(1),start_node(2),'o');
plot(end_node(1),end_node(2),'o');
hold on,

%%
% theta=atan2(start_node(2)-radiotower(1,2),start_node(1)-radiotower(1,1));
% while theta > 2*pi
%     theta=theta-2*pi;
% end
% while theta< 0
%     theta=theta+2*pi;
% end
% 
% psi= acos((d(1)^2+dline^2-d(2)^2)/(2*d(1)*dline));
% angbetween=atan2(radiotower(2,2)-radiotower(1,2),radiotower(2,1)-radiotower(1,1));
% endang=psi+angbetween;
% while endang > 2*pi
%     endang=endang-2*pi;
% end
% while endang < 0
%     endang =endang+2*pi;
% end

% cir_x=[];
% cir_y=[];
% for ang=theta:-0.01:endang
%     cirx=d(1)*cos(ang)+radiotower(1,1);
%     ciry=d(1)*sin(ang)+radiotower(1,2);
%     cir_x=[cir_x;cirx];
%     cir_y=[cir_y;ciry];
% end
% figure(1)
% plot(cir_x,cir_y,'r','linewidth',3);
%%

%%
% theta=atan2(cir_y(end)-radiotower(2,2),cir_x(end)-radiotower(2,1));
% while theta > 2*pi
%     theta=theta-2*pi;
% end
% while theta< 0
%     theta=theta+2*pi;
% end
% 
% endang=atan2(end_node(2)-radiotower(2,2),end_node(1)-radiotower(2,1));
% psi= acos((d(1)^2+dline^2-d(2)^2)/(2*d(1)*dline));
% angbetween=atan2(radiotower(2,2)-radiotower(1,2),radiotower(2,1)-radiotower(1,1));
% endang=angbetween-psi;
% endang=endang-2*pi;
% while endang > 2*pi
%     endang=endang-2*pi;
% end
% while endang < 0
%     endang =endang+2*pi;
% end
% 
% cir_x=[];
% cir_y=[];
% for ang=theta:-0.01:endang
%     cirx=d(2)*cos(ang)+radiotower(2,1);
%     ciry=d(2)*sin(ang)+radiotower(2,2);
%     cir_x=[cir_x;cirx];
%     cir_y=[cir_y;ciry];
% end
% figure(1)
% plot(cir_x,cir_y,'r','linewidth',3);


circle(radiotower(1,1),radiotower(1,2),d(1));
circle(radiotower(2,1),radiotower(2,2),d(2));
circle(radiotower(1,1),radiotower(1,2),d(3));
circle(radiotower(2,1),radiotower(2,2),d(4));