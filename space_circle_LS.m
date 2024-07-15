%输入变量：input_pnts(nx3) 圆空间坐标  参考文献[1]刘仰川,唐玉国,高欣.基于双目立体视觉测量数据的空间圆拟合方法对比[J].江苏大学学报(自然科学版),2018,39(04):431-437+444.
%输出变量：theta 圆曲线从0到360°的跨度; 拟合圆曲线上的点circle_fit(nx3)
%%resolution为多少距离为一个点 即规则化后的圆型轮廓点云的距离分辨率  
function [circle_fit,circle_radius,theta] = space_circle_LS(input_pnts,resolution)

num=size(input_pnts,1);
%确定空间圆拟合的平面，并将所有点投影到该平面上
[parameter] = TLS_Plane(input_pnts); %4x1
[project_plane] = PC_Proj(parameter,input_pnts);
 normal1=transpose(parameter(1:3,:));
 normal2=[0 0 1];
 [R]=Rotation_matrix(normal1,normal2);
 transform_pnt=project_plane'*R; %nx3
 %以下是利用参考文献中的最小二乘对其进行求解空间圆的半径和中心点
 B=[transform_pnt(:,1:2),-ones(num,1)];
 L=sum(transform_pnt(:,1:2).^2,2);
 circle_para=inv(B'*B)*B'*L; %3x1
 transform_center=circle_para(1:2,:)./2;%2x1
 transform_center=[transform_center;transform_pnt(1,3)]';  
 circle_radius=sqrt(sum(transform_center(1:2).^2)- circle_para(3,:));
 circle_center= transform_center*inv(R);
 r=circle_radius;
 c=circle_center;
 n=parameter(1:3,:)';
 theta_interval=2*asin(resolution/ (2*r));
 theta=(0:theta_interval:2*pi)'; %theta角从0到2*pi
%theta=(0:2*pi/100:2*pi)';
a=cross(n,[1 0 0]); %n与i叉乘，求取a向量
if ~any(a) %如果a为零向量，将n与j叉乘
a=cross(n,[0 1 0]);
end
b=cross(n,a); %求取b向量
a=a/norm(a); %单位化a向量
b=b/norm(b);
c1=c(1)*ones(size(theta,1),1);
c2=c(2)*ones(size(theta,1),1);
c3=c(3)*ones(size(theta,1),1);
x=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta);%圆上各点的x坐标
y=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta);%圆上各点的y坐标
z=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta);
circle_fit=[x,y,z];
