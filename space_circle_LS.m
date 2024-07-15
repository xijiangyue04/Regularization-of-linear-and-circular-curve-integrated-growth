%���������input_pnts(nx3) Բ�ռ�����  �ο�����[1]������,�����,����.����˫Ŀ�����Ӿ��������ݵĿռ�Բ��Ϸ����Ա�[J].���մ�ѧѧ��(��Ȼ��ѧ��),2018,39(04):431-437+444.
%���������theta Բ���ߴ�0��360��Ŀ��; ���Բ�����ϵĵ�circle_fit(nx3)
%%resolutionΪ���پ���Ϊһ���� �����򻯺��Բ���������Ƶľ���ֱ���  
function [circle_fit,circle_radius,theta] = space_circle_LS(input_pnts,resolution)

num=size(input_pnts,1);
%ȷ���ռ�Բ��ϵ�ƽ�棬�������е�ͶӰ����ƽ����
[parameter] = TLS_Plane(input_pnts); %4x1
[project_plane] = PC_Proj(parameter,input_pnts);
 normal1=transpose(parameter(1:3,:));
 normal2=[0 0 1];
 [R]=Rotation_matrix(normal1,normal2);
 transform_pnt=project_plane'*R; %nx3
 %���������òο������е���С���˶���������ռ�Բ�İ뾶�����ĵ�
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
 theta=(0:theta_interval:2*pi)'; %theta�Ǵ�0��2*pi
%theta=(0:2*pi/100:2*pi)';
a=cross(n,[1 0 0]); %n��i��ˣ���ȡa����
if ~any(a) %���aΪ����������n��j���
a=cross(n,[0 1 0]);
end
b=cross(n,a); %��ȡb����
a=a/norm(a); %��λ��a����
b=b/norm(b);
c1=c(1)*ones(size(theta,1),1);
c2=c(2)*ones(size(theta,1),1);
c3=c(3)*ones(size(theta,1),1);
x=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta);%Բ�ϸ����x����
y=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta);%Բ�ϸ����y����
z=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta);
circle_fit=[x,y,z];
