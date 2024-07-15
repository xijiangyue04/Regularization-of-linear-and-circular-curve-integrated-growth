
%第四步： 该步骤是对于圆曲线型特征分割点云进行点云规则化处理
%输入的是第二步的Circle_sort_segment
% 输出：规则化后的点云 Circle_feature_segment
%%resolution为多少距离为一个点 即规则化后的圆型轮廓点云的距离分辨率  
function  [Circle_feature_segment] = CIFPR_improve(Circle_sort_segment,resolution)  % Curve Feature point cloud regularization(CFPR)
for i=1:length(Circle_sort_segment)
    
    pnts=Circle_sort_segment{i};
    if size(pnts,1)>18
    [circle_fit,~,theta] = space_circle_LS(pnts,resolution);
    theta=theta*180/pi;
     [neighbor_idx,Pc_dis]=knnsearch(circle_fit,pnts,'k',1); 
     neighbor_idx=unique(neighbor_idx,'rows');%mx1
     
     interval_angle=theta(neighbor_idx,:);
     first_interval_Angle=interval_angle(interval_angle>=0 & interval_angle<=90,: );
     second_interval_Angle=interval_angle(interval_angle>90 & interval_angle<=180,: );
     third_interval_Angle=interval_angle(interval_angle>180 & interval_angle<=270,: );
     fourth_interval_Angle=interval_angle(interval_angle>270 & interval_angle<=360,: );
     if  ~isempty(first_interval_Angle)
         interval_No1=1;
         first_min_index=find(theta==min(first_interval_Angle));
         first_max_index= find(theta==max(first_interval_Angle));
         first_fit=circle_fit(first_min_index:first_max_index,:);
         first_fit_number=size( first_fit,1);
     else
         interval_No1=0;
         first_fit_number=0;
         first_fit=[];
     end
     
     if  ~isempty(second_interval_Angle)
         interval_No2=1;
         second_min_index=find(theta==min(second_interval_Angle));
         second_max_index= find(theta==max(second_interval_Angle));
         second_fit=circle_fit(second_min_index:second_max_index,:);
         second_fit_number=size(second_fit,1);
     else
         interval_No2=0;
         second_fit_number=0;
         second_fit=[];
     end
     
     if  ~isempty(third_interval_Angle)
         interval_No3=1;
         third_min_index=find(theta==min(third_interval_Angle));
         third_max_index= find(theta==max(third_interval_Angle));
         third_fit=circle_fit(third_min_index:third_max_index,:);
         third_fit_number=size(third_fit,1);
     else
         interval_No3=0;
         third_fit_number=0;
         third_fit=[];
     end
     
     if  ~isempty(fourth_interval_Angle)
         interval_No4=1;
         fourth_min_index=find(theta==min(fourth_interval_Angle));
         fourth_max_index= find(theta==max(fourth_interval_Angle));
         fourth_fit=circle_fit(fourth_min_index:fourth_max_index,:);
         fourth_fit_number=size(fourth_fit,1);
     else
         interval_No4=0;
         fourth_fit_number=0;
         fourth_fit=[];
     end
     interval_NO=interval_No1+interval_No2+interval_No3+interval_No4;
     pnts_ratio=(first_fit_number+second_fit_number+third_fit_number+fourth_fit_number)/size(circle_fit,1);
     total_fit=[first_fit;second_fit;third_fit;fourth_fit];
%     circle_number=size(circle_fit,1);
    %将圆分为四个区间，并判断每个区间是否有点
%     interval=round(circle_number/4);
%     interval=1:interval:circle_number;
%     First_interval_index=find(neighbor_idx>=interval(1) & neighbor_idx<interval(2));
%     Second_interval_index=find(neighbor_idx>=interval(2) & neighbor_idx<interval(3));
%     Third_interval_index=find(neighbor_idx>=interval(3) & neighbor_idx<interval(4));
%     Fourth_interval_index=find(neighbor_idx>=interval(4) & neighbor_idx<=circle_number);
%     pnts_ratio=size(neighbor_idx,1)/circle_number;
%     First_interval_logic=double(~isempty(First_interval_index));
%     Second_interval_logic=double(~isempty(Second_interval_index)) ;  
%     Third_interval_logic=double(~isempty(Third_interval_index));   
%     Fourth_interval_logic=double(~isempty(Fourth_interval_index));
%     Each_interval_isempty=[First_interval_logic;Second_interval_logic;Third_interval_logic;Fourth_interval_logic]; 
%     interval_NO=size(find(Each_interval_isempty==1),1);
     if interval_NO>=3 & pnts_ratio>0.25
          [circle_fit2,~]=space_circle_LS(pnts,resolution);
         Circle_feature_segment{i}=circle_fit2;
     else
         Circle_feature_segment{i}=total_fit;
     end
    end
end
end

