
%���Ĳ��� �ò����Ƕ���Բ�����������ָ���ƽ��е��ƹ��򻯴���
%������ǵڶ�����Circle_sort_segment
% ��������򻯺�ĵ��� Circle_feature_segment
%%resolutionΪ���پ���Ϊһ���� �����򻯺��Բ���������Ƶľ���ֱ���  
function  [Circle_feature_segment] = CIFPR(Circle_sort_segment,range_resolution,resolution)  % Curve Feature point cloud regularization(CFPR)
for i=1:length(Circle_sort_segment)
    
    pnts=Circle_sort_segment{i};
    if size(pnts,1)>5
    [circle_fit,~,theta] = space_circle_LS(pnts,range_resolution);
     [neighbor_idx,Pc_dis]=knnsearch(circle_fit,pnts,'k',1); 
     neighbor_idx=unique(neighbor_idx,'rows');%mx1
    circle_number=size(circle_fit,1);
    %��Բ��Ϊ�ĸ����䣬���ж�ÿ�������Ƿ��е�
    interval=round(circle_number/4);
    interval=1:interval:circle_number;
    First_interval_index=find(neighbor_idx>=interval(1) & neighbor_idx<interval(2));
    Second_interval_index=find(neighbor_idx>=interval(2) & neighbor_idx<interval(3));
    Third_interval_index=find(neighbor_idx>=interval(3) & neighbor_idx<interval(4));
    Fourth_interval_index=find(neighbor_idx>=interval(4) & neighbor_idx<=circle_number);
    pnts_ratio=size(neighbor_idx,1)/circle_number;
    First_interval_logic=double(~isempty(First_interval_index));
    Second_interval_logic=double(~isempty(Second_interval_index)) ;  
    Third_interval_logic=double(~isempty(Third_interval_index));   
    Fourth_interval_logic=double(~isempty(Fourth_interval_index));
    Each_interval_isempty=[First_interval_logic;Second_interval_logic;Third_interval_logic;Fourth_interval_logic]; 
    interval_NO=size(find(Each_interval_isempty==1),1);
     if interval_NO>=3 & pnts_ratio>0.25
          [circle_fit2,~]=space_circle_LS(pnts,resolution);
         Circle_feature_segment{i}=circle_fit2;
     else
         Circle_feature_segment{i}=circle_fit(neighbor_idx,:);
     end
    end
end
end

