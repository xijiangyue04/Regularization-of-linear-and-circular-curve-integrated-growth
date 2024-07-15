%�ڶ����� �ò����Ƕ��ڵ�һ���õ���segment����Բ���ߺ�ֱ�߷��봦�����ұ������ڵ���number�ķָ����
%������ǵ�һ����segment�� 
%number  �����ķָ�Ĵ��ڵ��ڸ��������ĵ���
%%resolutionΪ���پ���Ϊһ���� �����򻯺��Բ���������Ƶľ���ֱ���  
function  [line_sort_segment,Circle_sort_segment] = segment_divide(segment,number,resolution)  % Curve Feature point cloud regularization(FPR)


for i=1:length(segment)
    segment_n(i,:)=[size(segment{i},1),i];
end
    sort_segment_n=sortrows(segment_n,1,'descend');
    
 for i=1:length(segment)
     sort_segment{i}=segment{sort_segment_n(i,2)};
 end

    L=1;C=1;
for i=1:length(sort_segment)

    pnts=sort_segment{i};
        if size(pnts,1)>=number
    [line_vector1,mean_pnt1] = space_line_TLS(pnts);
    [PL_dis1] = PL_distance_TLS(pnts, mean_pnt1, line_vector1);
    mean_PLdis=mean(PL_dis1);
    
    
    [circle_fit,~,~] = space_circle_LS(pnts,resolution);
    [neighbor_idx1,Pc_dis1]=knnsearch(circle_fit,pnts,'k',1); 
    mean_Pcdis=mean(Pc_dis1); %
   

    if mean_PLdis<=mean_Pcdis
        line_sort_segment{L}=sort_segment{i};
        L=L+1;
         Circle_sort_segment{C}=[];
    else
        line_sort_segment{L}=[];
        Circle_sort_segment{C}=sort_segment{i};
        C=C+1;
    end
        end
        
        
end

    
    