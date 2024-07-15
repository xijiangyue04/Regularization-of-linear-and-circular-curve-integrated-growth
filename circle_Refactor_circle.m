%����������һЩCircle_sort_segment�ۺ���һ��
%�ع���Circle_sort_segment���棬�Ӷ��õ����յ�Circle_sort_segment
%%resolutionΪ���پ���Ϊһ���� �����򻯺��Բ���������Ƶľ���ֱ���  
function  [Circle_sort_segment] = circle_Refactor_circle(Circle_sort_segment,range_resolution,resolution)

 for i=1:length(Circle_sort_segment)
     pnts=Circle_sort_segment{i};
     if ~isempty(pnts)
     [circle_fit,theta] = space_circle_LS(pnts,resolution);
     
     for j=1:length(Circle_sort_segment)-i 
         if ~isempty(Circle_sort_segment{j+i})
          [neighbor_idx2,Pc_dis2]=knnsearch(circle_fit,Circle_sort_segment{j+i},'k',1);
          Circle_sort_no=size(Circle_sort_segment{j+i},1);
          attribution_no= size(find(Pc_dis2<range_resolution),1);
          if attribution_no/Circle_sort_no>0.7 
              pnts=[pnts;Circle_sort_segment{j+i}];
              Circle_sort_segment{j+i}=[]; 
          end
         end

     end
     end
          Circle_sort_segment{i}=pnts;
 end
 Circle_sort_segment(cellfun(@isempty,Circle_sort_segment))=[];