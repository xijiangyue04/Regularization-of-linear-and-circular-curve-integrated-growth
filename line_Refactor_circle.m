%����������һЩ�����line_sort_segment
%�ع���Circle_sort_segment���棬�Ӷ��õ����յ�Circle_sort_segment��line_sort_segment
%%resolutionΪ���پ���Ϊһ���� �����򻯺��Բ���������Ƶľ���ֱ���  
function  [Circle_sort_segment,line_sort_segment] = line_Refactor_circle(Circle_sort_segment,line_sort_segment,range_resolution,number,resolution)

 for i=1:length(Circle_sort_segment)
     pnts=Circle_sort_segment{i};
      if size(pnts,1)>=number
     [circle_fit,theta] = space_circle_LS(pnts,resolution);
     for j=1:length(line_sort_segment)
         if ~isempty(line_sort_segment{j})
          [neighbor_idx2,Pc_dis2]=knnsearch(circle_fit,line_sort_segment{j},'k',1);
          line_sort_no=size(line_sort_segment{j},1);
          attribution_no= size(find(Pc_dis2<range_resolution),1);
          if attribution_no/line_sort_no>0.7 
              Circle_sort_segment{i}=[pnts;line_sort_segment{j}];
              line_sort_segment{j}=[]; 
          end
         end

     end
      end
 end
 line_sort_segment(cellfun(@isempty,line_sort_segment))=[];
     
      
     
     
     
     
     
