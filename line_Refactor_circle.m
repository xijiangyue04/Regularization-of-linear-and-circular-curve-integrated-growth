%第三步，将一些错误的line_sort_segment
%重构到Circle_sort_segment里面，从而得到最终的Circle_sort_segment与line_sort_segment
%%resolution为多少距离为一个点 即规则化后的圆型轮廓点云的距离分辨率  
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
     
      
     
     
     
     
     
