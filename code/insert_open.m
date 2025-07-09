function row = insert_open(Start,BackPointer,f)
% insert_open - funtion which must insert the value to the row of OPEN list
% in below way:
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % There is that data (0/1) | Start_point_X | Start_point_Y | BackPointer_X | BackPointer_Y | h(n) | g(n) | f(n) %  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

row = [1,8];
row(1,1) = 1;
row(1,2) = Start(:,1);
row(1,3) = Start(:,2);
row(1,4) = BackPointer(:,1);
row(1,5) = BackPointer(:,2);
row(1,6) = f(3,:);
row(1,7) = f(2,:);
row(1,8) = f(1,:);
end