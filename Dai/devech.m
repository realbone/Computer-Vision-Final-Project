function M=devech(vec)
%     k=size(vec,1);
    M=[];
    row=0;
    while ~isempty(vec)
        row=row+1;
        M(row,1:row)=vec(1:row);
        vec(1:row)=[];
    end
    M=M+M'-diag(diag(M));
end
