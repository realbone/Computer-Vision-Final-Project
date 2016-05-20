function v=vech(X)

M=size(X,1);

v=zeros((M^2-M)/2+M,1);

count=0;
for i=1:M
    for j=1:M
        
    if j<=i
        count=count+1;
        v(count)=X(i,j);
    end
    end
end
end

