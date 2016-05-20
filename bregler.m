load p_2d_the_case


[N,P]=size(p_2d);N=N/2;

% center the p_2d data

for i=1:2*N;p_2d(i,:)=p_2d(i,:)-mean(p_2d(i,:));end;

[U,D,V]=svd(p_2d);

sv=diag(D);
plot(sv(1:20));


K=3;

Qhat=U(:,1:3*K)*sqrt(D(1:3*K,1:3*K));
Bhat=sqrt(D(1:3*K,1:3*K))*V(:,1:3*K)';

R={};
A=[];b=[];
L=zeros(N,K);
for t=1:N
    qt=Qhat(2*t-1:2*t,:);
    qt_=zeros(K,6);
    for i=1:K;
        qt_(i,1:3)=qt(1,3*i-2:3*i);
        qt_(i,4:6)=qt(2,3*i-2:3*i);
    end;
    [l_,d_,r_]=svd(qt_);
    r_=sqrt(d_(1,1))*r_(:,1)';
    L(t,:)=sqrt(d_(1,1))*l_(:,1)';
    R{t}=reshape(r_,[3,2])';
    
    r=r_;
    A(3*t-2,:)=[r(1)^2 r(2)^2 r(3)^2 2*r(1)*r(2) 2*r(2)*r(3) 2*r(1)*r(3)];
    A(3*t-1,:)=[r(4)^2 r(5)^2 r(6)^2 2*r(4)*r(5) 2*r(5)*r(6) 2*r(4)*r(6)];
    A(3*t,:)=[r(1)*r(4) r(2)*r(5) r(3)*r(6) r(1)*r(5)+r(2)*r(4) ...
        r(2)*r(6)+r(3)*r(5) r(1)*r(6)+r(3)*r(4)];
    b(3*t-2:3*t,:)=[1 1 0]';
end

x=(A'*A)\(A'*b);

GG=[x(1) x(4) x(6);
    x(4) x(2) x(5);
    x(6) x(5) x(3)];

% G=chol(GG,'lower');
[U,D,V]=svd(GG);

G=U*sqrt(D);

B=Bhat;
for i=1:K;B(3*i-2:3*i,:)=inv(G)*Bhat(3*i-2:3*i,:);end

pt_3d=zeros(3*N,P);
for t=1:N
pt_3d(3*t-2:3*t,:)=kron(L(t,:),eye(3))*B;
end


for t=1:N; 
figure(1);    
scatter3(pt_3d(3*t-2,:),pt_3d(3*t-1,:),pt_3d(3*t,:), 'ro');axis([-200 200 -200 200 -200 200]);

% plot(pt_3d(3*t-2,:),pt_3d(3*t,:), 'ro');

% plot(p_2d(2*t-1,:),p_2d(2*t,:), 'ro');

% axis([-600 600 -300 300 -400 400]);
pause(0.1);end

