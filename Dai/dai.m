clear all;
load p_2d_3


[F,P]=size(p_2d);F=F/2;

% center the p_2d data

for i=1:2*F;p_2d(i,:)=p_2d(i,:)-mean(p_2d(i,:));end;

[U,D,V]=svd(p_2d);

sv=diag(D);
% plot(sv(1:20));

% Specify K
K=3;

Pihat=U(:,1:3*K)*sqrt(D(1:3*K,1:3*K));
Bhat=sqrt(D(1:3*K,1:3*K))*V(:,1:3*K)';

W=Pihat*Bhat;

% G=zeros(3*K,3*K);


num_unknown=3*K*(3*K+1)/2;

A=zeros(2*F,num_unknown);

for i=1:F
%         Pihat_i=Pihat(2*i-1:2*i,:);
%         PiPi=kron(Pihat_i,Pihat_i);
%         A(2*i-1:2*i,:)=[PiPi(1,:)-PiPi(4,:);PiPi(2,:)];
          xf=Pihat(2*i-1,:)';yf=Pihat(2*i,:)';
        
          A(2*i-1:2*i,:)=[vc_fun(xf,yf), vc_fun(xf-yf,xf+yf)]';
end
    
%     [U,D,V]=svd(A);
% %     basis=V(:,end-2*K^2+K+1:end);
%     A_=U(:,1:end-2*K^2+K)*D(1:end-2*K^2+K,1:end-2*K^2+K)*V(:,1:end-2*K^2+K)';
%     basis=null(A_);
    mask=devech(1:num_unknown');
    
%     cvx_begin sdp
%         variable a(2*K^2-K,1)
%         q=basis*a;
%         for r=1:3*K
%             for c=1:3*K
%                 Q(r,c)=q(mask(r,c));
%             end
%         end
%         
%         minimize(trace(Q))
%         
%         subject to
%         Q>=0
%         sum(a)==1
%     cvx_end
    
    
    cvx_begin sdp
        variable q(num_unknown,1)
        
        for r=1:3*K
            for c=1:3*K
                Q(r,c)=q(mask(r,c));
            end
        end
        
        minimize(q'*A'*A*q+50*trace(Q))
        
        subject to
        Q>=0
        sum(q)==1
        
    cvx_end
    
    
    [U,D,V]=svd(Q);
    plot(diag(D))
    
    Gk=U(:,1:3)*sqrt(D(1:3,1:3));

    cRi=zeros(2,3,F);
    Ri=zeros(2,3,F);
    
    R_mat=[];
    
    
    N_mat=zeros(2*F,9*K);
    for i=1:F
            Pihat_i=Pihat(2*i-1:2*i,:);
            cRi(:,:,i)=Pihat_i*Gk;
            rri=cRi(:,:,i);
            
            rri(1,:)=rri(1,:)/norm(rri(1,:));
            rri(2,:)=rri(2,:)/norm(rri(2,:));
            
            R_i=[rri;null(rri)'];
            
            if i==1
                R_mat=rri;
                Ri(:,:,i)=rri;
            end
            
            if i>1
                ro=vrrotmat2vec(R_i*R_i_prev');
                if abs(ro(4))<=pi/2
                    Ri(:,:,i)=rri;
                else
                    Ri(:,:,i)=-rri;
                    
%                     pause();
                end
                
                R_mat=blkdiag(R_mat,Ri(:,:,i));
            end
            
            R_i_prev=R_i;
            
            N_mat(2*i-1:2*i,:)=kron(null(rri)',Pihat_i);
    end
    
    [U,D,V]=svd(N_mat);
    
    vecG=V(:,end-K+1:end);
    
    G=zeros(3*K,3*K);
    for i=1:size(vecG,2)
        G(:,3*i-2:3*i)=reshape(vecG(:,i),3*K,3);
    end
    G=G/norm(G);
    
    Pi=Pihat*G;
    
    B=inv(G)*Bhat;
%     
%     
%    
% S=R_mat'*inv(R_mat*R_mat')*W;

    
% B=inv([Gk Gk Gk])*Bhat;


% for i=1:F;scatter3(S(3*i-2,:),S(3*i-1,:),S(3*i,:));pause();end
