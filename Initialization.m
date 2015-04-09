function X0=Initialization(XYZ,DSE,T,Vmax)

if nargin~=4
    error('Either not sufficient or too many inputs')
end
DMV=size(XYZ{1,1},2);
X0=zeros(size(XYZ{1,1},1),DSE*DMV);
for i=1:DSE
    if i==1
        for j=1:DMV
            X0(:,(j-1)*DSE+i)=XYZ{1,i}(:,j);
        end
    else
        %         i
        %         XYZ{1,i}
        %         XYZ{1,1}
        if ~isempty(XYZ{1,i})
            [~, indx]= min(pdist2(XYZ{1,i},XYZ{1,1}));
            XYZ2M=XYZ{1,i}(indx,:);
            for j=1:DMV
                XYZ2N=X0(:,(j-1)*DSE+1);
                for k=2:i-1
                    XYZ2N=XYZ2N+(X0(:,(j-1)*DSE+k)*T^(k-1))/factorial(k-1);
                end
                X0(:,(j-1)*DSE+i)=(DSE-1)*(XYZ2M(:,j)-XYZ2N)/T^(DSE-1);
                X0(:,(j-1)*DSE+i)=(abs(X0(:,(j-1)*DSE+i))<2^(i-2)*Vmax/T^(i-2)).*X0(:,(j-1)*DSE+i);
            end
        else
            for j=1:DMV
            
                X0(:,(j-1)*DSE+i)=0;
            end
        end
            
        end
        
    end
    
    
