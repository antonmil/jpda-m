function [XeT,PeT,Xe,Pe,Ff,Term_Con,mui]=MULTISCAN_JPDA(XYZ,F,Q,H,R,X0,P0,Tracking_Scheme,JPDA_P,mbest,...
    JPDA_multiscale,Min_PD,S_limit,mu0i,TPM,TPM_Option,H_TPM,Term_Frame,Initial,I,gate_plot,Mx_tr)

UV=size(I);
if nargin~=22
    error('Not enough inputs. Please check your inputs')
end

% hwait = waitbar(0,'Background Extraction');
elapsedTime=inf;


% %***************************** Initialization ********************************
Kt=size(F,3);%The Number of Dynamic Models
if strcmp(TPM_Option,'State-Dependent')
    if Kt~=2
        error('The state-dependent transion probabilty matrix (TPM) is only designed for two models; if you have more or less than two models, use constant option')
    end
    warndlg('please note that p(Kt = 1 |Kt-1,.) = 1 - S^k * exp(.) and p(Kt = 2 |Kt-1,.) = S^k*exp(.) necessarily')
end

Frame=size(XYZ,2);DMV=size(H,1);DSV=size(H,2);

N_Target=size(X0,1);
%*********************** Initial IMM Parameters ************************

%Initial State Vector & IMM Initial Parameters
Xe=cell(Kt,N_Target);
XeT=cell(1,N_Target);
Pe=cell(Kt,N_Target);
PeT=cell(1,N_Target);

mui=cell(1,N_Target);muij=cell(1,N_Target);
PT=cell(1,N_Target);
C=cell(1,N_Target);
Lambda=cell(1,N_Target);

Ff=cell(1,N_Target);
Term_Con=cell(1,N_Target);

for ij=1:N_Target
    if strcmp(TPM_Option,'Constant')
        PT{1,ij}=TPM;
    elseif strcmp(TPM_Option,'State-Dependent') %It is not generalized
        P22=TPM{2,2}*(det(TPM{2,1})/det(TPM{2,1}+H_TPM*P0*H_TPM'))^0.5...
            *exp(-0.5*(X0(ij,:)*H_TPM')*((TPM{2,1}+H_TPM*P0*H_TPM')^(-1))*(X0(ij,:)*H_TPM')');
        P12=TPM{1,2}*(det(TPM{1,1})/det(TPM{1,1}+H_TPM*P0*H_TPM'))^0.5...
            *exp(-0.5*(X0(ij,:)*H_TPM')*((TPM{1,1}+H_TPM*P0*H_TPM')^(-1))*(X0(ij,:)*H_TPM')');
        PT{1,ij}=[1-P12 P12;1-P22 P22];
    end
    mui{1,ij}=mu0i;
    Ff{1,ij}=[1 1];
    Term_Con{1,ij}=0;
    
    for r=1:Kt
        Xe{r,ij}=X0(ij,:)';
        if r==1
            XeT{1,ij}=mui{1,ij}(r)*Xe{r,ij};
        else
            XeT{1,ij}=XeT{1,ij}+mui{1,ij}(r)*Xe{r,ij};
        end
    end
    for r=1:Kt
        Pe{r,ij}=P0;
        if r==1
            PeT{1,ij}=mui{1,ij}(r)*(Pe{r,ij}+(XeT{1,ij}-Xe{r,ij})*(XeT{1,ij}-Xe{r,ij})');
        else
            PeT{1,ij}=PeT{1,ij}+mui{1,ij}(r)*(Pe{r,ij}+(XeT{1,ij}-Xe{r,ij})*(XeT{1,ij}-Xe{r,ij})');
        end
    end
    
end


%JPDA Initial Parameters

Beta=JPDA_P(1);Gate=JPDA_P(2);
msp=JPDA_multiscale;

Terminated_objects_index=[];


%************************ Kalman Filter Tracking **************************
% waitbar((1)/(Frame),hwait,['Frame # ',num2str(1),'   Estimated time: ',num2str((Frame-1)*elapsedTime),' s'])
if strcmp(gate_plot,'Yes')
    colorord=.25+.75*rand(Mx_tr,3);
    figure,imshow(I(:,:,:,1))
    hold on
    plot(XYZ{1}(:,1),XYZ{1}(:,2),'ow')
    hold on
    for no=1:N_Target
        Ellipse_plot(inv(H*P0*H')/Gate, H*XeT{1,no},colorord(no,:));
    end
    hold off
    pause(0.05)
end

for f=2:Frame
    if ~mod(f,10),    fprintf('.'); end
    ticID = tic;
    if strcmp(gate_plot,'Yes')
%         imshow(I(:,:,:,f))
%         hold on
        plot(XYZ{f}(:,1),XYZ{f}(:,2),'ok')
    end
    %************* Measurements **************
    
    %Kalman Parameters' Allocation
    N_Target=size(Xe,2);
    Mes_Tar=false(size(XYZ{1,f},1),N_Target,Kt);
    MXe=zeros(DSV,Kt,N_Target);PXe=zeros(DSV,DSV,Kt,N_Target);
    S=zeros(DMV,DMV,Kt,N_Target);K=zeros(DSV,DMV,Kt,N_Target);
    Target_Obs_indx=cell(N_Target,Kt);
    Rt_In_Pr=cell(N_Target,Kt);
    Target_Obs_indx_Total=cell(N_Target,1);
    Target_probabilty=cell(N_Target,Kt);
    Target_probabilty_Total=cell(N_Target,1);
    
    Curr_Obs=[];
    %************* Predict Step **************
    
    for no=1:N_Target
        if ~ismember(no,Terminated_objects_index)
            k=f-Ff{1,no}(1,1)+1;
            C{1,no}(:,k-1)=mui{1,no}(k-1,:)*PT{1,no};
            X0j=zeros(DSV,Kt);
            PI0j=zeros(DSV,DSV,Kt);
            for r2=1:Kt
                %
                for r1=1:Kt
                    muij{1,no}(r1,r2,k-1)= PT{1,no}(r1,r2)*mui{1,no}(k-1,r1)/ C{1,no}(r2,k-1);
                    X0j(:,r2)=X0j(:,r2)+muij{1,no}(r1,r2,k-1)*Xe{r1,no}(:,k-1);
                end
                
                for r1=1:Kt
                    PI0j(:,:,r2)=PI0j(:,:,r2)+muij{1,no}(r1,r2,k-1)...
                        *(Pe{r1,no}(:,:,k-1)+(Xe{r1,no}(:,k-1)-X0j(:,r2))...
                        *(Xe{r1,no}(:,k-1)-X0j(:,r2))');
						% muij{1,no}(r1,r2,k-1)
						% Pe{r1,no}(:,:,k-1)
						% (Xe{r1,no}(:,k-1)-X0j(:,r2))
                end
                
                % Kalman Preditction Step& Hypothesis Tree Reconstruction Step
                [Target_Obs_indx{no,r2},Target_probabilty{no,r2},MXe(:,r2,no),...
                    PXe(:,:,r2,no),S(:,:,r2,no),K(:,:,r2,no),Rt_In_Pr{no,r2}]=...
                    Tree_Constructor(X0j(:,r2),PI0j(:,:,r2),F(:,:,r2),...
                    Q(:,:,r2),H,R,XYZ(1,f:min(f+msp-1,Frame)),S_limit,Gate,Min_PD,Beta,DMV);
                if strcmp(gate_plot,'Yes')
                     Sprim=S(:,:,r2,no);
                     if max(max(S(:,:,r2,no)))>S_limit %limit S matrix
                         Sprim=S(:,:,r2,no)*S_limit/max(max(S(:,:,r2,no)));
                     end
                    hold on
                    Ellipse_plot(inv(Sprim)/Gate, H*MXe(:,r2,no),colorord(no,:))
                end
                
                Mes_Tar(Target_Obs_indx{no,r2}{1}{1},no,r2)=true;
                Target_Obs_indx_Total{no,1}=[Target_Obs_indx_Total{no,1};...
                    Target_Obs_indx{no,r2}{1}{1}(~ismember(Target_Obs_indx{no,r2}{1}{1}, ...
                    Target_Obs_indx_Total{no,1}))];
                Lambda{1,no}(r2,k)=sum(Target_probabilty{no,r2}{1}{1});
                
            end
        end
    end
    if strcmp(gate_plot,'Yes')
        hold off
        pause(0.05)
    end
    %***************** Joint Probabilty Data Association ******************
    exist_ind=(~ismember(1:N_Target,Terminated_objects_index));
    Final_probabilty=cell(1,Kt);
    Mes_Tar2=Mes_Tar(:,exist_ind,:);[Umt, Vmt, Zmt]=size(Mes_Tar2);
    Mes_Tar=false(Vmt+Umt,Vmt+Umt,Zmt);
    for r=1:Kt
        Final_probabilty{1,r}=cell(1,N_Target);
        if strcmp(Tracking_Scheme,'JPDA')
            Mes_Tar(:,:,r)=[false(Vmt,Vmt+Umt);Mes_Tar2(:,:,r) false(Umt,Umt)];
            % Final_probabilty{1,r}(1,exist_ind) =JPDA_Probabilities(Mes_Tar(:,:,r),Target_Obs_indx(exist_ind,r)',Target_probabilty(exist_ind,r)');
            % Final_probabilty{1,r}(1,exist_ind) =Multiscan_JPDA_Probabilities(Mes_Tar(:,:,r),Target_Obs_indx(exist_ind,r)',Target_probabilty(exist_ind,r)');
            Final_probabilty{1,r}(1,exist_ind) =Approx_Multiscan_JPDA_Probabilities(Mes_Tar(:,:,r),Rt_In_Pr(exist_ind,r),mbest);

            
        elseif strcmp(Tracking_Scheme,'PDA')
            Final_probabilty{1,r}=cellfun(@(x) x/sum(x), Target_probabilty(:,r), 'UniformOutput', false)';
        else
            error('"Tracking_Scheme" should be either "PDA" or "JPDA"')
        end
    end
    %**************************** Update step *****************************
    
    for no=1:N_Target
        if ~ismember(no,Terminated_objects_index)
            k=f-Ff{1,no}(1,1)+1;
            Pij=cell(Kt,1);
            XeT{1,no}(:,k)=zeros(DSV,1);
            PeT{1,no}(:,:,k)=zeros(DSV,DSV);
            CTotal= Lambda{1,no}(:,k)'*C{1,no}(:,k-1);
            PIJT=zeros(1,size(Target_Obs_indx_Total{no,1},1)+1);
            for r=1:Kt
                     NN=length(Target_Obs_indx{no,r}{1}{1})+1;
                Pij{r,1}=(Final_probabilty{1,r}{1,no})';
                if isempty(Target_Obs_indx{no,r}{1}{1})
                    Xe{r,no}(:,k)=MXe(:,r,no);
                    dP=0;
                else
                    Yij=XYZ{1,f}(Target_Obs_indx{no,r}{1}{1},1:DMV)-repmat((H*MXe(:,r,no))',[size(Target_Obs_indx{no,r}{1}{1}),1]);
                    Ye=(Pij{r,1}(2:NN)*Yij)';
                    Xe{r,no}(:,k)=MXe(:,r,no)+K(:,:,r,no)*Ye;
                    dP=K(:,:,r,no)*(repmat(Pij{r,1}(2:NN),[DMV 1]).*Yij'*Yij-(Ye*Ye'))*K(:,:,r,no)';
                end


                Pst=PXe(:,:,r,no)-K(:,:,r,no)*S(:,:,r,no)*K(:,:,r,no)';
                Po=Pij{r,1}(1)*(PXe(:,:,r,no))+(1-Pij{r,1}(1))*Pst;
                Pe{r,no}(:,:,k)=Po+dP;
                
                mui{1,no}(k,r)=Lambda{1,no}(r,k)*C{1,no}(r,k-1)/CTotal;
                PIJT([true,(ismember(Target_Obs_indx_Total{no,1},Target_Obs_indx{no,r}{1}{1}))'])=...
                    PIJT([true,(ismember(Target_Obs_indx_Total{no,1},Target_Obs_indx{no,r}{1}{1}))'])+...
                    mui{1,no}(k,r)*Pij{r,1};
                XeT{1,no}(:,k)=XeT{1,no}(:,k)+mui{1,no}(k,r)*Xe{r,no}(:,k);
            end
            Target_probabilty_Total{no,1}=PIJT;
            for r=1:Kt
                PeT{1,no}(:,:,k)= PeT{1,no}(:,:,k)+mui{1,no}(k,r)*(Pe{r,no}(:,:,k)+...
                    (XeT{1,no}(:,k)-Xe{r,no}(:,k))*(XeT{1,no}(:,k)-Xe{r,no}(:,k))');
            end
            
            if strcmp(TPM_Option,'State-Dependent') %It is not generalized
                P22=TPM{2,2}*(det(TPM{2,1})/det(TPM{2,1}+H_TPM*Pe{1,no}(:,:,k)*H_TPM'))^0.5...
                    *exp(-0.5*(H_TPM*Xe{1,no}(:,k))'*((TPM{2,1}+H_TPM*Pe{1,no}(:,:,k)*H_TPM')^(-1))*H_TPM*Xe{1,no}(:,k));
                P12=TPM{1,2}*(det(TPM{1,1})/det(TPM{1,1}+H_TPM*Pe{2,no}(:,:,k)*H_TPM'))^0.5...
                    *exp(-0.5*(H_TPM*Xe{2,no}(:,k))'*((TPM{1,1}+H_TPM*Pe{2,no}(:,:,k)*H_TPM')^(-1))*H_TPM*Xe{2,no}(:,k));
                PT{1,no}=[1-P12 P12;1-P22 P22];
            end
            
            %********************* Initiation & Termination  ***********************
            if strcmp(Initial,'ModeBased')
                indM=zeros(1,Kt);
                for r=1:Kt
                    [~, indM(r)]=max(Pij{r,1});
                    
                    if ~isempty(Target_Obs_indx{no,r}{1}{1})
                        Curr_Obs =[Curr_Obs;Target_Obs_indx{no,r}{1}{1}]; %#ok<AGROW>
                        % Curr_Obs =[Curr_Obs;Target_Obs_indx{no,r}((Pij{r,1}(2:length(Pij{r,1}))>0.1))];
                    end
                end
                if sum(indM)==length(indM)
                    Term_Con{1,no}=Term_Con{1,no}+1;
                else
                    Term_Con{1,no}=0;
                end
                
            elseif strcmp(Initial,'TotalBased')
                [~, indM]=max(PIJT);
                if indM==1
                    Term_Con{1,no}=Term_Con{1,no}+1;
                else
                    Term_Con{1,no}=0;
                end
                if ~isempty(Target_Obs_indx_Total{no,1})
                    Curr_Obs =[Curr_Obs;Target_Obs_indx_Total{no,1}];   %#ok<AGROW>
                    %                 Curr_Obs =[Curr_Obs;Target_Obs_indx_Total{no,1}((PIJT(2:length(PIJT))>0.1))];
                end
                
            end
            
            if  (Term_Con{1,no}<=Term_Frame&&size(XeT{1,no},2)>2)||...
                    (Term_Con{1,no}==0&&size(XeT{1,no},2)<=2)%Termination Condition
                Ff{1,no}(1,2)=f;
            else
                Terminated_objects_index=[Terminated_objects_index no]; %#ok<AGROW>
            end
        end
    end
    
    
    All_Obs=1:size(XYZ{1,f},1);
    New_Targets=All_Obs(~ismember(All_Obs,Curr_Obs));%Initiation Condition
    if ~isempty(New_Targets)
        for ij=1:length(New_Targets)
            
            mui{1,N_Target+ij}=mu0i;
            XeT{1,N_Target+ij}=zeros(DSV,1);
            PeT{1,N_Target+ij}=zeros(DSV,DSV);
            for r=1:Kt
                Xe{r,N_Target+ij}=H'*XYZ{1,f}(New_Targets(ij),:)';
                Pe{r,N_Target+ij}=P0;
                
                
                XeT{1,N_Target+ij}=XeT{1,N_Target+ij}+mui{1,N_Target+ij}(1,r)*Xe{r,N_Target+ij};
            end
            for r=1:Kt
                PeT{1,N_Target+ij}= PeT{1,N_Target+ij}+mui{1,N_Target+ij}(1,r)*...
                    (Pe{r,N_Target+ij}+(XeT{1,N_Target+ij}-Xe{r,N_Target+ij})...
                    *(XeT{1,N_Target+ij}-Xe{r,N_Target+ij})');
                
            end
            
            if strcmp(TPM_Option,'Constant')
                PT{1,N_Target+ij}=TPM;
            elseif strcmp(TPM_Option,'State-Dependent') %It is not genralized
                P22=TPM{2,2}*(det(TPM{2,1})/det(TPM{2,1}+H_TPM*Pe{1,N_Target+ij}(:,:,1)*H_TPM'))^0.5...
                    *exp(-0.5*(H_TPM*Xe{1,N_Target+ij}(:,1))'*((TPM{2,1}+H_TPM*Pe{1,N_Target+ij}(:,:,1)*H_TPM')^(-1))*H_TPM*Xe{1,N_Target+ij}(:,1));
                P12=TPM{1,2}*(det(TPM{1,1})/det(TPM{1,1}+H_TPM*Pe{2,N_Target+ij}(:,:,1)*H_TPM'))^0.5...
                    *exp(-0.5*(H_TPM*Xe{2,N_Target+ij}(:,1))'*((TPM{1,1}+H_TPM*Pe{2,N_Target+ij}(:,:,1)*H_TPM')^(-1))*H_TPM*Xe{2,N_Target+ij}(:,1));
                PT{1,N_Target+ij}=[1-P12 P12;1-P22 P22];
            end
            
            Ff{1,N_Target+ij}=[f f];
            Term_Con{1,N_Target+ij}=0;
        end
    end
    elapsedTime = toc(ticID);
%     waitbar((f)/(Frame),hwait,['Frame # ',num2str(f),'   Estimated time: ',num2str(round((Frame-f)*elapsedTime)),' s (',...
%         num2str(round(elapsedTime)),' Sec/Frame)'])
end
% close(hwait)

fprintf('\n');