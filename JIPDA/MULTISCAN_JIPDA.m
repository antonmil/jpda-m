function [XeT,PeT,Xe,Pe,Ff,P_dnt_T,ten_cnf_ter,mui]=MULTISCAN_JIPDA(XYZ,F,Q,H,R,X0,P0,Tracking_Scheme,JPDA_P,mbest,...
    JPDA_multiscale,Min_PD,S_limit,mu0i,TPM,TPM_Option,H_TPM,Pdnt0,TPM_dnt,Int_Term_tre,Initial,I,trk_plot,vis_par)

if nargin~=24
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
colorord=.25+.75*rand(N_Target,3);
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

P_dnt_T=cell(1,N_Target);
P_dnt=cell(Kt,N_Target);
P_dnt_prim=cell(Kt,N_Target);
ten_cnf_ter=cell(1,N_Target);

Ff=cell(1,N_Target);

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
    ten_cnf_ter{1,ij}=0;
    for r=1:Kt
        Xe{r,ij}=X0(ij,:)';
        P_dnt_prim{r,ij}=Pdnt0;
        if r==1
            XeT{1,ij}=mui{1,ij}(r)*Xe{r,ij};
            P_dnt_T{1,ij}=mui{1,ij}(r)*P_dnt_prim{r,ij};
            
        else
            XeT{1,ij}=XeT{1,ij}+mui{1,ij}(r)*Xe{r,ij};
            P_dnt_T{1,ij}=P_dnt_T{1,ij}+mui{1,ij}(r)*P_dnt_prim{r,ij};
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
if strcmp(trk_plot,'Yes')
    Im_dim=ndims(I);
    cln(1:Im_dim-1) = {':'};
    cln(Im_dim)= {1};
    imshow(I(cln{:}))
end

for f=2:Frame
    if ~mod(f,10),    fprintf('.'); end
    ticID = tic;
        if strcmp(trk_plot,'Yes')
            cln(Im_dim)= {f};
            imshow(I(cln{:}))
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
            P_dnt{1,no}=zeros(3,1);
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
                end
                P_dn=TPM_dnt*P_dnt_prim{r2,no};
                P_dnt{r2,no}=[P_dn;1-sum(P_dn)];
                
                % Kalman Preditction Step& Hypothesis Tree Reconstruction Step
                [Target_Obs_indx{no,r2},Target_probabilty{no,r2},MXe(:,r2,no),...
                    PXe(:,:,r2,no),S(:,:,r2,no),K(:,:,r2,no),Rt_In_Pr{no,r2}]=...
                    Tree_Constructor(X0j(:,r2),PI0j(:,:,r2),F(:,:,r2),...
                    Q(:,:,r2),H,R,XYZ(1,f:min(f+msp-1,Frame)),S_limit,Gate,Min_PD,Beta,DMV);
                
                Mes_Tar(Target_Obs_indx{no,r2}{1}{1},no,r2)=true;
                Target_Obs_indx_Total{no,1}=[Target_Obs_indx_Total{no,1};...
                    Target_Obs_indx{no,r2}{1}{1}(~ismember(Target_Obs_indx{no,r2}{1}{1}, ...
                    Target_Obs_indx_Total{no,1}))];
                Lambda{1,no}(r2,k)=sum(Target_probabilty{no,r2}{1}{1});
                
            end
        end
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
            Pjpda=cell(Kt,1);
            Pij=cell(Kt,1);
            XeT{1,no}(:,k)=zeros(DSV,1);
            P_dnt_T{1,no}(:,k)=zeros(3,1);
            PeT{1,no}(:,:,k)=zeros(DSV,DSV);
            CTotal= Lambda{1,no}(:,k)'*C{1,no}(:,k-1);
            PIJT=zeros(1,size(Target_Obs_indx_Total{no,1},1)+1);
            for r=1:Kt
                NN=length(Target_Obs_indx{no,r}{1}{1})+1;   
                Pjpda{r,1}=(Final_probabilty{1,r}{1,no})';
                Pij{r,1}=zeros(size(Pjpda{r,1}));
                P_n=P_dnt{r,no}(2)*Pjpda{r,1}(1)/(1-Min_PD*P_dnt{r,no}(1));
                P_dd=(1-Min_PD)*P_dnt{r,no}(1)*Pjpda{r,1}(1)/(1-Min_PD*P_dnt{r,no}(1));
                if NN>1
                P_d=P_dd+sum(Pjpda{r,1}(2:end));
                else
                    P_d=P_dd;
                end
                P_dnt{r,no}=[P_d;P_n;(1-(P_d+P_n))];
                
                Pij{r,1}(1)=(P_dd+P_n)/(P_d+P_n);
                Pij{r,1}(2:end)=(Pjpda{r,1}(2:end))/(P_d+P_n);
                Pij{r,1}=Pij{r,1}/sum(Pij{r,1});%
                
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
                P_dnt_T{1,no}(:,k)=P_dnt_T{1,no}(:,k)+mui{1,no}(k,r)*P_dnt{r,no};
            end
            Target_probabilty_Total{no,1}=PIJT;
            for r=1:Kt
                PeT{1,no}(:,:,k)= PeT{1,no}(:,:,k)+mui{1,no}(k,r)*(Pe{r,no}(:,:,k)+...
                    (XeT{1,no}(:,k)-Xe{r,no}(:,k))*(XeT{1,no}(:,k)-Xe{r,no}(:,k))');
                P_dnt_prim{r,no}=P_dnt_T{1,no}(:,k);
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
                for r=1:Kt
                    
                    if ~isempty(Target_Obs_indx{no,r}{1}{1})
                        Curr_Obs =[Curr_Obs;Target_Obs_indx{no,r}{1}{1}]; %#ok<AGROW>
                        % Curr_Obs =[Curr_Obs;Target_Obs_indx{no,r}((Pij{r,1}(2:length(Pij{r,1}))>0.1))];
                    end
                end
                
            elseif strcmp(Initial,'TotalBased')
                
                if ~isempty(Target_Obs_indx_Total{no,1})
                    Curr_Obs =[Curr_Obs;Target_Obs_indx_Total{no,1}];   %#ok<AGROW>
                    % Curr_Obs =[Curr_Obs;Target_Obs_indx_Total{no,1}((PIJT(2:length(PIJT))>0.1))];
                end
            end
            if (P_dnt_T{1,no}(1,k)>Int_Term_tre(1))%Track confirmation
                ten_cnf_ter{1,no}(k)=1;
            else
                ten_cnf_ter{1,no}(k)=0;
            end
            if  (P_dnt_T{1,no}(3,k)<Int_Term_tre(2))%Termination Condition
                Ff{1,no}(1,2)=f;
            else
                Terminated_objects_index=[Terminated_objects_index no]; %#ok<AGROW>
                ten_cnf_ter{1,no}(k)=-1;
            end
            if strcmp(trk_plot,'Yes')&&(ten_cnf_ter{1,no}(k)~=-1)...
                    &&(any(ten_cnf_ter{1,no})==1)
                hold on
                tim1=max(k-vis_par(1),1);
                rectangle('Position',[XeT{1,no}(1,k)-vis_par(2)/2,XeT{1,no}(3,k)-vis_par(3),...
                    vis_par(2),vis_par(3)],'EdgeColor',colorord(no,:),'LineWidth',1)
                text(XeT{1,no}(1,k),XeT{1,no}(3,k),num2str(no),'Color',colorord(no,:))
                hold on
                plot(XeT{1,no}(1,tim1:k)+vis_par(2)/2,XeT{1,no}(3,tim1:k),...
                    'Color',colorord(no,:),'LineWidth',2)
            end
        end
    end
    if strcmp(trk_plot,'Yes')
        hold off
        pause(0.000001)
    end
    
    All_Obs=1:size(XYZ{1,f},1);
    New_Targets=All_Obs(~ismember(All_Obs,Curr_Obs));%Initiation (tentative tracks)
    if ~isempty(New_Targets)
        for ij=1:length(New_Targets)
            colorord(N_Target+ij,:)=.25+.75*rand(1,3);
            mui{1,N_Target+ij}=mu0i;
            ten_cnf_ter{1,N_Target+ij}=0;
            P_dnt_T{1,N_Target+ij}=Pdnt0;
            XeT{1,N_Target+ij}=zeros(DSV,1);
            PeT{1,N_Target+ij}=zeros(DSV,DSV);
            for r=1:Kt
                Xe{r,N_Target+ij}=H'*XYZ{1,f}(New_Targets(ij),:)';
                Pe{r,N_Target+ij}=P0;
                P_dnt_prim{r,N_Target+ij}=Pdnt0;
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
        end
    end
    elapsedTime = toc(ticID);
%     waitbar((f)/(Frame),hwait,['Frame # ',num2str(f),'   Estimated time: ',num2str(round((Frame-f)*elapsedTime)),' s (',...
%         num2str(round(elapsedTime)),' Sec/Frame)'])
end
% close(hwait)

