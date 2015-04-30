function param = setOptionsJIPDA(options,fR)

param.Prun_Thre = 0.9999999;
param.tret = 15;
param.Term_Frame = 15;

param.PD = 0.89;
param.q1 = .5;
param.Mcov = 7;

param.AR = .45;

param.fpn=10;

if isempty(options)    
    return;
end

fR=1;
param.Prun_Thre = options.Parameters.Prun_Thre;
param.tret = round(options.Parameters.tret*fR);
param.Term_Frame = round(options.Parameters.Term_Frame*fR);

param.PD = min(options.Parameters.PD,.99); param.PD=max(0.51,param.PD);
param.q1 = options.Parameters.q1;
param.Mcov = options.Parameters.Mcov;

param.Gatesq = options.Parameters.Gatesq;
param.FPPI = options.Parameters.FPPI;
param.Upos=options.Parameters.Upos; % uncertainty in initial position
param.Uvel=options.Parameters.Uvel; % uncertainty in initial velocity

param.MF = options.Parameters.MF;
param.m = options.Parameters.m;

param.AR = options.Parameters.AR;
param.AR = max(0.2,param.AR);param.AR = min(0.5,param.AR);

param.fpn = options.Parameters.fpn;
% param.fpn = max(1,param.fpn);param.fpn = min(100,param.fpn);

%%% JIPDA
% existing;  ex+missed; terminated, sums to 1
Pexist = min(1,options.Parameters.Pexist);
Pexmissed= min(1,options.Parameters.Pexmissed);
Pexist = max(0,Pexist); Pexmissed = max(0,Pexmissed);

% assure these params sum to one
s12=Pexist + Pexmissed;
if (s12) >= 1
    Pexist = Pexist/s12 - 0.1;
    Pexmissed = Pexmissed/s12 - 0.1;
end
Pterm = 1 - Pexist - Pexmissed;
param.Pexist = Pexist;
param.Pexmissed = Pexmissed;
param.Pterm = Pterm;

% Pdnt0=[Pexist; Pexmissed; Pterm];




param.TPM1=options.Parameters.TPM1;
param.TPM2=options.Parameters.TPM2;
param.TPM3=options.Parameters.TPM3;
param.TPM4=options.Parameters.TPM4;
param.TPM5=options.Parameters.TPM5;
param.TPM6=options.Parameters.TPM6;


p1=param.TPM1; p1=max(0,min(p1,1));
p2=param.TPM2; p2=max(0,min(p2,1));
p3=param.TPM3; p3=max(0,min(p3,1));
p4=param.TPM4; p4=max(0,min(p4,1));
p5=param.TPM5; p5=max(0,min(p5,1));
p6=param.TPM6; p6=max(0,min(p6,1));

tiny=0.1;
s1=p1+p2; if s1>=1, p1=p1/s1- tiny; p2=p2-tiny; end
s2=p3+p4; if s2>=1, p3=p3/s2- tiny; p4=p4-tiny; end
s3=p5+p6; if s3>=1, p5=p5/s3- tiny; p6=p6-tiny; end

param.TPM_dnt=[
    p1,p3,p5;
    p2,p4,p6;]; 

% conf < term, both < 1
p1=options.Parameters.Term_tre1; p1=max(0,min(p1,1));
p2=options.Parameters.Term_tre2; p2=max(0,min(p2,1));

if p1>=p2, p1=p2-tiny; end
s1=p1+p2; if s1>=1, p1=p1/s1- tiny; p2=p2-tiny; end
param.Term_tre=[p1,p2];

%     Term_tre=[0.4 0.5]; % The probabilities for track confirmation and termination


% TPM_dnt=[0.59, 0.49, 0; %   exist and remains,      exist -> occluded,      terminated
%         0.409, 0.5, 0]; %       occlusion -> visible,   occlusion -> occlusion, terminated
