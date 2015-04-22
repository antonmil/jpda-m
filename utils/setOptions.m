function param = setOptions(options,fR)

param.Prun_Thre = 0.9999999;
param.tret = 15;
param.Term_Frame = 15;

param.PD = 0.89;
param.q1 = .5;
param.Mcov = 7;

param.AR = .45;

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