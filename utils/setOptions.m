function param = setOptions(options)

param.Prun_Thre = 0.9999999;
param.tret = 15;
param.Term_Frame = 15;

param.PD = 0.89;
param.q1 = .5;
param.Mcov = 7;

if isempty(options)    
    return;
end

param.Prun_Thre = options.Parameters.Prun_Thre;
param.tret = round(options.Parameters.tret);
param.Term_Frame = round(options.Parameters.Term_Frame);

param.PD = min(options.Parameters.PD,1); param.PD=max(0.5,param.PD);
param.q1 = options.Parameters.q1;
param.Mcov = options.Parameters.Mcov;

param.MF = options.Parameters.MF;
param.m = options.Parameters.m;