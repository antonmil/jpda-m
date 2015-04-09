function options = parseOptions(options)
% get options for TC_ODAL tracker

% options to be set
%  params={'c_en','c_ex','c_ij','betta','max_it','thr_cost'};
    

try
    if ischar(options)
        options=readConfig(options);
    end
    
%      for p=params
%          assert(isfield(options.Parameters,char(p)));
%      end
catch err
%     default options
    fprintf('Error parsing options: %s\n',err.message);
    fprintf('Using defaults\n');
    options=[];

end