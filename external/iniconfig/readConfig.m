function options=readConfig(inifile)
% parse configuration in ini format

if ~exist(inifile,'file')
    error('Config file %s does not exist!\n',inifile);
end

options=[];
ini=IniConfig();
ini.ReadFile(inifile);

% Sections
secs=ini.GetSections;
            secs = strrep(secs, '[', '');
            secs = strrep(secs, ']', '');

            
for sec=secs'
    secName = char(sec);
    keys = ini.GetKeys(secName);
    secOptions=[];
    for key=keys'
        keyName=char(key);
        keyValue=getKeyValue(ini.GetValues(secName,keyName));
        secOptions=setfield(secOptions,keyName,keyValue);
    end
    secName = strrep(secName, ' ','_');
    options = setfield(options,secName,secOptions);    
end
end

function keyValue=getKeyValue(keyValue)

if strcmpi(keyValue,'inf')
    keyValue = Inf;
elseif strcmpi(keyValue,'-inf');
    keyValue = -Inf;
end
end