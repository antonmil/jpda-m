function status = writeOptions(opt,inifile)
% write ini config file

ini = IniConfig();
ini.AddSections('Parameters');

opttmp=opt;

while ~isempty(fieldnames(opttmp))
    fnames=fieldnames(opttmp);
    fieldname=fnames{1};
    ini=parseField(ini,opt,fieldname);
    opttmp=rmfield(opttmp,fieldname);
end

status = ini.WriteFile(inifile);

end


function ini=parseField(ini,opt,fieldname)
    fvalue=getfield(opt,fieldname);
    sec='Parameters';
    ini.AddKeys(sec,fieldname,fvalue);    
end

