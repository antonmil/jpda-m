function convertSTInfoToTXT(stInfo, txtFile)
% convert Anton's struct format to simple CSV file


stInfo.Xi=stInfo.Xi';
stInfo.Yi=stInfo.Yi';
stInfo.W=stInfo.W';
stInfo.H=stInfo.H';

numBoxes = numel(find(stInfo.Xi(:)));
exGT = find(stInfo.Xi(:));
[id,fr]=find(stInfo.Xi);
wd = stInfo.W(exGT);
ht = stInfo.H(exGT);
bx = stInfo.Xi(exGT)-wd/2;
by = stInfo.Yi(exGT)-ht;

x = -1*ones(numBoxes,1);
y = -1*ones(numBoxes,1);
z = -1*ones(numBoxes,1);

if isfield(stInfo,'Xgp') && isfield(stInfo,'Ygp')
    stInfo.Xgp=stInfo.Xgp';
    stInfo.Ygp=stInfo.Ygp';
    x = stInfo.Xgp(exGT);
    y = stInfo.Ygp(exGT);
end

allData = [fr, id, bx, by, wd, ht, -1*ones(numBoxes,1), x,y,z];


dlmwrite(txtFile,allData);
