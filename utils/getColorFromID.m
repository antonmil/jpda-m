function col=getColorFromID(id)
	% get rgb [0,1] values from id
% 

    if id==0, col=zeros(1,3); return; end
    
    colors=getIDColors;
    col=colors((mod(id,size(colors,1)))+1,:);
end

function colors=getIDColors()
colors=[
    128 255 255;    % 
    255 0 0;        % red           1
    0 255 0;        % green         2
    0 0 255;        % blue          3
    0 255 255;      % cyan          4
    255 0 255;      % magenta       5
    212 212 0;      % yellow        6
    25 25 25;       % black         7
    34,139,34;      % forestgreen   8
    0,191,255;      % deepskyblue   9
    139,0,0 ;       % darkred       10
    218,112,214;    % orchid        11
    244,164,96 ;];  % sandybrown    12
colors = colors / 255;
end