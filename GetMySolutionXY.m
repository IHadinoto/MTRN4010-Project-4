%example function 


function xyE=GetMySolutionXY()

%this is an empty function; you write the code.
global Data5;


disp('testing: being called with these values in Data5.ranges:'); disp('ranges'); disp(Data5.ranges');

%  Data5.Lx: [5x1 double]       : Landmarks positions, x-coordinate, in  global coordinate frame, in meters
%  Data5.Ly: [5x1 double]      : Landmarks positions, y-coordinate, in  global coordinate frame, in meters
%  Data5.ranges: [5x1 double]                               % measured  ranges, in meters
%  Data5.ranges(k) is the measured range to a landmark located at    (Data5.Lx(k),Data5.Ly(k))


xyE=fminsearch(@model,[0;0]);  % must be actually calculated
end

function fun = model(X)
global Data5;
dx = X(1)-Data5.Lx;
dy = X(2)-Data5.Ly;
r2 = sqrt(dx.^2+dy.^2);
error = abs(r2-Data5.ranges);
fun = sum(error);
end
