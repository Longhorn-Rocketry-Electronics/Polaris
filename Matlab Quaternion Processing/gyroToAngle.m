%gyroToAngle.m
%Converts a csv of clock (us), gx, gy, gz (deg/s) to Euler angles.
%Assumes starting angle is (0,0,0) Euler.

%Read file of data
fn = 'gyro10.csv';
data = readcell(fn);
data = data(2:end,:);

fps = 24;

wx = cell2mat(data(:,2)) .* 0.0174533;
wy = cell2mat(data(:,3)) .* 0.0174533;
wz = cell2mat(data(:,4)) .* 0.0174533;

clock = cell2mat(data(:,1));
clock = clock - clock(1);
dtArr = [0];

for i = 1:length(clock)-1
    dt = (clock(i+1)-clock(i)) ./ 1000000;
    dtArr = [dtArr; dt];
end

quat = quaternion(1,0,0,0);

dQuat = quaternion(ones(length(wx),1),wx.*dtArr./2,wy.*dtArr./2,wz.*dtArr./2);

for i = 1:length(wx)
    quat = [quat; quat(i) .* dQuat(i)];
    quat = quatnormalize(quat);
end

quat
angles = num2cell(rad2deg(quat2eul(quat)));
selector = 10000;
period = 1;

mask = mod(clock, period) < selector;
clock = num2cell(round(clock(mask) ./ 1000000),3);
angles = [clock angles(mask,:)];
writecell(angles, [fn(1:end-4) 'euler.txt']);

