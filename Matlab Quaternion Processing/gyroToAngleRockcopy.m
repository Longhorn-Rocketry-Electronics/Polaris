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

clock = num2cell(clock ./ 1000000);




dQuat = quatStep(wx.*dtArr,wy.*dtArr,wz.*dtArr);

quat = quaternion(1,0,0,0);

for i = 1:length(wx)
    quat = [quat; quat(i) .* dQuat(i)];
    quat = quatnormalize(quat);
end

quat

angles = num2cell(rad2deg(quat2eul(quat)));

angles = [clock angles(mask,:)];
writecell(angles, [fn(1:end-4) 'euler.txt']);


function output = quatStep(gX, gY, gZ) 

  %// store the tiny rotation from the gyro in a new temporary quaternion
  %// roll, pitch, yaw input is in degrees

  x = (gX ./ 2);  %half the degree and convert it to radians
  y = (gY ./ 2);
  z = (gZ ./ 2);

  s_x = sin(x); c_x = cos(x);
  s_y = sin(y); c_y = cos(y);
  s_z = sin(z); c_z = cos(z);

  %// This is our quaternion that represents the rotation difference
  qr = c_x .* c_y .* c_z + s_x .* s_y .* s_z;
  qx = s_x .* c_y .* c_z - c_x .* s_y .* s_z;
  qy = c_x .* s_y .* c_z + s_x .* c_y .* s_z;
  qz = c_x .* c_y .* s_z - s_x .* s_y .* c_z;
  
  output = quaternion(qr,qx,qy,qz);
end