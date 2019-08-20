%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:getYawFromMag.m
% date:2019/08/20
% author:YangYue
% email:qinguoxiaozi@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function yaw = getYawFromMag(mag, roll, pitch)

hx = mag(1)*cos(pitch) + mag(2)*sin(pitch)*sin(roll) + mag(3)*sin(pitch)*cos(roll);
hy = mag(2)*cos(roll) - mag(3)*sin(roll);
yaw = atan2(-hy, hx);
if yaw < 0
    yaw = yaw + 2*pi;
end

end