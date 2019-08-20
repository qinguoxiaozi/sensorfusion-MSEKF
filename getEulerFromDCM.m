%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:getEulerFromDCM.m
% date:2019/08/20
% author:YangYue
% email:qinguoxiaozi@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [roll,pitch,yaw] = getEulerFromDCM(dcm)

pitch = -asin(dcm(3,1));
roll = atan2(dcm(3,2),dcm(3,3));
yaw = atan2(dcm(2,1),dcm(1,1));
if yaw < 0
    yaw = yaw + 2*pi;
end

end