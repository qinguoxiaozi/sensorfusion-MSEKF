%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:normalizeQuaternion.m
% date:2019/08/20
% author:YangYue
% email:qinguoxiaozi@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function quaternion = normalizeQuaternion(quat)

quaternion = zeros(4,1);
length = norm(quat,2);
for i=1:4
    quaternion(i) = quat(i) / length;
end

end