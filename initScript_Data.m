%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:initScript_Data.m
% date:2019/08/20
% author:YangYue
% email:qinguoxiaozi@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% load data
load data1.mat
clearvars -except IMU IMU_label MAG MAG_label EKF1 EKF1_label GPS GPS_label BARO BARO_label ;
%% find IMUStart
% data length
 [IMULength,~] = size(IMU);
 indexIMU = 1;
 [MAGLength,~] = size(MAG);
 indexMAG = 1; 
 [GPSLength,~] = size(GPS);
 indexGPS = 1;
%  [BAROLength,~] = size(BARO);
%  indexBARO = 1;

% measurement start time
if MAG(1,2) < GPS(1,2)
    startTime = GPS(1,2);
    for i=1:MAGLength
        if MAG(i,2) <= startTime && MAG(i+1, 2) > startTime
            indexMAG = i;
            break;
        end
    end
else
    startTime = MAG(1,2);
    for i=1:GPSLength
        if GPS(i,2) <= startTime && GPS(i+1, 2) > startTime
            indexGPS = i;
            break;
        end
    end
end

%IMU start time
for i=1:IMULength
    if startTime <= IMU(i,2)
        indexIMU = i;
        break;
    end
end

if MAG(indexMAG, 2) < IMU(indexIMU, 2)
    indexMAG = indexMAG + 1;
end

if GPS(indexGPS, 2) < IMU(indexIMU, 2)
    indexGPS = indexGPS + 1;
end
%% ReadMeasurements
  %% readIMUData
    angRate =IMU(:,3:5);%rad/s
    accel=IMU(:,6:8);%m/s^2
  %% readGpsData
   %%% velocity(m/s):Navigation coordinate system N-E-D %%%
   %%% position(m):Navigation coordinate system N-E-D %%%
   deg2rad = single(pi/180);
   earthRadius = single(6378145);
   GndSpd = GPS(:,12);
   CourseDeg = GPS(:,13);
   VelD= GPS(:,14);
   for i= 2:length(GPS)
    LatDelta(i)   =  GPS(i,8) - GPS(1,8);
    LongDelta(i)  =  GPS(i,9) - GPS(1,9);
    PosNE(1,i) = earthRadius * LatDelta(i)/100;% m
    PosNE(2,i) = earthRadius * cos(GPS(1,8)*deg2rad) * LongDelta (i)/100;% m
    VelNED(1,i) = GndSpd(i)*cos(CourseDeg(i)*deg2rad);% m/s
    VelNED(2,i) = GndSpd(i)*sin(CourseDeg(i)*deg2rad);% m/s
    VelNED(3,i) = VelD(i);% m/s
   end
   VelNED = VelNED';
   PosNE = PosNE';
  %% readHgtData
 %  Alt_BARO = BARO(:,3)-BARO(1,3);
   Alt_GPS = GPS(:,11)-GPS(1,11);
  %% readMagData
  Mag = MAG(:,3:5);
  magData = Mag* 0.001;
%% CalculateInitialStates
 %% EulAHRS
accel = IMU(6, 6:8);
init_roll = atan2(-accel(2), -accel(3));
init_pitch = atan2(accel(1), -accel(3));

mag = MAG(indexMAG, 3:5);
hx = mag(1)*cos(init_pitch) + mag(2)*sin(init_pitch)*sin(init_roll) + mag(3)*sin(init_pitch)*cos(init_roll);
hy = mag(2)*cos(init_roll) - mag(3)*sin(init_roll);
init_yaw = atan2(-hy, hx);
if(init_yaw < 0)
    init_yaw = init_yaw + 2*pi;
end
DCM = getDCMFromEuler(init_roll, init_pitch, init_yaw);
%% states¡ª(q0,q1,q2,q3,vn,ve,vd,pn,pe,pd,Delta_Angle_bias_X_Y_Z,Delta_Vel_bias_X_Y_Z)
 states = zeros(16,1);
 % q0 q1 q2 q3
 q0 = 0.5*sqrt(1+DCM(1,1)+DCM(2,2)+DCM(3,3));
 q1 = 0.5*sqrt(1+DCM(1,1)-DCM(2,2)-DCM(3,3))*sign(DCM(3,2)-DCM(2,3));
 q2 = 0.5*sqrt(1-DCM(1,1)+DCM(2,2)-DCM(3,3))*sign(DCM(1,3)-DCM(3,1));
 q3 = 0.5*sqrt(1-DCM(1,1)-DCM(2,2)+DCM(3,3))*sign(DCM(2,1)-DCM(1,2));
 Quat = normalizeQuaternion([q0,q1,q2,q3]);
 % PosNE
 PosNE =  PosNE;
 % VelNED
 VelNED = VelNED;
 % Alt
 Alt_GPS = Alt_GPS;
 % Delta_Angle_bias_X_Y_Z
 DeAnglebias = [0;0;0];
 % Delta_Vel_bias_X_Y_Z
 DeVelbias = [0;0;0];
%% initStates
   states(1:4,:) = Quat;
   states(5:7,:) = VelNED(1,:);
   states(8:9,:) = PosNE(1,:);
   states(10)    = Alt_GPS(1);
   states(11:13,:) = DeAnglebias(1,:);
   states(14:16,:) = DeVelbias(1,:);
%% initCovariance
   P =calculateInitialData();
