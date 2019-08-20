%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:RunEKF.m
% date:2019/08/20
% author:YangYue
% email:qinguoxiaozi@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Run MSEKF
clear all;
clc;
close all;
%% initScript
initScript_Data;
%% start_run
  IMU_time = IMU(6,2); 
  angRate = IMU(6,3:5);
  accel = IMU(6,6:8);
  for i =7:IMULength 
         prevTimeIMU = IMU_time;
         IMU_time = IMU(i,2);
         dt = single((IMU_time - prevTimeIMU) / 10^6);% s
   % Convert IMU data to delta angles and velocities using trapezoidal integration
         prevAngRate = angRate;
         angRate=IMU(i,3:5);
         dAng = 0.5*(angRate + prevAngRate)*dt;
         prevAccel   = accel;
         accel = IMU(i,6:8);
         dVel = 0.5*(accel + prevAccel)*dt; 
%% TimeUpdate
   % UpdateStrapdownEquationsNED  
        [states,correctedDelAng,correctedDelVel,accNavMag]  = UpdateStrapdownEquationsNED(states, dAng,dVel,dt);
        Tbn = convertQuaternion2DCM(states(1:4,1));
    % CovariancePrediction  
          P  = CovariancePrediction(correctedDelAng,correctedDelVel,states,P,dt);
%% MeasurmentUpdate 
%  FuseVelPosNED-GPS
         if (indexGPS < GPSLength && GPS(indexGPS,2) > prevTimeIMU && GPS(indexGPS,2) < IMU_time)
          [states,P,innovation_GPS, varInnov_GPS]= FuseVelPosNED( states,P ,accNavMag,1,VelNED(indexGPS ,1:3),1,PosNE(indexGPS,1:2),1,Alt_GPS(indexGPS));
          indexGPS = indexGPS +1;
         end
%    FuseVelPosNED¡ªGPS(
%        if (indexGPS < GPSLength && GPS(indexGPS,2) > prevTimeIMU && GPS(indexGPS,2) < IMU_time)
%            [states,P,innovation_GPS, varInnov_GPS]= FuseVelPosNED( states, P  ,accNavMag,1,VelNED(indexGPS ,1:3),PosNE(indexGPS,1:2),0,0);% Fuse_GPS_VelNED_PosNE
%            indexGPS = indexGPS + 1; 
%        end
%    % FuseVelPosNED¡ªBARO
%      if (indexBARO < BAROLength && BARO (indexBARO ,2) > prevTimeIMU && BARO(indexBARO ,2) < IMU_time)
%         [states,P,innovation_GPS_BARO, varInnov_GPS_BARO]= FusePosD(states, P ,1,Alt_BARO(indexBARO));% Fuse_BARO_PosD
%         indexBARO = indexBARO + 1;
%      end
%  
%  %% FuseMAG
%          if(indexMAG < MAGLength && MAG(indexMAG,2) > prevTimeIMU  && MAG(indexMAG,2) < IMU_time)
%              [states,P,innovatoin_MAG,varInnov_MAG] =  FuseMagnetometer(states,P,1,magData(indexMAG,1:3),1); 
%              indexMAG = indexMAG+1;
%          end        
%% store states
   Cbn = convertQuaternion2DCM(states(1:4,1));
   [roll1, pitch1, yaw1] = getEulerFromDCM(Cbn);
   euler(:,6) = [init_roll, init_pitch , init_yaw];
   euler(:,i) = [roll1, pitch1, yaw1];

   velNEDEKF(:,i) = states(5:7,1);
   
   posNEDEKF(:,i) = states(8:10,1);     
   
   DeltaAngleBias(:,i) = states(11:13,1);
   
   DeltaVelBias(:,i) = states(14:16,1);    
 end
  
%% plotData
 plotData;
