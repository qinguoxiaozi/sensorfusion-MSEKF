%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file:plotData.m
% date:2019/08/20
% author:YangYue
% email:qinguoxiaozi@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plotData
% roll
figure(1)
hold on;
grid on;
plot(EKF1(:,2)/10^6,EKF1(:,3),'r');
plot(IMU(:,2)/10^6,euler(1,:)*180/pi,'b');
xlabel('Time/s');ylabel('Degree/。');
legend('Pixhawk','MSEKF');
title('roll');
box on;

% pitch
figure(2)
hold on
grid on
plot(EKF1(:,2)/10^6,EKF1(:,4),'r');
plot(IMU(:,2)/10^6,euler(2,:)*180/pi,'b');
xlabel('Time/s');ylabel('Degree/。');
legend('Pixhawk','MSEKF');
title('pitch');
box on

% yaw
figure(3)
hold on
grid on
plot(EKF1(:,2)/10^6,EKF1(:,5),'r');
plot(IMU(:,2)/10^6,euler(3,:)*180/pi,'b');
xlabel('Time/s');ylabel('Degree/。');
legend('Pixhawk','MSEKF');
title('yaw');
box on

% velN
figure(4);
hold on
grid on
plot(EKF1(:,2)/10^6,EKF1(:,6),'r');
plot(IMU(:,2)/10^6,velNEDEKF(1,:),'b');
xlabel('Time/s');ylabel('Vel/(m/s)');
legend('Pixhawk','MSEKF');
title('VN');
box on 

% velE
figure(5)
hold on
grid on
plot(EKF1(:,2)/10^6,EKF1(:,7),'r');
plot(IMU(:,2)/10^6,velNEDEKF(2,:),'b');
xlabel('Time/s');ylabel('Vel/(m/s)');
legend('Pixhawk','MSEKF');
title('VE');
box on 

% velD
figure(6);
hold on
grid on
plot(EKF1(:,2)/10^6,EKF1(:,8),'r');
plot(IMU(:,2)/10^6,velNEDEKF(3,:),'b');
xlabel('Time/s');ylabel('Vel/(m/s)');
legend('Pixhawk','MSEKF');
title('VD');
box on 

% posN
figure(7);
hold on
grid on
plot(EKF1(:,2)/10^6,EKF1(:,9),'r');
plot(IMU(:,2)/10^6,posNEDEKF(1,:),'b');
xlabel('Time/s');ylabel('Pos/m');
legend('Pixhawk','MSEKF');
title('PN');
box on 

% posE
figure(8);
hold on
grid on
plot(EKF1(:,2)/10^6,EKF1(:,10),'r');
plot(IMU(:,2)/10^6, posNEDEKF(2,:),'b');
xlabel('Time/s');ylabel('Pos/m');
legend('Pixhawk','MSEKF');
title('PE');
box on 

% posD
figure(9);
hold on
grid on
plot(EKF1(:,2)/10^6,EKF1(:,11),'r');
plot(IMU(:,2)/10^6, (posNEDEKF(3,:)-12.5),'b');
xlabel('Time/s');ylabel('Pos/m');
legend('Pixhawk','MSEKF');
title('PD');
box on 

% deltaAngleBias
figure(10);
hold on
grid on
plot(IMU(:,2)/10^6,DeltaAngleBias(1,:),'r');
plot(IMU(:,2)/10^6,DeltaAngleBias(2,:),'b');
plot(IMU(:,2)/10^6,DeltaAngleBias(3,:),'g');
xlabel('Time/s');ylabel('Degree/。');
legend('x','y','z');
title('deltaAngleBias');
box on 

% deltaVelBias
figure(11);
hold on
grid on
plot(IMU(:,2)/10^6,DeltaVelBias(1,:),'r');
plot(IMU(:,2)/10^6,DeltaVelBias(2,:),'b');
plot(IMU(:,2)/10^6,DeltaVelBias(3,:),'g');
xlabel('Time/s');ylabel('deltaVel/(m/s)');
legend('x','y','z');
title('deltaVelBias');
box on 

% track 
figure(12);
plot3(EKF1(:,9),EKF1(:,10),EKF1(:,11),'r','linewidth',2);hold on;grid on;
xlabel('North');ylabel('Earth');zlabel('Down');legend('EKF(uint:m)');
title('flight path');
box on;


