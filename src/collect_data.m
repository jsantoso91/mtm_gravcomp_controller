clear all
close all
clc

rosshutdown
rosinit('130.215.170.143');
joint_state_sub = rossubscriber('mtm/joint_states',rostype.sensor_msgs_JointState);
q_des = [0 0 0 0 0 0 0];
n = 1;
tic
while(1)
    scan = receive(joint_state_sub);
    q_values(n,:) = [ scan.Position(2:3)', scan.Position(1), scan.Position(5:-1:4)'...
        scan.Position(7:-1:6)'];
    q_error(n,:) = q_values(n,:) - q_des;
    n = n+1;
end
toc

figure(1)
plot(1:length(q_values),q_values2,'LineWidth',2)
title('q_{values} progression','FontSize',20)
xlabel('number of iteration','FontSize',20)
ylabel('q_{values} (rad)','FontSize',20)
legend('joint1','joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7')
