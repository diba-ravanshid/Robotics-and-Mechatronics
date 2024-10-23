close all
figure(1)
subplot(2,2,1)
%plot(out.f1.Time,out.f1.Data);
plot(0:Ts_M:tf,TAU(1,:));
xlabel('t (s)')
ylabel('f1 (n.m)')

subplot(2,2,2)
% plot(out.tau2.Time,out.tau2.Data);
 plot(0:Ts_M:tf,TAU(2,:));
xlabel('t (s)')
ylabel('\tau2 (n.m)')

subplot(2,2,3)
% plot(out.tau3.Time,out.tau3.Data);
plot(0:Ts_M:tf,TAU(3,:));
xlabel('t (s)')
ylabel('\tau3 (n.m)')

subplot(2,2,4)
% plot(out.f4.Time,out.f4.Data);
plot(0:Ts_M:tf,TAU(4,:));
xlabel('t (s)')
ylabel('f4 (n.m)')