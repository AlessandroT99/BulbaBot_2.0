clear all, close all, clc

sim("BulbaBot_ControlLoop.slx")

figure, title("Path followed from the alghoritms to get the final position")
plot3(re.data(:,1),re.data(:,2),re.data(:,3),'r--','LineWidth',1), hold on, grid on
plot3(rd.data(end,1),rd.data(end,2),rd.data(end,3),'b.','MarkerSize',20)
plot3(re.data(1,1),re.data(1,2),re.data(1,3),'r.','MarkerSize',20)
legend("Real position","Expected position","Real starting position")

figure, title("System behavior")
coordinates = ['x','y','z'];
for i = 1:3
    subplot(2,3,i)
    yline(rd.data(i),'r-','LineWidth',1.5), hold on, grid on
    plot(re.time,re.data(:,i),'k--'), legend([coordinates(i),'_d'],[coordinates(i),'_e'])
    title([coordinates(i),' coordinate behavior'])
    subplot(2,3,3+i)
    plot(e.time,e.data(:,i),'k-'), hold on, grid on
    title([coordinates(i),' error behavior'])
end