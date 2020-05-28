a = importdata('example.txt');
data = a.data;
lidar1 = [];
lidar2 = [];
lidar3 = [];
radar1 = [];
radar2 = [];
radar3 = [];
for i=1:1800 
    if mod(i,6) == 1
        lidar1 = [lidar1 data(i)];
    end
    if mod(i,6) == 2
        lidar2 = [lidar2 data(i)];
    end
    if mod(i,6) == 3
        lidar3 = [lidar3 data(i)];
    end
    if mod(i,6) == 4
        radar1 = [radar1 data(i)];
    end
    if mod(i,6) == 5
        radar2 = [radar2 data(i)];
    end
    if mod(i,6) == 0
        radar3 = [radar3 data(i)];
    end
    
end
% 7.815 , lidar 5.991
figure;
subplot(3,2,1)
plot(lidar1)
yline(5.991)
title('Lidar NIS for car1')
grid on
subplot(3,2,3)
plot(lidar2)
yline(5.991)
title('Lidar NIS for car2')
grid on
subplot(3,2,5)
plot(lidar3)
yline(5.991)
title('Lidar NIS for car3')
grid on

subplot(3,2,2)
plot(radar1)
yline(7.815)
title('Radar NIS for car1')
grid on
subplot(3,2,4)
plot(radar2)
yline(7.815)
title('Radar NIS for car2')
grid on
subplot(3,2,6)
plot(radar3)
yline(7.815)
title('Radar NIS for car3')
grid on
    
