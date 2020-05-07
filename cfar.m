% Implement 1D CFAR using lagging cells on the given noise and target scenario.

close all;

% Data_points
Ns = 1000;

% Generate random noise
s=abs(randn(Ns,1));

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100 ,200, 300, 700])=[8 9 4 11];

T = 12;% Training Cells
G = 4;% Guard Cells 

% Offset : Adding room above noise threshold for desired SNR 
offset=5;

% Vector to hold threshold values 
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% Slide window across the signal length
for i = 1:(Ns-(G+T+1)) 
    noise_sum = sum(s(i:i+T-1));
    
    threshold = (noise_sum/T)*offset;
    threshold_cfar = [threshold_cfar, {threshold}];
    
    signal = s(i+G+T);
    if signal < threshold
        signal = 0;
    end
    signal_cfar = [signal_cfar, {signal}];
    
        
end

% plot the filtered signal
plot (cell2mat(signal_cfar),'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(s);
hold on,plot(cell2mat(circshift(threshold_cfar,G)),'r--','LineWidth',2)
hold on, plot (cell2mat(circshift(signal_cfar,(T+G))),'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')