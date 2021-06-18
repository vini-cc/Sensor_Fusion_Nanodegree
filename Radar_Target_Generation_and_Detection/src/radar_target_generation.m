% Name: Vinícius Conti da Costa
% Project for Udacity Sensor Fusion Engineer Nanodegree.
% 1st try.
% June 18, 2021.

%% Warmup
clear;
close all;
clc;

%% Variables (ranges and targets)
freq = 77e9; % frequency
rangeRes = 1; % range resolution
maxRange = 200; % maximum range
maxVelocity = 70; % maximum velocity
velocityRes = 3; % velocity resolution
sweepTime = 5.5; % sweep time
range_target = 100; % range target
c = 3e8; % speed of light
velocity = 60; % velocity target

%% Initial Equations (FMCW Bandwidth, Chirp Time and Slope)
B = c/(2*rangeRes); % bandwidth
Tchirp = (sweepTime *  2 * maxRange) / c; % chirp time
slope = B/Tchirp; % slope

%% Numbers for Doppler estimation (chirps in one sequence) and numbers
%%of samples in one chirp.
Nd = 128;
Nr = 1024;

%% TimeStamp
t=linspace(0,Nd*Tchirp,Nr*Nd);

%% Creating vectors for Tx, Rx and Mix
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%% Vectors for range_covered and time_delay
r_t=zeros(1,length(t)); % range covered
td=zeros(1,length(t)); % time delay

%% Generating sinal and target simulation
for i=1:length(t)
    r_t(i) = range_target + (velocity * t(i));
    td(i) = (2 * r_t(i)) / c;
    Tx(i) = cos(2 * pi * (freq * (t(i)) + (0.5 * slope * t(i) * t(i))));
    Rx(i) = cos(2 * pi * (freq * (t(i) - td(i)) + (0.5 * slope * (t(i) ...
        - td(i))^2)));
    Mix(i) = Tx(i) .* Rx(i);
end

%% reshape vector (Nr x Nd)
Mix = reshape(Mix, [Nr, Nd]); % Nr = size of range.
                              % Nd = size of Doppler FFT.

%% FFT1
signal_fft1 = fft(Mix, Nr); % Running FFT on beat signal.
signal_fft1 = signal_fft1 ./ Nr; 
signal_fft1 = abs(signal_fft1); % Normalize

single_fft1 = signal_fft1(1:Nr/2);

%% plot 1
figure('Name', 'Range from FFT');
plot(single_fft1);
axis([0 200 0 1]);
title('Range from first FFT1');
ylabel('Normalized Amplitude');
xlabel('Range');

%% Range Doppler map generation
Mix = reshape(Mix, [Nr, Nd]);
signal_fft2 = fft2(Mix,Nr,Nd);
signal_fft2=signal_fft2(1:Nr/2,1:Nd);
signal_fft2=fftshift(signal_fft2);

RDM = abs(signal_fft2);
RDM = 10 * log10(RDM);

%% plot 2
doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200, 200, Nr/2) * ((Nr/2)/400);
figure('Name', 'Range from FFT2');
surf(doppler_axis, range_axis, RDM);
colormap(jet);
colorbar;
title('Amplitude x Range FFT2');
xlabel('Speed');
ylabel('Range');
zlabel('Amplitude');

%% CFAR Training cells and Guard Cells
Tr = 8;
Td = 2;
Gr = 2;
Gd = 4;
offset = 1.6;

%% CFAR Implementation
noise_level = zeros(1,1);
n_cell = (2 * (Td + Gd + 1) * 2 * (Tr + Gr + 1) - (Gr * Gd) - 1);
RDM = RDM/max(max(RDM));

for i = Tr + Gr + 1:(Nr/2) - (Gr + Tr)
    for j = Td + Gd + 1:Nd - (Gd + Td)
        noise_level = zeros(1, 1);
        for p = i - (Tr + Gr) : i + (Tr + Gr)
            for q = j - (Td + Gd) : j + (Td + Gd)
                if (abs(i - p) > Gr || abs(j - q) > Gd)
                    noise_level = noise_level + db2pow(RDM(p,q));
                end
            end
        end
        
        threshold = noise_level/n_cell;
        threshold = pow2db(threshold) + offset;   
        
        signal = RDM(i,j);        
        if (signal <= threshold)
            RDM(i,j) = 0;
        else
            RDM(i,j) = 1;
        end     
    end
end

RDM(1:(Tr+Gr), :) = 0;
RDM(end-Tr-Gr:end, :) = 0;
RDM(:, 1:(Td+Gd)) = 0;
RDM(:, end-Td-Gd:end) = 0;

%% plot 3
figure('Name','CA-CFAR RDM');
surf(doppler_axis,range_axis,RDM);
colormap(jet);
colorbar;
% shading interp;
title( 'CA-CFAR RDM');
xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');







