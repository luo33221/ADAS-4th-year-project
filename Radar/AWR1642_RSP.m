clc;clear all;close all;

%% set parameters
parameter  = generateParameter();

%% %% reading data
fileName1 = '10m.bin';
fid1 = fopen(fileName1,'r');
adcDataRow0 = fread(fid1, 'int16');

numADCSamples = parameter.Samples; % number of ADC samples per chirp
numADCBits = 16;     % number of ADC bits per sample
numRx = parameter.rxNum;           % number of receivers
numTx =parameter.txNum;
numLanes = 2;        % do not change. number of lanes is always 2
isReal = 0;          % set to 1 if real only data, 0 if complex data0
chirpLoop = parameter.Chirps;
Frames =800;% no. of frames
allValues = zeros(0, 2);
%% Reading data ploting by frames
count =0; %counter

for frame =1:1:Frames %frame setting

    %Read data by frames
    adcDataRow = adcDataRow0(((frame-1)*parameter.Samples*parameter.Chirps*numRx*numTx*2+1):frame*parameter.Samples*parameter.Chirps*numRx*numTx*2,:);

    if numADCBits ~= 16
        l_max = 2^(numADCBits-1)-1;
        adcDataRow(adcDataRow > l_max) = adcDataRow(adcDataRow > l_max) - 2^numADCBits;
    end

    fileSize = size(adcDataRow, 1);
    PRTnum = fix(fileSize/(numADCSamples*numRx));
    fileSize = PRTnum * numADCSamples*numRx;
    adcData = adcDataRow(1:fileSize);

    if isReal
        numChirps = fileSize/numADCSamples/numRx;
        LVDS = zeros(1, fileSize);
        LVDS = reshape(adcData, numADCSamples*numRx, numChirps);%create column for each chirp
        LVDS = LVDS.';                                          %each row is data from one chirp
    else
        
        numChirps = fileSize/2/numADCSamples/numRx;     %/2
        LVDS = zeros(1, fileSize/2);
        counter = 1;
        for i=1:4:fileSize-1
            LVDS(1,counter) = adcData(i) + sqrt(-1)*adcData(i+2);
            LVDS(1,counter+1) = adcData(i+1)+sqrt(-1)*adcData(i+3); counter = counter + 2;
        end
        
        LVDS = reshape(LVDS, numADCSamples*numRx*numTx, numChirps);% create column for each chirp
        LVDS = LVDS.';%each row is data from one chirp
    end
    
    % combine data
    adcData = zeros(numRx*numTx,numChirps*numADCSamples);
    for row = 1:numRx*numTx         
        for i = 1: numChirps   
            adcData(row, (i-1)*numADCSamples+1:i*numADCSamples) = LVDS(i, (row-1)*numADCSamples+1:row*numADCSamples);
        end
    end
    clear LVDS
    rawData=reshape(adcData',[numADCSamples,chirpLoop,numRx*numTx]);
    
    %single chirp receiving signal
     firstChirp = rawData(:,1,1);
     figure(1);
     plot(real(firstChirp));
     hold on;
     plot(imag(firstChirp));
     grid on
     xlabel('sampling number'); ylabel('Amplitude');title('Raw data');%first chirp。
     %saveas(gcf, fullfile('C:\Users\LUO\Desktop\code\code', ['figure_1_frame_', num2str(frame), '.png']))   
    %% Signal processing
    rangeRes     = parameter.c / (2 * parameter.BandwidthValid); % range resolution valid bandwidth
    rangeIndex   = (0:parameter.rangeBin-1) * rangeRes;
    speedRes     = parameter.lambda / (2 * parameter.dopplerBin * parameter.Tr);
    dopplerIndex = (-parameter.dopplerBin/2:1:parameter.dopplerBin/2 - 1) * speedRes;
    angleRes     = parameter.lambda / (parameter.virtualAntenna * parameter.dx) * 180 / pi;
    angleIndex   = (-parameter.virtualAntenna/2:1:parameter.virtualAntenna/2 - 1) * angleRes;
        
 %% 1D FFT
     fft1dData    = fft(rawData(:,:,1));
     figure(2);
     mesh(dopplerIndex,rangeIndex,db(abs(fft1dData)));
     ylabel('Distance(m)'); xlabel('Chirp（N）'),zlabel('Amplitude(dB)');
     grid on
     title('Distance axis FFT');
     %saveas(gcf, fullfile('C:\Users\LUO\Desktop\code\code', ['figure_2_frame_', num2str(frame), '.png']))  
   
    %% remove zeros
     rawDataZeros = zeros(parameter.rangeBin,parameter.dopplerBin,numRx*numTx);
     for n=1:numRx*numTx
         avg = (sum(rawData(:,:,n),2)/parameter.dopplerBin).';
         for chirp=1:parameter.dopplerBin
             rawDataZeros(:,chirp,n) = rawData(:,chirp,n).'-avg;
        end
     end
    %% MTI 
    rawDataZeros= zeros(parameter.rangeBin,parameter.dopplerBin,numRx*numTx);
    for m =1:numRx*numTx
        for n =1:parameter.rangeBin-1
            rawDataZeros (n,:,m) = rawData(n+1,:,m)-rawData(n,:,m);
        end
    end

    for m =1:numRx*numTx
        for n =1:parameter.dopplerBin-1
            rawDataZeros (:,n,m) = rawData(:,n+1,m)-rawData(:,n,m);
        end
    end

    %% 2D FFT
    rangebinNum   = size(rawData,1);
    dopplerbinNum = size(rawData,2);
    channelNum    = size(rawData,3);
    fft2dDataPower= zeros(size(rawData));
    fft2dDataDB   = zeros(size(rawData));
    fftRADataPower= zeros(size(rawData));
    for chanId = 1:1:channelNum
        fft2dDataPower(:,:,chanId) = RDfftMatrix(rawDataZeros(:,:,chanId));
        fft2dDataPower(:,:,chanId) = rot90(squeeze(fft2dDataPower(:,:,chanId)),2);
    end

     A =squeeze(fft2dDataPower(:,:,chanId));
     figure(3);
     mesh(dopplerIndex',rangeIndex,20*log10(abs(A)));
     view(2);
     xlabel('Speed(m/s)'); ylabel('Distance(m)'); zlabel('Amplitude(dB)');
     title(['range-doppler:',num2str(frame)]);
     %saveas(gcf, fullfile('C:\Users\LUO\Desktop\code\code', ['figure_3_frame_', num2str(frame), '.png']))  
    %% Non-Coherent Integration
    accumulateRD = chan_Accumulate((fft2dDataPower));

    figure(4);
    mesh(dopplerIndex',rangeIndex,(accumulateRD));
    view(2);
    xlabel('speed(m/s)'); ylabel('distance(m)'); zlabel('amplitude');
    title(['Channel Accumulation No',num2str(frame),'frame']);
    %saveas(gcf, fullfile('C:\Users\LUO\Desktop\code\code', ['figure_4_frame_', num2str(frame), '.png']))  
    %% CFAR
    cfarParameter = generateCfarParameter(); %generate cfar parameter
    [pointList,cfarRD] = dynamicCfar(cfarParameter,20*log10(accumulateRD));
     figure(5);
     mesh(dopplerIndex',rangeIndex,cfarRD);
     view(2)
     xlabel('Speed(m/s)'); ylabel('Distance(m)'); zlabel('Amplitude');
     title('CFAR');
     %saveas(gcf, fullfile('C:\Users\LUO\Desktop\code\code', ['figure_5_frame_', num2str(frame), '.png']))  
 %% peakSearch
    [RD_pearkSearch,peakSearchList] = peakSearch(cfarRD,pointList);
     figure(6);
     mesh(dopplerIndex',rangeIndex,RD_pearkSearch);
     view(2)
     xlabel('Speed(m/s)'); ylabel('Distance(m)'); zlabel('Amplitude');
     title('Peaksearch');
     %saveas(gcf, fullfile('C:\Users\LUO\Desktop\code\code', ['figure_6_frame_', num2str(frame), '.png']))  
     peakSearchList = pointList;
    detectPointNum = size(peakSearchList,2);

    if detectPointNum==0
        continue;
    end
    
     %% DOAestimation
    ang_ax = -60:1:60; % angle axis
    doa_abs2 =  zeros(length(ang_ax),numADCSamples);
    music_spectrum =zeros(length(ang_ax),numADCSamples);

    for targetIdx = 1:detectPointNum

        rangeBin = peakSearchList(1,targetIdx);
        speedBin = peakSearchList(2,targetIdx);
        range = (rangeBin - 1) * rangeRes;
        speed = (speedBin - parameter.dopplerBin/2 - 1) * speedRes;
    
        ant = squeeze(fft2dDataPower(rangeBin,speedBin,:));
        compCoff = generateCompCoff(parameter,speedBin);  %doppler compcoff
      
        if 1 
            ant = ant .* compCoff;
        end

      %% MUSIC
        A =ant;
        d = 0.5;
        M = 50;  % number of snapshots
        for k=1:length(ang_ax)
            a1(:,k)=exp(1i*2*pi*(d*(0:parameter.txNum*parameter.rxNum-1)'*sin(ang_ax(k).'*pi/180)));
        end

        %set M larger value to get more accurate estimation
        Rxx = zeros(parameter.txNum*parameter.rxNum,parameter.txNum*parameter.rxNum);
        for m = 1:M
            Rxx = Rxx + 1/M * (A*A');
        end
    
        [Q,D] = eig(Rxx); % Q: eigenvectors (columns), D: eigenvalues
        [D, I] = sort(diag(D),'descend');
        Q = Q(:,I);       % Sort the eigenvectors to put signal eigenvectors first
        Qs = Q(:,1);      % Get the signal eigenvectors
        Qn = Q(:,2:end);  % Get the noise eigenvectors
        
        for k=1:length(ang_ax) %angel search
            theta(k)=((a1(:,k)'*a1(:,k))/(a1(:,k)'*(Qn*Qn')*a1(:,k)));
        end

        [~,index] = max(theta);
        angle = ang_ax(index);
        music_spectrum(index,rangeBin) = angle;
        doa_abs2 = doa_abs2 +(music_spectrum);
        allValues = [allValues;frame, range, speed , angle];
        pointcloud(targetIdx,:) = [range,speed,angle];% pointcloud data
    end
   
        
       
    
      figure(7);
      A = 20*log10(abs((doa_abs2)))';
      imagesc(-ang_ax,rangeIndex,A);
      view(2);
      axis xy
      grid on
      xlabel('angle（°）');
      ylabel('distance（m）');
      title(['MUSICresult：',num2str(frame),'frame']);
      %saveas(gcf, fullfile('C:\Users\LUO\Desktop\code\code', ['figure_7_frame_', num2str(frame), '.png']))  
  
     figure(8);
     plot(-pointcloud(:,3),pointcloud(:,1),'.');
     xlim([-90,90]);ylim([0,20]);
     xlabel('angle（°）'); grid on 
     ylabel('distances（m）');title('pointcould ');
     %saveas(gcf, fullfile('C:\Users\LUO\Desktop\code\code', ['figure_8_frame_', num2str(frame), '.png']))  
    
end
% Specify the filename
filename = 'Radar-data.csv';

% Write the matrix to the CSV file
writematrix(allValues, filename);

disp(['Data has been saved to ' filename]);