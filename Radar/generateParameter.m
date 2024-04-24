% set parameter
function parameter = generateParameter()
 
    parameter.c = 3e8;             %lightspeed
    parameter.stratFreq = 77e9;    %start Freq

    parameter.Tr = 104e-6;          %period
    parameter.Samples = 256;        %No. chirps
    parameter.Fs = 2.5e6;           %sample rate

    parameter.rangeBin = parameter.Samples ;      %rangebin
    parameter.Chirps = 200;                       %chirpno
    parameter.dopplerBin = parameter.Chirps;      %dopplerbin

    parameter.Slope = 37.513e12;       %chirpslop
    parameter.Bandwidth = parameter.Slope * parameter.Tr ; %BW
    parameter.BandwidthValid = (parameter.Samples/parameter.Fs)*parameter.Slope; %balidBW
    parameter.centerFreq = parameter.stratFreq + parameter.Bandwidth / 2; %center Freq
    parameter.lambda = parameter.c / parameter.centerFreq; %wave lenth

    parameter.txAntenna = ones(1,1); %RT
    parameter.rxAntenna = ones(1,4); %RX
    parameter.txNum = length(parameter.txAntenna);
    parameter.rxNum = length(parameter.rxAntenna);
    parameter.virtualAntenna = length(parameter.txAntenna) * length(parameter.rxAntenna);
    
    parameter.dz = parameter.lambda / 2; 
    parameter.dx = parameter.lambda / 2; 

end
