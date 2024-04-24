function accumulateRD = chan_Accumulate(fft2dDataDB)
    [rangeBin,dopplerBin,channelNum] = size(fft2dDataDB);
    accumulateRD = zeros(rangeBin,dopplerBin);

    for channelId = 1:channelNum
        accumulateRD = accumulateRD + (abs(squeeze(fft2dDataDB(:,:,channelId))));
    end
end
