function [pointList, cfarRD] = dynamicCfar(parameter, accumulateRD)

    [rangeLen, dopplerLen] = size(accumulateRD);

    % Doppler dimension
   
    dopplerWinGuardLen = parameter.dopplerWinGuardLen;
    dopplerWinTrainLen = parameter.dopplerWinTrainLen;

    dopplerCfarList = [];
    cfarRDdoppler = zeros(rangeLen, dopplerLen);

    % Calculate threshold factor based on PFA
    cfar_N = parameter.dopplerWinTrainLen;
    cfar_pfa = parameter.cfar_pfa;
    cfar_a = cfar_N * ((cfar_pfa)^(-1/cfar_N) - 1);

    % Doppler CFAR
    for rangeIdx = 1:rangeLen
        for dopplerIdx = 1:dopplerLen
            %leftCell = accumulateRD(rangeIdx, dopplerIdx:dopplerIdx + dopplerWinTrainLen - 1);
            leftCell = accumulateRD(rangeIdx, max(1, dopplerIdx - dopplerWinTrainLen + 1) : dopplerIdx);
            %rightCell = accumulateRD(rangeIdx, dopplerIdx + dopplerWinGuardLen + dopplerWinTrainLen: dopplerIdx + 2 * dopplerWinGuardLen + dopplerWinTrainLen - 1);
            rightCell = accumulateRD(rangeIdx, dopplerIdx + dopplerWinGuardLen + dopplerWinTrainLen : min(dopplerIdx + 2 * dopplerWinGuardLen + dopplerWinTrainLen - 1, dopplerLen));

            leftNoise = mean(leftCell);
            rightNoise = mean(rightCell);

            % Calculate dynamic threshold based on PFA and apply it to each cell
            threshold = cfar_a * ((leftNoise + rightNoise) / 2);
            if accumulateRD(rangeIdx, dopplerIdx) > threshold
                dopplerCfarList = [dopplerCfarList, dopplerIdx];
                cfarRDdoppler(rangeIdx, dopplerIdx) = accumulateRD(rangeIdx, dopplerIdx);
            end
        end
    end

    dopplerCfarList = unique(dopplerCfarList);

    % Range dimension
  
    rangeWinGuardLen = parameter.rangeWinGuardLen;
    rangeWinTrainLen = parameter.rangeWinTrainLen;

    rangeCfarList = [];
    cfarRD = zeros(rangeLen, dopplerLen);

    % Range CFAR using the Doppler CFAR results
    for dopplerIdx = dopplerCfarList
        for rangeIdx = 1:rangeLen
            %upCell = accumulateRD(rangeIdx:rangeIdx + rangeWinTrainLen - 1, dopplerIdx);
            %downCell = accumulateRD(rangeIdx + rangeWinGuardLen + rangeWinTrainLen: rangeIdx + rangeWinGuardLen + 2 * rangeWinTrainLen - 1, dopplerIdx);
            upCell = accumulateRD(max(1, rangeIdx - rangeWinTrainLen + 1) : rangeIdx, dopplerIdx);
            downCell = accumulateRD(rangeIdx + rangeWinGuardLen + 1 : min(rangeIdx + rangeWinGuardLen + rangeWinTrainLen - 1, rangeLen), dopplerIdx);

            upNoise = mean(upCell);
            downNoise = mean(downCell);

            % Calculate dynamic threshold based on PFA and apply it to each cell
            threshold = cfar_a * ((upNoise + downNoise) / 2);
            if accumulateRD(rangeIdx, dopplerIdx) > threshold
                rangeCfarList = [rangeCfarList, [rangeIdx; dopplerIdx]];
                cfarRD(rangeIdx, dopplerIdx) = accumulateRD(rangeIdx, dopplerIdx);
            end
        end
    end

    pointList = rangeCfarList;
end
