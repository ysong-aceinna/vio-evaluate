function [avar,tau] = myAllanVar(data, fs)
%% Allan variance. Already verified
% avar = myAllanVar(x,tau)
% avar: Allan variance
% data: measurement
% fs: sampling frequency, Hz

ts = 1/fs;
datalen = length(data);
tau = [5*ts:ts:10 10+10*ts:10*ts:floor(datalen/9)*ts];
% tau  = ts * 2.^(linspace(0, log2(floor(datalen/9)), 100));
% tau = ts * [1:floor(datalen/9)];

n = length(tau);
avar = zeros(n,1);

for ii = 1:n
%     ii/n
    nBins = floor(datalen/fs/tau(ii)); % number of bins
    nSamplePerBin = floor(datalen/nBins);
    tmp = reshape(data(1:nBins*nSamplePerBin),nSamplePerBin,nBins);
    tmp = mean(tmp,1);
    diff = tmp(2:end) - tmp(1:end-1);
    avar(ii) = 0.5/(nBins-1)*sum(diff.^2);
end






















