clear classes;clc;
format compact
%% Init
FrameCount = 200;
FramePerMER = 2; % 几帧测一次MER(1 2 4 5 10 20)
SNRs = linspace(4,30,FrameCount+1); % 线性增大的SNR数组
SNRs = SNRs(1:FrameCount);
MERs = zeros(1,FrameCount/FramePerMER+1); % 记录MER（作图）
Mods = zeros(1,FrameCount); % 记录调制阶数M（作图）
BERs = zeros(1,FrameCount); % 记录BER（作图）
MER = SNRs(1);
ModAdpThres = [10 20]; %三种调制方式的MER的阈值(dB)
% My Msg
MyMsgStrLength = 240; % 随机字符串字符数
MyMsgLength = MyMsgStrLength*7;
MyCharSet = 'abcdefghijklmnopqrstuvwxyz0123456789';
MyMsgStr = MyCharSet(randi([1 36],1,MyMsgStrLength)); % 随机字符串
MyMsgBits = de2bi(int8(MyMsgStr),7,'left-msb')';
MyMsgBits = reshape(MyMsgBits,MyMsgStrLength*7,1); % 随机比特

UpsamplingFactor = 4;
DownsamplingFactor = 2; 
PostFilterOversampling = UpsamplingFactor/DownsamplingFactor;
SampleRate = 2e5; 

FrameSize = 111; % 一帧的总符号数
BarkerLength = 13; % Barker码符号数

ScramblerBase = 2;
ScramblerPolynomial = [1 1 1 0 1];
ScramblerInitialConditions = [0 0 0 0];
RxBufferedFrames = 10;
RaisedCosineFilterSpan = 10;

CoarseCompFrequencyResolution = 25; 
K = 1;A = 1/sqrt(2);
PhaseRecoveryLoopBandwidth = 0.01;
PhaseRecoveryDampingFactor = 1;
TimingRecoveryLoopBandwidth = 0.01;
TimingRecoveryDampingFactor = 1;
TimingErrorDetectorGain = 2.7*2*K*A^2+2.7*2*K*A^2; 
BarkerCode = [+1; +1; +1; +1; +1; -1; -1; +1; +1; -1; +1; -1; +1];    
ModulatedHeader = sqrt(2)/2 * (-1-1i) * BarkerCode;
Rolloff = 0.5;
TransmitterFilterCoefficients = rcosdesign(Rolloff, ...
    RaisedCosineFilterSpan, UpsamplingFactor);
ReceiverFilterCoefficients = rcosdesign(Rolloff, ...
    RaisedCosineFilterSpan, UpsamplingFactor);
%% Flags
useScopes = true;
printOption = true;
%% Objects
% Tx
pScrambler = comm.Scrambler(ScramblerBase,ScramblerPolynomial, ...
    ScramblerInitialConditions); % 扰码对象
p4QAMModulator  = comm.RectangularQAMModulator(4,'BitInput',true, ...
    'NormalizationMethod', 'Average power');
p16QAMModulator  = comm.RectangularQAMModulator(16,'BitInput',true, ...
    'NormalizationMethod', 'Average power');
p64QAMModulator  = comm.RectangularQAMModulator(64,'BitInput',true, ...
    'NormalizationMethod','Average power');
pTransmitterFilter = dsp.FIRInterpolator(UpsamplingFactor, ...
    TransmitterFilterCoefficients); % 脉冲成型+上采样滤波器对象
% Channel
% Rx
p4QAMDemodulator = comm.RectangularQAMDemodulator(4,'BitOutput',true, ...
    'NormalizationMethod', 'Average power');
p16QAMDemodulator = comm.RectangularQAMDemodulator(16,'BitOutput',true, ...
    'NormalizationMethod', 'Average power');
p64QAMDemodulator = comm.RectangularQAMDemodulator(64,'BitOutput',true, ...
    'NormalizationMethod', 'Average power');
pAGC = comm.AGC;
pRxFilter = dsp.FIRDecimator( ...
    'Numerator', ReceiverFilterCoefficients, ...
    'DecimationFactor', DownsamplingFactor); % 匹配滤波+下采样滤波器对象
pTimingRec = comm.SymbolSynchronizer( ...
    'TimingErrorDetector',     'Zero-Crossing (decision-directed)', ...
    'SamplesPerSymbol',        PostFilterOversampling, ...
    'DampingFactor',           TimingRecoveryDampingFactor, ...
    'NormalizedLoopBandwidth', TimingRecoveryLoopBandwidth, ...
    'DetectorGain',            TimingErrorDetectorGain);   % 符号同步对象
pFrameSync = FrameFormation( ...
    'OutputFrameLength',      FrameSize, ...
    'PerformSynchronization', true, ...
    'FrameHeader',            ModulatedHeader); % 帧同步对象
pCorrelator = dsp.Crosscorrelator;
pDescrambler = comm.Descrambler(ScramblerBase,ScramblerPolynomial, ...
    ScramblerInitialConditions);
pErrorRateCalc = comm.ErrorRate;  % 计算BER对象
pMER = comm.MER; % 计算MER对象
% Scopes
pRxConstellation = comm.ConstellationDiagram( ...
        'ShowGrid', true, ...
        'Position', figposition([1.5 72 17 20]), ...                    
        'SamplesPerSymbol', 2, ...                    
        'YLimits', [-1.5 1.5], ...
        'XLimits', [-1.5 1.5], ...
        'Title', 'After Raised Cosine Rx Filter', ...
        'ReferenceConstellation',[]); % 星座图对象
%% Run
% Msg
MyMsgCount = 0; % 文本指针，为上一帧数据发送的最后一个字符
msg = zeros(196,1);
for count = 1:FrameCount
    % Tx
    msg_pre = msg;
    % Modulation Adaptive
    if MER<ModAdpThres(1)
        M = 4; % 调制阶数
        MessageLength = (FrameSize-BarkerLength)*2; % 信息比特数
        msgBin = [MyMsgBits;MyMsgBits]; % 循环发送文本
        msgBin = msgBin(MyMsgCount+1:MyMsgCount+MessageLength,1); 
        % 接着上一帧发
        msg = double(msgBin);
        scrambledData = step(pScrambler, msg); % 扰码
        modulatedData = step(p4QAMModulator, scrambledData); %信息调制
    elseif MER<ModAdpThres(2)
        M = 16;
        MessageLength = (FrameSize-BarkerLength)*4;
        msgBin = [MyMsgBits;MyMsgBits];
        msgBin = msgBin(MyMsgCount+1:MyMsgCount+MessageLength,1);
        msg = double(msgBin);
        scrambledData = step(pScrambler, msg);
        modulatedData = step(p16QAMModulator, scrambledData);
    else
        M = 64;
        MessageLength = (FrameSize-BarkerLength)*6;
        msgBin = [MyMsgBits;MyMsgBits];
        msgBin = msgBin(MyMsgCount+1:MyMsgCount+MessageLength,1);
        msg = double(msgBin);
        scrambledData = step(pScrambler, msg);
        modulatedData = step(p64QAMModulator, scrambledData);
    end
    MyMsgCount = mod(MyMsgCount+MessageLength,MyMsgStrLength*7);
    transmittedData = [ModulatedHeader; modulatedData]; % 组帧
    transmittedSignal=step(pTransmitterFilter,transmittedData);%成型滤波+上采样
    % Channel
    pAWGNChannel = comm.AWGNChannel( ...
        'NoiseMethod','Signal to noise ratio (SNR)', ...
        'SNR', SNRs(count)); % 信道
    corruptSignal = step(pAWGNChannel, transmittedSignal); % 经过信道
    % Rx
    AGCSignal=1/sqrt(UpsamplingFactor)*step(pAGC,corruptSignal);%自动能量控制
    RCRxSignal = step(pRxFilter, AGCSignal); % 匹配滤波+下采样
    [timingRecSignal,~] = step(pTimingRec, RCRxSignal); % 符号同步
    [symFrame,isFrameValid] = step(pFrameSync, timingRecSignal); % 帧同步
    if isFrameValid
        phaseEst = round(angle(mean(conj(ModulatedHeader) ...
            .* symFrame(1:BarkerLength)))*2/pi)/2*pi;
        phShiftedData = symFrame .* exp(-1i*phaseEst);
        HeaderSymbols = phShiftedData(1:13); % 头部符号
        if mod(count,FramePerMER) == 0
            MER = pMER(ModulatedHeader,HeaderSymbols); % 计算MER
        end
        MsgSymbols = phShiftedData(13+1:13+MessageLength/log2(M)); % 信息符号
        if M == 4
            demodOut = step(p4QAMDemodulator, MsgSymbols); % 解调
        elseif M == 16
            demodOut = step(p16QAMDemodulator, MsgSymbols); 
        else % M == 64
            demodOut = step(p64QAMDemodulator, MsgSymbols);
        end
        deScrData = step(pDescrambler,demodOut);
        if printOption
            disp(bits2ASCII(msg_pre)); % 显示发送信息
            disp(bits2ASCII(deScrData)); % 显示接收信息
        end
        if size(msg_pre) == size(deScrData)
            BER = step(pErrorRateCalc, msg_pre, deScrData); % 计算BER
        else 
            BER = [0.5 0 0];
        end
        Mods(count) = M;
        BERs(count) = BER(1);
        MERs(count) = MER;
    end
    if useScopes % 画这一帧数据的星座图，并调整参考星座点
        if M == 4
            pRxConstellation.ReferenceConstellation = ...
                [sqrt(1/2)*(1+1i)*[-1 1], ...
                reshape((repmat([-1 1],2,1)-1i* ...
                repmat([-1 1],2,1)')*sqrt(1/2),1,4)];
        elseif M == 16
            pRxConstellation.ReferenceConstellation = ...
                [sqrt(1/2)*(1+1i)*[-1 1], ...
                reshape((repmat([-3 -1 1 3],4,1)-1i* ...
                repmat([-3 -1 1 3],4,1)')*sqrt(1/10),1,16)];
        else % M == 64
            pRxConstellation.ReferenceConstellation = ...
                [sqrt(1/2)*(1+1i)*[-1 1], ...
                reshape((repmat ([-7 -5 -3 -1 1 3 5 7],8,1)-1i* ...
                repmat([-7 -5 -3 -1 1 3 5 7],8,1)')*sqrt(1/42),1,64)];
        end
        step(pRxConstellation,RCRxSignal);
    end
    figure(1)
    subplot 211 % 发送端RCR滤波器后的发送信号
    plot([real(transmittedSignal),imag(transmittedSignal)]);
    title('Tx signal after RCR Filter')
    subplot 212 % 接收端经信道后的接收信号
    plot([real(corruptSignal),imag(corruptSignal)]);
    title('Rx signal after Channel')
end
%% Plot
figure(2)
subplot 311 % SNR & MER vs. FrameCount
plot(SNRs);
hold on
plot(MERs);
axis([0,FrameCount,0,50])
legend('SNR','MER')
title('SNR & MER')

subplot 312 % 调制阶数（2/4/6） vs. FrameCount
plot(log2(Mods));
axis([0,FrameCount,0,8])
title('Bits Per Symbol Log_2(M)')

subplot 313 % BER vs. FrameCount
plot(BERs);
axis([0,FrameCount,0,0.55])
title('BER')