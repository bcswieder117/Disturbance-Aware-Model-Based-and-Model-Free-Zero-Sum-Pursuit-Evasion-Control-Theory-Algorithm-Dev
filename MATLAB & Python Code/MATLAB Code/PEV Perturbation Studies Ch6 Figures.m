%% PEV_Ch6_Perturbation_Studies_FIXED_Command_Window_Script.m
% Reproducible Chapter 6 perturbation-study script.
%
% Run from the MATLAB Command Window by typing:
%   PEV_Ch6_Perturbation_Studies_FIXED_Command_Window_Script
%
% This is a plain MATLAB script. It contains no local helper functions and
% no external file dependencies. It uses the same frozen nominal benchmark
% structure used for the Chapter 4--5 figures:
%   - fixed-operating-point kinematic-bicycle LQ game,
%   - finite-horizon zero-sum saddle policy,
%   - fitted-Q recovery of the same nominal saddle gains,
%   - nonlinear actuator-limited bicycle rollout.
%
% The purpose of this corrected Chapter 6 script is to avoid the misleading
% plots from the earlier draft. In particular, it does NOT plot an unstable
% or artificial "perturbed-data fitted-Q" curve. The nominal fitted-Q policy
% recovers the nominal Riccati policy, so the robustness sweeps are reported
% as exact-vs-learned consistency checks under the same perturbations.

clear;
clc;
close all;

rng(11,'twister');
format long g;

set(groot,'defaultFigureColor','w');
set(groot,'defaultFigureInvertHardcopy','off');
set(groot,'defaultAxesColor','w');
set(groot,'defaultAxesXColor','k');
set(groot,'defaultAxesYColor','k');
set(groot,'defaultTextColor','k');
set(groot,'defaultLegendTextColor','k');
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',12);
set(groot,'defaultTextFontSize',12);

scriptPath = mfilename('fullpath');
if isempty(scriptPath)
    scriptDir = pwd;
else
    scriptDir = fileparts(scriptPath);
end
outputFolder = fullfile(scriptDir,'PEV_Ch6_Perturbation_Output_FIXED');
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

%% ================================================================
% 1. NOMINAL BENCHMARK PARAMETERS
%% ================================================================

Ts = 0.02;
N  = 400;

Lp = 0.256;
Le = 0.256;

nx  = 8;
nup = 2;
nue = 2;
nz  = nx + nup + nue;

x0 = [ ...
    -2.00; ...
     0.00; ...
     0.00; ...
     0.75; ...
     0.00; ...
     0.60; ...
     0.00; ...
     0.60];

vbarP = 0.75;
vbarE = 0.60;

xbar = [0;0;0;vbarP;0;0;0;vbarE];

captureRadius = 0.35;

amaxP = 1.25;
amaxE = 0.75;

deltaMaxP = deg2rad(27);
deltaMaxE = deg2rad(22);

vminP = 0.0;
vmaxP = 1.45;
vminE = 0.0;
vmaxE = 1.05;

evaderPolicyScale = 0.55;

%% ================================================================
% 2. FIXED-OPERATING-POINT DISCRETE BICYCLE GAME
%% ================================================================

AcP = [ ...
    0 0 0      1; ...
    0 0 vbarP  0; ...
    0 0 0      0; ...
    0 0 0      0];

BcP = [ ...
    0 0; ...
    0 0; ...
    0 vbarP/Lp; ...
    1 0];

AcE = [ ...
    0 0 0      1; ...
    0 0 vbarE  0; ...
    0 0 0      0; ...
    0 0 0      0];

BcE = [ ...
    0 0; ...
    0 0; ...
    0 vbarE/Le; ...
    1 0];

MP = expm([AcP BcP; zeros(2,6)]*Ts);
AP = MP(1:4,1:4);
BPsmall = MP(1:4,5:6);

ME = expm([AcE BcE; zeros(2,6)]*Ts);
AE = ME(1:4,1:4);
BEsmall = ME(1:4,5:6);

A  = blkdiag(AP,AE);
Bp = [BPsmall; zeros(4,2)];
Be = [zeros(4,2); BEsmall];

Crel = [eye(4), -eye(4)];
QrBase = diag([35,35,5,1]);
Qbase = Crel'*QrBase*Crel + 1e-6*eye(nx);
RpBase = diag([0.35,0.18]);
ReBase = diag([8.0,8.0]);

%% ================================================================
% 3. SEARCH FOR A VALID FINITE-HORIZON SADDLE GAME
%% ================================================================

qScaleCandidates   = [1.25, 1.00, 0.80, 0.60];
terminalCandidates = [12, 10, 8, 6, 4];
rpScaleCandidates  = [1.00, 1.25, 1.50, 2.00, 2.50];
reScaleCandidates  = [1.00, 1.25, 1.50, 1.75, 2.00, 2.50, 3.00, 3.50, 4.00, 4.50, 5.00, 6.00, 8.00];

validGameFound = false;
selectedQScale = NaN;
selectedTerminal = NaN;
selectedRpScale = NaN;
selectedReScale = NaN;
bestMinPcurv = NaN;
bestMaxEschur = NaN;
bestMaxCond = NaN;

for qScale = qScaleCandidates

    Qtrial = qScale*Qbase;

    for terminalMultiplier = terminalCandidates

        QfTrial = terminalMultiplier*Qtrial;

        for rpScale = rpScaleCandidates

            RpTrial = rpScale*RpBase;

            for reScale = reScaleCandidates

                ReTrial = reScale*ReBase;

                Ptrial  = zeros(nx,nx,N+1);
                KpTrial = zeros(nup,nx,N);
                KeTrial = zeros(nue,nx,N);

                Ptrial(:,:,N+1) = QfTrial;

                candidateValid = true;
                minPcurvCandidate = inf;
                maxEschurCandidate = -inf;
                maxCondCandidate = 0;

                for k = N:-1:1

                    Pnext = Ptrial(:,:,k+1);

                    Gpp = RpTrial + Bp'*Pnext*Bp;
                    Gpe =           Bp'*Pnext*Be;
                    Gep =           Be'*Pnext*Bp;
                    Gee = -ReTrial + Be'*Pnext*Be;

                    GppSym = (Gpp+Gpp')/2;
                    minEigP = min(eig(GppSym));

                    if minEigP <= 1e-9
                        candidateValid = false;
                        break;
                    end

                    SchurE = Gee - Gep*(Gpp\Gpe);
                    SchurESym = (SchurE+SchurE')/2;
                    maxEigE = max(eig(SchurESym));

                    if maxEigE >= -1e-9
                        candidateValid = false;
                        break;
                    end

                    G = [Gpp Gpe; Gep Gee];

                    if rcond(G) < 1e-11
                        candidateValid = false;
                        break;
                    end

                    F = [Bp'*Pnext*A; Be'*Pnext*A];
                    Kjoint = G\F;

                    KpTrial(:,:,k) = Kjoint(1:nup,:);
                    KeTrial(:,:,k) = Kjoint(nup+1:end,:);

                    Pk = Qtrial + A'*Pnext*A - F'*(G\F);
                    Ptrial(:,:,k) = (Pk+Pk')/2;

                    minPcurvCandidate = min(minPcurvCandidate,minEigP);
                    maxEschurCandidate = max(maxEschurCandidate,maxEigE);
                    maxCondCandidate = max(maxCondCandidate,cond(G));

                end

                if candidateValid

                    validGameFound = true;

                    Q = Qtrial;
                    Qf = QfTrial;
                    Rp = RpTrial;
                    Re = ReTrial;

                    Pexact = Ptrial;
                    KpExact = KpTrial;
                    KeExact = KeTrial;

                    selectedQScale = qScale;
                    selectedTerminal = terminalMultiplier;
                    selectedRpScale = rpScale;
                    selectedReScale = reScale;
                    bestMinPcurv = minPcurvCandidate;
                    bestMaxEschur = maxEschurCandidate;
                    bestMaxCond = maxCondCandidate;

                    break;
                end
            end
            if validGameFound, break; end
        end
        if validGameFound, break; end
    end
    if validGameFound, break; end
end

if ~validGameFound
    error('No valid saddle game found. Increase Rp/Re or reduce Q/Qf.');
end

fprintf('\n================================================\n');
fprintf('VALID NOMINAL SADDLE GAME FOUND\n');
fprintf('================================================\n');
fprintf('Q scale                   = %.3f\n',selectedQScale);
fprintf('Terminal multiplier       = %.3f\n',selectedTerminal);
fprintf('Rp scale                  = %.3f\n',selectedRpScale);
fprintf('Re scale                  = %.3f\n',selectedReScale);
fprintf('Minimum pursuer curvature = %.6e\n',bestMinPcurv);
fprintf('Maximum evader Schur eig. = %.6e\n',bestMaxEschur);
fprintf('Maximum saddle condition  = %.6e\n\n',bestMaxCond);

%% ================================================================
% 4. FITTED-Q RECOVERY OF THE NOMINAL SADDLE POLICY
%% ================================================================

nFeatures = nz*(nz+1)/2;
samplesPerStage = max(850,7*nFeatures);
lambda = 1e-10;

Plearn  = zeros(nx,nx,N+1);
KpLearn = zeros(nup,nx,N);
KeLearn = zeros(nue,nx,N);

bellmanResidual = zeros(N,1);
featureRank = zeros(N,1);
featureCondition = zeros(N,1);

Plearn(:,:,N+1) = Qf;

for k = N:-1:1

    Pnext = Plearn(:,:,k+1);

    Phi = zeros(samplesPerStage,nFeatures);
    Y   = zeros(samplesPerStage,1);

    for sample = 1:samplesPerStage

        x = [1.20*randn; 1.20*randn; 0.30*randn; 0.30*randn; ...
             1.20*randn; 1.20*randn; 0.30*randn; 0.30*randn];

        up = [0.45*randn; 0.18*randn];
        ue = [0.40*randn; 0.16*randn];

        xnext = A*x + Bp*up + Be*ue;

        target = x'*Q*x + up'*Rp*up - ue'*Re*ue + xnext'*Pnext*xnext;
        zeta = [x;up;ue];

        phi = zeros(nFeatures,1);
        idx = 1;
        for row = 1:nz
            for col = row:nz
                if row == col
                    phi(idx) = zeta(row)*zeta(col);
                else
                    phi(idx) = 2*zeta(row)*zeta(col);
                end
                idx = idx + 1;
            end
        end

        Phi(sample,:) = phi';
        Y(sample) = target;
    end

    featureRank(k) = rank(Phi);
    featureCondition(k) = cond(Phi'*Phi + lambda*eye(nFeatures));

    if featureRank(k) < nFeatures
        error('Feature matrix rank deficient at stage %d.',k);
    end

    theta = (Phi'*Phi + lambda*eye(nFeatures))\(Phi'*Y);

    H = zeros(nz,nz);
    idx = 1;
    for row = 1:nz
        for col = row:nz
            H(row,col) = theta(idx);
            H(col,row) = theta(idx);
            idx = idx + 1;
        end
    end
    H = (H+H')/2;

    Hxx = H(1:nx,1:nx);
    pIdx = nx+1:nx+nup;
    eIdx = nx+nup+1:nz;

    Hpp = H(pIdx,pIdx);
    Hpe = H(pIdx,eIdx);
    Hep = H(eIdx,pIdx);
    Hee = H(eIdx,eIdx);
    Hpx = H(pIdx,1:nx);
    Hex = H(eIdx,1:nx);
    Hxp = H(1:nx,pIdx);
    Hxe = H(1:nx,eIdx);

    Huu = [Hpp Hpe; Hep Hee];
    Hux = [Hpx; Hex];
    Hxu = [Hxp Hxe];

    HppSym = (Hpp+Hpp')/2;
    SchurE = Hee - Hep*(Hpp\Hpe);
    SchurESym = (SchurE+SchurE')/2;

    if min(eig(HppSym)) <= 1e-8
        error('Learned pursuer curvature invalid at stage %d.',k);
    end
    if max(eig(SchurESym)) >= -1e-8
        error('Learned evader Schur condition invalid at stage %d.',k);
    end

    KjointLearn = Huu\Hux;

    KpLearn(:,:,k) = KjointLearn(1:nup,:);
    KeLearn(:,:,k) = KjointLearn(nup+1:end,:);

    PkLearn = Hxx - Hxu*(Huu\Hux);
    Plearn(:,:,k) = (PkLearn+PkLearn')/2;

    prediction = Phi*theta;
    bellmanResidual(k) = norm(prediction-Y)/max(norm(Y),eps);
end

pursuerGainError = norm(KpLearn(:)-KpExact(:))/max(norm(KpExact(:)),eps);
evaderGainError  = norm(KeLearn(:)-KeExact(:))/max(norm(KeExact(:)),eps);

fprintf('================================================\n');
fprintf('FITTED-Q NOMINAL RECOVERY CHECK\n');
fprintf('================================================\n');
fprintf('Unique Q coefficients       = %d\n',nFeatures);
fprintf('Samples per stage           = %d\n',samplesPerStage);
fprintf('Relative pursuer gain error = %.6e\n',pursuerGainError);
fprintf('Relative evader gain error  = %.6e\n',evaderGainError);
fprintf('Maximum Bellman residual    = %.6e\n',max(bellmanResidual));
fprintf('Minimum feature rank        = %d of %d\n\n',min(featureRank),nFeatures);

%% ================================================================
% 5. CHAPTER 6 PERTURBATION SWEEPS
%% ================================================================

% Perturbations are evaluated using the full saddle evader mode. The exact
% and learned nominal policies are both included as a consistency check. In
% the figures, the two curves should overlap because Chapter 5 recovered the
% same nominal policy from data.

controllerNames = {'Nominal Riccati','Nominal fitted-Q'};
numControllers = 2;

% Storage for all rows written to CSV.
summaryCell = {};
summaryRow = 0;

% A reusable loop pattern is repeated below intentionally. This file is kept
% function-free so it can be run reliably from the MATLAB Command Window on
% systems where local functions in scripts cause trouble.

%% 5.1 Wheelbase mismatch sweep
wheelbaseScales = [0.90 0.95 1.00 1.05 1.10];
wheelTime = zeros(numel(wheelbaseScales),numControllers);
wheelMinSep = zeros(numel(wheelbaseScales),numControllers);

for s = 1:numel(wheelbaseScales)
    for controller = 1:numControllers

        LpSim = Lp*wheelbaseScales(s);
        LeSim = Le*wheelbaseScales(s);
        steerScaleP = 1.0; steerScaleE = 1.0;
        longScaleP = 1.0; longScaleE = 1.0;
        actuatorScale = 1.0;
        delaySteps = 0;
        processNoiseScale = 0.0;
        measurementNoiseScale = 0.0;
        rng(1000 + 10*s + controller,'twister');

        X = zeros(nx,N+1); X(:,1) = x0;
        rawUP = zeros(nup,N); rawUE = zeros(nue,N);
        dist = zeros(N+1,1); dist(1) = norm(x0(1:2)-x0(5:6));
        captured = false; captureStep = N;
        pursuerSatA = 0; pursuerSatD = 0;

        for k = 1:N
            currentState = X(:,k);
            measuredState = currentState + measurementNoiseScale*randn(nx,1);
            errorState = measuredState - xbar;

            if controller == 1
                KpCurrent = KpExact(:,:,k); KeCurrent = KeExact(:,:,k);
            else
                KpCurrent = KpLearn(:,:,k); KeCurrent = KeLearn(:,:,k);
            end

            rawUP(:,k) = -KpCurrent*errorState;
            rawUE(:,k) = -KeCurrent*errorState;

            commandIndex = k - delaySteps;
            if commandIndex < 1
                upRaw = [0;0]; ueRaw = [0;0];
            else
                upRaw = rawUP(:,commandIndex); ueRaw = rawUE(:,commandIndex);
            end

            upCmd = [longScaleP*upRaw(1); steerScaleP*upRaw(2)];
            ueCmd = [longScaleE*ueRaw(1); steerScaleE*ueRaw(2)];

            up = upCmd;
            ue = ueCmd;
            up(1) = min(max(up(1),-amaxP*actuatorScale),amaxP*actuatorScale);
            up(2) = min(max(up(2),-deltaMaxP*actuatorScale),deltaMaxP*actuatorScale);
            ue(1) = min(max(ue(1),-amaxE*actuatorScale),amaxE*actuatorScale);
            ue(2) = min(max(ue(2),-deltaMaxE*actuatorScale),deltaMaxE*actuatorScale);

            if abs(up(1)-upCmd(1)) > 1e-10, pursuerSatA = pursuerSatA + 1; end
            if abs(up(2)-upCmd(2)) > 1e-10, pursuerSatD = pursuerSatD + 1; end

            zP = currentState(1:4); zE = currentState(5:8);

            k1P = [zP(4)*cos(zP(3)); zP(4)*sin(zP(3)); zP(4)/LpSim*tan(up(2)); up(1)];
            k2P = [(zP(4)+0.5*Ts*k1P(4))*cos(zP(3)+0.5*Ts*k1P(3)); (zP(4)+0.5*Ts*k1P(4))*sin(zP(3)+0.5*Ts*k1P(3)); (zP(4)+0.5*Ts*k1P(4))/LpSim*tan(up(2)); up(1)];
            k3P = [(zP(4)+0.5*Ts*k2P(4))*cos(zP(3)+0.5*Ts*k2P(3)); (zP(4)+0.5*Ts*k2P(4))*sin(zP(3)+0.5*Ts*k2P(3)); (zP(4)+0.5*Ts*k2P(4))/LpSim*tan(up(2)); up(1)];
            k4P = [(zP(4)+Ts*k3P(4))*cos(zP(3)+Ts*k3P(3)); (zP(4)+Ts*k3P(4))*sin(zP(3)+Ts*k3P(3)); (zP(4)+Ts*k3P(4))/LpSim*tan(up(2)); up(1)];
            zPnext = zP + Ts/6*(k1P+2*k2P+2*k3P+k4P);

            k1E = [zE(4)*cos(zE(3)); zE(4)*sin(zE(3)); zE(4)/LeSim*tan(ue(2)); ue(1)];
            k2E = [(zE(4)+0.5*Ts*k1E(4))*cos(zE(3)+0.5*Ts*k1E(3)); (zE(4)+0.5*Ts*k1E(4))*sin(zE(3)+0.5*Ts*k1E(3)); (zE(4)+0.5*Ts*k1E(4))/LeSim*tan(ue(2)); ue(1)];
            k3E = [(zE(4)+0.5*Ts*k2E(4))*cos(zE(3)+0.5*Ts*k2E(3)); (zE(4)+0.5*Ts*k2E(4))*sin(zE(3)+0.5*Ts*k2E(3)); (zE(4)+0.5*Ts*k2E(4))/LeSim*tan(ue(2)); ue(1)];
            k4E = [(zE(4)+Ts*k3E(4))*cos(zE(3)+Ts*k3E(3)); (zE(4)+Ts*k3E(4))*sin(zE(3)+Ts*k3E(3)); (zE(4)+Ts*k3E(4))/LeSim*tan(ue(2)); ue(1)];
            zEnext = zE + Ts/6*(k1E+2*k2E+2*k3E+k4E);

            zPnext(4) = min(max(zPnext(4),vminP),vmaxP);
            zEnext(4) = min(max(zEnext(4),vminE),vmaxE);
            zPnext(3) = atan2(sin(zPnext(3)),cos(zPnext(3)));
            zEnext(3) = atan2(sin(zEnext(3)),cos(zEnext(3)));

            nextState = [zPnext; zEnext] + processNoiseScale*randn(nx,1);
            X(:,k+1) = nextState;
            dist(k+1) = norm(nextState(1:2)-nextState(5:6));

            if dist(k+1) <= captureRadius
                captured = true; captureStep = k; break;
            end
        end

        idxEnd = captureStep+1;
        dUsed = dist(1:idxEnd);
        if captured
            captureTime = captureStep*Ts;
        else
            captureTime = N*Ts;
        end
        wheelTime(s,controller) = captureTime;
        wheelMinSep(s,controller) = min(dUsed);

        summaryRow = summaryRow + 1;
        summaryCell(summaryRow,:) = {'wheelbase_scale', wheelbaseScales(s), controllerNames{controller}, captured, captureTime, min(dUsed), pursuerSatA/max(captureStep,1)*100, pursuerSatD/max(captureStep,1)*100}; %#ok<SAGROW>
    end
end

%% 5.2 Steering-gain mismatch sweep
steeringScales = [0.85 0.95 1.00 1.05 1.15];
steerTime = zeros(numel(steeringScales),numControllers);
steerMinSep = zeros(numel(steeringScales),numControllers);

for s = 1:numel(steeringScales)
    for controller = 1:numControllers
        LpSim = Lp; LeSim = Le;
        steerScaleP = steeringScales(s); steerScaleE = steeringScales(s);
        longScaleP = 1.0; longScaleE = 1.0; actuatorScale = 1.0;
        delaySteps = 0; processNoiseScale = 0.0; measurementNoiseScale = 0.0;
        rng(2000 + 10*s + controller,'twister');
        X = zeros(nx,N+1); X(:,1) = x0; rawUP = zeros(nup,N); rawUE = zeros(nue,N);
        dist = zeros(N+1,1); dist(1) = norm(x0(1:2)-x0(5:6));
        captured = false; captureStep = N; pursuerSatA = 0; pursuerSatD = 0;
        for k = 1:N
            currentState = X(:,k); measuredState = currentState + measurementNoiseScale*randn(nx,1); errorState = measuredState - xbar;
            if controller == 1, KpCurrent = KpExact(:,:,k); KeCurrent = KeExact(:,:,k); else, KpCurrent = KpLearn(:,:,k); KeCurrent = KeLearn(:,:,k); end
            rawUP(:,k) = -KpCurrent*errorState; rawUE(:,k) = -KeCurrent*errorState;
            upRaw = rawUP(:,k); ueRaw = rawUE(:,k);
            upCmd = [longScaleP*upRaw(1); steerScaleP*upRaw(2)]; ueCmd = [longScaleE*ueRaw(1); steerScaleE*ueRaw(2)];
            up = upCmd; ue = ueCmd;
            up(1) = min(max(up(1),-amaxP*actuatorScale),amaxP*actuatorScale); up(2) = min(max(up(2),-deltaMaxP*actuatorScale),deltaMaxP*actuatorScale);
            ue(1) = min(max(ue(1),-amaxE*actuatorScale),amaxE*actuatorScale); ue(2) = min(max(ue(2),-deltaMaxE*actuatorScale),deltaMaxE*actuatorScale);
            if abs(up(1)-upCmd(1)) > 1e-10, pursuerSatA = pursuerSatA + 1; end; if abs(up(2)-upCmd(2)) > 1e-10, pursuerSatD = pursuerSatD + 1; end
            zP = currentState(1:4); zE = currentState(5:8);
            k1P=[zP(4)*cos(zP(3));zP(4)*sin(zP(3));zP(4)/LpSim*tan(up(2));up(1)]; k2P=[(zP(4)+.5*Ts*k1P(4))*cos(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))*sin(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))/LpSim*tan(up(2));up(1)]; k3P=[(zP(4)+.5*Ts*k2P(4))*cos(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))*sin(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))/LpSim*tan(up(2));up(1)]; k4P=[(zP(4)+Ts*k3P(4))*cos(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))*sin(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))/LpSim*tan(up(2));up(1)]; zPnext=zP+Ts/6*(k1P+2*k2P+2*k3P+k4P);
            k1E=[zE(4)*cos(zE(3));zE(4)*sin(zE(3));zE(4)/LeSim*tan(ue(2));ue(1)]; k2E=[(zE(4)+.5*Ts*k1E(4))*cos(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))*sin(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))/LeSim*tan(ue(2));ue(1)]; k3E=[(zE(4)+.5*Ts*k2E(4))*cos(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))*sin(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))/LeSim*tan(ue(2));ue(1)]; k4E=[(zE(4)+Ts*k3E(4))*cos(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))*sin(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))/LeSim*tan(ue(2));ue(1)]; zEnext=zE+Ts/6*(k1E+2*k2E+2*k3E+k4E);
            zPnext(4)=min(max(zPnext(4),vminP),vmaxP); zEnext(4)=min(max(zEnext(4),vminE),vmaxE); zPnext(3)=atan2(sin(zPnext(3)),cos(zPnext(3))); zEnext(3)=atan2(sin(zEnext(3)),cos(zEnext(3)));
            nextState=[zPnext;zEnext]+processNoiseScale*randn(nx,1); X(:,k+1)=nextState; dist(k+1)=norm(nextState(1:2)-nextState(5:6));
            if dist(k+1)<=captureRadius, captured=true; captureStep=k; break; end
        end
        idxEnd=captureStep+1; dUsed=dist(1:idxEnd); if captured, captureTime=captureStep*Ts; else, captureTime=N*Ts; end
        steerTime(s,controller)=captureTime; steerMinSep(s,controller)=min(dUsed);
        summaryRow=summaryRow+1; summaryCell(summaryRow,:)={'steering_gain',steeringScales(s),controllerNames{controller},captured,captureTime,min(dUsed),pursuerSatA/max(captureStep,1)*100,pursuerSatD/max(captureStep,1)*100}; %#ok<SAGROW>
    end
end

%% 5.3 Longitudinal-gain mismatch sweep
longitudinalScales = [0.85 0.95 1.00 1.05 1.15];
longTime = zeros(numel(longitudinalScales),numControllers);
longMinSep = zeros(numel(longitudinalScales),numControllers);

for s = 1:numel(longitudinalScales)
    for controller = 1:numControllers
        LpSim = Lp; LeSim = Le; steerScaleP = 1.0; steerScaleE = 1.0;
        longScaleP = longitudinalScales(s); longScaleE = longitudinalScales(s); actuatorScale = 1.0;
        delaySteps = 0; processNoiseScale = 0.0; measurementNoiseScale = 0.0;
        rng(3000 + 10*s + controller,'twister');
        X = zeros(nx,N+1); X(:,1)=x0; rawUP=zeros(nup,N); rawUE=zeros(nue,N); dist=zeros(N+1,1); dist(1)=norm(x0(1:2)-x0(5:6)); captured=false; captureStep=N; pursuerSatA=0; pursuerSatD=0;
        for k=1:N
            currentState=X(:,k); measuredState=currentState+measurementNoiseScale*randn(nx,1); errorState=measuredState-xbar;
            if controller==1, KpCurrent=KpExact(:,:,k); KeCurrent=KeExact(:,:,k); else, KpCurrent=KpLearn(:,:,k); KeCurrent=KeLearn(:,:,k); end
            rawUP(:,k)=-KpCurrent*errorState; rawUE(:,k)=-KeCurrent*errorState; upRaw=rawUP(:,k); ueRaw=rawUE(:,k);
            upCmd=[longScaleP*upRaw(1); steerScaleP*upRaw(2)]; ueCmd=[longScaleE*ueRaw(1); steerScaleE*ueRaw(2)];
            up=upCmd; ue=ueCmd; up(1)=min(max(up(1),-amaxP*actuatorScale),amaxP*actuatorScale); up(2)=min(max(up(2),-deltaMaxP*actuatorScale),deltaMaxP*actuatorScale); ue(1)=min(max(ue(1),-amaxE*actuatorScale),amaxE*actuatorScale); ue(2)=min(max(ue(2),-deltaMaxE*actuatorScale),deltaMaxE*actuatorScale);
            if abs(up(1)-upCmd(1))>1e-10, pursuerSatA=pursuerSatA+1; end; if abs(up(2)-upCmd(2))>1e-10, pursuerSatD=pursuerSatD+1; end
            zP=currentState(1:4); zE=currentState(5:8);
            k1P=[zP(4)*cos(zP(3));zP(4)*sin(zP(3));zP(4)/LpSim*tan(up(2));up(1)]; k2P=[(zP(4)+.5*Ts*k1P(4))*cos(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))*sin(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))/LpSim*tan(up(2));up(1)]; k3P=[(zP(4)+.5*Ts*k2P(4))*cos(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))*sin(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))/LpSim*tan(up(2));up(1)]; k4P=[(zP(4)+Ts*k3P(4))*cos(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))*sin(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))/LpSim*tan(up(2));up(1)]; zPnext=zP+Ts/6*(k1P+2*k2P+2*k3P+k4P);
            k1E=[zE(4)*cos(zE(3));zE(4)*sin(zE(3));zE(4)/LeSim*tan(ue(2));ue(1)]; k2E=[(zE(4)+.5*Ts*k1E(4))*cos(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))*sin(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))/LeSim*tan(ue(2));ue(1)]; k3E=[(zE(4)+.5*Ts*k2E(4))*cos(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))*sin(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))/LeSim*tan(ue(2));ue(1)]; k4E=[(zE(4)+Ts*k3E(4))*cos(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))*sin(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))/LeSim*tan(ue(2));ue(1)]; zEnext=zE+Ts/6*(k1E+2*k2E+2*k3E+k4E);
            zPnext(4)=min(max(zPnext(4),vminP),vmaxP); zEnext(4)=min(max(zEnext(4),vminE),vmaxE); zPnext(3)=atan2(sin(zPnext(3)),cos(zPnext(3))); zEnext(3)=atan2(sin(zEnext(3)),cos(zEnext(3))); nextState=[zPnext;zEnext]+processNoiseScale*randn(nx,1); X(:,k+1)=nextState; dist(k+1)=norm(nextState(1:2)-nextState(5:6)); if dist(k+1)<=captureRadius, captured=true; captureStep=k; break; end
        end
        idxEnd=captureStep+1; dUsed=dist(1:idxEnd); if captured, captureTime=captureStep*Ts; else, captureTime=N*Ts; end
        longTime(s,controller)=captureTime; longMinSep(s,controller)=min(dUsed); summaryRow=summaryRow+1; summaryCell(summaryRow,:)={'longitudinal_gain',longitudinalScales(s),controllerNames{controller},captured,captureTime,min(dUsed),pursuerSatA/max(captureStep,1)*100,pursuerSatD/max(captureStep,1)*100}; %#ok<SAGROW>
    end
end

%% 5.4 Delay sweep
delayValues = [0 1 2 3 4];
delayTime = zeros(numel(delayValues),numControllers);
delayMinSep = zeros(numel(delayValues),numControllers);

for s = 1:numel(delayValues)
    for controller = 1:numControllers
        LpSim = Lp; LeSim = Le; steerScaleP = 1.0; steerScaleE = 1.0; longScaleP = 1.0; longScaleE = 1.0; actuatorScale = 1.0; delaySteps = delayValues(s); processNoiseScale = 0; measurementNoiseScale = 0;
        rng(4000+10*s+controller,'twister');
        X=zeros(nx,N+1); X(:,1)=x0; rawUP=zeros(nup,N); rawUE=zeros(nue,N); dist=zeros(N+1,1); dist(1)=norm(x0(1:2)-x0(5:6)); captured=false; captureStep=N; pursuerSatA=0; pursuerSatD=0;
        for k=1:N
            currentState=X(:,k); errorState=currentState+measurementNoiseScale*randn(nx,1)-xbar; if controller==1, KpCurrent=KpExact(:,:,k); KeCurrent=KeExact(:,:,k); else, KpCurrent=KpLearn(:,:,k); KeCurrent=KeLearn(:,:,k); end
            rawUP(:,k)=-KpCurrent*errorState; rawUE(:,k)=-KeCurrent*errorState; commandIndex=k-delaySteps; if commandIndex<1, upRaw=[0;0]; ueRaw=[0;0]; else, upRaw=rawUP(:,commandIndex); ueRaw=rawUE(:,commandIndex); end
            upCmd=[longScaleP*upRaw(1);steerScaleP*upRaw(2)]; ueCmd=[longScaleE*ueRaw(1);steerScaleE*ueRaw(2)]; up=upCmd; ue=ueCmd; up(1)=min(max(up(1),-amaxP*actuatorScale),amaxP*actuatorScale); up(2)=min(max(up(2),-deltaMaxP*actuatorScale),deltaMaxP*actuatorScale); ue(1)=min(max(ue(1),-amaxE*actuatorScale),amaxE*actuatorScale); ue(2)=min(max(ue(2),-deltaMaxE*actuatorScale),deltaMaxE*actuatorScale); if abs(up(1)-upCmd(1))>1e-10, pursuerSatA=pursuerSatA+1; end; if abs(up(2)-upCmd(2))>1e-10, pursuerSatD=pursuerSatD+1; end
            zP=currentState(1:4); zE=currentState(5:8); k1P=[zP(4)*cos(zP(3));zP(4)*sin(zP(3));zP(4)/LpSim*tan(up(2));up(1)]; k2P=[(zP(4)+.5*Ts*k1P(4))*cos(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))*sin(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))/LpSim*tan(up(2));up(1)]; k3P=[(zP(4)+.5*Ts*k2P(4))*cos(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))*sin(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))/LpSim*tan(up(2));up(1)]; k4P=[(zP(4)+Ts*k3P(4))*cos(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))*sin(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))/LpSim*tan(up(2));up(1)]; zPnext=zP+Ts/6*(k1P+2*k2P+2*k3P+k4P); k1E=[zE(4)*cos(zE(3));zE(4)*sin(zE(3));zE(4)/LeSim*tan(ue(2));ue(1)]; k2E=[(zE(4)+.5*Ts*k1E(4))*cos(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))*sin(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))/LeSim*tan(ue(2));ue(1)]; k3E=[(zE(4)+.5*Ts*k2E(4))*cos(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))*sin(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))/LeSim*tan(ue(2));ue(1)]; k4E=[(zE(4)+Ts*k3E(4))*cos(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))*sin(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))/LeSim*tan(ue(2));ue(1)]; zEnext=zE+Ts/6*(k1E+2*k2E+2*k3E+k4E); zPnext(4)=min(max(zPnext(4),vminP),vmaxP); zEnext(4)=min(max(zEnext(4),vminE),vmaxE); zPnext(3)=atan2(sin(zPnext(3)),cos(zPnext(3))); zEnext(3)=atan2(sin(zEnext(3)),cos(zEnext(3))); nextState=[zPnext;zEnext]+processNoiseScale*randn(nx,1); X(:,k+1)=nextState; dist(k+1)=norm(nextState(1:2)-nextState(5:6)); if dist(k+1)<=captureRadius, captured=true; captureStep=k; break; end
        end
        idxEnd=captureStep+1; dUsed=dist(1:idxEnd); if captured, captureTime=captureStep*Ts; else, captureTime=N*Ts; end
        delayTime(s,controller)=captureTime; delayMinSep(s,controller)=min(dUsed); summaryRow=summaryRow+1; summaryCell(summaryRow,:)={'delay_steps',delayValues(s),controllerNames{controller},captured,captureTime,min(dUsed),pursuerSatA/max(captureStep,1)*100,pursuerSatD/max(captureStep,1)*100}; %#ok<SAGROW>
    end
end

%% 5.5 Actuator-limit severity sweep
actuatorScales = [0.70 0.80 0.90 1.00];
actTime = zeros(numel(actuatorScales),numControllers);
actMinSep = zeros(numel(actuatorScales),numControllers);
actSatA = zeros(numel(actuatorScales),numControllers);
actSatD = zeros(numel(actuatorScales),numControllers);

for s=1:numel(actuatorScales)
    for controller=1:numControllers
        LpSim=Lp; LeSim=Le; steerScaleP=1; steerScaleE=1; longScaleP=1; longScaleE=1; actuatorScale=actuatorScales(s); delaySteps=0; processNoiseScale=0; measurementNoiseScale=0;
        rng(5000+10*s+controller,'twister'); X=zeros(nx,N+1); X(:,1)=x0; rawUP=zeros(nup,N); rawUE=zeros(nue,N); dist=zeros(N+1,1); dist(1)=norm(x0(1:2)-x0(5:6)); captured=false; captureStep=N; pursuerSatA=0; pursuerSatD=0;
        for k=1:N
            currentState=X(:,k); errorState=currentState+measurementNoiseScale*randn(nx,1)-xbar; if controller==1, KpCurrent=KpExact(:,:,k); KeCurrent=KeExact(:,:,k); else, KpCurrent=KpLearn(:,:,k); KeCurrent=KeLearn(:,:,k); end
            rawUP(:,k)=-KpCurrent*errorState; rawUE(:,k)=-KeCurrent*errorState; upRaw=rawUP(:,k); ueRaw=rawUE(:,k); upCmd=[longScaleP*upRaw(1);steerScaleP*upRaw(2)]; ueCmd=[longScaleE*ueRaw(1);steerScaleE*ueRaw(2)]; up=upCmd; ue=ueCmd; up(1)=min(max(up(1),-amaxP*actuatorScale),amaxP*actuatorScale); up(2)=min(max(up(2),-deltaMaxP*actuatorScale),deltaMaxP*actuatorScale); ue(1)=min(max(ue(1),-amaxE*actuatorScale),amaxE*actuatorScale); ue(2)=min(max(ue(2),-deltaMaxE*actuatorScale),deltaMaxE*actuatorScale); if abs(up(1)-upCmd(1))>1e-10, pursuerSatA=pursuerSatA+1; end; if abs(up(2)-upCmd(2))>1e-10, pursuerSatD=pursuerSatD+1; end
            zP=currentState(1:4); zE=currentState(5:8); k1P=[zP(4)*cos(zP(3));zP(4)*sin(zP(3));zP(4)/LpSim*tan(up(2));up(1)]; k2P=[(zP(4)+.5*Ts*k1P(4))*cos(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))*sin(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))/LpSim*tan(up(2));up(1)]; k3P=[(zP(4)+.5*Ts*k2P(4))*cos(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))*sin(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))/LpSim*tan(up(2));up(1)]; k4P=[(zP(4)+Ts*k3P(4))*cos(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))*sin(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))/LpSim*tan(up(2));up(1)]; zPnext=zP+Ts/6*(k1P+2*k2P+2*k3P+k4P); k1E=[zE(4)*cos(zE(3));zE(4)*sin(zE(3));zE(4)/LeSim*tan(ue(2));ue(1)]; k2E=[(zE(4)+.5*Ts*k1E(4))*cos(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))*sin(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))/LeSim*tan(ue(2));ue(1)]; k3E=[(zE(4)+.5*Ts*k2E(4))*cos(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))*sin(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))/LeSim*tan(ue(2));ue(1)]; k4E=[(zE(4)+Ts*k3E(4))*cos(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))*sin(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))/LeSim*tan(ue(2));ue(1)]; zEnext=zE+Ts/6*(k1E+2*k2E+2*k3E+k4E); zPnext(4)=min(max(zPnext(4),vminP),vmaxP); zEnext(4)=min(max(zEnext(4),vminE),vmaxE); zPnext(3)=atan2(sin(zPnext(3)),cos(zPnext(3))); zEnext(3)=atan2(sin(zEnext(3)),cos(zEnext(3))); nextState=[zPnext;zEnext]+processNoiseScale*randn(nx,1); X(:,k+1)=nextState; dist(k+1)=norm(nextState(1:2)-nextState(5:6)); if dist(k+1)<=captureRadius, captured=true; captureStep=k; break; end
        end
        idxEnd=captureStep+1; dUsed=dist(1:idxEnd); if captured, captureTime=captureStep*Ts; else, captureTime=N*Ts; end
        actTime(s,controller)=captureTime; actMinSep(s,controller)=min(dUsed); actSatA(s,controller)=pursuerSatA/max(captureStep,1)*100; actSatD(s,controller)=pursuerSatD/max(captureStep,1)*100; summaryRow=summaryRow+1; summaryCell(summaryRow,:)={'actuator_limit_scale',actuatorScales(s),controllerNames{controller},captured,captureTime,min(dUsed),actSatA(s,controller),actSatD(s,controller)}; %#ok<SAGROW>
    end
end

%% 5.6 Noise Monte Carlo sweep
noiseScales = [0.000 0.005 0.015 0.030];
numTrials = 25;
noiseCaptureRate = zeros(numel(noiseScales),numControllers);
noiseCaptureRateSE = zeros(numel(noiseScales),numControllers);
noiseMeanTime = zeros(numel(noiseScales),numControllers);
noiseStdTime = zeros(numel(noiseScales),numControllers);

for s=1:numel(noiseScales)
    for controller=1:numControllers
        capturedTrials = false(numTrials,1);
        timeTrials = zeros(numTrials,1);
        for trial=1:numTrials
            LpSim=Lp; LeSim=Le; steerScaleP=1; steerScaleE=1; longScaleP=1; longScaleE=1; actuatorScale=1; delaySteps=0; processNoiseScale=noiseScales(s); measurementNoiseScale=0.5*noiseScales(s);
            rng(6000+100*s+10*controller+trial,'twister'); X=zeros(nx,N+1); X(:,1)=x0; rawUP=zeros(nup,N); rawUE=zeros(nue,N); dist=zeros(N+1,1); dist(1)=norm(x0(1:2)-x0(5:6)); captured=false; captureStep=N;
            for k=1:N
                currentState=X(:,k); errorState=currentState+measurementNoiseScale*randn(nx,1)-xbar; if controller==1, KpCurrent=KpExact(:,:,k); KeCurrent=KeExact(:,:,k); else, KpCurrent=KpLearn(:,:,k); KeCurrent=KeLearn(:,:,k); end
                rawUP(:,k)=-KpCurrent*errorState; rawUE(:,k)=-KeCurrent*errorState; upRaw=rawUP(:,k); ueRaw=rawUE(:,k); upCmd=[longScaleP*upRaw(1);steerScaleP*upRaw(2)]; ueCmd=[longScaleE*ueRaw(1);steerScaleE*ueRaw(2)]; up=upCmd; ue=ueCmd; up(1)=min(max(up(1),-amaxP*actuatorScale),amaxP*actuatorScale); up(2)=min(max(up(2),-deltaMaxP*actuatorScale),deltaMaxP*actuatorScale); ue(1)=min(max(ue(1),-amaxE*actuatorScale),amaxE*actuatorScale); ue(2)=min(max(ue(2),-deltaMaxE*actuatorScale),deltaMaxE*actuatorScale);
                zP=currentState(1:4); zE=currentState(5:8); k1P=[zP(4)*cos(zP(3));zP(4)*sin(zP(3));zP(4)/LpSim*tan(up(2));up(1)]; k2P=[(zP(4)+.5*Ts*k1P(4))*cos(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))*sin(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))/LpSim*tan(up(2));up(1)]; k3P=[(zP(4)+.5*Ts*k2P(4))*cos(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))*sin(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))/LpSim*tan(up(2));up(1)]; k4P=[(zP(4)+Ts*k3P(4))*cos(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))*sin(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))/LpSim*tan(up(2));up(1)]; zPnext=zP+Ts/6*(k1P+2*k2P+2*k3P+k4P); k1E=[zE(4)*cos(zE(3));zE(4)*sin(zE(3));zE(4)/LeSim*tan(ue(2));ue(1)]; k2E=[(zE(4)+.5*Ts*k1E(4))*cos(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))*sin(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))/LeSim*tan(ue(2));ue(1)]; k3E=[(zE(4)+.5*Ts*k2E(4))*cos(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))*sin(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))/LeSim*tan(ue(2));ue(1)]; k4E=[(zE(4)+Ts*k3E(4))*cos(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))*sin(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))/LeSim*tan(ue(2));ue(1)]; zEnext=zE+Ts/6*(k1E+2*k2E+2*k3E+k4E); zPnext(4)=min(max(zPnext(4),vminP),vmaxP); zEnext(4)=min(max(zEnext(4),vminE),vmaxE); zPnext(3)=atan2(sin(zPnext(3)),cos(zPnext(3))); zEnext(3)=atan2(sin(zEnext(3)),cos(zEnext(3))); nextState=[zPnext;zEnext]+processNoiseScale*randn(nx,1); X(:,k+1)=nextState; dist(k+1)=norm(nextState(1:2)-nextState(5:6)); if dist(k+1)<=captureRadius, captured=true; captureStep=k; break; end
            end
            capturedTrials(trial)=captured; if captured, timeTrials(trial)=captureStep*Ts; else, timeTrials(trial)=N*Ts; end
        end
        phat=mean(capturedTrials); noiseCaptureRate(s,controller)=100*phat; noiseCaptureRateSE(s,controller)=100*sqrt(phat*(1-phat)/numTrials); noiseMeanTime(s,controller)=mean(timeTrials); noiseStdTime(s,controller)=std(timeTrials);
        summaryRow=summaryRow+1; summaryCell(summaryRow,:)={'noise_scale',noiseScales(s),controllerNames{controller},phat>0,noiseMeanTime(s,controller),NaN,NaN,NaN}; %#ok<SAGROW>
    end
end

%% 5.7 Moderate combined perturbation trajectory
combinedScaleWheel = 1.05;
combinedScaleSteer = 0.95;
combinedScaleLong = 0.95;
combinedActuatorScale = 0.90;
combinedDelay = 1;
combinedNoise = 0.002;
combinedMeas = 0.001;
rng(777,'twister');
controller = 1;
LpSim=Lp*combinedScaleWheel; LeSim=Le*combinedScaleWheel; steerScaleP=combinedScaleSteer; steerScaleE=combinedScaleSteer; longScaleP=combinedScaleLong; longScaleE=combinedScaleLong; actuatorScale=combinedActuatorScale; delaySteps=combinedDelay; processNoiseScale=combinedNoise; measurementNoiseScale=combinedMeas;
Xc=zeros(nx,N+1); Xc(:,1)=x0; rawUP=zeros(nup,N); rawUE=zeros(nue,N); distC=zeros(N+1,1); distC(1)=norm(x0(1:2)-x0(5:6)); capturedC=false; captureStepC=N;
for k=1:N
    currentState=Xc(:,k); errorState=currentState+measurementNoiseScale*randn(nx,1)-xbar; KpCurrent=KpExact(:,:,k); KeCurrent=KeExact(:,:,k); rawUP(:,k)=-KpCurrent*errorState; rawUE(:,k)=-KeCurrent*errorState; commandIndex=k-delaySteps; if commandIndex<1, upRaw=[0;0]; ueRaw=[0;0]; else, upRaw=rawUP(:,commandIndex); ueRaw=rawUE(:,commandIndex); end
    upCmd=[longScaleP*upRaw(1);steerScaleP*upRaw(2)]; ueCmd=[longScaleE*ueRaw(1);steerScaleE*ueRaw(2)]; up=upCmd; ue=ueCmd; up(1)=min(max(up(1),-amaxP*actuatorScale),amaxP*actuatorScale); up(2)=min(max(up(2),-deltaMaxP*actuatorScale),deltaMaxP*actuatorScale); ue(1)=min(max(ue(1),-amaxE*actuatorScale),amaxE*actuatorScale); ue(2)=min(max(ue(2),-deltaMaxE*actuatorScale),deltaMaxE*actuatorScale);
    zP=Xc(1:4,k); zE=Xc(5:8,k); k1P=[zP(4)*cos(zP(3));zP(4)*sin(zP(3));zP(4)/LpSim*tan(up(2));up(1)]; k2P=[(zP(4)+.5*Ts*k1P(4))*cos(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))*sin(zP(3)+.5*Ts*k1P(3));(zP(4)+.5*Ts*k1P(4))/LpSim*tan(up(2));up(1)]; k3P=[(zP(4)+.5*Ts*k2P(4))*cos(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))*sin(zP(3)+.5*Ts*k2P(3));(zP(4)+.5*Ts*k2P(4))/LpSim*tan(up(2));up(1)]; k4P=[(zP(4)+Ts*k3P(4))*cos(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))*sin(zP(3)+Ts*k3P(3));(zP(4)+Ts*k3P(4))/LpSim*tan(up(2));up(1)]; zPnext=zP+Ts/6*(k1P+2*k2P+2*k3P+k4P); k1E=[zE(4)*cos(zE(3));zE(4)*sin(zE(3));zE(4)/LeSim*tan(ue(2));ue(1)]; k2E=[(zE(4)+.5*Ts*k1E(4))*cos(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))*sin(zE(3)+.5*Ts*k1E(3));(zE(4)+.5*Ts*k1E(4))/LeSim*tan(ue(2));ue(1)]; k3E=[(zE(4)+.5*Ts*k2E(4))*cos(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))*sin(zE(3)+.5*Ts*k2E(3));(zE(4)+.5*Ts*k2E(4))/LeSim*tan(ue(2));ue(1)]; k4E=[(zE(4)+Ts*k3E(4))*cos(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))*sin(zE(3)+Ts*k3E(3));(zE(4)+Ts*k3E(4))/LeSim*tan(ue(2));ue(1)]; zEnext=zE+Ts/6*(k1E+2*k2E+2*k3E+k4E); zPnext(4)=min(max(zPnext(4),vminP),vmaxP); zEnext(4)=min(max(zEnext(4),vminE),vmaxE); zPnext(3)=atan2(sin(zPnext(3)),cos(zPnext(3))); zEnext(3)=atan2(sin(zEnext(3)),cos(zEnext(3))); nextState=[zPnext;zEnext]+processNoiseScale*randn(nx,1); Xc(:,k+1)=nextState; distC(k+1)=norm(nextState(1:2)-nextState(5:6)); if distC(k+1)<=captureRadius, capturedC=true; captureStepC=k; break; end
end
Xc = Xc(:,1:captureStepC+1);
distC = distC(1:captureStepC+1);

%% ================================================================
% 6. THESIS-READY FIGURES
%% ================================================================

% The nominal Riccati and nominal fitted-Q controllers recover the same
% saddle policy to numerical precision.  To avoid redundant overlapping
% curves in Chapter 6, the deterministic robustness figures below plot the
% common nominal saddle policy once.  The Chapter 5 recovery errors are
% already reported separately.

policyLabel = 'Nominal saddle policy';
figSize = [100 100 900 560];

% ---------- Figure 6.1: Wheelbase mismatch ----------
fig = figure('Color','w','InvertHardcopy','off','Position',figSize);
plot(wheelbaseScales,wheelTime(:,1),'-o','LineWidth',2.0,'MarkerSize',7);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('Wheelbase scale','Color','k');
ylabel('Capture time [s]','Color','k');
title('Wheelbase-Mismatch Sweep','Color','k');
lgd = legend(policyLabel,'Location','northwest'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_1_wheelbase_capture_time_THESIS.png'),'-dpng','-r300');

% ---------- Figure 6.2: Steering-gain mismatch ----------
fig = figure('Color','w','InvertHardcopy','off','Position',figSize);
plot(steeringScales,steerTime(:,1),'-o','LineWidth',2.0,'MarkerSize',7);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('Steering gain scale','Color','k');
ylabel('Capture time [s]','Color','k');
title('Steering-Gain Mismatch Sweep','Color','k');
lgd = legend(policyLabel,'Location','northeast'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_2_steering_capture_time_THESIS.png'),'-dpng','-r300');

% ---------- Figure 6.3: Longitudinal-gain mismatch ----------
fig = figure('Color','w','InvertHardcopy','off','Position',figSize);
plot(longitudinalScales,longTime(:,1),'-o','LineWidth',2.0,'MarkerSize',7);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('Longitudinal gain scale','Color','k');
ylabel('Capture time [s]','Color','k');
title('Longitudinal-Gain Mismatch Sweep','Color','k');
lgd = legend(policyLabel,'Location','northwest'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_3_longitudinal_capture_time_THESIS.png'),'-dpng','-r300');

% ---------- Figure 6.4: Noise sensitivity ----------
fig = figure('Color','w','InvertHardcopy','off','Position',figSize);
errorbar(noiseScales,noiseCaptureRate(:,1),noiseCaptureRateSE(:,1),'-o','LineWidth',2.0,'MarkerSize',7,'CapSize',10);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('Noise scale','Color','k');
ylabel('Capture rate [%]','Color','k');
title('Noise Sensitivity Monte Carlo','Color','k');
ylim([0 105]);
lgd = legend(policyLabel,'Location','southwest'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_4_noise_capture_rate_THESIS.png'),'-dpng','-r300');

% ---------- Figure 6.5: Command delay ----------
fig = figure('Color','w','InvertHardcopy','off','Position',figSize);
plot(delayValues,delayTime(:,1),'-o','LineWidth',2.0,'MarkerSize',7);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('Command delay [steps]','Color','k');
ylabel('Capture time [s]','Color','k');
title('Command-Delay Sweep','Color','k');
lgd = legend(policyLabel,'Location','northwest'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_5_delay_capture_time_THESIS.png'),'-dpng','-r300');

% ---------- Figure 6.6: Actuator-limit severity ----------
fig = figure('Color','w','InvertHardcopy','off','Position',figSize);
plot(actuatorScales,actTime(:,1),'-o','LineWidth',2.0,'MarkerSize',7);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('Actuator limit scale','Color','k');
ylabel('Capture time [s]','Color','k');
title('Actuator-Limit Severity Sweep','Color','k');
lgd = legend(policyLabel,'Location','northwest'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_6_actuator_limit_capture_time_THESIS.png'),'-dpng','-r300');

% ---------- Figure 6.7: Steering saturation under actuator limits ----------
fig = figure('Color','w','InvertHardcopy','off','Position',figSize);
plot(actuatorScales,actSatD(:,1),'-s','LineWidth',2.0,'MarkerSize',7);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('Actuator limit scale','Color','k');
ylabel('Steering saturation [% of pre-capture steps]','Color','k');
title('Steering Saturation Under Actuator-Limit Sweep','Color','k');
lgd = legend(policyLabel,'Location','northeast'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_7_steering_saturation_THESIS.png'),'-dpng','-r300');

% ---------- Figure 6.8: Combined-perturbation trajectory ----------
fig = figure('Color','w','InvertHardcopy','off','Position',[100 100 930 620]);
plot(Xc(1,:),Xc(2,:),'-','LineWidth',2.0); hold on;
plot(Xc(5,:),Xc(6,:),'--','LineWidth',2.0);
plot(Xc(1,1),Xc(2,1),'o','MarkerSize',8,'LineWidth',1.6);
plot(Xc(5,1),Xc(6,1),'s','MarkerSize',8,'LineWidth',1.6);
th = linspace(0,2*pi,240);
plot(Xc(5,end)+captureRadius*cos(th),Xc(6,end)+captureRadius*sin(th),':','LineWidth',1.6);
axis equal; grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('x position [m]','Color','k');
ylabel('y position [m]','Color','k');
title('Representative Moderate Combined-Perturbation Trajectory','Color','k');
lgd = legend('Pursuer','Evader','P initial','E initial','Capture boundary','Location','southwest'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_8_combined_trajectory_THESIS.png'),'-dpng','-r300');

% ---------- Figure 6.9: Combined-perturbation separation history ----------
fig = figure('Color','w','InvertHardcopy','off','Position',figSize);
tC = (0:numel(distC)-1)*Ts;
plot(tC,distC,'-','LineWidth',2.0); hold on;
yline(captureRadius,':','Capture radius','LineWidth',1.6);
grid on; box on;
set(gca,'Color','w','XColor','k','YColor','k','FontName','Times New Roman','FontSize',13,'LineWidth',1.0);
xlabel('Time [s]','Color','k');
ylabel('Separation distance [m]','Color','k');
title('Combined-Perturbation Separation History','Color','k');
lgd = legend(policyLabel,'Capture radius','Location','northeast'); set(lgd,'Color','w','TextColor','k','EdgeColor','k');
print(fig,fullfile(outputFolder,'fig6_9_combined_separation_THESIS.png'),'-dpng','-r300');

% Save the combined case data so the trajectory and separation figures are
% reproducible without re-running the entire simulation.
tC = (0:numel(distC)-1)'*Ts;
combinedTable = table(tC, Xc(1,:)', Xc(2,:)', Xc(3,:)', Xc(4,:)', Xc(5,:)', Xc(6,:)', Xc(7,:)', Xc(8,:)', distC(:), ...
    'VariableNames', {'time_s','xP','yP','psiP','vP','xE','yE','psiE','vE','separation_m'});
writetable(combinedTable, fullfile(outputFolder,'chapter6_combined_trajectory_THESIS.csv'));

%% ================================================================
% 7. TABLES AND MANIFEST
%% ================================================================

summaryTable = cell2table(summaryCell,'VariableNames',{'perturbation','level','controller','captured','capture_time_s','min_separation_m','pursuer_accel_saturation_percent','pursuer_steering_saturation_percent'});
writetable(summaryTable, fullfile(outputFolder,'chapter6_perturbation_summary_FIXED.csv'));

noiseTable = table(noiseScales(:), noiseCaptureRate(:,1), noiseCaptureRateSE(:,1), noiseMeanTime(:,1), noiseStdTime(:,1), noiseCaptureRate(:,2), noiseCaptureRateSE(:,2), noiseMeanTime(:,2), noiseStdTime(:,2), ...
    'VariableNames', {'noise_scale','riccati_capture_rate_percent','riccati_capture_rate_se','riccati_mean_capture_time_s','riccati_std_capture_time_s','fittedq_capture_rate_percent','fittedq_capture_rate_se','fittedq_mean_capture_time_s','fittedq_std_capture_time_s'});
writetable(noiseTable, fullfile(outputFolder,'chapter6_noise_monte_carlo_FIXED.csv'));

fid = fopen(fullfile(outputFolder,'run_manifest.txt'),'w');
fprintf(fid,'Chapter 6 fixed perturbation script run manifest\n');
fprintf(fid,'Seed: 11 twister\n');
fprintf(fid,'Ts: %.6f\n',Ts);
fprintf(fid,'N: %d\n',N);
fprintf(fid,'Initial state: [%s]\n',sprintf(' %.6f',x0));
fprintf(fid,'Capture radius: %.6f\n',captureRadius);
fprintf(fid,'Wheelbase: %.6f\n',Lp);
fprintf(fid,'Pursuer limits: a=%.6f, delta=%.6f rad\n',amaxP,deltaMaxP);
fprintf(fid,'Evader limits: a=%.6f, delta=%.6f rad\n',amaxE,deltaMaxE);
fprintf(fid,'Q scale: %.6f, terminal multiplier: %.6f, Rp scale: %.6f, Re scale: %.6f\n',selectedQScale,selectedTerminal,selectedRpScale,selectedReScale);
fprintf(fid,'Relative pursuer gain recovery error: %.12e\n',pursuerGainError);
fprintf(fid,'Relative evader gain recovery error: %.12e\n',evaderGainError);
fprintf(fid,'Max Bellman residual: %.12e\n',max(bellmanResidual));
fprintf(fid,'Note: exact Riccati and nominal fitted-Q curves are expected to overlap because Chapter 5 recovers the nominal saddle policy.\n');
fclose(fid);

fprintf('================================================\n');
fprintf('CHAPTER 6 FIXED PERTURBATION STUDY COMPLETE\n');
fprintf('================================================\n');
fprintf('Figures and CSV files saved to:\n  %s\n',outputFolder);
fprintf('This corrected script omits the unstable/artificial perturbed-data fitted-Q curve.\n');
fprintf('Exact and fitted-Q nominal curves should overlap by design.\n');
