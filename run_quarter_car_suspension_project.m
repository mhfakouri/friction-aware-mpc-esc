%% run_quarter_car_suspension_project.m
% Quarter-Car Suspension Dynamics & Road-Disturbance Response
%
% Clean reproducible modernization of the archived suspension.m project.
%
% SOURCE-DERIVED MODEL
% --------------------
% The archived MATLAB script defines a 4-state suspension model with:
%
%   sprung mass      m  = 100 kg
%   unsprung mass    mu = 10 kg
%   suspension spring k1 = 200 N/m
%   tire spring       k2 = 200 N/m
%   suspension damper c  = 500 N s/m
%
% State order:
%   x = [z_s; z_s_dot; z_u; z_u_dot]
%
% where z_s is sprung-mass displacement and z_u is unsprung-mass displacement.
%
% The archived B matrix contains:
%   input 1 = force acting between sprung and unsprung masses
%   input 2 = road displacement entering through the tire spring
%
% The original simulation sets the control-force input to zero, so this public
% project is presented as a PASSIVE suspension disturbance-response study.
%
% SOURCE-DERIVED ROAD PROFILE
% ---------------------------
% The archived 2000-sample / 100-s road input is equivalent, to sampling
% precision, to two smooth half-sine bumps:
%
%   0-20 s   : 0.10 m peak
%   40-60 s  : 0.06 m peak
%
% CLEANUP / MODERNIZATION
% -----------------------
% The original script used Control System Toolbox (ss/lsim) and plotted only
% body displacement against the road disturbance.
%
% This runner:
%   - integrates the same state-space model directly with ode45;
%   - requires no Control System Toolbox;
%   - verifies eigenvalues/stability;
%   - reports sprung/unsprung motion, suspension travel, tire deflection,
%     body acceleration, and RMS metrics;
%   - computes the road-to-body frequency response directly from (jwI-A)^-1B;
%   - adds a clearly labelled NEW damping-sensitivity analysis.
%
% The damping sweep is a modernization study; only c = 500 N s/m is the
% source-derived nominal value.
%
% REQUIREMENTS
% ------------
% MATLAB R2016b or newer.
% No additional toolbox required.
%
% Author: Mohammad Hossein Fakouri
% -------------------------------------------------------------------------

clear;
clc;
close all;

fprintf('\n============================================================\n');
fprintf(' QUARTER-CAR SUSPENSION DYNAMICS & ROAD-DISTURBANCE RESPONSE\n');
fprintf('============================================================\n\n');

%% 1. Output folders
scriptPath = mfilename('fullpath');
if isempty(scriptPath)
    rootDir = pwd;
else
    rootDir = fileparts(scriptPath);
end

resultsDir = fullfile(rootDir,'results');
figuresDir = fullfile(resultsDir,'figures');

if ~exist(resultsDir,'dir')
    mkdir(resultsDir);
end
if ~exist(figuresDir,'dir')
    mkdir(figuresDir);
end

%% 2. Source-derived parameters
p.m  = 100;     % sprung mass [kg]
p.mu = 10;      % unsprung mass [kg]
p.k1 = 200;     % suspension spring [N/m]
p.k2 = 200;     % tire spring [N/m]
p.c  = 500;     % suspension damping [N s/m]

fprintf('Source-derived suspension parameters:\n');
fprintf('  sprung mass m        = %.1f kg\n',p.m);
fprintf('  unsprung mass mu     = %.1f kg\n',p.mu);
fprintf('  suspension spring k1 = %.1f N/m\n',p.k1);
fprintf('  tire spring k2       = %.1f N/m\n',p.k2);
fprintf('  damping c            = %.1f N s/m\n\n',p.c);

%% 3. Source-derived state-space matrices
[A,B] = suspensionMatrices(p);

Cbody = [1 0 0 0];

eigA = eig(A);
stableNominal = all(real(eigA) < 0);

fprintf('Nominal state-space model:\n');
fprintf('  state order = [z_s, z_s_dot, z_u, z_u_dot]\n');
fprintf('  input 1     = inter-mass force [N]\n');
fprintf('  input 2     = road displacement [m]\n');
fprintf('  open-loop/passive poles:\n');

for k = 1:numel(eigA)
    fprintf('    %+.6f %+.6fi\n',real(eigA(k)),imag(eigA(k)));
end

fprintf('  asymptotic stability = %s\n\n',passFail(stableNominal));

%% 4. Source-derived road disturbance
Tend = 100;
t = linspace(0,Tend,5001).';
road = roadProfile(t);

fprintf('Road profile:\n');
fprintf('  bump 1: 0-20 s, peak 0.10 m\n');
fprintf('  bump 2: 40-60 s, peak 0.06 m\n');
fprintf('  control-force input: 0 N (passive case)\n\n');

%% 5. Nominal passive simulation
x0 = zeros(4,1);
odeOpts = odeset('RelTol',1e-9,'AbsTol',1e-11);

[tSim,x] = ode45(@(tt,xx) passiveDynamics(tt,xx,p),t,x0,odeOpts);

roadSim = roadProfile(tSim);

zs  = x(:,1);
vzs = x(:,2);
zu  = x(:,3);
vzu = x(:,4);

suspTravel = zs - zu;
tireDeflection = zu - roadSim;

bodyAcc = zeros(size(tSim));
wheelAcc = zeros(size(tSim));

for k = 1:numel(tSim)
    dxk = passiveDynamics(tSim(k),x(k,:).',p);
    bodyAcc(k) = dxk(2);
    wheelAcc(k) = dxk(4);
end

%% 6. Quantitative metrics
maxRoad = max(abs(roadSim));
maxBodyDisp = max(abs(zs));
maxWheelDisp = max(abs(zu));
maxSuspTravel = max(abs(suspTravel));
maxTireDeflection = max(abs(tireDeflection));
maxBodyAcc = max(abs(bodyAcc));
rmsBodyAcc = sqrt(mean(bodyAcc.^2));
rmsBodyDisp = sqrt(mean(zs.^2));
rmsSuspTravel = sqrt(mean(suspTravel.^2));

% Amplification relative to maximum road displacement.
bodyDispAmplification = maxBodyDisp/maxRoad;

% Post-disturbance residual at the end of 100 s.
finalStateNorm = norm(x(end,:),2);

fprintf('Nominal passive-response metrics:\n');
fprintf('  maximum road displacement      = %.6f m\n',maxRoad);
fprintf('  maximum sprung displacement    = %.6f m\n',maxBodyDisp);
fprintf('  maximum unsprung displacement  = %.6f m\n',maxWheelDisp);
fprintf('  maximum suspension travel      = %.6f m\n',maxSuspTravel);
fprintf('  maximum tire deflection        = %.6f m\n',maxTireDeflection);
fprintf('  maximum |body acceleration|    = %.6f m/s^2\n',maxBodyAcc);
fprintf('  RMS body acceleration          = %.6f m/s^2\n',rmsBodyAcc);
fprintf('  RMS body displacement          = %.6f m\n',rmsBodyDisp);
fprintf('  RMS suspension travel          = %.6f m\n',rmsSuspTravel);
fprintf('  peak body/road displacement    = %.6f\n',bodyDispAmplification);
fprintf('  final state norm at 100 s      = %.3e\n\n',finalStateNorm);

%% 7. Direct frequency response: road displacement -> body displacement
freqHz = logspace(-2,1.5,600).';
omega = 2*pi*freqHz;

HroadToBody = zeros(size(freqHz));

for k = 1:numel(freqHz)
    jwIminusA = 1i*omega(k)*eye(4)-A;
    H = Cbody*(jwIminusA\B(:,2));
    HroadToBody(k) = abs(H);
end

[peakFreqGain,idxPeakFreq] = max(HroadToBody);
peakFreqHz = freqHz(idxPeakFreq);

fprintf('Road-to-body displacement frequency response:\n');
fprintf('  peak magnitude = %.6f\n',peakFreqGain);
fprintf('  peak frequency = %.6f Hz\n\n',peakFreqHz);

%% 8. NEW damping-sensitivity sweep
dampingValues = linspace(100,1200,45).';

sensMaxBodyAcc = zeros(size(dampingValues));
sensRMSBodyAcc = zeros(size(dampingValues));
sensMaxSuspTravel = zeros(size(dampingValues));
sensMaxBodyDisp = zeros(size(dampingValues));

% Use a moderately dense fixed time vector for consistent comparison.
tSens = linspace(0,Tend,2501).';

for j = 1:numel(dampingValues)
    pj = p;
    pj.c = dampingValues(j);

    [~,xj] = ode45(@(tt,xx) passiveDynamics(tt,xx,pj),tSens,x0, ...
        odeset('RelTol',1e-7,'AbsTol',1e-9));

    roadj = roadProfile(tSens);
    zsj = xj(:,1);
    zuj = xj(:,3);

    accj = zeros(size(tSens));

    for k = 1:numel(tSens)
        dxk = passiveDynamics(tSens(k),xj(k,:).',pj);
        accj(k) = dxk(2);
    end

    sensMaxBodyAcc(j) = max(abs(accj));
    sensRMSBodyAcc(j) = sqrt(mean(accj.^2));
    sensMaxSuspTravel(j) = max(abs(zsj-zuj));
    sensMaxBodyDisp(j) = max(abs(zsj));
end

[bestRMSAcc,idxBestRMS] = min(sensRMSBodyAcc);
bestRMSDamping = dampingValues(idxBestRMS);

fprintf('NEW damping-sensitivity result:\n');
fprintf('  sweep range            = %.0f to %.0f N s/m\n', ...
    dampingValues(1),dampingValues(end));
fprintf('  nominal archived value = %.0f N s/m\n',p.c);
fprintf('  minimum RMS body accel = %.6f m/s^2 at c = %.1f N s/m\n\n', ...
    bestRMSAcc,bestRMSDamping);

%% 9. Figure 1 — road disturbance
fig1 = figure('Color','w','Name','Road Disturbance');

plot(tSim,roadSim,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Road displacement [m]');
title('Source-Derived Two-Bump Road Disturbance');

saveFigure(fig1,figuresDir,'01_source_road_disturbance');

%% 10. Figure 2 — sprung / unsprung displacement
fig2 = figure('Color','w','Name','Suspension Displacements');

plot(tSim,roadSim,'--','LineWidth',1.2);
hold on;
plot(tSim,zs,'LineWidth',1.4);
plot(tSim,zu,'LineWidth',1.4);

grid on;
xlabel('Time [s]');
ylabel('Displacement [m]');
title('Passive Quarter-Car Displacement Response');
legend('Road','Sprung mass','Unsprung mass','Location','best');

saveFigure(fig2,figuresDir,'02_body_and_wheel_displacements');

%% 11. Figure 3 — suspension travel and tire deflection
fig3 = figure('Color','w','Name','Relative Deflections');

subplot(2,1,1);
plot(tSim,suspTravel,'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('z_s-z_u [m]');
title('Suspension Travel');

subplot(2,1,2);
plot(tSim,tireDeflection,'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('z_u-z_r [m]');
title('Tire Deflection Relative to Road');

saveFigure(fig3,figuresDir,'03_suspension_travel_and_tire_deflection');

%% 12. Figure 4 — body and wheel acceleration
fig4 = figure('Color','w','Name','Vertical Accelerations');

subplot(2,1,1);
plot(tSim,bodyAcc,'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('z_s double dot [m/s^2]');
title('Sprung-Mass Vertical Acceleration');

subplot(2,1,2);
plot(tSim,wheelAcc,'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('z_u double dot [m/s^2]');
title('Unsprung-Mass Vertical Acceleration');

saveFigure(fig4,figuresDir,'04_body_and_wheel_accelerations');

%% 13. Figure 5 — body phase plane
fig5 = figure('Color','w','Name','Sprung Mass Phase Plane');

plot(zs,vzs,'LineWidth',1.3);
grid on;
xlabel('Sprung displacement z_s [m]');
ylabel('Sprung velocity dz_s/dt [m/s]');
title('Sprung-Mass Phase Plane');

saveFigure(fig5,figuresDir,'05_sprung_mass_phase_plane');

%% 14. Figure 6 — road-to-body frequency response
fig6 = figure('Color','w','Name','Road to Body Frequency Response');

semilogx(freqHz,20*log10(max(HroadToBody,1e-12)),'LineWidth',1.4);
grid on;
xlabel('Frequency [Hz]');
ylabel('|Z_s/Z_r| [dB]');
title('Road-Displacement to Sprung-Displacement Frequency Response');

saveFigure(fig6,figuresDir,'06_road_to_body_frequency_response');

%% 15. Figure 7 — damping sensitivity
fig7 = figure('Color','w','Name','Damping Sensitivity');

subplot(2,1,1);
plot(dampingValues,sensRMSBodyAcc,'LineWidth',1.4);
hold on;
xline(p.c,'--');
plot(bestRMSDamping,bestRMSAcc,'o','MarkerFaceColor','k');
grid on;
xlabel('Suspension damping c [N s/m]');
ylabel('RMS body acceleration [m/s^2]');
title('NEW Analysis: Ride-Acceleration Sensitivity to Damping');
legend('RMS body acceleration','Archived c = 500','Minimum in sweep', ...
    'Location','best');

subplot(2,1,2);
plot(dampingValues,sensMaxSuspTravel,'LineWidth',1.4);
hold on;
xline(p.c,'--');
grid on;
xlabel('Suspension damping c [N s/m]');
ylabel('Peak |z_s-z_u| [m]');
title('NEW Analysis: Suspension-Travel Sensitivity to Damping');

saveFigure(fig7,figuresDir,'07_damping_sensitivity');

%% 16. Save matrices
writematrix(A,fullfile(resultsDir,'state_matrix_A.csv'));
writematrix(B,fullfile(resultsDir,'input_matrix_B.csv'));

poleTable = table(real(eigA),imag(eigA), ...
    'VariableNames',{'RealPart','ImagPart'});
writetable(poleTable,fullfile(resultsDir,'passive_poles.csv'));

freqTable = table(freqHz,HroadToBody,20*log10(max(HroadToBody,1e-12)), ...
    'VariableNames',{'Frequency_Hz','Magnitude','Magnitude_dB'});
writetable(freqTable,fullfile(resultsDir,'road_to_body_frequency_response.csv'));

sensTable = table( ...
    dampingValues,sensMaxBodyDisp,sensMaxBodyAcc,sensRMSBodyAcc, ...
    sensMaxSuspTravel, ...
    'VariableNames',{ ...
    'Damping_Ns_per_m', ...
    'MaxBodyDisplacement_m', ...
    'MaxBodyAcceleration_mps2', ...
    'RMSBodyAcceleration_mps2', ...
    'MaxSuspensionTravel_m'});
writetable(sensTable,fullfile(resultsDir,'damping_sensitivity.csv'));

%% 17. Save metrics
metrics = table( ...
    p.m,p.mu,p.k1,p.k2,p.c,double(stableNominal), ...
    maxRoad,maxBodyDisp,maxWheelDisp,maxSuspTravel,maxTireDeflection, ...
    maxBodyAcc,rmsBodyAcc,rmsBodyDisp,rmsSuspTravel, ...
    bodyDispAmplification,finalStateNorm,peakFreqGain,peakFreqHz, ...
    bestRMSDamping,bestRMSAcc, ...
    'VariableNames',{ ...
    'SprungMass_kg', ...
    'UnsprungMass_kg', ...
    'SuspensionStiffness_Npm', ...
    'TireStiffness_Npm', ...
    'NominalDamping_Nspm', ...
    'NominalAsymptoticallyStable', ...
    'MaxRoadDisplacement_m', ...
    'MaxSprungDisplacement_m', ...
    'MaxUnsprungDisplacement_m', ...
    'MaxSuspensionTravel_m', ...
    'MaxTireDeflection_m', ...
    'MaxAbsBodyAcceleration_mps2', ...
    'RMSBodyAcceleration_mps2', ...
    'RMSBodyDisplacement_m', ...
    'RMSSuspensionTravel_m', ...
    'PeakBodyToRoadDisplacementRatio', ...
    'FinalStateNorm', ...
    'PeakRoadToBodyFrequencyGain', ...
    'PeakRoadToBodyFrequency_Hz', ...
    'DampingSweep_MinRMSAcceleration_Damping_Nspm', ...
    'DampingSweep_MinRMSAcceleration_mps2'});

writetable(metrics,fullfile(resultsDir,'suspension_metrics.csv'));

%% 18. Human-readable summary
summaryFile = fullfile(resultsDir,'suspension_summary.txt');
fid = fopen(summaryFile,'w');

fprintf(fid,'Quarter-Car Suspension Dynamics & Road-Disturbance Response\n');
fprintf(fid,'Generated: %s\n\n',datestr(now,31));

fprintf(fid,'SOURCE-DERIVED PARAMETERS\n');
fprintf(fid,'m = %.10f kg\n',p.m);
fprintf(fid,'mu = %.10f kg\n',p.mu);
fprintf(fid,'k1 = %.10f N/m\n',p.k1);
fprintf(fid,'k2 = %.10f N/m\n',p.k2);
fprintf(fid,'c = %.10f N s/m\n\n',p.c);

fprintf(fid,'SOURCE-DERIVED SIMULATION\n');
fprintf(fid,'Control force = 0 N\n');
fprintf(fid,'Road bump 1 = 0.10 m half-sine, 0-20 s\n');
fprintf(fid,'Road bump 2 = 0.06 m half-sine, 40-60 s\n');
fprintf(fid,'Simulation duration = 100 s\n\n');

fprintf(fid,'PASSIVE MODEL CHECK\n');
fprintf(fid,'Asymptotically stable = %d\n',stableNominal);
for k = 1:numel(eigA)
    fprintf(fid,'pole_%d = %+.12f %+.12fi\n', ...
        k,real(eigA(k)),imag(eigA(k)));
end

fprintf(fid,'\nNOMINAL RESPONSE METRICS\n');
fprintf(fid,'Max road displacement = %.12e m\n',maxRoad);
fprintf(fid,'Max sprung displacement = %.12e m\n',maxBodyDisp);
fprintf(fid,'Max unsprung displacement = %.12e m\n',maxWheelDisp);
fprintf(fid,'Max suspension travel = %.12e m\n',maxSuspTravel);
fprintf(fid,'Max tire deflection = %.12e m\n',maxTireDeflection);
fprintf(fid,'Max abs body acceleration = %.12e m/s^2\n',maxBodyAcc);
fprintf(fid,'RMS body acceleration = %.12e m/s^2\n',rmsBodyAcc);
fprintf(fid,'RMS body displacement = %.12e m\n',rmsBodyDisp);
fprintf(fid,'RMS suspension travel = %.12e m\n',rmsSuspTravel);
fprintf(fid,'Peak body/road displacement ratio = %.12e\n', ...
    bodyDispAmplification);
fprintf(fid,'Final state norm = %.12e\n\n',finalStateNorm);

fprintf(fid,'FREQUENCY RESPONSE\n');
fprintf(fid,'Peak road-to-body gain = %.12e\n',peakFreqGain);
fprintf(fid,'Frequency at peak gain = %.12e Hz\n\n',peakFreqHz);

fprintf(fid,'NEW DAMPING-SENSITIVITY ANALYSIS\n');
fprintf(fid,'Sweep range = %.6f to %.6f N s/m\n', ...
    dampingValues(1),dampingValues(end));
fprintf(fid,'Archived nominal damping = %.6f N s/m\n',p.c);
fprintf(fid,'Minimum RMS body acceleration in sweep = %.12e m/s^2\n', ...
    bestRMSAcc);
fprintf(fid,'Damping at minimum RMS body acceleration = %.12e N s/m\n', ...
    bestRMSDamping);

fprintf(fid,'\nINTERPRETATION NOTE\n');
fprintf(fid,['The archived script is a passive suspension simulation because ', ...
             'the first/control-force input is set identically to zero. The public ', ...
             'project does not claim active suspension control.\n']);
fprintf(fid,['The frequency-response and damping-sensitivity sections are new ', ...
             'analysis added during modernization and are clearly separated from ', ...
             'the source-derived nominal simulation.\n']);

fclose(fid);

%% 19. Save MATLAB results
save(fullfile(resultsDir,'suspension_results.mat'), ...
    'p','A','B','eigA','tSim','x','roadSim','zs','zu','vzs','vzu', ...
    'suspTravel','tireDeflection','bodyAcc','wheelAcc', ...
    'freqHz','HroadToBody','dampingValues','sensMaxBodyDisp', ...
    'sensMaxBodyAcc','sensRMSBodyAcc','sensMaxSuspTravel','metrics');

fprintf('Files saved successfully to:\n  %s\n\n',resultsDir);
fprintf('Please send back:\n');
fprintf('  1) the entire results folder as a ZIP\n');
fprintf('  2) the complete MATLAB Command Window output\n');
fprintf('  3) any warning/error message, if MATLAB shows one\n\n');
fprintf('Done.\n');

%% ========================================================================
% Local functions
% ========================================================================

function [A,B] = suspensionMatrices(p)

A = [ ...
    0,              1,                  0,               0;
   -p.k1/p.m,      -p.c/p.m,            p.k1/p.m,        p.c/p.m;
    0,              0,                  0,               1;
    p.k1/p.mu,      p.c/p.mu,           -(p.k1+p.k2)/p.mu, -p.c/p.mu];

B = [ ...
    0,          0;
    1/p.m,      0;
    0,          0;
   -1/p.mu,     p.k2/p.mu];
end

function dx = passiveDynamics(t,x,p)
[A,B] = suspensionMatrices(p);

forceInput = 0;
roadInput = roadProfile(t);

u = [forceInput;roadInput];
dx = A*x+B*u;
end

function z = roadProfile(t)
% Continuous-time equivalent of the archived indexed disturbance.

z = zeros(size(t));

idx1 = (t >= 0) & (t < 20);
z(idx1) = 0.10*sin(pi*t(idx1)/20);

idx2 = (t > 40) & (t < 60);
z(idx2) = 0.06*sin(pi*(t(idx2)-40)/20);
end

function txt = passFail(tf)
if tf
    txt = 'PASS';
else
    txt = 'FAIL';
end
end

function saveFigure(figHandle,figuresDir,baseName)
pngFile = fullfile(figuresDir,[baseName '.png']);
figFile = fullfile(figuresDir,[baseName '.fig']);

set(figHandle,'PaperPositionMode','auto');
print(figHandle,pngFile,'-dpng','-r200');
savefig(figHandle,figFile);
end
