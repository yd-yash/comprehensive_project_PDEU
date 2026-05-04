function dist = disturbance_model(t)

% 1. No disturbance
% 2. Sinusoidal disturbance
% 3. Random (noise) disturbance

% 1
    dist = zeros(6, 1);


% 2
    % Translational disturbance amplitudes (N) and frequencies (Hz)
    % ATx = 0.5;   fTx = 0.5;
    % ATy = 0.5;   fTy = 0.5;
    % ATz = 0.3;   fTz = 0.3;
    
    % Rotational disturbance amplitudes (N·m) and frequencies (Hz)
    % ARx = 0.01;  fRx = 0.7;
    % ARy = 0.01;  fRy = 0.7;
    % ARz = 0.01;  fRz = 0.5;
    
    % dTx = ATx * sin(2*pi * fTx * t);
    % dTy = ATy * sin(2*pi * fTy * t);
    % dTz = ATz * sin(2*pi * fTz * t);
    % dRx = ARx * sin(2*pi * fRx * t);
    % dRy = ARy * sin(2*pi * fRy * t);
    % dRz = ARz * sin(2*pi * fRz * t);
     
    % dist = [dTx; dTy; dTz; dRx; dRy; dRz];
     

% 3
    % Translational noise standard deviations (N)
    % sigTx = 0.2;
    % sigTy = 0.2;
    % sigTz = 0.1;
    
    % Rotational noise standard deviations (N·m)
    % sigRx = 0.005;
    % sigRy = 0.005;
    % sigRz = 0.005;
    
    % dist = [sigTx * randn; sigTy * randn; sigTz * randn;
    %         sigRx * randn; sigRy * randn; sigRz * randn];

end
