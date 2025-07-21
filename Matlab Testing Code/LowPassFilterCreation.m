Fs = 200; % Hz
Fc = 15;  % Hz
order = 2;
[B, A] = butter(order,Fc/(Fs/2))