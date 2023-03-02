t = 0:.001:1.023;
X = sin(2*pi*60*t');
Y = fft(X,1024);
Pyy = Y.*conj(Y)/1024;
f = 1000/1024*(0:511);
figure()
plot(f,Pyy(1:512))
title('Power spectral density 1')
xlabel('Frequency (Hz) 1')
figure()
plot(t,X)
%%
t = 0:.001:1.023;
X = sin(2*pi*60*t')+sin(2*pi*29*t');
Y = fft(X,1024);
Pyy = Y.*conj(Y)/1024;
f = 1000/1024*(0:511);
figure()
plot(f,Pyy(1:512))
title('Power spectral density2')
xlabel('Frequency (Hz)2')
figure()
plot(t,X)
%%
t = 0:.001:1.023;
X = sin(2*pi*250*t')+sin(2*pi*300*t');
Y = fft(X,1024);
Pyy = Y.*conj(Y)/1024;
f = 1000/1024*(0:511);
figure()
plot(f,Pyy(1:512))
title('Power spectral density3')
xlabel('Frequency (Hz)3')
figure()
plot(t,X)
%%
t = 0:.001:1.023;
X = sin(2*pi*250*t')+sin(2*pi*300*t') + 1.5
Y = fft(X,1024);
Pyy = Y.*conj(Y)/1024;
f = 1000/1024*(0:511);
figure()
plot(f,Pyy(1:512))
title('Power spectral density4')
xlabel('Frequency (Hz)4')
figure()
plot(t,X)
