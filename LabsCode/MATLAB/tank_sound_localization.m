% Last edited by Rai Wandeler (rwandeler@hmc.edu) 2026-02-03

clf;

Ptx = 5; % source accoustic power
k = 1; % Voltage vs Power proportionality costant
V = k .* sqrt(Ptx./(4.*pi.*x.^2));

testing = true;

if testing
    % For testing with sample data only
    measdists = 0.01:0.05:0.8;
    Ptx = 100; % source accoustic power
    k = 1; % Voltage vs Power proportionality costant
    V = k .* sqrt(Ptx./(4.*pi.*measdists.^2)); % assuming no weird behaviour
    noise1 = 10 .* (0.5-rand(1, length(measdists)));
    noise2 = 10 .* (0.5-rand(1, length(measdists)));
    noise3 = 10 .* (0.5-rand(1, length(measdists)));

    interference1 = V./1 .* sin(2*pi*9000./1500 .* measdists);
    interference2 = V./1 .* sin(2*pi*11000./1500 .* measdists);
    interference3 = V./1 .* sin(2*pi*13000./1500 .* measdists);

    beacon1(:,1) = measdists;
    beacon1(:,2) = V + noise1 + interference1;
    beacon2(:,1) = measdists;
    beacon2(:,2) = V + noise2 + interference2;
    beacon3(:,1) = measdists;
    beacon3(:,2) = V + noise3 + interference3;
else
    beacon1 = [];
    beacon2 = [];
    beacon3 = [];
end



figure(1);
scatter(beacon1(:,1), beacon1(:,2)); % plot our collected data
hold on;
options = fitoptions('Method', 'NonlinearLeastSquares', ...
                     'Lower',[0.1 0.1], 'Upper',[1e10 1e10]);
model = fittype('k*sqrt(p/(4*pi*x.^2))',dependent="y",independent="x",coefficients=["k" "p"]); % assume no vertical shift
fittedmodel = fit(beacon1(:,1),beacon1(:,2),model,options);
axis manual;
plot(fittedmodel);
xlabel("Distance [m]");
ylabel("Voltage [V]");
legend("Data Points", "Decay Relationship without Multipath Interference");
hold off;
model_coeffs = coeffvalues(fittedmodel);
disp(model_coeffs);

figure(2);
scatter(beacon2(:,1), beacon2(:,2)); % plot our collected data
hold on;
options = fitoptions('Method', 'NonlinearLeastSquares', ...
                     'Lower',[0.1 0.1], 'Upper',[1e10 1e10]);
model = fittype('k*sqrt(p/(4*pi*x.^2))',dependent="y",independent="x",coefficients=["k" "p"]); % assume no vertical shift
fittedmodel = fit(beacon2(:,1),beacon2(:,2),model,options);
axis manual;
plot(fittedmodel);
xlabel("Distance [m]");
ylabel("Voltage [V]");
legend("Data Points", "Decay Relationship without Multipath Interference");
hold off;
model_coeffs = coeffvalues(fittedmodel);
disp(model_coeffs);

figure(3);
scatter(beacon3(:,1), beacon3(:,2)); % plot our collected data
hold on;
options = fitoptions('Method', 'NonlinearLeastSquares', ...
                     'Lower',[0.1 0.1], 'Upper',[1e10 1e10]);
model = fittype('k*sqrt(p/(4*pi*x.^2))',dependent="y",independent="x",coefficients=["k" "p"]); % assume no vertical shift
fittedmodel = fit(beacon3(:,1),beacon3(:,2),model,options);
axis manual;
plot(fittedmodel);
xlabel("Distance [m]");
ylabel("Voltage [V]");
legend("Data Points", "Decay Relationship without Multipath Interference");
hold off;
model_coeffs = coeffvalues(fittedmodel);
disp(model_coeffs);
