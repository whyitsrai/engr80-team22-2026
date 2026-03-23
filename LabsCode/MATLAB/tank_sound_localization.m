% Last edited by Rai Wandeler (rwandeler@hmc.edu) 2026-03-03

clf;

Ptx = 5; % source accoustic power
k = 1; % Voltage vs Power proportionality costant
V = k .* sqrt(Ptx./(4.*pi.*x.^2));

testing = false;

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
    %measspan = [1e-10 0.01 0.02 0.04 0.06 0.08 0.10 0.12 0.14 0.16 0.18];
    %beacon1y = [6.43 1.107 1.04 0.993 0.950 0.813 0.723 0.695 0.602 0.498 0.354];
    measspan = [5e-3 0.02 0.04 0.06 0.08 0.10 0.12 0.14 0.16 0.18]; % like 5mm
    beacon1y = [6.43 1.04 0.993 0.950 0.813 0.723 0.695 0.602 0.498 0.354];
    beacon1 = [measspan' beacon1y']; % 9kHz
    %beacon2 = [measspan beacon2y]; % 13kHz
    %beacon3 = [measspan beacon3y]; % 11kHz
end



figure(1);
scatter(beacon1(:,1), beacon1(:,2)); % plot our collected data
hold on;
options = fitoptions('Method', 'NonlinearLeastSquares', ...
                     'Lower',[1e-1 1e-1], 'Upper',[1e10 2]);
model = fittype('k*sqrt(1/(4*pi*x.^2)) + c',dependent="y",independent="x",coefficients=["k" "c"]); % assume no vertical shift
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
scatter(beacon1(2:end,1), beacon1(2:end,2)); % plot our collected data
hold on;
axis manual;
xlabel("Distance [m]");
ylabel("Voltage [V]");
legend("Data Points", "Decay Relationship without Multipath Interference");
hold off;
model_coeffs = coeffvalues(fittedmodel);
disp(model_coeffs);

%figure(2);
%scatter(beacon2(:,1), beacon2(:,2)); % plot our collected data
%hold on;
%options = fitoptions('Method', 'NonlinearLeastSquares', ...
%                     'Lower',[0.1 0.1], 'Upper',[1e10 1e10]);
%model = fittype('k*sqrt(p/(4*pi*x.^2))',dependent="y",independent="x",coefficients=["k" "p"]); % assume no vertical shift
%fittedmodel = fit(beacon2(:,1),beacon2(:,2),model,options);
%axis manual;
%plot(fittedmodel);
%xlabel("Distance [m]");
%ylabel("Voltage [V]");
%legend("Data Points", "Decay Relationship without Multipath Interference");
%hold off;
%model_coeffs = coeffvalues(fittedmodel);
%disp(model_coeffs);
%
%figure(3);
%scatter(beacon3(:,1), beacon3(:,2)); % plot our collected data
%hold on;
%options = fitoptions('Method', 'NonlinearLeastSquares', ...
%                     'Lower',[0.1 0.1], 'Upper',[1e10 1e10]);
%model = fittype('k*sqrt(p/(4*pi*x.^2))',dependent="y",independent="x",coefficients=["k" "p"]); % assume no vertical shift
%fittedmodel = fit(beacon3(:,1),beacon3(:,2),model,options);
%axis manual;
%plot(fittedmodel);
%xlabel("Distance [m]");
%ylabel("Voltage [V]");
%legend("Data Points", "Decay Relationship without Multipath Interference");
%hold off;
%model_coeffs = coeffvalues(fittedmodel);
%disp(model_coeffs);
