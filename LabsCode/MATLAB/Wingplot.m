% Wing plot

% use chord length for reynolds number characteristic length (straight line between leading and
% trailing edge)


% Get values
% Reynolds Numbers
%vels = [5 10 15 20 25 30 35 40 45];
R5 = [1 2 3 4 5 6 7 8 9];
R15 = [1 2 3 4 5 6 7 8 9];
R45 = [1 2 3 4 5 6 7 8 9];

% Wing Lift Forces: Get Directly from COMSOL
LF5 = [1 1 1 1 1 1 1 1 1];
LF15 = [2 2 2 1 1 1 1 1 1];
LF45 = [3 3 3 1 1 1 1 1 1];

% Wing Lift Coefficients: Program COMSOL to compute these
C5 = [1 1 1 1 1 1 1 1 1];
C15 = [4 4 1 1 1 1 1 1 1];
C45 = [3 3 1 1 1 1 1 1 1];


subplot(2, 1, 1);
plot(R5,LF5,'k');
hold on;
plot(R15,LF15,'b');
plot(R45,LF45,'r');
ylabel('Lift Force [N]');
xlabel('Reynolds Number (Re = Vl)');
legend('Attack angle: 5°','Attack angle: 15°','Attack angle:45°');
title("Wings Lift Force vs. Reynolds Number ");
hold off;

subplot(2, 1, 2);
plot(R5,C5,'k');
hold on;
plot(R15,C15,'b');
plot(R45,C45,'r');

ylabel('Lift Coefficient');
xlabel('Reynolds Number (Re = Vl)');
legend('Attack angle: 5°','Attack angle: 15°','Attack angle: 45°');
title("Wings Lift Coefficient vs. Reynolds Number ");
hold off;


