% Wing plot


% Get values
R1 = [1 2 3 4 5 6 7 8 9]
R2 = [1 2 3 4 5 6 7 8 9]
R3 = [1 2 3 4 5 6 7 8 9]
C5 = [1 1 1 1 1 1 1 1 1]
C15 = [4 4 1 1 1 1 1 1 1]
C45 = [3 3 1 1 1 1 1 1 1]
LF5 = [1 1 1 1 1 1 1 1 1]
LF15 = [2 2 2 1 1 1 1 1 1]
LF45 = [3 3 3 1 1 1 1 1 1]

subplot(2, 1, 1)
plot(R1,LF5,'k')
hold on
plot(R2,LF15,'b')
plot(R3,LF45,'r')

ylabel('Lift Force [N]')
xlabel('Reynolds Number (Re = Vl)')
legend('Attack angle: 5°','Attack angle: 15°','Attack angle:45°')
title("Wings Lift Force vs. Reynolds Number ")
hold off

subplot(2, 1, 2)
plot(R1,C5,'k')
hold on
plot(R2,C15,'b')
plot(R3,C45,'r')

ylabel('Lift Coefficient')
xlabel('Reynolds Number (Re = Vl)')
legend('Attack angle: 5°','Attack angle: 15°','Attack angle: 45°')
title("Wings Lift Coefficient vs. Reynolds Number ")
hold off


