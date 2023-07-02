voltage_power=16.8
voltage_power_supply=10
Uq=voltage_power_supply/3





theta = linspace(0, 4*pi, 360);
angle_el=theta;
% 帕克逆变换
Ualpha =  -Uq*sin(angle_el); 
Ubeta =   Uq*cos(angle_el); 

% 克拉克逆变换
Ua = Ualpha + voltage_power_supply/2;
Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;


plot(theta,Ua);
hold on
plot(theta,Ua-Ub);
hold on
plot(theta,Ua-Uc);

% polarplot(theta,Ua);
% hold on
% polarplot(theta,Ub);
% hold on
% polarplot(theta,Uc);


