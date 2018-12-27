clear;
clc;

fileID = fopen('data.txt','r');
formatSpec = '%f %f %f';

data = textscan(fileID,formatSpec);

p = 0.2;
i = 0.0004;
d = 3;
p_e = 0;
i_e = 0;
d_e = 0;
result = zeros(1,length(data{1}));
cte_arr = data{1};
steer_arr = data{3};
for step=1:length(data{1})
    cte = cte_arr(step);
    [p_error, i_error, d_error] = pidupdate(cte, p_e, i_e);
    p_e = p_error;
    i_e = i_error;
    d_e = d_error;
    result(step) = pidctrl(p, i, d, p_e, i_e, d_e);
end

figure(1)
plot(steer_arr);
hold on;
plot(result);
grid on;
hold off;