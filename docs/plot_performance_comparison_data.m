
close all, clear;

f_path = 'C:\Users\jeffs\OneDrive\Documents\Coding_Projects\git\Matrix-Batch-Optimization\docs\la_performance_runs_v2.csv';

tbl = readtable(f_path);

x_data = 1:20;
idx_mkl = tbl.Backend == "mkl";
idx_eigen = tbl.Backend == "eigen";
idx_std = tbl.Backend == "std";

figure;
plot(x_data, tbl.Time_s(idx_mkl), 'r.-', x_data, tbl.Time_s(idx_eigen), 'g.-', x_data, tbl.Time_s(idx_std), 'b.-')
legend('MKL', 'Eigen', 'STD')

std.mean    = mean(tbl.Time_s(idx_std));
mkl.mean    = mean(tbl.Time_s(idx_mkl));
eigen.mean  = mean(tbl.Time_s(idx_eigen));

std.std    = std(tbl.Time_s(idx_std));
mkl.std    = std(tbl.Time_s(idx_mkl));
eigen.std  = std(tbl.Time_s(idx_eigen));

figure;
bar(["MKL", "Eigen", "STD"], [mkl.mean, eigen.mean, std.mean])

%% value as bars

close all; clear;

f_path = 'C:\Users\jeffs\OneDrive\Documents\Coding_Projects\git\Matrix-Batch-Optimization\docs\la_performance_runs_v2.csv';
tbl = readtable(f_path);

% Identify each backend
idx_mkl   = tbl.Backend == "mkl";
idx_eigen = tbl.Backend == "eigen";
idx_std   = tbl.Backend == "std";

% Compute means and std devs
mean_vals = [
    mean(tbl.Time_s(idx_mkl));
    mean(tbl.Time_s(idx_eigen));
    mean(tbl.Time_s(idx_std))
];

std_vals = [
    std(tbl.Time_s(idx_mkl));
    std(tbl.Time_s(idx_eigen));
    std(tbl.Time_s(idx_std))
];

labels = {'MKL', 'Eigen', 'STD'};

% --- Plot ---
% --- Plot ---
figure; hold on;

% Create bar plot at x = 1,2,3
b = bar(1:3, mean_vals);

% Force only three x-ticks and label them
ax = gca;
ax.XTick      = 1:3;
ax.XTickLabel = labels;

% Add standard deviation error bars
er = errorbar(1:3, mean_vals, std_vals, std_vals, ...
              'k', 'linestyle','none', 'LineWidth', 1.5);

ylabel('Runtime (s)');
title('Mean Runtime with Standard Deviation (100{,}000-Track Batch Update)');
grid on;
box on;
hold off;
