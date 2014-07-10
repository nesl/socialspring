%% Housekeeping
clc; close all; clear all;

%% sweep arrays
noise_arr = [0.001 0.01 0.05 0.1 0.2 0.5 0.8 1];
%noise = 0.01;
eprob = 1;
k_world = 20;
k_dist = 1;
%k_encounter_arr = [0.1 0.5 1 5 10 12 15 18 20];
kenc = 1;

%% evaluate
num_iters = 5;
ea_arr = [];
eb_arr = [];

for s=1:length(noise_arr)
    noise = noise_arr(s);
    fprintf('noise = %.4f\n  ', noise);

        
    eb_sum = 0;
    ea_sum = 0;
    
    for i=1:num_iters
        seed = 100 + i;
        [eb,ea] = getEncounterResults(seed, noise, eprob, k_world, k_dist, kenc);
        ea_sum = ea_sum + mean(ea);
        eb_sum = eb_sum + mean(eb);
        fprintf('.');
    end
    ea_arr = [ea_arr (ea_sum/num_iters)];
    eb_arr = [eb_arr (eb_sum/num_iters)];
    fprintf(' %.2f -> %.2f \n', (eb_sum/num_iters), (ea_sum/num_iters));
end

save('cache/sense_noise');

%% plot
load('cache/sense_noise');
cfigure(14,4);
percentages = 100*(eb_arr - ea_arr)./ea_arr;
semilogx(noise_arr, percentages, 'o-b', 'LineWidth',2,'MarkerSize', 10);
grid on;
xlabel('Noise Level','FontSize',14);
ylabel({'Error','Reduction (%)'},'FontSize',14);
saveplot('../BuildSys14/figures/sweep_noise_small');
