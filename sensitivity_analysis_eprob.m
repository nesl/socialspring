%% Housekeeping
clc; close all; clear all;

%% sweep arrays
noise = 0.01;
eprob_arr = [0.05 0.1 0.2 0.5 0.75 1.0];
kworld = 20;
kdist = 1;
kenc = 1;

%% evaluate
num_iters = 5;
ea_arr = [];
eb_arr = [];

for s=1:length(eprob_arr)
    eprob = eprob_arr(s);
    fprintf('eprob = %.4f\n  ', eprob);

        
    eb_sum = 0;
    ea_sum = 0;
    
    for i=1:num_iters
        seed = 100 + i;
        [eb,ea] = getEncounterResults(seed, noise, eprob, kworld, kdist, kenc);
        ea_sum = ea_sum + mean(ea);
        eb_sum = eb_sum + mean(eb);
        fprintf('.');
    end
    ea_arr = [ea_arr (ea_sum/num_iters)];
    eb_arr = [eb_arr (eb_sum/num_iters)];
    fprintf(' %.2f -> %.2f \n', (eb_sum/num_iters), (ea_sum/num_iters));
end

save('cache/sense_eprob');

%% plot
cfigure(14,4);
percentages = 100*(eb_arr - ea_arr)./ea_arr;
plot(eprob_arr, percentages, 'o-b', 'LineWidth',2,'MarkerSize', 10);
grid on;
xlabel('Encounter Probability','FontSize',14);
ylabel({'Error','Reduction (%)'},'FontSize',14);
saveplot('../BuildSys14/figures/sweep_eprob_small');
