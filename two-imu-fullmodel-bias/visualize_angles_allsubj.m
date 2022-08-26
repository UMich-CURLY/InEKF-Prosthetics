subjdates = ["AB06","10_09_18";
             "AB07","10_14_18";
             "AB08","10_21_2018";
             "AB09","10_21_2018";
             "AB10","10_28_2018";
             "AB11","10_28_2018";
             "AB12","11_04_2018";
             "AB13","11_04_2018";
             "AB14","11_25_2018";];
subj_errors = {};
trials_persub = 5;
type = "ccw_slow";
for subjdatenum = 1:length(subjdates)
    subj = subjdates(subjdatenum,1);
    date = subjdates(subjdatenum,2);
    % load the file names for the subject
    % add below function to path, from scripts folder from base
    % addpath 'C:\Users\<user>\Documents\InEKF-Prosthetics\scripts'
    dataset_params_list = generate_filenames(subj,date,type);

    for data_param_num = 1:length(dataset_params_list)
        clear fkTable;  % force loading of fktable each time, could edit this in
                        % load_dataset but I like the flexibility
        clear log;
        dataset_params = dataset_params_list{data_param_num};
        % run the test on this set
        run runFilter.m;
        % extract y euler error from the log
        % 5 trials per subject, currently
        % Little need to assign time sync, since this test is concerned w/
        % convergence
        column = trials_persub*(subjdatenum-1)+data_param_num;
        subj_errors(1:length(log),column) = log(1:end,8); % figure out if this is correct, if it needs expansion, etc.
    end

end
save('fullrun','subj_errors')
% Look into pickling the subj_errors
% may need to deal with time synchronization issues - look into this if it
% comes up
figure;
for i = 1:width(subj_errors)
    col = cell2mat(subj_errors(:,i));
    l = plot(1:length(col),col);
    alpha(l, 0.5)
    hold on;
end
avg = mean(subj_errors,2);
plot(avg,'Color','blk','LineStyle','--')
xlabel("Time")
ylabel("Error (rad)")
title("Error in thigh angle estimate over time")
