subjdates = ["AB06","10_09_18";
             "AB07","10_14_18";
             %"AB08","10_21_18";
             %"AB09","10_21_18";
             %"AB10","10_28_18";
             %"AB11","10_28_18";
             %"AB12","11_04_18";
             %"AB13","11_04_18";
             "AB14","11_25_18";];
subj_errors = {};
trials_persub = 5;
for i = 1:length(subjdates)
    subj = subjdates(i,1);
    date = subjdates(i,2);
    % load the file names for the subject
    % add below function to path, from scripts folder from base
    % addpath 'C:\Users\gluef\Documents\InEKF-Prosthetics\scripts'
    dataset_params_list = generate_filenames(subj,date);

    for j = 1:length(dataset_params_list)
        clear fkTable;  % force loading of fktable each time, could edit this in
                        % load_dataset but I like the flexibility
        dataset_params = dataset_params_list{j};
        % run the test on this set
        run load_dataset.m;
        run runFilter.m;
        % extract y euler error from the log. If this is the first time, get
        % times as well, or shift based on observed times
        if length(subj_errors) < 1
            subj_errors{1:length(log),1} = log{1:end,1};
        end
        % 5 trials per subject, currently
        subj_errors{1:end,trials_persub*(i-1)+j} = log{1:end,8}; % figure out if this is correct, if it needs expansion, etc.
    end

end
% may need to deal with time synchronization issues - look into this if it
% comes up
subjects = cell2mat(subj_errors{1:end,2:end});
% also, how to visualize subjects vs. trials? just run across all trials?

plot(times, subjects);
xlabel("Time")
ylabel("Error (rad)")
title("Error in thigh angle estimate over time")
