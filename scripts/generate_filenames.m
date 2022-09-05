function files = generate_filenames(subject, date, type)
    files = {};
    keynames = ["filepre","motfile","imufile","fpfile","gonfile","osimfile"];
    filepre = "../../" + subject + "/" + subject + "/";
    for i = ["1","2","3","4","5"]
        filenames = [filepre];
        for cat = ["ik","imu","fp","gon"]
            % Find new iterative/exhaustive way to go through trials
            filedir = date + "/levelground/" + cat + "/";
            trial = dir(filepre+filedir+"*"+type+"*"+i+"*.mat");
            if length(trial) < 1
                continue
            end
            if length(trial) > 1
                % Hacky, but should work given they are in lexigraphic
                % order. Also need to figure out how to integrate _02_02
                % and _01_01 case
                trial = trial(1);
            end
            filename = filedir + trial.name;
            filenames = [filenames filename];
        end
        osimfile = "osimxml/" + subject + ".osim";
        filenames = [filenames osimfile];
        if length(keynames) ~= length(filenames)
            continue
        end
        params = containers.Map(keynames,filenames);
        files{str2num(i)} = params;
    end
end