function files = generate_filenames(subject, date)
    files = {};
    keynames = ["filepre","motfile","imufile","fpfile","gonfile","osimfile"];
    for i = ["1","2","3","4","5"]
        filenames = "../../" + subject + "/" + subject + "/";
        for cat = ["ik","imu","fp","gon"]
            filename = date + "/levelground/" + cat + "/" + ...
                "levelground_ccw_slow_0" + i + "_01.mat";
            filenames = [filenames filename];
        end
        osimfile = "osimxml/" + subject + ".osim";
        filenames = [filenames osimfile];
        params = containers.Map(keynames,filenames);
        files{str2num(i)} = params;
    end
end