load("lidar_data")
r_react = 0.4;      %reaction radius
r_avoid = 3;        %avoidance radius
n_part = 4;         %number of partitions for reactive layer
n_sec = 72;
a = r_avoid;
b = 1;
vfh_threshold = 0.5;
window_size_threshold = 5;
angle = 0;

for i = 1:length(data)
    laser_scan = data{i};
    
    p_plan = (laser_scan>=r_react)-(laser_scan>r_avoid);
    p_plan = laser_scan.*p_plan;
    
    if(any(p_plan) && mod(i,2)==0)%check p_plan and if new scan
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %                    Do VFH Approach         %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        fprintf("planning...\n");
        figure
        for z = 1:n_sec %iterate through VFH sectors
            start_pos = (z-1)*360/n_sec + 1;
            end_pos = (z)*360/n_sec;
            sect{z} = p_plan(start_pos:end_pos);
            h(z) = 0;
            for j = length(sect{z})
                if(sect{z}(j) ~=0)
                    h(z) = h(z) + a - b*sect{z}(j);
                end
            end
        end
        plot(1:72,h)
        
        %select candidate sectors
        candidate_sectors = [];
        for z = 1:length(h)
            if h(z)<vfh_threshold
                candidate_sectors = [candidate_sectors z];
            end
        end
        
        
        clear window;
        %select closest suitable section
        candidate_sectors = [candidate_sectors candidate_sectors(1)]; %basically fixes the "wrap-around"
        window_num = 1;
        window{1} = candidate_sectors(1);
        for z = 2:length(candidate_sectors)-1
            if isConsecutive(candidate_sectors(z-1),candidate_sectors(z), n_sec)
                window{window_num} = [window{window_num} candidate_sectors(z)];
            else
                window_num = window_num + 1;
                window{window_num} = candidate_sectors(z);
            end
        end
        %fix wraparound
        if(isConsecutive( window{end}(end), window{1}(1), n_sec))
            window{1} = [window{end}(1:end-1) window{1}]; %merge first and last window
            window{end} = {}; %remove last window (duplicate)
        end
        
        %check window sizes
        for y = 1:length(window)
            curr_win = window{y};
            if (length(curr_win)>window_size_threshold)
                window_center(y) = curr_win(ceil(length(curr_win)/2));
                window_score(y) = abs(window_center(y) - n_sec/2);
            end
        end
        [~,window_ind] = max(window_score);
        goal_window = window_center(window_ind)
        goal_hdg = angle + (goal_window)*pi/36;
        fprintf("plan set goal_hdg: %f\n",goal_hdg);
        
        
    else
        goal_hdg = angle;
    end
end