%%
clc;clear;
%generate fake scan
laser_scan = 10*rand(1,360);

%react stuff
r_react = 1.5;      %reaction radius
n_part = 6;         %number of partitions

%
n_sec = 72;         %number of sectors for vfh
r_avoid = 5;        %avoidance radius
a = r_avoid;
b = 1;
vfh_threshold = 0.5;


%fix fake scan for VFH
laser_scan = laser_scan.*(laser_scan>r_react);
%%
p_react = laser_scan<r_react; %find points inside reaction radius;
p_react = laser_scan.*p_react;

p_plan = (laser_scan>=r_react)-(laser_scan>r_avoid);
p_plan = laser_scan.*p_plan;

if(any(p_react))    %if any points within reaction radius
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %       reactive measures       %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %partition the cloud into zones
    l = length(p_react);
    
    %first zone straddles the forward direction
    zone{1} = [p_react(1:(l/n_part/2)) p_react(l*(1-1/n_part/2)+1:end)];
    
    %other n zones
    for z = 1:n_part-1
        zone{z+1} = p_react((l/n_part)*(z-0.5)+1:(l/n_part)*(z+0.5));
    end
    
    %iterate over each zone
    if(any(zone{1}) || any(zone{n_part/2+1}))
        for z = 1:length(zone) %exclude forward and backward directions
            count(z) = sum(any(zone{z}));
        end
        
    elseif(~any(zone{1}))
        %drive forward
    else
        %drive backward
    end
    
else
    if(any(p_plan))
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %        Do VFH Approach         %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
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
                window_center(y) = median(window);
                window_score(y) = abs(window_center - n_sec/2);
            end
        end
        [~,window_ind] = max(window_score);
        goal_window = window_center(window_ind);
        
    end
    
    goal_hdg = (goal_window-1)*pi/36;
    
end

end



