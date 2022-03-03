function consec = isConsecutive(cand1, cand2, nsec)
    if (mod(cand2 - cand1,nsec) == 1 || mod(cand2 - cand1,nsec) == nsec-1)
       consec = 1;
    else
        consec = 0;
    end
end