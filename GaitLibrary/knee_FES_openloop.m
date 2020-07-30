function [ u ] = knee_FES_openloop( thisErrorKnee, STIM )

    if thisErrorKnee >= 0
        u = STIM;
    else
        u = -STIM;
    end

end

