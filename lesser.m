function isBigger = lesser(x1, x2, e)
    isBigger = true;
    if abs(x1 - x2) < e || ...
        x2 < x1
        isBigger = false;
    end
end

