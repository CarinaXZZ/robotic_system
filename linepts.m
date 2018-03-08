function Line = linepts(a,b)

    if a(1) == b(1)
        m = 0; % if line is vertical, the slope is zero
        if a(2) == b(2) % is begin/end point are the same, output the point
            Line = [a;b];
            return
        else
            long = linspace( (min(a(2),b(2))),(max(a(2), b(2))), 20);
            x = a(1)*ones(1,length(long));  % x is a vector with the same value with size/5
        end
    else
        m = (a(2) - b(2)) / (a(1) - b(1));
        x = linspace( (min(a(1), b(1))) ,(max(a(1), b(1))),20 );
    end

    q = b(2) - b(1) * m;
%     y = round(m * x + q);
    y = (m * x + q);

    for i=1:length(x)
        Line(i,1) = x(i);
        Line(i,2) = y(i);
    end
end