function inside_map = makeReducedMap(map,margin)
    
inside_map = map;
    for ii = 1:size(map,1)-2
       inside_map(ii+1,:) = insider(map(ii,:),map(ii+1,:),map(ii+2,:),margin,map);   
    end

inside_map(1,:) = insider(map(end,:),map(1,:),map(2,:),margin,map);
inside_map(end,:) = insider(map(end-1,:),map(end,:),map(1,:),margin,map);

end