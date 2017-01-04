function nc = noCollision(n2, n1, o)
    A = [n1(1) n1(2)];
    B = [n2(1) n2(2)];
    obs = [o(1) o(2) o(1)+o(3) o(2)+o(4)];
    
    C1 = [obs(1),obs(2)];
    D1 = [obs(1),obs(4)];
    C2 = [obs(1),obs(2)];
    D2 = [obs(3),obs(2)];
    C3 = [obs(3),obs(4)];
    D3 = [obs(3),obs(2)];
    C4 = [obs(3),obs(4)];
    D4 = [obs(1),obs(4)];
    
    % Check if path from n1 to n2 intersects any of the four edges of the
    % obstacle
    
    ints1 = ccw(A,C1,D1) ~= ccw(B,C1,D1) && ccw(A,B,C1) ~= ccw(A,B,D1); 
    ints2 = ccw(A,C2,D2) ~= ccw(B,C2,D2) && ccw(A,B,C2) ~= ccw(A,B,D2);
    ints3 = ccw(A,C3,D3) ~= ccw(B,C3,D3) && ccw(A,B,C3) ~= ccw(A,B,D3);
    ints4 = ccw(A,C4,D4) ~= ccw(B,C4,D4) && ccw(A,B,C4) ~= ccw(A,B,D4);
    if ints1==0 && ints2==0 && ints3==0 && ints4==0
        nc = 1;
    else
        nc = 0;
    end
end