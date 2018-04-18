function DecodedStr = bits2ASCII(u)
    w = [64 32 16 8 4 2 1]; 
    Nbits = numel(u);
    Ny = Nbits/7;
    y = zeros(1,Ny);
    for i = 0:Ny-1
        y(i+1) = w*u(7*i+(1:7));
    end
    DecodedStr = char(y);
end