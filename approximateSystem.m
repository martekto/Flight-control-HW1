function sysApprox = approximateSystem(AFull,BFull,modeName)

C = eye(2); D = 0;
A = AFull(1:2,1:2);
B = BFull(1:2,:);

TimeUnit = 'seconds';
switch modeName
    case {"short period","SP"}
        A(2,1) = 1; % approximation for steady horizontal flight without wind
        InputName = {'\delta_t','\delta_e'};
        StateName = {'q','\alpha'};

    case {"dutch roll","DR"}
        InputName = {'\delta_a','\delta_r'};
        StateName = {'r','\beta'};

    otherwise
        error("Unexpected mode name.")

end
OutputName = StateName;

sysApprox = ss(A,B,C,D);
sysApprox.InputName = InputName;
sysApprox.StateName = StateName;
sysApprox.OutputName = OutputName;
sysApprox.TimeUnit = TimeUnit;

end