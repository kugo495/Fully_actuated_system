function [sys,x0,str,ts,simStateCompliance] = sub_system_OL(t,x,u,flag)

switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes = simsizes;

sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [3 -0.5 -1.5];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
g = x(1)-3.5; f = sin(x(1)); 
dg = x(2); df = cos(x(1))*x(2); ddf = -sin(x(1))*x(2)*x(2)+cos(x(1))*x(3);
f1 = df+x(1)*(x(2)-f);
df1 = ddf+x(2)*(x(2)-f)+x(1)*(x(3)-df);
f2 = g*df1+dg*(x(3)-f1);
sys(1) = x(2);
sys(2) = x(3);
sys(3) = f2/g+g*u;
    


function sys=mdlUpdate(t,x,u)

sys = [];

function sys=mdlOutputs(t,x,u)
sys(1) = x(1);
sys(2) = x(2);
sys(3) = x(3);

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];

