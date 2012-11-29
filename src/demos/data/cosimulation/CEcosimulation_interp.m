function cosim_interpolator(block)
% Level-2 M file S-Function for multirate demo:
%
% The S-function will take one 1-D input, and output a 1-D signal that
% is 6 times downsampled version of the input signal.

%   Copyright 2007-2009 The MathWorks, Inc.

  setup(block);
%endfunction

function setup(block)
  %% Register number of input port and output port
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;
  
  %% Setup functional port properties
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).DatatypeID    = 0;
  block.InputPort(1).Complexity    = 0;
  block.InputPort(1).Dimensions    = 1;
  block.InputPort(1).SamplingMode  = 0;
  
  block.InputPort(2).DatatypeID    = 0;
  block.InputPort(2).Complexity    = 0;
  block.InputPort(2).Dimensions    = -1;
  block.InputPort(2).SamplingMode  = 0;
  
  block.OutputPort(1).DatatypeID   = 0;
  block.OutputPort(1).Complexity   = 0;
  block.OutputPort(1).Dimensions   = -1;
  block.OutputPort(1).SamplingMode = 0;
  
  block.InputPort(1).SampleTime  = [-1 0];
  block.InputPort(2).SampleTime  = [-1 0];
  block.OutputPort(1).SampleTime = [0 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  block.SetAccelRunOnTLC(true);
  
  %% Reg methods
  block.RegBlockMethod('SetInputPortSampleTime',  @SetInpPortST);
  block.RegBlockMethod('SetOutputPortSampleTime', @SetOutPortST);
  block.RegBlockMethod('SetInputPortDimensions',  @SetInpPortDims);
  block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
%endfunction

% Set input port sample time
function SetInpPortST(block, idx, st)
  block.InputPort(1).SampleTime = st;
  block.InputPort(2).SampleTime = st;
  block.OutputPort(1).SampleTime = [0, 0];
%endfunction

% Set output port sample time
function SetOutPortST(block, idx, st)
  block.OutputPort(1).SampleTime = [0, 0];
  block.InputPort(1).SampleTime = st;
  block.InputPort(2).SampleTime = st;
%endfunction

% Set input port dimensions
function SetInpPortDims(block, idx, di)
    block.InputPort(idx).Dimensions = di;
%   if idx ~= 2
%     DAStudio.error('Simulink:blocks:multirateOnePort'); 
%   end
%   
%   if block.InputPort(1).SamplingMode == 0
%     width = prod(di);
%     if width ~= 1
%       DAStudio.error('Simulink:blocks:multirateInvaliDimension'); 
%     end
%   end
%   
%   block.InputPort(1).Dimensions = 1;
%   block.InputPort(2).Dimensions = di;
%   block.OutputPort(1).Dimensions = di;
%endfunction

% Set output port dimensions
function SetOutPortDims(block, idx, di)
    block.OutputPort(idx).Dimensions = di;
%   if block.InputPort(1).SamplingMode == 0
%     width = prod(di);
%     if width ~= 1
%       DAStudio.error('Simulink:blocks:multirateInvaliDimension'); 
%     end
%   end
%   
%   block.InputPort(1).Dimensions = 1;
%   block.InputPort(2).Dimensions  = di;
%   block.OutputPort(1).Dimensions = di;
%endfunction

% Do post-propagation process
function DoPostPropSetup(block)
%% Setup DWork
  block.NumDworks = 4;
  block.Dwork(1).Name = 't_B';

  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 0;
  block.Dwork(1).UsedAsDiscState = 1;

  block.Dwork(2).Name = 'x_B';

  block.Dwork(2).Dimensions      = block.OutputPort(1).Dimensions;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 0;
  block.Dwork(2).UsedAsDiscState = 1;
  
  
  block.Dwork(3).Name = 't_A';

  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;
  block.Dwork(3).Complexity      = 0;
  block.Dwork(3).UsedAsDiscState = 1;
  
  block.Dwork(4).Name = 'x_A';

  block.Dwork(4).Dimensions      = block.OutputPort(1).Dimensions;
  block.Dwork(4).DatatypeID      = 0;
  block.Dwork(4).Complexity      = 0;
  block.Dwork(4).UsedAsDiscState = 1;
  
  
  block.RegBlockMethod('Outputs', @OutputNonFrame);  

%endfunction

function Start(block)

  block.Dwork(1).Data = 0;
  block.Dwork(2).Data = 0;
  block.Dwork(3).Data = 0;
  block.Dwork(4).Data = 0;
  
%endfunction

function OutputNonFrame(block)

  if block.Dwork(1).Data ~= block.InputPort(1).Data
     block.Dwork(3).Data = block.Dwork(1).Data;  % t_A
     block.Dwork(4).Data = block.Dwork(2).Data;  % x_A
     block.Dwork(1).Data = block.InputPort(1).Data;  % t_B
     block.Dwork(2).Data = block.InputPort(2).Data; % x_B
  end
  
  t = block.CurrentTime;
  t_A = block.Dwork(3).Data;
  x_A = block.Dwork(4).Data;
  t_B = block.Dwork(1).Data;
  x_B = block.Dwork(2).Data;
  Dt = t_B-t_A;
  
  if Dt > 0
    block.OutputPort(1).Data =  ((t_B-t)/Dt)*x_A + ((t-t_A)/Dt)*x_B;
  end
  if t > t_B
    block.OutputPort(1).Data =  x_B;
  end
  if t < t_A
    block.OutputPort(1).Data =  x_A;
  end
  
  
  %block.OutputPort(1).Data = block.Dwork(1).Data;
  
  
%endfunction


