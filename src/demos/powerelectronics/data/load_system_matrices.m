function [ M, Cq, E,  R, Qc, Dv, L] = load_system_matrices(framenumber, solvenumber, directory)
% This is an utility that loads all the matrices that have been saved on disk
% at each  my_system->StateSolveCorrection() when using
% SetDumpMatrices(true).
% The solver calls is asked to solve each:
%    |Dv| = [ M   Cq' ]^-1 * | R |
%    |DL|   [ Cq  E   ]      | Qc|
% For example, this dump mode is enabled in Irrlicht demos when pressing
% the F7 key: for each frame the .exe saves on disk the files
%  dump_0001_01_M.dat   dump_0001_01_Cq.dat .... etc.
%  dump_0002_01_M.dat   dump_0001_01_Cq.dat .... etc.
% In general, one gets
%  dump_<frame number>_<solver run>_<matrix name>.dat 
% where 
%  frame number:   increases from frame to frame, or stays the same for
%                  statics analysis
%  solver run:     in some solvers (ex. HHT) the solver is called more 
%                  than once per timestep, so it could be 01 02 03...
%  matrix name:    these matrices/vectors are saved:
%                  M  (sparse matrix format row,col,val)
%                  Cq (sparse matrix format row,col,val)
%                  E  (sparse matrix format row,col,val)
%                  R, Qc (rhs vectors) Dv, L (solved vectors),
%                  v_pre, v_pre (just to report the state at
%                  solver's evaluation)

if nargin < 3
    directory = '';
end

if nargin < 2
    solvenumber = 1;
end
  
prefix_a = num2str(framenumber,'%04d');
prefix_b = num2str(solvenumber,'%02d');

prefix = ['solve_', prefix_a, '_', prefix_b, '_'];

if (directory ~= '')
    prefix = [directory, '/', prefix];
end

M_compr = load( [prefix, 'M.dat'] );
M = spconvert(M_compr);

Cq_compr = load( [prefix, 'Cq.dat'] )
if size(Cq_compr)==[0 0]
    Cq =[]
else
    Cq = spconvert(Cq_compr);
end

E_compr = load( [prefix, 'E.dat'] );
if size(Cq_compr)==[0 0]
    E =[]
else
    E  = spconvert(E_compr);
end

R = load( [prefix, 'R.dat'] );

Qc = load( [prefix, 'Qc.dat'] );

Dv = load( [prefix, 'Dv.dat'] );

L = load( [prefix, 'L.dat'] );

end

