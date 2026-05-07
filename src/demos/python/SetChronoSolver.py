import pychrono as chrono

def SetChronoSolver(sys, solver_type, integrator_type, num_threads_mkl=1, verbose=False):
    contact_method = sys.GetContactMethod()
    slvr_type = solver_type
    intg_type = integrator_type
    
    prefix = "[SetChronoSolver] "

    # For NSC systems, suggest implicit linearized Euler and an iterative VI solver
    if verbose:
        if contact_method == chrono.ChContactMethod_NSC:
            if intg_type != chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED:
                print(f"{prefix}NSC system - recommended integrator: EULER_IMPLICIT_LINEARIZED")
            
            if (slvr_type != chrono.ChSolver.Type_BARZILAIBORWEIN and 
                slvr_type != chrono.ChSolver.Type_APGD and 
                slvr_type != chrono.ChSolver.Type_PSOR):
                print(f"{prefix}NSC system - recommended solver: BARZILAIBORWEIN")

    # Barzilai-Borwein cannot be used with stiffness matrices
    if (slvr_type == chrono.ChSolver.Type_BARZILAIBORWEIN and 
        not sys.GetSystemDescriptor().SupportsSchurComplement()):
        print(f"{prefix}BARZILAIBORWEIN cannot be used if:\n"
              f" - there are stiffness or damping matrices, or\n "
              f" - no inverse mass matrix was provided")
        return False

    # If the requested direct sparse solver module is not enabled, default to SPARSE_QR
    if slvr_type == chrono.ChSolver.Type_PARDISO_MKL:
        if not hasattr(chrono, 'ChSolverPardisoMKL'):
            slvr_type = chrono.ChSolver.Type_SPARSE_QR
            print(f"{prefix}Chrono::PardisoMKL not enabled. Setting solver to SPARSE_QR")
    elif slvr_type == chrono.ChSolver.Type_MUMPS:
        if not hasattr(chrono, 'ChSolverMumps'):
            slvr_type = chrono.ChSolver.Type_SPARSE_QR
            print(f"{prefix}Chrono::MUMPS not enabled. Setting solver to SPARSE_QR")

    # Set solver
    if slvr_type == chrono.ChSolver.Type_PARDISO_MKL:
        if hasattr(chrono, 'ChSolverPardisoMKL'):
            solver = chrono.ChSolverPardisoMKL(num_threads_mkl)
            solver.LockSparsityPattern(True)
            sys.SetSolver(solver)
            if verbose:
                print(f"{prefix}Setting solver PARDISO_MKL with locked sparsity pattern")
                
    elif slvr_type == chrono.ChSolver.Type_MUMPS:
        if hasattr(chrono, 'ChSolverMumps'):
            solver = chrono.ChSolverMumps()
            solver.LockSparsityPattern(True)
            solver.EnableNullPivotDetection(True)
            solver.GetMumpsEngine().SetICNTL(14, 50)
            sys.SetSolver(solver)
            if verbose:
                print(f"{prefix}Setting solver MUMPS with locked sparsity pattern")
                
    else:
        sys.SetSolverType(slvr_type)
        if verbose:
            type_name = chrono.ChSolver.GetTypeAsString(slvr_type)
            print(f"{prefix}Setting solver {type_name}")
            
        if slvr_type in [chrono.ChSolver.Type_SPARSE_LU, chrono.ChSolver.Type_SPARSE_QR]:
            solver = chrono.CastToChDirectSolverLS(sys.GetSolver())
            solver.LockSparsityPattern(False)
            solver.UseSparsityPatternLearner(False)
            
        elif slvr_type in [chrono.ChSolver.Type_BARZILAIBORWEIN, 
                           chrono.ChSolver.Type_APGD, 
                           chrono.ChSolver.Type_PSOR]:
            solver = chrono.CastToChIterativeSolverVI(sys.GetSolver())
            solver.SetMaxIterations(100)
            solver.SetOmega(0.8)
            solver.SetSharpnessLambda(1.0)
            
        elif slvr_type in [chrono.ChSolver.Type_BICGSTAB, 
                           chrono.ChSolver.Type_MINRES, 
                           chrono.ChSolver.Type_GMRES]:
            solver = chrono.CastToChIterativeSolverLS(sys.GetSolver())
            solver.SetMaxIterations(200)
            solver.SetTolerance(1e-10)
            solver.EnableDiagonalPreconditioner(True)

    # Set Integrator
    sys.SetTimestepperType(intg_type)
    if verbose:
        type_name = chrono.ChTimestepper.GetTypeAsString(intg_type)
        print(f"{prefix}Setting integrator {type_name}")

    if intg_type == chrono.ChTimestepper.Type_HHT:
        integrator = chrono.CastToChTimestepperHHT(sys.GetTimestepper())
        integrator.SetAlpha(-0.2)
        integrator.SetMaxIters(50)
        integrator.SetAbsTolerances(1e-4, 1e2)
        integrator.SetStepControl(False)
        integrator.SetJacobianUpdateMethod(
            chrono.ChTimestepperImplicit.JacobianUpdate_EVERY_ITERATION)
            
    elif intg_type == chrono.ChTimestepper.Type_EULER_IMPLICIT:
        integrator = chrono.CastToChTimestepperEulerImplicit(sys.GetTimestepper())
        integrator.SetMaxIters(50)
        integrator.SetAbsTolerances(1e-4, 1e2)

    return True