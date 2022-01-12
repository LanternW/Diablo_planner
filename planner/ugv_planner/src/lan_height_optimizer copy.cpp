/*
#include <ugv_planner/lan_height_optimizer.h>
using namespace std;    
using namespace Eigen;

int lanHeightOptimizer::heightCurveGeneration( 
        const double max_height,
        const double min_height,
        vector<double> ceil_heights)
{   
    const int nx  = ceil_heights.size();

    // upper and lower bounds for all unknowns x
    double  xupp[nx];    
    char   ixupp[nx];
    double  xlow[nx];
    char   ixlow[nx];

    // stacking all bnounds
    for(int i = 0; i < nx; i++)
    {   
        ixlow[i] = 1;
        ixupp[i] = 1;
        xlow[i]  = min_height;
        xupp[i]  = max_height;
    }


    int my = 0; // equality constarins
    double* b=0;

    int nnzA  = 0 ;
    double* dA = 0;
    int* irowA = 0;
    int* jcolA = 0;



    ////////////Cx < c
    vector<int> ieq_con_indexes;
    for(int i = 0 ; i < ceil_heights.size() ; i ++)
    {
        if (ceil_heights[i] < max_height)
        {
            ieq_con_indexes.push_back(i);
        }
    }
    const int mz  = ieq_con_indexes.size(); 
    char iclow[mz];
    char icupp[mz];
    double clow[mz];
    double cupp[mz];
 
    for(int i = 0; i < mz; i++)
    {   
        iclow[i] = 0; 
        icupp[i] = 1;

        clow[i]  = 0;
        cupp[i]  = ceil_heights[ieq_con_indexes[i]];  //x[i] < height[i]
    }

    // add velocity     constraints on all segments of the trajectory
    // add acceleration constraints on all segments of the trajectory
    // linear constraints, equations
    
    int nnzC = ieq_con_indexes.size(); 
    int irowC[nnzC];
    int jcolC[nnzC];
    double dC[nnzC];
   
    int nn_idx  = 0;

    for(int i = 0; i < nnzC; i++)
    {  
        dC[i] = 1;
        irowC[nn_idx]   = i;
        jcolC[nn_idx]   = ieq_con_indexes[i]; 
        nn_idx ++;
    }

    double  c[nx];

    for(int i = 0; i < nx; i++)
    {
        c[i] = -1.0;
    }
    //////////////////////////////////////
/*
    const int nnzQ = 0;
    int*    irowQ = 0; 
    int*    jcolQ = 0;
    double *   dQ = 0;
    

    //const int nnzQ = 5;
    const int nnzQ = 2 * nx - 1;
    int    irowQ[nnzQ]; 
    int    jcolQ[nnzQ];
    double    dQ[nnzQ];

    int idx = 0;
    double k = 10;

    for( int i = 0; i < nx; i ++ )
    {
        irowQ[idx] = i;   
        jcolQ[idx] = i; 
        if(i == 0 || i == nx - 1)
            dQ[idx]    = k * 1.0;
        else
            dQ[idx]    = k * 2.0;
        idx ++ ;
    }

    for( int i = 0; i < nx - 1; i ++ )
    {
        irowQ[idx] = i+1;   
        jcolQ[idx] = i; 
        dQ[idx]    = k * (-1.0);
        idx ++ ;
    }
    
    

    QpGenSparseMa27 * qp 
    = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );



    QpGenData * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
        c,      irowQ,  nnzQ,   jcolQ,  dQ,
        xlow,   ixlow,  xupp,   ixupp,
        irowA,  nnzA,   jcolA,  dA,     b,
        irowC,  nnzC,   jcolC,  dC,
        clow,   iclow,  cupp,   icupp );
    


    QpGenVars      * vars  = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid = (QpGenResiduals *) qp->makeResiduals( prob );
    GondzioSolver  * s     = new GondzioSolver( qp, prob );
    
    // Turn Off/On the print of the solving process
    // s->monitorSelf();
    int ierr = s->solve(prob, vars, resid);

    if( ierr == 0 ) 
    {
        double d_var[nx];
        vars->x->copyIntoArray(d_var);

        h_ctrl_points.clear();
        for(int i = 0; i < nx; i++ )
        {   
            h_ctrl_points.push_back(d_var[i]);
          
        }   
    } 
    else if( ierr == 3)
        cout << "The program is provably infeasible, check the formulation.\n";
    else if (ierr == 4)
    {
        cout << "The program is very slow in convergence, may have numerical issue.\n";
        double d_var[nx];
        vars->x->copyIntoArray(d_var);

        h_ctrl_points.clear();
        for(int i = 0; i < nx; i++ )
        {   
            h_ctrl_points.push_back(d_var[i]); 
        } 
    }
    else
        cout << "Solver numerical error.\n";
    
    return ierr;
}
*/