#ifndef _HEIGHT_GENERATOR_OOQP_H_
#define _HEIGHT_GENERATOR_OOQP_H_


#include "bspline_opt/lbfgs.hpp"
#include <Eigen/Eigen>

/*
Eigen::VectorXd optimal_list;
double max_height;
double min_height;
vector<double> ceil_heights;
*/

class lanHeightOptimizer 
{
    
    static double max_height;
    static double min_height;
    static vector<double> ceil_heights;

    static double w_smooth;
    static double w_safty;

    private:
        Eigen::VectorXd optimal_list;
    public:
        lanHeightOptimizer(){}
        ~lanHeightOptimizer(){}

        int heightCurveGeneration( 
        double max_height,
        double min_height, 
        vector<double> ceil_heights)
        {          
            lanHeightOptimizer::max_height = max_height;
            lanHeightOptimizer::min_height = min_height;
            lanHeightOptimizer::ceil_heights = ceil_heights;
            
            const int N = ceil_heights.size();
            
            double finalCost;
            Eigen::VectorXd x(N);
            /* Set the initial var */
            for (int i = 0; i < N; i ++)
            {
                x(i) = ceil_heights[i];
            }

            /* Set the minimization parameters */
            lbfgs::lbfgs_parameter_t params;
            params.g_epsilon = 1.0e-8;
            params.past = 3;
            params.delta = 1.0e-10;
            /* Start minimization */
            int ret = lbfgs::lbfgs_optimize(x,
                                            finalCost,
                                            costFunction,
                                            nullptr,
                                            monitorProgress,
                                            this,
                                            params);
            /* Report the result. */
            std::cout << std::setprecision(4)
                    << "================================" << std::endl
                    << "L-BFGS Optimization Returned: " << ret << std::endl;
            optimal_list = x;
            return ret;
        }

        vector<double> getHeightCps()
        {
            vector<double> opl;
            for(int i= 0 ; i <  optimal_list.rows();i++)
            {
                opl.push_back( optimal_list(i) );
            }
            return opl;
        };


    private:

    static double costFunction(void *instance,
                            const Eigen::VectorXd &x,
                            Eigen::VectorXd &g)
    {
        const int n = x.size();
        double cost = 0.0;
        double cost_smooth = 0.0;
        double cost_safty = 0.0;
        double gi_smooth = 0.0;
        double gi_safty = 0.0;
        double safe_mid = 0.6;
        double max_gap = 1.0;
        double max_height = lanHeightOptimizer::max_height;
        double min_height = lanHeightOptimizer::min_height;
        double w_smooth = lanHeightOptimizer::w_smooth;
        double w_safty = lanHeightOptimizer::w_safty;

        max_gap     = max_height - min_height;
        for (int i = 0; i < n; i ++)
        {   

            safe_mid = lanHeightOptimizer::ceil_heights[i];

            // sigmoid
            //safe_mid = (max_gap) / (1 + exp( -(1/(2*max_gap)) * (safe_mid + middle)));
            // sigmoid is not good

            safe_mid = ( pow( safe_mid - min_height , 2) * max_height )/ pow(max_gap,2);

            gi_smooth    = 0.0;
            gi_safty     = 0.0;
            cost_smooth  = 0.0;
            cost_safty   = 0.0;

            //safty
            cost_safty += pow( (x(i) - safe_mid) , 2 ); 
            gi_safty    = 2 * (x(i) - safe_mid) ; 

            //smooth ( Js = acc^2 )
            if(i < n-2)
            {
                cost_smooth += pow( (x(i+2) - 2*x(i+1) + x(i)) , 2);
            }
            if(i == 0 )
            {
                gi_smooth += 2 * x(i) - 4 * x(i+1) + 2 * x(i+2);
            }
            else if(i == 1)
            {
                gi_smooth += 10 * x(i) - 4 * x(i-1) - 8 * x(i+1) + 2 * x(i + 2);
            }
            else if(i == n - 1)
            {
                gi_smooth += 2 * x(i) - 4 * x(i-1) + 2 * x(i-2);
            }
            else if(i == n - 2)
            {
                gi_smooth += 10 * x(i) - 4 * x(i+1) - 8 * x(i-1) + 2 * x(i - 2);
            }
            else
            {
                gi_smooth += 12 * x(i) - 8 * (x(i-1) + x(i+1)) + 2 *(x(i-2) + x(i+2));
            }

            cost += w_smooth * cost_smooth + w_safty * cost_safty;
            g(i)  = w_smooth * gi_smooth + w_safty * gi_safty; 
        }
        return cost;
    }

    static int monitorProgress(void *instance,
                            const Eigen::VectorXd &x,
                            const Eigen::VectorXd &g,
                            const double fx,
                            const double step,
                            const int k,
                            const int ls)
    {
        /*
        std::cout << std::setprecision(4)
            << "================================" << std::endl
            << "Iteration: " << k << std::endl
            << "Function Value: " << fx << std::endl
            << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << std::endl
            << "Variables: " << std::endl
            << x.transpose() << std::endl;
        */
        return 0;
    }
};


#endif