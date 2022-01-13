#ifndef _TRAJECTORY_SOFT_OOQP_H_
#define _TRAJECTORY_SOFT_OOQP_H_


#include "bspline_opt/lbfgs.hpp"
#include "grid_map/global_map_manager.h"
#include <Eigen/Eigen>

namespace ugv_planner
{
    class lanMiddleOptimizer 
    {
        
        static LanGridMapManager* global_map_manager;
        static Eigen::Vector3d start,end;
        static double w_smooth;
        static double w_safty;

        private:
            Eigen::VectorXd optimal_list;
            vector<Eigen::Vector3d> ictrlpts;
        public:
            lanMiddleOptimizer(){}
            ~lanMiddleOptimizer(){}

            int segments;

            int midSoftOptimize(vector<Eigen::Vector3d> init_ctrlpts, 
                                Eigen::Vector3d start,
                                Eigen::Vector3d end,
                                LanGridMapManager* gm)
            {  
               

                lanMiddleOptimizer::global_map_manager = gm;
                lanMiddleOptimizer::start = start;
                lanMiddleOptimizer::end   = end;

                ictrlpts = init_ctrlpts;
                const int N = 2 * init_ctrlpts.size();
                
                double finalCost;
                Eigen::VectorXd x(N);
                /* Set the initial var */
                for (int i = 0; i < N/2; i ++)
                {
                    x(i)         = init_ctrlpts[i](0);
                    x(i + (N/2)) = init_ctrlpts[i](1);
                }

                /* Set the minimization parameters */
                lbfgs::lbfgs_parameter_t params;
                params.g_epsilon = 1.0e-5;
                params.past = 3;
                params.delta = 1.0e-8;
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
                        << "Middle Optimization Returned: " << ret << std::endl;

                optimal_list = x;
                return ret;
            }

            vector<Eigen::Vector3d> getOptimizedCps()
            {
                int N = 2 * ictrlpts.size();
                vector<Eigen::Vector3d> opl;
                
                Eigen::Vector3d pt;
                for(int i= 0 ; i < N/2 ;i++)
                {
                    pt(0)   =   optimal_list(i);
                    pt(1)   =   optimal_list(i + (N/2));
                    pt(2)   =   ictrlpts[i](2);
                    opl.push_back(pt);
                }
                
                return opl;
            };


        private:

        static double costFunction(void *instance,
                                const Eigen::VectorXd &x,
                                Eigen::VectorXd &g)
        {
            const int N = x.size();
            double cost = 0.0;

            Eigen::Vector4d posw, posm;
            posw(3) = posm(3) = 1;
            posw(2) = posm(2) = 0;
            double posxw, posyw;      //posw
            double posx, posy;        //posm
            double index_x , index_y;

            int lines_index_x1 , lines_index_x2;
            int lines_index_y1 , lines_index_y2;

            double var_lu, var_ru;
            double var_ld, var_rd;
            double var_u, var_d;

            double g_u, g_d;
            double g_l, g_r;
            double gx,gy;

            double esdf = 0.0;
            double cost_esdf = 0.0, cost_smooth = 0.0;
            double gx_esdf, gy_esdf, gx_smooth, gy_smooth;

            double cost_boundary = 0.0 , g_boundary;

            int map_index_xmax = lanMiddleOptimizer::global_map_manager -> map_index_xmax;
            
            for(int i = 0 ; i < N/2 ; i++)
            {
               
                posxw = x(i);
                posyw = x(i + N/2);
                posw(0) = posxw ; 
                posw(1) = posyw ; 
                posm = lanMiddleOptimizer::global_map_manager -> posW2posM( posw );
                posx = posm(0);
                posy = posm(1);

                ////////////////////////
                index_x = posx / lanMiddleOptimizer::global_map_manager -> p_grid_resolution;
                if(index_x - floor(index_x) <= 0.5)
                {
                    lines_index_x1 = floor(index_x);
                    lines_index_x2 = floor(index_x) - 1;
                }
                else
                {
                    lines_index_x1 = ceil(index_x);
                    lines_index_x2 = floor(index_x);
                }
                ////////////////////////
                index_y = posy / lanMiddleOptimizer::global_map_manager -> p_grid_resolution;
                if(index_y - floor(index_y) <= 0.5)
                {
                    lines_index_y1 = floor(index_y);
                    lines_index_y2 = floor(index_y) - 1;
                }
                else
                {
                    lines_index_y1 = ceil(index_y);
                    lines_index_y2 = floor(index_y);
                }

                ////////////////////////
                var_lu = lanMiddleOptimizer::global_map_manager -> esdf_var[lanMiddleOptimizer::global_map_manager -> toAddress(lines_index_x1 , lines_index_y1)];
                var_ru = lanMiddleOptimizer::global_map_manager -> esdf_var[lanMiddleOptimizer::global_map_manager -> toAddress(lines_index_x2 , lines_index_y1)];
                var_ld = lanMiddleOptimizer::global_map_manager -> esdf_var[lanMiddleOptimizer::global_map_manager -> toAddress(lines_index_x1 , lines_index_y2)];
                var_rd = lanMiddleOptimizer::global_map_manager -> esdf_var[lanMiddleOptimizer::global_map_manager -> toAddress(lines_index_x2 , lines_index_y2)];

                var_d  = var_rd + (var_ld - var_rd) * (index_x - (lines_index_x2 + 0.5));
                var_u  = var_ru + (var_lu - var_ru) * (index_x - (lines_index_x2 + 0.5));

                esdf       = var_d + (var_u - var_d) * (index_y - (lines_index_y2 + 0.5));
                cost_esdf  -= esdf;


                g_u = (var_lu - var_ru) / lanMiddleOptimizer::global_map_manager -> p_grid_resolution;
                g_d = (var_ld - var_rd) / lanMiddleOptimizer::global_map_manager -> p_grid_resolution;
                g_l = (var_lu - var_ld) / lanMiddleOptimizer::global_map_manager -> p_grid_resolution;
                g_r = (var_ru - var_rd) / lanMiddleOptimizer::global_map_manager -> p_grid_resolution;

                //gx
                gx_esdf = -(  g_r + (g_l - g_r) * (index_x - (lines_index_x2 + 0.5))  );

                //gy
                gy_esdf = -(  g_d + (g_u - g_d) * (index_y - (lines_index_y2 + 0.5))  );

                
                //boundary
                if(i == 0 )
                {
                    cost_boundary += pow((x(i) - lanMiddleOptimizer::start(0)),2);
                    cost_boundary += pow((x(i + N/2) - lanMiddleOptimizer::start(1)),2);
                }
                if(i == N/2 - 1)
                {
                    cost_boundary += pow((x(i) - lanMiddleOptimizer::end(0)),2);
                    cost_boundary += pow((x(i + N/2) - lanMiddleOptimizer::end(1)),2);
                }

                //smooth ( Js = acc^2 )

                if(i < (N/2) -2)
                {
                    cost_smooth += pow( ( x(i+2) - 2*x(i+1) + x(i) ) , 2);
                    cost_smooth += pow( (x(i+2 + N/2) - 2*x(i+1+ N/2) + x(i+ N/2)) , 2);
                }
                
                if(i == 0 )
                {
                    gx_smooth = 2 * x(i) - 4 * x(i+1) + 2 * x(i+2);
                    gy_smooth = 2 * x(i+ N/2) - 4 * x(i+1+ N/2) + 2 * x(i+2+ N/2);
                }
                else if(i == 1)
                {
                    gx_smooth = 10 * x(i) - 4 * x(i-1) - 8 * x(i+1) + 2 * x(i + 2);
                    gy_smooth = 10 * x(i+ N/2) - 4 * x(i-1+ N/2) - 8 * x(i+1+ N/2) + 2 * x(i + 2+ N/2);
                }
                else if(i == (N/2) - 1)
                {
                    gx_smooth = 2 * x(i) - 4 * x(i-1) + 2 * x(i-2);
                    gy_smooth = 2 * x(i+ N/2) - 4 * x(i-1+ N/2) + 2 * x(i-2+ N/2);
                }
                else if(i == (N/2) - 2)
                {
                    gx_smooth = 10 * x(i) - 4 * x(i+1) - 8 * x(i-1) + 2 * x(i - 2);
                    gy_smooth = 10 * x(i+ N/2) - 4 * x(i+1+ N/2) - 8 * x(i-1+ N/2) + 2 * x(i - 2+ N/2);
                }
                else
                {
                    gx_smooth = 12 * x(i) - 8 * (x(i-1) + x(i+1)) + 2 *(x(i-2) + x(i+2));
                    gy_smooth = 12 * x(i+ N/2) - 8 * (x(i-1+ N/2) + x(i+1+ N/2)) + 2 *(x(i-2+ N/2) + x(i+2+ N/2));
                }
                
                g(i)       = lanMiddleOptimizer::w_smooth * gx_smooth + lanMiddleOptimizer::w_safty * gx_esdf;
                g(i + N/2) = lanMiddleOptimizer::w_smooth * gy_smooth + lanMiddleOptimizer::w_safty * gy_esdf;

                if(i == 0)
                {
                    g_boundary = 2 * (x(i) - lanMiddleOptimizer::start(0));
                    g(i) += 10000000 * g_boundary;
                    g_boundary = 2 * (x(i + N/2) - lanMiddleOptimizer::start(1));
                    g(i + N/2) += 10000000 * g_boundary;
                }

                if(i == N/2 - 1)
                {
                    g_boundary = 2 * (x(i) - lanMiddleOptimizer::end(0));
                    g(i) += 10000000 * g_boundary;
                    g_boundary = 2 * (x(i + N/2) - lanMiddleOptimizer::end(1));
                    g(i + N/2) += 10000000 * g_boundary;
                }
                //g(i)       = lanMiddleOptimizer::w_safty * gx_esdf;
                //g(i + (N/2)) = lanMiddleOptimizer::w_safty * gy_esdf;
                
            }

            
                cost += lanMiddleOptimizer::w_smooth * cost_smooth + lanMiddleOptimizer::w_safty * cost_esdf + 10000000 * cost_boundary;
                //cost += lanMiddleOptimizer::w_safty * cost_esdf;

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
            
            return 0;
        }
    };
}


#endif