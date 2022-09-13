#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_xy_prior.h"
#include "g2o/types/slam2d/edge_pointxy.h"

#include "edge_hor_angle.h"
#include "edge_hor_dist.h"

#include <iostream>

static int n_vertex = 0;
std::vector< g2o::EdgeXYPrior*> xy_prior;

//-----------------------------------------------------------------------------
void addPointXYPrior(
   const int v_id,
   const number_t* meas,
   Eigen::Matrix<number_t, 2, 2>& information,
   g2o::SparseOptimizer& optimizer)
{
   g2o::EdgeXYPrior* p = new g2o::EdgeXYPrior();

   p->setVertex(0, static_cast<g2o::VertexPointXY*>(optimizer.vertex(v_id)));
   p->setMeasurementData(meas);
   p->setInformation(information);

   xy_prior.push_back(p);
   optimizer.addEdge(p);
}

//-----------------------------------------------------------------------------
void addPointXY(
   const number_t* p,
   g2o::SparseOptimizer& optimizer,
   Eigen::Matrix<number_t, 2, 2>* information = nullptr)
{
   g2o::VertexPointXY* v = new g2o::VertexPointXY();

   v->setId(n_vertex);
   v->setEstimateDataImpl(p);
   v->setFixed(false);

   optimizer.addVertex(v);

   if (information)
      addPointXYPrior(n_vertex, p, *information, optimizer);

   n_vertex++;
}

//-----------------------------------------------------------------------------
void addEdgeHorAng(
   const int i_from,
   const int i_back,
   const int i_fore,
   const number_t* meas,
   Eigen::Matrix<number_t, 1, 1>& information,
   g2o::SparseOptimizer& optimizer)
{
   EdgeHorAngle* e = new EdgeHorAngle();

   e->vertices()[0] = optimizer.vertices()[i_from];
   e->vertices()[1] = optimizer.vertices()[i_back];
   e->vertices()[2] = optimizer.vertices()[i_fore];
   e->setMeasurementData(meas);
   e->setInformation(information);

   optimizer.addEdge(e);
}

//-----------------------------------------------------------------------------
void addEdgeHorDist(
   const int i_from,
   const int i_to,
   const number_t* meas,
   Eigen::Matrix<number_t, 1, 1>& information,
   g2o::SparseOptimizer& optimizer)
{
   EdgeHorDist* e = new EdgeHorDist();

   e->vertices()[0] = optimizer.vertices()[i_from];
   e->vertices()[1] = optimizer.vertices()[i_to];
   e->setMeasurementData(meas);
   e->setInformation(information);

   optimizer.addEdge(e);
}

//-----------------------------------------------------------------------------
void test_2d_survey(void)
{
   // allocating the optimizer
   g2o::SparseOptimizer optimizer;
   std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linear_solver = g2o::make_unique<
      g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();

   std::unique_ptr<g2o::BlockSolverX> block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
   auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

   optimizer.setAlgorithm(algorithm);
   optimizer.setVerbose(true);

   // Information matrix of edges
   Eigen::Matrix<number_t, 2, 2> information;
   information.setIdentity();

   number_t p1[2] = { 0, 0 };
   number_t p2[2] = { 10, 0 };
   number_t p3[2] = { 5 - 0.8938, 8.660 + 0.8664 };

   information(0, 0) = 1.0e+6;
   information(1, 1) = 1.0e+6;

   addPointXY(p1, optimizer, &information);
   addPointXY(p2, optimizer, &information);

   information(0, 0) = 1;
   information(1, 1) = 1;

   addPointXY(p3, optimizer, &information);

   // Add edge horizontal angle measurements
   double ang_meas = 60.;
   Eigen::Matrix<number_t, 1, 1> ang_info;
   ang_info(0) = 1.; // 1 deg

   addEdgeHorAng(0, 2, 1, &ang_meas, ang_info, optimizer);
   addEdgeHorAng(1, 0, 2, &ang_meas, ang_info, optimizer);

   // Add edge horizontal distance measurements

   double dist_meas = 10.;
   Eigen::Matrix<number_t, 1, 1> dist_info;

   dist_info(0) = 1.0e+6;
   addEdgeHorDist(0, 2, &dist_meas, dist_info, optimizer);
   addEdgeHorDist(1, 2, &dist_meas, dist_info, optimizer);

   optimizer.initializeOptimization();
   int result = optimizer.optimize(10);

   if (result > 0)
   {
      for (int i = 0; i < n_vertex; ++i)
      {
         g2o::SparseBlockMatrixX spinv;
         auto est = static_cast<g2o::VertexPointXY*>(optimizer.vertex(i))->estimate();

         std::cout << i << " " << est(0) << " " << est(1);

         if (optimizer.computeMarginals(spinv, optimizer.vertex(i)))
         {
            auto cov = spinv.block(i, i);

            std::cout << " " << sqrt((*cov)(0, 0)) << " " << sqrt((*cov)(1, 1));
         }
         std::cout << std::endl;

      }
   }
   else
      std::cout << "Optimization failed." << std::endl;

   // freeing the graph memory
   optimizer.clear();
}

