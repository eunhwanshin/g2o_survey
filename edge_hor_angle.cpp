#ifdef _MSC_VER 
// M_PI - Visual Studio
#define _USE_MATH_DEFINES 
#endif

#include <cmath>

#include "g2o/types/slam2d/vertex_point_xy.h"

#include "edge_hor_angle.h"

//-----------------------------------------------------------------------------
EdgeHorAngle::EdgeHorAngle(void) : g2o::BaseMultiEdge<1, number_t>()
{
   resize(3);
}

//-----------------------------------------------------------------------------
bool EdgeHorAngle::read(std::istream& is)
{
   is >> _measurement >> _information(0, 0);
   return is.good();
}

//-----------------------------------------------------------------------------
bool EdgeHorAngle::write(std::ostream& os) const
{
   os << _measurement << " " << _information(0, 0);

   return os.good();
}

//-----------------------------------------------------------------------------
void EdgeHorAngle::computeError(void)
{
   // occupied station
   const g2o::VertexPointXY* vi = static_cast<g2o::VertexPointXY*>(_vertices[0]);

   // backsight station
   const g2o::VertexPointXY* vj = static_cast<g2o::VertexPointXY*>(_vertices[1]);

   // foresight station
   const g2o::VertexPointXY* vk = static_cast<g2o::VertexPointXY*>(_vertices[2]);

   g2o::Vector2 dx_j = vj->estimate() - vi->estimate();
   g2o::Vector2 dx_k = vk->estimate() - vi->estimate();

   const double r2d = 180. / M_PI;

   double ang = (atan2(dx_k(0), dx_k(1)) - atan2(dx_j(0), dx_j(1))) * r2d;
   normalAngle(ang);

   ang -= _measurement;
   normalAngle(ang);

   _error(0) = ang;
}

//-----------------------------------------------------------------------------
bool EdgeHorAngle::setMeasurementFromState(void)
{
   // occupied station
   const g2o::VertexPointXY* vi = static_cast<g2o::VertexPointXY*>(_vertices[0]);

   // backsight station
   const g2o::VertexPointXY* vj = static_cast<g2o::VertexPointXY*>(_vertices[1]);

   // foresight station
   const g2o::VertexPointXY* vk = static_cast<g2o::VertexPointXY*>(_vertices[2]);

   g2o::Vector2 dx_j = vj->estimate() - vi->estimate();
   g2o::Vector2 dx_k = vk->estimate() - vi->estimate();

   const double r2d = 180. / M_PI;

   double ang = (atan2(dx_k(0), dx_k(1)) - atan2(dx_j(0), dx_j(1))) * r2d;
   normalAngle(ang);

   _measurement = ang;

   return true;
}

//-----------------------------------------------------------------------------
void EdgeHorAngle::linearizeOplus(void)
{
   const g2o::VertexPointXY* vi = static_cast<g2o::VertexPointXY*>(_vertices[0]);
   const g2o::VertexPointXY* vj = static_cast<g2o::VertexPointXY*>(_vertices[1]);
   const g2o::VertexPointXY* vk = static_cast<g2o::VertexPointXY*>(_vertices[2]);

   g2o::Vector2 ij = vj->estimate() - vi->estimate();
   g2o::Vector2 ik = vk->estimate() - vi->estimate();

   const double r2d = 180. / M_PI;
   double inv_ij2 = r2d / ij.squaredNorm();
   double inv_ik2 = r2d / ik.squaredNorm();

   Eigen::RowVector2d v;

   v(0) = -ik(1) * inv_ik2 + ij(1) * inv_ij2;
   v(1) =  ik(0) * inv_ik2 - ij(0) * inv_ij2;
   _jacobianOplus[0] = v;

   v(0) = -ij(1) * inv_ij2;
   v(1) =  ij(0) * inv_ij2;
   _jacobianOplus[1] = v;

   v(0) = ik(1) * inv_ik2;
   v(1) = -ik(0) * inv_ik2;
   _jacobianOplus[2] = v;
}