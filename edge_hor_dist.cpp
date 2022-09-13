#include "edge_hor_dist.h"

//-----------------------------------------------------------------------------
EdgeHorDist::EdgeHorDist(void): BaseBinaryEdge<1, number_t, g2o::VertexPointXY, g2o::VertexPointXY>()
{
   _information.setIdentity();
   _error.setZero();
}

//-----------------------------------------------------------------------------
bool EdgeHorDist::read(std::istream& is)
{
   is >> _measurement >> _information(0, 0);
   return is.good();
}

//-----------------------------------------------------------------------------
bool EdgeHorDist::write(std::ostream& os) const
{
   os << _measurement << " " << _information(0, 0);

   return os.good();
}

//-----------------------------------------------------------------------------
void EdgeHorDist::computeError(void)
{
   const g2o::VertexPointXY* v1 = static_cast<const g2o::VertexPointXY*>(_vertices[0]);
   const g2o::VertexPointXY* v2 = static_cast<const g2o::VertexPointXY*>(_vertices[1]);

   auto dv = v2->estimate() - v1->estimate();

   _error(0) = dv.norm() - _measurement;
}

//-----------------------------------------------------------------------------
bool EdgeHorDist::setMeasurementFromState(void)
{
   const g2o::VertexPointXY* v1 = static_cast<const g2o::VertexPointXY*>(_vertices[0]);
   const g2o::VertexPointXY* v2 = static_cast<const g2o::VertexPointXY*>(_vertices[1]);

   auto dv = v2->estimate() - v1->estimate();

   _measurement = dv.norm();

   return true;
}

//-----------------------------------------------------------------------------
void EdgeHorDist::linearizeOplus()
{
   const g2o::VertexPointXY* v1 = static_cast<const g2o::VertexPointXY*>(_vertices[0]);
   const g2o::VertexPointXY* v2 = static_cast<const g2o::VertexPointXY*>(_vertices[1]);

   auto dv = v2->estimate() - v1->estimate();

   double norm = dv.norm();

   Eigen::RowVector2d los;

   los(0) = dv(0) / norm;
   los(1) = dv(1) / norm;

   _jacobianOplusXi = -los;
   _jacobianOplusXj = los;
}
