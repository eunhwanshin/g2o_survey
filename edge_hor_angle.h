#pragma once

// Edge for horizontal angle measurement in g2o.
//
// E.H.Shin, 2021.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "g2o/config.h"
#include "g2o/core/base_multi_edge.h"

// 
// Since three stations are involved in one horizontal angle measurement, 
// the edge is derived from g2o::BaseMultiEdge. We assume that each vertex
// stores [East, North] coordinates. The heading between two stations 
// can be obtained by atan2(delta East, delta North).
//
class EdgeHorAngle : public g2o::BaseMultiEdge<1, number_t>
{
public:

   typedef typename Eigen::Matrix<number_t, 1, 2, Eigen::RowMajor>::AlignedMapType JacobianOplusType;

   EdgeHorAngle(void);
   
   void computeError(void);

   virtual bool read(std::istream& is);
   virtual bool write(std::ostream& os) const;
   
   virtual void setMeasurement(const number_t& m) { _measurement = m; }
   virtual bool setMeasurementData(const number_t* d) { _measurement = d[0]; return true;   }
   
   virtual bool getMeasurementData(number_t* d) const { d[0] = _measurement; return true; }
   virtual int measurementDimension() const { return 1; }
   
   virtual bool setMeasurementFromState(void);

   virtual number_t initialEstimatePossible(const g2o::OptimizableGraph::VertexSet&, g2o::OptimizableGraph::Vertex*) { return 0; }
   virtual void linearizeOplus(void);

private:

   void normalAngle(number_t& angle)
   {
      if (angle > 180.) angle -= 360.;
      else if (angle < -180.) angle += 360.;
   }
};