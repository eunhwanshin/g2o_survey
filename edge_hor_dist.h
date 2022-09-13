#pragma once

// Edge for horizontal distance measurement in g2o.
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

#include "g2o/types/slam2d/vertex_point_xy.h"

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"

class EdgeHorDist : public g2o::BaseBinaryEdge<1, number_t, g2o::VertexPointXY, g2o::VertexPointXY>
{
public:
   EdgeHorDist(void);

   void computeError(void);

   virtual bool read(std::istream& is);
   virtual bool write(std::ostream& os) const;

   virtual void setMeasurement(const number_t& m) { _measurement = m; }
   virtual bool setMeasurementData(const number_t* d) { _measurement = d[0]; return true; }

   virtual bool getMeasurementData(number_t* d) const { d[0] = _measurement; return true; }
   virtual int measurementDimension() const { return 1; }

   virtual bool setMeasurementFromState(void);

   virtual void linearizeOplus(void);
};