/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_INIT_CERES_JPLQUATLOCAL_H
#define OV_INIT_CERES_JPLQUATLOCAL_H

#include <ceres/ceres.h>

namespace ov_init {

/**
 * @brief JPL quaternion CERES state parameterization
 */
class State_JPLQuatLocal : public ceres::Manifold {
  public:
    /**
     * @brief State update function (applies quaternion perturbation)
     */
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
  
    /**
     * @brief Computes the local tangent-space difference between two states
     */
    bool Minus(const double *y, const double *x, double *delta) const override;
  
    /**
     * @brief Computes the Jacobian of the Plus function
     */
    bool PlusJacobian(const double *x, double *jacobian) const override;
  
    /**
     * @brief Computes the Jacobian of the Minus function
     */
    bool ComputeJacobian(const double *x, double *jacobian) const;
    bool MinusJacobian(const double *x, double *jacobian) const override;
  
    int GlobalSize() const { return 4; };
    int LocalSize() const  { return 3; };

    int AmbientSize() const override { return 4; }
    int TangentSize() const override { return 3; }
  };
  

} // namespace ov_init

#endif // OV_INIT_CERES_JPLQUATLOCAL_H