/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#pragma once

#include <string>
#include <math.h>

#include <chrono>

#include "polynomials/ebyshev.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"

#include "solvers/sqp_base.hpp"
#include "solvers/osqp_interface.hpp"
#include "control/mpc_wrapper.hpp"
#include "control/ocp_base.hpp"
#include "drone_model.hpp"
#include "drone_guidance_settings.hpp"

using namespace Eigen;
using namespace std;

static const int POLY_ORDER_G = 7;
static const int NUM_SEG_G = 2;

class DroneGuidanceOCP : public  polympc::OCPBase</*NX*/ 6, /*NU*/ 3, /*NP*/ 1, /*ND*/ 0, /*NG*/0, /*TYPE*/ double> {
public:
    ~DroneGuidanceOCP() = default;

    static const int NUM_NODES = POLY_ORDER_G * NUM_SEG_G + 1;

    Matrix<scalar_t, NX, 1> xs;

    Drone *drone;

    GuidanceSettings<scalar_t> settings;

    void init(Drone *drone_, GuidanceSettings<scalar_t> settings_) {
        drone = drone_;
        settings = settings_;
    }

    template<typename T>
    inline void dynamics_impl(const Ref<const state_t <T>> x,
                              const Ref<const control_t <T>> u,
                              const Ref<const parameter_t <T>> p,
                              const Ref<const static_parameter_t> &d,
                              const T &t, Ref<state_t <T>> xdot) const noexcept {
        T prop_av = u(0);
        T phi = u(1);
        T theta = u(2);

        T thrust = drone->getThrust(prop_av) * drone->props.thrust_scaling;

        Eigen::Matrix<T, 3, 1> thrust_vector;
        thrust_vector << thrust * sin(phi) * sin(theta),
                -thrust * cos(phi) * sin(theta),
                thrust * cos(theta);

        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T) 0, (T) 0, (T) -9.81;

        xdot.head(3) = x.segment(3, 3);
        xdot.segment(3, 3) = thrust_vector * (T) drone->props.dry_mass_inv + gravity;

        //minimal time
        xdot *= p(0);
    }

    template<typename T>
    EIGEN_STRONG_INLINE void
    inequality_constraints_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, Ref<constraint_t <T>> g) const noexcept {
    }

    template<typename T>
    inline void lagrange_term_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                   const Ref<const parameter_t <T>> p,
                                   const Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept {
        lagrange = p(0) * u(0) * u(0);
    }

    template<typename T>
    inline void mayer_term_impl(const Ref<const state_t <T>> x, const Ref<const control_t <T>> u,
                                const Ref<const parameter_t <T>> p, const Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept {
        mayer = settings.horizontal_slack * ((x(0) - xs(0)) * (x(0) - xs(0)) + (x(1) - xs(1)) * (x(1) - xs(1)));
    }
};