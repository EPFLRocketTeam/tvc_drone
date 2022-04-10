/* This file is part of the the TVC drone project (https://github.com/EPFLRocketTeam/tvc_drone).
 *
 * Copyright (C) 2021  Raphaël Linsen
 *
 * This Source Code Form is subject to the terms of the Mozilla
 * Public License v. 2.0. If a copy of the MPL was not distributed
 * with this file, You can obtain one at http://mozilla.org/MPL/2.0/
 */

#pragma once

using namespace Eigen;


#include "polynomials/ebyshev.hpp"
#include "control/continuous_ocp.hpp"
#include "polynomials/splines.hpp"

#include "solvers/sqp_base.hpp"
#include "solvers/osqp_interface.hpp"
#include "control/mpc_wrapper.hpp"

#include "drone_guidance_ocp.hpp"

/** create solver */
template<typename Problem, typename QPSolver>
class GuidanceSolver;

template<typename Problem, typename QPSolver = polympc::boxADMM<Problem::VAR_SIZE, Problem::NUM_EQ + Problem::NUM_INEQ,
        typename Problem::scalar_t, Problem::MATRIXFMT, polympc::linear_solver_traits<polympc::SPARSE>::default_solver>>
class GuidanceSolver : public polympc::SQPBase<GuidanceSolver<Problem, QPSolver>, Problem, QPSolver> {
public:
    using Base = polympc::SQPBase<GuidanceSolver<Problem, QPSolver>, Problem, QPSolver>;
    using typename Base::scalar_t;
    using typename Base::nlp_variable_t;
    using typename Base::nlp_hessian_t;
    using typename Base::nlp_jacobian_t;
    using typename Base::nlp_dual_t;
    using typename Base::parameter_t;
    using typename Base::nlp_constraints_t;


    /** change Hessian regularization to non-default*/
    EIGEN_STRONG_INLINE void hessian_regularisation_dense_impl(
            Eigen::Ref<nlp_hessian_t> lag_hessian) noexcept { //Three methods: adding a constant identity matrix, estimating eigen values with Greshgoring circles, Eigen Value Decomposition
        const int n = this->m_H.rows();

        /**Regularize by the estimation of the minimum negative eigen value--does not work with inexact Hessian update(matrix is already PSD)*/
        scalar_t aii, ri;
        for (int i = 0; i < n; i++) {
            aii = lag_hessian(i, i);
            ri = (lag_hessian.col(i).cwiseAbs()).sum() -
                 abs(aii); // The hessian is symmetric, Greshgorin discs from rows or columns are equal
            if (aii - ri <= 0) {
                lag_hessian(i, i) += (ri - aii) + scalar_t(0.01);
            } //All Greshgorin discs are in the positive half
        }
    }

    EIGEN_STRONG_INLINE void hessian_regularisation_sparse_impl(
            nlp_hessian_t &lag_hessian) noexcept {//Three methods: adding a constant identity matrix, estimating eigen values with Gershgorin circles, Eigen Value Decomposition
        const int n = this->m_H.rows(); //132=m_H.toDense().rows()

        /**Regularize by the estimation of the minimum negative eigen value*/
        scalar_t aii, ri;
        for (int i = 0; i < n; i++) {
            aii = lag_hessian.coeffRef(i, i);
            ri = (lag_hessian.col(i).cwiseAbs()).sum() -
                 abs(aii); // The hessian is symmetric, Greshgorin discs from rows or columns are equal
            if (aii - ri <= 0)
                lag_hessian.coeffRef(i, i) += (ri - aii) + (0.01);//All Gershgorin discs are in the positive half
        }
    }


    /** change Hessian update algorithm to the one provided by ContinuousOCP*/
    EIGEN_STRONG_INLINE void
    hessian_update_impl(Eigen::Ref<nlp_hessian_t> hessian, const Eigen::Ref<const nlp_variable_t> &x_step,
                        const Eigen::Ref<const nlp_variable_t> &grad_step) noexcept {
        this->problem.hessian_update_impl(hessian, x_step, grad_step);
    }

    /** for this problem it turned out that exact linearisation not only converges faster but also with a lower computation cost per iteration *
         *
         * So we tell the solver to use the exact linearisation here to update the Hessian
         *
         */
    EIGEN_STRONG_INLINE void
    update_linearisation_dense_impl(const Eigen::Ref<const nlp_variable_t> &x, const Eigen::Ref<const parameter_t> &p,
                                    const Eigen::Ref<const nlp_variable_t> &x_step,
                                    const Eigen::Ref<const nlp_dual_t> &lam,
                                    Eigen::Ref<nlp_variable_t> cost_grad, Eigen::Ref<nlp_hessian_t> lag_hessian,
                                    Eigen::Ref<nlp_jacobian_t> A, Eigen::Ref<nlp_constraints_t> b) noexcept {
        this->linearisation_dense_impl(x, p, lam, cost_grad, lag_hessian, A, b);
        polympc::ignore_unused_var(x_step);
    }

    EIGEN_STRONG_INLINE void
    update_linearisation_sparse_impl(const Eigen::Ref<const nlp_variable_t> &x, const Eigen::Ref<const parameter_t> &p,
                                     const Eigen::Ref<const nlp_variable_t> &x_step,
                                     const Eigen::Ref<const nlp_dual_t> &lam,
                                     Eigen::Ref<nlp_variable_t> cost_grad, nlp_hessian_t &lag_hessian,
                                     nlp_jacobian_t &A, Eigen::Ref<nlp_constraints_t> b) noexcept {
        this->linearisation_sparse_impl(x, p, lam, cost_grad, lag_hessian, A, b);
        polympc::ignore_unused_var(x_step);
    }

};