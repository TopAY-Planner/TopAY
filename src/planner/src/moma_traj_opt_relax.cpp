#include "planner/moma_traj_opt.h"

namespace nmoma_planner 
{
    bool MomaTrajOpt::optimizeTraj(std::vector<Eigen::VectorXd> init_path, 
                                    const Eigen::MatrixXd& boundary_vel_, 
                                    const Eigen::MatrixXd& boundary_acc_)
    {
        start_state = init_path[0];
        end_state = init_path.back();
        ee_pose = moma_param.getFKPose(init_path.back());

        /* sample begin. */
        std::vector<Eigen::VectorXd> sampled_path;
        sampled_path.clear();
        Eigen::VectorXd state12d = Eigen::VectorXd::Zero(12); // x y theta delta_theta delta_arc, q
        state12d.head(3) = init_path[0].head(3);
        state12d.tail(7) = init_path[0].tail(7);
        sampled_path.push_back(state12d);
        for (size_t i = 1; i<init_path.size(); i++)
        {
            state12d.setZero();
            double arc_len = (init_path[i].head(2) - init_path[i-1].head(2)).norm();
            double now_theta = init_path[i][2];
            normalizeAngle(sampled_path.back()[2], now_theta);
            double theta_diff = now_theta - sampled_path.back()[2];
            if (fabs(theta_diff) > 1e-2)
            {
                if (arc_len < 1e-2)
                {
                    state12d.head(2) = init_path[i].head(2);
                    state12d[2] = now_theta;
                    state12d[3] = theta_diff;
                    state12d[4] = 0.0;
                    state12d.tail(7) = init_path[i].tail(7);
                    sampled_path.push_back(state12d);
                }
                else
                {
                    state12d = sampled_path.back();
                    double direct_theta = atan2(init_path[i][1] - sampled_path.back()[1], 
                                                init_path[i][0] - sampled_path.back()[0]);
                    normalizeAngle(sampled_path.back()[2], direct_theta);
                    theta_diff = direct_theta - sampled_path.back()[2];
                    state12d[2] = direct_theta;
                    state12d[3] = theta_diff;
                    state12d[4] = 0.0;
                    sampled_path.push_back(state12d);

                    state12d.head(2) = init_path[i].head(2);
                    state12d[2] = direct_theta;
                    state12d[3] = 0.0;
                    state12d[4] = arc_len;
                    state12d.tail(7) = init_path[i].tail(7);
                    sampled_path.push_back(state12d);

                    normalizeAngle(sampled_path.back()[2], now_theta);
                    theta_diff = now_theta - sampled_path.back()[2];
                    state12d[2] = now_theta;
                    state12d[3] = theta_diff;
                    state12d[4] = 0.0;
                    sampled_path.push_back(state12d);
                }
            }
            else
            {
                if (arc_len > 1e-2)
                {
                    state12d.head(2) = init_path[i].head(2);
                    state12d[2] = now_theta;
                    state12d[3] = 0.0;
                    state12d[4] = arc_len;
                    state12d.tail(7) = init_path[i].tail(7);
                    sampled_path.push_back(state12d);
                }
            }
        }

#ifdef PUB_DEBUG
        PRINT_YELLOW("[Moma Traj Opt] sampled_path:");
        for (size_t i=0; i<sampled_path.size(); i++)
            PRINT_YELLOW(sampled_path[i].transpose());
#endif

        /* sample done. */

        /* init opt problem. */
        std::vector<double> path_arcs;
        std::vector<double> weighted_path_arcs;
        double total_len = 0;
        double weighted_total_len = 0;

        size_t path_num = sampled_path.size();
        path_arcs.push_back(0);
        weighted_path_arcs.push_back(0);
        for(size_t idx = 1; idx<path_num; idx++)
        {
            Eigen::VectorXd path_node = sampled_path[idx];
            total_len += path_node[4];
            path_arcs.push_back(total_len); 
            weighted_total_len += 0.2 * abs(path_node[3]) + 1.4 * abs(path_node[4]);
            weighted_path_arcs.push_back(weighted_total_len);
        }
        double total_time = getDurationTrapezoid(weighted_total_len, 
                                                 boundary_vel_(0, 0), 0.0, 
                                                 moma_param.max_v, moma_param.max_a);

        std::vector<Eigen::VectorXd> vector_inner_pts; // 储存分段采样后的坐标点 yaw,s,q
        double sample_interval = total_time / std::max(int(total_time / opt_param.sample_interval + 0.5), opt_param.min_piece_num);

        int now_idx = 1;
        init_inner_xy.clear();
        for(double t = sample_interval; t<total_time-1e-3; t+=sample_interval)
        {
            double arc = getArcTrapezoid(t, weighted_total_len, 
                                         boundary_vel_(0, 0), 0.0, 
                                         moma_param.max_v, moma_param.max_a);
            for (size_t k = now_idx; k<path_num; k++)
            {
                Eigen::VectorXd path_node = sampled_path[k];
                Eigen::VectorXd pre_path_node = sampled_path[k-1];
                double tmp_arc = weighted_path_arcs[k];
                if(tmp_arc >= arc)
                {
                    now_idx = k; 
                    double l1 = tmp_arc-arc;
                    double l = weighted_path_arcs[k] - weighted_path_arcs[k-1];
                    Eigen::VectorXd pts = Eigen::VectorXd::Zero(9);
                    pts(0) = pre_path_node[2] + (l-l1)/l*(path_node[3]);
                    pts(1) = path_arcs[k-1] + (l-l1)/l*(path_node[4]);
                    pts.tail(7) = pre_path_node.tail(7) + (l-l1)/l*(path_node.tail(7) - pre_path_node.tail(7));
                    vector_inner_pts.push_back(pts);

                    double interp_x = l1/l*pre_path_node[0] + (l-l1)/l*(path_node[0]);
                    double interp_y = l1/l*pre_path_node[1] + (l-l1)/l*(path_node[1]);
                    init_inner_xy.push_back(Eigen::Vector2d(interp_x, interp_y));
                    break;
                }
            }
        }
        init_inner_xy.push_back(init_path.back().head(2));

        //! NOTE: now tail va = 0
        // start pva
        minco_start_state = Eigen::MatrixXd::Zero(9, 3);
        minco_start_state(0, 0) = sampled_path[0][2];
        minco_start_state(0, 1) = boundary_vel_(1, 0);
        minco_start_state(0, 2) = boundary_acc_(1, 0);
        minco_start_state(1, 1) = boundary_vel_(0, 0);
        minco_start_state(1, 2) = boundary_acc_(0, 0);
        minco_start_state.col(0).tail(7) = sampled_path[0].tail(7);
        minco_start_state.col(1).tail(7) = boundary_vel_.col(0).tail(7);
        minco_start_state.col(2).tail(7) = boundary_acc_.col(0).tail(7);

        // end pva
        minco_end_state = Eigen::MatrixXd::Zero(9, 3);
        minco_end_state(0, 0) = sampled_path.back()[2];
        minco_end_state(1, 0) = path_arcs.back();
        minco_end_state.col(0).tail(7) = sampled_path.back().tail(7);
        minco_end_state.col(1).tail(7) = boundary_vel_.col(1).tail(7);
        minco_end_state.col(2).tail(7) = boundary_acc_.col(1).tail(7);

        // GO!
        piece_num = vector_inner_pts.size() + 1;
        times = Eigen::VectorXd::Constant(piece_num, sample_interval);
        inner_pts = Eigen::MatrixXd::Zero(9, piece_num-1);
        for (size_t i = 0; i < vector_inner_pts.size(); i++)
            inner_pts.col(i) = vector_inner_pts[i];

        /* init opt problem done. */
#ifdef PUB_DEBUG
        PRINT_YELLOW("inner_pts: \n" << inner_pts.transpose());
        PRINT_YELLOW("times: \n" << times.transpose());
        PRINT_YELLOW("minco start_state: \n" << minco_start_state);
        PRINT_YELLOW("minco end_state: \n" << minco_end_state);
        PRINT_YELLOW("start state:\n"<<start_state.transpose());
        PRINT_YELLOW("end state:\n"<<end_state.transpose());
        PRINT_YELLOW("init_inner_xy:");
        for (size_t i=0; i<init_inner_xy.size(); i++)
            PRINT_YELLOW(init_inner_xy[i].transpose());
#endif
        /* init opt problem done. */

        minco_opt.reset(piece_num, opt_param.energy_weights);
        minco_opt.generate(minco_start_state, minco_end_state, inner_pts, times);

        // init optimization variables
        int variable_num = 10 * piece_num;
        Eigen::VectorXd x;
        x.resize(variable_num);
        int opt_var_idx = 0;
        Eigen::Map<Eigen::VectorXd> Tau(x.data()+opt_var_idx, piece_num);
        opt_var_idx += piece_num;
        Eigen::Map<Eigen::VectorXd> Theta(x.data()+opt_var_idx, piece_num);
        opt_var_idx += piece_num;
        Eigen::Map<Eigen::VectorXd> Arc(x.data()+opt_var_idx, piece_num);
        opt_var_idx += piece_num;
        Eigen::Map<Eigen::MatrixXd> Vq(x.data()+opt_var_idx, moma_param.dof_num, piece_num);
        for (int i = 0; i < piece_num - 1; i++)
        {
            Tau(i) = logC2(times(i));
            Theta(i) = inner_pts(0, i);
            Arc(i) = inner_pts(1, i);
            for (size_t j = 0; j < moma_param.dof_num; j++)
                Vq(j, i) = invSigmoidC2(inner_pts(j+2, i), moma_param.joint_pos_limit_max(j));
        }
        Tau[piece_num-1] = logC2(times(piece_num-1));
        Theta[piece_num-1] = minco_end_state(0, 0);
        Arc[piece_num-1] = minco_end_state(1, 0);
        for (size_t j = 0; j < moma_param.dof_num; j++)
                Vq(j, piece_num - 1) = invSigmoidC2(minco_end_state(j+2, 0), moma_param.joint_pos_limit_max(j));
        
#ifdef PUB_DEBUG
        init_traj = getTraj();
        pubDebugTraj(init_traj);
        PRINT_GREEN("[Moma Opt] Before Optimization:");
        printConstraintsSituations(init_traj);
#endif

        // deal with too shot traj
        if (fabs(minco_end_state(1, 0)) < opt_param.first_stage.shot_path_horizon)
            opt_param.first_stage.lbfgs_param.past = opt_param.first_stage.lbgfs_shot_path_past;
        else
            opt_param.first_stage.lbfgs_param.past = opt_param.first_stage.lbgfs_normal_past;

        // optimize first stage
        double cost;
        ros::Time start_time = ros::Time::now();
        int result = lbfgs::lbfgs_optimize(x, cost,
                                            MomaTrajOpt::firstStageCostCallback,
                                            nullptr, nullptr, this,
                                            opt_param.first_stage.lbfgs_param);
        if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
            result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGSERR_MAXIMUMITERATION)
            // ;
            PRINT_GREEN("[Moma Opt] First stage finished in " << (ros::Time::now() - start_time).toSec() * 1000.0 << " ms with result: " << result << ", cost: " << cost);
        else
        {
            PRINT_RED("[Moma Opt] First stage failed. finished in " << (ros::Time::now() - start_time).toSec() * 1000.0 << " ms with result: " << result);
            return false;
        }

        // PRINT_GREEN("After first stage: ");
        // PRINT_YELLOW("inner_pts: \n" << inner_pts.topRows(2).transpose());
        // PRINT_YELLOW("times: \n" << times.transpose());
        // PRINT_YELLOW("minco start_state: \n" << minco_start_state.topRows(2));
        // PRINT_YELLOW("minco end_state: \n" << minco_end_state.topRows(2));
        // PRINT_YELLOW("start se2:\n"<<start_state.head(3).transpose());
        // PRINT_YELLOW("end se2:\n"<<end_state.head(3).transpose());

#ifdef PUB_DEBUG
        afirst_traj = getTraj();
        pubDebugTraj(afirst_traj);
        PRINT_GREEN("[Moma Opt] Before Second Optimization:");
        printConstraintsSituations(afirst_traj);
#endif

        // test
        // return true;

        // optimize second stage
        alm_lambda = opt_param.second_stage.alm_param.init_lambda;
        alm_rho = opt_param.second_stage.alm_param.init_rho;
        int iter_num = 0;
        bool success = false;
        start_time = ros::Time::now();
        while (ros::ok())
        {
            boost::this_thread::interruption_point();
            // if ((ros::Time::now() - start_time).toSec() > 1.0)
            // {
            //     PRINT_RED("[ALM Inner] Solver time out.");
            //     break;
            // }

            iter_num++;
            result = lbfgs::lbfgs_optimize(x, cost,
                                            MomaTrajOpt::secondStageCostCallback,
                                            nullptr, &earlyExit, this,
                                            opt_param.second_stage.lbfgs_param);
#ifdef PUB_DEBUG
            debugm = true;
#endif
            // debug_manager.reset();
            // Eigen::VectorXd grad_no;
            // grad_no.resize(x.size());
            // secondStageCostCallback(this, x, grad_no);
            // debugm = false;
            // debug_manager.printinfo();

#ifdef PUB_DEBUG
            pubDebugTraj(getTraj());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif
            if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
                result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION)
                PRINTF_WHITE("[ALM Inner] optimizer finish! result: "<<result<<", finalcost: "<<cost<<", iter_num: "<<iter_num<<"\n");
            else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
                PRINT_YELLOW("[ALM Inner] Lbfgs: The line-search routine reaches the maximum number of evaluations.");
            else
            {
                PRINT_RED("[ALM Inner] Solver error. Return = "<<result<<", "<<lbfgs::lbfgs_strerror(result)<<".");
                success = false;
                break;
            }

            // update ALM datas
            PRINTF_WHITE("[ALM Inner] alm_lambda: " << alm_lambda.transpose() << 
                         "\n[ALM Inner] alm_rho: " << alm_rho.transpose() << 
                         "\n[ALM Inner] Error.norm():" << final_pose_error.norm()<<"\n");

            if (final_pose_error.norm() < opt_param.second_stage.alm_param.tolerance[0])
            // if(final_xy_error.norm() < opt_param.second_stage.alm_param.tolerance[0])
            {
                success = true;
                break;
            }
            
            // update alm variables
            for (int i = 0; i < alm_rho.size(); i++)
            {
                alm_lambda[i] += alm_rho[i] * final_pose_error(i);
                alm_rho[i] = std::min((1 + opt_param.second_stage.alm_param.gamma[i]) * alm_rho[i], opt_param.second_stage.alm_param.rho_max[i]);
            }
            // alm_lambda[0] += alm_rho[0] * final_xy_error.x();
            // alm_lambda[1] += alm_rho[1] * final_xy_error.y();
            // alm_rho[0] = std::min((1 + opt_param.second_stage.alm_param.gamma[0]) * alm_rho[0], opt_param.second_stage.alm_param.rho_max[0]);
            // alm_rho[1] = std::min((1 + opt_param.second_stage.alm_param.gamma[1]) * alm_rho[1], opt_param.second_stage.alm_param.rho_max[1]);
        }

        if (success)
            PRINT_GREEN("[Moma Opt] Second stage done. finished in " << (ros::Time::now() - start_time).toSec() * 1000.0 << " ms.");
        else
            PRINT_RED("[Moma Opt] Second stage failed. finished in " << (ros::Time::now() - start_time).toSec() * 1000.0 << " ms.");
                
#ifdef PUB_DEBUG
        if (success)
        {
            int opt_var_idx = 0;
            Eigen::Map<const Eigen::VectorXd> Tau(x.data()+opt_var_idx, piece_num);
            opt_var_idx += piece_num;
            Eigen::Map<const Eigen::VectorXd> Theta(x.data()+opt_var_idx, piece_num-1);
            opt_var_idx += piece_num - 1;
            Eigen::Map<const Eigen::VectorXd> Arc(x.data()+opt_var_idx, piece_num);
            opt_var_idx += piece_num;
            Eigen::Map<const Eigen::MatrixXd> Q(x.data()+opt_var_idx, moma_param.dof_num, piece_num-1);
            PRINT_GREEN("After Optimization:");

            printConstraintsSituations(getTraj());
            
            for (int i = 0; i < piece_num - 1; i++)
            {
                PRINTF_YELLOW(Theta(i)<<" ");
            }
            PRINTF_YELLOW("\n");
            for (int i = 0; i < piece_num - 1; i++)
            {
                PRINTF_YELLOW(Arc(i)<<" ");
            }
            PRINTF_YELLOW("\n\n");
        }
#endif

        traj_cost = cost;

        return success;
    }

    double MomaTrajOpt::firstStageCostCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
    {
        MomaTrajOpt& obj = *(MomaTrajOpt*)(ptrObj);

        int opt_var_idx = 0;
        Eigen::Map<const Eigen::VectorXd> Tau(x.data()+opt_var_idx, obj.piece_num);
        Eigen::Map<Eigen::VectorXd> gradTau(grad.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::VectorXd> Theta(x.data()+opt_var_idx, obj.piece_num);
        Eigen::Map<Eigen::VectorXd> gradTheta(grad.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::VectorXd> Arc(x.data()+opt_var_idx, obj.piece_num);
        Eigen::Map<Eigen::VectorXd> gradArc(grad.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::MatrixXd> Vq(x.data()+opt_var_idx, obj.moma_param.dof_num, obj.piece_num);
        Eigen::Map<Eigen::MatrixXd> gradVq(grad.data()+opt_var_idx, obj.moma_param.dof_num, obj.piece_num);

        obj.calTfromTau(Tau, obj.times);
        obj.minco_end_state(0, 0) = Theta[obj.piece_num-1];
        obj.minco_end_state(1, 0) = Arc[obj.piece_num-1];
        obj.inner_pts.resize(9, obj.piece_num-1);
        obj.inner_pts.row(0) = Theta.head(obj.piece_num-1);
        obj.inner_pts.row(1) = Arc.head(obj.piece_num-1);
        for (int i = 0; i < obj.piece_num-1; i++)
        {
            for (size_t j = 0; j < obj.moma_param.dof_num; j++)
                obj.inner_pts(j+2, i) = obj.sigmoidC2(Vq(j, i), obj.moma_param.joint_pos_limit_max(j));
        }
        for (int i = 0; i < obj.moma_param.dof_num; i++)
            obj.minco_end_state(i+2, 0) = obj.sigmoidC2(Vq(i, obj.piece_num-1), obj.moma_param.joint_pos_limit_max(i));
        obj.minco_opt.generate(obj.minco_start_state, obj.minco_end_state, obj.inner_pts, obj.times);

        // get jerk cost with grad (C,T)
        double jerk_cost = 0.0;
        Eigen::MatrixXd gdC_jerk;
        Eigen::VectorXd gdT_jerk;
        obj.minco_opt.calJerkGradCT(gdC_jerk, gdT_jerk);
        jerk_cost = obj.minco_opt.getTrajJerkCost();

        // get penalty cost with grad (C,T)
        double penalty_cost = 0.0;
        Eigen::MatrixXd gdC_penalty;
        Eigen::VectorXd gdT_penalty;
        obj.calFirstStagePenalGrad(penalty_cost, gdC_penalty, gdT_penalty);

        // get grad (q, T) from (C, T)
        Eigen::MatrixXd gdC = gdC_jerk + gdC_penalty;
        Eigen::VectorXd gdT = gdT_jerk + gdT_penalty;
        Eigen::MatrixXd gdP, gdP_tail;
        obj.minco_opt.calGradCTtoQT(gdC, gdT, gdP, gdP_tail);

        // time cost
        double time_cost = obj.opt_param.first_stage.time_weight * obj.times.sum();
        
        // update grad
        gradTheta.head(obj.piece_num-1) = gdP.row(0);
        gradArc.head(obj.piece_num-1) = gdP.row(1);
        Eigen::MatrixXd gradQ = gdP.bottomRows(7);
        for (int i = 0; i < obj.piece_num-1; i++)
            for (size_t j = 0; j < obj.moma_param.dof_num; j++)
                gradVq(j, i) = gradQ(j, i) * obj.getQtoVqGrad(Vq(j, i), obj.moma_param.joint_pos_limit_max(j));

        for (int i = 0; i < obj.piece_num; i++)
            gradTau(i) = (gdT(i) + obj.opt_param.first_stage.time_weight) * obj.getTtoTauGrad(Tau(i));

        gradTheta[obj.piece_num-1] = gdP_tail(0, 0);
        gradArc[obj.piece_num-1] = gdP_tail(1, 0);
        for (size_t j = 0; j < obj.moma_param.dof_num; j++)
            gradVq(j, obj.piece_num-1) = gdP_tail(j+2, 0) * obj.getQtoVqGrad(Vq(j, obj.piece_num-1), obj.moma_param.joint_pos_limit_max(j));

        // PRINT_GREEN("jerk_cost: " << jerk_cost << ", penalty_cost: " << penalty_cost << ", time_cost: " << time_cost);

        return jerk_cost + penalty_cost + time_cost;
    }

    double MomaTrajOpt::secondStageCostCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
    {
        boost::this_thread::interruption_point();
        MomaTrajOpt& obj = *(MomaTrajOpt*)(ptrObj);
        obj.debug_manager.reset();

        int opt_var_idx = 0;
        Eigen::Map<const Eigen::VectorXd> Tau(x.data()+opt_var_idx, obj.piece_num);
        Eigen::Map<Eigen::VectorXd> gradTau(grad.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::VectorXd> Theta(x.data()+opt_var_idx, obj.piece_num);
        Eigen::Map<Eigen::VectorXd> gradTheta(grad.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::VectorXd> Arc(x.data()+opt_var_idx, obj.piece_num);
        Eigen::Map<Eigen::VectorXd> gradArc(grad.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::MatrixXd> Vq(x.data()+opt_var_idx, obj.moma_param.dof_num, obj.piece_num);
        Eigen::Map<Eigen::MatrixXd> gradVq(grad.data()+opt_var_idx, obj.moma_param.dof_num, obj.piece_num);

        obj.calTfromTau(Tau, obj.times);
        obj.minco_end_state(0, 0) = Theta[obj.piece_num-1];
        obj.minco_end_state(1, 0) = Arc[obj.piece_num-1];
        obj.inner_pts.resize(9, obj.piece_num-1);
        obj.inner_pts.row(0) = Theta.head(obj.piece_num-1);
        obj.inner_pts.row(1) = Arc.head(obj.piece_num-1);
        for (int i = 0; i < obj.piece_num-1; i++)
        {
            for (size_t j = 0; j < obj.moma_param.dof_num; j++)
                obj.inner_pts(j+2, i) = obj.sigmoidC2(Vq(j, i), obj.moma_param.joint_pos_limit_max(j));
        }
        for (int i = 0; i < obj.moma_param.dof_num; i++)
            obj.minco_end_state(i+2, 0) = obj.sigmoidC2(Vq(i, obj.piece_num-1), obj.moma_param.joint_pos_limit_max(i));
        obj.minco_opt.generate(obj.minco_start_state, obj.minco_end_state, obj.inner_pts, obj.times);

        // get jerk cost with grad (C,T)
        double jerk_cost = 0.0;
        Eigen::MatrixXd gdC_jerk;
        Eigen::VectorXd gdT_jerk;
        obj.minco_opt.calJerkGradCT(gdC_jerk, gdT_jerk);
        jerk_cost = obj.minco_opt.getTrajJerkCost();

        // get penalty cost with grad (C,T)
        double penalty_cost = 0.0;
        Eigen::MatrixXd gdC_penalty;
        Eigen::VectorXd gdT_penalty;
        obj.calSecondStagePenalGrad(penalty_cost, gdC_penalty, gdT_penalty);

        // get grad (q, T) from (C, T)
        Eigen::MatrixXd gdC = gdC_jerk + gdC_penalty;
        Eigen::VectorXd gdT = gdT_jerk + gdT_penalty;
        Eigen::MatrixXd gdP, gdP_tail;
        obj.minco_opt.calGradCTtoQT(gdC, gdT, gdP, gdP_tail);

        // time cost
        double time_cost = obj.opt_param.second_stage.time_weight * obj.times.sum();
        
        // update grad
        gradTheta.head(obj.piece_num-1) = gdP.row(0);
        gradArc.head(obj.piece_num-1) = gdP.row(1);
        Eigen::MatrixXd gradQ = gdP.bottomRows(7);
        for (int i = 0; i < obj.piece_num-1; i++)
            for (size_t j = 0; j < obj.moma_param.dof_num; j++)
                gradVq(j, i) = gradQ(j, i) * obj.getQtoVqGrad(Vq(j, i), obj.moma_param.joint_pos_limit_max(j));

        for (int i = 0; i < obj.piece_num; i++)
            gradTau(i) = (gdT(i) + obj.opt_param.second_stage.time_weight) * obj.getTtoTauGrad(Tau(i));
            
        gradTheta[obj.piece_num-1] = gdP_tail(0, 0);
        gradArc[obj.piece_num-1] = gdP_tail(1, 0);
        for (size_t j = 0; j < obj.moma_param.dof_num; j++)
            gradVq(j, obj.piece_num-1) = gdP_tail(j+2, 0) * obj.getQtoVqGrad(Vq(j, obj.piece_num-1), obj.moma_param.joint_pos_limit_max(j));

        // PRINT_GREEN("jerk_cost: " << jerk_cost << ", penalty_cost: " << penalty_cost << ", time_cost: " << time_cost);
        obj.debug_manager["jerk"] += jerk_cost;
        obj.debug_manager["time"] += time_cost;

        return jerk_cost + penalty_cost + time_cost;
    }

    void MomaTrajOpt::calFirstStagePenalGrad(double& cost, Eigen::MatrixXd& gdC, Eigen::VectorXd &gdT)
    {
        cost = 0.0;
        gdC.resize(6*piece_num, 9);
        gdC.setZero();
        gdT.resize(piece_num);
        gdT.setZero();

        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
        Eigen::VectorXd state, dstate, d2state, d3state;
        double alpha, omg;
        int inner_num = 2 * opt_param.int_K;
        int int_6K = opt_param.int_K * 6;

        double violaPos;
        double violaMom;
        double violaMomPena;
        double violaMomPenaD;

        Eigen::VectorXd IntegralChainCoeff(inner_num + 1);
        IntegralChainCoeff.setZero();
        for(int i=0; i<opt_param.int_K; i++)
            IntegralChainCoeff.segment(2*i, 3) += Eigen::Vector3d(1.0, 4.0, 1.0);

        std::vector<Eigen::MatrixXd> VecSingleXGradCTheta(piece_num);
        std::vector<Eigen::MatrixXd> VecSingleXGradCArc(piece_num);
        std::vector<Eigen::VectorXd> VecSingleXGradT(piece_num);
        std::vector<Eigen::MatrixXd> VecSingleYGradCTheta(piece_num);
        std::vector<Eigen::MatrixXd> VecSingleYGradCArc(piece_num);
        std::vector<Eigen::VectorXd> VecSingleYGradT(piece_num);

        Eigen::Vector2d CurrentXY = start_state.head(2);
        Eigen::VectorXd VecCoeffChainX(piece_num*(inner_num+1)); VecCoeffChainX.setZero();
        Eigen::VectorXd VecCoeffChainY(piece_num*(inner_num+1)); VecCoeffChainY.setZero();
        std::vector<Eigen::Vector2d> VecTrajFinalXY(piece_num+1);
        VecTrajFinalXY[0] = CurrentXY;

        double cost_path=0, cost_moment=0;

        for (int i = 0; i < piece_num; i++)
        {
            const Eigen::Matrix<double, 6, 9> &c = minco_opt.getCoeffs().block<6, 9>(6*i, 0);
            double step = times(i) / opt_param.int_K;
            double half_step = step / 2.0;
            double coeff = step / 6.0;

            Eigen::MatrixXd SingleXGradCTheta(6, inner_num+1);
            Eigen::MatrixXd SingleXGradCArc(6, inner_num+1);
            Eigen::VectorXd SingleXGradT(inner_num+1);
            Eigen::MatrixXd SingleYGradCTheta(6, inner_num+1);
            Eigen::MatrixXd SingleYGradCArc(6, inner_num+1);
            Eigen::VectorXd SingleYGradT(inner_num+1);
            Eigen::VectorXd IntegralX(opt_param.int_K); IntegralX.setZero();
            Eigen::VectorXd IntegralY(opt_param.int_K); IntegralY.setZero();
            
            s1 = 0.0;
            for (int j=0; j<=inner_num; j++)
            {
                if (j % 2 == 0)
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s3 * s2;
                    beta0 << 1.0, s1, s2, s3, s4, s5;
                    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                    s1 += half_step;
                    alpha = 1.0 / inner_num * j;
                    omg = (j==0||j==inner_num)?0.5:1.0;

                    state = c.transpose() * beta0;
                    dstate = c.transpose() * beta1;
                    d2state = c.transpose() * beta2;
                    d3state = c.transpose() * beta3;
                    double syaw = sin(state(0));
                    double cyaw = cos(state(0));

                    if(j!=0)
                    {
                        IntegralX[j/2-1] += coeff * dstate(1) * cyaw;
                        IntegralY[j/2-1] += coeff * dstate(1) * syaw;
                    }
                    if(j!=inner_num)
                    {
                        IntegralX[j/2] += coeff * dstate(1) * cyaw;
                        IntegralY[j/2] += coeff * dstate(1) * syaw;
                    }

                    SingleXGradCTheta.col(j) = -dstate(1) * beta0 * syaw;
                    SingleXGradCArc.col(j) = beta1 * cyaw;
                    SingleXGradT[j] = (d2state(1) * cyaw - dstate(1) * dstate(0) * syaw)
                                      * alpha * coeff + dstate(1) * cyaw / int_6K;
                    SingleYGradCTheta.col(j) = dstate(1) * beta0 * cyaw;
                    SingleYGradCArc.col(j) = beta1 * syaw;
                    SingleYGradT[j] = (d2state(1) * syaw + dstate(1) * dstate(0) * cyaw)
                                      * alpha * coeff + dstate(1) * syaw / int_6K;

                    if (j != 0) CurrentXY += Eigen::Vector2d(IntegralX[j/2-1],IntegralY[j/2-1]);

                    double gradViolaMt;
                    double real_alpha = 1.0 / opt_param.int_K * ( (double) j / 2.0 ); 
                    Eigen::MatrixXd gradBeta; gradBeta.resize(3, 2); gradBeta.setZero();
                    for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2)
                    {
                        violaMom = omg_sym * moma_param.max_v * dstate.x() 
                                          + moma_param.max_w * dstate.y() 
                                          - moma_param.max_v * moma_param.max_w;
                        if(violaMom > 0)
                        {
                            smoothL1Penalty(violaMom, violaMomPena, violaMomPenaD);
                            gradViolaMt = real_alpha * (omg_sym * moma_param.max_v * d2state.x() + moma_param.max_w * d2state.y());
                            gradBeta(1, 0) += omg * step * opt_param.first_stage.moment_weight * violaMomPenaD * omg_sym * moma_param.max_v;
                            gradBeta(1, 1) += omg * step * opt_param.first_stage.moment_weight * violaMomPenaD * moma_param.max_w;
                            gdT(i) += omg * opt_param.first_stage.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / opt_param.int_K);
                            cost += omg * step * opt_param.first_stage.moment_weight * violaMomPena;
                            cost_moment += omg * step * opt_param.first_stage.moment_weight * violaMomPena;
                        }
                    }
                    for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2)
                    {
                        violaMom = omg_sym * moma_param.max_v * dstate.x() 
                                   - moma_param.max_w * dstate.y() 
                                   - moma_param.max_v * moma_param.max_w;
                        if(violaMom > 0)
                        {
                            smoothL1Penalty(violaMom, violaMomPena, violaMomPenaD);
                            gradViolaMt = real_alpha * (omg_sym * moma_param.max_v * d2state.x() - moma_param.max_w * d2state.y());
                            gradBeta(1, 0) += omg * step * opt_param.first_stage.moment_weight * violaMomPenaD * omg_sym * moma_param.max_v;
                            gradBeta(1, 1) -= omg * step * opt_param.first_stage.moment_weight * violaMomPenaD * moma_param.max_w;
                            gdT(i) += omg * opt_param.first_stage.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / opt_param.int_K);
                            cost += omg * step * opt_param.first_stage.moment_weight * violaMomPena;
                            cost_moment += omg * step * opt_param.first_stage.moment_weight * violaMomPena;
                        }
                    }

                    double violaAcc = d2state.y()*d2state.y() - moma_param.max_a*moma_param.max_a;
                    double violaAlp = d2state.x()*d2state.x() - moma_param.max_dw*moma_param.max_dw;
                    double violaAccPena, violaAccPenaD, violaAlpPena, violaAlpPenaD;
                    if(violaAcc > 0)
                    {
                        smoothL1Penalty(violaAcc, violaAccPena, violaAccPenaD);
                        double gradViolaAT = 2.0 * real_alpha * d2state.y() * d3state.y();
                        gradBeta(2, 1) +=  omg * step * opt_param.first_stage.acc_weight * violaAccPenaD * 2.0 * d2state.y();
                        gdT(i) += omg * opt_param.first_stage.acc_weight * (violaAccPenaD * gradViolaAT * step + violaAccPena / opt_param.int_K);
                        cost += omg * step * opt_param.first_stage.acc_weight * violaAccPena;
                        cost_moment += omg * step * opt_param.first_stage.acc_weight * violaAccPena;
                    }
                    if(violaAlp > 0)
                    {
                        smoothL1Penalty(violaAlp, violaAlpPena, violaAlpPenaD);
                        double gradViolaDOT = 2.0 * real_alpha * d2state.x() * d3state.x();
                        gradBeta(2, 0) += omg * step * opt_param.first_stage.domega_weight * violaAlpPenaD * 2.0 * d2state.x();
                        gdT(i) += omg * opt_param.first_stage.domega_weight * (violaAlpPenaD * gradViolaDOT * step + violaAlpPena / opt_param.int_K);
                        cost += omg * step * opt_param.first_stage.domega_weight * violaAlpPena;
                        cost_moment += omg * step * opt_param.first_stage.domega_weight * violaAlpPena;
                    }
                    
                    if (false)
                    {
                        double violaVelPP = - dstate.y();
                        double violaVelPena, violaVelPenaD;
                        if(violaVelPP > 0)
                        {
                            smoothL1Penalty(violaVelPP, violaVelPena, violaVelPenaD);
                            double gradViolaDOT = -real_alpha * d2state.y();
                            gradBeta(1, 1) -= omg * step * opt_param.first_stage.domega_weight * violaVelPenaD;
                            gdT(i) += omg * opt_param.first_stage.domega_weight * (violaVelPenaD * gradViolaDOT * step + violaVelPena / opt_param.int_K);
                            cost += omg * step * opt_param.first_stage.domega_weight * violaVelPena;
                            cost_moment += omg * step * opt_param.first_stage.domega_weight * violaVelPena;
                        }
                    }
                    
                    gdC.block<6, 2>(i*6, 0) += beta0 * gradBeta.row(0) 
                                               + beta1 * gradBeta.row(1) 
                                               + beta2 * gradBeta.row(2);
                }
                else
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s3 * s2;
                    beta0 << 1.0, s1, s2, s3, s4, s5;
                    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                    s1 += half_step;
                    alpha = 1.0 / inner_num * j;
                    state = c.transpose() * beta0;
                    dstate = c.transpose() * beta1;
                    d2state = c.transpose() * beta2;

                    double cyaw = cos(state.x()), syaw = sin(state.x());
                    IntegralX[j/2] += 4 * coeff * dstate.y() * cyaw;
                    IntegralY[j/2] += 4 * coeff * dstate.y() * syaw;
                    
                    SingleXGradCArc.col(j) = beta1 * cyaw;
                    SingleXGradCTheta.col(j) = -dstate.y() * beta0 * syaw;
                    SingleXGradT[j] = (d2state.y() * cyaw - dstate.y() * dstate.x() * syaw)*alpha*coeff + dstate.y() * cyaw / int_6K;

                    SingleYGradCArc.col(j) = beta1 * syaw;
                    SingleYGradCTheta.col(j) = dstate.y() * beta0 * cyaw;
                    SingleYGradT[j] = (d2state.y() * syaw + dstate.y() * dstate.x() * cyaw)*alpha*coeff + dstate.y() * syaw / int_6K;
                }
            }

            VecSingleXGradCArc[i] = SingleXGradCArc * coeff;
            VecSingleXGradCTheta[i] = SingleXGradCTheta * coeff;
            VecSingleXGradT[i] = SingleXGradT;
            VecSingleYGradCArc[i] = SingleYGradCArc * coeff;
            VecSingleYGradCTheta[i] = SingleYGradCTheta * coeff;
            VecSingleYGradT[i] = SingleYGradT;

            // path point penalty
            VecTrajFinalXY[i+1] = VecTrajFinalXY[i] + Eigen::Vector2d(IntegralX.sum(), IntegralY.sum());
            violaPos = (VecTrajFinalXY[i+1] - init_inner_xy[i]).squaredNorm();
            VecCoeffChainX.head(i*(inner_num+1)).array() += opt_param.first_stage.path_pos_weight * 2.0 * (VecTrajFinalXY[i+1].x() - init_inner_xy[i].x());
            VecCoeffChainY.head(i*(inner_num+1)).array() += opt_param.first_stage.path_pos_weight * 2.0 * (VecTrajFinalXY[i+1].y() - init_inner_xy[i].y());
            cost += opt_param.first_stage.path_pos_weight * violaPos;
            cost_path += opt_param.first_stage.path_pos_weight * violaPos;
        }

        for(int i=0; i<piece_num; i++)
        {
            Eigen::VectorXd CoeffX = VecCoeffChainX.block(i*(inner_num+1),0,inner_num+1,1).cwiseProduct(IntegralChainCoeff);
            Eigen::VectorXd CoeffY = VecCoeffChainY.block(i*(inner_num+1),0,inner_num+1,1).cwiseProduct(IntegralChainCoeff);
            gdC.block<6, 1>(i*6, 1) += VecSingleXGradCArc[i] * CoeffX;
            gdC.block<6, 1>(i*6, 0) += VecSingleXGradCTheta[i] * CoeffX;
            gdC.block<6, 1>(i*6, 1) += VecSingleYGradCArc[i] * CoeffY;
            gdC.block<6, 1>(i*6, 0) += VecSingleYGradCTheta[i] * CoeffY;
            gdT(i) += (VecSingleXGradT[i].cwiseProduct(CoeffX)).sum();
            gdT(i) += (VecSingleYGradT[i].cwiseProduct(CoeffY)).sum();
        }

        // ROS_WARN("cost: %f", cost);
        // ROS_WARN("cost path: %f", cost_path);
        // ROS_WARN("cost moment: %f", cost_moment);

        return;
    }

    void MomaTrajOpt::calSecondStagePenalGrad(double& cost, Eigen::MatrixXd& gdC, Eigen::VectorXd &gdT)
    {
        cost = 0.0;
        gdC.resize(6*piece_num, 9);
        gdC.setZero();
        gdT.resize(piece_num);
        gdT.setZero();

        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
        Eigen::VectorXd state, dstate, d2state, d3state;
        double alpha, omg;
        int inner_num = 2 * opt_param.int_K;
        int int_6K = opt_param.int_K * 6;

        double avg_time = times.mean();

        double violaPos;
        double violaPosPena;
        double violaPosPenaD;
        double violaMom;
        double violaMomPena;
        double violaMomPenaD;

        Eigen::VectorXd IntegralChainCoeff(inner_num + 1);
        IntegralChainCoeff.setZero();
        for(int i=0; i<opt_param.int_K; i++)
            IntegralChainCoeff.segment(2*i, 3) += Eigen::Vector3d(1.0, 4.0, 1.0);

        std::vector<Eigen::MatrixXd> VecSingleXGradCTheta(piece_num);
        std::vector<Eigen::MatrixXd> VecSingleXGradCArc(piece_num);
        std::vector<Eigen::VectorXd> VecSingleXGradT(piece_num);
        std::vector<Eigen::MatrixXd> VecSingleYGradCTheta(piece_num);
        std::vector<Eigen::MatrixXd> VecSingleYGradCArc(piece_num);
        std::vector<Eigen::VectorXd> VecSingleYGradT(piece_num);

        Eigen::Vector2d CurrentXY = start_state.head(2);
        Eigen::VectorXd VecCoeffChainX(piece_num*(inner_num+1)); VecCoeffChainX.setZero();
        Eigen::VectorXd VecCoeffChainY(piece_num*(inner_num+1)); VecCoeffChainY.setZero();
        std::vector<Eigen::Vector2d> VecTrajFinalXY(piece_num+1);
        VecTrajFinalXY[0] = CurrentXY;

        for (int i = 0; i < piece_num; i++)
        {
            const Eigen::Matrix<double, 6, 9> &c = minco_opt.getCoeffs().block<6, 9>(6*i, 0);
            double step = times(i) / opt_param.int_K;
            double half_step = step / 2.0;
            double coeff = step / 6.0;

            Eigen::MatrixXd SingleXGradCTheta(6, inner_num+1);
            Eigen::MatrixXd SingleXGradCArc(6, inner_num+1);
            Eigen::VectorXd SingleXGradT(inner_num+1);
            Eigen::MatrixXd SingleYGradCTheta(6, inner_num+1);
            Eigen::MatrixXd SingleYGradCArc(6, inner_num+1);
            Eigen::VectorXd SingleYGradT(inner_num+1);
            Eigen::VectorXd IntegralX(opt_param.int_K); IntegralX.setZero();
            Eigen::VectorXd IntegralY(opt_param.int_K); IntegralY.setZero();
            
            s1 = 0.0;
            for (int j=0; j<=inner_num; j++)
            {
                if (j % 2 == 0)
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s3 * s2;
                    beta0 << 1.0, s1, s2, s3, s4, s5;
                    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                    s1 += half_step;
                    alpha = 1.0 / inner_num * j;
                    omg = (j==0||j==inner_num)?0.5:1.0;

                    state = c.transpose() * beta0;
                    dstate = c.transpose() * beta1;
                    d2state = c.transpose() * beta2;
                    d3state = c.transpose() * beta3;
                    double syaw = sin(state(0));
                    double cyaw = cos(state(0));

                    if(j!=0)
                    {
                        IntegralX[j/2-1] += coeff * dstate(1) * cyaw;
                        IntegralY[j/2-1] += coeff * dstate(1) * syaw;
                    }
                    if(j!=inner_num)
                    {
                        IntegralX[j/2] += coeff * dstate(1) * cyaw;
                        IntegralY[j/2] += coeff * dstate(1) * syaw;
                    }

                    SingleXGradCTheta.col(j) = -dstate(1) * beta0 * syaw;
                    SingleXGradCArc.col(j) = beta1 * cyaw;
                    SingleXGradT[j] = (d2state(1) * cyaw - dstate(1) * dstate(0) * syaw)
                                      * alpha * coeff + dstate(1) * cyaw / int_6K;
                    SingleYGradCTheta.col(j) = dstate(1) * beta0 * cyaw;
                    SingleYGradCArc.col(j) = beta1 * syaw;
                    SingleYGradT[j] = (d2state(1) * syaw + dstate(1) * dstate(0) * cyaw)
                                      * alpha * coeff + dstate(1) * syaw / int_6K;

                    if (j != 0) CurrentXY += Eigen::Vector2d(IntegralX[j/2-1],IntegralY[j/2-1]);

                    // collision cost
                    double sdf_value;
                    Eigen::Vector2d grad_sdf;
                    grid_map->getDisWithGradI2d(CurrentXY, sdf_value, grad_sdf);
                    violaPos = moma_param.chassis_colli_radius * 1.05 - sdf_value;
                    if (violaPos > 0)
                    {
                        smoothL1Penalty(violaPos, violaPosPena, violaPosPenaD);
                        Eigen::Vector2d grad_to_pos = -omg * step * opt_param.second_stage.collision_weight * violaPosPenaD * grad_sdf;
                        VecCoeffChainX.head(i*(inner_num+1)+j+1).array() += grad_to_pos.x();
                        VecCoeffChainY.head(i*(inner_num+1)+j+1).array() += grad_to_pos.y();
                        gdT(i) += omg * opt_param.second_stage.collision_weight * (violaPosPena / opt_param.int_K);
                        double chassis_colli = omg * step * opt_param.second_stage.collision_weight * violaPosPena;
                        cost += chassis_colli;
                        debug_manager["chassis_colli"] += chassis_colli;
                    }

                    // moment cost
                    double gradViolaMt;
                    double real_alpha = 1.0 / opt_param.int_K * ( (double) j / 2.0 ); 
                    Eigen::MatrixXd gradBeta = Eigen::MatrixXd::Zero(3, 2);
                    for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2)
                    {
                        violaMom = omg_sym * moma_param.max_v * dstate.x() 
                                          + moma_param.max_w * dstate.y() 
                                          - moma_param.max_v * moma_param.max_w;
                        if(violaMom > 0)
                        {
                            smoothL1Penalty(violaMom, violaMomPena, violaMomPenaD);
                            gradViolaMt = real_alpha * (omg_sym * moma_param.max_v * d2state.x() + moma_param.max_w * d2state.y());
                            gradBeta(1, 0) += omg * step * opt_param.second_stage.moment_weight * violaMomPenaD * omg_sym * moma_param.max_v;
                            gradBeta(1, 1) += omg * step * opt_param.second_stage.moment_weight * violaMomPenaD * moma_param.max_w;
                            gdT(i) += omg * opt_param.second_stage.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / opt_param.int_K);
                            double moment = omg * step * opt_param.second_stage.moment_weight * violaMomPena;
                            cost += moment;
                            debug_manager["moment"] += moment;
                        }
                    }
                    for(int omg_sym = -1; omg_sym <= 1; omg_sym += 2)
                    {
                        violaMom = omg_sym * moma_param.max_v * dstate.x() 
                                   - moma_param.max_w * dstate.y() 
                                   - moma_param.max_v * moma_param.max_w;
                        if(violaMom > 0)
                        {
                            smoothL1Penalty(violaMom, violaMomPena, violaMomPenaD);
                            gradViolaMt = real_alpha * (omg_sym * moma_param.max_v * d2state.x() - moma_param.max_w * d2state.y());
                            gradBeta(1, 0) += omg * step * opt_param.second_stage.moment_weight * violaMomPenaD * omg_sym * moma_param.max_v;
                            gradBeta(1, 1) -= omg * step * opt_param.second_stage.moment_weight * violaMomPenaD * moma_param.max_w;
                            gdT(i) += omg * opt_param.second_stage.moment_weight * (violaMomPenaD * gradViolaMt * step + violaMomPena / opt_param.int_K);
                            double moment = omg * step * opt_param.second_stage.moment_weight * violaMomPena;
                            cost += moment;
                            debug_manager["moment"] += moment;
                        }
                    }

                    if (false)
                    {
                        double violaVelPP = - dstate.y();
                        double violaVelPena, violaVelPenaD;
                        if(violaVelPP > 0)
                        {
                            smoothL1Penalty(violaVelPP, violaVelPena, violaVelPenaD);
                            double gradViolaDOT = -real_alpha * d2state.y();
                            gradBeta(1, 1) -= omg * step * opt_param.second_stage.domega_weight * violaVelPenaD;
                            gdT(i) += omg * opt_param.second_stage.domega_weight * (violaVelPenaD * gradViolaDOT * step + violaVelPena / opt_param.int_K);
                            cost += omg * step * opt_param.second_stage.domega_weight * violaVelPena;
                        }
                    }

                    // acc and domega cost
                    double violaAcc = d2state.y()*d2state.y() - moma_param.max_a*moma_param.max_a;
                    double violaAlp = d2state.x()*d2state.x() - moma_param.max_dw*moma_param.max_dw;
                    double violaAccPena, violaAccPenaD, violaAlpPena, violaAlpPenaD;
                    if(violaAcc > 0)
                    {
                        smoothL1Penalty(violaAcc, violaAccPena, violaAccPenaD);
                        double gradViolaAT = 2.0 * real_alpha * d2state.y() * d3state.y();
                        gradBeta(2, 1) +=  omg * step * opt_param.second_stage.acc_weight * violaAccPenaD * 2.0 * d2state.y();
                        gdT(i) += omg * opt_param.second_stage.acc_weight * (violaAccPenaD * gradViolaAT * step + violaAccPena / opt_param.int_K);
                        double cost_acc = omg * step * opt_param.second_stage.acc_weight * violaAccPena;
                        cost += cost_acc;
                        debug_manager["acc"] += cost_acc;    
                    }
                    if(violaAlp > 0)
                    {
                        smoothL1Penalty(violaAlp, violaAlpPena, violaAlpPenaD);
                        double gradViolaDOT = 2.0 * real_alpha * d2state.x() * d3state.x();
                        gradBeta(2, 0) += omg * step * opt_param.second_stage.domega_weight * violaAlpPenaD * 2.0 * d2state.x();
                        gdT(i) += omg * opt_param.second_stage.domega_weight * (violaAlpPenaD * gradViolaDOT * step + violaAlpPena / opt_param.int_K);
                        double cost_domega = omg * step * opt_param.second_stage.domega_weight * violaAlpPena;
                        cost += cost_domega;
                        debug_manager["domega"] += cost_domega;
                    }
                    gdC.block<6, 2>(i*6, 0) += beta0 * gradBeta.row(0) 
                                               + beta1 * gradBeta.row(1) 
                                               + beta2 * gradBeta.row(2);
                    
                    // manipulator
                    gradBeta = Eigen::MatrixXd::Zero(3, moma_param.dof_num);
                    // manipulator environment collision
                    Eigen::VectorXd moma_pos = Eigen::VectorXd::Zero(3+moma_param.dof_num);
                    moma_pos.head(2) = CurrentXY;
                    moma_pos(2) = state(0);
                    moma_pos.tail(moma_param.dof_num) = state.tail(moma_param.dof_num);
                    std::vector<Eigen::Vector4d> colli_pts = moma_param.getColliPts(moma_pos);
                    std::vector<Eigen::Vector3d> pos_grads;
                    double cost_scale = 10.0;
                    for (size_t cidx = 0; cidx < colli_pts.size(); cidx++)
                    {
                        Eigen::Vector3d pc = colli_pts[cidx].head(3);
                        Eigen::Vector3d grad_pc;
                        grid_map->getDisWithGradI3d(pc, sdf_value, grad_pc);
                        violaPos = colli_pts[cidx][3] * cost_scale * 1.1 - sdf_value * cost_scale;
                        Eigen::Vector3d grad_to_pos = Eigen::Vector3d::Zero();
                        if (violaPos > 0)
                        {
                            smoothL1Penalty(violaPos, violaPosPena, violaPosPenaD);
                            grad_to_pos = -omg * step * opt_param.second_stage.mani_colli_weight * violaPosPenaD * grad_pc * cost_scale;
                            gdT(i) += omg * opt_param.second_stage.mani_colli_weight * (violaPosPena / opt_param.int_K);
                            double mani_colli = omg * step * opt_param.second_stage.mani_colli_weight * violaPosPena;
                            cost += mani_colli;
                            debug_manager["mani_colli"] += mani_colli;
                        }
                        //! debug
                        // grad_to_pos.setZero();
                        pos_grads.push_back(grad_to_pos);
                    }
                    // moma self collision
                    for (size_t cidx=0; cidx<colli_pts.size(); cidx++)
                    {
                        // with chassis
                        if (cidx > 2)
                        {
                            double height = moma_param.chassis_height + moma_param.relative_t(2) + 
                                            colli_pts[cidx](3) - colli_pts[cidx](2);
                            if (height > 0)
                            {
                                double violaSelfPena;
                                double violaSelfPenaD;
                                smoothL1Penalty(height, violaSelfPena, violaSelfPenaD);
                                double grad_z = -omg * step * opt_param.second_stage.self_colli_weight * violaSelfPenaD;
                                gdT(i) += omg * opt_param.second_stage.self_colli_weight * (violaSelfPena / opt_param.int_K);
                                double self_colli = omg * step * opt_param.second_stage.self_colli_weight * violaSelfPena;
                                cost += self_colli;
                                debug_manager["self_colli"] += self_colli;
                                pos_grads[cidx](2) += grad_z;
                            }
                        }

                        for (size_t cj=cidx+1; cj<colli_pts.size(); cj++)
                        {
                            // with other link
                            if (moma_param.collision_matrix(cidx, cj) != -1)
                                continue;

                            Eigen::Vector3d diff = colli_pts[cidx].head(3) - colli_pts[cj].head(3);
                            double dist = (colli_pts[cidx](3) + colli_pts[cj](3)) * 
                                         (colli_pts[cidx](3) + colli_pts[cj](3)) - diff.squaredNorm();
                            if (dist > 0)
                            {
                                double violaSelfPena;
                                double violaSelfPenaD;
                                Eigen::Vector3d grad1 = Eigen::Vector3d::Zero();
                                smoothL1Penalty(dist, violaSelfPena, violaSelfPenaD);
                                grad1 = -omg * step * opt_param.second_stage.self_colli_weight * violaSelfPenaD * diff * 2.0;
                                gdT(i) += omg * opt_param.second_stage.self_colli_weight * (violaSelfPena / opt_param.int_K);
                                double self_colli = omg * step * opt_param.second_stage.self_colli_weight * violaSelfPena;
                                cost += self_colli;
                                debug_manager["self_colli"] += self_colli;
                                pos_grads[cidx] += grad1;
                                pos_grads[cj] -= grad1;
                            }
                        }
                    }

                    Eigen::VectorXd moma_grad = moma_param.getColliGrads(moma_pos, pos_grads);

                    // joint pos limit
                    for (size_t ji = 0; ji < moma_param.dof_num; ji++)
                    {
                        double violaJointPos = moma_pos(ji+3) - moma_param.joint_pos_limit_max(ji);
                        double violaJointPosPena, violaJointPosPenaD;
                        if (violaJointPos > 0)
                        {
                            smoothL1Penalty(violaJointPos, violaJointPosPena, violaJointPosPenaD);
                            moma_grad(ji+3) += omg * step * opt_param.second_stage.mani_pos_weight * violaJointPosPenaD;
                            gdT(i) += omg * opt_param.second_stage.mani_pos_weight * (violaJointPosPena / opt_param.int_K);
                            double cost_mani_pos = omg * step * opt_param.second_stage.mani_pos_weight * violaJointPosPena;
                            cost += cost_mani_pos;
                            debug_manager["mani_pos"] += cost_mani_pos;
                        }
                        violaJointPos = -moma_param.joint_pos_limit_max(ji) - moma_pos(ji+3);
                        if (violaJointPos > 0)
                        {
                            smoothL1Penalty(violaJointPos, violaJointPosPena, violaJointPosPenaD);
                            moma_grad(ji+3) -= omg * step * opt_param.second_stage.mani_pos_weight * violaJointPosPenaD;
                            gdT(i) += omg * opt_param.second_stage.mani_pos_weight * (violaJointPosPena / opt_param.int_K);
                            double cost_mani_pos = omg * step * opt_param.second_stage.mani_pos_weight * violaJointPosPena;
                            cost += cost_mani_pos;
                            debug_manager["mani_pos"] += cost_mani_pos;
                        }
                    }
                    VecCoeffChainX.head(i*(inner_num+1)+j+1).array() += moma_grad.x();
                    VecCoeffChainY.head(i*(inner_num+1)+j+1).array() += moma_grad.y();
                    gdC.block<6, 1>(i*6, 0) += beta0 * moma_grad(2);
                    gdT(i) += moma_grad(2) * dstate(0) * real_alpha;
                    gradBeta.block<1, 7>(0, 0) = moma_grad.tail(moma_param.dof_num);
                    gdT(i) += moma_grad.tail(moma_param.dof_num).dot(dstate.tail(moma_param.dof_num)) * real_alpha;
                                        
                    // joint vel and acc
                    Eigen::VectorXd dq = dstate.tail(moma_param.dof_num);
                    Eigen::VectorXd d2q = d2state.tail(moma_param.dof_num);
                    Eigen::VectorXd d3q = d3state.tail(moma_param.dof_num);
                    Eigen::VectorXd violaDq = dq.cwiseAbs2() - moma_param.joint_vel_limit.cwiseAbs2();
                    Eigen::VectorXd violaD2q = d2q.cwiseAbs2() - moma_param.joint_acc_limit.cwiseAbs2();
                    for (size_t jidx = 0; jidx < moma_param.dof_num; jidx++)
                    {
                        {
                            double violaDqPena;
                            double violaDqPenaD;
                            if (violaDq(jidx) > 0)
                            {
                                smoothL1Penalty(violaDq(jidx), violaDqPena, violaDqPenaD);
                                double gradViolaVT = 2.0 * real_alpha * dq(jidx) * d2q(jidx);
                                gradBeta(1, jidx) +=  omg * step * opt_param.second_stage.mani_vel_weight * violaDqPenaD * 2.0 * dq(jidx);
                                gdT(i) += omg * opt_param.second_stage.mani_vel_weight * (violaDqPenaD * gradViolaVT * step + violaDqPena / opt_param.int_K);
                                double cost_mani_vel = omg * step * opt_param.second_stage.mani_vel_weight * violaDqPena;
                                cost += cost_mani_vel;
                                debug_manager["mani_vel"] += cost_mani_vel;
                            }
                        }
                        {
                            double violaD2qPena;
                            double violaD2qPenaD;
                            if (violaD2q(jidx) > 0)
                            {
                                smoothL1Penalty(violaD2q(jidx), violaD2qPena, violaD2qPenaD);
                                double gradViolaAT = 2.0 * real_alpha * d2q(jidx) * d3q(jidx);
                                gradBeta(2, jidx) +=  omg * step * opt_param.second_stage.mani_acc_weight * violaD2qPenaD * 2.0 * d2q(jidx);
                                gdT(i) += omg * opt_param.second_stage.mani_acc_weight * (violaD2qPenaD * gradViolaAT * step + violaD2qPena / opt_param.int_K);
                                double cost_mani_acc = omg * step * opt_param.second_stage.mani_acc_weight * violaD2qPena;
                                cost += cost_mani_acc;
                                debug_manager["mani_acc"] += cost_mani_acc;
                            }
                        }
                    }
                    gdC.block<6, 7>(i*6, 2) += beta0 * gradBeta.row(0) 
                                               + beta1 * gradBeta.row(1) 
                                               + beta2 * gradBeta.row(2);
                }
                else
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s3 * s2;
                    beta0 << 1.0, s1, s2, s3, s4, s5;
                    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                    s1 += half_step;
                    alpha = 1.0 / inner_num * j;
                    state = c.transpose() * beta0;
                    dstate = c.transpose() * beta1;
                    d2state = c.transpose() * beta2;

                    double cyaw = cos(state.x()), syaw = sin(state.x());
                    IntegralX[j/2] += 4 * coeff * dstate.y() * cyaw;
                    IntegralY[j/2] += 4 * coeff * dstate.y() * syaw;
                    
                    SingleXGradCArc.col(j) = beta1 * cyaw;
                    SingleXGradCTheta.col(j) = -dstate.y() * beta0 * syaw;
                    SingleXGradT[j] = (d2state.y() * cyaw - dstate.y() * dstate.x() * syaw)*alpha*coeff + dstate.y() * cyaw / int_6K;

                    SingleYGradCArc.col(j) = beta1 * syaw;
                    SingleYGradCTheta.col(j) = dstate.y() * beta0 * cyaw;
                    SingleYGradT[j] = (d2state.y() * syaw + dstate.y() * dstate.x() * cyaw)*alpha*coeff + dstate.y() * syaw / int_6K;
                }
            }

            VecSingleXGradCArc[i] = SingleXGradCArc * coeff;
            VecSingleXGradCTheta[i] = SingleXGradCTheta * coeff;
            VecSingleXGradT[i] = SingleXGradT;
            VecSingleYGradCArc[i] = SingleYGradCArc * coeff;
            VecSingleYGradCTheta[i] = SingleYGradCTheta * coeff;
            VecSingleYGradT[i] = SingleYGradT;
            VecTrajFinalXY[i+1] = VecTrajFinalXY[i] + Eigen::Vector2d(IntegralX.sum(), IntegralY.sum());
            
            double mean_time_lowb = 0.5;
            double mean_time_uppb = 2.0;
            if( times[i] < avg_time * mean_time_lowb)
            {
                double cost_meant = opt_param.second_stage.mean_time_weight * (times[i] - avg_time * mean_time_lowb) * (times[i] - avg_time * mean_time_lowb);
                cost += cost_meant;
                debug_manager["mean_time"] += cost_meant;
                gdT.array() += opt_param.second_stage.mean_time_weight * 2.0 * (times[i] - avg_time * mean_time_lowb)  * (- mean_time_lowb / piece_num);
                gdT(i) += opt_param.second_stage.mean_time_weight * 2.0 * (times[i] - avg_time * mean_time_lowb);
            }
            if (times[i] > avg_time * mean_time_uppb)
            {
                double cost_meant = opt_param.second_stage.mean_time_weight * (times[i] - avg_time * mean_time_uppb) * (times[i] - avg_time * mean_time_uppb);
                cost += cost_meant;
                debug_manager["mean_time"] += cost_meant;
                gdT.array() += opt_param.second_stage.mean_time_weight * 2.0 * (times[i] - avg_time * mean_time_uppb) * (-mean_time_uppb / piece_num);
                gdT(i) += opt_param.second_stage.mean_time_weight * 2.0 * (times[i] - avg_time * mean_time_uppb);
            }
        }

        // final xy
        // final_xy_error = VecTrajFinalXY.back() - end_state.head(2);
        // cost += 0.5 * (alm_rho[0] * pow(final_xy_error.x() + alm_lambda[0]/alm_rho[0], 2) + alm_rho[1] * pow(final_xy_error.y() + alm_lambda[1]/alm_rho[1], 2));
        // cost_endp += 0.5 * (alm_rho[0] * pow(final_xy_error.x() + alm_lambda[0]/alm_rho[0], 2) + alm_rho[1] * pow(final_xy_error.y() + alm_lambda[1]/alm_rho[1], 2));
        // VecCoeffChainX.array() += alm_rho[0] * (final_xy_error.x() + alm_lambda[0]/alm_rho[0]);
        // VecCoeffChainY.array() += alm_rho[1] * (final_xy_error.y() + alm_lambda[1]/alm_rho[1]);

        // final ee
        Eigen::VectorXd end_moma_state = Eigen::VectorXd::Zero(moma_param.dof_num+3);
        end_moma_state.head(2) = VecTrajFinalXY.back();
        end_moma_state(2) = minco_end_state(0, 0);
        end_moma_state.tail(moma_param.dof_num) = minco_end_state.col(0).tail(moma_param.dof_num);
        Eigen::VectorXd ee_pose_now = moma_param.getFKPose(end_moma_state);
        final_pose_error = ee_pose_now - ee_pose;

        double cost_endp = 0.0;
        Eigen::VectorXd grad_ee = Eigen::VectorXd::Zero(ee_pose_now.size());
        for (size_t i = 0; i < ee_pose_now.size(); i++)
        {
            double hx = final_pose_error(i) + alm_lambda[i]/alm_rho[i];
            double hcost = 0.5 * alm_rho(i) * hx * hx;
            cost_endp += hcost;
            cost += hcost;
            grad_ee(i) = alm_rho(i) * hx;
        }
        debug_manager["endp"] += cost_endp;
        if (debug_manager.checkInf())
        {
            PRINT_RED("Inf detected in endp");
            PRINT_RED("final_xy_error: "<<final_xy_error);
            PRINT_RED("cost_endp: "<<cost_endp);
            PRINT_RED("inner_pts: \n" << inner_pts.transpose());
            PRINT_RED("times: \n" << times.transpose());
            PRINT_RED("minco start_state: \n" << minco_start_state);
            PRINT_RED("minco end_state: \n" << minco_end_state);
            PRINT_RED("start state:\n"<<start_state.transpose());
            PRINT_RED("end state:\n"<<end_state.transpose());
            debug_manager.printinfo();
            gdC.setZero();
            gdT.setZero();
            cost = 1.0e+22;
            return;
            exit(0);
        }

        // grad ee to moma
        {
            const Eigen::Matrix<double, 6, 9> &c = minco_opt.getCoeffs().block<6, 9>(6*(piece_num-1), 0);
            s1 = times(piece_num-1);
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s3 * s2;
            beta0 << 1.0, s1, s2, s3, s4, s5;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
            dstate = c.transpose() * beta1;

            Eigen::VectorXd grad_ee2moma = moma_param.getEEGrads(end_moma_state, grad_ee);
            
            gdC.block<6, 1>((piece_num-1)*6, 0) += beta0 * grad_ee2moma(2);
            Eigen::Matrix<double, 1, 7> grad_temp_tail = grad_ee2moma.tail(moma_param.dof_num);
            gdC.block<6, 7>((piece_num-1)*6, 2) += beta0 * grad_temp_tail;
            gdT(piece_num-1) += grad_ee2moma(2) * dstate(0);
            gdT(piece_num-1) += grad_ee2moma.tail(moma_param.dof_num).dot(dstate.tail(moma_param.dof_num));

            VecCoeffChainX.array() += grad_ee2moma(0);
            VecCoeffChainY.array() += grad_ee2moma(1);
        }
        
        for(int i=0; i<piece_num; i++)
        {
            Eigen::VectorXd CoeffX = VecCoeffChainX.block(i*(inner_num+1),0,inner_num+1,1).cwiseProduct(IntegralChainCoeff);
            Eigen::VectorXd CoeffY = VecCoeffChainY.block(i*(inner_num+1),0,inner_num+1,1).cwiseProduct(IntegralChainCoeff);
            gdC.block<6, 1>(i*6, 1) += VecSingleXGradCArc[i] * CoeffX;
            gdC.block<6, 1>(i*6, 0) += VecSingleXGradCTheta[i] * CoeffX;
            gdC.block<6, 1>(i*6, 1) += VecSingleYGradCArc[i] * CoeffY;
            gdC.block<6, 1>(i*6, 0) += VecSingleYGradCTheta[i] * CoeffY;
            gdT(i) += (VecSingleXGradT[i].cwiseProduct(CoeffX)).sum();
            gdT(i) += (VecSingleYGradT[i].cwiseProduct(CoeffY)).sum();
        }

        // ROS_WARN("cost: %f", cost);
        // ROS_WARN("cost path: %f", cost_path);
        // ROS_WARN("cost moment: %f", cost_moment);

        return;
    }

    int MomaTrajOpt::earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
        const double fx, const double step, int k, int ls)
    {
        MomaTrajOpt& obj = *(MomaTrajOpt*)ptrObj;

#ifdef PUB_DEBUG
        int opt_var_idx = 0;
        Eigen::Map<const Eigen::VectorXd> Tau(x.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::VectorXd> Theta(x.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::VectorXd> Arc(x.data()+opt_var_idx, obj.piece_num);
        opt_var_idx += obj.piece_num;
        Eigen::Map<const Eigen::MatrixXd> Vq(x.data()+opt_var_idx, obj.moma_param.dof_num, obj.piece_num);

        Eigen::VectorXd Ts;
        obj.calTfromTau(Tau, Ts);
        obj.minco_end_state(0, 0) = Theta[obj.piece_num-1];
        obj.minco_end_state(1, 0) = Arc[obj.piece_num-1];
        Eigen::MatrixXd Inner_pts;
        Inner_pts.resize(9, obj.piece_num-1);
        Inner_pts.row(0) = Theta.head(obj.piece_num-1);
        Inner_pts.row(1) = Arc.head(obj.piece_num-1);
        for (int i = 0; i < obj.piece_num-1; i++)
        {
            for (size_t j = 0; j < obj.moma_param.dof_num; j++)
                Inner_pts(j+2, i) = obj.sigmoidC2(Vq(j, i), obj.moma_param.joint_pos_limit_max(j));
        }
        for (int i = 0; i < obj.moma_param.dof_num; i++)
            obj.minco_end_state(i+2, 0) = obj.sigmoidC2(Vq(i, obj.piece_num-1), obj.moma_param.joint_pos_limit_max(i));
        obj.minco_opt.generate(obj.minco_start_state, obj.minco_end_state, Inner_pts, Ts);

        // MomaTraj temp_traj = obj.getTraj();
        // obj.pubDebugTraj(temp_traj);
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        obj.debug_manager.reset();
        Eigen::VectorXd grad_no;
        grad_no.resize(x.size());
        secondStageCostCallback(ptrObj, x, grad_no);
        obj.debug_manager.publish();
        // obj.debug_manager.printinfo();

#endif
        return k > obj.opt_param.second_stage.lbfgs_param.max_iterations;

    }
}
