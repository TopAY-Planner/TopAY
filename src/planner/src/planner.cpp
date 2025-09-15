#include "planner/planner.h"

namespace nmoma_planner
{
    void Planner::init(ros::NodeHandle& nh)
    {
        GET_PARAM_OR_THROW(nh, "agent/local_mode", local_mode);
        GET_PARAM_OR_THROW(nh, "agent/replan_interval", replan_interval);
        GET_PARAM_OR_THROW(nh, "agent/planning_horizon", planning_horizon);
        GET_PARAM_OR_THROW(nh, "agent/mode", mode);
        GET_PARAM_OR_THROW(nh, "agent/stat_num", stat_num);
        GET_PARAM_OR_THROW(nh, "agent/only_front", only_front);
        GET_PARAM_OR_THROW(nh, "agent/fixed_startgoal", fixed_startgoal);
        
        GET_PARAM_OR_THROW(nh, "agent/planner", planner_type);
        GET_PARAM_OR_THROW(nh, "agent/fixed_sequence", fixed_sequence);

        GET_PARAM_OR_THROW(nh, "agent/startgoal_dist_range", startgoal_dist_range);

        GET_PARAM_OR_THROW(nh, "agent/scene", scene);
        int number;
        bool random_ee = true;
        nh.getParam("agent/random_ee", random_ee);
        if (!random_ee)
        {
            std::vector<double> pick, mid, place;
            nh.param<std::vector<double>>("agent/pick_state", pick, std::vector<double>());
            nh.param<std::vector<double>>("agent/mid_state", mid, std::vector<double>());
            nh.param<std::vector<double>>("agent/place_state", place, std::vector<double>());
            pick_vec = Eigen::Map<Eigen::VectorXd>(pick.data(), pick.size());
            Eigen::VectorXd mid_vec = Eigen::Map<Eigen::VectorXd>(mid.data(), mid.size());
            place_vec = Eigen::Map<Eigen::VectorXd>(place.data(), place.size());
            wps_list.push_back(pick_vec);
            wps_list.push_back(mid_vec);
            wps_list.push_back(place_vec);
            wps_list.push_back(mid_vec);
            wps_list.push_back(Eigen::VectorXd::Zero(pick_vec.size()));
        }

        grid_map.reset(new GridMap);
        grid_map->init(nh);
        
        graph_search = std::make_shared<JPS::GraphSearch>(grid_map, moma_param.chassis_colli_radius);
        birrts = std::make_shared<BiRRTs>(grid_map);
        birrts->init(nh);
        topo_prm.reset(new TopologyPRM);
        topo_prm->setEnv(grid_map);
        topo_prm->init(nh);
        mcrrts = std::make_shared<MCRRTs>(grid_map);
        mcrrts->init(nh);
        ompl_planner = std::make_shared<OMPLPlanner>(grid_map);
        ompl_planner->init(nh);
        traj_opter = std::make_shared<MomaTrajOpt>(grid_map);
        traj_opter->init(nh);
        mpc.reset(new OMPC);
        // mpc.reset(new MPC);
        mpc->init(nh);

        traj_opters.resize(8);
        mc_rrtsers.resize(8);
        for (int i = 0; i < 8; i++)
        {
            traj_opters[i] = std::make_unique<MomaTrajOpt>(grid_map);
            traj_opters[i]->init(nh);
            mc_rrtsers[i].reset(new MCRRTs(grid_map));
            mc_rrtsers[i]->init(nh);
            opt_traj_pub_list.push_back(
                nh.advertise<visualization_msgs::MarkerArray>("/opt_traj_" + std::to_string(i + 1), 1)
            );
            front_traj_pub_list.push_back(
                nh.advertise<visualization_msgs::MarkerArray>("/front_traj_" + std::to_string(i + 1), 1)
            );

            vis_isAvailable.push_back(false);
        }
        vis_front_paths.resize(8);
        vis_opt_paths.resize(8);
        vis_timer = nh.createTimer(ros::Duration(0.1), &Planner::timerCallback, this);

        plot_traj_ee = nh.advertise<visualization_msgs::Marker>("plot_traj_ee", 1);
        plot_traj_ee = nh.advertise<visualization_msgs::Marker>("plot_traj_ee_2", 1);


        front_pub = nh.advertise<visualization_msgs::MarkerArray>("/front_path", 1);
        ompl_pub = nh.advertise<visualization_msgs::MarkerArray>("/ompl_path", 1);
        end_pub = nh.advertise<visualization_msgs::MarkerArray>("/end_path", 1);
        car_traj_pub = nh.advertise<nav_msgs::Path>("/car_traj", 1);
        car_target_pub = nh.advertise<visualization_msgs::Marker>("/car_target", 1);

        bk_front_pub = nh.advertise<visualization_msgs::MarkerArray>("/bk_front_path", 1);
        bk_end_pub = nh.advertise<visualization_msgs::MarkerArray>("/bk_end_path", 1);

        init_end_pub = nh.advertise<visualization_msgs::MarkerArray>("/init_end_path", 1);
        afirst_end_pub = nh.advertise<visualization_msgs::MarkerArray>("/afirst_end_path", 1);

        prm_pub = nh.advertise<visualization_msgs::MarkerArray>("/prm_path", 1);
        vis_prm_pub = nh.advertise<visualization_msgs::MarkerArray>("/vis_prm_path", 1);

        tracking_traj = nh.advertise<visualization_msgs::MarkerArray>("/tracking_traj", 1);
        time_txt = nh.advertise<visualization_msgs::MarkerArray>("/time_txt", 1);

        moma_cmd_pub = nh.advertise<fake_moma::MomaCmd>("cmd", 1);

        // wps_sub = nh.subscribe<geometry_msgs::Pose>("/manual_target", 1, &Planner::rcvWpsCallBack, this);
        state_sub = nh.subscribe("state", 1, &Planner::rcvStateCallBack, this);
        if (mode.compare("planner") == 0) 
        {
            if (random_ee)
                statistics_sub = nh.subscribe("/move_base_simple/goal", 1, &Planner::planCallBack, this);
            else
                statistics_sub = nh.subscribe("/manual_target", 1, &Planner::planCallBack, this);
        } else if (mode.compare("benchmark") == 0) {
            statistics_sub = nh.subscribe("/move_base_simple/goal", 1, &Planner::benchmarkCallback, this);
        } else if (mode.compare("ablation") == 0) {
            statistics_sub = nh.subscribe("/move_base_simple/goal", 1, &Planner::ablationCallback, this);
        } else {
            throw std::runtime_error("Unidentified mode");
        }


        if (local_mode)
        {
            std::thread cmd_thread(Planner::cmdCallback, this);
            cmd_thread.detach();
            std::thread replan_thread(Planner::replanCallback, this);
            replan_thread.detach();
            std::thread safe_thread(Planner::safeCallback, this);
            safe_thread.detach();
        }
        
        if (fixed_sequence) {
            eng = default_random_engine(42);
        } else {
            random_device rd;
            eng = default_random_engine(rd());

        }

        se2_set.setZero();
        front_path.clear();
        now_state.resize(3+moma_param.dof_num);
        now_state.setZero();
        now_dstate = now_state;
        begin_time = ros::Time::now();

        car_pts_pub =  nh.advertise<visualization_msgs::MarkerArray>("car_pts", 1);
        mani_pts_pub=  nh.advertise<visualization_msgs::Marker>("mani_pts", 100, true);
        
        plot_traj_ee = nh.advertise<visualization_msgs::Marker>("ee_traj", 1, true);
        plot_traj_ee_2=nh.advertise<visualization_msgs::Marker>("ee_traj_2", 1, true);

        mesh_traj_pub= nh.advertise<planner::MeshTraj>("mesh_traj", 1, true);


        return;
    }

    void Planner::cmdCallback(void *obj)
    {
        Planner *tsvr = reinterpret_cast<Planner *>(obj);
        while (true)
        {
            ros::Time start_time = ros::Time::now();
            if (tsvr->mpc->hasTraj())
            {
                // tsvr->moma_cmd_pub.publish(tsvr->mpc->getCmd(tsvr->now_state));
                tsvr->mpc->pubCmd(tsvr->now_state, tsvr->moma_cmd_pub, tsvr->gripper_open);
                double t_mpc = (ros::Time::now() - start_time).toSec() * 1000.0;
                if (t_mpc > 1000.0 / tsvr->mpc->ctrl_freq)
                    PRINT_YELLOW("[Planner] MPC time too long: " << t_mpc << " ms");
            }

            int cmd_mm_num = 1000.0 / tsvr->mpc->ctrl_freq;
            std::chrono::milliseconds dura(max(cmd_mm_num - (int)((ros::Time::now() - start_time).toSec() * 1000), 1));
            std::this_thread::sleep_for(dura);
        }
        return;
    }

    void Planner::planCallBack(const geometry_msgs::PoseStamped msg)
    {
        Eigen::VectorXd end_state = Eigen::VectorXd::Zero(3+moma_param.dof_num);

        if (msg.header.frame_id.compare("target") == 0)
        {
            if (!wps_list.empty())
                end_state = wps_list[0];
            // Eigen::VectorXd moma_set = Eigen::VectorXd::Zero(3+moma_param.dof_num);
            // moma_set.head(3) = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, 0.0);
            // Eigen::VectorXd ee_set = Eigen::VectorXd::Zero(9);
            // ee_set.head(3) = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
            // Eigen::Matrix3d R = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, 
            //                                      msg.pose.orientation.y, msg.pose.orientation.z).toRotationMatrix();
            // ee_set.segment(3, 3) = R.row(0);
            // ee_set.tail(3) = R.row(1);
            // if (traj_opter->optimizeEE(moma_set, ee_set))
            //     end_state = moma_set;
            // else
            //     return;
        }else
        {
            se2_set(0) = msg.pose.position.x;
            se2_set(1) = msg.pose.position.y;
            double dist;
            grid_map->getDistance2d(se2_set.head(2), dist);
            if (dist < moma_param.chassis_colli_radius)
                return;

            se2_set(2) = atan2(2.0*msg.pose.orientation.z*msg.pose.orientation.w, 
                                2.0*msg.pose.orientation.w*msg.pose.orientation.w-1.0);
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.id = 10086;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = moma_param.chassis_colli_radius * 2.0;
            marker.scale.y = moma_param.chassis_colli_radius * 2.0;
            marker.scale.z = moma_param.chassis_height;
            marker.pose.position.x = se2_set[0];
            marker.pose.position.y = se2_set[1];
            marker.pose.position.z = moma_param.chassis_height / 2.0;
            marker.pose.orientation.w = 1.0;
            marker.color.a = 0.5;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            car_target_pub.publish(marker);

            uniform_real_distribution<double> rand_q(0.0, 1.0);
            end_state.resize(3+moma_param.dof_num);
            end_state.setZero();
            end_state.head(3) = se2_set;
            
            ros::Time find_time_start = ros::Time::now();
            bool timeout;
            do
            {
                for (size_t i=0; i<moma_param.dof_num; i++)
                    end_state(3+i) = (moma_param.joint_pos_limit_max(i) - 
                                    moma_param.joint_pos_limit_min(i)) * rand_q(eng)
                                    + moma_param.joint_pos_limit_min(i);
            } while (grid_map->isWholeBodyCollision(end_state)
                    && !(timeout = (ros::Time::now() - find_time_start).toSec() > 1.0));

            if(timeout) return;
        }

        global_goal = end_state;
        has_goal = true;
        Eigen::VectorXd local_start = now_state;
        Eigen::VectorXd local_v = now_dstate;
        if (has_traj && local_mode)
        {
            double t = (ros::Time::now() - last_replan_time).toSec() +planning_budget;
            local_start = end_traj.getState(t);
            local_v = end_traj.getDState(t);
        }

        ros::Time start_time = ros::Time::now();
        bool succ = false;
        if (planner_type.compare("moma") == 0)
            succ = planMomaParallel(local_start, end_state, local_v).first;
        
        double duration = (ros::Time::now() - start_time).toSec() * 1000.0;

        vis_time(time_txt, duration, 4546);

        vis_path_mesh(end_traj, 16, 
            tracking_traj,
            {226.0/255.0, 145.0/255.0, 53.0/255.0, 0.1}, 2010);

        geometry_msgs::Point pos;
        pos.x = 1.0;
        pos.y = 2.0;
        pos.z = 1.0;
        

        if (succ)
        {
            global_traj = end_traj;
            mpc->setTraj(end_traj, 0.0);
            begin_time = ros::Time::now();
            has_traj = true;
        }

        return;
    }

    void Planner::ablationCallback(const geometry_msgs::PoseStamped msg)
    {
        Eigen::Vector3d start = Eigen::Vector3d::Zero(3);
        Eigen::Vector3d end;
        end << 3.0, 3.0, 0.0;
        
        Eigen::VectorXd start_state = Eigen::VectorXd::Zero(3+moma_param.dof_num);
        start_state.head(3) = start;
        Eigen::VectorXd end_state = Eigen::VectorXd::Zero(3+moma_param.dof_num);
        end_state.head(3) = end;

        auto _checkcollision = [&](Eigen::VectorXd state) -> bool
        {
            return grid_map->isWholeBodyCollision(state);
        };
        
        int comparison_num = 0;

        int num_moma_succ = 0;
        double mean_moma_path_length = 0.0;
        double mean_moma_duration = 0.0;

        int num_nontopo_succ = 0;
        double mean_nontopo_path_length = 0.0;
        double mean_nontopo_duration = 0.0;

        int num_seq_succ = 0;
        double mean_seq_path_length = 0.0;
        double mean_seq_duration = 0.0;

        size_t num_plan = 0;
        for (; num_plan<stat_num && ros::ok(); num_plan++) {
            // Randomly generate map, start and end state
            do {
                if (!fixed_startgoal) {
                    Eigen::Vector3d min_bound = grid_map->min_boundary;
                    Eigen::Vector3d max_bound = grid_map->max_boundary;
                    
                    uniform_real_distribution<double> rand_x(min_bound(0)+2.0, max_bound(0)-2.0);
                    uniform_real_distribution<double> rand_y(min_bound(1)+2.0, max_bound(1)-2.0);
                    uniform_real_distribution<double> rand_theta(-M_PI, M_PI);
                    uniform_real_distribution<double> rand_q(0.0, 1.0);
                    
                    end << rand_x(eng), rand_y(eng), rand_theta(eng);
                    end_state.head(3) = end;
                    
                    start_state.head(3) = start;
                    
                    double startgoal_dist = (start.head(2) - end.head(2)).norm();
                    
                    if (startgoal_dist < startgoal_dist_range[0]
                    || startgoal_dist > startgoal_dist_range[1]) continue;

                    
                    if (scene.compare("cuboids") == 0) {
                        grid_map->regenerateMap();
                    } else if (scene.compare("tables") == 0) {
                        grid_map->regenerateDesk({
                            start_state.head(2),
                            end_state.head(2)
                        });
                    }
                    else {
                        throw std::runtime_error("Invalid scene type.");
                    }

                    ros::Time time_start;
                    bool timeout, start_state_collision, end_state_collision;
                    time_start = ros::Time::now();
                    do
                    {
                        for (size_t i=0; i<moma_param.dof_num; i++)
                            end_state(3+i) = (moma_param.joint_pos_limit_max(i) - 
                                            moma_param.joint_pos_limit_min(i)) * rand_q(eng)
                                            + moma_param.joint_pos_limit_min(i);
                    } while ((end_state_collision = _checkcollision(end_state))
                            && !(timeout = (ros::Time::now() - time_start).toSec() > 1.0));
                    if(timeout || end_state_collision) continue;
    
                    time_start = ros::Time::now();
                    do
                    {
                        for (size_t i=0; i<moma_param.dof_num; i++)
                            start_state(3+i) = (moma_param.joint_pos_limit_max(i) - 
                                            moma_param.joint_pos_limit_min(i)) * rand_q(eng)
                                            + moma_param.joint_pos_limit_min(i);
                    } while ((start_state_collision = _checkcollision(start_state))
                            && !(timeout = (ros::Time::now() - time_start).toSec() > 1.0));
                    if(timeout || start_state_collision) continue;
                    break;
                } else if (fixed_startgoal) {
                    grid_map->regenerateDesk({
                        start_state.head(2),
                        end_state.head(2)
                    });
                    if(_checkcollision(start_state) || _checkcollision(end_state))
                        continue;
                    break;
                }
            } while (true);
            
            

            if (_checkcollision(start_state) || _checkcollision(end_state))
                PRINT_RED("Collision detected, skip this data point.");
            
            Eigen::VectorXd v = Eigen::VectorXd::Zero(3+moma_param.dof_num);
            
            bool moma_succ = false;
            double moma_path = 0.0;
            double moma_duration = 0.0;
            float moma_optim_time = 0.0;
            MomaTraj moma_traj;
            {
                ros::Time start_time = ros::Time::now();
                std::tie(moma_succ, moma_traj) = planMomaParallel(start_state, end_state, v);
                moma_duration = (ros::Time::now() - start_time).toSec();
                if(moma_succ) {
                    num_moma_succ++;
                    moma_path = moma_traj.getTotalDuration();
                }
            }

            bool nontopo_succ = false;
            double nontopo_path = 0.0;
            double nontopo_duration = 0.0;
            float nontopo_optim_time = 0.0;
            {
                ros::Time start_time = ros::Time::now();
                std::tie(nontopo_succ, nontopo_optim_time) = planMomaNonTOPO(start_state, end_state, v);
                nontopo_duration = (ros::Time::now() - start_time).toSec();
                if(nontopo_succ){
                    num_nontopo_succ++;
                    nontopo_path = end_traj.getTotalDuration();
                }
            }


            bool seq_succ = false;
            double seq_path = 0.0;
            double seq_duration = 0.0;
            float seq_optim_time = 0.0;
            {
                ros::Time start_time = ros::Time::now();
                std::tie(seq_succ, seq_optim_time) = planMomaSequential(start_state, end_state, v);
                seq_duration = (ros::Time::now() - start_time).toSec();
                if(seq_succ) {
                    num_seq_succ++;
                    seq_path = end_traj.getTotalDuration();
                }
            }

            if (moma_succ && nontopo_succ && seq_succ) {
                comparison_num++;

                // moma
                mean_moma_path_length += (moma_path - mean_moma_path_length) / comparison_num;
                mean_moma_duration += (moma_duration - mean_moma_duration) / comparison_num;

                // nontopo
                mean_nontopo_path_length += (nontopo_path - mean_nontopo_path_length) / comparison_num;
                mean_nontopo_duration += (nontopo_duration - mean_nontopo_duration) / comparison_num;

                // seq
                mean_seq_path_length += (seq_path - mean_seq_path_length) / comparison_num;
                mean_seq_duration += (seq_duration - mean_seq_duration) / comparison_num;
            }
        }

        PRINT_GREEN("[Planner] benchmark done.");
        PRINT_GREEN("MOMA:\t"       << num_moma_succ    << "/" << num_plan  << "\tAvg. Plan time: " << mean_moma_duration   << "ms\t Avg. Path Length: " << mean_moma_path_length   << "ms" << std::endl);
        PRINT_GREEN("NONTOPO:\t"    << num_nontopo_succ << "/" << num_plan  << "\tAvg. Plan time: " << mean_nontopo_duration<< "ms\t Avg. Path Length: " << mean_nontopo_path_length<< "ms" << std::endl);
        PRINT_GREEN("SEQ:\t"        << num_seq_succ     << "/" << num_plan  << "\tAvg. Plan time: " << mean_seq_duration    << "ms\t Avg. Path Length: " << mean_seq_path_length    << "ms" << std::endl);

        PRINT_GREEN("Comparison:\tNum: " << comparison_num << std::endl);
        return;
    }
    
    void Planner::benchmarkCallback(const geometry_msgs::PoseStamped msg)
    {
        Eigen::Vector3d start = Eigen::Vector3d::Zero(3);
        Eigen::Vector3d end;
        end << 3.0, 3.0, 0.0;
        
        Eigen::VectorXd start_state = Eigen::VectorXd::Zero(3+moma_param.dof_num);
        start_state.head(3) = start;
        Eigen::VectorXd end_state = Eigen::VectorXd::Zero(3+moma_param.dof_num);
        end_state.head(3) = end;

        auto _checkcollision = [&](Eigen::VectorXd state) -> bool
        {
            return grid_map->isWholeBodyCollision(state);
        };
        
        int comparison_num = 0;

        int num_moma_succ = 0;
        double mean_moma_path_length = 0.0;
        double mean_moma_duration = 0.0;

        size_t num_plan = 0;
        for (; num_plan<stat_num && ros::ok(); num_plan++) {
            // Randomly generate map, start and end state
            do {
                if (!fixed_startgoal) {
                    Eigen::Vector3d min_bound = grid_map->min_boundary;
                    Eigen::Vector3d max_bound = grid_map->max_boundary;
                    
                    uniform_real_distribution<double> rand_x(min_bound(0)+2.0, max_bound(0)-2.0);
                    uniform_real_distribution<double> rand_y(min_bound(1)+2.0, max_bound(1)-2.0);
                    uniform_real_distribution<double> rand_theta(-M_PI, M_PI);
                    uniform_real_distribution<double> rand_q(0.0, 1.0);
                    
                    end << rand_x(eng), rand_y(eng), rand_theta(eng);
                    end_state.head(3) = end;
                    
                    start << rand_x(eng), rand_y(eng), rand_theta(eng);
                    start_state.head(3) = start;
                    
                    double startgoal_dist = (start.head(2) - end.head(2)).norm();
                    
                    if (startgoal_dist < startgoal_dist_range[0]
                    || startgoal_dist > startgoal_dist_range[1]) continue;

                    if (scene.compare("cuboids") == 0) {
                        grid_map->regenerateMap();
                    } else if (scene.compare("tables") == 0) {
                        grid_map->regenerateDesk({
                            start_state.head(2),
                            end_state.head(2)
                        });
                    }
                    else {
                        throw std::runtime_error("Invalid scene type.");
                    }

                    ros::Time time_start;
                    bool timeout, start_state_collision, end_state_collision;
                    time_start = ros::Time::now();
                    do
                    {
                        for (size_t i=0; i<moma_param.dof_num; i++)
                            end_state(3+i) = (moma_param.joint_pos_limit_max(i) - 
                                            moma_param.joint_pos_limit_min(i)) * rand_q(eng)
                                            + moma_param.joint_pos_limit_min(i);
                    } while ((end_state_collision = _checkcollision(end_state))
                            && !(timeout = (ros::Time::now() - time_start).toSec() > 1.0));
                    if(timeout || end_state_collision) continue;
    
                    time_start = ros::Time::now();
                    do
                    {
                        for (size_t i=0; i<moma_param.dof_num; i++)
                            start_state(3+i) = (moma_param.joint_pos_limit_max(i) - 
                                            moma_param.joint_pos_limit_min(i)) * rand_q(eng)
                                            + moma_param.joint_pos_limit_min(i);
                    } while ((start_state_collision = _checkcollision(start_state))
                            && !(timeout = (ros::Time::now() - time_start).toSec() > 1.0));
                    if(timeout || start_state_collision) continue;
                    break;
                } else if (fixed_startgoal) {
                    grid_map->regenerateDesk({
                        start_state.head(2),
                        end_state.head(2)
                    });
                    if(_checkcollision(start_state) || _checkcollision(end_state))
                        continue;
                    break;
                }
            } while (true);
            
            if (_checkcollision(start_state) || _checkcollision(end_state))
                PRINT_RED("Collision detected, skip this data point.");
            
            Eigen::VectorXd v = Eigen::VectorXd::Zero(3+moma_param.dof_num);
            
            bool moma_succ = false;
            double moma_path = 0.0;
            double moma_duration = 0.0;
            float moma_optim_time = 0.0;
            MomaTraj moma_traj;
            {
                ros::Time start_time = ros::Time::now();
                std::tie(moma_succ, moma_traj) = planMomaParallel(start_state, end_state, v);
                moma_duration = (ros::Time::now() - start_time).toSec();
                if(moma_succ) {
                    num_moma_succ++;
                    moma_path = moma_traj.getTotalDuration();
                }
            }

            if (moma_succ) {
                comparison_num++;

                // moma
                mean_moma_path_length += (moma_path - mean_moma_path_length) / comparison_num;
                mean_moma_duration += (moma_duration - mean_moma_duration) / comparison_num;
            }
        }

        PRINT_GREEN("[Planner] benchmark done.");
        PRINT_GREEN("MOMA:\t"       << num_moma_succ    << "/" << num_plan  << "\tAvg. Plan time: " << mean_moma_duration   << "ms\t Avg. Path Length: " << mean_moma_path_length   << "ms" << std::endl);

        PRINT_GREEN("Comparison:\tNum: " << comparison_num << std::endl);
        return;
    }
    
    void Planner::safeCallback(void *obj)
    {
        Planner *tsvr = reinterpret_cast<Planner *>(obj);
        while (true)
        {
            ros::Time start_time = ros::Time::now();
            if (tsvr->has_goal && tsvr->has_traj && tsvr->is_safe && (!tsvr->in_plan) )
            {
                Eigen::VectorXd temp_state = Eigen::VectorXd::Zero(10);
                std::vector<Eigen::Vector4d> min_dist_mani = tsvr->moma_param.getColliPts(temp_state);
                double res = 0.01;
                for (double t=0.0; t<tsvr->end_traj.getTotalDuration(); t+=res)
                {
                    Eigen::VectorXd state = tsvr->end_traj.getState(t);
        
                    double d = 0.0;
                    tsvr->grid_map->getDistance2d(state.head(2), d);
                    if (d < tsvr->moma_param.chassis_colli_radius * 0.99)
                    {
                        tsvr->is_safe = false;
                        break;
                    }
                    std::vector<Eigen::Vector4d> mani_pts = tsvr->moma_param.getColliPts(state);
                    for (size_t i=0; i<mani_pts.size(); i++)
                    {
                        double d = 0.0;
                        tsvr->grid_map->getDistance3d(mani_pts[i].head(3), d);
                        if (d < min_dist_mani[i].w() * 0.99)
                        {
                            tsvr->is_safe = false;
                            break;
                        }
                    }
                    if (!tsvr->is_safe) break;
                }
            }

            int cmd_mm_num = 1000.0 / tsvr->mpc->ctrl_freq;
            std::chrono::milliseconds dura(max(cmd_mm_num - (int)((ros::Time::now() - start_time).toSec() * 1000), 1));
            std::this_thread::sleep_for(dura);
        }
    }

    void Planner::replanCallback(void *obj)
    {
        Planner *tsvr = reinterpret_cast<Planner *>(obj);
        while (true)
        {
            ros::Time start_time = ros::Time::now();
            
            if (tsvr->has_goal && tsvr->has_traj)
            {
                if ((tsvr->now_state.head(2)-tsvr->global_goal.head(2)).norm () < 0.5)
                {
                    tsvr->has_goal = false;
                    tsvr->has_traj = false;
                    if (tsvr->wps_list.size() > 1)
                    {
                        if ((tsvr->global_goal-tsvr->place_vec).norm() < 0.1 ||
                            (tsvr->global_goal-tsvr->pick_vec).norm() < 0.1)
                        {
                            while (!tsvr->mpc->atGoal())
                                ROS_DEBUG("Waiting reaching goal...");
                            // in
                            Eigen::Vector3d direct;
                            direct.head(2) = tsvr->now_state.head(2) + \
                                            0.1 * Eigen::Vector2d(cos(tsvr->now_state(2)), sin(tsvr->now_state(2)));
                            tsvr->mpc->setDirect(direct);
                            while (!tsvr->mpc->atGoal())
                                ROS_DEBUG("Waiting reaching goal...");
                            // pick or place
                            tsvr->gripper_open = !tsvr->gripper_open;
                            this_thread::sleep_for(chrono::milliseconds(1000));
                            // out
                            direct.head(2) = tsvr->now_state.head(2) - \
                                            1.0 * Eigen::Vector2d(cos(tsvr->now_state(2)), sin(tsvr->now_state(2)));
                            tsvr->mpc->setDirect(direct);
                            while (!tsvr->mpc->atGoal())
                                ROS_DEBUG("Waiting reaching goal...");
                            this_thread::sleep_for(chrono::milliseconds(1000));
                        }
                        
                        tsvr->wps_list.erase(tsvr->wps_list.begin());
                        tsvr->global_goal = tsvr->wps_list.front();
                        PRINT_GREEN("New goal: " << tsvr->global_goal.transpose());
                        tsvr->has_goal = true;
                        bool succ;
                        Eigen::VectorXd local_start = tsvr->now_state;
                        Eigen::VectorXd local_v = tsvr->now_dstate;
                        if (tsvr->planner_type.compare("moma") == 0)
                        {
                            tsvr->in_plan = true;
                            succ = tsvr->planMomaParallel(local_start, tsvr->global_goal, local_v).first;
                            tsvr->in_plan = false;
                        }
                        if (succ)
                        {
                            tsvr->global_traj = tsvr->end_traj;
                            tsvr->mpc->setTraj(tsvr->end_traj, 0.0);
                            tsvr->begin_time = ros::Time::now();
                            tsvr->has_traj = true;
                            tsvr->is_safe = true;
                        }
                    }
                }
                else
                {
                    if ((ros::Time::now() - tsvr->last_replan_time).toSec() > tsvr->replan_interval || \
                        ! tsvr->is_safe)
                    {
                        ros::Time plan_start_time = ros::Time::now();
                        Eigen::VectorXd local_goal;
                        Eigen::VectorXd local_start = tsvr->now_state;
                        Eigen::VectorXd local_v = tsvr->now_dstate;
                        {
                            double t = (ros::Time::now() - tsvr->last_replan_time).toSec() + tsvr->planning_budget;
                            local_start = tsvr->end_traj.getState(t);
                            local_v = tsvr->end_traj.getDState(t);
                        }
                        {
                            double t = (ros::Time::now() - tsvr->begin_time).toSec();
                            bool found = false;
                            for (; t<tsvr->global_traj.getTotalDuration(); t+=0.1)
                            {
                                Eigen::VectorXd state = tsvr->global_traj.getState(t);
                                if ((state.head(2)-local_start.head(2)).norm() > tsvr->planning_horizon)
                                {
                                    local_goal = state;
                                    found = true;
                                    break;
                                }
                            }
                            if (!found)
                                local_goal = tsvr->global_goal;
                        }

                        PRINT_GREEN("local goal: " << local_goal.transpose());

                        tsvr->in_plan = true;
                        if (tsvr->planMomaParallel(local_start, local_goal, local_v).first)
                        {
                            tsvr->is_safe = true;
                            while (ros::Time::now() - plan_start_time < ros::Duration(tsvr->planning_budget)) {;}
                            // planning_budget = (ros::Time::now() - start_time).toSec();
                            double wait_time = (ros::Time::now() - plan_start_time).toSec() - tsvr->planning_budget;
                            PRINT_GREEN("Wait time = "<<wait_time<<" s.");
                            tsvr->mpc->setTraj(tsvr->end_traj, std::max(0.0, wait_time));
                            tsvr->begin_time = ros::Time::now();
                            tsvr->last_replan_time = ros::Time::now();
                            tsvr->has_traj = true;
                        }
                        tsvr->in_plan = false;
                    }
                }
            }

            int cmd_mm_num = 1000.0 / tsvr->mpc->ctrl_freq;
            std::chrono::milliseconds dura(max(cmd_mm_num - (int)((ros::Time::now() - start_time).toSec() * 1000), 1));
            std::this_thread::sleep_for(dura);
        }
        return;
    }

    void Planner::rcvStateCallBack(const fake_moma::MomaStatePtr msg)
    {
        has_odom = true;
        now_state[0] = msg->chassis_odom.pose.pose.position.x;
        now_state[1] = msg->chassis_odom.pose.pose.position.y;
        double ori_z = msg->chassis_odom.pose.pose.orientation.z;
        double ori_w = msg->chassis_odom.pose.pose.orientation.w;
        now_state[2] = atan2(2.0*ori_z*ori_w, 
                             2.0*ori_w*ori_w-1.0);
        now_dstate[0] = msg->chassis_odom.twist.twist.linear.x;
        now_dstate[1] = msg->chassis_odom.twist.twist.angular.z;
        // now_dstate[0] = 0.0;
        for (size_t i=0; i<moma_param.dof_num; i++)
        {
            now_state[3+i] = msg->arm_odom[i].twist.twist.linear.x;
            now_dstate[3+i] = msg->arm_odom[i].twist.twist.angular.z;
        }
        return;
    }

    std::vector<Eigen::VectorXd> Planner::planOmpls(const Eigen::VectorXd& start, const Eigen::VectorXd& end, const Eigen::VectorXd& start_v) const
    {
        // front end
        std::vector<Eigen::VectorXd> path;
        // ompls
        PRINT_GREEN("\n[Planner] Begin OMPL planning...");
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
        ompl_planner->planRRT(start, end, path);

        return path;
    }

    std::pair<bool, MomaTraj> Planner::planMomaParallel(const Eigen::VectorXd& start, 
                                    const Eigen::VectorXd& end, 
                                    const Eigen::VectorXd& start_v)
    {
        bool succ = false;                              // flag for successful optimization
        bool _critical = false;                         // whether to use critical map
        std::vector<Eigen::Vector4d> colors;            // colors of prm paths
        std::vector<std::pair<bool, MomaTraj>> results; // storing results of optimization
        std::vector<std::vector<Eigen::VectorXd>> front_paths;       // storing pre-optimized paths
        MomaTraj ret_traj;
        do {
            // start first trial with non-critical
            topo_select_paths.clear();
            {
                list<GraphNode::Ptr> graph;
                vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths;
                Eigen::Vector3d topo_start(start(0), start(1), 0.0);
                Eigen::Vector3d topo_end(end(0), end(1), 0.0);
                std::vector<Eigen::Vector3d> start_pts, end_pts;
                start_pts.push_back(topo_start);
                end_pts.push_back(topo_end);
                topo_prm->findTopoPaths(topo_start, topo_end, start_pts, end_pts, graph,
                                       raw_paths, filtered_paths, topo_select_paths, _critical);
                if (!_critical) {
                    auto jps_result = graph_search->plan2dJPS(start.head(2), end.head(2), moma_param.chassis_colli_radius+0.1);
                    if (!jps_result.empty())
                    {
                        std::vector<Eigen::Vector3d> jps3_res;
                        for (size_t i = 0; i < jps_result.size(); ++i)
                            jps3_res.push_back(Eigen::Vector3d(jps_result[i].x(), jps_result[i].y(), 0.0));
                        topo_select_paths.push_back(jps3_res);
                    }
                }
                
                if (topo_select_paths.empty())
                    return std::make_pair(false, ret_traj);
        
                if(topo_select_paths.size() > traj_opters.size()) throw std::runtime_error("Too many paths to optimize");
                
                colors = vis_prm_paths();
        
                PRINT_GREEN("[MOMA] Start optimization");
        
                results.resize(topo_select_paths.size());
                front_paths.resize(topo_select_paths.size());
                for (auto res : results) { res.first = false; }
                
                std::promise<bool> promise_succ;
                auto future_succ = promise_succ.get_future();

                boost::mutex mtx;
                boost::condition_variable cv_first; // for the first successful thread
                boost::condition_variable cv_all;   // for all threads to finish
                std::atomic_flag rdy_flag = ATOMIC_FLAG_INIT;
                std::atomic<int> completed_threads{0};
                auto worker = [this, &results, &front_paths, &mtx, &promise_succ, &cv_first, &cv_all, &completed_threads, &rdy_flag] (
                    int idx, 
                    std::vector<Eigen::Vector3d>& topo_path, 
                    const Eigen::VectorXd& start, 
                    const Eigen::VectorXd& end, 
                    const Eigen::VectorXd& start_v)
                {
                    ros::Time start_time = ros::Time::now();
                    std::vector<Eigen::Vector2d> in_path;
                    for(auto &wp : topo_path)
                        in_path.push_back(wp.head(2));
                    auto dense_result = graph_search->getDensePath(in_path, 1.414, start(2), end(2), 
                        moma_param.max_v, moma_param.max_w);
                        
                    boost::this_thread::interruption_point();

                    bool _succ = false;
                    do
                    {
                        if (!mc_rrtsers[idx]->plan(start, end, dense_result, front_paths[idx]) || front_paths[idx].empty())
                        {
                            _succ = false;
                            PRINT_RED("MCRRT fail.");
                            break;
                        }
                        boost::this_thread::interruption_point();
                        Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(10, 2);
                        Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(10, 2);
                        boundary_vel.col(0) = start_v;
    
                        _succ = 
                            this->traj_opters[idx]->optimizeTraj(front_paths[idx], boundary_vel, boundary_acc)
                            && this->traj_opters[idx]->printConstraintsSituations(traj_opters[idx]->getTraj())
                            && this->traj_opters[idx]->getTraj().is_init;
                        
                    } while(false);
                    
                    results[idx].first = _succ;
                    if(_succ) results[idx].second = this->traj_opters[idx]->getTraj();
                    // results[idx] = std::make_pair(_succ, this->traj_opters[idx]->getTraj());
                    
                    if (_succ && !rdy_flag.test_and_set()) {
                        // boost::lock_guard<boost::mutex> lock(mtx);
                        // if (!first_success.exchange(true)) {  // Atomic check-and-set
                        //     cv_first.notify_all();  // Notify all waiters
                        // }
                        promise_succ.set_value(true);
                    }
                    // MomaTraj traj = this->traj_opters[idx]->getTraj();
                    // PRINT_RED("[Thread] Successful optimization with duration: " << traj.getTotalDuration() << std::endl);
                    // promise_traj.set_value(traj);
                    // promise_traj.set_value(this->traj_opters[idx]->getTraj());
                    // optim_time = (ros::Time::now() - start_time).toSec() * 1000.0;
                    // try {
                    // } catch (boost::thread_interrupted&) {
        
                    // } catch (...) {
        
                    // }
                    ros::Time end_time = ros::Time::now();
                    PRINT_GREEN("[Thread] ID: " << idx << " Optimization time: " << (end_time - start_time).toSec() * 1000.0 << " ms");
                    {
                        PRINT_GREEN("[Threads] Thread " << completed_threads+1 << " / " << topo_select_paths.size() << " completed");
                        boost::lock_guard<boost::mutex> lock(mtx);
                        if(++completed_threads == topo_select_paths.size()){
                            if(!rdy_flag.test_and_set()) promise_succ.set_value(true);
                            PRINT_GREEN("[Threads] All threads completed");
                            cv_all.notify_all();
                        }
                    }
                    
                };
                
                PRINT_YELLOW("[Threads] Starting " << topo_select_paths.size() << " threads");
                boost::thread_group threads;
                for (size_t i = 0; i < topo_select_paths.size(); ++i)
                    threads.create_thread(std::bind(
                        worker, i, topo_select_paths[i], start, end, start_v
                    ));
                // bool optSucc = future_traj.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
                // threads.interrupt_all();
                // threads.join_all();
                
                // === wait for first successful thread ===
                future_succ.wait();
                
                bool timeout; // indicate early termination of threads
                {
                    boost::unique_lock<boost::mutex> lock(mtx);
                    
                    PRINT_RED("[Threads] Waiting for First Successful Optimization");
                    // cv_first.wait(lock, [&]() { return first_success.load(); } );
                    // while (!first_success) {
                    //     cv_first.wait(lock);
                
                    // === wait additional 100ms for other threads to finish ===
                    while (completed_threads < topo_select_paths.size()){
                        PRINT_RED("[Threads] Waiting for All Threads");
                        if(timeout = 
                            boost::cv_status::timeout == cv_all.wait_for(lock, boost::chrono::milliseconds(100))
                        )
                            break;
                    }
                }
                threads.interrupt_all();
                threads.join_all();
                
                if(timeout) {
                    PRINT_YELLOW("[Threads] Timeout in waiting threads");
                    PRINT_YELLOW("[Threads] " << completed_threads << " / " << topo_select_paths.size() << " completed");
                }
                
                for(auto &res : results) succ = succ || res.first;
            }
            _critical = true;
            if (!succ) PRINT_YELLOW("Non-Critical optimization failed, try critical optimization");
        } while(!succ && !_critical);
        
        // ros::Time t1 = ros::Time::now();
        // vector<thread> optimize_threads;
        // parallel_ends.clear();
        // parallel_ends.resize(topo_select_paths.size());
        // for (size_t i = 0; i < topo_select_paths.size(); ++i) 
        //     optimize_threads.emplace_back(&Planner::optMomaOnce, this, topo_select_paths[i], i, start, end, start_v);
        // for (size_t i = 0; i < topo_select_paths.size(); ++i) optimize_threads[i].join();
        // optim_time = (ros::Time::now() - t1).toSec() * 1000.0;
        // PRINT_GREEN("[MOMA] End optimization");
        bool ompl_succ = false; // flag for OMPL optimization success
        if (!succ) {
        do {
            auto ompl_path = planOmpls(start, end, start_v);
            if (ompl_path.empty()) break; // OMPL failed
            Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(10, 2);
            Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(10, 2);
            boundary_vel.col(0) = start_v;
            if (!this->traj_opters[0]->optimizeTraj(ompl_path, boundary_vel, boundary_acc)
                || !this->traj_opters[0]->printConstraintsSituations(traj_opters[0]->getTraj())
                || !this->traj_opters[0]->getTraj().is_init
            ) break; // OMPL optimization failed
            end_traj = traj_opters[0]->getTraj();
            succ = true;
            results.resize(1);
            results[0].first = true;
            results[0].second = end_traj;
            ompl_succ = true;
        } while (false);
        }

        if (succ)
        {
            PRINT_GREEN("[planner]: First successful optimization!");

            int shortest_idx = -1;
            int idx = -1;
            for(auto &res : results) {
                idx++;
                if(!res.first) continue;
                if(shortest_idx == -1) shortest_idx = idx;
                double traj_duration = res.second.getTotalDuration();
                if(traj_duration < results[shortest_idx].second.getTotalDuration())
                    shortest_idx = idx;
            }
            PRINT_YELLOW("Shortest path index: " << shortest_idx+1);
            ret_traj = end_traj = results[shortest_idx].second;
            vis_prm = topo_select_paths[shortest_idx];
            vis_prm_color = colors[shortest_idx];

            PRINT_YELLOW("Publishing MeshTraj");
            auto mesh_traj = toMeshMsg(end_traj);
            mesh_traj_pub.publish(mesh_traj);
            last_replan_time = ros::Time::now();

            vis_ee_traj(end_traj, plot_traj_ee, {226.0/255.0, 145.0/255.0, 53.0/255.0, 0.5});

            int res_idx = -1;
            for (auto &res : results) {
                res_idx++;
                if(!res.first) continue;
                
                bool shortest = res_idx == shortest_idx;
                float r,g,b,a;
                r = shortest? 1.0 : 0.5;
                g = shortest? 0.0 : 0.5;
                b = shortest? 0.0 : 0.5;
                a = shortest? 1.0 : 0.2;
                int nsample = shortest ? 16 : 16;
                MomaTraj traj = res.second;
                
                if(!ompl_succ) {
                    vis_isAvailable[res_idx] = true;
                    vis_front_paths[res_idx] = front_paths[res_idx];
                    vis_opt_paths[res_idx]   = traj;

                    // vis_path_mesh(traj, nsample, 
                    //     opt_traj_pub_list[res_idx], 
                    //     {r, g, b, a}, 
                    //     res_idx*1000 + 800);
                    
                    // vis_path_mesh(sparsifyPath(front_paths[res_idx], 0.5), 
                    //     front_traj_pub_list[res_idx], 
                    //     {0.0, 0.0, 1.0, 0.2}, 
                    //     res_idx*1000 + 900);
                }
            }
            if(ompl_succ) {
                for (size_t i = 0; i < vis_isAvailable.size(); ++i) vis_isAvailable[i] = false;
            }
            
            // end_traj = future_traj.get();
            // last_replan_time = ros::Time::now();
            // vis_whole_path(end_pub);
        }

        return std::make_pair(succ, ret_traj);
    }

    std::pair<bool, float> Planner::planMomaSequential(const Eigen::VectorXd& start, 
                                    const Eigen::VectorXd& end, 
                                    const Eigen::VectorXd& start_v)
    {
        bool succ = false;                              // flag for successful optimization
        bool _critical = false;                         // whether to use critical map
        std::vector<Eigen::Vector4d> colors;            // colors of prm paths
        std::vector<std::pair<bool, MomaTraj>> results; // storing results of optimization
        do {
            // start first trial with non-critical
            topo_select_paths.clear();
            {
                list<GraphNode::Ptr> graph;
                vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths;
                Eigen::Vector3d topo_start(start(0), start(1), 0.0);
                Eigen::Vector3d topo_end(end(0), end(1), 0.0);
                std::vector<Eigen::Vector3d> start_pts, end_pts;
                start_pts.push_back(topo_start);
                end_pts.push_back(topo_end);
                topo_prm->findTopoPaths(topo_start, topo_end, start_pts, end_pts, graph,
                                       raw_paths, filtered_paths, topo_select_paths, _critical);
                if (!_critical) { // for the first trial, use JPS as backup
                    auto jps_result = graph_search->plan2dJPS(start.head(2), end.head(2), moma_param.chassis_colli_radius+0.1);
                    if (!jps_result.empty())
                    {
                        std::vector<Eigen::Vector3d> jps3_res;
                        for (size_t i = 0; i < jps_result.size(); ++i)
                            jps3_res.push_back(Eigen::Vector3d(jps_result[i].x(), jps_result[i].y(), 0.0));
                        topo_select_paths.push_back(jps3_res);
                    }
                }
                
                if (topo_select_paths.empty())
                    return std::make_pair(false, 0.0);
        
                if(topo_select_paths.size() > traj_opters.size()) throw std::runtime_error("Too many paths to optimize");
                
                colors = vis_prm_paths();
        
                results.resize(topo_select_paths.size());
                for (auto res : results) { res.first = false; }
                
                auto worker = [this, &results] (
                    int idx, 
                    std::vector<Eigen::Vector3d>& topo_path, 
                    const Eigen::VectorXd& start, 
                    const Eigen::VectorXd& end, 
                    const Eigen::VectorXd& start_v)
                {
                    ros::Time start_time = ros::Time::now();
                    std::vector<Eigen::Vector2d> in_path;
                    for(auto &wp : topo_path)
                        in_path.push_back(wp.head(2));
                    auto dense_result = graph_search->getDensePath(in_path, 1.414, start(2), end(2), 
                        moma_param.max_v, moma_param.max_w);
                        
                    std::vector<Eigen::VectorXd> full_path;
                    if (!mc_rrtsers[idx]->plan(start, end, dense_result, full_path) || full_path.empty())
                    {
                        PRINT_RED("MCRRT fail.");
                        return;
                    }
                    
                    Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(10, 2);
                    Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(10, 2);
                    boundary_vel.col(0) = start_v;

                    bool _succ = 
                        this->traj_opters[idx]->optimizeTraj(full_path, boundary_vel, boundary_acc)
                        && this->traj_opters[idx]->printConstraintsSituations(traj_opters[idx]->getTraj())
                        && this->traj_opters[idx]->getTraj().is_init;
                        
                    results[idx] = std::make_pair(_succ, this->traj_opters[idx]->getTraj());
                };
                
                // PRINT_YELLOW("[Threads] Starting " << topo_select_paths.size() << " threads");
                // boost::thread_group threads;
                // for (size_t i = 0; i < topo_select_paths.size(); ++i)
                //     threads.create_thread(std::bind(
                //         worker, i, topo_select_paths[i], start, end, start_v
                //     ));
                // bool optSucc = future_traj.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
                // threads.interrupt_all();
                // threads.join_all();

                for (size_t i = 0; i < topo_select_paths.size(); ++i)
                    worker(i, topo_select_paths[i], start, end, start_v);
                
                int n_succ = 0;
                for(auto &res : results) {
                    if(res.first) n_succ++;
                    succ = succ || res.first;
                }
                PRINT_YELLOW("[Threads] " << n_succ << " / " << topo_select_paths.size() << " succeed");
            }
            _critical = true;
            if (!succ) PRINT_YELLOW("Non-Critical optimization failed, try critical optimization");
        } while(!succ && !_critical);

        if (!succ) {
        do {
            auto ompl_path = planOmpls(start, end, start_v);
            if (ompl_path.empty()) break; // OMPL failed
            Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(10, 2);
            Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(10, 2);
            boundary_vel.col(0) = start_v;
            if (!this->traj_opters[0]->optimizeTraj(ompl_path, boundary_vel, boundary_acc)
                || !this->traj_opters[0]->printConstraintsSituations(traj_opters[0]->getTraj())
                || !this->traj_opters[0]->getTraj().is_init
            ) break; // OMPL optimization failed
            end_traj = traj_opters[0]->getTraj();
            succ = true;
            results.resize(1);
            results[0].first = true;
            results[0].second = end_traj;
        } while (false);
        }

        if (succ)
        {
            PRINT_GREEN("[planner]: First successful optimization!");

            int shortest_idx = -1;
            int idx = -1;
            for(auto &res : results) {
                idx++;
                if(!res.first) continue;
                if(shortest_idx == -1) shortest_idx = idx;
                double traj_duration = res.second.getTotalDuration();
                if(traj_duration < results[shortest_idx].second.getTotalDuration())
                    shortest_idx = idx;
            }
            PRINT_YELLOW("Shortest path index: " << shortest_idx);
            end_traj = results[shortest_idx].second;

            int res_idx = -1;
            for (auto &res : results) {
                res_idx++;
                if(!res.first) continue;
                
                bool shortest = res_idx == shortest_idx;
                float r,g,b,a;
                r = shortest? 1.0 : 0.5;
                g = shortest? 0.0 : 0.5;
                b = shortest? 0.0 : 0.5;
                a = shortest? 1.0 : 0.2;
                int nsample = shortest ? 16 : 16;
                
                MomaTraj traj = res.second;
                vis_path_mesh(traj, nsample, 
                    opt_traj_pub_list[res_idx], 
                    {r, g, b, a}, 
                    res_idx*1000 + 800);
            }
            
            // end_traj = future_traj.get();
            // last_replan_time = ros::Time::now();
            // vis_whole_path(end_pub);
        }

        return std::make_pair(succ, 0.0);
    }

    
    std::pair<bool, float> Planner::planMomaNonTOPO(const Eigen::VectorXd& start, 
                                    const Eigen::VectorXd& end, 
                                    const Eigen::VectorXd& start_v)
    {
        bool succ = false;                              // flag for successful optimization
        bool _critical = false;                         // whether to use critical map
        std::vector<Eigen::Vector4d> colors;            // colors of prm paths
        std::vector<std::pair<bool, MomaTraj>> results; // storing results of optimization
        do {
            // start first trial with non-critical
            topo_select_paths.clear();
            {
                // list<GraphNode::Ptr> graph;
                // vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths;
                // Eigen::Vector3d topo_start(start(0), start(1), 0.0);
                // Eigen::Vector3d topo_end(end(0), end(1), 0.0);
                // std::vector<Eigen::Vector3d> start_pts, end_pts;
                // start_pts.push_back(topo_start);
                // end_pts.push_back(topo_end);
                // topo_prm->findTopoPaths(topo_start, topo_end, start_pts, end_pts, graph,
                //                        raw_paths, filtered_paths, topo_select_paths, _critical);
                if (!_critical) {
                    auto jps_result = graph_search->plan2dJPS(start.head(2), end.head(2), moma_param.chassis_colli_radius+0.1);
                    if (!jps_result.empty())
                    {
                        std::vector<Eigen::Vector3d> jps3_res;
                        for (size_t i = 0; i < jps_result.size(); ++i)
                            jps3_res.push_back(Eigen::Vector3d(jps_result[i].x(), jps_result[i].y(), 0.0));
                        topo_select_paths.push_back(jps3_res);
                    }
                }
                
                if (topo_select_paths.empty())
                    return std::make_pair(false, 0.0);
        
                if(topo_select_paths.size() > traj_opters.size()) throw std::runtime_error("Too many paths to optimize");
                
                colors = vis_prm_paths();
        
                PRINT_GREEN("[MOMA] Start optimization");
        
                results.resize(topo_select_paths.size());
                for (auto res : results) { res.first = false; }
                
                std::promise<bool> promise_succ;
                auto future_succ = promise_succ.get_future();

                boost::mutex mtx;
                boost::condition_variable cv_first; // for the first successful thread
                boost::condition_variable cv_all;   // for all threads to finish
                std::atomic_flag rdy_flag = ATOMIC_FLAG_INIT;
                std::atomic<int> completed_threads{0};
                auto worker = [this, &results, &mtx, &promise_succ, &cv_first, &cv_all, &completed_threads, &rdy_flag] (
                    int idx, 
                    std::vector<Eigen::Vector3d>& topo_path, 
                    const Eigen::VectorXd& start, 
                    const Eigen::VectorXd& end, 
                    const Eigen::VectorXd& start_v)
                {
                    ros::Time start_time = ros::Time::now();
                    std::vector<Eigen::Vector2d> in_path;
                    for(auto &wp : topo_path)
                        in_path.push_back(wp.head(2));
                    auto dense_result = graph_search->getDensePath(in_path, 1.414, start(2), end(2), 
                        moma_param.max_v, moma_param.max_w);
                        
                    boost::this_thread::interruption_point();
                    std::vector<Eigen::VectorXd> full_path;

                    bool _succ = false;
                    do
                    {
                        if (!mc_rrtsers[idx]->plan(start, end, dense_result, full_path) || full_path.empty())
                        {
                            _succ = false;
                            PRINT_RED("MCRRT fail.");
                            break;
                        }
                        boost::this_thread::interruption_point();
                        Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(10, 2);
                        Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(10, 2);
                        boundary_vel.col(0) = start_v;
    
                        _succ = 
                            this->traj_opters[idx]->optimizeTraj(full_path, boundary_vel, boundary_acc)
                            && this->traj_opters[idx]->printConstraintsSituations(traj_opters[idx]->getTraj())
                            && this->traj_opters[idx]->getTraj().is_init;
                        
                    } while(false);
                    
                    results[idx].first = _succ;
                    if(_succ) results[idx].second = this->traj_opters[idx]->getTraj();
                    
                    if (_succ && !rdy_flag.test_and_set())
                        promise_succ.set_value(true);
                    
                    ros::Time end_time = ros::Time::now();
                    PRINT_GREEN("[Thread] ID: " << idx << " Optimization time: " << (end_time - start_time).toSec() * 1000.0 << " ms");
                    {
                        PRINT_GREEN("[Threads] Thread " << completed_threads+1 << " / " << topo_select_paths.size() << " completed");
                        boost::lock_guard<boost::mutex> lock(mtx);
                        if(++completed_threads == topo_select_paths.size()){
                            if(!rdy_flag.test_and_set()) promise_succ.set_value(true);
                            PRINT_GREEN("[Threads] All threads completed");
                            cv_all.notify_all();
                        }
                    }
                    
                };
                
                PRINT_YELLOW("[Threads] Starting " << topo_select_paths.size() << " threads");
                boost::thread_group threads;
                for (size_t i = 0; i < topo_select_paths.size(); ++i)
                    threads.create_thread(std::bind(
                        worker, i, topo_select_paths[i], start, end, start_v
                    ));
                
                // === wait for first successful thread ===
                future_succ.wait();
                
                bool timeout; // indicate early termination of threads
                {
                    boost::unique_lock<boost::mutex> lock(mtx);
                    
                    PRINT_RED("[Threads] Waiting for First Successful Optimization");
                
                    // === wait additional 100ms for other threads to finish ===
                    while (completed_threads < topo_select_paths.size()){
                        PRINT_RED("[Threads] Waiting for All Threads");
                        if(timeout = 
                            boost::cv_status::timeout == cv_all.wait_for(lock, boost::chrono::milliseconds(100))
                        )
                            break;
                    }
                }
                threads.interrupt_all();
                threads.join_all();
                
                if(timeout) {
                    PRINT_YELLOW("[Threads] Timeout in waiting threads");
                    PRINT_YELLOW("[Threads] " << completed_threads << " / " << topo_select_paths.size() << " completed");
                }
                
                for(auto &res : results) succ = succ || res.first;
            }
            _critical = true;
            if (!succ) PRINT_YELLOW("Non-Critical optimization failed, try critical optimization");
        } while(false);
        
        if (!succ) {
        do {
            auto ompl_path = planOmpls(start, end, start_v);
            if (ompl_path.empty()) break; // OMPL failed
            Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(10, 2);
            Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(10, 2);
            boundary_vel.col(0) = start_v;
            if (!this->traj_opters[0]->optimizeTraj(ompl_path, boundary_vel, boundary_acc)
                || !this->traj_opters[0]->printConstraintsSituations(traj_opters[0]->getTraj())
                || !this->traj_opters[0]->getTraj().is_init
            ) break; // OMPL optimization failed
            end_traj = traj_opters[0]->getTraj();
            succ = true;
            results.resize(1);
            results[0].first = true;
            results[0].second = end_traj;
        } while (false);
        }

        if (succ)
        {
            PRINT_GREEN("[planner]: First successful optimization!");

            int shortest_idx = -1;
            int idx = -1;
            for(auto &res : results) {
                idx++;
                if(!res.first) continue;
                if(shortest_idx == -1) shortest_idx = idx;
                double traj_duration = res.second.getTotalDuration();
                if(traj_duration < results[shortest_idx].second.getTotalDuration())
                    shortest_idx = idx;
            }
            PRINT_YELLOW("Shortest path index: " << shortest_idx);
            end_traj = results[shortest_idx].second;

            int res_idx = -1;
            for (auto &res : results) {
                res_idx++;
                if(!res.first) continue;
                
                bool shortest = res_idx == shortest_idx;
                float r,g,b,a;
                r = shortest? 1.0 : 0.5;
                g = shortest? 0.0 : 0.5;
                b = shortest? 0.0 : 0.5;
                a = shortest? 1.0 : 0.2;
                int nsample = shortest ? 16 : 16;
                
                MomaTraj traj = res.second;
                vis_path_mesh(traj, nsample, 
                    opt_traj_pub_list[res_idx], 
                    {r, g, b, a}, 
                    res_idx*1000 + 800);
            }
        }
        return std::make_pair(succ, 0.0);
    }

    bool Planner::optMomaOnce(const std::vector<Eigen::Vector3d>& topo_path, int idx, 
                            const Eigen::VectorXd& start, const Eigen::VectorXd& end,
                            const Eigen::VectorXd& start_v)
    {
        std::vector<Eigen::Vector2d> in_path;
        for (size_t i = 0; i < topo_path.size(); ++i)
            in_path.push_back(topo_path[i].head(2));
        auto dense_result = graph_search->getDensePath(in_path, 1.414, start(2), end(2), 
                                                       moma_param.max_v, moma_param.max_w);
        std::vector<Eigen::VectorXd> full_path;
        if (!mc_rrtsers[idx]->plan(start, end, dense_result, full_path))
        {
            PRINT_RED("MCRRTs fail, idx = "<<idx);
            return false;
        }
        if (full_path.empty())
            return false;
        Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(3+moma_param.dof_num, 2);
        Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(3+moma_param.dof_num, 2);
        boundary_vel.col(0) = start_v;
        if (!traj_opters[idx]->optimizeTraj(full_path, boundary_vel, boundary_acc)
            || !traj_opters[idx]->printConstraintsSituations(traj_opters[idx]->getTraj()) 
            )
            return false;
        else
        {
            parallel_ends[idx] = traj_opters[idx]->getTraj();
            return true;
        }
        return true;
    }

    bool Planner::optDenseOnce(const std::vector<Eigen::VectorXd>& full_path, int idx, 
                            const Eigen::VectorXd& start, const Eigen::VectorXd& end,
                            const Eigen::VectorXd& start_v)
    {
        Eigen::MatrixXd boundary_vel = Eigen::MatrixXd::Zero(3+moma_param.dof_num, 2);
        Eigen::MatrixXd boundary_acc = Eigen::MatrixXd::Zero(3+moma_param.dof_num, 2);
        boundary_vel.col(0) = start_v;
        if (!traj_opters[idx]->optimizeTraj(full_path, boundary_vel, boundary_acc)
            || !traj_opters[idx]->printConstraintsSituations(traj_opters[idx]->getTraj()) 
            )
            return false;
        else
        {
            parallel_ends[idx] = traj_opters[idx]->getTraj();
            return true;
        }
        return true;
    }


    std::vector<Eigen::Vector4d> Planner::vis_prm_paths()
    {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker line_strip, delet_p;

        delet_p.action = visualization_msgs::Marker::DELETEALL;
        delet_p.id = 0;
        markers.markers.push_back(delet_p);

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.header.frame_id = "world";
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.10;
        line_strip.scale.y = 0.10;
        line_strip.scale.z = 0.10;
        line_strip.color.a = 1.0;

        std::vector<Eigen::Vector4d> colors;

        for (size_t i=0; i<topo_select_paths.size(); i++)
        {
            Eigen::Vector4d color = {
                1.0 * (rand() % 1000) / 1000.0, 
                1.0 * (rand() % 1000) / 1000.0, 
                1.0 * (rand() % 1000) / 1000.0, 
                1.0};
            colors.push_back(color);
            line_strip.header.stamp = ros::Time::now();
            line_strip.id = i + 1;
            line_strip.color.r = color[0];
            line_strip.color.g = color[1];
            line_strip.color.b = color[2];
            line_strip.points.clear();
            for (size_t j=0; j<topo_select_paths[i].size(); j++)
            {
                geometry_msgs::Point pt;
                pt.x = topo_select_paths[i][j].x();
                pt.y = topo_select_paths[i][j].y();
                pt.z = 0.0;
                line_strip.points.push_back(pt);
            }
            markers.markers.push_back(line_strip);
        }
        prm_pub.publish(markers);
        return colors;
    }

    void Planner::vis_path(const std::vector<Eigen::VectorXd>& path, ros::Publisher& puber, vector<float> rgba, vector<int> ids)
    {
        if (path.empty())
            return;

        visualization_msgs::Marker line_strip, arrow, text;
        arrow.header.frame_id = line_strip.header.frame_id = text.header.frame_id = "world";
        arrow.header.stamp = line_strip.header.stamp = text.header.stamp = ros::Time::now();
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        arrow.type = visualization_msgs::Marker::ARROW;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.id = 1886666;
        text.action = visualization_msgs::Marker::ADD;
        text.scale.z = 0.1;
        text.color.a = 1.0;
        line_strip.id = 10086;
        arrow.scale.x = 0.03;
        arrow.scale.y = 0.05;
        arrow.color.a = 1.0;
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.pose.orientation.w = 1.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.03;
        line_strip.scale.y = 0.03;
        line_strip.scale.z = 0.03;
        line_strip.color.a = 1.0;
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;

        visualization_msgs::MarkerArray array_msg;
        visualization_msgs::Marker p;
        p.action = visualization_msgs::Marker::DELETEALL;
        p.id = 0;
        array_msg.markers.push_back(p);
        for (size_t i=0; i<path.size(); i++)
        {
            visualization_msgs::MarkerArray node_array = moma_param.getColliCylinderArray(path[i]);
            size_t array_size = node_array.markers.size();
            for (size_t j=0; j<array_size; j++)
            {
                node_array.markers[j].id = i*array_size+j;
                node_array.markers[j].color.a = rgba[3];
                node_array.markers[j].color.r = rgba[0];
                node_array.markers[j].color.g = rgba[1];
                node_array.markers[j].color.b = rgba[2];
                array_msg.markers.push_back(node_array.markers[j]);
            }
            geometry_msgs::Point pt;
            pt.x = path[i].x();
            pt.y = path[i].y();
            pt.z = 0.0;
            line_strip.points.push_back(pt);
            geometry_msgs::Point pt_arrow;
            pt_arrow.x = path[i].x() + moma_param.chassis_colli_radius*cos(path[i].z());
            pt_arrow.y = path[i].y() + moma_param.chassis_colli_radius*sin(path[i].z());
            arrow.points.clear();
            arrow.points.push_back(pt);
            arrow.points.push_back(pt_arrow);
            arrow.id = line_strip.id + i + 1;
            array_msg.markers.push_back(arrow);
            text.color.r = 0.0;
            arrow.color.b = 0.0;
            for (size_t j=0; j<ids.size(); j++)
            {
                if (ids[j] == (int)i)
                {
                    text.color.r = 1.0;
                    arrow.color.b = 1.0;
                    break;
                }
            }
            text.text = std::to_string(i);
            text.id = text.id + 1;
            text.pose.orientation.w = 1.0;
            text.pose.position = node_array.markers.back().pose.position;
            text.pose.position.z = text.pose.position.z + 0.1;
            array_msg.markers.push_back(text);
        }
        array_msg.markers.push_back(line_strip);
        puber.publish(array_msg);
        return;
    }

    void Planner::vis_whole_path(ros::Publisher& pub)
    {
        std::vector<Eigen::VectorXd> end_path;
        nav_msgs::Path car_traj;
        for (double t=0.0; t<end_traj.getTotalDuration(); t+=0.1)
        {
            Eigen::VectorXd state = end_traj.getState(t);
            end_path.push_back(state);
            car_traj.header.frame_id = "world";
            car_traj.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped gp;
            gp.pose.position.x = state.x();
            gp.pose.position.y = state.y();
            gp.pose.position.z = 0.0;
            gp.pose.orientation.w = cos(state.z()/2.0);
            gp.pose.orientation.x = 0.0;
            gp.pose.orientation.y = 0.0;
            gp.pose.orientation.z = sin(state.z()/2.0);
            car_traj.poses.push_back(gp);
        }
        if (pub == end_pub)
            car_traj_pub.publish(car_traj);
        vis_path(end_path, pub, {1.0, 0.0, 0.0, 0.15});
    }

    Eigen::Quaterniond euler2rotation(double r, double p, double y)
    {
        return Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) 
                * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) 
                * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
    }

    Eigen::Quaterniond euler2rotation(Eigen::Vector3d rpy)
    {
        return Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) 
                * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) 
                * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
    }

    void Planner::vis_path_mesh(const std::vector<Eigen::VectorXd>& path, ros::Publisher& pub, vector<float> rgba, int id)
    {
        if(path.empty()) return;

        visualization_msgs::MarkerArray moma_marker;
        
        visualization_msgs::Marker delet_p;
        delet_p.action = visualization_msgs::Marker::DELETEALL;
        delet_p.id = 9871;
        moma_marker.markers.push_back(delet_p);
        pub.publish(moma_marker);

        for(auto wp : path) {
            visualization_msgs::Marker diff_marker;
            {
                diff_marker.header.frame_id = "world";
                diff_marker.id = id++;
                diff_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                diff_marker.action = visualization_msgs::Marker::ADD;
                diff_marker.mesh_resource = "package://fake_moma/meshes/tracer.dae";
                // pose
                diff_marker.pose.position.x = wp[0];
                diff_marker.pose.position.y = wp[1];
                diff_marker.pose.position.z = moma_param.chassis_height;
                // orientation
                Eigen::Quaterniond q = euler2rotation(M_PI_2, wp[2], 0.0);
                diff_marker.pose.orientation.w = q.w();
                diff_marker.pose.orientation.x = q.x();
                diff_marker.pose.orientation.y = q.y();
                diff_marker.pose.orientation.z = q.z();
                // color
                diff_marker.color.a = rgba[3];
                diff_marker.color.r = rgba[0];
                diff_marker.color.g = rgba[1];
                diff_marker.color.b = rgba[2];
                // scale
                diff_marker.scale.x = 1.0;
                diff_marker.scale.y = 1.0;
                diff_marker.scale.z = 1.0;
            }
            moma_marker.markers.push_back(diff_marker);

            Eigen::Vector3d ap(
                diff_marker.pose.position.x, 
                diff_marker.pose.position.y, 
                diff_marker.pose.position.z
            );

            Eigen::Quaterniond aq(cos(wp[2]/2.0), 0.0, 0.0, sin(wp[2]/2.0));
            ap += aq.matrix() * moma_param.relative_t;
            aq = aq.matrix() * moma_param.relative_R;

            //link0
            visualization_msgs::Marker link_marker;
            {
                link_marker.header.frame_id = "world";
                link_marker.id = id++;
                link_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                link_marker.action = visualization_msgs::Marker::ADD;
                link_marker.pose.position.x = ap.x();
                link_marker.pose.position.y = ap.y();
                link_marker.pose.position.z = ap.z();
                link_marker.pose.orientation.w = aq.w();
                link_marker.pose.orientation.x = aq.x();
                link_marker.pose.orientation.y = aq.y();
                link_marker.pose.orientation.z = aq.z();
                link_marker.color.a = rgba[3];
                link_marker.color.r = rgba[0];
                link_marker.color.g = rgba[1];
                link_marker.color.b = rgba[2];
                link_marker.scale.x = 1.0;
                link_marker.scale.y = 1.0;
                link_marker.scale.z = 1.0;
                link_marker.mesh_resource = "package://fake_moma/meshes/link0.STL";
            }
            moma_marker.markers.push_back(link_marker);

            //link1-7
            for (size_t i = 0; i < moma_param.dof_num; i++)
            {
                visualization_msgs::Marker link_marker;
                {
                    link_marker.header.frame_id = "world";
                    link_marker.id = id++;
                    link_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                    link_marker.action = visualization_msgs::Marker::ADD;
                    ap += aq.matrix() * Eigen::Vector3d(0.0, 0.0, moma_param.link_length[i]);
                    link_marker.pose.position.x = ap.x();
                    link_marker.pose.position.y = ap.y();
                    link_marker.pose.position.z = ap.z();
                    aq = aq.matrix() * euler2rotation(moma_param.joint_offset.row(i))
                            * euler2rotation(moma_param.joint_dof_axis.row(i)*wp[i+3]);
                    link_marker.pose.orientation.w = aq.w();
                    link_marker.pose.orientation.x = aq.x();
                    link_marker.pose.orientation.y = aq.y();
                    link_marker.pose.orientation.z = aq.z();
                    link_marker.color.a = rgba[3];
                    link_marker.color.r = rgba[0];
                    link_marker.color.g = rgba[1];
                    link_marker.color.b = rgba[2];
                    link_marker.scale.x = 1.0;
                    link_marker.scale.y = 1.0;
                    link_marker.scale.z = 1.0;
                    link_marker.mesh_resource = "package://fake_moma/meshes/link"+std::to_string(i+1)+".STL";
                }
                moma_marker.markers.push_back(link_marker);
            }

            //gripper
            visualization_msgs::Marker gripper_marker;
            {
                gripper_marker.header.frame_id = "world";
                gripper_marker.id = id++;
                gripper_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                gripper_marker.action = visualization_msgs::Marker::ADD;
                gripper_marker.pose = moma_marker.markers.back().pose;
                gripper_marker.color.a = rgba[3];
                gripper_marker.color.r = rgba[0];
                gripper_marker.color.g = rgba[1];
                gripper_marker.color.b = rgba[2];
                gripper_marker.scale.x = 1.0;
                gripper_marker.scale.y = 1.0;
                gripper_marker.scale.z = 1.0;
                gripper_marker.mesh_resource = "package://fake_moma/meshes/gripper.dae";
            }
            moma_marker.markers.push_back(gripper_marker);
        }

        if (!path.empty()) pub.publish(moma_marker);
        
    }


    void Planner::vis_path_mesh(const MomaTraj& traj, int nsample, ros::Publisher& pub, vector<float> rgba, int id) {
        RowMatrixXd path_m = traj.sampleTimePoints(nsample);
        std::vector<Eigen::VectorXd> path;

        Eigen::VectorXd prev_state = path_m.row(0).head(10);
        Eigen::VectorXd end_state = path_m.row(path_m.rows()-1).head(10);
        path.push_back(prev_state);

        for (int i = 0; i < path_m.rows(); i++) {
            Eigen::VectorXd state = path_m.row(i).head(10);
            if ((state.head(2) - prev_state.head(2)).norm() > 0.5
            && (state.head(2) - end_state.head(2)).norm() > 0.5) {
                path.push_back(state);
                prev_state = state;
            }
            // path.push_back(path_m.row(i).head(10));
        }
        path.push_back(end_state);

        vis_path_mesh(path, pub, rgba, id);
    }

    Eigen::VectorXd Planner::generateRandomState(void) const {
        Eigen::VectorXd ret;
        ret.resize(3+moma_param.dof_num);
        // Eigen::Vector3d min_bound = grid_map->min_boundary;
        // Eigen::Vector3d max_bound = grid_map->max_boundary;

        // uniform_real_distribution<double> rand_x(min_bound(0)+2.0, max_bound(0)-2.0);
        // uniform_real_distribution<double> rand_y(min_bound(1)+2.0, max_bound(1)-2.0);
        // uniform_real_distribution<double> rand_theta(-M_PI, M_PI);
        // uniform_real_distribution<double> rand_q(0.0, 1.0);
        
        // eng = default_random_engine(42);
        // ret(0) = rand_x(eng);
        // ret(1) = rand_y(eng);
        // ret(2) = rand_theta(eng);
        // for (int i = 0; i < moma_param.dof_num; i++)
        //     ret(3+i) = (moma_param.joint_pos_limit_max(i) - moma_param.joint_pos_limit_min(i)) * rand_q(eng)
        //             + moma_param.joint_pos_limit_min(i);

        return ret;
    }

    std::vector<Eigen::VectorXd> Planner::sparsifyPath(const std::vector<Eigen::VectorXd>& path, double dist) const {
        std::vector<Eigen::VectorXd> ret;
        if (path.empty()) return ret;

        Eigen::VectorXd end_state = path.back();
        
        ret.push_back(path.front());
        for (size_t i = 1; i < path.size(); i++) {
            if ((path[i].head(2) - ret.back().head(2)).norm() > dist
                && (path[i].head(2) - end_state.head(2)).norm() > dist) {
                ret.push_back(path[i]);
            }
        }
        ret.push_back(end_state);
        return ret;
    }

    void Planner::timerCallback (const ros::TimerEvent& event) {

        bool vis_prm_available = false;
        for (size_t i = 0; i < vis_isAvailable.size(); i++) {
            vis_prm_available = vis_prm_available || vis_isAvailable[i];
            if (vis_isAvailable[i]) {
                vis_path_mesh(vis_opt_paths[i], 16, opt_traj_pub_list[i],   {226.0/255.0, 145.0/255.0, 53.0/255.0, 0.5}, i*800);
                vis_path_mesh(sparsifyPath(vis_front_paths[i], 0.5),   front_traj_pub_list[i], {0.0, 0.0, 1.0, 0.3}, i*900);
            } else {
                vis_path_mesh(std::vector<Eigen::VectorXd>(), opt_traj_pub_list[i], {1.0, 0.0, 0.0, 0.15}, i*800);
                vis_path_mesh(std::vector<Eigen::VectorXd>(), front_traj_pub_list[i], {1.0, 0.0, 0.0, 0.15}, i*900);
            }
        }

        if (vis_prm_available) {
            visualization_msgs::MarkerArray markers;
            visualization_msgs::Marker line_strip, delet_p;

            delet_p.action = visualization_msgs::Marker::DELETEALL;
            delet_p.id = 0;
            markers.markers.push_back(delet_p);

            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.header.frame_id = "world";
            line_strip.pose.orientation.w = 1.0;
            line_strip.scale.x = 0.10;
            line_strip.scale.y = 0.10;
            line_strip.scale.z = 0.10;
            line_strip.color.a = 1.0;

            line_strip.header.stamp = ros::Time::now();
            line_strip.id = 1;
            line_strip.color.r = vis_prm_color[0];
            line_strip.color.g = vis_prm_color[1];
            line_strip.color.b = vis_prm_color[2];
            line_strip.points.clear();
            for (size_t j=0; j<vis_prm.size(); j++)
            {
                geometry_msgs::Point pt;
                pt.x = vis_prm[j].x();
                pt.y = vis_prm[j].y();
                pt.z = 0.0;
                line_strip.points.push_back(pt);
            }
            markers.markers.push_back(line_strip);
            vis_prm_pub.publish(markers);
        }
    }

    void Planner::vis_ee_traj(const MomaTraj& traj, ros::Publisher& pub, vector<float> rgba) const {
        const int res = 200;
        const double intvl = traj.getTotalDuration() / res;
        double t = 0.0;

        visualization_msgs::Marker line_strip;
        {
            line_strip.header.frame_id = "world";
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "velocity_trajectory";
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = 2077;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.10;
            line_strip.scale.y = 0.10;
            line_strip.scale.z = 0.10;
        }

        std::vector<double> velocities;
        Eigen::Vector4d gripper_prev = moma_param.getColliPts(    
            traj.getState(0)
        ).back();

        for (size_t i = 0; i < res; i++) {
            Eigen::VectorXd state = traj.getState(t);
            Eigen::Vector4d gripper;
            gripper = moma_param.getColliPts(state).back();

            geometry_msgs::Point pt;
            pt.x = gripper (0);
            pt.y = gripper (1);
            pt.z = gripper (2);
            line_strip.points.push_back(pt);

            velocities.push_back((gripper.head(3) - gripper_prev.head(3)).norm() / intvl);
            t += intvl;
            gripper_prev = gripper;
        }
        velocities[0] = velocities[1]; // because the first velocity is not reliable

        double max_vel = *std::max_element(velocities.begin(), velocities.end());
        double min_vel = *std::min_element(velocities.begin(), velocities.end());
        // double avg_vel = std::accumulate(velocities.begin(), velocities.end(), 0.0) / velocities.size();


        std::cout << "max vel: " << max_vel << " min vel: " << min_vel << std::endl;
        for (size_t i = 0; i < res; i++){
            double vel = velocities[i];
            double r = (vel - min_vel) / (max_vel - min_vel);

            std_msgs::ColorRGBA color;
            {
                // Viridis
                // color.r = 0.267004 + 0.031242 * r - 1.17733 * pow(r, 2) + 
                //         0.781638 * pow(r, 3) + 0.46992 * pow(r, 4);
                // color.g = 0.004874 + 1.05819 * r - 0.218094 * pow(r, 2) - 
                //         1.52621 * pow(r, 3) + 1.80664 * pow(r, 4);
                // color.b = 0.329415 - 0.197112 * r - 5.8219 * pow(r, 3) + 
                //         5.30237 * pow(r, 4);
                // color.a = 1.0;
            }

            {
                // inferno
                // r = r > 0.9 ? 0.9 : r;
                // if (r < 0.25) {
                //     color.r = 0.2 * r * 4.0;
                //     color.g = 0.0;
                //     color.b = 0.3 + 0.7 * r * 4.0;
                // } 
                // else if (r < 0.5) {
                //     color.r = 0.2 + 0.8 * (r-0.25)*4.0;
                //     color.g = 0.1 * (r-0.25)*4.0;
                //     color.b = 1.0 - 0.8 * (r-0.25)*4.0;
                // }
                // else if (r < 0.75) {
                //     color.r = 1.0;
                //     color.g = 0.1 + 0.9 * (r-0.5)*4.0;
                //     color.b = 0.2 - 0.2 * (r-0.5)*4.0;
                // }
                // else {
                //     color.r = 1.0;
                //     color.g = 1.0;
                //     color.b = 0.0 + 1.0 * (r-0.75)*4.0;
                // }
                
                // color.a = 0.5;
            }
            {
                color.r = rgba[0];
                color.g = rgba[1];
                color.b = rgba[2];
                color.a = rgba[3];
            }
            line_strip.colors.push_back(color);
        }

        pub.publish(line_strip);
    }

    planner::MeshTraj Planner::toMeshMsg(const MomaTraj& traj) const {
        double traj_duration = traj.getTotalDuration();
        const int res = 1000;
        double intvl = traj_duration / res;


        planner::MeshTraj ret;
        std::vector<planner::MeshState> states;
        std::vector<double> arc_lengths;
        std::vector<double> yaws;

        double acc_arc_length = 0.0;
        Eigen::VectorXd prev_state = traj.getState(0);
        
        for (double t = 0.0; t < traj_duration; t += intvl) {
            Eigen::VectorXd state = traj.getState(t);
            std::vector<Eigen::VectorXd> mesh_poses = moma_param.getMeshPose(state);

            planner::MeshState mesh_state;
            std::vector<planner::MeshPart> mesh_poses_msg;

            for (Eigen::VectorXd mesh_pose : mesh_poses) {
                planner::MeshPart mesh_pose_msg;
                

                mesh_pose_msg.pos_x = mesh_pose(0);
                mesh_pose_msg.pos_y = mesh_pose(1);
                mesh_pose_msg.pos_z = mesh_pose(2);
                mesh_pose_msg.orient_w = mesh_pose(3);
                mesh_pose_msg.orient_x = mesh_pose(4);
                mesh_pose_msg.orient_y = mesh_pose(5);
                mesh_pose_msg.orient_z = mesh_pose(6);

                mesh_poses_msg.push_back(mesh_pose_msg);
            }


            mesh_state.parts = mesh_poses_msg;
            states.push_back(mesh_state);

            acc_arc_length += (state.head(2) - prev_state.head(2)).norm();

            arc_lengths.push_back(acc_arc_length);
            yaws.push_back(state(2));


            prev_state = state;
        }


        ret.states = states;
        ret.arc_lengths = arc_lengths;
        ret.yaws = yaws;
        return ret;
    }

    void Planner::vis_time(ros::Publisher& pub, double time, int id) const {
        visualization_msgs::MarkerArray array_msg;
        visualization_msgs::Marker p;
        p.action = visualization_msgs::Marker::DELETEALL;
        p.id = 0;
        array_msg.markers.push_back(p);

        visualization_msgs::Marker text;
        text.header.stamp = ros::Time::now();
        text.header.frame_id = "world";
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.id = 1886666;
        text.action = visualization_msgs::Marker::ADD;
        text.scale.z = 1.2;
        text.color.a = 1.0;

        int rounded = std::round(time);
        // Remove extra zeros if needed

        text.text = std::to_string(rounded) + " ms";
        text.id = id;
        text.pose.orientation.w = 1.0;
        text.pose.position.x = 0.0;
        text.pose.position.y = 0.0;
        text.pose.position.z = 2.0;
        array_msg.markers.push_back(text);

        pub.publish(array_msg);
    }

}