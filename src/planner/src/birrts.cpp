#include "planner/birrts.h"

namespace nmoma_planner
{
    bool BiRRTs::plan(const std::vector<Eigen::VectorXd>& starts, const std::vector<Eigen::VectorXd>& ends,
                      const std::vector<double>& start_costs, const std::vector<double>& end_costs,
                      const std::vector<int>& start_mcidx, const std::vector<int>& end_mcidx,
                      std::vector<Eigen::VectorXd>& path, std::vector<double>& time_list, std::pair<int, int>& layer)
    {
        reset(starts, ends);
        RRTNodePtr start_node, end_node;
        RRTNodePtr path_node_1 = nullptr, path_node_2 = nullptr;

        for (size_t i=0; i<starts.size(); i++)
        {
            PRINT_GREEN(start_costs[i]<<" "<<start_mcidx[i]<<" start state: " << starts[i].transpose());
            start_node = genNodeFromState(starts[i]);
            start_node->node_state = RRTNode::IN_TREE;
            start_node->cost = start_costs[i];
            start_node->mc_idx = start_mcidx[i];
        }
        for (size_t i=0; i<ends.size(); i++)
        {
            PRINT_GREEN(end_costs[i]<<"  "<<end_mcidx[i]<<" end state: " << ends[i].transpose());
            end_node = genNodeFromState(ends[i]);
            end_node->node_state = RRTNode::IN_ANTI_TREE;
            end_node->cost = end_costs[i];
            end_node->mc_idx = end_mcidx[i];
        }
        
        ros::Time time_begin = ros::Time::now();
        int tree_count_ = starts.size();
        int anti_tree_count_ = ends.size();
        bool anti = false;
        bool first_connected = false;

        while(ros::ok())
        {
            if(connected)
            {
                if (!first_connected)
                {
                    first_connected = true;
                    PRINT_GREEN("[BiRRTs] Found path, begin optimizing.");
                }
                if ((ros::Time::now() - time_begin).toSec() > max_time)
                    break;
            }

            if(tree_count_ > anti_tree_count_)
                anti = true;
            else
                anti = false;

            // sample a random state
            Eigen::VectorXd rand_state = sampleState();

            // get the nearest node in the tree
            RRTNodePtr q_nearest = getNearestNode(rand_state, anti);
            if(q_nearest == nullptr)
                continue;

            // steer to the random state
            RRTNodePtr q_new = steer(q_nearest, rand_state);
            if(q_new == nullptr)
                continue;
            
            // q_new is in both tree and anti-tree (already connected)
            if( (!anti && q_new->node_state == RRTNode::IN_ANTI_TREE) || 
                (anti && q_new->node_state == RRTNode::IN_TREE) )
            {
                // PRINT_GREEN("[BiRRTs] connected by sample the node in anti tree!");
                connected = true;
                double cost = q_nearest->cost + q_new->cost + estHeuristic(q_nearest, q_new);
                if(cost < c_max)
                {
                    c_max = cost;
                    path_node_1 = q_nearest;
                    path_node_2 = q_new;
                }
                continue;
            }

            // q_new is a new node
            // or q_new is in the same tree with q_nearest, and from q_new to end is shorter
            if( q_new->node_state == RRTNode::EXPANDED || 
                ( q_new->node_state == q_nearest->node_state && 
                  q_new->cost > q_nearest->cost + estHeuristic(q_nearest, q_new) ))
            {
                linkNode(q_nearest, q_new);
                q_new->node_state = q_nearest->node_state;
                if(q_new->node_state == RRTNode::IN_TREE)
                    ++tree_count_;
                else 
                    ++anti_tree_count_;
                rewire(q_new);

                // find nearest node in the opposite tree
                RRTNodePtr q_near_opp = getNearestNode(q_new->robo_state, !anti);
                if(q_near_opp == nullptr)
                    continue;

                // steer to the node in the opposite tree
                RRTNodePtr q_new_opp = steer(q_near_opp, q_new->robo_state);
                if(q_new_opp == nullptr)
                    continue;

                // q_new is in both tree and anti-tree (already connected)
                if((anti && q_new_opp->node_state == RRTNode::IN_ANTI_TREE) || 
                    (!anti && q_new_opp->node_state == RRTNode::IN_TREE))
                {
                    // PRINT_GREEN("[BiRRTs] connected by steer the node in anti tree to tree!");
                    connected = true;
                    double cost = q_new_opp->cost + q_near_opp->cost + estHeuristic(q_new_opp, q_near_opp);
                    if( cost < c_max)
                    {
                        c_max = cost;
                        path_node_1 = q_new_opp;
                        path_node_2 = q_near_opp;
                    }
                    continue;
                }

                // q_new is a new node
                // or q_new is in the same tree with q_nearest, and from q_new to end is shorter
                if(q_new_opp->node_state == RRTNode::EXPANDED || 
                    (q_new_opp->node_state == q_near_opp->node_state 
                     && q_new_opp->cost > q_near_opp->cost + estHeuristic(q_near_opp, q_new_opp)))
                {
                    linkNode(q_near_opp, q_new_opp);
                    q_new_opp->node_state = q_near_opp->node_state;
                    rewire(q_new_opp);

                    if(q_new_opp->node_state == RRTNode::IN_TREE)
                        ++tree_count_;
                    else 
                        ++anti_tree_count_;

                    // try connecting tree once
                    int try_count = 0;
                    while((q_new->robo_state - q_new_opp->robo_state).norm() > 1.0e-2 && ros::ok())
                    {
                        if (!first_connected && connected)
                        {
                            first_connected = true;
                            PRINT_GREEN("[BiRRTs] Found path, begin optimizing.");
                        }

                        try_count++;
                        if (try_count > 10)
                            break;

                        if((ros::Time::now() - time_begin).toSec() > max_time)
                            break;

                        RRTNodePtr q_new_2 = steer(q_new_opp, q_new->robo_state);
                        if (q_new_2 == nullptr)
                            break;
                        // q_new is a new node
                        // or q_new is in the same tree with q_nearest, and from q_new to end is shorter
                        if(q_new_2->node_state == RRTNode::EXPANDED || 
                            (q_new_2->node_state == q_new_opp->node_state && 
                            q_new_2->cost > q_new_opp->cost + estHeuristic(q_new_opp, q_new_2)))
                        {
                            linkNode(q_new_opp, q_new_2);
                            q_new_2->node_state = q_new_opp->node_state;
                            rewire(q_new_2);
                            if(q_new_2->node_state == RRTNode::IN_TREE)
                                ++tree_count_;
                            else 
                                ++anti_tree_count_;
                            q_new_opp = q_new_2;
                        }
                        // connected
                        else if(((!anti && q_new_2->node_state == RRTNode::IN_TREE) || 
                                 (anti && q_new_2->node_state == RRTNode::IN_ANTI_TREE)))
                        {
                            connected = true;
                            // PRINT_GREEN("[BiRRTs] connected by steer to tree!");
                            double cost = q_new_2->cost + q_new_opp->cost + estHeuristic(q_new_2, q_new_opp);
                            if(cost < c_max)
                            {
                                c_max = cost;
                                path_node_1 = q_new_2;
                                path_node_2 = q_new_opp;
                            }
                            break;
                        }
                        //! same tree with lower cost
                        else if(q_new_2->node_state == q_new_opp->node_state && 
                                q_new_2->cost < q_new_opp->cost + estHeuristic(q_new_opp, q_new_2))
                        {
                            PRINT_YELLOW("[BiRRTs] may have a problem!");
                            q_new_opp = q_new_2;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }

        path.clear();
        time_list.clear();

        if(connected)
        {
            
            mergeTree(path_node_1, path_node_2);
            RRTNodePtr node = goal_node;
            layer.second = goal_node->mc_idx;
            while(node != nullptr)
            {
                if (!path.empty())
                    time_list.push_back(estTime(path.back(), node->robo_state));
                path.push_back(node->robo_state);
                if(node->parent != nullptr)
                {
                    RRTNodePtr temp = node->parent;
                    double t = estTime(temp->robo_state, node->robo_state);
                    int res = floor(t / steer_time);
                    ompl::base::ScopedState<> from(se2_geop), to(se2_geop), s(se2_geop);
                    from[0] = node->robo_state[0]; from[1] = node->robo_state[1]; from[2] = node->robo_state[2];
                    to[0] = temp->robo_state[0]; to[1] = temp->robo_state[1]; to[2] = temp->robo_state[2];
                    Eigen::VectorXd temp_state(state_dim);
                    for(int i = 1; i < res; ++i)
                    {
                        double temp_time = (double)i / res;
                        se2_geop->interpolate(from(), to(), temp_time, s());
                        temp_state[0] = s[0]; temp_state[1] = s[1]; temp_state[2] = s[2];
                        temp_state.tail(moma_param.dof_num) = node->robo_state.tail(moma_param.dof_num) + 
                                                              (temp->robo_state.tail(moma_param.dof_num) 
                                                               - node->robo_state.tail(moma_param.dof_num)) * temp_time;
                        path.push_back(temp_state);
                        time_list.push_back(estTime(temp_state, node->robo_state));
                    }
                }
                else
                {
                    layer.first = node->mc_idx;
                }
                node = node->parent;
            }
            reverse(path.begin(), path.end());
            reverse(time_list.begin(), time_list.end());
        }
        else
        {
            PRINT_RED("[BiRRTs] No path found! expanded size: "<< node_pool.size());
            return false;
        }

        return true;
    }

    bool BiRRTs::plan(const Eigen::VectorXd& start, const Eigen::VectorXd &end,
                      std::vector<Eigen::VectorXd>& path, std::vector<double>& time_list)
    {
        reset(start, end);
        RRTNodePtr start_node, end_node;
        RRTNodePtr path_node_1 = nullptr, path_node_2 = nullptr;

        start_node = genNodeFromState(start);
        start_node->node_state = RRTNode::IN_TREE;
        start_node->cost = 0.0;
        end_node = genNodeFromState(end);
        end_node->node_state = RRTNode::IN_ANTI_TREE;
        end_node->cost = 0.0;

        ros::Time time_begin = ros::Time::now();
        int tree_count_ = 1;
        int anti_tree_count_ = 1;
        bool anti = false;
        bool first_connected = false;
        while(ros::ok())
        {
            if(connected)
            {
                if (!first_connected)
                {
                    first_connected = true;
                    PRINT_GREEN("[BiRRTs] Found path, begin optimizing.");
                }
                // if ((ros::Time::now() - time_begin).toSec() > max_time)
                //     break;
            }

            if ((ros::Time::now() - time_begin).toSec() > max_time)
                break;

            if(tree_count_ > anti_tree_count_)
                anti = true;
            else
                anti = false;

            // sample a random state
            Eigen::VectorXd rand_state = sampleState();

            // get the nearest node in the tree
            RRTNodePtr q_nearest = getNearestNode(rand_state, anti);
            if(q_nearest == nullptr)
                continue;

            // steer to the random state
            RRTNodePtr q_new = steer(q_nearest, rand_state);
            if(q_new == nullptr)
                continue;
            
            // q_new is in both tree and anti-tree (already connected)
            if( (!anti && q_new->node_state == RRTNode::IN_ANTI_TREE) || 
                (anti && q_new->node_state == RRTNode::IN_TREE) )
            {
                // PRINT_GREEN("[BiRRTs] connected by sample the node in anti tree!");
                connected = true;
                double cost = q_nearest->cost + q_new->cost + estHeuristic(q_nearest, q_new);
                if(cost < c_max)
                {
                    c_max = cost;
                    path_node_1 = q_nearest;
                    path_node_2 = q_new;
                }
                continue;
            }

            // q_new is a new node
            // or q_new is in the same tree with q_nearest, and from q_new to end is shorter
            if( q_new->node_state == RRTNode::EXPANDED || 
                ( q_new->node_state == q_nearest->node_state && 
                  q_new->cost > q_nearest->cost + estHeuristic(q_nearest, q_new) ))
            {
                linkNode(q_nearest, q_new);
                q_new->node_state = q_nearest->node_state;
                if(q_new->node_state == RRTNode::IN_TREE)
                    ++tree_count_;
                else 
                    ++anti_tree_count_;
                rewire(q_new);

                // find nearest node in the opposite tree
                RRTNodePtr q_near_opp = getNearestNode(q_new->robo_state, !anti);
                if(q_near_opp == nullptr)
                    continue;

                // steer to the node in the opposite tree
                RRTNodePtr q_new_opp = steer(q_near_opp, q_new->robo_state);
                if(q_new_opp == nullptr)
                    continue;

                // q_new is in both tree and anti-tree (already connected)
                if((anti && q_new_opp->node_state == RRTNode::IN_ANTI_TREE) || 
                    (!anti && q_new_opp->node_state == RRTNode::IN_TREE))
                {
                    // PRINT_GREEN("[BiRRTs] connected by steer the node in anti tree to tree!");
                    connected = true;
                    double cost = q_new_opp->cost + q_near_opp->cost + estHeuristic(q_new_opp, q_near_opp);
                    if( cost < c_max)
                    {
                        c_max = cost;
                        path_node_1 = q_new_opp;
                        path_node_2 = q_near_opp;
                    }
                    continue;
                }

                // q_new is a new node
                // or q_new is in the same tree with q_nearest, and from q_new to end is shorter
                if(q_new_opp->node_state == RRTNode::EXPANDED || 
                    (q_new_opp->node_state == q_near_opp->node_state 
                     && q_new_opp->cost > q_near_opp->cost + estHeuristic(q_near_opp, q_new_opp)))
                {
                    linkNode(q_near_opp, q_new_opp);
                    q_new_opp->node_state = q_near_opp->node_state;
                    rewire(q_new_opp);

                    if(q_new_opp->node_state == RRTNode::IN_TREE)
                        ++tree_count_;
                    else 
                        ++anti_tree_count_;

                    // try connecting tree once
                    int try_count = 0;
                    while((q_new->robo_state - q_new_opp->robo_state).norm() > 1.0e-2 && ros::ok())
                    {
                        if (!first_connected && connected)
                        {
                            first_connected = true;
                            PRINT_GREEN("[BiRRTs] Found path, begin optimizing.");
                        }

                        try_count++;
                        if (try_count > 10)
                            break;

                        if((ros::Time::now() - time_begin).toSec() > max_time)
                            break;

                        RRTNodePtr q_new_2 = steer(q_new_opp, q_new->robo_state);
                        if (q_new_2 == nullptr)
                            break;
                        // q_new is a new node
                        // or q_new is in the same tree with q_nearest, and from q_new to end is shorter
                        if(q_new_2->node_state == RRTNode::EXPANDED || 
                            (q_new_2->node_state == q_new_opp->node_state && 
                            q_new_2->cost > q_new_opp->cost + estHeuristic(q_new_opp, q_new_2)))
                        {
                            linkNode(q_new_opp, q_new_2);
                            q_new_2->node_state = q_new_opp->node_state;
                            rewire(q_new_2);
                            if(q_new_2->node_state == RRTNode::IN_TREE)
                                ++tree_count_;
                            else 
                                ++anti_tree_count_;
                            q_new_opp = q_new_2;
                        }
                        // connected
                        else if(((!anti && q_new_2->node_state == RRTNode::IN_TREE) || 
                                 (anti && q_new_2->node_state == RRTNode::IN_ANTI_TREE)))
                        {
                            connected = true;
                            // PRINT_GREEN("[BiRRTs] connected by steer to tree!");
                            double cost = q_new_2->cost + q_new_opp->cost + estHeuristic(q_new_2, q_new_opp);
                            if(cost < c_max)
                            {
                                c_max = cost;
                                path_node_1 = q_new_2;
                                path_node_2 = q_new_opp;
                            }
                            break;
                        }
                        //! same tree with lower cost
                        else if(q_new_2->node_state == q_new_opp->node_state && 
                                q_new_2->cost < q_new_opp->cost + estHeuristic(q_new_opp, q_new_2))
                        {
                            PRINT_YELLOW("[BiRRTs] may have a problem!");
                            q_new_opp = q_new_2;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }

        path.clear();
        time_list.clear();

        if(connected)
        {
            mergeTree(path_node_1, path_node_2);
            RRTNodePtr node = end_node;
            // while(node != nullptr)
            // {
            //     path.push_back(node->robo_state);
            //     if(node->parent != nullptr)
            //         time_list.push_back(estTime(node->parent->robo_state, node->robo_state));
            //     node = node->parent;
            // }
            while(node != nullptr)
            {
                if (!path.empty())
                    time_list.push_back(estTime(path.back(), node->robo_state));
                path.push_back(node->robo_state);
                if(node->parent != nullptr)
                {
                    RRTNodePtr temp = node->parent;
                    double t = estTime(temp->robo_state, node->robo_state);
                    int res = floor(t / steer_time);
                    ompl::base::ScopedState<> from(se2_geop), to(se2_geop), s(se2_geop);
                    from[0] = node->robo_state[0]; from[1] = node->robo_state[1]; from[2] = node->robo_state[2];
                    to[0] = temp->robo_state[0]; to[1] = temp->robo_state[1]; to[2] = temp->robo_state[2];
                    Eigen::VectorXd temp_state(state_dim);
                    for(int i = 1; i < res; ++i)
                    {
                        double temp_time = (double)i / res;
                        se2_geop->interpolate(from(), to(), temp_time, s());
                        temp_state[0] = s[0]; temp_state[1] = s[1]; temp_state[2] = s[2];
                        temp_state.tail(moma_param.dof_num) = node->robo_state.tail(moma_param.dof_num) + 
                                                              (temp->robo_state.tail(moma_param.dof_num) 
                                                               - node->robo_state.tail(moma_param.dof_num)) * temp_time;
                        path.push_back(temp_state);
                        time_list.push_back(estTime(temp_state, node->robo_state));
                    }
                }
                node = node->parent;
            }
            reverse(path.begin(), path.end());
            reverse(time_list.begin(), time_list.end());
        }
        else
        {
            PRINT_RED("[BiRRTs] No path found! expanded size: "<< node_pool.size());
            return false;
        }

        return true;
    }

    RRTNodePtr BiRRTs::steer(RRTNodePtr &node, const Eigen::VectorXd &target_state)
    {
        Eigen::VectorXd diff = target_state - node->robo_state;
        Eigen::VectorXd state_new(state_dim);
        
        ompl::base::ScopedState<> from(se2_geop), to(se2_geop), s(se2_geop);
        from[0] = node->robo_state[0]; from[1] = node->robo_state[1]; from[2] = node->robo_state[2];
        to[0] = target_state[0]; from[1] = target_state[1]; from[2] = target_state[2];
        double len = se2_geop->distance(from(), to());
        if(len > moma_param.max_v * steer_time)
        {
            se2_geop->interpolate(from(), to(), moma_param.max_v * steer_time / len, s());
            auto reals = s.reals();
            state_new(0) = reals[0]; 
            state_new(1) = reals[1];
            state_new(2) = reals[2];
        }
        else
        {
            state_new.head(3) = target_state.head(3);
        }

        if(diff.tail(moma_param.dof_num).norm() > 1.0e-2)
            state_new.tail(moma_param.dof_num) = node->robo_state.tail(moma_param.dof_num) 
                                                + diff.tail(moma_param.dof_num).normalized()
                                                .cwiseProduct(moma_param.joint_vel_limit) * steer_time * 0.5;
        else
            state_new.tail(moma_param.dof_num) = node->robo_state.tail(moma_param.dof_num);

        for(int i = 3; i < state_dim; ++i)
        {
            if(state_new(i) < min(target_state(i), node->robo_state(i)) ||
               state_new(i) > max(target_state(i), node->robo_state(i)))
                state_new(i) = target_state(i);
        }

        if(connectCollision(node->robo_state, state_new))
            return nullptr;

        return genNodeFromState(state_new);
    }

    void BiRRTs::rewire(RRTNodePtr q_new)
    {
        // get the neighbours of q_new
        std::vector<RRTNodePtr> neighbour;
        RRTNodePtr temp;

        for(auto it = node_pool.begin(); it != node_pool.end(); ++it)
        {
            temp = it->second;
            bool near = true;
            if(temp->node_state != q_new->node_state)
                continue;
            if(temp == q_new)
                continue;
            if((temp->robo_state.head(2) - q_new->robo_state.head(2)).norm() >
                moma_param.max_v * rewire_time_range)
                continue;
            for(size_t j = 0; j < moma_param.dof_num; ++j)
            {
                if (fabs(temp->robo_state(3+j)-q_new->robo_state(3+j)) > moma_param.joint_vel_limit[j] * rewire_time_range)
                {
                    near = false;
                    break;
                }
            }
            if(!near)
                continue;
            neighbour.push_back(temp);
        }

        if(neighbour.empty())
            return;

        // find the best parent of q_new
        temp = nullptr;
        double min_cost = q_new->cost;
        for(size_t i = 0; i < neighbour.size(); ++i)
        {
            double cost = neighbour[i]->cost + estHeuristic(neighbour[i], q_new);
            if( cost < min_cost && !connectCollision(neighbour[i]->robo_state, q_new->robo_state))
            {
                min_cost = cost;
                temp = neighbour[i];
            }
        }
        if(temp != nullptr)
            linkNode(temp, q_new);
        
        // update neighbours of q_new
        for(size_t i = 0; i < neighbour.size(); ++i)
        {
            if(q_new->cost + estHeuristic(q_new, neighbour[i]) < neighbour[i]->cost 
                && !connectCollision(q_new->robo_state, neighbour[i]->robo_state))
                linkNode(q_new, neighbour[i]);
        }
        return;
    }
}
