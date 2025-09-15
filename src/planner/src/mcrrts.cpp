#include "planner/mcrrts.h"

namespace nmoma_planner
{
    bool MCRRTs::plan(const Eigen::VectorXd& start, const Eigen::VectorXd &end,
                      const std::vector<Eigen::Vector4d>& path, std::vector<Eigen::VectorXd>& wb_path)
    {
        reset(path);
        MCRRTNodePtr start_node, end_node;
        MCRRTNodePtr path_node_1 = nullptr, path_node_2 = nullptr;

        start_node = genNodeFromState(std::make_pair(0, start.tail(moma_param.dof_num)));
        start_node->node_state = MCRRTNode::IN_TREE;
        start_node->cost = 0.0;
        end_node = genNodeFromState(std::make_pair(path.size()-1, end.tail(moma_param.dof_num)));
        end_node->node_state = MCRRTNode::IN_ANTI_TREE;
        end_node->cost = 0.0;

        ros::Time time_begin = ros::Time::now();
        int tree_count_ = 1;
        int anti_tree_count_ = 1;
        bool anti = false;
        bool first_connected = false;

        if (car_path.size() == 2)
        {
            wb_path.clear();
            if (connectCollision(start_node->robo_state, end_node->robo_state))
                return false;
            wb_path.push_back(start);
            wb_path.push_back(end);
            return true;
        }

        std::uniform_real_distribution<double> goal_sampler(0.0, 1.0);
        while(ros::ok())
        {
            if(connected && !first_connected)
            {
                first_connected = true;
                // PRINT_GREEN("[MCRRTs] Found path, begin optimizing.");
            }
            if (connected || (ros::Time::now() - time_begin).toSec() > max_time)
                break;

            if(tree_count_ > anti_tree_count_)
                anti = true;
            else
                anti = false;

            // sample a random state
            MCState rand_state;
            if (goal_sampler(rand_gen) < goal_sample_rate)
                rand_state = anti ? start_node->robo_state : end_node->robo_state;
            else
                rand_state = sampleState();

            // get the nearest node in the tree
            MCRRTNodePtr q_nearest = getNearestNode(rand_state, anti);
            if(q_nearest == nullptr)
                continue;

            // steer to the random state
            MCRRTNodePtr q_new = steer(q_nearest, rand_state);
            if(q_new == nullptr)
                continue;
            
            // q_new is in both tree and anti-tree (already connected)
            if( (!anti && q_new->node_state == MCRRTNode::IN_ANTI_TREE) || 
                (anti && q_new->node_state == MCRRTNode::IN_TREE) )
            {
                // PRINT_GREEN("[MCRRTs] connected by sample the node in anti tree!");
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
            if( q_new->node_state == MCRRTNode::EXPANDED || 
                ( q_new->node_state == q_nearest->node_state && 
                  q_new->cost > q_nearest->cost + estHeuristic(q_nearest, q_new) ))
            {
                linkNode(q_nearest, q_new);
                q_new->node_state = q_nearest->node_state;
                if(q_new->node_state == MCRRTNode::IN_TREE)
                    ++tree_count_;
                else 
                    ++anti_tree_count_;
                updateMinMaxIdx(q_new);
                rewire(q_new);

                // find nearest node in the opposite tree
                MCRRTNodePtr q_near_opp = getNearestNode(q_new->robo_state, !anti);
                if(q_near_opp == nullptr)
                    continue;

                // steer to the node in the opposite tree
                MCRRTNodePtr q_new_opp = steer(q_near_opp, q_new->robo_state);
                if(q_new_opp == nullptr)
                    continue;

                // q_new is in both tree and anti-tree (already connected)
                if((anti && q_new_opp->node_state == MCRRTNode::IN_ANTI_TREE) || 
                    (!anti && q_new_opp->node_state == MCRRTNode::IN_TREE))
                {
                    // PRINT_GREEN("[MCRRTs] connected by steer the node in anti tree to tree!");
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
                if(q_new_opp->node_state == MCRRTNode::EXPANDED || 
                    (q_new_opp->node_state == q_near_opp->node_state 
                     && q_new_opp->cost > q_near_opp->cost + estHeuristic(q_near_opp, q_new_opp)))
                {
                    linkNode(q_near_opp, q_new_opp);
                    q_new_opp->node_state = q_near_opp->node_state;
                    updateMinMaxIdx(q_new_opp);
                    rewire(q_new_opp);

                    if(q_new_opp->node_state == MCRRTNode::IN_TREE)
                        ++tree_count_;
                    else 
                        ++anti_tree_count_;

                    // try connecting tree once
                    int try_count = 0;
                    while(q_new->robo_state.first != q_new_opp->robo_state.first && ros::ok())
                    {
                        if (!first_connected && connected)
                        {
                            first_connected = true;
                            // PRINT_GREEN("[MCRRTs] Found path, begin optimizing.");
                        }

                        try_count++;
                        // if (try_count > 10)
                        //     break;

                        if((ros::Time::now() - time_begin).toSec() > max_time)
                            break;

                        MCRRTNodePtr q_new_2 = steer(q_new_opp, q_new->robo_state);
                        if (q_new_2 == nullptr)
                            break;
                        // q_new is a new node
                        // or q_new is in the same tree with q_nearest, and from q_new to end is shorter
                        if(q_new_2->node_state == MCRRTNode::EXPANDED || 
                            (q_new_2->node_state == q_new_opp->node_state && 
                            q_new_2->cost > q_new_opp->cost + estHeuristic(q_new_opp, q_new_2)))
                        {
                            linkNode(q_new_opp, q_new_2);
                            q_new_2->node_state = q_new_opp->node_state;
                            updateMinMaxIdx(q_new_2);
                            // rewire(q_new_2);
                            if(q_new_2->node_state == MCRRTNode::IN_TREE)
                                ++tree_count_;
                            else 
                                ++anti_tree_count_;
                            q_new_opp = q_new_2;
                        }
                        // connected
                        else if(((!anti && q_new_2->node_state == MCRRTNode::IN_TREE) || 
                                 (anti && q_new_2->node_state == MCRRTNode::IN_ANTI_TREE)))
                        {
                            connected = true;
                            // PRINT_GREEN("[MCRRTs] connected by steer to tree!");
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
                            PRINT_YELLOW("[MCRRTs] may have a!");
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

        wb_path.clear();

        if(connected)
        {
            mergeTree(path_node_1, path_node_2);
            MCRRTNodePtr node = end_node;
            while(node != nullptr)
            {
                Eigen::VectorXd full_state(state_dim+3);
                full_state.head(3) = car_path[node->robo_state.first].head(3);
                full_state.tail(state_dim) = node->robo_state.second;
                wb_path.push_back(full_state);
                node = node->parent;
            }
            reverse(wb_path.begin(), wb_path.end());
        }
        else
        {
            // PRINT_RED("[MCRRTs] No path found! expanded size: "<< node_pool.size());
            return false;
        }

        return true;
    }

    void MCRRTs::getHoleNodes(std::vector<Eigen::VectorXd>& starts, std::vector<Eigen::VectorXd>& ends,
                              std::vector<double>& start_costs, std::vector<double>& end_costs,
                              std::vector<int>& start_mcidx, std::vector<int>& end_mcidx)
    {
        int index_front, index_last;
        starts.clear();
        ends.clear();
        start_costs.clear();
        end_costs.clear();
        start_mcidx.clear();
        end_mcidx.clear();
        // starts
        if(near_max_idx <= near_min_idx)
        {
            index_front = max(near_max_idx - 1, 0);
            index_last = near_max_idx;
        }
        else
        {
            index_front = max(near_min_idx - 1, 0);
            index_last = near_max_idx;
        }
        auto it = node_pool.lower_bound(string(1, index_front));
        for(; it != node_pool.end() && it->second->robo_state.first <= index_last; ++it)
        {
            if(it->second->node_state == MCRRTNode::IN_TREE)
            {
                Eigen::VectorXd state_full(state_dim+3);
                state_full.head(3) = car_path[it->second->robo_state.first].head(3);
                state_full.tail(state_dim) = it->second->robo_state.second;
                starts.push_back(state_full);
                start_costs.push_back(it->second->cost);
                start_mcidx.push_back(it->second->robo_state.first);
            }
        }
        //ends
        if(near_max_idx <= near_min_idx)
        {
            index_front = near_min_idx;
            index_last = min(near_min_idx + 1, (int)(car_path.size()) - 1);
        }
        else
        {
            index_front = near_min_idx;
            index_last = min(near_max_idx + 1, (int)(car_path.size()) - 1);
        }
        for(it = node_pool.lower_bound(string(1, index_front)); 
            it != node_pool.end() && it->second->robo_state.first <= index_last; ++it)
        {
            if(it->second->node_state == MCRRTNode::IN_ANTI_TREE)
            {
                Eigen::VectorXd end_full(state_dim+3);
                end_full.head(3) = car_path[it->second->robo_state.first].head(3);
                end_full.tail(state_dim) = it->second->robo_state.second;
                ends.push_back(end_full);
                end_costs.push_back(it->second->cost);
                end_mcidx.push_back(it->second->robo_state.first);
            }
        }

        return;
    }

    void MCRRTs::fillHole(const std::vector<Eigen::VectorXd>& hole_path, std::pair<int, int>& layer,
                          std::vector<Eigen::VectorXd>& wb_path)
    {
        wb_path.clear();
        MCState hole_start(layer.first, hole_path[0].tail(state_dim));
        MCState hole_end(layer.second, hole_path.back().tail(state_dim));
        MCRRTNodePtr start_node, end_node;
        auto it = node_pool.find(getKey(hole_start));
        if (it != node_pool.end()) start_node = it->second;
        else return;
        it = node_pool.find(getKey(hole_end));
        if (it != node_pool.end()) end_node = it->second;
        else return;

        MCRRTNodePtr node = start_node;
        while(node != nullptr)
        {
            Eigen::VectorXd full_state(state_dim+3);
            full_state.head(3) = car_path[node->robo_state.first].head(3);
            full_state.tail(state_dim) = node->robo_state.second;
            wb_path.push_back(full_state);
            node = node->parent;
        }
        reverse(wb_path.begin(), wb_path.end());
        for (size_t i=0; i<hole_path.size()-1; ++i)
        {
            wb_path.push_back(hole_path[i]);
        }
        node = end_node;
        while(node != nullptr)
        {
            Eigen::VectorXd full_state(state_dim+3);
            full_state.head(3) = car_path[node->robo_state.first].head(3);
            full_state.tail(state_dim) = node->robo_state.second;
            wb_path.push_back(full_state);
            node = node->parent;
        }
        return;
    }

    MCRRTNodePtr MCRRTs::steer(MCRRTNodePtr &node, const MCState &target_state)
    {
        Eigen::VectorXd diff = target_state.second - node->robo_state.second;
        Eigen::VectorXd state_new(state_dim);

        double time = 0.0;
        int idx1 = node->robo_state.first;
        int idx2 = target_state.first;
        int min_idx = (idx1 < idx2) ? idx1 : idx2;
        int max_idx = (idx1 > idx2) ? idx1 : idx2;
        for (int i = min_idx; i < max_idx; ++i)
            time += car_path[i].w();
        Eigen::VectorXd vel = diff / time;
        for (int i = 0; i < state_dim; ++i)
        {
            double v_limit = moma_param.joint_vel_limit(i);
            vel(i) = max(min(vel(i), v_limit), -v_limit);
        }

        int new_idx;
        bool anti = (node->node_state == MCRRTNode::IN_ANTI_TREE);
        if (anti)
        {
            state_new = node->robo_state.second + vel * car_path[node->robo_state.first-1].w();
            new_idx = node->robo_state.first - 1;
        }
        else
        {
            state_new = node->robo_state.second + vel * car_path[node->robo_state.first].w();
            new_idx = node->robo_state.first + 1;
        }

        MCState new_mc_state(new_idx, state_new);

        if(connectCollision(node->robo_state, new_mc_state))
            return nullptr;

        return genNodeFromState(new_mc_state);
    }

    void MCRRTs::rewire(MCRRTNodePtr q_new)
    {
        if (q_new == nullptr || q_new->robo_state.first < 1)
            return;

        bool anti = (q_new->node_state == MCRRTNode::IN_ANTI_TREE);
        int next_idx = anti ? q_new->robo_state.first-1 : q_new->robo_state.first+1;
        if ( (anti && next_idx < near_min_idx) || (!anti && next_idx > near_max_idx) ) 
            return;

        for(auto it = node_pool.lower_bound(string(1, next_idx));
            it != node_pool.end() && it->second->robo_state.first == next_idx; ++it)
        {
            MCRRTNodePtr q_temp = it->second;
            if (q_temp->node_state != q_new->node_state)
                continue;

            if(q_temp->cost > q_new->cost + estHeuristic(q_new, q_temp) && 
               feasibleCheck(q_new->robo_state, q_temp->robo_state) &&
               !connectCollision(q_new->robo_state, q_temp->robo_state))
            {
                linkNode(q_new, q_temp);
            }
        }
    }
}
