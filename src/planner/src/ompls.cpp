#include "planner/ompls.h"

namespace nmoma_planner
{
    void OMPLPlanner::samplePRM(int traj_num, std::vector<std::vector<Eigen::VectorXd>>& paths_list)
    {
        bool save_map = true;

        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        default_random_engine eng(0);
        uniform_real_distribution<double> rand_x(grid_map->min_boundary(0)+2.0, grid_map->max_boundary(0)-2.0);
        uniform_real_distribution<double> rand_y(grid_map->min_boundary(1)+2.0, grid_map->max_boundary(1)-2.0);
        uniform_real_distribution<double> rand_theta(-M_PI, M_PI);
        ob::ScopedState<MomaStateSpace> start_state(space);
        ob::ScopedState<MomaStateSpace> end_state(space);

        paths_list.clear();
        for (int i=0; i<traj_num; i++)
        {
            Eigen::VectorXd start(moma_param.dof_num+3);
            Eigen::VectorXd end(moma_param.dof_num+3);
            while(true)
            {
                start[0] = rand_x(eng);
                start[1] = rand_y(eng);
                double dist;
                grid_map->getDistance2d(start.head(2), dist);
                if (dist < moma_param.chassis_colli_radius)
                    continue;
                end[0] = rand_x(eng);
                end[1] = rand_y(eng);
                grid_map->getDistance2d(end.head(2), dist);
                if (dist < moma_param.chassis_colli_radius)
                    continue;
                if ((start-end).head(2).norm() < 14.0 && (start-end).head(2).norm() > 1.0)
                {
                    int try_num = 0;
                    do
                    {
                        if (try_num ++ > 100)
                            break;
                        for (size_t j = 0; j < moma_param.dof_num+1; j++)
                            start[j+2] = rand_theta(eng);
                    }while (grid_map->isWholeBodyCollision(start));
                    if (try_num > 100)
                        continue;
                    try_num = 0;
                    do
                    {
                        if (try_num ++ > 100)
                            break;
                        for (size_t j = 0; j < moma_param.dof_num+1; j++)
                            end[j+2] = rand_theta(eng);
                    }while (grid_map->isWholeBodyCollision(end));
                    if (try_num > 100)
                        continue;
                    break;
                }
            }
            for (size_t j = 0; j < moma_param.dof_num+3; j++)
            {
                start_state[j] = start[j];
                end_state[j] = end[j];
            }
            pdef->setStartAndGoalStates(start_state, end_state);
            prm_planner->setProblemDefinition(pdef);
            if (i==0 && save_map)
            {
                prm_planner->setup();
                PRINT_GREEN("[PRM] Constructing roadmap ...");
                prm_planner->constructRoadmap(ob::timedPlannerTerminationCondition(construct_time));
                PRINT_GREEN("[PRM] Construct roadmap done.");
                ob::PlannerData plan_data(si);
                ob::PlannerDataStorage dataStorage;
                std::string filename = ros::package::getPath("planner")+"/roadmap/map";
                prm_planner->getPlannerData(plan_data);
                dataStorage.store(plan_data, filename.c_str());
                PRINT_GREEN("[PRM] Save roadmap done.");
            }
            PRINT_GREEN("iteration: " << i);
            if (prm_planner->solve(ob::timedPlannerTerminationCondition(plan_time)) != ob::PlannerStatus::EXACT_SOLUTION)
            {
                PRINT_RED("Failed to find a solution");
                continue;
            }
            og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
            PRINT_GREEN("path length: " << path->length()<<"   path nodes: ");
            path->printAsMatrix(std::cout);
            std::vector<Eigen::VectorXd> eigen_path;
            for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
            {
                const MomaStateSpace::StateType *moma_state = path->getState(idx)->as<MomaStateSpace::StateType>();

                Eigen::VectorXd node(moma_param.dof_num+3);
                for (size_t j=0; j<moma_param.dof_num+3; j++)
                    node(j) = moma_state->values[j];
                eigen_path.push_back(node);  
            }
            paths_list.push_back(eigen_path);
            pdef->clearSolutionPaths();
        }
        return;
    }

    void OMPLPlanner::planPRM(const Eigen::VectorXd& start, 
                            const Eigen::VectorXd& end, 
                            std::vector<Eigen::VectorXd>& path_list)
    {
        path_list.clear();

        //设置机器人的初始状态
        ob::ScopedState<MomaStateSpace> start_state(space);
        ob::ScopedState<MomaStateSpace> end_state(space);
        for (size_t i = 0; i < moma_param.dof_num + 3; i++)
        {
            start_state[i] = start[i];
            end_state[i] = end[i];
        }

        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start_state, end_state);
        pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));
        prm_planner->setProblemDefinition(pdef);
        
        ob::PlannerTerminationCondition condition = ob::timedPlannerTerminationCondition(plan_time);
        if (prm_planner->solve(condition) == ob::PlannerStatus::EXACT_SOLUTION)
        {
            og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
            PRINT_GREEN("path length: " << path->length());

            if (use_inter)
            {
                ob::State *moma_state = space->allocState();
                for (std::size_t idx = 0; idx < path->getStateCount() - 1; idx++)
                {
                    
                    const MomaStateSpace::StateType *moma_state1 = path->getState(idx)->as<MomaStateSpace::StateType>();
                    const MomaStateSpace::StateType *moma_state2 = path->getState(idx+1)->as<MomaStateSpace::StateType>();

                    double time = si->distance(moma_state1, moma_state2);
                    int cnt = max( (int) (time / inter_time), 1 );
                    for (int i = 0; i < cnt; i++)
                    {
                        double t = (double) i / (double) cnt;
                        PRINT_GREEN("add node: " << t);
                        space->interpolate(moma_state1, moma_state2, t, moma_state);
                        Eigen::VectorXd node(moma_param.dof_num+3);
                        for (size_t j=0; j<moma_param.dof_num+3; j++)
                            node(j) = moma_state->as<MomaStateSpace::StateType>()->values[j];
                        path_list.push_back(node);  
                        PRINT_GREEN("add node: " << node.transpose());
                    }
                }
                path_list.push_back(end);  
                space->freeState(moma_state);
            }
            else
                for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
                {
                    const MomaStateSpace::StateType *moma_state = path->getState(idx)->as<MomaStateSpace::StateType>();

                    Eigen::VectorXd node(moma_param.dof_num+3);
                    for (size_t i=0; i<moma_param.dof_num+3; i++)
                        node(i) = moma_state->values[i];
                    path_list.push_back(node);  
                } 
        }
        else
        {
            PRINT_RED("Failed to find a solution");
        }

        
        return;
    }

    bool OMPLPlanner::planRRT(const Eigen::VectorXd& start, 
                            const Eigen::VectorXd& end, 
                            std::vector<Eigen::VectorXd>& path_list)
    {
        path_list.clear();

        ob::Planner* planner;
        if (planner_type.compare("BiTRRT") == 0) {
            planner = new og::BiTRRT(si);
        } else if (planner_type.compare("InformedRRTstar") == 0) {
            planner = new og::InformedRRTstar(si);
        } else if (planner_type.compare("LazyRRT") == 0) {
            planner = new og::LazyRRT(si);
        } else if (planner_type.compare("BFMT") == 0) {
            planner = new og::BFMT(si);
        } else {
            throw std::runtime_error("Invalid planner type: " + planner_type);
        }
        // og::BiTRRT *planner = new og::BiTRRT(si);
        // og::RRTstar *planner = new og::InformedRRTstar(si);
        // og::LazyRRT *planner = new og::LazyRRT(si);
        // planner->setRange(1.0);

        //设置机器人的初始状态
        ob::ScopedState<MomaStateSpace> start_state(space);
        ob::ScopedState<MomaStateSpace> end_state(space);
        for (size_t i = 0; i < moma_param.dof_num + 3; i++)
        {
            start_state[i] = start[i];
            end_state[i] = end[i];
        }

        auto pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start_state, end_state);
        pdef->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));
        planner->setProblemDefinition(pdef);
        planner->setup();
        
        ob::CostConvergenceTerminationCondition condition1(pdef, 0.005);
        ob::PlannerTerminationCondition condition2 = ob::timedPlannerTerminationCondition(plan_time);
        ob::PlannerTerminationCondition cc = plannerOrTerminationCondition(condition1, condition2);
        // ob::PlannerTerminationCondition cc = plannerAndTerminationCondition(condition1, condition2);
        bool solved = false;
        if (solved = (planner->solve(cc) == ob::PlannerStatus::EXACT_SOLUTION))
        {
            og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();

            // og::PathGeometric reduced_path(*path);
            // og::PathSimplifier pathSimplifier(si);
            // pathSimplifier.reduceVertices(reduced_path);
            // path = &reduced_path;

            og::PathGeometric simplified_path(*path);
            MomaSimplifier simplifier(si);
            if (simplifier.simplifyMax(simplified_path)) {
                path = &simplified_path;
            }

            double path_length = path->length();
            double path_dt = path_length / (PNUM - 1);
            PRINT_GREEN("path length: " << path_length);

            // for(size_t i = 0; i < path->getStateCount() - 1; i++) {
            //     if (!si->checkMotion(path->getState(i), path->getState(i+1))) {
            //         PRINT_RED("Invalid state transition");
            //         break;
            //     }
            // }

            if (use_inter)
            {
                int cnt = 0;
                double temp_time = 0.0;
                ob::State *moma_state = space->allocState();

                for (std::size_t idx = 0; idx < path->getStateCount() - 1; idx++)
                {
                    const MomaStateSpace::StateType *moma_state1 = path->getState(idx)->as<MomaStateSpace::StateType>();
                    const MomaStateSpace::StateType *moma_state2 = path->getState(idx+1)->as<MomaStateSpace::StateType>();

                    // test metric
                    if (false)
                    {
                        default_random_engine eng(0);
                        uniform_real_distribution<double> rand_percent(0.0, 1.0);
                        double t = rand_percent(eng);
                        space->interpolate(moma_state1, moma_state2, t, moma_state);
                        double add = space->distance(moma_state1, moma_state) + space->distance(moma_state, moma_state2);
                        double time = si->distance(moma_state1, moma_state2);
                        if (fabs(add - time) > 1e-4)
                        {
                            PRINT_RED("Error: add time != time");
                        }
                    }

                    double time = si->distance(moma_state1, moma_state2);
                    temp_time += time;
                    while (temp_time > cnt * path_dt && cnt < PNUM - 1)
                    {
                        double temp = temp_time - cnt * path_dt;
                        double t = (double) (time - temp) / (double) time;
                        space->interpolate(moma_state1, moma_state2, t, moma_state);
                        Eigen::VectorXd node(moma_param.dof_num+3);
                        for (size_t j=0; j<moma_param.dof_num+3; j++)
                            node(j) = moma_state->as<MomaStateSpace::StateType>()->values[j];
                        path_list.push_back(node);  
                        cnt++;
                    }
                }
                path_list.push_back(end);  
                space->freeState(moma_state);

                // VectorXd first = path_list.front();
                // for (size_t i = 0; i < path_list.size() && i < 20; i++)
                // {
                //     VectorXd node = path_list[i];
                //     std::cout << std::setprecision(10) << node.transpose() << std::endl;
                //     Vector2d diff = node.head(2) - first.head(2);
                //     std::cout << std::setprecision(10) << diff.transpose() << "\t" << diff.norm() << std::endl;
                //     if (diff.norm() < 1e-1) {
                //         std::cout << "Start and end point are too close, skip the first point" << std::endl;
                //     }
                // }
                
                bool prune_path = false;
                for(auto start_iter = path_list.begin(); start_iter < path_list.end()-2; start_iter++){
                    auto end_iter = start_iter;

                    for(auto next_iter = start_iter + 1;
                        (next_iter != path_list.end()) && (((*start_iter).head(2) - (*next_iter).head(2)).norm() < 1e-1);
                        next_iter++)
                        end_iter = next_iter;

                    if(start_iter == end_iter) {continue;}
                    // std::cout << "start idx: " << start_iter - path_list.begin() << "\tend idx: " << end_iter - path_list.begin() << std::endl;
                    auto mid_iter = (end_iter + 1 < path_list.end()) ? end_iter + 1 : end_iter;

                    double dist_so2 = std::dynamic_pointer_cast<MomaStateSpace>(space)->distSO2(
                        (*start_iter)(2), (*mid_iter)(2));
                    
                    int idx = mid_iter - path_list.begin();
                    // std::cout << "mid idx: " << idx << std::endl;

                    bool detect_u_turn = false;
                    for (auto it = start_iter+1; it != end_iter; it++)
                        if(std::dynamic_pointer_cast<MomaStateSpace>(space)->distSO2((*it)(2), (*mid_iter)(2)) > dist_so2 
                        || std::dynamic_pointer_cast<MomaStateSpace>(space)->distSO2((*it)(2), (*start_iter)(2)) > dist_so2) {
                            detect_u_turn = true;
                            break;
                        }

                    // Detect U turn
                    if (detect_u_turn) {
                        // PRINT_YELLOW("[OMPL] Pruned path due to U-turn");
                        prune_path = true;
                        start_iter = path_list.erase(start_iter+1, end_iter);
                        // PRINT_YELLOW("[OMPL] New end: " << start_iter - path_list.begin());
                    } else {
                        start_iter = end_iter;
                    }
                }
                // if(prune_path) PRINT_YELLOW("[OMPL] Pruned path due to U-turn");

            }
            else
            {
                for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
                {
                    const MomaStateSpace::StateType *moma_state = path->getState(idx)->as<MomaStateSpace::StateType>();

                    Eigen::VectorXd node(moma_param.dof_num+3);
                    for (size_t i=0; i<moma_param.dof_num+3; i++)
                        node(i) = moma_state->values[i];
                    path_list.push_back(node);  
                } 
            }
        }
        else
        {   
            PRINT_RED("Failed to find a solution");
        }

        delete planner;
        
        return solved;
    }
    void OMPLPlanner::reduceVertices(std::vector<Eigen::VectorXd>& path_list) {
        og::PathGeometric path(si);

        std::vector<ob::State*> state_list;
        for (auto& waypoint : path_list) {
            ob::State* state = space->allocState();
            for (size_t i = 0; i < moma_param.dof_num + 3; i++)
                state->as<MomaStateSpace::StateType>()->values[i] = waypoint(i);
            path.append(state);
        };

        og::PathSimplifier pathSimplifier(si);
        pathSimplifier.reduceVertices(path);

        path_list.clear();
        for (std::size_t idx = 0; idx < path.getStateCount(); idx++)
        {
            const MomaStateSpace::StateType *moma_state = path.getState(idx)->as<MomaStateSpace::StateType>();

            Eigen::VectorXd node(moma_param.dof_num+3);
            for (size_t i=0; i<moma_param.dof_num+3; i++){
                node(i) = moma_state->values[i];
            }
            path_list.push_back(node);  
        }
    }

}
     