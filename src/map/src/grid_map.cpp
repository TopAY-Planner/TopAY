#include "map/grid_map.h"

using nmoma_planner::random_map::Box;
namespace nmoma_planner
{
    void GridMap:: init(ros::NodeHandle& nh)
    {
        GET_PARAM_OR_THROW(nh, "grid_map/map_size_x", map_size[0]);
        GET_PARAM_OR_THROW(nh, "grid_map/map_size_y", map_size[1]);
        GET_PARAM_OR_THROW(nh, "grid_map/map_size_z", map_size[2]);
        // nh.getParam("grid_map/map_size_x", map_size[0]);
        // nh.getParam("grid_map/map_size_y", map_size[1]);
        // nh.getParam("grid_map/map_size_z", map_size[2]);

        GET_PARAM_OR_THROW(nh, "grid_map/resolution", resolution);
        GET_PARAM_OR_THROW(nh, "agent/mode", mode);
        int number;

        GET_PARAM_OR_THROW(nh, "agent/fixed_sequence", fixed_sequence);
        GET_PARAM_OR_THROW(nh, "grid_map/use_rog", use_rog);

        if (use_rog)
        {
            pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
            rog_map = std::make_shared<rog_map::ROGMap>(nh);
        }
        else
        {
            esdf_2d_pub = nh.advertise<sensor_msgs::PointCloud2>("/esdf_map_2d", 1);
            esdf_3d_pub = nh.advertise<sensor_msgs::PointCloud2>("/esdf_map_3d", 1);
        }
        
        // origin and boundary
        min_boundary = -map_size / 2.0;
        max_boundary = map_size / 2.0;
        min_boundary(2) = 0.0;
        max_boundary(2) = map_size[2];
        map_origin = min_boundary;

        // resolution
        resolution_inv = 1.0 / resolution;

        // voxel num
        voxel_num(0) = ceil(map_size(0) / resolution);
        voxel_num(1) = ceil(map_size(1) / resolution);
        voxel_num(2) = ceil(map_size(2) / resolution);

        // idx
        min_idx = Eigen::Vector3i::Zero();
        max_idx = voxel_num - Eigen::Vector3i::Ones();

        // datas
        buffer_size_2d  = voxel_num(0) * voxel_num(1);
        buffer_size_3d  = buffer_size_2d * voxel_num(2);
        esdf_buffer_2d = vector<double>(buffer_size_2d, 0.0);
        esdf_buffer_2d_inflate = vector<double>(buffer_size_2d, 0.0);
        esdf_buffer_2d_critical = vector<double>(buffer_size_2d, 0.0);
        occ_buffer_2d = vector<char>(buffer_size_2d, 0);
        occ_buffer_2d_critical = vector<char>(buffer_size_2d, 0);
        esdf_buffer_3d = vector<double>(buffer_size_3d, 0.0);
        occ_buffer_3d = vector<char>(buffer_size_3d, 0);
        grid_node_map = new GridNodePtr[buffer_size_3d];
        for (int i=0; i<buffer_size_3d; i++)
            grid_node_map[i] = new GridNode();
        map_ready = false;

        // only use map node in 'planner' mode
        bool use_map_node = mode.compare("planner") == 0;
        if (!use_map_node)
        {
            this->map_gener.init(nh);
            // this->regenerateDesk();
            this->regenerateMap();
            global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/generated_map", 1);
        } 
        
        // MARK: stat_mode
        if (use_map_node && !use_rog)
        {
            cloud_sub = nh.subscribe("/global_map", 1, &GridMap::cloudCallback, this);
            box_sub = nh.subscribe("/global_boxes", 1, &GridMap::boxCallback, this);
        }
        vis_timer = nh.createTimer(ros::Duration(1.0), &GridMap::visCallback, this);
        

        return;
    }

    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int size) 
    {
        int v[size];
        double z[size + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q) k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    void GridMap::updateESDF()
    {
        int rows = voxel_num[0];
        int cols = voxel_num[1];

        Eigen::MatrixXd tmp_buffer;
        Eigen::MatrixXd neg_buffer;
        Eigen::MatrixXi neg_map;
        Eigen::MatrixXd dist_buffer;
        tmp_buffer.resize(rows, cols);
        neg_buffer.resize(rows, cols);
        neg_map.resize(rows, cols);
        dist_buffer.resize(rows, cols);
        /* ========== compute positive DT ========== */
        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            fillESDF(
                [&](int y)
                {
                    return occ_buffer_2d[toAddress2d(x, y)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    dist_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }
        /* ========== compute negative distance ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                if (occ_buffer_2d[toAddress2d(x, y)] == 0)
                {
                    neg_map(x, y) = 1;
                } else if (occ_buffer_2d[toAddress2d(x, y)] == 1)
                {
                    neg_map(x, y) = 0;
                } else 
                {
                    ROS_ERROR("what?");
                }
            }
        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            fillESDF(
                [&](int y)
                {
                    return neg_map(x, y) == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    neg_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }
        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                esdf_buffer_2d[toAddress2d(x, y)] = dist_buffer(x, y);
                if (neg_buffer(x, y) > 0.0)
                    esdf_buffer_2d[toAddress2d(x, y)] += (-neg_buffer(x, y) + resolution);
            }

        // 2d critical
        /* ========== compute positive DT ========== */
        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            fillESDF(
                [&](int y)
                {
                    return occ_buffer_2d_critical[toAddress2d(x, y)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    dist_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }
        /* ========== compute negative distance ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                if (occ_buffer_2d_critical[toAddress2d(x, y)] == 0)
                {
                    neg_map(x, y) = 1;
                } else if (occ_buffer_2d_critical[toAddress2d(x, y)] == 1)
                {
                    neg_map(x, y) = 0;
                } else 
                {
                    ROS_ERROR("what?");
                }
            }
        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            fillESDF(
                [&](int y)
                {
                    return neg_map(x, y) == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    neg_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }
        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                esdf_buffer_2d_critical[toAddress2d(x, y)] = dist_buffer(x, y);
                if (neg_buffer(x, y) > 0.0)
                    esdf_buffer_2d_critical[toAddress2d(x, y)] += (-neg_buffer(x, y) + resolution);
            }

        // 2d critical inflate
        /* ========== compute positive DT ========== */
        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            fillESDF(
                [&](int y)
                {
                    return esdf_buffer_2d_critical[toAddress2d(x, y)] < moma_param.chassis_colli_radius ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    dist_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }
        /* ========== compute negative distance ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                if (esdf_buffer_2d_critical[toAddress2d(x, y)] >= moma_param.chassis_colli_radius)
                {
                    neg_map(x, y) = 1;
                } else if (esdf_buffer_2d_critical[toAddress2d(x, y)] < moma_param.chassis_colli_radius)
                {
                    neg_map(x, y) = 0;
                } else 
                {
                    ROS_ERROR("what?");
                }
            }
        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            fillESDF(
                [&](int y)
                {
                    return neg_map(x, y) == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    neg_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }
        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                esdf_buffer_2d_critical[toAddress2d(x, y)] = dist_buffer(x, y);
                if (neg_buffer(x, y) > 0.0)
                    esdf_buffer_2d_critical[toAddress2d(x, y)] += (-neg_buffer(x, y) + resolution);
            }
        
        // 2d inflate
        /* ========== compute positive DT ========== */
        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            fillESDF(
                [&](int y)
                {
                    return esdf_buffer_2d[toAddress2d(x, y)] < moma_param.chassis_colli_radius ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    dist_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }
        /* ========== compute negative distance ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                if (esdf_buffer_2d[toAddress2d(x, y)] >= moma_param.chassis_colli_radius)
                {
                    neg_map(x, y) = 1;
                } else if (esdf_buffer_2d[toAddress2d(x, y)] < moma_param.chassis_colli_radius)
                {
                    neg_map(x, y) = 0;
                } else 
                {
                    ROS_ERROR("what?");
                }
            }
        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            fillESDF(
                [&](int y)
                {
                    return neg_map(x, y) == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }
        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    neg_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }
        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                esdf_buffer_2d_inflate[toAddress2d(x, y)] = dist_buffer(x, y);
                if (neg_buffer(x, y) > 0.0)
                    esdf_buffer_2d_inflate[toAddress2d(x, y)] += (-neg_buffer(x, y) + resolution);
            }

        // 3d esdf
        std::vector<char> occupancy_buffer_neg = vector<char>(buffer_size_3d, 0);
        std::vector<double> distance_buffer_ = vector<double>(buffer_size_3d, 10000.0);
        std::vector<double> distance_buffer_neg_ = vector<double>(buffer_size_3d, 10000.0);
        std::vector<double> tmp_buffer1_ = vector<double>(buffer_size_3d, 0.0);
        std::vector<double> tmp_buffer2_ = vector<double>(buffer_size_3d, 0.0);

        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            fillESDF(
                [&](int z) {
                    return occ_buffer_3d[toAddress3d(x, y, z)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int z, double val) { tmp_buffer1_[toAddress3d(x, y, z)] = val; }, min_idx[2],
                max_idx[2], max_idx[2]+1);
            }
        }

        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            for (int z = min_idx[2]; z <= max_idx[2]; z++) {
                fillESDF([&](int y) { return tmp_buffer1_[toAddress3d(x, y, z)]; },
                        [&](int y, double val) { tmp_buffer2_[toAddress3d(x, y, z)] = val; }, min_idx[1],
                        max_idx[1], max_idx[1]+1);
            }
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            for (int z = min_idx[2]; z <= max_idx[2]; z++) {
                fillESDF([&](int x) { return tmp_buffer2_[toAddress3d(x, y, z)]; },
                        [&](int x, double val) {
                            distance_buffer_[toAddress3d(x, y, z)] = resolution * std::sqrt(val);
                        },
                        min_idx[0], max_idx[0], max_idx[0]+1);
            }
        }

        /* ========== compute negative distance ========== */
        for (int x = min_idx[0]; x <= max_idx[0]; ++x)
            for (int y = min_idx[1]; y <= max_idx[1]; ++y)
                for (int z = min_idx[2]; z <= max_idx[2]; ++z) {

                    int idx = toAddress3d(x, y, z);
                    if (occ_buffer_3d[idx] == 0) {
                        occupancy_buffer_neg[idx] = 1;
                    } else if (occ_buffer_3d[idx] == 1) {
                        occupancy_buffer_neg[idx] = 0;
                    } else {
                        ROS_ERROR("what?");
                    }
                }

        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            for (int y = min_idx[1]; y <= max_idx[1]; y++) {
                fillESDF(
                    [&](int z) {
                        return occupancy_buffer_neg[toAddress3d(x, y, z)] == 1 ?
                            0 :
                            std::numeric_limits<double>::max();
                    },
                    [&](int z, double val) { tmp_buffer1_[toAddress3d(x, y, z)] = val; }, min_idx[2],
                    max_idx[2], max_idx[2]+1);
            }
        }

        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            for (int z = min_idx[2]; z <= max_idx[2]; z++) {
                fillESDF([&](int y) { return tmp_buffer1_[toAddress3d(x, y, z)]; },
                        [&](int y, double val) { tmp_buffer2_[toAddress3d(x, y, z)] = val; }, min_idx[1],
                        max_idx[1], max_idx[1]+1);
            }
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            for (int z = min_idx[2]; z <= max_idx[2]; z++) {
                fillESDF([&](int x) { return tmp_buffer2_[toAddress3d(x, y, z)]; },
                        [&](int x, double val) {
                            distance_buffer_neg_[toAddress3d(x, y, z)] = resolution * std::sqrt(val);
                        },
                        min_idx[0], max_idx[0], max_idx[0]+1);
            }
        }

        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx[0]; x <= max_idx[0]; ++x)
            for (int y = min_idx[1]; y <= max_idx[1]; ++y)
                for (int z = min_idx[2]; z <= max_idx[2]; ++z) {

                    int idx = toAddress3d(x, y, z);
                    esdf_buffer_3d[idx] = distance_buffer_[idx];
                    if (distance_buffer_neg_[idx] > 0.0)
                        esdf_buffer_3d[idx] += (-distance_buffer_neg_[idx] + resolution);
                }
        
        return;
    }

    void GridMap::boxCallback(const std_msgs::Float32MultiArray msg)
    {
        this->boxes.clear();
        if (msg.data.empty())
            return;
        size_t _size = msg.layout.dim[0].size;
        for(size_t i = 0; i < _size; i++) {
            this->boxes.push_back({
                msg.data[i*8 + 0],
                msg.data[i*8 + 1],
                msg.data[i*8 + 2],
                msg.data[i*8 + 3],
                msg.data[i*8 + 4],
                msg.data[i*8 + 5],
                msg.data[i*8 + 6],
                msg.data[i*8 + 7]
            });
        }
    }

    void GridMap::cloudCallback(const sensor_msgs::PointCloud2 msg)
    {
        if (map_ready)
        {
            return;
        }

	    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
        pcl::PointCloud<pcl::PointXYZI> pc;
        pcl::fromROSMsg(msg, pc);

        for (size_t i=0; i<pc.points.size(); i++)
        {
            Eigen::Vector2i id;
            posToIndex2d(Eigen::Vector2d(pc.points[i].x, pc.points[i].y), id);
            if (isInMap2d(id))
            {
                occ_buffer_2d_critical[toAddress2d(id)] = 1;
                if (pc.points[i].z < moma_param.chassis_height)
                    occ_buffer_2d[toAddress2d(id)] = 1;
            }
            Eigen::Vector3i id3;
            posToIndex3d(Eigen::Vector3d(pc.points[i].x, pc.points[i].y, pc.points[i].z), id3);
            if (isInMap3d(id3))
                occ_buffer_3d[toAddress3d(id3)] = 1;
        }
        
        updateESDF();
        toMsg();

        map_ready = true;

        PRINT_GREEN("Map ready.");
        
        return;
    }

    void GridMap::visCallback(const ros::TimerEvent& /*event*/)
    {
        if (!map_ready)
            return;
        esdf_2d_pub.publish(esdf_cloud_2d);
        esdf_3d_pub.publish(esdf_cloud_3d);

        if (mode.compare("planner") != 0) {
            pcl::toROSMsg(cloud_map, cloud_msg);
            cloud_msg.header.frame_id = "world";
            global_map_pub.publish(cloud_msg);
        }
        return;
    }

    /*
    void GridMap::toFile(void)
    {
        if (stat_mode)
        {
            std::ofstream outfile;
            outfile.open(ros::package::getPath("planner") + stat_file + "map2d_" + stat_number, std::ofstream::app);
            for (int i=0; i<voxel_num(0); i++)
            {
                for (int j=0; j<voxel_num(1); j++)
                {
                    Eigen::Vector2i id(i, j);
                    double occ = occ_buffer_2d[toAddress2d(id)];
                    double esdf = esdf_buffer_2d[toAddress2d(id)];
                    outfile << occ << " " << esdf << " " << std::endl;
                }
            }
            outfile.close();

            outfile.open(ros::package::getPath("planner") + stat_file + "map3d_" + stat_number, std::ofstream::app);
            for (int i=0; i<voxel_num(0); i++)
            {
                for (int j=0; j<voxel_num(1); j++)
                {
                    for (int k=0; k<voxel_num(2); k++)
                    {
                        Eigen::Vector3i id(i, j, k);
                        double occ = occ_buffer_3d[toAddress3d(id)];
                        double esdf = esdf_buffer_3d[toAddress3d(id)];
                        outfile << occ << " " << esdf << " " << std::endl;
                    }
                }
            }
            outfile.close();
        }
    }
    */

    void GridMap::toMsg(void)
    {
        // 2d cloud
        pcl::PointCloud<pcl::PointXYZI> pc;
        /// test esdf computation
        for (int i=0; i<voxel_num(0); i++)
        {
            for (int j=0; j<voxel_num(1); j++)
            {
                Eigen::Vector2i id(i, j);
                Eigen::Vector2d pos;
                indexToPos2d(id, pos);
                
                double esdf = esdf_buffer_2d[toAddress2d(id)];
                esdf = esdf_buffer_2d_inflate[toAddress2d(id)];
                if (esdf < 0.0)
                // if (esdf < moma_param.chassis_colli_radius)
                {
                    pcl::PointXYZI p;
                    p.x = pos(0);
                    p.y = pos(1);
                    p.z = esdf_buffer_2d[toAddress2d(id)];
                    p.intensity = 1.0;
                    pc.push_back(p);
                }
            }
        }
        /// test interpolation 
        // for (double i=min_boundary(0); i<max_boundary(0); i+=0.2*resolution)
        // {
        //     for (double j=min_boundary(1); j<max_boundary(1); j+=0.2*resolution)
        //     {
        //         Eigen::Vector2d pos(i, j);
        //         Eigen::Vector2d grad;
        //         double dist = 0.0;
        //         getDisWithGradI(pos, dist, grad);
        //         pcl::PointXYZI p;
        //         p.x = pos(0);
        //         p.y = pos(1);
        //         p.z = 0.0;
        //         p.intensity = dist;
        //         pc.push_back(p);
        //     }
        // }
        pc.header.frame_id = "world";
        pc.width = pc.points.size();
        pc.height = 1;
        pc.is_dense = true;
        pcl::toROSMsg(pc, esdf_cloud_2d);

        // 3d cloud
        pc.clear();
        /// test esdf computation
        for (int i=0; i<voxel_num(0); i++)
        {
            for (int j=0; j<voxel_num(1); j++)
            {
                for (int k=0; k<voxel_num(2); k++)
                {
                    Eigen::Vector3i id(i, j, k);
                    Eigen::Vector3d pos;
                    indexToPos3d(id, pos);

                    double esdf = esdf_buffer_3d[toAddress3d(id)];
                    if (esdf < 0.2)
                    {
                        pcl::PointXYZI p;
                        p.x = pos(0);
                        p.y = pos(1);
                        p.z = pos(2);
                        p.intensity = esdf_buffer_3d[toAddress3d(id)];
                        pc.push_back(p);
                    }
                }
            }
        }
        pc.header.frame_id = "world";
        pc.width = pc.points.size();
        pc.height = 1;
        pc.is_dense = true;
        pcl::toROSMsg(pc, esdf_cloud_3d);
    }

    void GridMap::regenerateMap(void)
    {
        map_ready = false;
        esdf_buffer_2d = vector<double>(buffer_size_2d, 0.0);
        occ_buffer_2d = vector<char>(buffer_size_2d, 0);
        esdf_buffer_3d = vector<double>(buffer_size_3d, 0.0);
        occ_buffer_3d = vector<char>(buffer_size_3d, 0);
        pcl::PointCloud<pcl::PointXYZ>& pc = cloud_map;
        if(!fixed_sequence)
            std::tie(pc, this->boxes) = map_gener.generateRandomCase();
        else
            std::tie(pc, this->boxes) = map_gener.generateRandomCase(map_seed++);

        pc.width = pc.points.size();
        pc.height = 1;
        pc.is_dense = true;

        for (size_t i=0; i<pc.points.size(); i++)
        {
            Eigen::Vector2i id;
            posToIndex2d(Eigen::Vector2d(pc.points[i].x, pc.points[i].y), id);
            if (isInMap2d(id))
            {
                occ_buffer_2d_critical[toAddress2d(id)] = 1;
                if (pc.points[i].z < moma_param.chassis_height)
                    occ_buffer_2d[toAddress2d(id)] = 1;
            }
            Eigen::Vector3i id3;
            posToIndex3d(Eigen::Vector3d(pc.points[i].x, pc.points[i].y, pc.points[i].z), id3);
            if (isInMap3d(id3))
                occ_buffer_3d[toAddress3d(id3)] = 1;
        }
        updateESDF();
        toMsg();
        map_ready = true;
        esdf_3d_pub.publish(esdf_cloud_3d);
        return;
    }

    void GridMap::regenerateDesk(std::vector<Eigen::Vector2d> spawn)
    {
        map_ready = false;
        esdf_buffer_2d = vector<double>(buffer_size_2d, 0.0);
        occ_buffer_2d = vector<char>(buffer_size_2d, 0);
        esdf_buffer_3d = vector<double>(buffer_size_3d, 0.0);
        occ_buffer_3d = vector<char>(buffer_size_3d, 0);
        pcl::PointCloud<pcl::PointXYZ>& pc = cloud_map;
        // if(!fixed_sequence)
        //     std::tie(pc, this->boxes) = map_gener.generateRandomCase();
        // else
        //     std::tie(pc, this->boxes) = map_gener.generateRandomCase(map_seed++);

        std::vector<Box> spawn_boxes;
        std::transform(spawn.begin(), spawn.end(), std::back_inserter(spawn_boxes), [](const Eigen::Vector2d& p) {
            return Box(Eigen::Vector3d(p(0) - 0.5, p(1) - 0.5, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0), 0.0);
        });
        std::tie(pc, this->boxes) = map_gener.generateDeskCase(spawn_boxes);

        pc.width = pc.points.size();
        pc.height = 1;
        pc.is_dense = true;

        for (size_t i=0; i<pc.points.size(); i++)
        {
            Eigen::Vector2i id;
            posToIndex2d(Eigen::Vector2d(pc.points[i].x, pc.points[i].y), id);
            if (isInMap2d(id))
            {
                occ_buffer_2d_critical[toAddress2d(id)] = 1;
                if (pc.points[i].z < moma_param.chassis_height)
                    occ_buffer_2d[toAddress2d(id)] = 1;
            }
            Eigen::Vector3i id3;
            posToIndex3d(Eigen::Vector3d(pc.points[i].x, pc.points[i].y, pc.points[i].z), id3);
            if (isInMap3d(id3))
                occ_buffer_3d[toAddress3d(id3)] = 1;
        }
        updateESDF();
        toMsg();
        map_ready = true;
        esdf_3d_pub.publish(esdf_cloud_3d);
        return;
    }
    
    void GridMap::loadMap(const vector<char>& occ_2d, const vector<char>& occ_3d) {
        map_ready = false;
        occ_buffer_2d = vector<char>(occ_2d);
        occ_buffer_3d = vector<char>(occ_3d);
        esdf_buffer_2d = vector<double>(buffer_size_2d, 0.0);
        esdf_buffer_3d = vector<double>(buffer_size_3d, 0.0);
        updateESDF();
        map_ready = true;
        return;
    }

    sensor_msgs::PointCloud2 GridMap::getSurround(void)
    {
        if (use_rog)
        {
            return rog_map->public_occ;
        }
        
        pcl::PointCloud<pcl::PointXYZ> pc;
        sensor_msgs::PointCloud2 clouds;

        // 3d cloud
        pc.clear();
        /// test esdf computation
        for (int i=0; i<voxel_num(0); i++)
        {
            for (int j=0; j<voxel_num(1); j++)
            {
                for (int k=0; k<voxel_num(2); k++)
                {
                    Eigen::Vector3i id(i, j, k);
                    Eigen::Vector3d pos;
                    indexToPos3d(id, pos);

                    double esdf = esdf_buffer_3d[toAddress3d(id)];
                    if (fabs(esdf) < 1e-2)
                    {
                        pcl::PointXYZ p;
                        p.x = pos(0);
                        p.y = pos(1);
                        p.z = pos(2);
                        pc.push_back(p);
                    }
                }
            }
        }
        pc.header.frame_id = "world";
        pc.width = pc.points.size();
        pc.height = 1;
        pc.is_dense = true;
        pcl::toROSMsg(pc, clouds);

        return clouds;
    }

    std::pair<std::vector<Eigen::Vector2d>, double> GridMap::astarPlan2d(const Eigen::Vector2d& start, 
                                                                        const Eigen::Vector2d& end,
                                                                        const double& threshold)
    {
        std::vector<Eigen::Vector2d> path;

        if (!map_ready)
            return make_pair(path, 0.0);

        for (int i=0; i<buffer_size_2d; i++)
        {
            grid_node_map[i]->reset();
        }

        if(!isInMap2d(start) || !isInMap2d(end))
        {
            ROS_ERROR("[Astar] boundary points out of map.");
            return make_pair(path, 0.0);
        }

        std::multimap<double, GridNodePtr> openSet;
        Eigen::Vector2i start_index;
        posToIndex2d(start, start_index);
        GridNodePtr start_point = grid_node_map[toAddress2d(start_index)];
        start_point->index = start_index;
        GridNodePtr currentPtr = nullptr;

        openSet.insert(make_pair(0.0, start_point));

        Eigen::Vector2i end_index;
        posToIndex2d(end, end_index);

        while ( !openSet.empty() )
        {
            auto iter  = std::begin(openSet);
            currentPtr = iter -> second;
            openSet.erase(iter);

            grid_node_map[toAddress2d(currentPtr->index)]->id = IN_CLOSE;
 
            if( currentPtr->index == end_index )
            {
                GridNode* p = currentPtr;
                double cost = p->gScore;
                Eigen::Vector2d p_world;
                while (p->parent != nullptr)
                {
                    indexToPos2d(p->index, p_world);
                    path.push_back(p_world);
                    p = p->parent;
                }
                indexToPos2d(p->index, p_world);
                path.push_back(p_world);

                reverse(path.begin(), path.end());

                return make_pair(path, cost);
            }

            Eigen::Vector2i neighbor_index;
            for(int i = -1; i <= 1; i++)
            {
                for(int j = -1; j <= 1; j++)
                {
                    if(i == 0 && j == 0) { continue; }
                    neighbor_index = currentPtr->index + Eigen::Vector2i(i ,j);

                    if(isInMap2d(neighbor_index))
                    {  
                        GridNodePtr neighborPtr = grid_node_map[toAddress2d(neighbor_index)];
                        Eigen::Vector2d neighbor_world;
                        indexToPos2d(neighbor_index, neighbor_world);
                        if (neighborPtr->id == IS_UNKNOWN && !isCollision2d(neighbor_world, threshold))
                        {
                            double tg = ((i * j == 0) ? resolution : (resolution * 1.41)) + currentPtr -> gScore;
                            double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;

                            neighborPtr -> parent = currentPtr;
                            neighborPtr -> gScore = tg;
                            neighborPtr -> fScore = tg + heu;
                            neighborPtr -> index = neighbor_index;
                            neighborPtr -> id = IN_OPEN;
                            openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                        }
                        else if (neighborPtr->id == IN_OPEN)
                        {
                            double tg = ((i * j == 0) ? resolution : (resolution * 1.414)) + currentPtr -> gScore;
                            if (tg < neighborPtr->gScore)
                            {
                                double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;
                                neighborPtr -> parent = currentPtr;
                                neighborPtr -> gScore = tg;
                                neighborPtr -> fScore = tg + heu;
                            }
                        }
                        else
                        {
                            double tg = ((i * j == 0) ? resolution : (resolution * 1.414)) + currentPtr -> gScore;
                            if(tg < neighborPtr -> gScore)
                            {
                                double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;
                                neighborPtr -> parent = currentPtr;
                                neighborPtr -> gScore = tg;
                                neighborPtr -> fScore = tg + heu;
                                
                                neighborPtr -> id = IN_OPEN;
                                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                            }
                        }
                    }
                }
            }   
        }

        ROS_ERROR("[Astar] Fails!!!");
        path.clear();

        return make_pair(path, 0.0);
    }
}