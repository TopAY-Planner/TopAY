#include "random_map_generator/random_map.hpp"

namespace nmoma_random_map = nmoma_planner::random_map;
using nmoma_random_map::Box;

pcl::PointCloud<pcl::PointXYZ> nmoma_random_map::Box::generatePCL(double resolution) const {
    pcl::PointCloud<pcl::PointXYZ> cloud_box;
    int x_num, y_num, z_num;
    x_num = ceil(size.x()/resolution);
    y_num = ceil(size.y()/resolution);
    z_num = ceil(size.z()/resolution);
    pcl::PointXYZ pt;
    for (int i=0; i<x_num; i++)
    for (int j=0; j<y_num; j++)
    for (int k=0; k<z_num; k++)
    {
        pt.x = i * resolution;
        pt.y = j * resolution;
        pt.z = k * resolution;

        double x = pt.x * cos(theta) - pt.y * sin(theta);
        double y = pt.x * sin(theta) + pt.y * cos(theta);

        pt.x = x + pos.x();
        pt.y = y + pos.y();
        pt.z = pt.z + pos.z();

        cloud_box.points.push_back(pt);
    }
    return cloud_box;
}

void nmoma_random_map::RandomPCGenerator::init(
    const vector<int>& obs_num,
    const vector<double>& wall_size_range,
    const vector<double>& wall_height_range,
    const vector<double>& float_size_range,
    const vector<double>& float_height_range,
    double resolution,
    double size_x,
    double size_y,
    double min_obs_dis
) {
    this->obs_num = obs_num;
    this->wall_size_range = wall_size_range;
    this->wall_height_range = wall_height_range;
    this->float_size_range = float_size_range;
    this->float_height_range = float_height_range;
    this->resolution = resolution;
    this->size_x = size_x;
    this->size_y = size_y;
    this->min_obs_dis = min_obs_dis;
    
    rand_x = uniform_real_distribution<double>(-size_x / 2.0, size_x / 2.0);
    rand_y = uniform_real_distribution<double>(-size_y / 2.0, size_y / 2.0);
    rand_wall_size = uniform_real_distribution<double>(wall_size_range[0], wall_size_range[1]);
    rand_wall_height = uniform_real_distribution<double>(wall_height_range[0], wall_height_range[1]);
    rand_float_size = uniform_real_distribution<double>(float_size_range[0], float_size_range[1]);
    rand_float_height = uniform_real_distribution<double>(float_height_range[0], float_height_range[1]);
    rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);

    eng = std::mt19937(rd());

    return;
}

void nmoma_random_map::RandomPCGenerator::init(ros::NodeHandle& nh)
{
    nh.param<std::vector<int>>("/map/obs_num", obs_num, std::vector<int>());
    nh.param<std::vector<double>>("/map/wall_size_range", wall_size_range, std::vector<double>());
    nh.param<std::vector<double>>("/map/wall_height_range", wall_height_range, std::vector<double>());
    nh.param<std::vector<double>>("/map/float_size_range", float_size_range, std::vector<double>());
    nh.param<std::vector<double>>("/map/float_height_range", float_height_range, std::vector<double>());

    nh.param<std::vector<double>>("/map/desk_length_range", desk_length_range, std::vector<double>());
    nh.param<std::vector<double>>("/map/desk_width_range", desk_width_range, std::vector<double>());
    nh.param<std::vector<double>>("/map/desk_height_range", desk_height_range, std::vector<double>());
    nh.param<std::vector<int>>("/map/desk_arrangement_range", desk_arrangement_range, std::vector<int>());

    nh.getParam("/map/resolution", resolution);
    nh.getParam("/map/size_x", size_x);
    nh.getParam("/map/size_y", size_y);
    nh.getParam("/map/min_obs_dis", min_obs_dis);
    
    rand_x = uniform_real_distribution<double>(-size_x / 2.0, size_x / 2.0);
    rand_y = uniform_real_distribution<double>(-size_y / 2.0, size_y / 2.0);
    rand_wall_size = uniform_real_distribution<double>(wall_size_range[0], wall_size_range[1]);
    rand_wall_height = uniform_real_distribution<double>(wall_height_range[0], wall_height_range[1]);
    rand_float_size = uniform_real_distribution<double>(float_size_range[0], float_size_range[1]);
    rand_float_height = uniform_real_distribution<double>(float_height_range[0], float_height_range[1]);
    rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);

    rand_desk_length= uniform_real_distribution<double>(desk_length_range[0], desk_length_range[1]);
    rand_desk_width = uniform_real_distribution<double>(desk_width_range[0], desk_width_range[1]);
    rand_desk_height= uniform_real_distribution<double>(desk_height_range[0], desk_height_range[1]);
    rand_arragement = uniform_int_distribution<int>(desk_arrangement_range[0], desk_arrangement_range[1]);

    eng = std::mt19937(rd());

    return;
}

pcl::PointCloud<pcl::PointXYZ> nmoma_random_map::RandomPCGenerator::generateBox(const Eigen::Vector3d& size)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_box;
    int x_num, y_num, z_num;
    x_num = ceil(size.x()/resolution);
    y_num = ceil(size.y()/resolution);
    z_num = ceil(size.z()/resolution);
    pcl::PointXYZ pt;
    for (int i=0; i<x_num; i++)
        for (int j=0; j<y_num; j++)
            for (int k=0; k<z_num; k++)
            {
                pt.x = i * resolution;
                pt.y = j * resolution;
                pt.z = k * resolution;
                cloud_box.points.push_back(pt);
            }
    return cloud_box;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> nmoma_random_map::RandomPCGenerator::generateDesk(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& size,
    double theta)
{
    double leg_width = 0.05;
    double desktop_thichness = 0.05;
    pcl::PointCloud<pcl::PointXYZ> cloud_desk;

    Eigen::Matrix3d R;
    R << cos(theta), -sin(theta), 0.0,
         sin(theta),  cos(theta), 0.0,
         0.0,         0.0,         1.0;
         
    std::vector<Box> boxes;
    std::array<Eigen::Vector3d, 4> corners;
    corners[0] = pos;
    corners[1] = pos + R * Eigen::Vector3d(size.x() - leg_width, 0.0, 0.0);
    corners[2] = pos + R * Eigen::Vector3d (0.0, size.y() - leg_width, 0.0);
    corners[3] = pos + R * Eigen::Vector3d(size.x() - leg_width, size.y() - leg_width, 0.0);

    Eigen::Vector3d leg_size(leg_width, leg_width, size(2));
    boxes.push_back(Box(corners[0], leg_size, theta));
    boxes.push_back(Box(corners[1], leg_size, theta));
    boxes.push_back(Box(corners[2], leg_size, theta));
    boxes.push_back(Box(corners[3], leg_size, theta));

    // desktop
    Eigen::Vector3d desktop_size(size(0), size(1), desktop_thichness);
    boxes.push_back(Box(Eigen::Vector3d(pos(0), pos(1), size(2)), desktop_size, theta));

    
    std::vector<Box::array_repr> box_array_repr(boxes.size());
    for (Box& box : boxes) {
        auto cloud = box.generatePCL(resolution);
        cloud_desk.points.insert(cloud_desk.points.end(), cloud.points.begin(), cloud.points.end());
        box_array_repr.push_back(box.toArray());        
    }

    return std::make_pair(cloud_desk, box_array_repr);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> nmoma_random_map::RandomPCGenerator::generateDesk(
    const Eigen::Vector3d& pos,
    const Eigen::Vector3d& size,
    double theta, 
    const Eigen::Vector2i& arrangement)
{
    pcl::PointCloud<pcl::PointXYZ> ret_cloud;
    std::vector<Box::array_repr> ret_box;

    Eigen::Matrix3d rotation_matrix;

    rotation_matrix << 
        cos(theta), -sin(theta), 0, 
        sin(theta),  cos(theta), 0, 
        0, 0, 1;

    for (int r = 0; r < arrangement(0); r++)
    for (int c = 0; c < arrangement(1); c++)
    {
        Eigen::Vector3d pos_desk(pos(0), pos(1), pos(2));
        pos_desk += rotation_matrix * Eigen::Vector3d(r * size(0), c * size(1), 0.0);
        pcl::PointCloud<pcl::PointXYZ> cloud_desk;
        std::vector<Box::array_repr> box_array_repr;
        std::tie(cloud_desk, box_array_repr) = generateDesk(pos_desk, size, theta);
        ret_cloud.points.insert(ret_cloud.points.end(), cloud_desk.points.begin(), cloud_desk.points.end());
        ret_box.insert(ret_box.end(), box_array_repr.begin(), box_array_repr.end());
    }

    return std::make_pair(ret_cloud, ret_box);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> 
nmoma_random_map::RandomPCGenerator::generateDeskCase() {
    std::vector<Box> obs_boxes;
    obs_boxes.push_back(Box(
        Eigen::Vector3d(-0.5, -0.5, 0.0),
        Eigen::Vector3d::Ones(),
        0.0
    ));
    return generateDeskCase(obs_boxes);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> 
nmoma_random_map::RandomPCGenerator::generateDeskCase(std::vector<Box> obs) {
    std::vector<Eigen::Vector2d> centers;
    pcl::PointCloud<pcl::PointXYZ> cloud_map;
        
    pcl::PointXYZ pt_random;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_box = generateBox(Eigen::Vector3d(size_x, resolution*2.0, 1.0));
    for (size_t i=0; i<cloud_box.points.size(); i++)
    {
        pt_random.x = cloud_box.points[i].x - size_x / 2.0 - resolution;
        pt_random.y = cloud_box.points[i].y + size_y / 2.0 - resolution;
        pt_random.z = cloud_box.points[i].z;
        cloud_map.points.push_back(pt_random);
        pt_random.y = cloud_box.points[i].y - size_y / 2.0 - resolution;
        cloud_map.points.push_back(pt_random);
    }
    cloud_box = generateBox(Eigen::Vector3d(resolution*2.0, size_y, 1.0));
    for (size_t i=0; i<cloud_box.points.size(); i++)
    {
        pt_random.x = cloud_box.points[i].x + size_x / 2.0 - resolution;
        pt_random.y = cloud_box.points[i].y - size_y / 2.0 - resolution;
        pt_random.z = cloud_box.points[i].z;
        cloud_map.points.push_back(pt_random);
        pt_random.x = cloud_box.points[i].x - size_x / 2.0 - resolution;
        cloud_map.points.push_back(pt_random);
    }
    std::vector<Box::array_repr> obs_array_repr;
    obs_array_repr.clear();

    std::vector<Box> obs_boxes;
    // obs_boxes.push_back(Box(
    //     Eigen::Vector3d(-0.5, -0.5, 0.0),
    //     Eigen::Vector3d::Ones(),
    //     0.0
    // ));
    std::transform(obs.begin(), obs.end(), std::back_inserter(obs_boxes), [](const Box& box) {return box;});

    // generate desk obs
    for (int i = 0; i < obs_num[0]; i++) {
        double x(rand_x(eng)), y(rand_y(eng));
        x = floor(x / resolution) * resolution + resolution / 2.0;
        y = floor(y / resolution) * resolution + resolution / 2.0;


        double size_x = rand_desk_width(eng);
        double size_y = rand_desk_length(eng);
        double height = rand_desk_height(eng);
        double theta = 0.0;
        double row_arrangement = rand_arragement(eng);
        double col_arrangement = rand_arragement(eng);


        Box test_box = Box(
            Eigen::Vector3d(x, y, 0.0),
            Eigen::Vector3d(size_x * row_arrangement, size_y * col_arrangement, height),
            theta
        );

        bool collision = false; 

        for (auto &other : obs_boxes) if(collision = test_box.overlap(other)) {
            i--;
            break;
        }
        if (collision) continue;
        
        obs_boxes.push_back(test_box);
        pcl::PointCloud<pcl::PointXYZ> cloud_desk;
        std::vector<Box::array_repr> box_array_repr;
        std::tie(cloud_desk, box_array_repr) = generateDesk(
            Eigen::Vector3d(x, y, 0.0),
            Eigen::Vector3d(size_x, size_y, height),
            theta,
            Eigen::Vector2i(row_arrangement, col_arrangement)
        );
        cloud_map.points.insert(cloud_map.points.end(), cloud_desk.points.begin(), cloud_desk.points.end());
    }

    // generate wall
    for (int i = 0; i < obs_num[1]; i++) {
        double x(rand_x(eng)), y(rand_y(eng));
        x = floor(x / resolution) * resolution + resolution / 2.0;
        y = floor(y / resolution) * resolution + resolution / 2.0;

        Eigen::Vector3d box_size;
        box_size.x() = rand_wall_size(eng);
        box_size.y() = rand_wall_size(eng);
        box_size.z() = rand_wall_height(eng);
        // box_size.z() = rand_wall_height(eng);
        // double height = rand_float_height(eng);

        Eigen::Vector3d box_pos(x, y, 0.0);
        Box box(box_pos, box_size, 0.0);
        bool collision = false;
        for (auto& other : obs_boxes) if(collision = box.overlap(other)) {i--;break;}
        if (collision) continue;

        obs_boxes.push_back(box);
        pcl::PointCloud<pcl::PointXYZ> cloud_box = box.generatePCL(resolution);

        for (size_t i=0; i<cloud_box.points.size(); i++)
        {
            pt_random.x = cloud_box.points[i].x;
            pt_random.y = cloud_box.points[i].y;
            pt_random.z = cloud_box.points[i].z;
            
            cloud_map.points.push_back(pt_random);
        }
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;

    // obs_array_repr.insert(obs_array_repr.end(), box_array_repr.begin(), box_array_repr.end());

    return make_pair(cloud_map, obs_array_repr);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>>
nmoma_random_map::RandomPCGenerator::generateRandomCase()
{
    // random_device rd;
    // eng = default_random_engine(rd());
    return this->generataRandomCaseAux();
}

std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>>
nmoma_random_map::RandomPCGenerator::generateRandomCase(unsigned int seed)
{
    // eng = default_random_engine(seed);
    return this->generataRandomCaseAux();
}

std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>>
nmoma_random_map::RandomPCGenerator::generataRandomCaseAux()
{
    std::vector<Eigen::Vector2d> centers;
    pcl::PointCloud<pcl::PointXYZ> cloud_map;
        
    pcl::PointXYZ pt_random;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_box = generateBox(Eigen::Vector3d(size_x, resolution*2.0, 1.0));
    for (size_t i=0; i<cloud_box.points.size(); i++)
    {
        pt_random.x = cloud_box.points[i].x - size_x / 2.0 - resolution;
        pt_random.y = cloud_box.points[i].y + size_y / 2.0 - resolution;
        pt_random.z = cloud_box.points[i].z;
        cloud_map.points.push_back(pt_random);
        pt_random.y = cloud_box.points[i].y - size_y / 2.0 - resolution;
        cloud_map.points.push_back(pt_random);
    }
    cloud_box = generateBox(Eigen::Vector3d(resolution*2.0, size_y, 1.0));
    for (size_t i=0; i<cloud_box.points.size(); i++)
    {
        pt_random.x = cloud_box.points[i].x + size_x / 2.0 - resolution;
        pt_random.y = cloud_box.points[i].y - size_y / 2.0 - resolution;
        pt_random.z = cloud_box.points[i].z;
        cloud_map.points.push_back(pt_random);
        pt_random.x = cloud_box.points[i].x - size_x / 2.0 - resolution;
        cloud_map.points.push_back(pt_random);
    }
    
    Box spawn_box(
        Eigen::Vector3d::Ones() * -0.5,
        Eigen::Vector3d::Ones(),
        0.0
    );
    std::vector<Box> obs_boxes; // put everthing boxes except walls here
    // generate wall obs
    for (size_t k=0; k<2; k++)
    for (int j = 0; j < obs_num[k]; j++) 
    {
        double x, y;
        x = rand_x(eng);
        y = rand_y(eng);

        x = floor(x / resolution) * resolution + resolution / 2.0;
        y = floor(y / resolution) * resolution + resolution / 2.0;

        Eigen::Vector3d box_size;
        double height = 0.0;
        if (k==0)
        {
            box_size.x() = rand_wall_size(eng);
            box_size.y() = rand_wall_size(eng);
            box_size.z() = rand_wall_height(eng);
        }
        else if (k==1)
        {
            box_size.x() = rand_float_size(eng);
            box_size.y() = rand_float_size(eng);
            box_size.z() = rand_float_size(eng);
            height = rand_float_height(eng);
        }
        
        Eigen::Vector3d box_pos(x, y, height);
        double theta = 0.0;
        Box box(box_pos, box_size, theta);

        bool collision = false;
        for (auto &other : obs_boxes) if(collision = box.overlap(other)) break;
        if (collision || box.overlap2d(spawn_box)) {j--;continue;}

        pcl::PointCloud<pcl::PointXYZ> cloud_box = box.generatePCL(resolution);
        obs_boxes.push_back(box);

        for (size_t i=0; i<cloud_box.points.size(); i++)
        {
            pt_random.x = cloud_box.points[i].x;
            pt_random.y = cloud_box.points[i].y;
            pt_random.z = cloud_box.points[i].z;
            
            float free_range = 0.5;
            if (pt_random.x > -free_range &&
                pt_random.x < free_range &&
                pt_random.y > -free_range &&
                pt_random.y < free_range)
            {
                continue;
            }
            cloud_map.points.push_back(pt_random);
        }
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;

    std::vector <Box::array_repr> obs_array_repr(obs_boxes.size());
    std::transform(obs_boxes.begin(), obs_boxes.end(),
                   obs_array_repr.begin(),
                   [](const Box& box) { return box.toArray(); });

    return make_pair(cloud_map, obs_array_repr);
}