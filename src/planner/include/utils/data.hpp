/*
This file contains code for serialization and hashing data.
*/
#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Eigen>

#include <boost/functional/hash.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/version.hpp>


namespace nmoma_planner {
namespace data {

    typedef std::array<float, 10> State;
    typedef std::array<float, 8> Box;
    template<int dof>
    class DataPoint {
        private:
        std::size_t hash = 0;
        std::vector<char> occ_2d;
        std::vector<char> occ_3d;
        std::vector<double> esdf_2d; 
        std::vector<float> esdf_3d; 
        std::vector<State> traj;
        std::vector<Box> boxes;

        public:
        // constructors
        DataPoint() {}
        DataPoint(std::string file_path) {
            std::ifstream ifs(file_path);
            if (!ifs.is_open()) { throw std::runtime_error("Failed to open file: " + file_path); }
            boost::archive::text_iarchive ia(ifs);
            ia >> *this;
            ifs.close();
        }
        // DataPoint(
        //     const std::vector<char>& occ_2d,
        //     const std::vector<char>& occ_3d, 
        //     const std::vector<double>& esdf_2d, 
        //     const std::vector<double>& esdf_3d, 
        //     const std::vector<Eigen::VectorXd>& traj
        // ) : 
        //     occ_2d(occ_2d),
        //     occ_3d(occ_3d),
        //     esdf_2d(esdf_2d),
        //     esdf_3d(esdf_3d)
        // {
        //     for (const Eigen::VectorXd& state : traj) {
        //         std::array<double, dof> state_arr;
        //         for (int i = 0; i < dof; i++) {
        //             state_arr[i] = state(i);
        //         }
        //         this->traj.push_back(state_arr);
        //     }
        // }

        DataPoint(
            const std::vector<double>& esdf_3d,
            const std::vector<Eigen::VectorXd>& traj
        )
        {
            // esdf
            this->esdf_3d.resize(esdf_3d.size());
            std::transform(
                esdf_3d.begin(),
                esdf_3d.end(),
                this->esdf_3d.begin(),
                [](double d) { return static_cast<float>(d); }
            );
            
            // trajectory
            for (const Eigen::VectorXd& state : traj) {
                std::array<float, dof> state_arr;
                for (int i = 0; i < dof; i++) {
                    state_arr[i] = static_cast<float>(state(i));
                }
                this->traj.push_back(state_arr);
            }
        }

        DataPoint(
            const std::vector<double>& esdf_3d,
            const std::vector<std::array<float, 8>>& boxes,
            const std::vector<Eigen::VectorXd>& traj
        )
        {
            // esdf
            this->esdf_3d.resize(esdf_3d.size());
            std::transform(
                esdf_3d.begin(),
                esdf_3d.end(),
                this->esdf_3d.begin(),
                [](double d) { return static_cast<float>(d); }
            );

            // boxes representations
            for (const Box& box : boxes) {
                this->boxes.push_back(box);
            }
            
            // trajectory
            for (const Eigen::VectorXd& state : traj) {
                std::array<float, dof> state_arr;
                for (int i = 0; i < dof; i++) {
                    state_arr[i] = static_cast<float>(state(i));
                }
                this->traj.push_back(state_arr);
            }
        }
        
        void setHash(std::size_t hash) { this->hash = hash; }

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version) {
            if (version == 0) {
                ar & occ_2d;
                ar & occ_3d;
                ar & esdf_2d;
            }
            if (version == 3) {
                ar & boxes;
            }
            if (version == 3) {
                ar & boxes;
            }
            ar & esdf_3d;
            ar & traj;
            ar & hash;
        }
        
        // getter functions
        // const std::vector<char>& getOcc2D() const { return occ_2d; }
        // const std::vector<char>& getOcc3D() const { return occ_3d; }
        // const std::vector<double>& getESDF2D() const { return esdf_2d; }
        const std::vector<float>& getESDF3D() const { return esdf_3d; }
        const std::vector<State>& getTraj() const { return traj; }
        const std::vector<Box>& getBoxes() const { return boxes;}
        const std::size_t getHash() const { return hash; }
    };

    template<int dof>
    std::size_t hash_value(const DataPoint<dof>& data_point) {
        std::size_t seed = 0;
        // boost::hash_combine(seed, data_point.getOcc2D());
        // boost::hash_combine(seed, data_point.getOcc3D());
        // boost::hash_combine(seed, data_point.getESDF2D());
        boost::hash_combine(seed, data_point.getESDF3D());
        boost::hash_combine(seed, data_point.getTraj());
        return seed;
    }

    template<int dof>
    std::vector<DataPoint<dof>> load_binary_datapoints(const std::string& file_path) {
        std::vector<DataPoint<dof>> data_points;
        std::ifstream ifs(file_path);
        if (!ifs.is_open()) { throw std::runtime_error("Failed to open file: " + file_path); }
        
        boost::archive::binary_iarchive ia(ifs);
        ifs.peek();
        while (!ifs.eof()) {
            DataPoint<dof> data_point;
            ia >> data_point;
            data_points.push_back(data_point);
            std::cout << "Loaded data point" << std::endl;
            ifs.peek();
        }

        ifs.close();
        return data_points;
    }


    template<int dof>
    class DataLoader {
        private:
        int pos = 0;
        int length = 0;
        std::ifstream ifs;
        boost::archive::binary_iarchive ia;

        public:
        DataLoader(std::string file_path) : 
            ifs(file_path),
            ia(ifs)
        {
            if (!ifs.is_open()) { throw std::runtime_error("Failed to open file: " + file_path); }
            ia >> length;
        }

        ~DataLoader() { ifs.close(); }

        bool has_next() { return pos < length; }

        int size() { return length; }

        DataPoint<dof> next() {
            if(!has_next()) throw std::runtime_error("No more data points to load");
            DataPoint<dof> data_point;
            ia >> data_point;
            pos++;
            return data_point;
        }
    };    

}
}

namespace boost {
namespace serialization {
    template<int dof>
    struct version<nmoma_planner::data::DataPoint<dof>>
    {
        typedef mpl::int_<3> type;
        typedef mpl::integral_c_tag tag;
        BOOST_STATIC_CONSTANT(unsigned int, value = 3);
    };
}
}