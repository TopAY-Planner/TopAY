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

// For shared memory
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

#define SHM_SIZE 1024 << 20 // 1GB

namespace nmoma_planner {
namespace shared_data {

    typedef std::array<float, 10> State;
    typedef std::array<float, 8> Box;
    
    class DataPoint {
        private:
        std::array<float, 160000> esdf_3d;
        std::array<State, 64> traj;
        std::array<Box, 50> boxes;
        unsigned int box_num;

        public:
        // constructors
        DataPoint() {}

        DataPoint(
            const std::vector<double>& esdf_3d,
            const std::vector<Box>& boxes,
            const std::vector<Eigen::VectorXd>& traj
        ) {
            box_num = boxes.size();
            // boxes representations
            for(size_t i = 0; i < boxes.size(); i++)
                this->boxes[i] = boxes[i];

            // esdf_3d representation
            for (size_t i = 0; i < esdf_3d.size(); i++)
                this->esdf_3d[i] = static_cast<float>(esdf_3d[i]);

            for(size_t i = 0; i < traj.size(); i++)
            for(int j = 0; j < 10; j++)
                this->traj[i][j] = traj[i](j);
        }

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version) {
            ar & esdf_3d;
            ar & boxes;
            ar & traj;
        }

        // getter functions
        const std::array<float, 160000>& getESDF3D() const { return esdf_3d; }
        const std::array<State, 64>& getTraj() const { return traj; }
        const std::array<Box, 50>& getBoxes() const { return boxes;}
        int getBoxNum() const { return box_num; }
    };

    template<int dof>
    std::size_t hash_value(const DataPoint& data_point) {
        std::size_t seed = 0;
        boost::hash_combine(seed, data_point.getBoxes());
        boost::hash_combine(seed, data_point.getTraj());
        return seed;
    }

    namespace bip = boost::interprocess;
    typedef bip::allocator<DataPoint, bip::managed_shared_memory::segment_manager> DataPointAllocator;
    typedef bip::deque<DataPoint, DataPointAllocator> SharedDataQueue;

    struct SharedData {
        bip::interprocess_mutex mutex;
        bip::interprocess_condition cond_free;
        bip::interprocess_condition cond_any;

        bool ready = false;
        bool done = false;
        unsigned int active_producers = 0;
    };

    class SharedDataProducer {
        private:
        bip::managed_shared_memory msm;
        SharedDataQueue* pQueue;
        unsigned int max_size;
        SharedData* pSharedData;
        std::string shm_name;

        public:
        SharedDataProducer (const std::string& name, unsigned int max_size) :
            msm(bip::open_or_create, name.c_str(), SHM_SIZE), max_size(max_size), shm_name(name)
        {
            DataPointAllocator alloc(msm.get_segment_manager());
            pQueue =    msm.find_or_construct<SharedDataQueue>("queue")(alloc);
            pSharedData = msm.find_or_construct<SharedData>("shared_data")();
            bip::scoped_lock<bip::interprocess_mutex> lock(pSharedData->mutex);
            std::cout << "Current active producers: " << pSharedData->active_producers << std::endl;
            if(pSharedData->active_producers == 0) {
                pQueue->clear();
                pSharedData->ready = true;
                pSharedData->done = false;
            }

            pSharedData->active_producers++;
        }
        
        ~SharedDataProducer() {
            bip::scoped_lock<bip::interprocess_mutex> lock(pSharedData->mutex);
            pSharedData->active_producers--;
            if (pSharedData->active_producers == 0) {
                lock.unlock();
                msm.destroy_ptr(pQueue);
                msm.destroy_ptr(pSharedData);
                bip::shared_memory_object::remove(shm_name.c_str());
                std::cout << "All producers have stopped, shared memory removed." << std::endl;
            }
        }

        bool produce(const DataPoint& data_point) {
            bip::scoped_lock<bip::interprocess_mutex> lock(pSharedData->mutex);
            
            while (pQueue->size() >= max_size && !pSharedData->done)
                pSharedData->cond_free.wait(lock);

            if (pSharedData->done) return false;

            pQueue->push_back(data_point);
            pSharedData->cond_any.notify_one();
            return true;
        }
    };

    class SharedDataConsumer {
        private:
        bip::managed_shared_memory msm;
        SharedData* pSharedData;
        SharedDataQueue* pQueue;

        public:
        SharedDataConsumer (const std::string& name) :
            msm(bip::open_or_create, name.c_str(), SHM_SIZE)
        {
            DataPointAllocator alloc(msm.get_segment_manager());
            pQueue =    msm.find_or_construct<SharedDataQueue>("queue")(alloc);
            pSharedData = msm.find_or_construct<SharedData>("shared_data")();
        }

        DataPoint consume() {
            bip::scoped_lock<bip::interprocess_mutex> lock(pSharedData->mutex);

            while (pQueue->empty() && !pSharedData->done)
                pSharedData->cond_any.wait(lock);
            
            if (pSharedData->done) throw std::runtime_error("No more data points to consume");
            if (pQueue->empty()) throw std::runtime_error("No more data points to consume");

            DataPoint datapoint = pQueue->back();
            pQueue->pop_back();
            pSharedData->cond_free.notify_one();
            return datapoint;
        }

        void signal_done() {
            bip::scoped_lock<bip::interprocess_mutex> lock(pSharedData->mutex);
            pSharedData->done = true;
            pSharedData->cond_free.notify_all();
        }

        unsigned int getProducerNum() const {
            bip::scoped_lock<bip::interprocess_mutex> lock(pSharedData->mutex);
            return pSharedData->active_producers;
        }

        ~SharedDataConsumer() {}
    };
}
}
