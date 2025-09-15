#include "planner/pinocchio_ik.h"

void PinocchioIK::init(ros::NodeHandle& nh)
{
    nh.getParam("ik/eps", eps);
    nh.getParam("ik/IT_MAX", IT_MAX);
    nh.getParam("ik/DT", DT);
    nh.getParam("ik/damp", damp);
    std::string urdf_filename = ros::package::getPath("fake_moma") + "/urdf/rm_75.urdf";
    pinocchio::urdf::buildModel(urdf_filename, model);
    data = pinocchio::Data(model);
    return;
}

std::pair<bool, Eigen::VectorXd> PinocchioIK::solveIK(const Eigen::Matrix3d& R, const Eigen::Vector3d& p)
{
    Eigen::VectorXd q = pinocchio::neutral(model);
    const pinocchio::SE3 oMdes(R, p);
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model.nv);

    for (int i = 0;; i++)
    {
        pinocchio::forwardKinematics(model, data, q);
        const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);
        err = pinocchio::log6(iMd).toVector(); // in joint frame
        if (err.norm() < eps)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            break;
        }
        pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J); // J in joint frame
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(iMd.inverse(), Jlog);
        J = -Jlog * J;
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(model, q, v * DT);
        // if (!(i % 10))
        //     std::cout << i << ": error = " << err.transpose() << std::endl;
    }

    if (success)
    {
        std::cout << "Convergence achieved!" << std::endl;
    }
    else
    {
        std::cout
        << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
        << std::endl;
    }

    std::cout << "\nresult: " << q.transpose() << std::endl;
    std::cout << "\nfinal error: " << err.transpose() << std::endl;

    for (int i=0; i<model.nv; i++)
    {
        while (q(i) < 0.0) q(i) += 2*M_PI;
        while (q(i) > 2*M_PI) q(i) -= 2*M_PI;
        if (q(i) > M_PI) q(i) -= 2*M_PI;
    }
    return std::make_pair(success, q);
}