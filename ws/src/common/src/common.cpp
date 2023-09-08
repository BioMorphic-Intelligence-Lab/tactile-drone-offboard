#include "common.hpp"

namespace common
{
    const Eigen::Quaterniond NED_ENU_Q = 
        quaternion_from_euler(M_PI, 0.0, M_PI_2);

    Eigen::Quaterniond quaternion_from_euler(const Eigen::Vector3d &euler)
    {
        // YPR is ZYX axes
        return Eigen::Quaterniond(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
    }

    Eigen::Quaterniond quaternion_from_euler(const double roll, const double pitch, const double yaw)
    {
        return quaternion_from_euler(Eigen::Vector3d(roll, pitch, yaw));
    }
    Eigen::Vector3d enu_2_ned(const Eigen::Vector3d &enu)
    {
        Eigen::Vector3d ned(enu.y(), enu.x(), -enu.z());
        return ned;
    }
    Eigen::Vector3d ned_2_enu(const Eigen::Vector3d &ned)
    {
        Eigen::Vector3d enu(ned.y(), ned.x(), -ned.z());
        return enu;

    }
    Eigen::Quaterniond enu_2_ned(const Eigen::Quaterniond &enu)
    {
        Eigen::Quaternion ned = NED_ENU_Q * enu;
        return ned;
    }
    Eigen::Quaterniond ned_2_enu(const Eigen::Quaterniond &ned)
    {
        Eigen::Quaternion enu = NED_ENU_Q * ned;
        return enu;
    }

}