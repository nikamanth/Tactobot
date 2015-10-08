#ifndef SSHMOBJWAM_HPP_
#define SSHMOBJWAM_HPP_

#define DOF_ARM 7

#ifdef ENABLE_BASE
#define DOF_BASE 2
#else
#define DOF_BASE 0
#endif

#ifdef ENABLE_HAND
#define DOF_HAND 3
#else
#define DOF_HAND 0
#endif

#define DOF_WAM (DOF_BASE+DOF_HAND+DOF_ARM)

namespace scl-wam{

class SShmObjWAM
{
public:
        SShmObjWAM(){}

        // Joint positions
        double q_[DOF_WAM];

        // Joint velocities
        double dq_[DOF_WAM];

        // Joint accelerations
        double ddq_[DOF_WAM];

        // Joint forces (forces in general coordinates)
        double f_q_[DOF_WAM];

        // Time as recorded by the server;
        double server_time;

        // If the current experiment should be logged
        bool log_enabled;
};

}

#endif
