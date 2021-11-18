//
// Created by andi on 20-09-19.
//

#ifndef REVOLVE_DIFFERENTIALCPGCLEAN_H
#define REVOLVE_DIFFERENTIALCPGCLEAN_H

#include <revolve/brains/controller/actuators/Actuator.h>
#include <revolve/brains/controller/DifferentialCPG.h>
#include "Brain.h"

namespace revolve
{
    namespace gazebo
    {
        /// \brief connection between gazebo and revolve CPG
        /// \details gets the sdf - model data and passes them to revolve
        class DifferentialCPGClean: public revolve::DifferentialCPG
        {
        public:
            /// \brief Constructor
            /// \param[in] brain_sdf ElementPtr containing the "brain" - tag of the model sdf
            /// \param[in] _motors vector<MotorPtr> list of motors
            /// \details Extracts controller parameters
            ///  from brain_sdf and calls revolve::DifferentialCPG's contructor.
            explicit DifferentialCPGClean(const sdf::ElementPtr brain_sdf,
                                          const std::vector< MotorPtr > &_motors,
                                          std::shared_ptr<revolve::AngleToTargetDetector> angle_to_target_sensor = nullptr);

            explicit DifferentialCPGClean(const sdf::ElementPtr brain_sdf, const std::vector<MotorPtr> &_motors,
                                 std::shared_ptr<AngleToTargetDetector> angle_to_target_sensor,
                                 bool use_secondary_brain);

        protected:
            explicit DifferentialCPGClean(const sdf::ElementPtr brain_sdf,
                    const std::vector<MotorPtr> &_motors,
                    const NEAT::Genome &genome,
                    std::shared_ptr<revolve::AngleToTargetDetector> angle_to_target_sensor = nullptr);

/// \brief extracts CPG controller parameters from brain_sdf
            /// \param[in] brain_sdf ElementPtr containing the "brain" - tag of the model sdf
            /// \return parameters of the CPG controller
            /// \details get the strings of the controller parameters and convert them to the
            /// appropriate datatype. Store them in a revolve::DifferentialCPG::ControllerParams
            /// struct and return them.
            static revolve::DifferentialCPG::ControllerParams load_params_from_sdf(sdf::ElementPtr brain_sdf, bool use_secondary_brain);
        };
    }
}

#endif //REVOLVE_DIFFERENTIALCPGCLEAN_H
