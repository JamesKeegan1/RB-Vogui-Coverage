#ifndef JOINT_READ_INTERFACE_H
#define JOINT_READ_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{
/** \brief A handle used to read and command a single joint. */
class JointReadHandle : public JointHandle
{
public:
  JointReadHandle() : JointHandle()
  {
  }
  JointReadHandle(const JointHandle& jh) : JointHandle(jh)
  {
  }

  void setCommand(double command)
  {
  }
};

class JointReadInterface : public HardwareResourceManager<JointReadHandle, DontClaimResources>
{
};

/// \ref JointCommandInterface for commanding effort-based joints.
class EffortReadInterface : public JointReadInterface
{
};

/// \ref JointReadInterface for commanding velocity-based joints.
class VelocityReadInterface : public JointReadInterface
{
};

/// \ref JointReadInterface for commanding position-based joints.
class PositionReadInterface : public JointReadInterface
{
};

namespace internal
{
// we have to override this templated methods, because XXXReadInterface must return the same values as XXXJointInterface
template <>
inline std::string demangledTypeName<hardware_interface::EffortReadInterface>()
{
  return demangleSymbol(typeid(hardware_interface::EffortJointInterface).name());
}
inline std::string demangledTypeName(const hardware_interface::EffortReadInterface& val)
{
  return demangleSymbol(typeid(hardware_interface::EffortJointInterface).name());
}
template <>
inline std::string demangledTypeName<hardware_interface::PositionReadInterface>()
{
  return demangleSymbol(typeid(hardware_interface::PositionJointInterface).name());
}
inline std::string demangledTypeName(const hardware_interface::PositionReadInterface& val)
{
  return demangleSymbol(typeid(hardware_interface::PositionJointInterface).name());
}
template <>
inline std::string demangledTypeName<hardware_interface::VelocityReadInterface>()
{
  return demangleSymbol(typeid(hardware_interface::VelocityJointInterface).name());
}
inline std::string demangledTypeName(const hardware_interface::VelocityReadInterface& val)
{
  return demangleSymbol(typeid(hardware_interface::VelocityJointInterface).name());
}
}
}
#endif  // JOINT_READ_INTERFACE_H
