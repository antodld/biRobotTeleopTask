// associated header
#include "../../include/Tasks/biRobotTeleopTask.h"
// includes
// std
#include <cmath>
#include <iterator>
#include <set>

// Eigen
#include <Eigen/Geometry>

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>


namespace tasks
{

namespace qp
{

biRobotTeleopTask::biRobotTeleopTask(const std::vector<rbd::MultiBody> & mbs,
                                                    int r1Index,
                                                    int r2Index,
                                                    double stiffness,
                                                    double weight)
:Task(weight), alphaDBegin_(-1), stiffness_(stiffness), stiffnessSqrt_(2. * std::sqrt(stiffness)),
   posInQ_(2, -1), robotIndexes_({r1Index, r2Index}),Q_(), C_(),
  CSum_(Eigen::Vector6d::Zero()), preQ_()
{
  assert(r2Index < mbs.sze() && r1Index < mbs.size());
  dimWeight_ = Eigen::Vector6d::Ones() * weight;
  dimWeight_.segment(0,3).setZero();
  int maxDof = 0;
  for(int r : robotIndexes_) { maxDof = std::max(maxDof, mbs[r].nrDof()); }
  preQ_.resize(6, maxDof);
}

void biRobotTeleopTask::stiffness(double stiffness)
{
  stiffness_ = stiffness;
  stiffnessSqrt_ = 2. * std::sqrt(stiffness);
}

void biRobotTeleopTask::dimWeight(const Eigen::Vector6d & dim)
{
  dimWeight_ = dim;
  dimWeight_.segment(0,3).setZero();
}

void biRobotTeleopTask::updateNrVars(const std::vector<rbd::MultiBody> & /* mbs */, const SolverData & data)
{
  auto minMaxIndex = std::minmax_element(robotIndexes_.begin(), robotIndexes_.end());
  alphaDBegin_ = data.alphaDBegin(*(minMaxIndex.first));
  int lastBegin = data.alphaDBegin(*(minMaxIndex.second));
  int lastAlphaD = data.alphaD(*(minMaxIndex.second));
  int size = lastBegin + lastAlphaD - alphaDBegin_;

  Q_.setZero(size, size);
  C_.setZero(size);

  posInQ_.clear();
  for(int r : robotIndexes_) { posInQ_.push_back(data.alphaDBegin(r) - alphaDBegin_); }
}

void biRobotTeleopTask::update(const std::vector<rbd::MultiBody> & mbs,
                                     const std::vector<rbd::MultiBodyConfig> & mbcs,
                                     const SolverData & data)
{

  CSum_.noalias() = stiffness_ * eval_;
  CSum_.noalias() += stiffnessSqrt_ * speed_;
  CSum_.noalias() += normalAcc_;

  // first we set to zero used part of Q and C
  for(int i = 0; i < int(posInQ_.size()); ++i)
  {
    int r = robotIndexes_[i];
    int begin = posInQ_[i];
    int dof = data.alphaD(r);
    Q_.block(begin, begin, dof, dof).setZero();
    C_.segment(begin, dof).setZero();
  }

  for(int i = 0; i < 2; ++i)
  {
    int r = robotIndexes_[i];
    int begin = posInQ_[i];
    int dof = data.alphaD(r);

    const Eigen::MatrixXd activeJointsMat = activeJointsMatrix(unactiveJointsName_[i],mbs[r]);
    const Eigen::MatrixXd J = J_[i];
    const Eigen::Matrix6d W = dimWeight_.asDiagonal();
    const Eigen::MatrixXd M =  W * J * activeJointsMat;
    const Eigen::VectorXd c = activeJointsMat * J.transpose() * W  * CSum_;

    // since the two robot index could be the same
    // we had to increment the Q and C matrix
    Q_.block(begin, begin, dof, dof) += activeJointsMat * J.transpose()  * M;
    C_.segment(begin, dof) += c;

  }

}

Eigen::MatrixXd biRobotTeleopTask::activeJointsMatrix(const std::vector<std::string> & unactiveJointsNames, const rbd::MultiBody & mb)
{
  const int nDof = mb.nrDof();
  Eigen::MatrixXd dofMat = Eigen::MatrixXd::Identity(nDof,nDof);

  for (auto & j : unactiveJointsNames)
  {
    int indx = getMatrixIndx(mb,mb.jointIndexByName(j));
    const int dof = mb.joints()[mb.jointIndexByName(j)].dof();
    dofMat.block(indx,indx,dof,dof) = Eigen::MatrixXd::Zero(dof,dof);
  }

  return dofMat;

}

const Eigen::VectorXd & biRobotTeleopTask::eval() const
{
  return eval_;
}

const Eigen::VectorXd & biRobotTeleopTask::speed() const
{
  return speed_;
}

const Eigen::MatrixXd & biRobotTeleopTask::Q() const
{
  return Q_;
}

const Eigen::VectorXd & biRobotTeleopTask::C() const
{
  return C_;
}

} //namespace qp

} // namespace tasks
