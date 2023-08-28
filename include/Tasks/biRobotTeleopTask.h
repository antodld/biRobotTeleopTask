#pragma once

// includes
//
#include <array>

#include <tasks/config.hh>

// Eigen
#include <eigen3/Eigen/Core>

#include <mc_tasks/MetaTask.h>
#include <Tasks/QPSolver.h>
#include <Tasks/QPSolverData.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/CD/CD_Pair.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <Tasks/Tasks.h>


namespace tasks
{

namespace qp
{

class TASKS_DLLAPI biRobotTeleopTask : public Task
{
public:

    biRobotTeleopTask(const std::vector<rbd::MultiBody> & mbs,
                                                    int r1Index,
                                                    int r2Index,
                                                    double stiffness,
                                                    double weight);

    void setJacobians(const Eigen::MatrixXd & J1 , const Eigen::MatrixXd & J2)
    {
        J_.clear();
        J_.push_back(J1);
        J_.push_back(J2);
  
    }

    void unactiveJoints(const std::vector<std::string> & robot_joints,const int rIndex)
    {

        if(robotIndexes_[0] == rIndex)
        {
            std::cout << "here" << std::endl;
            unactiveJointsName_[0] = robot_joints;
        }
        if(robotIndexes_[1] == rIndex)
        {
            unactiveJointsName_[1] = robot_joints;
        }
    }

    std::vector<std::string> unactiveJoints(const int rIndex)
    {
        if(robotIndexes_[0] == rIndex)
        {
            return unactiveJointsName_[0];
        }
        if(robotIndexes_[1] == rIndex)
        {
            return unactiveJointsName_[1];
        }
        return {};
    }


    double stiffness() const { return stiffness_; }

    void stiffness(double stiffness);
    
    void dimWeight(const Eigen::Vector6d & dim);

    const Eigen::Vector6d & dimWeight() const { return dimWeight_; }

    virtual std::pair<int, int> begin() const override { return {alphaDBegin_, alphaDBegin_}; }

    virtual void updateNrVars(const std::vector<rbd::MultiBody> & mbs, const SolverData & data) override;
    virtual void update(const std::vector<rbd::MultiBody> & mbs,
                        const std::vector<rbd::MultiBodyConfig> & mbcs,
                        const SolverData & data) override;
    
    Eigen::MatrixXd activeJointsMatrix(const std::vector<std::string> & unactiveJointsNames, const rbd::MultiBody & mb);

    int getMatrixIndx(const rbd::MultiBody & mb,const int targetIndx)
    {
        int indx = 0;
        int count = 0;
        while(count != targetIndx)
        {
            indx += mb.joints()[count].dof();
            count++;
        }
        return indx;
    }

    virtual const Eigen::MatrixXd & Q() const override;
    virtual const Eigen::VectorXd & C() const override;

    const Eigen::VectorXd & eval() const;
    const Eigen::VectorXd & speed() const;

    void eval(const Eigen::Vector6d & eval)
    {
        eval_ = eval;
    }
    void speed(const Eigen::Vector6d & s)
    {
        speed_ = s;
    }
    void normalAcc(const Eigen::Vector6d & a)
    {
        normalAcc_ = a;
    }

private:
    int alphaDBegin_;
    double stiffness_, stiffnessSqrt_;
    Eigen::Vector6d dimWeight_;
    std::vector<int> posInQ_, robotIndexes_;
    Eigen::MatrixXd Q_;
    Eigen::VectorXd C_;
    Eigen::VectorXd CSum_;
    // cache
    Eigen::MatrixXd preQ_;

    std::vector<Eigen::MatrixXd> J_;
    Eigen::VectorXd eval_;
    Eigen::VectorXd speed_;
    Eigen::VectorXd normalAcc_;

    std::vector<std::vector<std::string>> unactiveJointsName_ = {{},{}};
};

} //namespace qp

} // namespace tasks
