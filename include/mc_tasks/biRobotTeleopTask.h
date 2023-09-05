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
#include "../../include/Tasks/biRobotTeleopTask.h"
#include "../../include/bilateralteleop/HumanRobotPose.h"
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/CD/CD_Pair.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <Tasks/Tasks.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI biRobotTeleopTask : public MetaTask
{

public:

    biRobotTeleopTask(const mc_solver::QPSolver & solver, unsigned int r1Index, unsigned int r2Index, bilateralTeleop::Limbs link1 = bilateralTeleop::Limbs::LeftHand, bilateralTeleop::Limbs link2 = bilateralTeleop::Limbs::RightHand ,double stiffness = 1., double weight = 10.);

    void reset() override
    {
        eval_ = Eigen::Vector6d::Zero();
        speed_ = Eigen::Vector6d::Zero();
    }

    /*! \brief Load parameters from a Configuration object */
    void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

    /*! \brief Set the task dimensional weight
    *
    */
    void dimWeight(const Eigen::VectorXd & dimW) override
    {
        task_.dimWeight(dimW);
    }

    Eigen::VectorXd dimWeight() const override
    {
        return task_.dimWeight();
    }

    /*! \brief Select active joints for this task
    *
    * Manipulate \ref dimWeight() to achieve its effect
    */
    void selectActiveJoints(mc_solver::QPSolver & solver,
                            const int rIndex,
                            const std::vector<std::string> & activeJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {});
    /**
     * @brief Dummy (To implement) use the other one
     * 
     * @param solver 
     * @param activeJointsName 
     * @param activeDofs 
     */
    void selectActiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & activeJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

    /*! \brief Select inactive joints for this task
    *
    * Manipulate \ref dimWeight() to achieve its effect
    */
    void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const int rIndex,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {});

    /**
     * @brief Dummy (To implement) use the other one
     * 
     * @param solver 
     * @param unactiveJointsName 
     * @param unactiveDofs 
     */
    void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

    /*! \brief Reset the joint selector effect
    *
    * Reset dimWeight to ones
    */
    void resetJointsSelector(mc_solver::QPSolver & solver,const int rIndex);
    
    void resetJointsSelector(mc_solver::QPSolver & solver) override;

    Eigen::VectorXd eval() const override
    {
        return task_.eval();
    }

    Eigen::VectorXd speed() const override
    {
        return task_.speed();
    }


    void stiffness(double s)
    {
        task_.stiffness(s);
    }

    /** Get the current task's stiffness */
    double stiffness() const
    {
        return task_.stiffness();
    }

    /** Set the task damping, leaving its stiffness unchanged
     *
     * \param damping Task stiffness
     *
     */
    void damping(double d);

    /** Get the task's current damping */
    double damping() const;

    /** Set both stiffness and damping
     *
     * \param stiffness Task stiffness
     *
     * \param damping Task damping
     *
     */
    void setGains(double stiffness, double damping);

    /** Set task's weight */
    void weight(double w)
    {
        weight_ = w;
        task_.dimWeight(Eigen::Vector6d::Ones() * weight_);
    }

    /** Get task's weight */
    double weight() const
    {
        return weight_;
    }

    /** True if the task is in the solver */
    bool inSolver() const;

    void updateHuman(const bilateralTeleop::HumanPose & human_1, const bilateralTeleop::HumanPose & human_2)
    {
        human_1_pose_ = human_1;
        human_2_pose_ = human_2;
 
    }

    std::vector<bilateralTeleop::HumanPose> getHumanPose()
    {
        return {human_1_pose_,human_2_pose_};
    }

    bilateralTeleop::Limbs getTargetLink(const int rIndex)
    {
        if(rIndex == r1Index_)
        {
            return link_1_;
        }
        else if(rIndex == r2Index_)
        {
            return link_2_;
        }
        mc_rtc::log::error("[{} , getTargetLink] Robot index {} not in the task, available indexes : {} and {}",name(),rIndex,r1Index_,r2Index_);
        return bilateralTeleop::Limbs::LeftHand;
    }

    void setTargetLink(const int rIndex,const bilateralTeleop::Limbs link)
    {
        if(rIndex == r1Index_)
        {
            link_1_ = link;
        }
        else if(rIndex == r2Index_)
        {
            link_2_ = link;
        }
        else
        {
            mc_rtc::log::error("[{} , setTargetLink] Robot index {} not in the task, available indexes : {} and {}",name(),rIndex,r1Index_,r2Index_);
        }
    }   

    std::string getLinkName(const int rIndex,bilateralTeleop::Limbs limb)
    {
        if(rIndex == r1Index_)
        {
            return robot_1_pose_links_.get(limb);
        }
        if(rIndex == r2Index_)
        {
            return robot_2_pose_links_.get(limb);
        }
        mc_rtc::log::error("[{} , getLinkName] Robot index {} not in the task, available indexes : {} and {}",name(),rIndex,r1Index_,r2Index_);
        return "";

    }

    /**
     * @brief Get the Relative Transfo from human 1 to robot 2 and from robot 1 to human 2 in that order
     * 
     * @return std::vector<sva::PTransformd> 
     */
    std::vector<sva::PTransformd> getRelativeTransfo()
    {
        return {X_h1_r2_,X_r1_h2_};
    }

protected:
    void addToSolver(mc_solver::QPSolver & solver) override;

    void removeFromSolver(mc_solver::QPSolver & solver) override;

    void update(mc_solver::QPSolver &) override;

    void addToGUI(mc_rtc::gui::StateBuilder &) override;

    void addToLogger(mc_rtc::Logger & logger) override;

    void loadRobotConf(mc_solver::QPSolver & solver, const int rIndex, const mc_rtc::Configuration & conf);

private:

    sch::Matrix4x4 convertTransfo(const sva::PTransformd & X_0_cvx)
    {
    sch::Matrix4x4 m;
    const Eigen::Matrix3d & rot = X_0_cvx.rotation();
    const Eigen::Vector3d & tran = X_0_cvx.translation();

    for(unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j) { m(i, j) = rot(j, i); }
    }

    m(0, 3) = tran(0);
    m(1, 3) = tran(1);
    m(2, 3) = tran(2);

    return m;
    }
    void get_q_dq(Eigen::VectorXd & q, Eigen::VectorXd & dq ,const mc_rbdyn::Robot & robot ,const rbd::Jacobian & jac)
    {
        std::vector<double> dot_q_vec = {};
        std::vector<double> q_vec = {};
        for (auto & indx : jac.jointsPath())
        {
        for (auto & qd : robot.mbc().alpha[indx])
        {
            dot_q_vec.push_back(qd);
        }  
        for (auto & q : robot.mbc().q[indx])
        {
            q_vec.push_back(q);
        }  
        }
        dq = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(dot_q_vec.data(), dot_q_vec.size());
        q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_vec.data(), q_vec.size());
    }

    /**
     * @brief Compute the offset corresponding to the closest point on the robot to the current human convex sphere
     * 
     */
    void getOffset(sva::PTransformd & X_r_rOff,sva::PTransformd & X_h_hOff, 
                                sch::CD_Pair & cvx_pair_h_r,
                                const sva::PTransformd & X_0_robotLink, const sva::PTransformd & X_0_humanLink )
    {
        sch::Point3 p1, p2;
        cvx_pair_h_r.getClosestPoints(p1,p2);
        Eigen::Vector3d robot_point_link;
        robot_point_link << p2.m_x,p2.m_y,p2.m_z;
        Eigen::Vector3d human_point_link;
        human_point_link << p1.m_x,p1.m_y,p1.m_z;
        X_r_rOff = sva::PTransformd(X_0_robotLink.rotation(),robot_point_link) * X_0_robotLink.inv();
        X_h_hOff = sva::PTransformd(X_0_humanLink.rotation(),human_point_link) * X_0_humanLink.inv();

    }
    void removeRootDofAndRotComp(Eigen::MatrixXd & J)
    {
        J.block(0,0,6,6).setZero();
        J.block(0,0,3,J.cols()).setZero();
    }
    sva::MotionVecd getTargetVel(const mc_rbdyn::Robot & robot, const std::string & link ,const Eigen::Vector3d & off)
    {
    const sva::MotionVecd v_l_l = robot.bodyVelB(link);
    const sva::PTransformd X_0_l = robot.bodyPosW(link);
    const sva::PTransformd X_l_t = sva::PTransformd(Eigen::Matrix3d::Identity(),off);
    return (sva::PTransformd(X_0_l.rotation().transpose(),Eigen::Vector3d::Zero()) * X_l_t) * v_l_l;
    }

    tasks::qp::biRobotTeleopTask task_; 

    /** True if added to solver */
    bool inSolver_ = false;
    /** Robot handled by the task */
    const mc_rbdyn::Robots & robots_;
    unsigned int r2Index_ = 1;
    unsigned int r1Index_ = 0;

    /** Solver timestep */
    double dt_ = 0.005;

    double weight_ = 0;

    bilateralTeleop::Limbs link_1_;
    bilateralTeleop::Limbs link_2_;


    bilateralTeleop::RobotPose robot_1_pose_links_;
    bilateralTeleop::RobotPose robot_2_pose_links_;
    bilateralTeleop::RobotPose human_pose_links_;
    bilateralTeleop::HumanPose human_1_pose_;
    bilateralTeleop::HumanPose human_2_pose_;
    Eigen::Vector3d robot_2_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d human_1_point_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d robot_1_point_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d human_2_point_ = Eigen::Vector3d::Zero();

    //transfo from human_1 to robot_2
    sva::PTransformd X_h1_r2_ = sva::PTransformd::Identity();

    //transfo from robot to human_2
    sva::PTransformd X_r1_h2_ = sva::PTransformd::Identity();

    /** Store the previous eval vector */
    Eigen::VectorXd eval_;
    /** Store the task speed */
    Eigen::VectorXd speed_;

    mc_rtc::gui::ArrowConfig arrowConfig_;
};

using biRobotTelopTaskPtr = std::shared_ptr<biRobotTeleopTask>;


} //namespace tasks