// associated header
#include "../../include/mc_tasks/biRobotTeleopTask.h"
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

#include <mc_solver/TasksQPSolver.h>
#include <mc_tasks/MetaTaskLoader.h>  
#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Arrow.h>



namespace mc_tasks
{

biRobotTeleopTask::biRobotTeleopTask(const mc_solver::QPSolver & solver, unsigned int r1Index, unsigned int r2Index, bilateralTeleop::Limbs link1, bilateralTeleop::Limbs link2 ,double stiffness, double weight)
: robots_(solver.robots()), r1Index_(r1Index),r2Index_(r2Index), link_1(link1), link_2(link2),
  dt_(solver.dt()),task(solver.robots().mbs(),r1Index,r2Index,stiffness,weight)
{   
    
    eval_ = this->eval();
    speed_ = Eigen::VectorXd::Zero(eval_.size());
    type_ = "biRobotTeleopTask";
    name_ = std::string("biRobotTeleopTask") + robots_.robot(r1Index_).name() + "_" + robots_.robot(r2Index_).name();

    reset();
}

void biRobotTeleopTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  const auto & conf = config("biRobotTeleop");

  if(conf.has("stiffness")) { this->stiffness(conf("stiffness")); }
  if(conf.has("weight")) { this->weight(conf("weight")); }
  robot_1_pose_links_.load(conf("robot_1"));
  robot_2_pose_links_.load(conf("robot_2"));
  human_1_pose_.setCvx(conf("human"));
  human_2_pose_.setCvx(conf("human"));
}


void biRobotTeleopTask::removeFromSolver(mc_solver::QPSolver & solver)
{
    if(!inSolver_) { return; }
    inSolver_ = false;
    tasks_solver(solver).removeTask(&task);
}

void biRobotTeleopTask::addToSolver(mc_solver::QPSolver & solver)
{
    if(inSolver_) { return; }
    inSolver_ = true;
    tasks_solver(solver).addTask(&task);
}

void biRobotTeleopTask::update(mc_solver::QPSolver &)
{

    const mc_rbdyn::Robot & robot_2 = robots_.robot(r2Index_);
    const mc_rbdyn::Robot & robot_1 = robots_.robot(r1Index_);

    const std::string robot_2_link_name = robot_2_pose_links_.get(link_2);
    const std::string robot_1_link_name = robot_1_pose_links_.get(link_1);

    auto robot_1_cvx = robot_1.convex(robot_1_link_name);
    robot_1_cvx.second.get()->setTransformation(convertTransfo(robot_1.bodyPosW(robot_1_link_name)));
    auto robot_2_cvx = robot_2.convex(robot_2_link_name);
    robot_2_cvx.second.get()->setTransformation(convertTransfo(robot_2.bodyPosW(robot_2_link_name)));
    auto human_1_cvx = human_1_pose_.getConvex(link_1);
    auto human_2_cvx = human_2_pose_.getConvex(link_2);

    sch::CD_Pair pair_h1_r2(&human_1_cvx,robot_2_cvx.second.get());
    sch::CD_Pair pair_h2_r1(&human_2_cvx,robot_1_cvx.second.get());

    sch::Point3 p1, p2;
    pair_h1_r2.getClosestPoints(p1,p2);

    human_1_point << p1[0],p1[1],p1[2];
    robot_2_point << p2[0],p2[1],p2[2];

    pair_h2_r1.getClosestPoints(p1,p2);

    human_2_point << p1[0],p1[1],p1[2];
    robot_1_point << p2[0],p2[1],p2[2];


    sva::PTransformd X_h1_h1p = sva::PTransformd::Identity();
    sva::PTransformd X_h2_h2p = sva::PTransformd::Identity();
    sva::PTransformd X_r2_r2p = sva::PTransformd::Identity();
    sva::PTransformd X_r1_r1p = sva::PTransformd::Identity();

    getOffset(X_r2_r2p,X_h1_h1p,pair_h1_r2,robot_2.bodyPosW(robot_2_link_name),human_1_pose_.getPose(link_1));
    getOffset(X_r1_r1p,X_h2_h2p,pair_h2_r1,robot_1.bodyPosW(robot_1_link_name),human_2_pose_.getPose(link_2));

    eval_.segment(3,3) =  (X_r2_r2p * robot_2.bodyPosW(robot_2_link_name)).translation() 
                                - (X_h1_h1p * human_1_pose_.getPose(link_1)).translation()  
                                - ( (X_h2_h2p * human_2_pose_.getPose(link_2)).translation() 
                                - (X_r1_r1p * robot_1.bodyPosW(robot_1_link_name)).translation());

    speed_.segment(3,3) =  getTargetVel(robot_2,robot_2_link_name,X_r2_r2p.translation()).linear()
                        - human_1_pose_.getVel(link_1,X_h1_h1p).linear()  
                        - (human_2_pose_.getVel(link_2,X_h2_h2p).linear() 
                            - getTargetVel(robot_1,robot_1_link_name,X_r1_r1p.translation()).linear());


    rbd::Jacobian robot_1_jac = rbd::Jacobian(robot_1.mb(),robot_1_link_name);
    const auto & shortJ1 = robot_1_jac.jacobian(robot_1.mb(),robot_1.mbc());
    Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(6,robot_1.mb().nrDof()); 
    robot_1_jac.fullJacobian(robot_1.mb(),shortJ1,J1); 
    auto dot_J1 = robot_1_jac.jacobianDot(robot_1.mb(),robot_1.mbc());


    rbd::Jacobian robot_2_jac = rbd::Jacobian(robot_2.mb(),robot_2_link_name);
    const auto & shortJ2 = robot_2_jac.jacobian(robot_2.mb(),robot_2.mbc());
    Eigen::MatrixXd J2 = Eigen::MatrixXd::Zero(6,robot_2.mb().nrDof()); 
    robot_2_jac.fullJacobian(robot_2.mb(),shortJ2,J2); 
    auto dotJ2 = robot_2_jac.jacobianDot(robot_2.mb(),robot_2.mbc());


    Eigen::VectorXd dot_q2;
    Eigen::VectorXd q2;
    Eigen::VectorXd dot_q1;
    Eigen::VectorXd q1;
    get_q_dq(q2,dot_q2,robot_2,robot_2_jac);
    get_q_dq(q1,dot_q1,robot_1,robot_1_jac);


    task.setJacobians(J1,J2);

    task.normalAcc(dotJ2 * dot_q2
            - human_1_pose_.getAcc(link_1,X_h1_h1p).vector()
            - (human_2_pose_.getAcc(link_2,X_h2_h2p).vector() - dot_J1 * dot_q1)
            );
    
    task.eval(eval_);
    task.speed(speed_);



}

void biRobotTeleopTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_eval", this, [this]() { return eval(); });
  logger.addLogEntry(name_ + "_speed", this, [this]() -> const Eigen::VectorXd & { return speed_; });
}

void biRobotTeleopTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput(
                     "stiffness & damping", [this]() { return this->stiffness(); },
                     [this](const double & g) { this->stiffness(g); }),
                 mc_rtc::gui::NumberInput(
                     "weight", [this]() { return this->weight(); }, [this](const double & w) { this->weight(w); }),
                 mc_rtc::gui::Arrow("h1 -> r2 convex distance",[this](){return  human_1_point;},
                                                                    [this] (){return  robot_2_point; }),
                  mc_rtc::gui::Arrow("h2 -> r1 convex distance",[this](){return  human_2_point;},
                                                        [this] (){return  robot_1_point; })
                                                                    );


}

void biRobotTeleopTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                     const int rIndex,
                                     const std::vector<std::string> & activeJointsName,
                                     const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex), activeJointsName, "[" + name() + "::selectActiveJoints]");
  std::vector<std::string> unactiveJoints = {};
  for(const auto & j : robots_.robot(rIndex).mb().joints())
  {
    if(j.dof() && std::find(activeJointsName.begin(), activeJointsName.end(), j.name()) == activeJointsName.end())
    {
      unactiveJoints.push_back(j.name());
    }
  }
  selectUnactiveJoints(solver, rIndex ,unactiveJoints);
}
void biRobotTeleopTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                     const std::vector<std::string> & activeJointsName,
                                     const std::map<std::string, std::vector<std::array<int, 2>>> &)
{

}

void biRobotTeleopTask::selectUnactiveJoints(mc_solver::QPSolver &,
                                       const int rIndex,
                                       const std::vector<std::string> & unactiveJointsName,
                                       const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex), unactiveJointsName, "[" + name() + "::selectUnActiveJoints]");

  task.unactiveJoints(unactiveJointsName,rIndex);

}
void biRobotTeleopTask::selectUnactiveJoints(mc_solver::QPSolver &,
                                       const std::vector<std::string> & unactiveJointsName,
                                       const std::map<std::string, std::vector<std::array<int, 2>>> &)
{


}


void biRobotTeleopTask::resetJointsSelector(mc_solver::QPSolver & solver,const int rIndex)
{
  selectUnactiveJoints(solver, rIndex ,{});
}

void biRobotTeleopTask::resetJointsSelector(mc_solver::QPSolver & solver)
{


}

} // namespace mc_tasks