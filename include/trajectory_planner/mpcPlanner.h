/*
	FILE: mpcPlanner.h
	----------------------------
	mpc trajectory solver header based on occupancy grid map
*/

#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H
#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <trajectory_planner/clustering/obstacleClustering.h>
#include <trajectory_planner/utils.h>
#include <map_manager/occupancyMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_planner/third_party/OsqpEigen/OsqpEigen.h>

using std::cout; using std::endl;
namespace trajPlanner{
	class mpcPlanner{
	private:
		std::string ns_;
		std::string hint_;
		ros::NodeHandle nh_;
		ros::Publisher mpcTrajVisPub_;
		ros::Publisher mpcTrajHistVisPub_;
		ros::Publisher localCloudPub_;
		ros::Publisher staticObstacleVisPub_;
		ros::Publisher dynamicObstacleVisPub_;
		ros::Publisher facingPub_;

		ros::Timer visTimer_;
		ros::Timer clusteringTimer_;

		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<obstacleClustering> obclustering_;
		double ts_; // timestep
		Eigen::Vector3d currPos_;
		Eigen::Vector3d currVel_;
		std::vector<Eigen::Vector3d> inputTraj_;
		int lastRefStartIdx_ = 0;
		bool firstTime_ = true;
		bool stateReceived_ = false;
		std::vector<Eigen::VectorXd> currentStatesSol_;
		std::vector<Eigen::VectorXd> currentControlsSol_;
		std::vector<Eigen::Vector3d> currentTraj_;
		std::vector<Eigen::Vector3d> trajHist_;
		std::vector<Eigen::Vector3d> currCloud_;
		std::vector<bboxVertex> refinedBBoxVertices_;
		std::vector<Eigen::Vector3d> dynamicObstaclesPos_;
		std::vector<Eigen::Vector3d> dynamicObstaclesVel_;
		std::vector<Eigen::Vector3d> dynamicObstaclesSize_;

		// parameters
		static const int numStates = 8;
		static const int numControls = 5;
		int horizon_;
		double maxVel_ = 1.0;
		double maxAcc_ = 1.0;
		double zRangeMin_;
		double zRangeMax_;
		double safetyDist_;
		double staticSlack_;
		double dynamicSlack_;

		// clustering params
		double cloudRes_;
		double regionSizeX_;
		double regionSizeY_;
		double groundHeight_;
		double ceilingHeight_;
		double angle_;

	public:
		mpcPlanner(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void setMap(const std::shared_ptr<mapManager::occMap>& map);

		void staticObstacleClusteringCB(const ros::TimerEvent&);

		void updateMaxVel(double maxVel);
		void updateMaxAcc(double maxAcc);
		void updateCurrStates(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel);
		void updatePath(const nav_msgs::Path& path, double ts);
		void updatePath(const std::vector<Eigen::Vector3d>& path, double ts);
		void updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos, const std::vector<Eigen::Vector3d>& obstaclesVel, const std::vector<Eigen::Vector3d>& obstaclesSize); // position, velocity, size
		bool makePlan();
		

		// OSQP Solver Setup
		void setDynamicsMatrices(Eigen::Matrix<double, numStates, numStates> &A, Eigen::Matrix<double, numStates, numControls> &B); //TODO
		void setInequalityConstraints(Eigen::Matrix<double, numStates, 1> &xMax, Eigen::Matrix<double, numStates, 1> &xMin, Eigen::Matrix<double, numControls, 1> &uMax, Eigen::Matrix<double, numControls, 1> &uMin); //TODO
		void getXRef(std::vector<Eigen::Matrix<double, numStates, 1>>& xRef);
		void setWeightMatrices(Eigen::DiagonalMatrix<double,numStates> &Q, Eigen::DiagonalMatrix<double, numControls> &R);
		void castMPCToQPHessian(const Eigen::DiagonalMatrix<double,numStates> &Q, const Eigen::DiagonalMatrix<double,numControls> &R, int mpcWindow, Eigen::SparseMatrix<double>& hessianMatrix);
		void castMPCToQPGradient(const Eigen::DiagonalMatrix<double,numStates> &Q, const std::vector<Eigen::Matrix<double, numStates, 1>>& xRef, int mpcWindow, Eigen::VectorXd& gradient);
		void castMPCToQPConstraintMatrix(Eigen::Matrix<double, numStates, numStates> &A, Eigen::Matrix<double, numStates, numControls> &B, 
			Eigen::SparseMatrix<double> &constraintMatrix, int numObs, int mpcWindow, 
			std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw,
			std::vector<std::vector<int>> &isDynamic);
		void castMPCToQPConstraintVectors(Eigen::Matrix<double,numStates,1> &xMax,
			Eigen::Matrix<double,numStates,1> &xMin,
			Eigen::Matrix<double,numControls,1> &uMax,
			Eigen::Matrix<double,numControls,1> &uMin,
			const Eigen::Matrix<double, numStates, 1>& x0,
			Eigen::Matrix<double, Eigen::Dynamic, 1> &lowerBound, Eigen::Matrix<double, Eigen::Dynamic, 1> &upperBound, int numObs, int mpcWindow, 
			std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw);

		void updateObstacleParam(const std::vector<staticObstacle> &staticObstacles, 
			                     int &numObs, 
			                     int mpcWindow, 
								 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, 
								 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, 
								 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw, 
								 std::vector<std::vector<int>> &isDynamic);
	
		void getReferenceTraj(std::vector<Eigen::Vector3d>& referenceTraj);

		void getTrajectory(std::vector<Eigen::Vector3d>& traj);
		void getTrajectory(nav_msgs::Path& traj);

		Eigen::Vector3d getPos(double t);
		Eigen::Vector3d getVel(double t);
		Eigen::Vector3d getAcc(double t);
		double getTs();
		double getHorizon();

		void visCB(const ros::TimerEvent&);
		void publishMPCTrajectory();
		void publishHistoricTrajectory();
		void publishLocalCloud();
		void publishStaticObstacles();
		void publishDynamicObstacles();
	};
}
#endif