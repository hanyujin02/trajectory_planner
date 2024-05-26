/*
	FILE: mpcPlanner.cpp
	-----------------------------
	mpc trajectory solver implementation based on occupancy grid map 
*/

#include <trajectory_planner/mpcPlanner.h>

namespace trajPlanner{
	mpcPlanner::mpcPlanner(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "mpc_planner";
		this->hint_ = "[MPCPlanner]";
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerCallback();
	}

	void mpcPlanner::initParam(){ 
		// planning horizon
		if (not this->nh_.getParam(this->ns_ + "/horizon", this->horizon_)){
			this->horizon_ = 20;
			cout << this->hint_ << ": No planning horizon param. Use default: 20" << endl;
		}
		else{
			cout << this->hint_ << ": Planning horizon is set to: " << this->horizon_ << endl;
		}	

		// mininimum height
		if (not this->nh_.getParam(this->ns_ + "/z_range_min", this->zRangeMin_)){
			this->zRangeMin_ = 0.7;
			cout << this->hint_ << ": No z range min param. Use default: 0.7m" << endl;
		}
		else{
			cout << this->hint_ << ": Z range min is set to: " << this->zRangeMin_ << "m" << endl;
		}

		// maximum height
		if (not this->nh_.getParam(this->ns_ + "/z_range_max", this->zRangeMax_)){
			this->zRangeMax_ = 1.2;
			cout << this->hint_ << ": No z range max param. Use default: 1.2m" << endl;
		}
		else{
			cout << this->hint_ << ": Z range max is set to: " << this->zRangeMax_ << "m" << endl;
		}	

		// pointcloud resolution for clustering
		if (not this->nh_.getParam(this->ns_ + "/cloud_res", this->cloudRes_)){
			this->cloudRes_ = 0.2;
			cout << this->hint_ << ": No cloud res param. Use default: 0.2" << endl;
		}
		else{
			cout << this->hint_ << ": Cloud res is set to: " << this->cloudRes_ << endl;
		}	

		// local cloud region size x 
		if (not this->nh_.getParam(this->ns_ + "/local_cloud_region_x", this->regionSizeX_)){
			this->regionSizeX_ = 5.0;
			cout << this->hint_ << ": No local cloud region size x param. Use default: 5.0m" << endl;
		}
		else{
			cout << this->hint_ << ": Local cloud region size x is set to: " << this->regionSizeX_ << "m" << endl;
		}	

		// local cloud region size y 
		if (not this->nh_.getParam(this->ns_ + "/local_cloud_region_y", this->regionSizeY_)){
			this->regionSizeY_ = 2.0;
			cout << this->hint_ << ": No local cloud region size y param. Use default: 2.0m" << endl;
		}
		else{
			cout << this->hint_ << ": Local cloud region size y is set to: " << this->regionSizeY_ << "m" << endl;
		}	

		// ground height
		if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.3;
			cout << this->hint_ << ": No ground height param. Use default: 0.3m" << endl;
		}
		else{
			cout << this->hint_ << ": Ground height is set to: " << this->groundHeight_ << "m" << endl;
		}		

		// ceiling height
		if (not this->nh_.getParam(this->ns_ + "/ceiling_height", this->ceilingHeight_)){
			this->ceilingHeight_ = 2.0;
			cout << this->hint_ << ": No ceiling height param. Use default: 2.0m" << endl;
		}
		else{
			cout << this->hint_ << ": Ceiling height is set to: " << this->ceilingHeight_ << "m" << endl;
		}		

		// safety distance
		if (not this->nh_.getParam(this->ns_ + "/safety_dist", this->safetyDist_)){
			this->safetyDist_ = 0.5;
			cout << this->hint_ << ": No safety distance param. Use default: 0.5m" << endl;
		}
		else{
			cout << this->hint_ << ": Safety distance is set to: " << this->safetyDist_ << "m" << endl;
		}	

		// static slack variable
		if (not this->nh_.getParam(this->ns_ + "/static_constraint_slack_ratio", this->staticSlack_)){
			this->staticSlack_ = 0.5;
			cout << this->hint_ << ": No static slack variable param. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": Static slack variable is set to: " << this->staticSlack_ << endl;
		}				

		// dynamic slack variable
		if (not this->nh_.getParam(this->ns_ + "/dynamic_constraint_slack_ratio", this->dynamicSlack_)){
			this->dynamicSlack_ = 0.5;
			cout << this->hint_ << ": No dynamic slack variable param. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": Dynamic slack variable is set to: " << this->dynamicSlack_ << endl;
		}

	}

	void mpcPlanner::initModules(){
		this->obclustering_.reset(new obstacleClustering (this->cloudRes_));
	}

	void mpcPlanner::registerPub(){
		this->mpcTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>(this->ns_ + "/mpc_trajectory", 10);
		this->mpcTrajHistVisPub_ = this->nh_.advertise<nav_msgs::Path>(this->ns_ + "/traj_history", 10);
		this->localCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/local_cloud", 10);
		this->staticObstacleVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/static_obstacles", 10);
		this->dynamicObstacleVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/dynamic_obstacles", 10);
		this->facingPub_ = this->nh_.advertise<visualization_msgs::Marker>(this->ns_ + "/clustering_facing", 10);
	}

	void mpcPlanner::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &mpcPlanner::visCB, this);
		this->clusteringTimer_ = this->nh_.createTimer(ros::Duration(0.05), &mpcPlanner::staticObstacleClusteringCB, this);
	}

	void mpcPlanner::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void mpcPlanner::staticObstacleClusteringCB(const ros::TimerEvent&){
		// ros::Time clusteringStartTime = ros::Time::now();
		// cout<<"[MPC Planner]: clustering CB start time "<<clusteringStartTime<<endl;
		if (this->inputTraj_.size() == 0 or not this->stateReceived_) return;
		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		std::vector<Eigen::Vector3d> currCloud;
		double offset = 2.0;
		
		double angle = atan2(this->currVel_(1), this->currVel_(0));		
		if (this->currVel_.norm()>=0.3 or this->firstTime_){
			this->angle_ = angle;
		}
		Eigen::Vector3d faceDirection (cos(this->angle_), sin(this->angle_), 0);

		Eigen::Vector3d sideDirection (-sin(this->angle_), cos(this->angle_), 0); // positive (left side)
		Eigen::Vector3d pOrigin = this->currPos_ - offset * faceDirection;

		// find four vextex of the bounding boxes
		Eigen::Vector3d p1, p2, p3, p4;
		p1 = pOrigin + this->regionSizeY_ * sideDirection;
		p2 = pOrigin - this->regionSizeY_ * sideDirection;
		p3 = p1 + (this->regionSizeX_ + offset) * faceDirection;
		p4 = p2 + (this->regionSizeX_ + offset) * faceDirection;

		double xStart = floor(std::min({p1(0), p2(0), p3(0), p4(0)})/this->cloudRes_)*this->cloudRes_;
		double xEnd = ceil(std::max({p1(0), p2(0), p3(0), p4(0)})/this->cloudRes_)*this->cloudRes_;
		double yStart = floor(std::min({p1(1), p2(1), p3(1), p4(1)})/this->cloudRes_)*this->cloudRes_;
		double yEnd = ceil(std::max({p1(1), p2(1), p3(1), p4(1)})/this->cloudRes_)*this->cloudRes_;

		for (double ix=xStart; ix<=xEnd; ix+=this->cloudRes_){
			for (double iy=yStart; iy<=yEnd; iy+=this->cloudRes_){
				for (double iz=this->groundHeight_; iz<=this->ceilingHeight_; iz+=this->cloudRes_){
					Eigen::Vector3d p (ix, iy, iz);
					if ((p - pOrigin).dot(faceDirection) >= 0){
						if (this->map_->isInMap(p) and this->map_->isInflatedOccupied(p)){
							currCloud.push_back(p);
						}
					}
				}
			}
		}	
		this->currCloud_ = currCloud;
		this->obclustering_->run(currCloud);
		this->refinedBBoxVertices_ = this->obclustering_->getRefinedBBoxes();
		// ros::Time clusteringEndTime = ros::Time::now();
		// cout<<"[MPC Planner]: clustering time: "<<(clusteringEndTime-clusteringStartTime).toSec()<<endl;
	}

	void mpcPlanner::updateMaxVel(double maxVel){
		this->maxVel_ = maxVel;
	}

	void mpcPlanner::updateMaxAcc(double maxAcc){
		this->maxAcc_ = maxAcc;
	}

	void mpcPlanner::updateCurrStates(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel){
		this->currPos_ = pos;
		this->currVel_ = vel;
		this->trajHist_.push_back(pos);
		this->stateReceived_ = true;
	}

	void mpcPlanner::updatePath(const nav_msgs::Path& path, double ts){
		std::vector<Eigen::Vector3d> pathTemp;
		for (int i=0; i<(path.poses.size()); ++i){
			Eigen::Vector3d p (path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
			pathTemp.push_back(p); 
		}
		this->updatePath(pathTemp, ts);

	}

	void mpcPlanner::updatePath(const std::vector<Eigen::Vector3d>& path, double ts){
		this->ts_ = ts;
		this->inputTraj_ = path;
		this->firstTime_ = true;
		this->stateReceived_ = false;
		this->trajHist_.clear();
		this->lastRefStartIdx_ = 0;
	}

	void mpcPlanner::updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos, const std::vector<Eigen::Vector3d>& obstaclesVel, const std::vector<Eigen::Vector3d>& obstaclesSize){
		this->dynamicObstaclesPos_.clear();
		this->dynamicObstaclesSize_.clear();
		this->dynamicObstaclesVel_.clear();
		for (int i=0; i<int(obstaclesPos.size()); ++i){
            Eigen::Vector3d pos = obstaclesPos[i];
            Eigen::Vector3d size = obstaclesSize[i];
            pos(2) = (pos(2) + size(2)/2)/2;
			size(2) = 2*pos(2);
			// pos(2) = pos(2)+size(2)/2;
            this->dynamicObstaclesPos_.push_back(pos);
			this->dynamicObstaclesSize_.push_back(size);
        }
		this->dynamicObstaclesVel_ = obstaclesVel;
	}

	bool mpcPlanner::makePlan(){
		// set the preview window
		if (this->firstTime_){
			this->currentStatesSol_.clear();
			this->currentControlsSol_.clear();
		}
	    int mpcWindow = this->horizon_-1;
		int numObs;

	    // // allocate the dynamics matrices
	    Eigen::Matrix<double, numStates, numStates> a;
	    Eigen::Matrix<double, numStates, numControls> b;

	    // // allocate the constraints vector
	    Eigen::Matrix<double, numStates, 1> xMax;
	    Eigen::Matrix<double, numStates, 1> xMin;
	    Eigen::Matrix<double, numControls, 1> uMax;
	    Eigen::Matrix<double, numControls, 1> uMin;

	    // allocate the weight matrices
	    Eigen::DiagonalMatrix<double, numStates> Q;
	    Eigen::DiagonalMatrix<double, numControls> R;

	    // allocate the initial and the reference state space
	    Eigen::Matrix<double, numStates, 1> x0;
		x0.setZero();
		x0(0,0) = this->currPos_(0);
		x0(1,0) = this->currPos_(1);
		x0(2,0) = this->currPos_(2);
		x0(3,0) = this->currVel_(0);
		x0(4,0) = this->currVel_(1);
		x0(5,0) = this->currVel_(2);
	    std::vector<Eigen::Matrix<double, numStates, 1>> xRef;

		this->getXRef(xRef);
	    // allocate QP problem matrices and vectores
	    Eigen::SparseMatrix<double> hessian;
	    Eigen::VectorXd gradient;
	    Eigen::SparseMatrix<double> constraintMatrix;
	    Eigen::Matrix<double, Eigen::Dynamic, 1> lowerBound;
	    Eigen::Matrix<double, Eigen::Dynamic, 1> upperBound;

		std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> oxyz;
		std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> osize;
		std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> yaw;
		std::vector<std::vector<int>> isDynamic;
		std::vector<staticObstacle> staticObstacles = this->obclustering_->getStaticObstacles();
		this->updateObstacleParam(staticObstacles, numObs, mpcWindow, oxyz, osize, yaw, isDynamic);

	    // set MPC problem quantities
	    this->setDynamicsMatrices(a, b);
	    this->setInequalityConstraints(xMax, xMin, uMax, uMin);
	    this->setWeightMatrices(Q, R);

	    // cast the MPC problem as QP problem
	    this->castMPCToQPHessian(Q, R, mpcWindow, hessian);
	    this->castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
	    // castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
		this->castMPCToQPConstraintMatrix(a, b,constraintMatrix, numObs, mpcWindow, oxyz, osize, yaw, isDynamic);
	    this->castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, lowerBound, upperBound, numObs,mpcWindow,oxyz,osize,yaw);
	    // // instantiate the solver
	    OsqpEigen::Solver solver;
		// OSQPWrapper::OptimizatorSolver solver;

	    // // settings
	    solver.settings()->setVerbosity(false);
	    solver.settings()->setWarmStart(true);
		// solver.settings()->setTimeLimit(0.01);
		// solver.settings()->setAlpha(1.8);
		// solver.settings()->setDualInfeasibilityTolerance(1e-5);
		// solver.settings()->setDualInfeasibilityTollerance();
		// solver.settings()->setAdaptiveRho()

	    // set the initial data of the QP solver
	    solver.data()->setNumberOfVariables(numStates * (mpcWindow + 1) + numControls * mpcWindow);
	    // solver.data()->setNumberOfConstraints(numStates * (mpcWindow + 1) + 3*(mpcWindow+1)+ numControls * mpcWindow + numOb*mpcWindow);
	    solver.data()->setNumberOfConstraints(numStates * (mpcWindow + 1)+numStates * (mpcWindow + 1)+numControls*mpcWindow+numObs*mpcWindow);
	    
		if (!solver.data()->setHessianMatrix(hessian))
	        return false;
	    if (!solver.data()->setGradient(gradient))
	        return false;
	    if (!solver.data()->setLinearConstraintsMatrix(constraintMatrix))
	        return false;
	    if (!solver.data()->setLowerBound(lowerBound))
	        return false;
	    if (!solver.data()->setUpperBound(upperBound))
	        return false;

	    // instantiate the solver
	    if (!solver.initSolver())
	        return false;

	    // controller input and QPSolution vector
	    // Eigen::Vector4d ctr;
	    Eigen::VectorXd QPSolution;
		Eigen::VectorXd control;
		Eigen::VectorXd state;


		// solve the QP problem
		if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
			return false;

		// get the controller input

		QPSolution = solver.getSolution();
		this->currentControlsSol_.clear();
		this->currentStatesSol_.clear();			
		for (int i=0;i<mpcWindow+1;i++){
				state = QPSolution.block(numStates*i, 0, numStates, 1);
				this->currentStatesSol_.push_back(state);
			}
		for (int i=0;i<mpcWindow;i++){
				control = QPSolution.block(numStates*(mpcWindow+1)+numControls*i, 0, numControls, 1);
				this->currentControlsSol_.push_back(control);
			}

		this->firstTime_ = false;
	    return true;
	}

	void mpcPlanner::setDynamicsMatrices(Eigen::Matrix<double, numStates, numStates> &A, Eigen::Matrix<double, numStates, numControls> &B){
		A.setZero();
	    A.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
	    A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * this->ts_;
	    A.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

		B.setZero();
		B.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 1/2 * pow(this->ts_, 2);
		B.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * this->ts_;
		B.block(6, 3, 2, 2) = Eigen::Matrix2d::Identity();
	}


	void mpcPlanner::setInequalityConstraints(Eigen::Matrix<double, numStates, 1> &xMax, Eigen::Matrix<double, numStates, 1> &xMin,
	                              			  Eigen::Matrix<double, numControls, 1> &uMax, Eigen::Matrix<double, numControls, 1> &uMin){
	    // state bound
		xMin <<  -INFINITY, -INFINITY, this->zRangeMin_, -this->maxVel_, -this->maxVel_, -this->maxVel_, -INFINITY, -INFINITY;
	    xMax << INFINITY, INFINITY, this->zRangeMax_, this->maxVel_, this->maxVel_, this->maxVel_, INFINITY, INFINITY;

	    // control bound
		double skslimit = 1.0 - pow((1 - this->staticSlack_), 2);
		double skdlimit = 1.0 - pow((1 - this->dynamicSlack_), 2);
	    uMin << -this->maxAcc_, -this->maxAcc_, -this->maxAcc_, 0.0, 0.0;
	    uMax << this->maxAcc_, this->maxAcc_, this->maxAcc_, skdlimit, skslimit;
	}

	void mpcPlanner::getXRef(std::vector<Eigen::Matrix<double, numStates, 1>>& xRef){
		std::vector<Eigen::Vector3d> referenceTraj;
		this->getReferenceTraj(referenceTraj);
		std::vector<Eigen::Matrix<double, numStates, 1>> xRefTemp;
		Eigen::Matrix<double, numStates, 1> ref;
		ref.setZero();
		for (int i = 0; i<int(referenceTraj.size()); ++i){
			ref(0,0) = referenceTraj[i](0);
			ref(1,0) = referenceTraj[i](1);
			ref(2,0) = referenceTraj[i](2);
			xRefTemp.push_back(ref);
		}
		xRef = xRefTemp;
	}

	void mpcPlanner::setWeightMatrices(Eigen::DiagonalMatrix<double,numStates> &Q, Eigen::DiagonalMatrix<double,numControls> &R){
		Q.diagonal() << 1000.0, 1000.0, 1000.0, 0, 0, 0, 100.0, 1000.0;
	    R.diagonal() << 200.0, 200.0, 200.0, 1.0, 1.0;
	}

	void mpcPlanner::castMPCToQPHessian(const Eigen::DiagonalMatrix<double,numStates> &Q, const Eigen::DiagonalMatrix<double,numControls> &R, int mpcWindow, Eigen::SparseMatrix<double>& hessianMatrix){
		hessianMatrix.resize(numStates * (mpcWindow + 1) + numControls * mpcWindow,
	                         numStates * (mpcWindow + 1) + numControls * mpcWindow);

	    // populate hessian matrix
	    for (int i = 0; i < numStates * (mpcWindow + 1) + numControls * mpcWindow; i++){
	        if (i < numStates * (mpcWindow + 1)){
	            int posQ = i % numStates;
	            float value = Q.diagonal()[posQ];
	            if (value != 0)
	                hessianMatrix.insert(i, i) = value;
	        } 
			else{
	            int posR = i % numControls;
	            float value = R.diagonal()[posR];
	            if (value != 0)
	                hessianMatrix.insert(i, i) = value;
	        }
	    }
	}

	void mpcPlanner::castMPCToQPGradient(const Eigen::DiagonalMatrix<double,numStates> &Q, const std::vector<Eigen::Matrix<double, numStates, 1>>& xRef, int mpcWindow, Eigen::VectorXd& gradient){
		std::vector<Eigen::Matrix<double, numStates, 1>> QxRef;
		for (int i = 0; i < xRef.size(); i++){
			Eigen::Matrix<double, numStates, 1> ref = Q * (-xRef[i]);
			QxRef.push_back(ref);
		}

	    // populate the gradient vector
	    gradient = Eigen::VectorXd::Zero(numStates * (mpcWindow + 1) + numControls * mpcWindow, 1);
	    for (int i = 0; i < (mpcWindow + 1); i++){
			for (int j = 0; j < numStates; j++){
				double value = QxRef[i](j, 0);
				gradient(i*numStates+j, 0) = value;
			}
	    }
	}




	void mpcPlanner::castMPCToQPConstraintMatrix(Eigen::Matrix<double, numStates, numStates> &A, 
												 Eigen::Matrix<double, numStates, numControls> &B, 
											     Eigen::SparseMatrix<double> &constraintMatrix, 
											     int numObs, 
											     int mpcWindow, 
												 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, 
												 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, 
												 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw, 
												 std::vector<std::vector<int>> &isDynamic){
		constraintMatrix.resize(numStates * (mpcWindow+1) + numStates * (mpcWindow+1) + numControls * mpcWindow + numObs * mpcWindow,
		    					numStates * (mpcWindow+1) + numControls * mpcWindow);
		
		// populate linear constraint matrix

		//equality
	    for (int i = 0; i < numStates * (mpcWindow + 1); i++)
	    {
	        constraintMatrix.insert(i, i) = -1;
	    }

	    for (int i = 0; i < mpcWindow; i++)
	        for (int j = 0; j < numStates; j++)
	            for (int k = 0; k < numStates; k++)
	            {
	                float value = A(j, k);
	                if (value != 0)
	                {
	                    constraintMatrix.insert(numStates * (i + 1) + j, numStates * i + k) = value;
	                }
	            }

	    for (int i = 0; i < mpcWindow; i++)
	        for (int j = 0; j < numStates; j++)
	            for (int k = 0; k < numControls; k++)
	            {
	                float value = B(j, k);
	                if (value != 0)
	                {
	                    constraintMatrix.insert(numStates * (i + 1) + j, numControls * i + k + numStates * (mpcWindow + 1))
	                        = value;
	                }
	            }

		//inequality
		for (int i = 0; i < numStates * (mpcWindow + 1) + numControls * mpcWindow; i++)
	    {
	        constraintMatrix.insert(i + (mpcWindow + 1) * numStates, i) = 1;
	    }

		for (int i = 0; i < mpcWindow; i++){
			double cx, cy, cz;
			if (this->currentStatesSol_.size()!=0){
				cx = this->currentStatesSol_[i](0);
				cy = this->currentStatesSol_[i](1);
				cz = this->currentStatesSol_[i](2);
			}
			else{
				cx = this->currPos_(0);
				cy = this->currPos_(1);
				cz = this->currPos_(2);
			}
			for (int j = 0; j < numObs; j++){
				double fxx,fyy,fzz;
				// fxyz = pow((cx-oxyz(j,0))*cos(yaw(j,0))+(cy-oxyz(j,1))*sin(yaw(j,0)), 2)/pow(osize(j,0),2) + pow(-(cx-oxyz(j,0))*sin(yaw(j,0))+(cy-oxyz(j,1))*cos(yaw(j,0)), 2)/pow(osize(j,1),2) + pow((cz-oxyz(j,2)), 2)/pow(osize(j,2),2);
				fxx = 2*((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)))/pow(osize[i](j,0),2)*cos(yaw[i](j,0))+ 2*(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)))/pow(osize[i](j,1),2)*(-sin(yaw[i](j,0)));
				fyy = 2*((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)))/pow(osize[i](j,0),2)*sin(yaw[i](j,0))+ 2*(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)))/pow(osize[i](j,1),2)*(cos(yaw[i](j,0)));
				fzz = 2*((cz-oxyz[i](j,2)))/pow(osize[i](j,2),2);
				// fxx = 2*(cx-oxyz(j,0))/pow(osize(j,0),2);
				// fyy = 2*(cy-oxyz(j,1))/pow(osize(j,1),2);
				// fzz = 2*(cz-oxyz(j,2))/pow(osize(j,2),2);
				constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates*i) = fxx;//x 
				constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates*i+1) = fyy;//y 
				constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates*i+2) = fzz;//z 	
				if (isDynamic[i][j]){
					constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates * (mpcWindow + 1) + numControls*i + 3) = -1;
				}
				else{
					constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates * (mpcWindow + 1) + numControls*i + 4) = -1;
				}
			}
		}
	}

	void mpcPlanner::castMPCToQPConstraintVectors(Eigen::Matrix<double,numStates,1> &xMax, Eigen::Matrix<double,numStates,1> &xMin,
		Eigen::Matrix<double,numControls,1> &uMax, Eigen::Matrix<double,numControls,1> &uMin,
		const Eigen::Matrix<double, numStates, 1>& x0, Eigen::Matrix<double, Eigen::Dynamic, 1> &lowerBound, Eigen::Matrix<double, Eigen::Dynamic, 1> &upperBound, 
		int numObs, int mpcWindow, 
		std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw){

		// evaluate the lower and the upper equality vectors
	    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(numStates * (mpcWindow + 1), 1);
	    Eigen::VectorXd upperEquality;
	    lowerEquality.block(0, 0, numStates, 1) = -x0;
	    upperEquality = lowerEquality;
	    lowerEquality = lowerEquality;

		Eigen::VectorXd lowerInequality
	        = Eigen::MatrixXd::Zero(numStates * (mpcWindow + 1) + numControls * mpcWindow, 1);
	    Eigen::VectorXd upperInequality
	        = Eigen::MatrixXd::Zero(numStates * (mpcWindow + 1) + numControls * mpcWindow, 1);
	    for (int i = 0; i < mpcWindow + 1; i++)
	    {
	        lowerInequality.block(numStates * i, 0, numStates, 1) = xMin;
	        upperInequality.block(numStates * i, 0, numStates, 1) = xMax;
	    }
	    for (int i = 0; i < mpcWindow; i++)
	    {
	        lowerInequality.block(numControls * i + numStates * (mpcWindow + 1), 0, numControls, 1) = uMin;
	        upperInequality.block(numControls * i + numStates * (mpcWindow + 1), 0, numControls, 1) = uMax;
	    }


		Eigen::VectorXd lowerObstacle
	        = Eigen::MatrixXd::Zero(numObs * mpcWindow, 1);
	    Eigen::VectorXd upperObstacle
	        = Eigen::MatrixXd::Ones(numObs * mpcWindow, 1)*INFINITY;
			// Zero(numObs * mpcWindow, 1);
		for (int i = 0; i < mpcWindow; i++){
			double cx, cy, cz;
			if (this->currentStatesSol_.size()!=0){
				cx = this->currentStatesSol_[i](0);
				cy = this->currentStatesSol_[i](1);
				cz = this->currentStatesSol_[i](2);
			}
			else{
				cx = this->currPos_(0);
				cy = this->currPos_(1);
				cz = this->currPos_(2);
			}
			for (int j = 0; j < numObs; j++){
				double fxyz,fxx,fyy,fzz;
				fxyz = pow((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)), 2)/pow(osize[i](j,0),2) + pow(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)), 2)/pow(osize[i](j,1),2) + pow((cz-oxyz[i](j,2)), 2)/pow(osize[i](j,2),2);
				fxx = 2*((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)))/pow(osize[i](j,0),2)*cos(yaw[i](j,0))+ 2*(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)))/pow(osize[i](j,1),2)*(-sin(yaw[i](j,0)));
				fyy = 2*((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)))/pow(osize[i](j,0),2)*sin(yaw[i](j,0))+ 2*(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)))/pow(osize[i](j,1),2)*(cos(yaw[i](j,0)));
				fzz = 2*((cz-oxyz[i](j,2)))/pow(osize[i](j,2),2);
				// fxyz = pow(cx-oxyz(j,0),2)/pow(osize(j,0),2)+pow(cy-oxyz(j,1),2)/pow(osize(j,1),2) + pow(cz-oxyz(j,2),2)/pow(osize(j,2),2);
				// fxx = 2*(cx-oxyz(j,0))/pow(osize(j,0),2);
				// fyy = 2*(cy-oxyz(j,1))/pow(osize(j,1),2);
				// fzz = 2*(cz-oxyz(j,2))/pow(osize(j,2),2);
				lowerObstacle(i*numObs+j) = 1 - fxyz + fxx * cx + fyy * cy + fzz * cz;
			}
		}
		
		lowerBound.resize(numStates * (mpcWindow+1) + numStates * (mpcWindow+1) + numControls * mpcWindow + numObs * mpcWindow, 1);
	    upperBound.resize(numStates * (mpcWindow+1) + numStates * (mpcWindow+1) + numControls * mpcWindow + numObs * mpcWindow, 1);

		lowerBound << lowerEquality, lowerInequality, lowerObstacle;
		upperBound << upperEquality, upperInequality, upperObstacle;
	}



	void mpcPlanner::updateObstacleParam(const std::vector<staticObstacle> &staticObstacles, 
										 int &numObs, 
										 int mpcWindow, 
										 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, 
										 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, 
										 std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw, 
										 std::vector<std::vector<int>> &isDynamic){
		isDynamic.clear();
		isDynamic.resize(mpcWindow);
		int numDynamicOb = int(this->dynamicObstaclesPos_.size());
		int numStaticOb = int(staticObstacles.size());
		numObs = numDynamicOb + numStaticOb;
		oxyz.resize(mpcWindow);
		osize.resize(mpcWindow);
		yaw.resize(mpcWindow);
		for (int j=0; j<mpcWindow; j++){
			oxyz[j].resize(numObs,3);
			osize[j].resize(numObs,3);
			yaw[j].resize(numObs,1);
			isDynamic[j].resize(numObs);
			for (int i=0; i<numDynamicOb; i++){
				oxyz[j](i,0) = this->dynamicObstaclesPos_[i](0);
				oxyz[j](i,1) = this->dynamicObstaclesPos_[i](1);
				oxyz[j](i,2) = this->dynamicObstaclesPos_[i](2);
				osize[j](i,0) = this->dynamicObstaclesSize_[i](0)/2 + this->safetyDist_;
				osize[j](i,1) = this->dynamicObstaclesSize_[i](1)/2 + this->safetyDist_;
				osize[j](i,2) = this->dynamicObstaclesSize_[i](2)/2 + this->safetyDist_;
				yaw[j](i,0) = 0.0;
				isDynamic[j][i] = 1;
			}
			for (int i=0; i<numStaticOb; i++){
				oxyz[j](i+numDynamicOb,0) = staticObstacles[i].centroid(0);
				oxyz[j](i+numDynamicOb,1) = staticObstacles[i].centroid(1);
				oxyz[j](i+numDynamicOb,2) = staticObstacles[i].centroid(2);
				osize[j](i+numDynamicOb,0) = staticObstacles[i].size(0)/2 + this->safetyDist_;
				osize[j](i+numDynamicOb,1) = staticObstacles[i].size(1)/2 + this->safetyDist_;
				osize[j](i+numDynamicOb,2) = staticObstacles[i].size(2)/2 + this->safetyDist_;
				yaw[j](i+numDynamicOb,0) = staticObstacles[i].yaw;
				isDynamic[j][i] = 0;
			}
		}
	}

	
	void mpcPlanner::getReferenceTraj(std::vector<Eigen::Vector3d>& referenceTraj){
		// find the nearest position in the reference trajectory
		double leastDist = std::numeric_limits<double>::max();
		double maxForwardTime = 3.0; // # 3.0s ahead
		int maxForwardIdx = maxForwardTime/this->ts_;
		int startIdx = this->lastRefStartIdx_;
		for (int i=this->lastRefStartIdx_; i<this->lastRefStartIdx_+maxForwardIdx; ++i){
			double dist = (this->currPos_ - this->inputTraj_[i]).norm();
			if (dist < leastDist){
				leastDist = dist;
				startIdx = i;
			}
		}
		this->lastRefStartIdx_ = startIdx; // update start idx

		referenceTraj.clear();
		for (int i=startIdx; i<startIdx+this->horizon_; ++i){
			if (i < int(this->inputTraj_.size())){
				referenceTraj.push_back(this->inputTraj_[i]);
			}
			else{
				referenceTraj.push_back(this->inputTraj_.back());
			}
		}
	}


	void mpcPlanner::getTrajectory(std::vector<Eigen::Vector3d>& traj){
		traj.clear();
		for (int i=0; i<this->horizon_; ++i){
			// DVector states = this->currentStatesSol_.getVector(i);
			Eigen::VectorXd states = this->currentStatesSol_[i];
			Eigen::Vector3d p (states(0), states(1), states(2));
			traj.push_back(p);
		}
	}

	void mpcPlanner::getTrajectory(nav_msgs::Path& traj){
		traj.header.frame_id = "map";
		std::vector<Eigen::Vector3d> trajTemp;
		this->getTrajectory(trajTemp);
		for (int i=0; i<int(trajTemp.size()); ++i){
			geometry_msgs::PoseStamped ps;
			ps.pose.position.x = trajTemp[i](0);
			ps.pose.position.y = trajTemp[i](1);
			ps.pose.position.z = trajTemp[i](2);
			traj.poses.push_back(ps);
		}
	}

	Eigen::Vector3d mpcPlanner::getPos(double t){
		// int idx = int(t/this->ts_);
		// idx = std::max(0, std::min(idx, this->horizon_-1));
		// DVector states = this->currentStatesSol_.getVector(idx);
		// Eigen::Vector3d p (states(0), states(1), states(2));
		int idx = floor(t/this->ts_);
		double dt = t-idx*this->ts_;
		idx = std::max(0, std::min(idx, this->horizon_-1));
		Eigen::VectorXd states = this->currentStatesSol_[idx];
		Eigen::VectorXd nextStates = this->currentStatesSol_[std::min(idx+1, this->horizon_-1)];
		Eigen::Vector3d p;
		p(0) = states(0)+(nextStates(0)-states(0))/this->ts_*dt;
		p(1) = states(1)+(nextStates(1)-states(1))/this->ts_*dt;
		p(2) = states(2)+(nextStates(2)-states(2))/this->ts_*dt;
		return p;
	}

	Eigen::Vector3d mpcPlanner::getVel(double t){
		// int idx = int(t/this->ts_);
		// idx = std::max(0, std::min(idx, this->horizon_-1));
		// DVector states = this->currentStatesSol_.getVector(idx);
		// Eigen::Vector3d v (states(3), states(4), states(5));
		int idx = floor(t/this->ts_);
		double dt = t-idx*this->ts_;
		idx = std::max(0, std::min(idx, this->horizon_-1));
		Eigen::VectorXd states = this->currentStatesSol_[idx];
		Eigen::VectorXd nextStates = this->currentStatesSol_[std::min(idx+1, this->horizon_-1)];
		Eigen::Vector3d v;
		v(0) = states(3)+(nextStates(3)-states(3))/this->ts_*dt;
		v(1) = states(4)+(nextStates(4)-states(4))/this->ts_*dt;
		v(2) = states(5)+(nextStates(5)-states(5))/this->ts_*dt;
		return v;
	}

	Eigen::Vector3d mpcPlanner::getAcc(double t){
		// int idx = int(t/this->ts_);
		// idx = std::max(0, std::min(idx, this->horizon_-1));
		// DVector states = this->currentControlsSol_.getVector(idx);
		// Eigen::Vector3d a (states(0), states(1), states(2));
		int idx = floor(t/this->ts_);
		double dt = t-idx*this->ts_;
		idx = std::max(0, std::min(idx, this->horizon_-1));
		Eigen::VectorXd states = this->currentControlsSol_[idx];
		Eigen::VectorXd nextStates = this->currentControlsSol_[std::min(idx+1, this->horizon_-1)];
		Eigen::Vector3d a;
		a(0) = states(0)+(nextStates(0)-states(0))/this->ts_*dt;
		a(1) = states(1)+(nextStates(1)-states(1))/this->ts_*dt;
		a(2) = states(2)+(nextStates(2)-states(2))/this->ts_*dt;
		return a;
	}

	double mpcPlanner::getTs(){
		return this->ts_;
	}

	double mpcPlanner::getHorizon(){
		return this->horizon_;
	}

	void mpcPlanner::visCB(const ros::TimerEvent&){
		this->publishMPCTrajectory();
		this->publishHistoricTrajectory();
		this->publishLocalCloud();
		this->publishStaticObstacles();
		this->publishDynamicObstacles();
	}

	void mpcPlanner::publishMPCTrajectory(){
		if (not this->firstTime_){
			nav_msgs::Path traj;
			this->getTrajectory(traj);
			this->mpcTrajVisPub_.publish(traj);
		}
	}

	void mpcPlanner::publishHistoricTrajectory(){
		if (not this->firstTime_){
			nav_msgs::Path histTraj;
			histTraj.header.frame_id = "map";
			for (int i=0; i<int(this->trajHist_.size()); ++i){
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = this->trajHist_[i](0);
				ps.pose.position.y = this->trajHist_[i](1);
				ps.pose.position.z = this->trajHist_[i](2);
				histTraj.poses.push_back(ps);
			}
			this->mpcTrajHistVisPub_.publish(histTraj);
		}
	}

	void mpcPlanner::publishLocalCloud(){
		if (this->currCloud_.size() != 0){
			pcl::PointXYZ pt;
			pcl::PointCloud<pcl::PointXYZ> cloud;

			for (int i=0; i<int(this->currCloud_.size()); ++i){
				pt.x = this->currCloud_[i](0);
				pt.y = this->currCloud_[i](1);
				pt.z = this->currCloud_[i](2);
				cloud.push_back(pt);
			}

			cloud.width = cloud.points.size();
			cloud.height = 1;
			cloud.is_dense = true;
			cloud.header.frame_id = "map";

			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			this->localCloudPub_.publish(cloudMsg);		
		}
	}

	void mpcPlanner::publishStaticObstacles(){
		if (this->refinedBBoxVertices_.size() != 0){
		    visualization_msgs::Marker line;
		    visualization_msgs::MarkerArray lines;

		    line.header.frame_id = "map";
		    line.type = visualization_msgs::Marker::LINE_LIST;
		    line.action = visualization_msgs::Marker::ADD;
		    line.ns = "mpc_static_obstacles";  
		    line.scale.x = 0.06;
		    line.color.r = 0;
		    line.color.g = 1;
		    line.color.b = 1;
		    line.color.a = 1.0;
		    line.lifetime = ros::Duration(0.2);
		    Eigen::Vector3d vertex_pose;
		    for(int i=0; i<int(this->refinedBBoxVertices_.size()); ++i){
		        bboxVertex v = this->refinedBBoxVertices_[i];
		        std::vector<geometry_msgs::Point> verts;
		        geometry_msgs::Point p;

				for (int j=0; j<int(v.vert.size());++j){
					p.x = v.vert[j](0); p.y = v.vert[j](1); p.z = v.vert[j](2);
		        	verts.push_back(p);
				}

		        int vert_idx[12][2] = {
		            {0,1},
		            {1,2},
		            {2,3},
		            {0,3},
		            {0,4},
		            {1,5},
		            {3,7},
		            {2,6},
		            {4,5},
		            {5,6},
		            {4,7},
		            {6,7}
		        };
		        for (int j=0;j<12;++j){
		            line.points.push_back(verts[vert_idx[j][0]]);
		            line.points.push_back(verts[vert_idx[j][1]]);
		        }
		        lines.markers.push_back(line);
		        line.id++;
		    }
		    this->staticObstacleVisPub_.publish(lines);		


		    // facing direction	
		    visualization_msgs::Marker currPoseMarker;
	    	currPoseMarker.header.frame_id = "map";
			currPoseMarker.header.stamp = ros::Time();
			currPoseMarker.ns = "currPoseVis";
			currPoseMarker.id = 0;
			currPoseMarker.type = visualization_msgs::Marker::ARROW;
			currPoseMarker.action = visualization_msgs::Marker::ADD;
			currPoseMarker.pose.position.x = this->currPos_(0);
			currPoseMarker.pose.position.y = this->currPos_(1);
			currPoseMarker.pose.position.z = this->currPos_(2);
			// double angle = atan2(this->currVel_(1), this->currVel_(0));
			currPoseMarker.pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, this->angle_);
			currPoseMarker.lifetime = ros::Duration(0.2);
			currPoseMarker.scale.x = 0.4;
			currPoseMarker.scale.y = 0.2;
			currPoseMarker.scale.z = 0.2;
			currPoseMarker.color.a = 1.0;
			currPoseMarker.color.r = 0.0;
			currPoseMarker.color.g = 0.5;
			currPoseMarker.color.b = 1.0;
			this->facingPub_.publish(currPoseMarker);
		}
	}

	void mpcPlanner::publishDynamicObstacles(){
		if (this->dynamicObstaclesPos_.size() != 0){
		    visualization_msgs::Marker line;
		    visualization_msgs::MarkerArray lines;

		    line.header.frame_id = "map";
		    line.type = visualization_msgs::Marker::LINE_LIST;
		    line.action = visualization_msgs::Marker::ADD;
		    line.ns = "mpc_dynamic_obstacles";  
		    line.scale.x = 0.06;
		    line.color.r = 0;
		    line.color.g = 0;
		    line.color.b = 1;
		    line.color.a = 1.0;
		    line.lifetime = ros::Duration(0.1);

			for (int i = 0; i<this->dynamicObstaclesPos_.size();i++){
				double x = this->dynamicObstaclesPos_[i](0); 
				double y = this->dynamicObstaclesPos_[i](1); 
				double z = this->dynamicObstaclesPos_[i](2);
				double x_width = this->dynamicObstaclesSize_[i](0);
				double y_width = this->dynamicObstaclesSize_[i](1);
				double z_width = this->dynamicObstaclesSize_[i](2);
				
				std::vector<geometry_msgs::Point> verts;
				geometry_msgs::Point p;
				// vertice 0
				p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
				verts.push_back(p);

				// vertice 1
				p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
				verts.push_back(p);

				// vertice 2
				p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
				verts.push_back(p);

				// vertice 3
				p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
				verts.push_back(p);

				// vertice 4
				p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
				verts.push_back(p);

				// vertice 5
				p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
				verts.push_back(p);

				// vertice 6
				p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
				verts.push_back(p);

				// vertice 7
				p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
				verts.push_back(p);
				
				int vert_idx[12][2] = {
					{0,1},
					{1,2},
					{2,3},
					{0,3},
					{0,4},
					{1,5},
					{3,7},
					{2,6},
					{4,5},
					{5,6},
					{4,7},
					{6,7}
				};
				
				for (size_t i=0;i<12;i++){
					line.points.push_back(verts[vert_idx[i][0]]);
					line.points.push_back(verts[vert_idx[i][1]]);
				}
				
				lines.markers.push_back(line);
				
				line.id++;
			}
		    this->dynamicObstacleVisPub_.publish(lines);	
		}	
	}
}