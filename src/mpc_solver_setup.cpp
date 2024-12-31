/*
*	File: mpc_solver_setup.cpp
*	---------------
*   MPC solver setup code generation
*/
#include <acado_code_generation.hpp>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <vector>
// #include <acado_gnuplot.hpp>
// #include <matrix_vector/vector.hpp>
// #include <stdio.h>

USING_NAMESPACE_ACADO
float my_logf (float a)
{
    float i, m, r, s, t;
    int e;

    m = frexpf (a, &e);
    if (m < 0.666666667f) { // 0x1.555556p-1
        m = m + m;
        e = e - 1;
    }
    i = (float)e;
    /* m in [2/3, 4/3] */
    m = m - 1.0f;
    s = m * m;
    /* Compute log1p(m) for m in [-1/3, 1/3] */
    r =             -0.130310059f;  // -0x1.0ae000p-3
    t =              0.140869141f;  //  0x1.208000p-3
    r = fmaf (r, s, -0.121484190f); // -0x1.f19968p-4
    t = fmaf (t, s,  0.139814854f); //  0x1.1e5740p-3
    r = fmaf (r, s, -0.166846052f); // -0x1.55b362p-3
    t = fmaf (t, s,  0.200120345f); //  0x1.99d8b2p-3
    r = fmaf (r, s, -0.249996200f); // -0x1.fffe02p-3
    r = fmaf (t, m, r);
    r = fmaf (r, m,  0.333331972f); //  0x1.5554fap-2
    r = fmaf (r, m, -0.500000000f); // -0x1.000000p-1
    r = fmaf (r, s, m);
    r = fmaf (i,  0.693147182f, r); //  0x1.62e430p-1 // log(2)
    if (!((a > 0.0f) && (a <= 3.40282346e+38f))) { // 0x1.fffffep+127
        r = a + a;  // silence NaNs if necessary
        if (a  < 0.0f) r = ( 0.0f / 0.0f); //  NaN
        if (a == 0.0f) r = (-1.0f / 0.0f); // -Inf
    }
    return r;
}

float my_erfinvf (float a)
{
    float p, r, t;
    t = fmaf (a, 0.0f - a, 1.0f);
    t = my_logf (t);
    if (fabsf(t) > 6.125f) { // maximum ulp error = 2.35793
        p =              3.03697567e-10f; //  0x1.4deb44p-32 
        p = fmaf (p, t,  2.93243101e-8f); //  0x1.f7c9aep-26 
        p = fmaf (p, t,  1.22150334e-6f); //  0x1.47e512p-20 
        p = fmaf (p, t,  2.84108955e-5f); //  0x1.dca7dep-16 
        p = fmaf (p, t,  3.93552968e-4f); //  0x1.9cab92p-12 
        p = fmaf (p, t,  3.02698812e-3f); //  0x1.8cc0dep-9 
        p = fmaf (p, t,  4.83185798e-3f); //  0x1.3ca920p-8 
        p = fmaf (p, t, -2.64646143e-1f); // -0x1.0eff66p-2 
        p = fmaf (p, t,  8.40016484e-1f); //  0x1.ae16a4p-1 
    } else { // maximum ulp error = 2.35002
        p =              5.43877832e-9f;  //  0x1.75c000p-28 
        p = fmaf (p, t,  1.43285448e-7f); //  0x1.33b402p-23 
        p = fmaf (p, t,  1.22774793e-6f); //  0x1.499232p-20 
        p = fmaf (p, t,  1.12963626e-7f); //  0x1.e52cd2p-24 
        p = fmaf (p, t, -5.61530760e-5f); // -0x1.d70bd0p-15 
        p = fmaf (p, t, -1.47697632e-4f); // -0x1.35be90p-13 
        p = fmaf (p, t,  2.31468678e-3f); //  0x1.2f6400p-9 
        p = fmaf (p, t,  1.15392581e-2f); //  0x1.7a1e50p-7 
        p = fmaf (p, t, -2.32015476e-1f); // -0x1.db2aeep-3 
        p = fmaf (p, t,  8.86226892e-1f); //  0x1.c5bf88p-1 
    }
    r = a * p;
    return r;
}

int main( )
{

    double Ts = 0.1;  // prediction sampling time
    double N  = 31;   // Prediction horizon
    // // double g = 9.8066;
    // // double PI = 3.1415926535897932;
    DifferentialState x;
	DifferentialState y;
	DifferentialState z;
	DifferentialState vx;
	DifferentialState vy;
	DifferentialState vz;
	DifferentialState dummy1;
	DifferentialState dummy2;

	// //Control Input
	Control ax;
	Control ay;
	Control az;
	Control sk_d;
	Control sk_s;

	// // MODEL Definition
	DifferentialEquation f;
	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == ax; 
	f << dot(vy) == ay;
	f << dot(vz) == az;
	f << dot(dummy1) == sk_d;
	f << dot(dummy2) == sk_s;

	// // Least Square Function
	Function h, hN;
	h << x << y << z << ax << ay << az << sk_d << sk_s;
	hN << x << y << z;

	// // setup OCP
	OCP ocp(0.0, N*Ts, N);
	DMatrix Q(8,8);
	// Q.setIdentity(); Q(0,0) = 1; Q(1,1) = 1; Q(2,2) = 10; Q(3,3) = 1.0; Q(4,4) = 1.0; Q(5,5) = 1.0; 
	Q.setIdentity(); Q(0,0) = 10.0; Q(1,1) = 10.0; Q(2,2) = 10.0; Q(3,3) = 1.0; Q(4,4) = 1.0; Q(5,5) = 1.0; 
	Q(6,6) = 100.0;Q(7,7) = 1000.0;
	
	DMatrix QN(3,3);
	QN.setIdentity(); QN(0,0) = 10.0; QN(1,1) = 10.0; QN(2,2) = 10.0;
	// QN.setIdentity(); QN(0,0) = 1; QN(1,1) = 1; QN(2,2) = 1;
	
	// // ocp.minimizeLSQ(Q, h, r); // Objective
	ocp.minimizeLSQ(Q, h); // Objectives
	ocp.minimizeLSQEndTerm(QN, hN);
	// // Dynamic Constraint
	ocp.subjectTo(f); 


	// State constraint
	// double slackRatio = 0.4;
	// double skLimit = 1 - pow((1 - slackRatio), 2);
	// double skLimit = 1.0;

	OnlineData maxZ;
	OnlineData minZ;
	OnlineData maxVel;
	OnlineData maxAcc;
	OnlineData skLimitStatic;
	OnlineData skLimitDynamic;

	// ocp.subjectTo( 0.8 <= z <= 1.2);
	// ocp.subjectTo( -2 <= vx <= 2 );
	// ocp.subjectTo( -2 <= vy <= 2 );
	// ocp.subjectTo( -2 <= vz <= 2 );
	// ocp.subjectTo( -1.5 <= ax <= 1.5 );
	// ocp.subjectTo( -1.5 <= ay <= 1.5 );
	// ocp.subjectTo( -1.5 <= az <= 1.5 );
	// ocp.subjectTo(0 <= sk_d<= 0.64);
	ocp.subjectTo(z-maxZ <= 0); ocp.subjectTo(z-minZ >= 0);
	ocp.subjectTo(vx-maxVel <= 0); ocp.subjectTo(vx+maxVel>=0);
	ocp.subjectTo(vy-maxVel <= 0); ocp.subjectTo(vy+maxVel>=0);
	ocp.subjectTo(vz-maxVel <= 0); ocp.subjectTo(vz+maxVel>=0);
	ocp.subjectTo(ax-maxAcc <= 0); ocp.subjectTo(ax+maxAcc>=0);
	ocp.subjectTo(ay-maxAcc <= 0); ocp.subjectTo(ay+maxAcc>=0);
	ocp.subjectTo(az-maxAcc <= 0); ocp.subjectTo(az+maxAcc>=0);
	ocp.subjectTo(sk_d >= 0); 
	ocp.subjectTo(sk_d - skLimitDynamic <= 0);
	ocp.subjectTo(sk_s >= 0);
	ocp.subjectTo(sk_s - skLimitStatic <= 0);
	// ocp.subjectTo(0 <= sk_d<= 0.2);

	// Ellipsoid Obstacle Constraint
	int numObstacle = 40;
	int numDynamicOb = 8;
	std::vector<OnlineData> obx(numObstacle);
	std::vector<OnlineData> oby(numObstacle);
	std::vector<OnlineData> obz(numObstacle);
	std::vector<OnlineData> obxsize(numObstacle);
	std::vector<OnlineData> obysize(numObstacle);
	std::vector<OnlineData> obzsize(numObstacle);
	std::vector<OnlineData> yaw(numObstacle);
	ocp.setNOD(6+7*numObstacle);

	double varX, varY, varZ;
	varX = 0.04;
	varY = 0.04;
	varZ = 0.01;
	float delta = 0.1;
	// for(int i=0;i<numDynamicOb;i++){
	// 	ocp.subjectTo(pow((x-obx[i])*cos(yaw[i])+(y-oby[i])*sin(yaw[i]), 2)/pow(obxsize[i],2) + pow(-(x-obx[i])*sin(yaw[i])+(y-oby[i])*cos(yaw[i]), 2)/pow(obysize[i],2) + pow((z-obz[i]), 2)/pow(obzsize[i],2) -1.0 + sk_d >=  0 );
	// }
	// for(int i=numDynamicOb;i<numObstacle;i++){
	// 	ocp.subjectTo(pow((x-obx[i])*cos(yaw[i])+(y-oby[i])*sin(yaw[i]), 2)/pow(obxsize[i],2) + pow(-(x-obx[i])*sin(yaw[i])+(y-oby[i])*cos(yaw[i]), 2)/pow(obysize[i],2) + pow((z-obz[i]), 2)/pow(obzsize[i],2) -1.0 + sk_s >=  0 );
	// }
	for (int i=0;i<numObstacle;i++){
		// ocp.subjectTo(pow(x-obx[i], 2)/pow(obxsize[i], 2) + pow(y-oby[i], 2)/pow(obysize[i], 2) + pow(z-obz[i], 2)/pow(obzsize[i],2) -1
		// 					// - my_erfinvf(1-2*delta) * sqrt(2 * (varX*pow(x-obx[i], 2)/pow(obxsize[i], 4) + 
		// 					// 								varY*pow(y-oby[i], 2)/pow(obysize[i], 4) + // with probability
		// 					// 								varZ*pow(z-obz[i], 2)/pow(obzsize[i], 4))
		// 					// 								* 1/(pow((x - obx[i])/obxsize[i], 2) + pow((y - oby[i])/obysize[i], 2) + pow((z - obz[i])/obzsize[i], 2))) 
		// 													>= 0 ) ;
		ocp.subjectTo(sqrt(pow((x-obx[i]), 2)/pow(obxsize[i]/2, 2) + pow((y-oby[i]), 2)/pow(obysize[i]/2, 2) + pow((z-obz[i]), 2)/pow(obzsize[i]/2,2)+1e-8) -1
			            - my_erfinvf(1-2*delta) * sqrt(2 * (varX*pow(x-obx[i], 2)/pow(obxsize[i]/2, 4) + 
			               								varY*pow(y-oby[i], 2)/pow(obysize[i]/2, 4) + // with probability
														varZ*pow(z-obz[i], 2)/pow(obzsize[i]/2, 4))
														* 1/(pow((x - obx[i])/obxsize[i]/2, 2) + pow((y - oby[i])/obysize[i]/2, 2) + pow((z - obz[i])/obzsize[i]/2, 2)+1e-8)+1e-8) 
						>= 0 ) ;	
	}

	// if (obstacle_distance < 5.0){
	// 	// ocp.subjectTo(t,   sqrt(pow((x-pred_ob.x), 2)/pow(pred_ob.xsize/2, 2) + pow((y-pred_ob.y), 2)/pow(pred_ob.ysize/2, 2) + pow((z-pred_ob.z), 2)/pow(pred_ob.zsize/2,2)) -1
	// 	//                >= safe_dist ) ; // without probability
	// 	ocp.subjectTo(t, sqrt(pow((x-pred_ob.x), 2)/pow(pred_ob.xsize/2, 2) + pow((y-pred_ob.y), 2)/pow(pred_ob.ysize/2, 2) + pow((z-pred_ob.z), 2)/pow(pred_ob.zsize/2,2)) -1
	// 			- my_erfinvf(1-2*delta) * sqrt(2 * (pred_ob.varX*pow(x-pred_ob.x, 2)/pow(pred_ob.xsize/2, 4) + 
	// 											pred_ob.varY*pow(y-pred_ob.y, 2)/pow(pred_ob.ysize/2, 4) + // with probability
	// 											pred_ob.varZ*pow(z-pred_ob.z, 2)/pow(pred_ob.zsize/2, 4))
	// 											* 1/(pow((x - pred_ob.x)/pred_ob.xsize/2, 2) + pow((y - pred_ob.y)/pred_ob.ysize/2, 2) + pow((z - pred_ob.z)/pred_ob.zsize/2, 2))) >= 0 ) ;						
	// }

	// //Linearized Obstacle Constraint
	// int numObstacle = 25;
	// int numDynamicOb = 4;
	// ocp.setNOD(numObstacle*4+9);
	// OnlineData cx;
	// OnlineData cy;
	// OnlineData cz;
	// std::vector<OnlineData> fxyz(numObstacle);
	// std::vector<OnlineData> fxx(numObstacle);
	// std::vector<OnlineData> fyy(numObstacle);
	// std::vector<OnlineData> fzz(numObstacle);
	
	// for (int i=0; i<numDynamicOb;i++){
	// 	ocp.subjectTo(fxyz[i]+fxx[i]*(x-cx)+fyy[i]*(y-cy)+fzz[i]*(z-cz)-1+sk_d>=0);
	// }
	// for (int i=numDynamicOb; i<numObstacle;i++){
	// 	ocp.subjectTo(fxyz[i]+fxx[i]*(x-cx)+fyy[i]*(y-cy)+fzz[i]*(z-cz)-1+sk_s>=0);
	// }


	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        30              );

	// mpc.set(FIX_INITIAL_STATE, BT_FALSE);
	mpc.set( QP_SOLVER,                   QP_QPOASES      );
	mpc.set( MAX_NUM_QP_ITERATIONS, 		 1000		   	);
	mpc.set( HOTSTART_QP,                 NO  );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( GENERATE_TEST_FILE,          BT_FALSE             );
	mpc.set( GENERATE_MAKE_FILE,          BT_FALSE             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   BT_FALSE             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, BT_FALSE             );
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );

	if (mpc.exportCode( "./src/CERLAB-UAV-Autonomy-v2/trajectory_planner/include/trajectory_planner/mpc_solver" ) != SUCCESSFUL_RETURN){
		exit( EXIT_FAILURE );
	}
		

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}