/* --------------------------------------------------------------------------
 Author: Lucas Vago Santana
 Year: 2019
 Compilation Mode: 
     Release
 Details:
	ArDrone 2.0 + GPS control/navigation program 
 ---------------------------------------------------------------------------*/

#include "ardrone/ardrone.h"

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)

// Handle log data file
FILE *fp;

int main(int argc, char **argv)
{
	// Log data file
	fp = fopen("data.txt", "w");

	// AR.Drone class
	ARDrone d1("192.168.1.1"); //d1 - nomenclature of the drone 1

        // Position control variables
	double d1_xd = 0.0, d1_yd = 0.0, d1_zd = 0.0, d1_psid = 0.0; //positions
	double d1_xdp = 0.0, d1_ydp = 0.0, d1_zdp = 0.0, d1_psidp = 0.0; //velocities
	double d1_xdpp = 0.0, d1_ydpp = 0.0, d1_zdpp = 0.0, d1_psidpp = 0.0; //accelerations
	double d1_xd_ant = 0.0, d1_yd_ant = 0.0, d1_zd_ant = 0.0, d1_psid_ant = 0.0; //auxiliary variables for numerical derivation
	double d1_xdp_ant = 0.0, d1_ydp_ant = 0.0, d1_zdp_ant = 0.0, d1_psidp_ant = 0.0; //auxiliary variables for numerical derivation
	
	// Orientation control variables
	double d1_phid = 0.0, d1_thetad = 0.0, d1_F = 0.0; //
	double d1_Kphi = 2.2, d1_Ktheta = 2.2; // DC gain of the orientation dynamics (fine adjustments can be required)

	// GPS
	double d1_latitude = 0.0, d1_longitude = 0.0, d1_elevation = 0.0;
	double d1_lat0 = 0.0, d1_lng0 = 0.0, d1_Xg = 0.0, d1_Yg = 0.0;

	// Yaw offset
	double d1_psi_off = 0.0;

	// Z offset
	double d1_z_off = 0.0;

	// Drone RAW data variables
	double d1_z = 0.0, d1_phi = 0.0, d1_theta = 0.0, d1_psi = 0.0, d1_vx = 0.0, d1_vy = 0.0, d1_vz = 0.0;
	int d1_battery = 0;

	// Drone Command - The nomenclature follows the CVDrone standard
	double d1_cmd_vx = 0.0, d1_cmd_vy = 0.0, d1_cmd_vz = 0.0, d1_cmd_vr = 0.0;
	double umax = 1.0;

	// Time Loop Control
	int64 last_dt = cvGetTickCount();
	double t = 0.0, dt = 0.0, delta_t = 0.02; //20ms

	// Flags
	bool flag_auto_control = 0;
	bool flag_record = 0;	

	//-----------------------------------------------------------------------------------------
	// KF initialization
	// The following is a double implementation, estimating all the state variables of control
	//-----------------------------------------------------------------------------------------
	// KF matrices initialization
	cv::Mat d1_x = cv::Mat::zeros(4,1, CV_64FC1);  //  x [4 x 1] = [x y xp yp]'
	cv::Mat d1_xx = cv::Mat::zeros(12,1, CV_64FC1); // xx [12 x 1] = [x y z phi theta psi xp yp zp phip thetap psip]'	

	// E - Sigma - Error covariance
	cv::Mat d1_E = cv::Mat::eye(4,4,CV_64FC1);
	cv::Mat d1_EE = cv::Mat::eye(12,12,CV_64FC1);
	
	// Q [4 x 4] - Proccess variance 
	cv::Mat d1_Q = cv::Mat::eye(4,4,CV_64FC1);
	d1_Q.at<double>(0,0) = 0.001; //x
	d1_Q.at<double>(1,1) = 0.001; //y
	d1_Q.at<double>(2,2) = 0.1; //xp
	d1_Q.at<double>(3,3) = 0.1; //yp

	// R1 [2 x 2] - Observation variance 1
	cv::Mat d1_R1 = cv::Mat::eye(2,2,CV_64FC1);
	d1_R1.at<double>(0,0) = 0.01; //xp = cos(psi) * vx - sin(psi) * vy
	d1_R1.at<double>(1,1) = 0.01; //yp = sin(psi) * vx + cos(psi) * vy

	// R2 [2 x 2] - Observation variance 2
	cv::Mat d1_R2 = cv::Mat::eye(2,2,CV_64FC1);
	d1_R2.at<double>(0,0) = 2.0; //x
	d1_R2.at<double>(1,1) = 2.0; //y
	
	// QQ [12 x 12] - Proccess variance
	cv::Mat d1_QQ = cv::Mat::eye(12,12,CV_64FC1);
	d1_QQ.at<double>(0,0) = 0.0; //x
	d1_QQ.at<double>(1,1) = 0.0; //y
	d1_QQ.at<double>(2,2) = 0.0; //z
	d1_QQ.at<double>(3,3) = 0.0; //phi
	d1_QQ.at<double>(4,4) = 0.0; //theta
	d1_QQ.at<double>(5,5) = 0.0; //psi
	d1_QQ.at<double>(6,6) = 5.0; //xp
	d1_QQ.at<double>(7,7) = 5.0; //yp
	d1_QQ.at<double>(8,8) = 5.0; //zp
	d1_QQ.at<double>(9,9) = 5.0; //phip
	d1_QQ.at<double>(10,10) = 5.0; //thetap
	d1_QQ.at<double>(11,11) = 5.0; //psip
	
	// RR [6 x 6] - Observation variance
	cv::Mat d1_RR = cv::Mat::eye(6,6,CV_64FC1);
	d1_RR.at<double>(0,0) = 0.2; // x
	d1_RR.at<double>(1,1) = 0.2; // y
	d1_RR.at<double>(2,2) = 0.2; // z
	d1_RR.at<double>(3,3) = 0.2; // phi
	d1_RR.at<double>(4,4) = 0.2; // theta
	d1_RR.at<double>(5,5) = 0.2; // psi	

	//-----------------------------------------------------------------------------------------
	// KF: General Matrices
	//-----------------------------------------------------------------------------------------
	// G [4 x 4]
	double _G[] = {
		1, 0, delta_t, 0,
		0, 1, 0, delta_t, 
		0, 0, 1, 0, 
		0, 0, 0, 1
	};		
	cv::Mat G(4, 4, CV_64FC1, _G);
			
	// GG [12 x 12]
	double _GG[] = {
		1, 0, 0, 0, 0, 0, delta_t, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, delta_t, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, delta_t, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, delta_t, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, delta_t, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, delta_t,
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
	};		
	cv::Mat GG(12, 12, CV_64FC1, _GG);

	// H1 [2 x 4] - [xp:1 yp:1]'
	double _H1[] = {
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	cv::Mat H1(2, 4, CV_64FC1, _H1);

	// H [2 x 4] - [x:1 y:1]'
	double _H2[] = {
		1, 0, 0, 0,
		0, 1, 0, 0
	};
	cv::Mat H2(2, 4, CV_64FC1, _H2);

	// HH [6 x 12]
	double _HH[] = {
		1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //x
		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //y
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, //z
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, //phi
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, //theta
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 //psi
	};
	cv::Mat HH(6, 12, CV_64FC1, _HH);

	// OOSM treatment
	// Usually TAU = 20, but this parameter can require fine adjustment. 
	// With the GPS firmware version 4.1.0-P1, TAU parameter works fine with (TAU = 20 ~ 40)
	int TAU = 40;
	cv::Mat Gc = cv::Mat::eye(4,4,CV_64FC1);
	for (int i=0; i < TAU; i++){
		Gc = Gc * G;
	}
	Gc = Gc.inv();

	//-----------------------------------------------------------------------------------------
	// Keyboard command menu list
	//-----------------------------------------------------------------------------------------
	printf("--------------------------------------------------------------------------------------\n");
	printf("------------- KEYBOARD COMMAND LIST---------------------------------------------------\n");
	printf("--------------------------------------------------------------------------------------\n");
	printf("		  W,S:  Altitude (Up, Down)\n");
	printf("		  A,D:  Yaw (Left, Right)\n");
	printf("		  I,K:  Nose movement (Forward, Backward) \n");
	printf("		  J,L:  Lateral movement (Left, Right)\n");
	printf("		  ESC:  End/Close\n");
	printf("		SPACE:  Takeoff/Land\n");
	printf("		    E:  Start data log and reset offsets\n");
	printf("		    Q:  Switch to automatic controller (ON/OFF)\n");
	printf("		    M:  Magnetometer calibration\n");
	printf("		    0:  Reset Yaw offset\n");
	printf("            1,2,3,4,5:  Change the movement intensity\n");
	printf("--------------------------------------------------------------------------------------\n");
	printf("------------- EXPERIMENT GUIDE SEQUENCE ----------------------------------------------\n");
	printf("--------------------------------------------------------------------------------------\n");
	printf("1) Take off and put the vehicle in the starting position;\n");	
	printf("2) Press the key 'E' to start data log and reset offsets;\n");	
	printf("3) Press the key 'Q' to switch to the automatic mode;\n");	
	printf("\n");
	printf("For safety try the HOVER experiment before the trajectory one.\n");
	printf("The type of experiment (HOVER or TRAJECTORY) can be changed in the source code. \n");
	printf("--------------------------------------------------------------------------------------\n");
	msleep(5000);
	
	//-----------------------------------------------------------------------------------------
	// Main loop - Finalize pssing the ESC button
	//-----------------------------------------------------------------------------------------
    while (!GetAsyncKeyState(VK_ESCAPE)) {
		//-----------------------------------------------------------------------------------------
		// Sensors readings (vector q)
		//-----------------------------------------------------------------------------------------
		// Battery
        d1_battery = d1.getBatteryPercentage();

		// GPS
		int d1_gps_ok = d1.getPosition(&d1_latitude, &d1_longitude, &d1_elevation);		

		// Velocity
		double d1_velocity = d1.getVelocity(&d1_vx, &d1_vy, &d1_vz);

		// Altitude and BUG correction
		d1_z = d1.getAltitude() - d1_z_off;
		//if(d1_z <= 0.0) d1_z = 0.0; // temporary
		
		// Yaw and its offset
		d1_psi = d1.getYaw() - d1_psi_off;
		if(d1_psi < - M_PI) d1_psi = d1_psi + 2 * M_PI;
		if(d1_psi > + M_PI) d1_psi = d1_psi - 2 * M_PI;

		// Angles
		d1_phi = d1.getRoll();		
		d1_theta = d1.getPitch();

		if(d1_gps_ok){
			if(flag_record){
				//------------------------------------------------------------------------
				// State Estimation
				//------------------------------------------------------------------------
				
				// State Vector definition (values referenced in the global frame)
				#define d1_x1 d1_x.at<double>(0)		//x
				#define d1_x2 d1_x.at<double>(1)		//y
				#define d1_x3 d1_xx.at<double>(2)		//z
				#define d1_x4 d1_xx.at<double>(3)		//phi
				#define d1_x5 d1_xx.at<double>(4)		//theta
				#define d1_x6 d1_xx.at<double>(5)		//psi
				#define d1_x7 d1_xx.at<double>(6)		//xp
				#define d1_x8 d1_xx.at<double>(7)		//yp
				#define d1_x9 d1_xx.at<double>(8)		//zp
				#define d1_x10 d1_xx.at<double>(9)		//phip
				#define d1_x11 d1_xx.at<double>(10)		//thetap
				#define d1_x12 d1_xx.at<double>(11)		//psip

				//------------------------------------------------------------------------
				// Kalman filters - Prediction Step --------------------------------------
				//------------------------------------------------------------------------
				//Drones
				d1_x = G * d1_x;	//  x = [x y xp yp]'
				d1_xx = GG * d1_xx;	// xx = [x y z phi theta psi xp yp zp phip thetap psip]'
		
				// Sigma update
				d1_E = G * d1_E * G.t() + d1_Q;
				d1_EE = GG * d1_EE * GG.t() + d1_QQ;

				//------------------------------------------------------------------------
				// Kalman filter - Update Step -------------------------------------------
				//------------------------------------------------------------------------
				// Convertion GPS to Meters ----------------------------------------------
				double d1_lambda1 = 90.0 * DEG_TO_RAD; // Pre-calibrated offset angle, depends on the map
				
				double R = 6371000.0; //~ Earth radius in meters 
				double d1_lat1 = d1_lat0 * DEG_TO_RAD; // Start Point latitude
				double d1_lat2 = d1_latitude * DEG_TO_RAD; // Current latitude 
				double d1_deltalat = (d1_longitude - d1_lng0) * DEG_TO_RAD; // Delta calculation with longitudes
				double d = acos(sin(d1_lat1)*sin(d1_lat2) + cos(d1_lat1)*cos(d1_lat2)*cos(d1_deltalat)) * R;

				double d1_lambda2 = atan2((d1_lng0 - d1_longitude),(d1_latitude - d1_lat0));

				d1_Xg = d*cos(d1_lambda2-d1_lambda1);
				d1_Yg = d*sin(d1_lambda2-d1_lambda1);
				// ----------------------------------------------------------------------				

				// Z1 [2 x 1] - [xp yp]'
				double _d1_Z1[] = {
					cos(d1_psi) * d1_vx - sin(d1_psi) * d1_vy,
					sin(d1_psi) * d1_vx + cos(d1_psi) * d1_vy
				};
				cv::Mat d1_Z1(2, 1, CV_64FC1, _d1_Z1);

				// Z2 [2 x 1] - [x y]'
				double _d1_Z2[] = {
					d1_Xg,
					d1_Yg
				};
				cv::Mat d1_Z2(2, 1, CV_64FC1, _d1_Z2);
		
				// ZZ - [6 x 1] - [x y z phi theta psi]
				double _d1_ZZ[] = {
					d1_x1,
					d1_x2,
					d1_z,
					d1_phi,
					d1_theta,
					d1_psi
				};
				cv::Mat d1_ZZ(6, 1, CV_64FC1, _d1_ZZ);
				
				// OOSM Kalman - Update 1 (velocity)
				cv::Mat d1_K = cv::Mat::eye(4,4,CV_64FC1);
				d1_K = d1_E * (H1.t()) * (H1 * d1_E * (H1.t()) + d1_R1).inv();
				d1_x = d1_x + d1_K * (d1_Z1 - H1*d1_x);
				cv::Mat I = cv::Mat::eye(4,4,CV_64FC1);
				d1_E = (I - d1_K * H1) * d1_E;
				
				// OOSM Kalman - Update 2 (position)
				cv::Mat d1_xc = Gc * d1_x;
				cv::Mat d1_Ec = Gc * d1_E * Gc.t();
				cv::Mat d1_Sc = H2 * d1_Ec * H2.t() + d1_R2;
				cv::Mat d1_Ecxz = d1_E * Gc.t() * H2.t();
				cv::Mat d1_Wc = d1_Ecxz * d1_Sc.inv();
				d1_x = d1_x + d1_Wc * (d1_Z2 - H2*d1_xc);
				d1_E = d1_E - d1_Ecxz * d1_Sc.inv() * d1_Ecxz.t();

				// Regular Kalman: Tracks all the other states
				cv::Mat d1_KK;
				d1_KK = d1_EE * (HH.t()) * (HH * d1_EE * (HH.t()) + d1_RR).inv();
				d1_xx = d1_xx + d1_KK * (d1_ZZ - HH*d1_xx);
				cv::Mat II = cv::Mat::eye(12,12,CV_64FC1); // Dimension
				d1_EE = (II - d1_KK * HH) * d1_EE;

				//------------------------------------------------------------------------
				// Automatic Controller --------------------------------------------------
				//------------------------------------------------------------------------
				if(flag_auto_control){
					//----------------------------------------------------
					//	Trajectory/Position definition
					//----------------------------------------------------
					int d1_tjt = 0; // 0: Hold 3D position
									// 1: Square
								    // 2: Circle with altitude
									// 3: Yaw and altitude
									// 4: S trajectory
					
					if(d1_tjt == 0){ 
						d1_xd = 0.0;
						d1_yd = 0.0;
						d1_zd = 1.0;
						d1_psid = 0.0;
					}

					if(d1_tjt == 1){
						double it = 10;
						d1_zd = 0.5;
						d1_psid = 0.0;
						
						if(t>=it)
							d1_yd = 5.0;
						
						if(t>=2*it)
							d1_xd = 5.0;
						
						if(t>=3*it)
							d1_yd = 0.0;

						if(t>=4*it)
							d1_xd = 0.0;
						

						if(t>=5*it)
							d1_yd = 5.0;
						
						if(t>=6*it)
							d1_xd = 5.0;

						if(t>=7*it)
							d1_yd = 0.0;

						if(t>=8*it)
							d1_xd = 0.0;
					}

					if(d1_tjt == 2){  // Circle with height
						d1_xd = 3.0 * sin(2.0 * M_PI/10.0 * t);
						d1_yd = 3.0 * cos(2.0 * M_PI/10.0 * t);
						d1_zd = 1.5 + 1.0 * cos(2 * M_PI/10.0 * t);
						d1_psid = 0.0;

						// Desired trajectories derivation
						d1_xdp = (d1_xd - d1_xd_ant) / dt;
						d1_ydp = (d1_yd - d1_yd_ant) / dt;
						d1_zdp = (d1_zd - d1_zd_ant) / dt;
						d1_psidp = (d1_psid - d1_psid_ant) / dt;
						d1_xdpp = (d1_xdp - d1_xdp_ant) / dt;
						d1_ydpp = (d1_ydp - d1_ydp_ant) / dt;
						d1_zdpp = (d1_zdp - d1_zdp_ant) / dt;
						d1_psidpp = (d1_psidp - d1_psidp_ant) / dt;
						d1_xd_ant = d1_xd;
						d1_yd_ant = d1_yd;
						d1_zd_ant = d1_zd;
						d1_psid_ant = d1_psid;
						d1_xdp_ant = d1_xdp;
						d1_ydp_ant = d1_ydp;
						d1_zdp_ant = d1_zdp;
						d1_psidp_ant = d1_psidp;
					}

					if(d1_tjt == 3){ // Yaw and Height
						d1_xd = 0.0;
						d1_yd = 0.0;
						d1_zd = 1.0 + 1.0 * sin(2.0 * M_PI/15.0 * t);
						d1_psid = M_PI/2 * sin(2.0 * M_PI/15.0 * t);

						// Desired trajectories derivation
						d1_xdp = (d1_xd - d1_xd_ant) / dt;
						d1_ydp = (d1_yd - d1_yd_ant) / dt;
						d1_zdp = (d1_zd - d1_zd_ant) / dt;
						d1_psidp = (d1_psid - d1_psid_ant) / dt;
						d1_xdpp = (d1_xdp - d1_xdp_ant) / dt;
						d1_ydpp = (d1_ydp - d1_ydp_ant) / dt;
						d1_zdpp = (d1_zdp - d1_zdp_ant) / dt;
						d1_psidpp = (d1_psidp - d1_psidp_ant) / dt;
						d1_xd_ant = d1_xd;
						d1_yd_ant = d1_yd;
						d1_zd_ant = d1_zd;
						d1_psid_ant = d1_psid;
						d1_xdp_ant = d1_xdp;
						d1_ydp_ant = d1_ydp;
						d1_zdp_ant = d1_zdp;
						d1_psidp_ant = d1_psidp;
					}

					if(d1_tjt == 4){ // S trajectory
						d1_xd = 4.0 * sin(2.0 * M_PI/10.0 * t);
						d1_yd = 4.0 * sin(2.0 * M_PI/30.0 * t);
						d1_zd = 0.5;
						d1_psid = 0.0;

						// Desired trajectories derivation
						d1_xdp = (d1_xd - d1_xd_ant) / dt;
						d1_ydp = (d1_yd - d1_yd_ant) / dt;
						d1_zdp = (d1_zd - d1_zd_ant) / dt;
						d1_psidp = (d1_psid - d1_psid_ant) / dt;
						d1_xdpp = (d1_xdp - d1_xdp_ant) / dt;
						d1_ydpp = (d1_ydp - d1_ydp_ant) / dt;
						d1_zdpp = (d1_zdp - d1_zdp_ant) / dt;
						d1_psidpp = (d1_psidp - d1_psidp_ant) / dt;
						d1_xd_ant = d1_xd;
						d1_yd_ant = d1_yd;
						d1_zd_ant = d1_zd;
						d1_psid_ant = d1_psid;
						d1_xdp_ant = d1_xdp;
						d1_ydp_ant = d1_ydp;
						d1_zdp_ant = d1_zdp;
						d1_psidp_ant = d1_psidp;
					}

					//----------------------------------------------------
					// Controller Implementation 
					//----------------------------------------------------
					
					// Control Parameters
					double m = 0.45, g = 9.81;
					double zpmax = 1.0, Kzp = 1.5, Tauz = 0.5;
					double psipmax = 100.0 * DEG_TO_RAD, Kpsi = 1.0, Taupsi = 0.3;

					double d1_zpp = Kzp*zpmax/Tauz * d1_cmd_vz - d1_x9/Tauz; //d1_x9 is the estimation of the global velocity (see the regular KF implementation in the code)
					
					d1_F = (m/(cos(d1_phi)*cos(d1_theta)))*(g + d1_zpp); // Thrust Force

					// Auxiliary Matrix
					double _d1_f1[] = {
						sin(d1_psi), -cos(d1_psi),
						cos(d1_psi), sin(d1_psi)
					};

					cv::Mat d1_f1(2, 2, CV_64FC1, _d1_f1);			
					
					// PD Controller for translation. Refine the controller gains if necessary					
					double _d1_v_AUX[] = {
						d1_xdpp + 1.5 * (d1_xd - d1_x1) + 2.0 * (d1_xdp - d1_x7), // Kpx = 1.5 | Kdx = 2.0
						d1_ydpp + 1.5 * (d1_yd - d1_x2) + 2.0 * (d1_ydp - d1_x8)  // Kpy = 1.5 | Kdy = 2.0
					};

					cv::Mat d1_v_AUX(2, 1, CV_64FC1, _d1_v_AUX);

					cv::Mat d1_F_aux_sin = (m/d1_F) * d1_f1 * d1_v_AUX;

					// Saturation of the desired angle
					double inc_max = 15 * DEG_TO_RAD; // Same as autopilot (used only during the identification) - Do not change
					double ang_max = 12 * DEG_TO_RAD; // Maximum inclination (During the automatic control) - Change for less or maximum of inc)max
						
					double d1_aPhi = d1_F_aux_sin.at<double>(0);
					double d1_aTheta = d1_F_aux_sin.at<double>(1)/cos(d1_phi);
					
					if(d1_aPhi >= ang_max)
						d1_aPhi = ang_max;
					if(d1_aPhi < -ang_max)
						d1_aPhi = -ang_max;
					if(d1_aTheta >= ang_max)
						d1_aTheta = ang_max;
					if(d1_aTheta < -ang_max)
						d1_aTheta = -ang_max;

					d1_phid = asin(d1_aPhi);
					d1_thetad = asin(d1_aTheta);

					// Ui (for identification procedure only)
					// double u = 0.5 * sin(2 * M_PI/2.5 * t) + 0.3 * sin(2 * M_PI/.5 * t); //rapido
					// double u = 0.5 * sin(2 * M_PI/5 * t) + 0.3 * sin(2 * M_PI/1 * t); //medio
					// double u = 0.5 * sin(2 * M_PI/7.5 * t) + 0.3 * sin(2 * M_PI/1.5 * t); //lento
					
					// d1_Kphi = 2.4; // Refining the DC gain value, if necessary
					// d1_Ktheta = 2.3; // Refining the DC gain value, if necessary

					// Control actions calculation
					// State Vector = [  x     y      z      phi   theta   psi     xp    yp     zp     phip   thetap  psip]'
					// State Vector = [d1_x1  d1_x2  d1_x3  d1_x4  d1_x5  d1_x6  d1_x7  d1_x8  d1_x9  d1_x10  d1_x11  d1_x12]'
					double u_phi = d1_phid/(inc_max * d1_Kphi);
					double u_theta = d1_thetad/(inc_max * d1_Ktheta);
					double u_zp = (Tauz * ( d1_x9/Tauz + d1_zdpp + 4.0 * (d1_zd - d1_x3) + 4.2 * (d1_zdp - d1_x9)))/(Kzp * zpmax);
					double u_psip = (Taupsi * ( d1_x12/Taupsi + d1_psidpp + 5.0 * (d1_psid - d1_x6) + 4.5 * (d1_psidp - d1_x12)))/(Kpsi*psipmax);

					// Simple convertion of nomenclatures to CVDrone standard
					d1_cmd_vx = u_theta;
					d1_cmd_vy = - u_phi;
					d1_cmd_vz = u_zp;
					d1_cmd_vr = u_psip;

					// Saturation (For depuration security reasons - as no effect)
					if(d1_cmd_vy <= -1.0) d1_cmd_vy = -1.0;
					if(d1_cmd_vy >= 1.0)  d1_cmd_vy = 1.0;
					if(d1_cmd_vx <= -1.0) d1_cmd_vx = -1.0;
					if(d1_cmd_vx >= 1.0)  d1_cmd_vx = 1.0;
					if(d1_cmd_vz <= -1.0) d1_cmd_vz = -1.0;
					if(d1_cmd_vz >= 1.0)  d1_cmd_vz = 1.0;
					if(d1_cmd_vr <= -1.0) d1_cmd_vr = -1.0;
					if(d1_cmd_vr >= 1.0)  d1_cmd_vr = 1.0;
				} // if(flag_auto_control) Automatic controller
				
				// Record data
				fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", t, d1_cmd_vx, d1_cmd_vy, d1_cmd_vz, d1_cmd_vr, d1_Xg, d1_Yg, d1_vx, d1_vy, d1_vz, d1_z, d1_phi, d1_theta, d1_psi, d1_phid, d1_thetad, d1_xd, d1_yd, d1_zd, d1_psid, d1_x1, d1_x2, d1_x3, d1_x4, d1_x5, d1_x6, d1_x7, d1_x8, d1_x9, d1_x10, d1_x11, d1_x12);				
			} //if(flag_record)

			//-----------------------------------------------------------------------------------------
			// Data
			//-----------------------------------------------------------------------------------------
			//Show data
			//printf("latitude: %f longitude=%f \n", d1_latitude, d1_longitude);
			printf("       Xd=%2.2f Yd=%2.2f Zd=%2.2f PSId=%2.2f\n", d1_xd, d1_yd, d1_zd, d1_psid);
			printf("%d [%%]  X=%2.2f  Y=%2.2f  Z=%2.2f  PSI=%2.2f\n\n", d1_battery, d1_x1, d1_x2, d1_x3, d1_x6);
			printf("t=%2.1f dt=%f \n\n", t, dt);
	
		} // if(d1_gps_OK)
		else{
			//-----------------------------------------------------------------------------------------
			// Show NO GPS Data
			//-----------------------------------------------------------------------------------------
			printf("---NO-GPS--SIGNAL---\nt=%2.1f dt=%f \n\n", t, dt);
		}

		// Time Loop END -> START
		do{
			dt = ((cvGetTickCount() - last_dt) / cvGetTickFrequency()) * 0.000001; // Elapsed time in [sec]				
		}while(dt <= delta_t);

		// Apply control commands AR.Drone
		if(flag_auto_control){ // Automatic
			d1.move3D(d1_cmd_vx, d1_cmd_vy, d1_cmd_vz, d1_cmd_vr, 0);
		}
		else{ 
			d1.move3D(d1_cmd_vx, d1_cmd_vy, d1_cmd_vz, d1_cmd_vr, !flag_record);			
		}

		// Take off / Landing
		if (KEY_PUSH(VK_SPACE)) {
			if (d1.onGround()) {
				//Takeoff
				d1.takeoff();
			}
			else{
				// Land
				d1.landing();
			}
		}

		// Control and record data flag
		if (KEY_PUSH('Q')){
			flag_auto_control = !flag_auto_control;
		}

		// Start estimation and the experiment
		if (KEY_PUSH('E')){
			t = 0.0;
			d1_psi_off   = d1.getYaw();

			d1_z_off   = d1.getAltitude();

			d1_lat0 = d1_latitude; 
			d1_lng0 = d1_longitude;

			flag_record = !flag_record;
		}

		//Calibration Magnetometer
		if (KEY_PUSH('M')){
			d1.setCalibration();		
		}

		//-----------------------------------------------------------------------------------------
		// Manual controls
		//-----------------------------------------------------------------------------------------
		d1_cmd_vx = 0.0, d1_cmd_vy = 0.0, d1_cmd_vz = 0.0, d1_cmd_vr = 0.0;

		// Manual control
		if (KEY_DOWN('I')){
			d1_cmd_vx =  umax;
		}
		if (KEY_DOWN('K')){
			d1_cmd_vx =  -umax;
		}
		if (KEY_DOWN('J')){
			d1_cmd_vy =  umax;			
		}
		if (KEY_DOWN('L')){
			d1_cmd_vy =  -umax;
		}
		if (KEY_DOWN('W')){
			d1_cmd_vz =  umax;
		}
		if (KEY_DOWN('S')){
			d1_cmd_vz =  -umax;		
		}
		if (KEY_DOWN('A')){
			d1_cmd_vr =  umax;
		}
		if (KEY_DOWN('D')){
			d1_cmd_vr =  -umax;
		}

		if (KEY_DOWN('1')){
			umax = 0.1;
		}

		if (KEY_DOWN('2')){
			umax = 0.25;
		}

		if (KEY_DOWN('3')){
			umax = 0.5;
		}

		if (KEY_DOWN('4')){
			umax = 0.75;
		}

		if (KEY_DOWN('5')){
			umax = 1.0;
		}

		last_dt = cvGetTickCount();
		t += dt;
    }

	//-----------------------------------------------------------------------------------------
	// Closing program
	//-----------------------------------------------------------------------------------------
    // See you
	fclose(fp);
    d1.close();

    return 0;
}
