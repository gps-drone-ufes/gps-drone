------------------------------------------------------------------------------------------------------
 An Open-Source Testbed for Outdoor Navigation with the AR.Drone Quadrotor 
------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------
This code example was compiled using Windows 7 and Visual Studio 2015.
------------------------------------------------------------------------------------------------------
Important modifications:
  "command.cpp"
  1) Function Move3D:
      Change of the nomenclature of the high-level commands:
        cmd_vx <-> u_theta; 
        cmd_vy <-> -u_phi;
        cmd_vz <-> u_z
        cmd_vr <-> u_psi
      Exclude the internal function gains;
  2) Change the maximum values for the controller
      Set vz_max for 1 [m/s]: sockCommand.sendf("AT*CONFIG=%d,\"control:control_vz_max\",\"%d\"\r", ++seq, 1000);
      Set inc_max for 15 [°]: sockCommand.sendf("AT*CONFIG=%d,\"control:euler_angle_max\",\"%f\"\r", ++seq, 15.0 * DEG_TO_RAD);
      Set vr_max for 100 [°/s]: sockCommand.sendf("AT*CONFIG=%d,\"control:control_yaw\",\"%f\"\r", ++seq, 100.0 * DEG_TO_RAD);
      Set z_max for 10 [m]:  sockCommand.sendf("AT*CONFIG=%d,\"control:altitude_max\",\"%d\"\r", ++seq, 10000);
  
  "navdata.cpp"
  1) Disable bootstrap mode
      sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", ++seq);
