#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Eigen>
#include <stdio.h>

/*
 * this function is to get desired states for specific trajectory, just generated, at time dt.
 * input:
 * dT   -> the time
 * hover_pos -> the desired position where you want quadrotor to hover
 * now_vel -> maybe useless
 *
 * output:
 * desired_pos -> desired position at dT
 * desired_vel -> desired velocity at dT
 * desired_acc -> desired acceleration at dT
 * return:
 * true  -> you have alread configured desired states
 * false -> no desired state
 */
bool
trajectory_control( const double dT,
                    const Eigen::Vector3d hover_pos,
                    const Eigen::Vector3d now_vel,
                    double& end_time,
                    Eigen::Vector3d& desired_pos,
                    Eigen::Vector3d& desired_vel,
                    Eigen::Vector3d& desired_acc )
{
    // if you don't want to use Eigen, then you can use these arrays
    // or you can delete them and use Eigen
    double hover_p[3], now_v[3];
    double desired_p[3], desired_v[3], desired_a[3];
    hover_p[0] = hover_pos.x( );
    hover_p[1] = hover_pos.y( );
    hover_p[2] = hover_pos.z( );
    now_v[0]   = now_vel.x( );
    now_v[1]   = now_vel.y( );
    now_v[2]   = now_vel.z( );
    // your code // please use coefficients from matlab to get desired states

    printf( "x = %.2f, y = %.2f, z = %.2f\n", desired_p[0], desired_p[1], desired_p[2] );

    /*---------------------------------------------------------------------------------*/
    /*--- YOUR CODE FROM HERE --- YOUR CODE FROM HERE --- YOUR CODE FROM HERE ---------*/
    /*---------------------------------------------------------------------------------*/
    //	printf("dt= %.2f\n",dT);
    double max_run_time = 25.0; // set your trajectory max run time.
    double end_position[3];     // set your trajectory end point.
    
    int point_num = 9; // waypoints number
    int poly_order = 7;  // polynomial order
    double T = max_run_time/poly_order;
    double T_segment[point_num] = [0.0, T, 2*T, 3*T, 4*T, 5*T, 6*T, 7*T, 8*T];
    int Segment;
    
    double path[point_num*3] = { 0.0, 0.0, 0.5,
                                 2.0, 1.0, 0.7,
                                 2.5, 2.5, 0.9,
                                 1.0, 3.5, 1.0,
                                 0.0, 2.5, 1.0,
                                -1.0, 3.5, 1.0,
                                -2.5, 2.5, 0.9,
                                -2.0, 1.0, 0.7,
                                 0.0, 0.0, 0.5};
  
    end_position[0] = 0.0;
    end_position[1] = 0.0;
    end_position[2] = 0.5;
    
    double P_x[(poly_order+1)*(point_num-1)] = { 0.00000, 0.00000, 0.00000, 0.00000, 0.11838,-0.05254, 0.00829,-0.00046,
                                                 2.00000, 1.19339,-0.14463,-0.14063, 0.01618, 0.00777,-0.00185, 0.00012,
                                                 2.50000,-0.69884, 0.06063, 0.08059,-0.00874,-0.00296, 0.00071,-0.00004,                                         
                                                 1.00000,-0.21364, 0.01931,-0.03107, 0.00219, 0.00139,-0.00024, 0.00001,
                                                 0.00000,-0.43855, 0.00000, 0.01990, 0.00000,-0.00090, 0.00000, 0.00001,
                                                -1.00000,-0.21364,-0.01931,-0.03107,-0.00219, 0.00139, 0.00024,-0.00004,
                                                -2.50000,-0.69884, 0.06063, 0.08059, 0.00874,-0.00296,-0.00071, 0.00012,
                                                -2.00000, 1.19339, 0.14463,-0.14063,-0.01618, 0.00777, 0.00185,-0.00046, };
    
    double P_y[(poly_order+1)*(point_num-1)] = { 0.00000, 0.00000, 0.00000, 0.00000, 0.05672,-0.02526, 0.00408,-0.00023, 
                                                 1.00000, 0.64883,-0.01383,-0.04912, 0.00951, 0.00319,-0.00105, 0.00008, 
                                                 2.50000, 0.43594, 0.04227, 0.00057,-0.01125,-0.00053, 0.00065,-0.00006, 
                                                 3.50000,-0.12083,-0.20236, 0.01351, 0.01442,-0.00011,-0.00061, 0.00006, 
                                                 2.50000, 0.00000, 0.21936, 0.00000,-0.01630, 0.00000, 0.00062,-0.00006, 
                                                 3.50000, 0.12083,-0.20236,-0.01351, 0.01442, 0.00011,-0.00061, 0.00006,
                                                 2.50000,-0.43594, 0.04227,-0.00057,-0.01125, 0.00053, 0.00065,-0.00008,
                                                 1.00000,-0.64883,-0.01383, 0.04912, 0.00951,-0.00319,-0.00105, 0.00023, };
  
    double P_z[(poly_order+1)*(point_num-1)] = { 0.50000, 0.00000, 0.00000, 0.00000, 0.01129,-0.00496, 0.00079,-0.00004, 
                                                 0.70000, 0.12743,-0.00631,-0.01172, 0.00157, 0.00069,-0.00018, 0.00001, 
                                                 0.90000, 0.01890, 0.00204, 0.00481,-0.00129,-0.00022, 0.00009,-0.00001, 
                                                 1.00000, 0.02423,-0.01154,-0.00143, 0.00103, 0.00007,-0.00006, 0.00000, 
                                                 1.00000, 0.00000, 0.00637, 0.00000,-0.00098, 0.00000, 0.00005, 0.00000, 
                                                 1.00000,-0.02423,-0.01154, 0.00143, 0.00103,-0.00007,-0.00006, 0.00001,
                                                 0.90000,-0.01890, 0.00204,-0.00481,-0.00129, 0.00022, 0.00009,-0.00001, 
                                                 0.70000,-0.12743,-0.00631, 0.01172, 0.00157,-0.00069,-0.00018, 0.00004, }
 
  for (int i = 0; i < point_num; i++)
    {
      if dT > T_segment[i] && dT < T_segment[i+1]
      {
        Segment = i;
        break;
      }
    }
  
    for (int j = 0; j < =Poly_order; j++)
    {
      desired_p[0] = desired_p[0] + P_x[j+(Poly_order+1)*Segment] * pow((dT - T_segment[Segment]), j);
      desired_p[1] = desired_p[1] + P_y[j+(Poly_order+1)*Segment] * pow((dT - T_segment[Segment]), j);
      desired_p[2] = desired_p[2] + P_z[j+(Poly_order+1)*Segment] * pow((dT - T_segment[Segment]), j);
      
      desired_v[0] = desired_v[0] + j * P_x[j+(Poly_order+1)*Segment] * pow((dT - T_segment[Segment]), (j-1));
      desired_v[1] = desired_v[1] + j * P_y[j+(Poly_order+1)*Segment] * pow((dT - T_segment[Segment]), (j-1));
      desired_v[2] = desired_v[2] + j * P_z[j+(Poly_order+1)*Segmentt] * pow((dT - T_segment[Segment]), (j-1));
      
      desired_a[0] = 0;
      desired_a[1] = 0;
      desired_a[2] = 0;
    }
  
    /*---------------------------------------------------------------------------------*/
    /*---- YOUR CODE END HERE ---- YOUR CODE END HERE ---- YOUR CODE END HERE ---------*/
    /*---------------------------------------------------------------------------------*/
    if ( dT < max_run_time )
    {
        // output trajectory
        desired_pos.x( ) = desired_p[0];
        desired_pos.y( ) = desired_p[1];
        desired_pos.z( ) = desired_p[2];
        desired_vel.x( ) = desired_v[0];
        desired_vel.y( ) = desired_v[1];
        desired_vel.z( ) = desired_v[2];
        desired_acc.x( ) = desired_a[0];
        desired_acc.y( ) = desired_a[1];
        desired_acc.z( ) = desired_a[2];
        end_time         = max_run_time;
        return true; // if you have got desired states, true.
    }
    else
    {
        // output end point trajectory
        desired_pos.x( ) = end_position[0];
        desired_pos.y( ) = end_position[1];
        desired_pos.z( ) = end_position[2];
        desired_vel.x( ) = 0.0;
        desired_vel.y( ) = 0.0;
        desired_vel.z( ) = 0.0;
        desired_acc.x( ) = 0.0;
        desired_acc.y( ) = 0.0;
        desired_acc.z( ) = 0.0;
        return false; // if you have got desired states, true.
    }
}

#endif // TRAJECTORY_H
