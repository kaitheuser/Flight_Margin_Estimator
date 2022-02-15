//
//  main.cpp
//  Zipline_Interview: Flight Margin Estimator
//
//  Created by Kai Chuen Tan on 2/12/22.
//

// Import necessary C++ standard libraries.
//-----------------------------------------
#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#define _USE_MATH_DEFINES

// Declare and Define Parameters for the flight_Margin_Est() function.
//--------------------------------------------------------------------
double UAV_PWR_Usage = 50.5;                          // UAV power consumption at constant 30 m/s airspeed [W].
double UAV_BATT_Wh = 570.0;                           // UAV fully charged 6S 25000mAh 22.8 V battery [Wh].

// Additional UAV Flight Parameters.
//----------------------------------
double wayPt_Radius = 10.0;                           // Target is considered reached if the UAV is within the waypoint distance range [m].
double UAV_airSPD = 30.0;                             // UAV airspeed [m/s] 30 m/s = 108 km/hr.

// Fligh Margin Estimator Settings.
//---------------------------------
double path_Res = 1.0;                                // Path resolution/step [m].
double UAV_BATT_remaining_Wh;                         // Remaining battery energy after mission [Wh].

// Structs Declaration.
//---------------------
// Struct that contains UAV status information.
struct UAV_status {
    double x;                                        // x-coordinate [m].
    double y;                                        // y-coordinate [m].
    double heading;                                  // heading angle from the north [rad].
};

// Struct that contains wind sample information.
struct wind_Sample {
    double x;                                        // x-coordinate [m].
    double y;                                        // y-coordinate [m].
    double speed;                                    // wind speed [m/s].
    double angle;                                    // wind angle from the north [rad].
};

// Struct that contains waypoint coordinate.
struct wayPt {
    double x;                                        // x-coordinate [m].
    double y;                                        // y-coordinate [m].
};



// Global Function Declaration.
//-----------------------------
/*
    Function that calculates total energy required to complete a flight path mission
    and outputs remaining battery energy [Wh].
*/
double flight_Margin_Est(double BATT_Wh, std::list<wayPt> wayPts, std::list<wind_Sample> wind_Samples, double airSPD, double PWR_Usage);

/*
    Function that perform pythagorean theorem to calculate the distance between two points.
 */
double euclidean_dist(double curr_loc_x, double curr_loc_y, double next_wayPt_x, double next_wayPt_y);

/*
    Function that determines the heading angle of the UAV.
 */
double heading_angle_UAV(double curr_loc_x, double curr_loc_y, double next_wayPt_x, double next_wayPt_y);

/*
    Function that calculates the weighted average wind component at the UAV location
 */
wind_Sample weighted_AVG_Wind(UAV_status uav_status, std::list<wind_Sample> wind_Samples);

/*
    Function that calculates the UAV ground speed.
 */
double ground_SPD(UAV_status uav_status, wind_Sample local_weighted_AVG_Wind, double airSPD);



// Main function.
//---------------
int main(int argc, const char * argv[]) {
    
    // Define a Simulated Environment.
    //--------------------------------
    // Create 4 waypoints.
    wayPt wayPt_1; wayPt_1.x = 10.0; wayPt_1.y = 10.0;
    wayPt wayPt_2; wayPt_2.x = 30000.0; wayPt_2.y = 10.0;
    wayPt wayPt_3; wayPt_3.x = 30000.0; wayPt_3.y = 30000.0;
    wayPt wayPt_4; wayPt_4.x = 10.0; wayPt_4.y = 30000.0;
    // Store waypoints in the waypoints list.
    std::list<wayPt> wayPts = {wayPt_1, wayPt_2, wayPt_3, wayPt_4};
    
    // Create 5 wind samples.
    wind_Sample wind_Sample_1; wind_Sample_1.x = 5500.0   ; wind_Sample_1.y = 5.0     ; wind_Sample_1.speed = 12.0 ; wind_Sample_1.angle = M_PI/4;
    wind_Sample wind_Sample_2; wind_Sample_2.x = 5.0    ; wind_Sample_2.y = 5500.0    ; wind_Sample_2.speed = 5.5  ; wind_Sample_2.angle = M_PI * 3/4;
    wind_Sample wind_Sample_3; wind_Sample_3.x = 5500.0   ; wind_Sample_3.y = 5500.0    ; wind_Sample_3.speed = 8.0  ; wind_Sample_3.angle = M_PI * 5/4;
    wind_Sample wind_Sample_4; wind_Sample_4.x = 10005.0  ; wind_Sample_4.y = 5500.0    ; wind_Sample_4.speed = 7.0  ; wind_Sample_4.angle = M_PI * 7/4;
    wind_Sample wind_Sample_5; wind_Sample_5.x = 5500.0   ; wind_Sample_5.y = 10005.0   ; wind_Sample_5.speed = 15.5 ; wind_Sample_5.angle = M_PI/6;
    // Store wind samples in the waypoints list.
    std::list<wind_Sample> wind_Samples = {wind_Sample_1, wind_Sample_2, wind_Sample_3, wind_Sample_4, wind_Sample_5};
    
    // Run Flight Margin Estimator.
    //-----------------------------
    // Determine remaining battery energy after the mission [Wh].
    UAV_BATT_remaining_Wh = flight_Margin_Est(UAV_BATT_Wh, wayPts, wind_Samples, UAV_airSPD, UAV_PWR_Usage);
    
    // Display the Flight Margin Estimator output.
    //--------------------------------------------
    // Print out the UAV's remaining battery after the mission [Wh].
    std::cout << "Zipline UAV's Remaining Battery Energy:\t" << UAV_BATT_remaining_Wh << " Wh" << std::endl;
    
    return 0;
}

// Global Function Definition
//---------------------------
/*
    Function that calculates total energy required to complete a flight path mission
    and outputs remaining battery energy [Wh].
*/
double flight_Margin_Est(double BATT_Wh, std::list<wayPt> wayPts, std::list<wind_Sample> wind_Samples, double airSPD, double PWR_Usage){
    
    // Initialization and Declaration.
    //--------------------------------
    double total_Flight_Time = 0;                                                           // Total flight time [seconds].
    double BATT_remaining_Wh;                                                               // Remaining battery energy [Wh].
    
    
    // Extract Information from Inputs.
    //---------------------------------
    wayPt starting_Pt = wayPts.front();                                                     // Get the first waypoint.
    UAV_status uav_status; uav_status.x = starting_Pt.x; uav_status.y = starting_Pt.y;      // Update UAV GPS location.
    
    // Convert waypoints list to a 2D vector.
    size_t num_wayPts = wayPts.size();                                                      // Get number of waypoints.
    size_t wayPt_ID = 0;                                                                    // Waypoint Number.
    double wayPts_2D_arr[num_wayPts][2];                                                    // Waypoints in 2D array.
    for (auto ptr = wayPts.begin(); ptr != wayPts.end(); ++ptr){
        wayPts_2D_arr[wayPt_ID][0] = ptr -> x;                                              // Column 0 is x-coordinate.
        wayPts_2D_arr[wayPt_ID][1] = ptr -> y;                                              // Column 1 is y-coordinate.
        wayPt_ID += 1;
    }
    
    // Convert wind samples list to a 2D vector.
    size_t num_wind_Samples = wind_Samples.size();                                          // Get number of wind samples.
    size_t wind_Sample_ID = 0;                                                              // Wind Sample Number.
    double wind_Samples_2D_arr[num_wind_Samples][4];                                        // Wind Samples in 2D array.
    for (auto ptr = wind_Samples.begin(); ptr != wind_Samples.end(); ++ptr){
        wind_Samples_2D_arr[wind_Sample_ID][0] = ptr -> x;                                  // Column 0 is x-coordinate.
        wind_Samples_2D_arr[wind_Sample_ID][1] = ptr -> y;                                  // Column 1 is y-coordinate.
        wind_Samples_2D_arr[wind_Sample_ID][2] = ptr -> speed;                              // Column 2 is wind speed.
        wind_Samples_2D_arr[wind_Sample_ID][3] = ptr -> angle;                              // Column 3 is wind direction.
        wind_Sample_ID += 1;
    }
    
    // Start running the mission (Set second waypoint as the first target)
    for (size_t wayPt_ID = 1; wayPt_ID < num_wayPts; ++wayPt_ID){
        
        // Initialize a flag to determine whether the UAV arrived at the waypoint.
        double dist_2_wayPt = euclidean_dist(uav_status.x, uav_status.y, wayPts_2D_arr[wayPt_ID][0], wayPts_2D_arr[wayPt_ID][1]);
        
        // Update UAV heading angle
        uav_status.heading = heading_angle_UAV(uav_status.x, uav_status.y, wayPts_2D_arr[wayPt_ID][0], wayPts_2D_arr[wayPt_ID][1]);
        
        // While the UAV is still on the way to the next waypoint.
        while(dist_2_wayPt > wayPt_Radius){
            
            // Get local weighted average wind sample
            wind_Sample local_weighted_AVG_Wind = weighted_AVG_Wind(uav_status, wind_Samples);
            
            // Get the UAV ground speed
            double curr_UAV_ground_SPD = ground_SPD(uav_status, local_weighted_AVG_Wind, airSPD);
            
            // Update Total Flight Time
            total_Flight_Time += path_Res/curr_UAV_ground_SPD;
            
            // Update UAV current location
            uav_status.x += path_Res * sin(uav_status.heading);
            uav_status.y += path_Res * cos(uav_status.heading);
            
            // Calculate new distance to waypoint
            dist_2_wayPt = euclidean_dist(uav_status.x, uav_status.y, wayPts_2D_arr[wayPt_ID][0], wayPts_2D_arr[wayPt_ID][1]);
        }
    }
    
    // Calculate the remaining battery energy [Wh].
    BATT_remaining_Wh = BATT_Wh - PWR_Usage * total_Flight_Time /3600;
    std::cout << "Estimated Total Flight Time: " << total_Flight_Time/3600 << "hour" << std::endl;
    
    // Battery is low if each cell is less than 3.6V
    if (BATT_remaining_Wh > 525 && BATT_remaining_Wh <= 540){
        // Insufficient battery energy
        std::cout << "Warning! Zipline UAV battery low. Each battery cell must not fall below 3.5 V." << std::endl;
    }
    // Each cell must not below 3.5V
    else if (BATT_remaining_Wh < 525){
        // Insufficient battery energy
        std::cout << "Warning! Zipline UAV will not have sufficient battery energy to complete the mission." << std::endl;
    }
    else if (BATT_remaining_Wh < 0){
        // Insufficient battery energy
        BATT_remaining_Wh = 0.0;
        std::cout << "Warning! Zipline UAV will be dead before completing the mission." << std::endl;
    }
        
    return BATT_remaining_Wh;
}

/*
    Function that perform pythagorean theorem to calculate the distance between two points
 */
double euclidean_dist(double curr_loc_x, double curr_loc_y, double next_wayPt_x, double next_wayPt_y){
    return sqrt(pow(next_wayPt_x - curr_loc_x, 2) + pow(next_wayPt_y - curr_loc_y, 2));
}

/*
    Function that calculates the weighted average wind component at the UAV location
 */
wind_Sample weighted_AVG_Wind(UAV_status uav_status, std::list<wind_Sample> wind_Samples){
    
    // Initialization and Declaration.
    //--------------------------------
    std::vector<double> wind_Sample_weights;                                                // Wind sample weights
    double weighted_AVG_SPD = 0;                                                            // Weighted average wind speed [m/s]
    double weighted_AVG_angle = 0;                                                          // Weighted average wind direction [rad]
    wind_Sample weighted_AVG_wind;                                                          // Weight average wind component list
    
    
    // Convert wind samples list to a 2D vector.
    size_t num_wind_Samples = wind_Samples.size();                                          // Get number of wind samples.
    size_t wind_Sample_ID = 0;                                                              // Wind Sample Number.
    double wind_Samples_2D_arr[num_wind_Samples][4];                                        // Wind Samples in 2D array.
    for (auto ptr = wind_Samples.begin(); ptr != wind_Samples.end(); ++ptr){
        wind_Samples_2D_arr[wind_Sample_ID][0] = ptr -> x;                                  // Column 0 is x-coordinate.
        wind_Samples_2D_arr[wind_Sample_ID][1] = ptr -> y;                                  // Column 1 is y-coordinate.
        wind_Samples_2D_arr[wind_Sample_ID][2] = ptr -> speed;                              // Column 2 is wind speed.
        wind_Samples_2D_arr[wind_Sample_ID][3] = ptr -> angle;                              // Column 3 is wind direction.
        wind_Sample_ID += 1;
    }
    
    // Calculate the sum of distances between the UAV location and a wind sample
    double sum_dist = 0;
    for (size_t idx = 0; idx < num_wind_Samples; ++idx){
        double dist = euclidean_dist(uav_status.x, uav_status.y, wind_Samples_2D_arr[idx][0], wind_Samples_2D_arr[idx][1]);
        wind_Sample_weights.push_back(dist);                                                // Store the weights to the vector.
        sum_dist += dist;
    }
    
    
    // Finalize wind sample weights (greater the distance, lower the weight) Reversed.
    double rev_sum_dist = 0;
    for (size_t idx = 0; idx < num_wind_Samples; ++idx){
        double rev_dist = sum_dist -  wind_Sample_weights[idx];
        wind_Sample_weights[idx] = rev_dist;                                                // Update the weights to the vector.
        rev_sum_dist += rev_dist;
    }
    for (size_t idx = 0; idx < num_wind_Samples; ++idx){
        wind_Sample_weights[idx] = wind_Sample_weights[idx]/rev_sum_dist;                   // Update the weights to the vector.
    }
    
    // Find weighted resultant wind speed and direction.
    double sum_of_X = 0;
    double sum_of_Y = 0;
    for (size_t idx = 0; idx < num_wind_Samples; ++idx){
        // Quadrant I
        if (wind_Samples_2D_arr[idx][3] >= 0 && wind_Samples_2D_arr[idx][3] <= M_PI/2) {
            sum_of_X += wind_Samples_2D_arr[idx][2] * sin(wind_Samples_2D_arr[idx][3]) * wind_Sample_weights[idx];
            sum_of_Y += wind_Samples_2D_arr[idx][2] * cos(wind_Samples_2D_arr[idx][3]) * wind_Sample_weights[idx];
        }
        // Quadrant II
        else if (wind_Samples_2D_arr[idx][3] > M_PI/2 && wind_Samples_2D_arr[idx][3] <= M_PI){
            sum_of_X += wind_Samples_2D_arr[idx][2] * cos(wind_Samples_2D_arr[idx][3] - M_PI/2) * wind_Sample_weights[idx];
            sum_of_Y -= wind_Samples_2D_arr[idx][2] * sin(wind_Samples_2D_arr[idx][3] - M_PI/2) * wind_Sample_weights[idx];
        }
        // Quadrant III
        else if (wind_Samples_2D_arr[idx][3] > M_PI && wind_Samples_2D_arr[idx][3] <= M_PI * 3/2){
            sum_of_X -= wind_Samples_2D_arr[idx][2] * sin(wind_Samples_2D_arr[idx][3] - M_PI) * wind_Sample_weights[idx];
            sum_of_Y -= wind_Samples_2D_arr[idx][2] * cos(wind_Samples_2D_arr[idx][3] - M_PI) * wind_Sample_weights[idx];
        }
        // Quadrant IV
        else if (wind_Samples_2D_arr[idx][3] > M_PI * 3/2 && wind_Samples_2D_arr[idx][3] < M_PI * 2){
            sum_of_X -= wind_Samples_2D_arr[idx][2] * cos(wind_Samples_2D_arr[idx][3] - M_PI * 3/2) * wind_Sample_weights[idx];
            sum_of_Y += wind_Samples_2D_arr[idx][2] * sin(wind_Samples_2D_arr[idx][3] - M_PI * 3/2) * wind_Sample_weights[idx];
        }
    }
    
    // Resultant weighted average wind speed.
    weighted_AVG_SPD = sqrt(pow(sum_of_X, 2) + pow(sum_of_Y, 2));
    weighted_AVG_wind.speed = weighted_AVG_SPD;
    
    // Calculate the weighted average wind direction.
    if (sum_of_X >= 0 && sum_of_Y >= 0){
        weighted_AVG_angle = M_PI/2 - atan2(sum_of_Y, sum_of_X);
    }
    else if (sum_of_X >= 0 && sum_of_Y < 0){
        weighted_AVG_angle = M_PI/2 - atan2(sum_of_Y, sum_of_X);
    }
    else if (sum_of_X < 0 && sum_of_Y < 0){
        weighted_AVG_angle = M_PI/2 - atan2(sum_of_Y, sum_of_X);
    }
    else if (sum_of_X < 0 && sum_of_Y >= 0){
        weighted_AVG_angle = 5/2 * M_PI - atan2(sum_of_Y, sum_of_X);
    }
    
    // Store the weighted average wind componenet in a list.
    weighted_AVG_wind.angle = weighted_AVG_angle;
    
    return weighted_AVG_wind;
}

/*
    Function that calculates the UAV ground speed.
 */
double ground_SPD(UAV_status uav_status, wind_Sample local_weighted_AVG_Wind, double airSPD){
    
    // Convert Angle that is defined from the angle between north vector to the direction of the wind is blowing.
    double converted_local_weighted_AVG_Wind_angle;
    if (local_weighted_AVG_Wind.angle >= 0 && local_weighted_AVG_Wind.angle < M_PI){
        converted_local_weighted_AVG_Wind_angle = local_weighted_AVG_Wind.angle + M_PI;
    }
    else {
        converted_local_weighted_AVG_Wind_angle = local_weighted_AVG_Wind.angle - M_PI;
    }
    
    // Correction angle.
    double alpha = asin(local_weighted_AVG_Wind.speed/airSPD * sin(converted_local_weighted_AVG_Wind_angle - uav_status.heading));
    
    // Calculate ground speed
    double UAV_ground_SPD = sqrt(pow(airSPD,2) + pow(local_weighted_AVG_Wind.speed, 2) - 2 * airSPD * local_weighted_AVG_Wind.speed * cos(uav_status.heading - converted_local_weighted_AVG_Wind_angle + alpha));
    
    return UAV_ground_SPD;
}

/*
    Function that determines the heading angle of the UAV.
 */
double heading_angle_UAV(double curr_loc_x, double curr_loc_y, double next_wayPt_x, double next_wayPt_y){
    
    // Define North vector
    double north_vec_x = 0;
    double north_vec_y = 1;
    
    // Heading vector
    double head_vec_x = next_wayPt_x - curr_loc_x;
    double head_vec_y = next_wayPt_y - curr_loc_y;
    
    // Calculate course angle.
    double course_angle = acos((north_vec_x * head_vec_x + north_vec_y * head_vec_y) / (sqrt(pow(north_vec_x, 2) + pow(north_vec_y, 2)) * sqrt(pow(head_vec_x, 2) + pow(head_vec_y, 2))));
    // Make sure it is 0 to 2 pi format.
    if (head_vec_x < 0){
        course_angle = 2 * M_PI - course_angle;
    }
    
    return course_angle;
}
