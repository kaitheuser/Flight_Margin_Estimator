# Zipline Flight Margin Estimator
## How to run the code:
1.) Download and extract the "Zipline_Flight_Margin_Estimator.zip".

2.) Open a terminal window.

3.) Change the directory to the workspace directory.

`cd Zipline_Flight_Margin_Estimator`

4.) Compile the c++ code.

`g++ -std=c++11 main.cpp`

5.) Run the code.

`./a.out`

## Flight Margin Estimator Pseudocode
1.) Create a simulated environment by defining waypoints and wind samples

2.) Get UAV Starting Location.

3.) Convert Wind Samples and Waypoints lists to 2D arrays.

4.) Start mission by iterating through the waypoints.

5.) Calculate the distance between the UAV current point and the next waypoint.

6.) While the UAV is on the way to the next waypoint,

    a.) Calculate the weighted average wind component with the wind samples.
    b.) Calculate the ground speed of the aircraft with the weighted average wind component.
    c.) Update the total flight time
    d.) Update the location of the UAV with small increment step.
    e.) the distance between the UAV current point and the next waypoint.
    f.) Repeat a.) to e.) if the distance to waypoint is not reached.
7.) Repeat 5.) and 6.) until the UAV visited all the waypoints.
8.) Calculate remaining battery energy in Wh.

## Waypoints and Wind Samples Inputs Requirements
* Waypoints and Wind Samples must be list type.
* In the Waypoints list, it must consists of wayPt structs that have x and y coordinates.
* In the Wind Samples list, it must consists of wind_Sample structs that have x coordinate, y coordinate, speed, and angle.

## Customizable Parameters
1.) `UAV_PWR_Usage` `line 18` - UAV power consumption at constant airspeed [W]
2.) `UAV_BATT_Wh` `line 19` - UAV fully charged battery energy [Wh]
3.) `wayPt_Radius` `line 23`- Target is considered reached if the UAV is within the waypoint distance range [m].
4.) `UAV_airSPD` `line 24`- UAV airspeed [m/s].
5.) `path_Res` `line 28`- Path resolution/step[m]

