/**
 * @file   fcs_landing.c
 * @author Tim Bakker (bakkert@vcu.edu)
 * @date   thu dec 17 22:45:38 2014
 * @copyright 2014 virginia commonwealth university.
 * 
 * @brief  landing functionality
 * 
 */

#include <stdio.h>
#include "fcs_fixed.h"
#include "../nav.h"
#include "fcs_atol.h"
#include "fixed_fcs_data.h"
#include "fixed_fcs_types.h"
#include "../fcs_math.h"
#include "../fcs_data.h"
#include "../fcs_comm.h"
#include "fcs_landing.h"
#include "fixed_fcs_comm.h"
#include "../defines.h"

#include "pid.h"

static void landing_reset(void);
static void landing_traffic(float);
static void landing_align_runway(float _dt);
static void landing_pre_glidepath(float _dt);
static void landing_glidepath(float);
static void landing_flare_attitude(float _dt);
static void landing_flare_descent(float _target_pitch, float _target_descent_rate, float _dt);
static void landing_runway(float _dt);

#define G_TO_FEET_PER_SECOND2(x) (x * 32.18503937)

#define CT_KI 0.5f
#define CT_I_MAX 4.0f

/**
 * @var descent_rate_error_to_throttle
 * @brief Descent rate error to throttle PID
 */
static pid_controller_t descent_rate_error_to_airspeed_rate = PID_CONTROLLER_INITIALIZER;

/**
 * @var api
 * @brief API access pointer
 *
 * Must be static per-file because having a global "api" symbol can
 * conflict with the global "api" symbol in the Aries code, at least
 * when linked as a static library. 
 */
static api_t *api;


/** 
 * @brief Initialize control code. Just sets the local API pointer
 * 
 * @param api_p 
 */
void landing_init(api_t *api_p) {
    api = api_p;
}


/**
 * @brief Landing logic
 *
 * @param dt Time in seconds since the last iteration
 * @return -1 on abort, 1 on finished and 0 on busy.
 */
int32_t landing_run(float _dt){


    // Update runway heading to heading rate parameters
	update_params(atol_state.runway_heading_to_heading_rate.current,
                  atol_state.runway_heading_to_heading_rate.params,
                  atol_state.runway_heading_to_heading_rate.params + PID_PARAM_LOW_OFFSET,
                  PID_PARAM_COUNT,
                  aircraft_state.airspeed,
                  true);

	// Update descent rate to throttle parameters
	update_params(atol_state.descent_rate_to_airspeed_rate.current,
				  atol_state.descent_rate_to_airspeed_rate.params,
				  atol_state.descent_rate_to_airspeed_rate.params + PID_PARAM_LOW_OFFSET,
				  PID_PARAM_COUNT,
                  aircraft_state.airspeed,
                  true);

    atol_state.cross_track_error = calc_crosstrack_distance(atol_state.glidepath_wp_lat_ext,
                                                            atol_state.glidepath_wp_lon_ext,
                                                            atol_state.runway_settings.stop_lat,
                                                            atol_state.runway_settings.stop_lon,
															aircraft_state.latitude, 
															aircraft_state.longitude);

//fcs_setDebugCoordinate(HIGH_RATE_COORDINATES_SOURCE_FIXED_GLIDEPATH, 0, atol_state.landing_wp_lat, atol_state.landing_wp_lon);
    //fcs_setDebugCoordinate(HIGH_RATE_COORDINATES_SOURCE_FIXED_GLIDEPATH, 1, atol_state.glidepath_wp_lat, atol_state.glidepath_wp_lon);

	// Run different phases
	switch(atol_state.landing_state){
		case LANDING_MODE_init:
		{
			landing_reset();
			atol_state.landing_state = LANDING_MODE_enter_traffic;
			atol_state.abort_error = 0;
            message_report_send(MSG_CODE_info, "Landing initiated");
			return 0;
		}
		case LANDING_MODE_enter_traffic:
		{
			landing_traffic(_dt);

			/**** Determine when this stage is completed ****/
			if(((atol_state.traffic_pattern_state.wp_index + 1) % flightpath_state.wp_count) == flightpath_state.wp_index)
				atol_state.landing_state = LANDING_MODE_traffic;

			/**** Determine ABORT conditions ****/


			return 0;
		}
		case LANDING_MODE_traffic:
		{
            
			landing_traffic(_dt);

			/**** Determine when this stage is completed ****/
			// Stage is completed when we are in between wp0 and wp1 and have reached glidepath altitude within 20 feet
            float distance = calc_distance(aircraft_state.latitude,
                                   aircraft_state.longitude,
                                   flightpath_state.prev_wp->latitude,
                                   flightpath_state.prev_wp->longitude);
            if(flightpath_state.wp_index == 0 &&  distance <  fcs_options.arrival_range && 30 > fabsf(aircraft_state.altitude - atol_state.runway_settings.traffic_pattern_altitude)){
				atol_state.landing_state = LANDING_MODE_align_runway;

			}
			
			/**** Determine ABORT conditions ****/
			
			

			return 0;
		}
		case LANDING_MODE_align_runway:
		{
            printf("ENTERING ALIGN RUNWAY\n");

			landing_align_runway(_dt);

			/**** Determine when this stage is completed ****/
			float heading_error = 0.0f;

            heading_error = fabsf(normalize_degree(aircraft_state.heading - (atol_state.runway_bearing)));
           // } -180.0f
			//else {
            //    heading_error = fabsf(normalize_degree(aircraft_state.heading - atol_state.runway_bearing));
           // }
			// EXIT condition
			printf("%f  <  %f  && 20 > %f ", fabsf(atol_state.cross_track_error), ( .5 * atol_state.runway_settings.runway_width), heading_error);
			if(fabsf(atol_state.cross_track_error) < ( .5 * atol_state.runway_settings.runway_width) && 20.0f > heading_error)
				atol_state.landing_state = LANDING_MODE_pre_glidepath;


			/**** Determine ABORT conditions ****/
			float distance = calc_distance(aircraft_state.latitude, 
										   aircraft_state.longitude, 
										   atol_state.runway_settings.start_lat, 
										   atol_state.runway_settings.start_lon);
			if(((0.5 * atol_state.glidepath_wp_distance) > distance ) && (fabsf(atol_state.cross_track_error) > atol_state.runway_settings.runway_width * 2.0f)){
				atol_state.landing_state = LANDING_MODE_abort;
				atol_state.abort_error = LANDING_ABORT_ar_distance;
                message_report_send(MSG_CODE_error, "Landing aborted, cross-track not within runway width");
			}

			return 0;

		}
        case LANDING_MODE_pre_glidepath:    
        {
            printf("ENTERING PRE GLIDE\n");

            landing_pre_glidepath(_dt);

        	/**** Determine when this stage is completed ****/


            float d_glidepath = calc_distance(atol_state.landing_wp_lat,
                                              atol_state.landing_wp_lon,
                                              atol_state.glidepath_wp_lat,
                                              atol_state.glidepath_wp_lon);
            float d_current = calc_distance(atol_state.landing_wp_lat,
                                            atol_state.landing_wp_lon,
                                            aircraft_state.latitude,
                                            aircraft_state.longitude);


            if ((d_current < d_glidepath ) && (aircraft_state.airspeed <= atol_state.landing_settings.glidepath_airspeed + 10)){
                atol_state.landing_state = LANDING_MODE_glidepath;
                atol_state.target_airspeed = aircraft_state.airspeed;
            }
			/**** Determine ABORT conditions ****/
        	// Abort when target roll angle is above 20 degrees and altitude is below 20 feet
			if((20 > aircraft_state.altitude) && (20 < fabsf(control_state.target_roll))){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_gp_bank;
                message_report_send(MSG_CODE_error, "Landing aborted, to much bank angle");
			}
			

			// Abort when getting to close to the end of the runway
			float distance_stop = 0;
            if(atol_state.runway_settings.traffic_pattern == 0){
                distance_stop = calc_distance(aircraft_state.latitude, aircraft_state.longitude, atol_state.runway_settings.start_lat, atol_state.runway_settings.start_lon);
            } else{
                distance_stop = calc_distance(aircraft_state.latitude, aircraft_state.longitude, atol_state.runway_settings.stop_lat, atol_state.runway_settings.stop_lon);
            }


			// If distance to end of runway is less than half the runway distance
			if(distance_stop < (atol_state.runway_length * 0.3f)){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_gp_distance;
                message_report_send(MSG_CODE_error, "Landing aborted, to close to end of runway");
			}



            return 0;
        
        }
		case LANDING_MODE_glidepath:
        {
            printf("ENTERING GLIDE\n");
			landing_glidepath(_dt);

			/**** Determine when this stage is completed ****/
			if(atol_state.landing_settings.flare_altitude > aircraft_state.altitude){
				atol_state.landing_state = LANDING_MODE_flare_attitude;
			}


			/**** Determine ABORT conditions ****/
			// Abort when cross-track distance is to large below 50 feet
			if(50 > aircraft_state.altitude){
				if(fabsf(atol_state.cross_track_error) > (atol_state.runway_settings.runway_width * 2)){
					atol_state.landing_state = LANDING_MODE_abort1;
					atol_state.abort_error = LANDING_ABORT_gp_crosstrack_distance;
                    message_report_send(MSG_CODE_error, "Landing aborted, cross-track not within runway width");
				}
			}

			// Abort when target roll angle is above 20 degrees and altitude is below 20 feet
			if(20 > aircraft_state.altitude && 20 < fabsf(control_state.target_roll)){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_gp_bank;
                message_report_send(MSG_CODE_error, "Landing aborted, to much bank angle");
			}
			

			// Abort when getting to close to the end of the runway
			float distance_stop = 0;
            if(atol_state.runway_settings.traffic_pattern == 0){
                distance_stop = calc_distance(aircraft_state.latitude, aircraft_state.longitude, atol_state.runway_settings.start_lat, atol_state.runway_settings.start_lon);
            } else {
                distance_stop = calc_distance(aircraft_state.latitude, aircraft_state.longitude, atol_state.runway_settings.stop_lat, atol_state.runway_settings.stop_lon);
            }


			// If distance to end of runway is half the runway distance
			if(distance_stop < (atol_state.runway_length * 0.3f)){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_gp_distance;
                message_report_send(MSG_CODE_error, "Landing aborted, to close to end of runway");
			}

			return 0;
		}
        case LANDING_MODE_flare_attitude:
        {
            printf("LANDING_MODE_flare_attitude:\n");
            float distance = 0;
            distance = calc_distance(aircraft_state.latitude,
                                     aircraft_state.longitude,
                                     atol_state.landing_wp_lat,
                                     atol_state.landing_wp_lon);

            printf("Flare_Att - distance = %03.1f\n", distance);

            landing_flare_attitude(_dt);

    		/**** Determine when this stage is completed ****/
			if(aircraft_state.pitch > 1.0f){
				atol_state.landing_state = LANDING_MODE_flare_descent;
                atol_state.target_airspeed = aircraft_state.airspeed;
            }

			/**** Determine ABORT conditions ****/
				
			// Abort when getting to close to the start of the runway or to close to the end of the runway
			float distance_stop = 0;
            if(atol_state.runway_settings.traffic_pattern == 0){
                distance_stop = calc_distance(aircraft_state.latitude, aircraft_state.longitude, atol_state.runway_settings.start_lat, atol_state.runway_settings.start_lon);
            }else{
                distance_stop = calc_distance(aircraft_state.latitude, aircraft_state.longitude, atol_state.runway_settings.stop_lat, atol_state.runway_settings.stop_lon);
            }


			// If distance to end of runway is half the runway distance
			if(distance_stop < (atol_state.runway_length * 0.3f)){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_fl_distance;
                message_report_send(MSG_CODE_error, "Landing aborted, to close to end of runway");
			}



			// If crosstrack error gets bigger than half runway width
			if(fabsf(atol_state.cross_track_error) > (atol_state.runway_settings.runway_width / 2.2f)){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_fl_crosstrack_distance;
                message_report_send(MSG_CODE_error, "Landing aborted, cross-track not within runway ");
			}

			// If bank angle gets to big
			if(20 < fabsf(control_state.target_roll)){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_fl_bank;
                message_report_send(MSG_CODE_error, "Landing aborted, to much bank angle flare");
			}

            return 0;
            
        }
		case LANDING_MODE_flare_descent:
		{
            printf("LANDING_MODE_flare_descent:\n");
			landing_flare_descent(control_state.target_pitch = atol_state.landing_settings.flare_pitch, atol_state.landing_settings.flare_descent_rate, _dt);
		
            float altitude = aircraft_state.altitude;
	        float descent_rate = -aircraft_state.v_u;

	        float flare_altitude_diff = (altitude - atol_state.landing_settings.flare_altitude);

            float distance = 0;
            distance = calc_distance(aircraft_state.latitude,
                                     aircraft_state.longitude,
                                     atol_state.landing_wp_lat,
                                     atol_state.landing_wp_lon);

            printf("Flare_Descent - distance = %03.1f, target_flare_alt: %03.1f, actual_altitude: %03.1f, error: %03.1f, dr: %03.1f\n", distance, atol_state.landing_settings.flare_altitude, altitude, flare_altitude_diff, fabsf(descent_rate));

			float distance_stop = 0;
            if(atol_state.runway_settings.traffic_pattern == 0){
                distance_stop = calc_distance(aircraft_state.latitude, aircraft_state.longitude, atol_state.runway_settings.start_lat, atol_state.runway_settings.start_lon);
                distance_stop = distance_stop - 250.0f ;/*offset*/
            }else{
                distance_stop = calc_distance(aircraft_state.latitude, aircraft_state.longitude, atol_state.runway_settings.stop_lat, atol_state.runway_settings.stop_lon);
            }


            // If above runway use LIDAR for altitude measurement
            /*if(distance_stop < atol_state.runway_length){
                altitude = api->get_altitude(ALT_LIDAR);
            }*/

			/**** Determine when this stage is completed ****/
			//if descent rate slows while aircraft below 20% of flare altitude
			// ---->   <----  fix this if it needs to be used. Goes in the following 'if'
            printf("NEXT CON  0.5  > %f  && %f > %f \n", fabsf(descent_rate) , (0.2 * atol_state.landing_settings.flare_altitude), (altitude - 1.4));
			if( 0.5 > fabsf(descent_rate) && (0.2 * atol_state.landing_settings.flare_altitude) > (altitude - 1.4))
				atol_state.landing_state = LANDING_MODE_runway;

			/**** Determine ABORT conditions ****/
			// If not above runway
			//8274.045898 > 8108.344238
            printf(" AB CON %f > %f \n", distance_stop , atol_state.runway_length);
            if(distance_stop > atol_state.runway_length){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_fl_distance;
                message_report_send(MSG_CODE_error, "Landing aborted, NOT above runway");
            }

			// If distance to end of runway is less than half the runway distance
			if(distance_stop < (atol_state.runway_length * 0.3f)){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_fl_distance;
                message_report_send(MSG_CODE_error, "Landing aborted, too close to end of runway");
			}

			// If bank angle gets to big
			if(10 < fabsf(control_state.target_roll)){
				atol_state.landing_state = LANDING_MODE_abort1;
				atol_state.abort_error = LANDING_ABORT_fl_bank;
                message_report_send(MSG_CODE_error, "Landing aborted, too much bank angle");
			}

			return 0;
		}
		case LANDING_MODE_runway:
		{
            printf("landing_runway\n");
			landing_runway(_dt);
            float distance = 0;
            distance = calc_distance(aircraft_state.latitude,
                                     aircraft_state.longitude,
                                     atol_state.landing_wp_lat,
                                     atol_state.landing_wp_lon);

            printf("Runway Mode - distance = %03.1f, rudder = %01.2f\n", distance, control_state.target_rudder);

			/**** Determine when this stage is completed ****/
			if(5 > aircraft_state.airspeed){
				atol_state.landing_state  = LANDING_MODE_init;
                
                // Restore old flightpath and send to GCS
            	flightpath_state = atol_state.old_flightpath_state;
                flightpath_set_waypoint(0);
	            
                flightpath_cmd_send();		
                message_report_send(MSG_CODE_info, "Landing completed");

				return 1;
			}

			/**** Determine ABORT conditions ****/
			// NONE

			return 0;
		}
		case LANDING_MODE_abort:

			landing_report_send();
			atol_state.landing_state = LANDING_MODE_init;
			nav_state.nav_mode = NAV_MODE_waypoint;
            flightpath_set_waypoint(0);            
            
			return -1;
		default:
        case LANDING_MODE_abort1: // Transition to takeoff climbout state
            
            landing_report_send();
            atol_state.landing_state = LANDING_MODE_init;
            atol_state.takeoff_state = TAKEOFF_MODE_climbout;
           
            // Set waypoint to wp0
            flightpath_set_waypoint(0);            
			nav_state.nav_mode = NAV_MODE_waypoint;

            atol_state.target_course = calc_bearing(aircraft_state.latitude, 
			                    					aircraft_state.longitude, 
				                				    atol_state.runway_settings.stop_lat, 
								                    atol_state.runway_settings.stop_lon);

			return -2;
	}
}


/**
 * @brief Landing mode initialization, calculating the traffic pattern waypoints
 *
 *	Left traffic pattern
 *	0 ------------------------------------- 3
 *	|										|
 *	|										|
 *	|       ->---->---->---->---->---->---- |
 *	|										|
 *	|										|
 *	1 ------------------------------------- 2
 *
 * Right traffic pattern
 *	1 ------------------------------------- 2
 *	|										|
 *	|										|
 *	|       ->---->---->---->---->---->---- |
 *	|										|
 *	|										|
 *	0 ------------------------------------- 3
 *
 */
static void landing_reset(void){

	/**** Calculate traffic pattern waypoints based on traffic pattern width (settings) and the glidepath pitch and traffic pattern altitude ****/

	atol_state.traffic_pattern_state.wp_count = 4;

	/** Calculate the starting waypoint of the glidepath **/


	// Calculate runway distance
	float runway_length = calc_distance(atol_state.runway_settings.start_lat,
										  atol_state.runway_settings.start_lon,
										  atol_state.runway_settings.stop_lat,
										  atol_state.runway_settings.stop_lon);

    // Calculate landing point on runway 30 feet from start of runway

    atol_state.glidepath_wp_distance = atol_state.runway_settings.traffic_pattern_altitude / tanf(DEG_TO_RAD(atol_state.landing_settings.glidepath_angle));

    if(atol_state.runway_settings.traffic_pattern == 0){
        // Calculate bearing from end to start of runway
        atol_state.runway_bearing = calc_bearing(atol_state.runway_settings.stop_lat,
                                                 atol_state.runway_settings.stop_lon,
                                                 atol_state.runway_settings.start_lat,
                                                 atol_state.runway_settings.start_lon);

        float reverse_runway_bearing = normalize_degree(atol_state.runway_bearing + 180.0f);




        calc_wp_from_ref(atol_state.runway_settings.stop_lat,
                         atol_state.runway_settings.stop_lon,
                         &(atol_state.landing_wp_lat),
                         &(atol_state.landing_wp_lon),
                         reverse_runway_bearing,
                         175.0f);
        calc_wp_from_ref(atol_state.runway_settings.stop_lat,
                         atol_state.runway_settings.stop_lon,
                         &(atol_state.glidepath_wp_lat),
                         &(atol_state.glidepath_wp_lon),
                         reverse_runway_bearing,
                         (atol_state.glidepath_wp_distance));//

        calc_wp_from_ref(atol_state.runway_settings.stop_lat,
                         atol_state.runway_settings.stop_lon,
                         &(atol_state.glidepath_wp_lat_ext),
                         &(atol_state.glidepath_wp_lon_ext),
                         reverse_runway_bearing,
                         atol_state.glidepath_wp_distance + 300);
        float bearing = normalize_degree(reverse_runway_bearing + 90.0f);

        calc_wp_from_ref(atol_state.glidepath_wp_lat_ext,
                         atol_state.glidepath_wp_lon_ext,
                         &(atol_state.traffic_pattern_state.waypoints[0].latitude),
                         &(atol_state.traffic_pattern_state.waypoints[0].longitude),
                         bearing,
                         atol_state.runway_settings.runway_width/2);

//	if(atol_state.runway_settings.traffic_pattern == 0)
//		bearing = normalize_degree(reverse_runway_bearing - 90.0f); // LEFT pattern
//	else
//		bearing = normalize_degree(reverse_runway_bearing + 90.0f); // RIGHT pattern

        // Calculate wp3
        calc_wp_from_ref(atol_state.traffic_pattern_state.waypoints[0].latitude,
                         atol_state.traffic_pattern_state.waypoints[0].longitude,
                         &(atol_state.traffic_pattern_state.waypoints[3].latitude),
                         &(atol_state.traffic_pattern_state.waypoints[3].longitude),
                         bearing,
                         atol_state.runway_settings.traffic_pattern_width);
        // Calculate wp2
        calc_wp_from_ref(atol_state.traffic_pattern_state.waypoints[3].latitude,
                         atol_state.traffic_pattern_state.waypoints[3].longitude,
                         &(atol_state.traffic_pattern_state.waypoints[2].latitude),
                         &(atol_state.traffic_pattern_state.waypoints[2].longitude),
                         atol_state.runway_bearing,
                         runway_length + atol_state.glidepath_wp_distance);
        // Calculate wp1
        calc_wp_from_ref(atol_state.traffic_pattern_state.waypoints[0].latitude,
                         atol_state.traffic_pattern_state.waypoints[0].longitude,
                         &(atol_state.traffic_pattern_state.waypoints[1].latitude),
                         &(atol_state.traffic_pattern_state.waypoints[1].longitude),
                         atol_state.runway_bearing,
                         runway_length + atol_state.glidepath_wp_distance);

    }else{

        atol_state.runway_bearing = calc_bearing(atol_state.runway_settings.start_lat,
                                                 atol_state.runway_settings.start_lon,
                                                 atol_state.runway_settings.stop_lat,
                                                 atol_state.runway_settings.stop_lon);

        float reverse_runway_bearing = normalize_degree(atol_state.runway_bearing + 180.0f);

        calc_wp_from_ref(atol_state.runway_settings.start_lat,
                         atol_state.runway_settings.start_lon,
                         &(atol_state.landing_wp_lat),
                         &(atol_state.landing_wp_lon),
                         atol_state.runway_bearing,
                         75.0f);
        calc_wp_from_ref(atol_state.runway_settings.start_lat,
                         atol_state.runway_settings.start_lon,
                         &(atol_state.glidepath_wp_lat),
                         &(atol_state.glidepath_wp_lon),
                         reverse_runway_bearing,
                         atol_state.glidepath_wp_distance);

        calc_wp_from_ref(atol_state.runway_settings.start_lat,
                         atol_state.runway_settings.start_lon,
                         &(atol_state.glidepath_wp_lat_ext),
                         &(atol_state.glidepath_wp_lon_ext),
                         reverse_runway_bearing,
                         atol_state.glidepath_wp_distance + 300);

        float bearing = normalize_degree(reverse_runway_bearing - 90.0f);
        calc_wp_from_ref(atol_state.glidepath_wp_lat_ext,
                         atol_state.glidepath_wp_lon_ext,
                         &(atol_state.traffic_pattern_state.waypoints[0].latitude),
                         &(atol_state.traffic_pattern_state.waypoints[0].longitude),
                         bearing,
                         atol_state.runway_settings.runway_width/2);

//	if(atol_state.runway_settings.traffic_pattern == 0)
//		bearing = normalize_degree(reverse_runway_bearing - 90.0f); // LEFT pattern
//	else
//		bearing = normalize_degree(reverse_runway_bearing + 90.0f); // RIGHT pattern

        // Calculate wp1
        calc_wp_from_ref(atol_state.traffic_pattern_state.waypoints[0].latitude,
                         atol_state.traffic_pattern_state.waypoints[0].longitude,
                         &(atol_state.traffic_pattern_state.waypoints[3].latitude),
                         &(atol_state.traffic_pattern_state.waypoints[3].longitude),
                         bearing,
                         atol_state.runway_settings.traffic_pattern_width);
        // Calculate wp2
        calc_wp_from_ref(atol_state.traffic_pattern_state.waypoints[3].latitude,
                         atol_state.traffic_pattern_state.waypoints[3].longitude,
                         &(atol_state.traffic_pattern_state.waypoints[2].latitude),
                         &(atol_state.traffic_pattern_state.waypoints[2].longitude),
                         atol_state.runway_bearing,
                         runway_length + atol_state.glidepath_wp_distance);
        // Calculate wp3
        calc_wp_from_ref(atol_state.traffic_pattern_state.waypoints[0].latitude,
                         atol_state.traffic_pattern_state.waypoints[0].longitude,
                         &(atol_state.traffic_pattern_state.waypoints[1].latitude),
                         &(atol_state.traffic_pattern_state.waypoints[1].longitude),
                         atol_state.runway_bearing,
                         runway_length + atol_state.glidepath_wp_distance);

    }


    char buf[100] = "";

    sprintf(buf, "atol_state.landing_wp_lat = %f, atol_state.landing_wp_lon = %f\0",atol_state.landing_wp_lat, atol_state.landing_wp_lon);
    printf("length of string %d\n", strlen(buf));
    printf("%s\n", buf);
//    message_report_send(MSG_CODE_info,buf);

	// Calculate ground distance of the glidepath wp


	 //CHANGE IF NECESSARY - future parameter


    fcs_setDebugCoordinate(HIGH_RATE_COORDINATES_SOURCE_FIXED_GLIDEPATH, 0, atol_state.landing_wp_lat, atol_state.landing_wp_lon);
    fcs_setDebugCoordinate(HIGH_RATE_COORDINATES_SOURCE_FIXED_GLIDEPATH, 1, atol_state.glidepath_wp_lat, atol_state.glidepath_wp_lon);
    fcs_setDebugCoordinate(HIGH_RATE_COORDINATES_SOURCE_FIXED_GLIDEPATH, 2, atol_state.glidepath_wp_lat_ext, atol_state.glidepath_wp_lon_ext);

	//float bearing = 0;
	//if(atol_state.runway_settings.traffic_pattern == 0)
	//	bearing = normalize_degree(reverse_runway_bearing + 90.0f);	// LEFT pattern
	//else
	//	bearing = normalize_degree(reverse_runway_bearing - 90.0f);	// RIGHT pattern

	// Calculate wp0




	/** Determine the waypoint to fly to **/
	int32_t wp_index = 0;
//	float distance = 0;
//	float min_distance = calc_distance(aircraft_state.latitude,
//									   aircraft_state.longitude,
//									   atol_state.traffic_pattern_state.waypoints[0].latitude,
//									   atol_state.traffic_pattern_state.waypoints[0].longitude);
//
//
//	for(int32_t i = 1; i < 4; i++){
//		distance = calc_distance(aircraft_state.latitude,
//								 aircraft_state.longitude,
//								 atol_state.traffic_pattern_state.waypoints[i].latitude,
//								 atol_state.traffic_pattern_state.waypoints[i].longitude);
//
//		// Record lower distance
//		if(distance < min_distance){
//			min_distance = distance;
//			wp_index = i;
//		}
//	}

	// Use the next waypoint to fly to
	atol_state.traffic_pattern_state.wp_index = wp_index;//(wp_index + 1) % 4;
	atol_state.traffic_pattern_state.prev_wp = &(atol_state.traffic_pattern_state.waypoints[(wp_index - 1) %4]);
	atol_state.traffic_pattern_state.current_wp = &(atol_state.traffic_pattern_state.waypoints[wp_index]);
	atol_state.traffic_pattern_state.next_wp = &(atol_state.traffic_pattern_state.waypoints[(wp_index + 1) % 4]);

	// Save old flightpath state and replace with traffic pattern flightpath
	atol_state.old_flightpath_state = flightpath_state;
    flightpath_state = atol_state.traffic_pattern_state;
    flightpath_state.type = WAYPOINT_TYPE_circular;
	
	// Change to normal waypoint mode
	nav_state.nav_mode = NAV_MODE_waypoint;

	// Send waypoint to GCS
	flightpath_cmd_send();		

}


/**
 * @brief Traffic pattern using regular controllers
 *
 * @param _dt Iteration time
 */

static void landing_traffic(float _dt){

	// Fly traffic pattern, using regular controllers (navigation will take care of determing target course)
	
    
	// Calculate target course with crosstrack enabled when entered traffic otherwise just use normal waypoint
    if(atol_state.landing_state == LANDING_MODE_traffic){
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(flightpath_state.prev_wp->latitude, 
                                                           flightpath_state.prev_wp->longitude,	
                                                           flightpath_state.current_wp->latitude,
                                                           flightpath_state.current_wp->longitude,
                                                           aircraft_state.latitude,
                                                           aircraft_state.longitude,
                                                           fcs_options.crosstrack_kp,
                                                           0,
                                                           0,
                                                           _dt);
    }

    control_state.target_roll = target_heading_to_roll(nav_state.target_course, _dt);

	control_state.target_climb_rate = target_altitude_to_climb_rate(atol_state.runway_settings.traffic_pattern_altitude);
    control_state.target_pitch = target_climb_rate_to_pitch(control_state.target_climb_rate,
															control_state.target_roll,
															_dt);
	control_state.target_airspeed = nav_state.target_airspeed;
	control_state.target_yaw = nav_state.target_course;

	// Inner loop stuff
	control_state.target_aileron = target_roll_to_aileron(control_state.target_roll, _dt);
	control_state.target_elevator = target_pitch_to_elevator(control_state.target_pitch, _dt);
	control_state.target_throttle = target_airspeed_to_throttle(control_state.target_airspeed,
                                                                aircraft_state.airspeed,
																control_state.target_roll,
																control_state.target_pitch,
																_dt);

    control_state.target_throttle = limit(control_state.target_throttle, -1, 1);
	control_state.target_yaw = 0;// target_yaw_to_rudder(control_state.target_yaw, _dt);

}


/**
 * @brief Landing align runwayalign
 *
 * @param _dt Iteration time
 */
static void landing_align_runway(float _dt){


	/**** Roll controller ****/
	// Calculate target course with crosstrack enabled

    if(atol_state.runway_settings.traffic_pattern == 0){
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(atol_state.glidepath_wp_lat_ext,
                                                                                     atol_state.glidepath_wp_lon_ext,
                                                                                     atol_state.runway_settings.start_lat,
                                                                                     atol_state.runway_settings.start_lon,
                                                                                     aircraft_state.latitude,
                                                                                     aircraft_state.longitude,
                                                                                     fcs_options.crosstrack_kp,
                                                                                     0,
                                                                                     0,
                                                                                     _dt);

        control_state.target_roll = target_heading_to_roll(atol_state.target_course, _dt);
        control_state.target_aileron = target_roll_to_aileron(control_state.target_roll, _dt);


        /**** Pitch controller ****/
        control_state.target_climb_rate = target_altitude_to_climb_rate(
                atol_state.runway_settings.traffic_pattern_altitude);
        control_state.target_pitch = target_climb_rate_to_pitch(control_state.target_climb_rate,
                                                                control_state.target_roll,
                                                                _dt);
        control_state.target_elevator = target_pitch_to_elevator(control_state.target_pitch, _dt);

        /**** Throttle controller ****/
        control_state.target_airspeed = atol_state.landing_settings.approach_airspeed;
        control_state.target_throttle = target_airspeed_to_throttle(control_state.target_airspeed,
                                                                    aircraft_state.airspeed,
                                                                    control_state.target_roll,
                                                                    control_state.target_pitch,
                                                                    _dt);

        control_state.target_throttle = limit(control_state.target_throttle, -1, 1);

        /**** Rudder controller ****/
        control_state.target_rudder = 0;

    }else {

        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(atol_state.glidepath_wp_lat_ext,
                                                                                     atol_state.glidepath_wp_lon_ext,
                                                                                     atol_state.runway_settings.stop_lat,
                                                                                     atol_state.runway_settings.stop_lon,
                                                                                     aircraft_state.latitude,
                                                                                     aircraft_state.longitude,
                                                                                     fcs_options.crosstrack_kp,
                                                                                     0,
                                                                                     0,
                                                                                     _dt);

        control_state.target_roll = target_heading_to_roll(atol_state.target_course, _dt);
        control_state.target_aileron = target_roll_to_aileron(control_state.target_roll, _dt);


        /**** Pitch controller ****/
        control_state.target_climb_rate = target_altitude_to_climb_rate(
                atol_state.runway_settings.traffic_pattern_altitude);
        control_state.target_pitch = target_climb_rate_to_pitch(control_state.target_climb_rate,
                                                                control_state.target_roll,
                                                                _dt);
        control_state.target_elevator = target_pitch_to_elevator(control_state.target_pitch, _dt);

        /**** Throttle controller ****/
        control_state.target_airspeed = atol_state.landing_settings.approach_airspeed;
        control_state.target_throttle = target_airspeed_to_throttle(control_state.target_airspeed,
                                                                    aircraft_state.airspeed,
                                                                    control_state.target_roll,
                                                                    control_state.target_pitch,
                                                                    _dt);

        control_state.target_throttle = limit(control_state.target_throttle, -1, 1);

        /**** Rudder controller ****/
        control_state.target_rudder = 0;// atol_state.target_rudder = target_yaw_to_rudder(control_state.target_yaw, _dt);
    }

}


/**
 * @brief pre glidepath controller using regular crosstrack to determine heading, reduces throttle to reach target glidepath speed
 * @param _dt Iteration time
*/ 
static void landing_pre_glidepath(float _dt){
	/**** Roll controller ****/
	// Calculate target course with crosstrack enabled
    if(atol_state.runway_settings.traffic_pattern == 0){
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(atol_state.glidepath_wp_lat_ext,
                                                                                     atol_state.glidepath_wp_lon_ext,
                                                                                     atol_state.runway_settings.start_lat,
                                                                                     atol_state.runway_settings.start_lon,
                                                                                     aircraft_state.latitude,
                                                                                     aircraft_state.longitude,
                                                                                     atol_state.landing_settings.crosstrack_kp,
                                                                                     atol_state.landing_settings.crosstrack_ki,
                                                                                     aircraft_state.altitude < 50.0f ? atol_state.landing_settings.crosstrack_imax : 0,
                                                                                     _dt);

        control_state.target_roll = target_heading_to_roll(atol_state.target_course, _dt);
        control_state.target_aileron = target_roll_to_aileron(control_state.target_roll, _dt);

        float distance = 0;
        distance = calc_distance(aircraft_state.latitude,
                                 aircraft_state.longitude,
                                 atol_state.landing_wp_lat,
                                 atol_state.landing_wp_lon);


        printf("distance = %03.1f\n", distance);


        //printf("")

        /**** Pitch controller ****/
        // Zero 0 pitch
        control_state.target_elevator = target_pitch_to_elevator(0, _dt);

        /**** Throttle controller ****/
        control_state.target_airspeed = atol_state.landing_settings.glidepath_airspeed;
        control_state.target_throttle = target_airspeed_to_throttle(control_state.target_airspeed,
                                                                    aircraft_state.airspeed,
                                                                    0,
                                                                    0,
                                                                    _dt);

        control_state.target_throttle = limit(control_state.target_throttle, -1, 1);

        /**** Rudder controller ****/
        control_state.target_rudder = atol_state.target_rudder = 0;// target_yaw_to_rudder(control_state.target_yaw, _dt);

    } else{
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(atol_state.glidepath_wp_lat_ext,
                                                                                     atol_state.glidepath_wp_lon_ext,
                                                                                     atol_state.runway_settings.stop_lat,
                                                                                     atol_state.runway_settings.stop_lon,
                                                                                     aircraft_state.latitude,
                                                                                     aircraft_state.longitude,
                                                                                     atol_state.landing_settings.crosstrack_kp,
                                                                                     atol_state.landing_settings.crosstrack_ki,
                                                                                     aircraft_state.altitude < 50.0f ? atol_state.landing_settings.crosstrack_imax : 0,
                                                                                     _dt);

        control_state.target_roll = target_heading_to_roll(atol_state.target_course, _dt);
        control_state.target_aileron = target_roll_to_aileron(control_state.target_roll, _dt);

        float distance = 0;
        distance = calc_distance(aircraft_state.latitude,
                                 aircraft_state.longitude,
                                 atol_state.landing_wp_lat,
                                 atol_state.landing_wp_lon);


        printf("distance = %03.1f\n", distance);


        //printf("")

        /**** Pitch controller ****/
        // Zero 0 pitch
        control_state.target_elevator = target_pitch_to_elevator(0, _dt);

        /**** Throttle controller ****/
        control_state.target_airspeed = atol_state.landing_settings.glidepath_airspeed;
        control_state.target_throttle = target_airspeed_to_throttle(control_state.target_airspeed,
                                                                    aircraft_state.airspeed,
                                                                    0,
                                                                    0,
                                                                    _dt);

        control_state.target_throttle = limit(control_state.target_throttle, -1, 1);

        /**** Rudder controller ****/
        control_state.target_rudder = atol_state.target_rudder = 0;// target_yaw_to_rudder(control_state.target_yaw, _dt);

    }





}

/**
 * @brief Glidepath controller using regular crosstrack calculation to determine heading, throttle controls descent rate and elevator controls airspeed.
 *
 * @param _dt Iteration time
 */
static void landing_glidepath(float _dt){

	
	/**** Roll controller ****/
	// Calculate target course with crosstrack enabled
    if(atol_state.runway_settings.traffic_pattern == 0){
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(atol_state.glidepath_wp_lat_ext,
                                                                                     atol_state.glidepath_wp_lon_ext,
                                                                                     atol_state.runway_settings.start_lat,
                                                                                     atol_state.runway_settings.start_lon,
                                                                                     aircraft_state.latitude,
                                                                                     aircraft_state.longitude,
                                                                                     atol_state.landing_settings.crosstrack_kp,
                                                                                     atol_state.landing_settings.crosstrack_ki,
                                                                                     aircraft_state.altitude < 50.0f ? atol_state.landing_settings.crosstrack_imax : 0,
                                                                                     _dt);
    }else{
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(atol_state.glidepath_wp_lat_ext,
                                                                                     atol_state.glidepath_wp_lon_ext,
                                                                                     atol_state.runway_settings.stop_lat,
                                                                                     atol_state.runway_settings.stop_lon,
                                                                                     aircraft_state.latitude,
                                                                                     aircraft_state.longitude,
                                                                                     atol_state.landing_settings.crosstrack_kp,
                                                                                     atol_state.landing_settings.crosstrack_ki,
                                                                                     aircraft_state.altitude < 50.0f ? atol_state.landing_settings.crosstrack_imax : 0,
                                                                                     _dt);

    }


	control_state.target_roll = target_heading_to_roll(atol_state.target_course, _dt);
	control_state.target_aileron = target_roll_to_aileron(control_state.target_roll, _dt);

	/**** Pitch controller ****/
	float airspeed_error = atol_state.landing_settings.glidepath_airspeed - aircraft_state.airspeed;

	control_state.target_pitch -= airspeed_error * atol_state.landing_settings.airspeed_to_pitch_kp; 
    atol_state.pitch_error = control_state.target_pitch - aircraft_state.pitch;

    float pitch_min = control_state.climb_rate_to_pitch.current[PID_PARAM_min];
    float pitch_max = control_state.climb_rate_to_pitch.current[PID_PARAM_max];
    control_state.target_pitch = limit(control_state.target_pitch, pitch_min, pitch_max); 

	control_state.target_elevator = target_pitch_to_elevator(control_state.target_pitch, _dt);

	/**** Throttle controller ****/
	// Calculate updated glidepath angle
	float distance = 0;
    distance = calc_distance(aircraft_state.latitude, 
								   aircraft_state.longitude, 
								   atol_state.landing_wp_lat, 
								   atol_state.landing_wp_lon);

    printf("distance = %03.1f\n", distance);

    float glidepath_angle = atanf((aircraft_state.altitude-atol_state.landing_settings.flare_altitude) / distance);

    // Calculate predicted altitude
    float pred_alt = tanf(DEG_TO_RAD(atol_state.landing_settings.glidepath_angle)) * distance + atol_state.landing_settings.flare_altitude;
    atol_state.altitude_error = pred_alt - aircraft_state.altitude;
    printf("glidepath angle = %03.1f, predicted alt = %03.1f, altitude error = %03.1f\n", RAD_TO_DEG(glidepath_angle), pred_alt, atol_state.altitude_error);
	// Calculate descent rate from updated glidpath angle and current airspeed
	atol_state.target_descent_rate = tanf(glidepath_angle) * KNOTS_TO_FEET_PER_SEC(aircraft_state.airspeed);

	float descent_rate = -aircraft_state.v_u;
    atol_state.target_airspeed += atol_state.landing_settings.descent_rate_to_throttle_kp * (descent_rate - atol_state.target_descent_rate);
    atol_state.target_airspeed = limit(atol_state.target_airspeed, 0, 150);
	// Proportional control from descent rate to target airspeed, NOTE current and target are switched to inverse sign of output
    control_state.target_throttle = atol_state.target_throttle = target_airspeed_to_throttle(atol_state.target_airspeed, aircraft_state.airspeed, 0, 0, _dt);
            
    //-(descent_rate_error * atol_state.landing_settings.descent_rate_to_throttle_kp);
    control_state.target_throttle = atol_state.target_throttle = limit(atol_state.target_throttle, -1, 1);
	
	/**** Rudder controller ****/
	//control_state.target_rudder = atol_state.target_rudder = target_yaw_to_rudder(control_state.target_yaw, _dt);
    control_state.target_rudder = atol_state.target_rudder = 0.0f;
}


/**
 * @brief Flare attitude controller, pitch is set to landing settings flare pitch
 */
static void landing_flare_attitude(float _dt){

    update_pid_from_params(&descent_rate_error_to_airspeed_rate, atol_state.descent_rate_to_airspeed_rate.current);

	
	/**** Roll controller ****/
	//control_state.target_aileron = target_roll_to_aileron(0, _dt);
    if(atol_state.runway_settings.traffic_pattern == 0){
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(atol_state.glidepath_wp_lat_ext,
                                                                                     atol_state.glidepath_wp_lon_ext,
                                                                                     atol_state.runway_settings.start_lat,
                                                                                     atol_state.runway_settings.start_lon,
                                                                                     aircraft_state.latitude,
                                                                                     aircraft_state.longitude,
                                                                                     atol_state.landing_settings.crosstrack_kp,
                                                                                     atol_state.landing_settings.crosstrack_ki,
                                                                                     atol_state.landing_settings.crosstrack_imax,
                                                                                     _dt);
    } else{
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(atol_state.glidepath_wp_lat_ext,
                                                                                     atol_state.glidepath_wp_lon_ext,
                                                                                     atol_state.runway_settings.stop_lat,
                                                                                     atol_state.runway_settings.stop_lon,
                                                                                     aircraft_state.latitude,
                                                                                     aircraft_state.longitude,
                                                                                     atol_state.landing_settings.crosstrack_kp,
                                                                                     atol_state.landing_settings.crosstrack_ki,
                                                                                     atol_state.landing_settings.crosstrack_imax,
                                                                                     _dt);
    }


	control_state.target_roll = target_heading_to_roll(atol_state.target_course, _dt);
	control_state.target_aileron = target_roll_to_aileron(control_state.target_roll, _dt);

	/**** Elevator controller ****/
	control_state.target_pitch += atol_state.landing_settings.pitch_rate * _dt;
    atol_state.pitch_error = control_state.target_pitch - aircraft_state.pitch; // For communication purposes
    float pitch_min = control_state.climb_rate_to_pitch.current[PID_PARAM_min];
	control_state.target_pitch = limit(control_state.target_pitch, pitch_min, atol_state.landing_settings.flare_pitch);
	control_state.target_elevator = target_pitch_to_elevator(control_state.target_pitch, _dt);
    
	/**** Throttle controller ****/
	//float descent_rate = -aircraft_state.v_u;
    //atol_state.target_airspeed += atol_state.landing_settings.descent_rate_to_throttle_kp * (descent_rate - 0);
    control_state.target_throttle = atol_state.target_throttle; //target_airspeed_to_throttle(atol_state.target_airspeed, aircraft_state.airspeed, 0, 0, _dt);
    control_state.target_throttle = limit(atol_state.target_throttle, -1, 1);

    /**** Rudder controller ****/
	control_state.target_rudder = atol_state.target_rudder = 0.0f;
	
}


/**
 * @brief Flare descent controller descent rate is controlled by airspeed
 */
static void landing_flare_descent(float _target_pitch, float _target_descent_rate, float _dt){
    update_pid_from_params(&descent_rate_error_to_airspeed_rate, atol_state.descent_rate_to_airspeed_rate.current);

	
	/**** Roll controller ****/
	//control_state.target_aileron = target_roll_to_aileron(0, _dt);
    if(atol_state.runway_settings.traffic_pattern == 0){
        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(
                atol_state.runway_settings.stop_lat,
                atol_state.runway_settings.stop_lon,
                atol_state.runway_settings.start_lat,
                atol_state.runway_settings.start_lon,
                aircraft_state.latitude,
                aircraft_state.longitude,
                atol_state.landing_settings.crosstrack_kp,
                atol_state.landing_settings.crosstrack_ki,
                atol_state.landing_settings.crosstrack_imax,
                _dt);

    } else {

        nav_state.target_course = atol_state.target_course = calc_crosstrack_heading(
                atol_state.runway_settings.start_lat,
                atol_state.runway_settings.start_lon,
                atol_state.runway_settings.stop_lat,
                atol_state.runway_settings.stop_lon,
                aircraft_state.latitude,
                aircraft_state.longitude,
                atol_state.landing_settings.crosstrack_kp,
                atol_state.landing_settings.crosstrack_ki,
                atol_state.landing_settings.crosstrack_imax,
                _dt);
    }

	control_state.target_roll = target_heading_to_roll(atol_state.target_course, _dt);
	control_state.target_aileron = target_roll_to_aileron(control_state.target_roll, _dt);

	/**** Elevator controller ****/
	control_state.target_elevator = target_pitch_to_elevator(_target_pitch, _dt);
	
	/**** Throttle controller ****/

	// Proportional control from descent rate to airspeed
	float descent_rate = -aircraft_state.v_u;
    atol_state.target_airspeed += atol_state.landing_settings.descent_rate_to_throttle_kp * (descent_rate - _target_descent_rate);
    atol_state.target_airspeed = limit(atol_state.target_airspeed, 25 /* should be a "minimum airspeed" parameter" */, 150);
    control_state.target_throttle = atol_state.target_throttle = target_airspeed_to_throttle(atol_state.target_airspeed, aircraft_state.airspeed, 0, 0, _dt);
    control_state.target_throttle = limit(atol_state.target_throttle, -1, 1);

    printf("flare_descent : descent_rate = %02.1f, target_airspeed = %03.2f, target_throttle = %03.2f", descent_rate, atol_state.target_airspeed, control_state.target_throttle);

	/**** Rudder controller ****/
	// Use rudder to align onto the runway
    atol_state.target_course = atol_state.runway_bearing;

	float course_error = normalize_degree(atol_state.target_course - aircraft_state.heading);
	//control_state.target_rudder = atol_state.target_rudder = rudder_control(atol_state.target_rudder, course_error, _dt);

}

/**
 * @brief Runway controller
 */
static void landing_runway(float _dt){

	/**** Roll controller ****/
	control_state.target_aileron = target_roll_to_aileron(0, _dt);

	/**** Elevator controller ****/
	// Pitch level
	control_state.target_elevator = target_pitch_to_elevator(0, _dt);

	/**** Throttle controller ****/
	control_state.target_throttle = -1;

	/**** Rudder controller ****/
	// Use rudder to align onto the runway
    atol_state.target_course = atol_state.runway_bearing;

	float course_error = normalize_degree(atol_state.target_course - aircraft_state.heading);
	//control_state.target_rudder = atol_state.target_rudder = rudder_control(atol_state.target_rudder, course_error, _dt);
    control_state.target_rudder = atol_state.target_rudder = target_yaw_to_rudder(atol_state.target_course, _dt);


}

