/**
 * @file   fcs_landing.h
 * @author Tim Bakker (bakkert@vcu.edu)
 * @date   thu dec 17 22:45:38 2014
 * @copyright 2014 virginia commonwealth university.
 * 
 * @brief  landing functionality
 * 
 */
#ifndef _FCS_LANDING_
#define _FCS_LANDING_

#include "api/api.h"


/**
 * @enum LANDING_ABORT
 * @brief Abort modes of landing
 */
typedef enum landing_abort {
	LANDING_ABORT_ar_distance,
	LANDING_ABORT_gp_crosstrack_distance,
	LANDING_ABORT_gp_bank,
	LANDING_ABORT_gp_distance,
	LANDING_ABORT_fl_distance,
	LANDING_ABORT_fl_crosstrack_distance,
	LANDING_ABORT_fl_bank,
}LANDING_ABORT;


extern void landing_init(api_t *api_p);
extern int32_t landing_run(float dt);

#endif
