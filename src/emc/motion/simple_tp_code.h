#include "simple_tp.h"
#include "rtapi_math.h"

/*
* This file provides the limit3_planner() designed
* by John Morris (zultron) as a refactor of the original
* limit3.comp by John Kasunich.
*
* The code is used by simple_tp.c for single joint or axis
* motion for homing and jogging and by limit3.comp for
* hal usage
*/

// Note: the legacy limit3 test for L3_in_limit may be overly strict
#define SET_NEXT_STATE(p, out_pos, out_vel, in_pos)             \
    do {                                                        \
        *p->L3_curr_pos    = out_pos;                           \
        *p->L3_out_vel_old = out_vel;                           \
        *p->L3_curr_vel    = out_vel;                           \
        *p->L3_in_pos_old  = in_pos;                            \
        *p->L3_active = fabs(*p->L3_curr_pos - *p->L3_pos_cmd)  \
                        >= TINY_DP(*p->L3_max_acc,period);      \
        *p->L3_in_limit = *p->L3_curr_pos != *p->L3_pos_cmd;    \
        return;                                                 \
    } while (0)

#define VALID_NEXT(pos, vel)   ((pos) <= max_pos && (pos) >= min_pos \
                            &&  (vel) <= max_vel && (vel) >= min_vel)

struct limit3_parms {
    int    *L3_active;
    int    *L3_disallow_backoff;
    int    *L3_in_limit;
    double *L3_pos_cmd;
    double *L3_max_vel;
    double *L3_max_acc;
    double *L3_curr_pos;
    double *L3_curr_vel;
    double *L3_in_pos_old;
    double *L3_out_vel_old;
    double *L3_min_pos;
    double *L3_max_pos;
};

void limit3_planner(struct limit3_parms *p,double period)
{
    double in_vel, min_vel, max_vel, min_pos, max_pos;
    double vel_0_time, vel_0_pos;
    double vel_match_time, vel_match_in_pos, vel_match_out_pos;
    int    out_dir, out_dir_rel;

    // Input velocity
    in_vel = (*p->L3_pos_cmd - *p->L3_in_pos_old) / period;
    // Most negative/positive velocity reachable in one period
    min_vel = fmax(*p->L3_out_vel_old - *p->L3_max_acc * period, - *p->L3_max_vel);
    max_vel = fmin(*p->L3_out_vel_old + *p->L3_max_acc * period,   *p->L3_max_vel);
    // Most negative/positive position reachable in one period
    min_pos = *p->L3_curr_pos + min_vel * period;
    max_pos = *p->L3_curr_pos + max_vel * period;

    // Direction (sign) of output movement
    out_dir = (*p->L3_out_vel_old < 0) ? -1 : 1;
    // Direction of output movement relative to input movement
    out_dir_rel = (*p->L3_out_vel_old - in_vel < 0) ? -1 : 1;

    // Respect max/min position limits:  stop at limit line
    vel_0_time = fabs(*p->L3_out_vel_old / *p->L3_max_acc); // min time to decel to stop
    vel_0_pos = *p->L3_curr_pos                               // position after stop
        + *p->L3_out_vel_old * (vel_0_time + period)
        + 0.5 * (-out_dir * *p->L3_max_acc) * pow(vel_0_time,2);

    // Follow input signal:  match position and velocity
    // - min time for velocity match
    vel_match_time = fabs(*p->L3_out_vel_old-in_vel) / *p->L3_max_acc;
    // - input position after velocity match
    vel_match_in_pos = *p->L3_pos_cmd + in_vel * vel_match_time;
    // - output position after velocity match
    vel_match_out_pos = *p->L3_curr_pos
        + *p->L3_out_vel_old * (vel_match_time + period)
        + 0.5 * (-out_dir_rel * *p->L3_max_acc) * pow(vel_match_time,2);

    // Respect max/min position limits
    //
    // - If not at the limit line but in danger of overshooting it,
    //   slow down
    if (vel_0_pos >= *p->L3_max_pos && !VALID_NEXT(*p->L3_max_pos,0)) // can't follow max limit
        SET_NEXT_STATE(p, min_pos, min_vel, *p->L3_pos_cmd);
    if (vel_0_pos <= *p->L3_min_pos && !VALID_NEXT(*p->L3_min_pos,0)) // can't follow min limit
        SET_NEXT_STATE(p, max_pos, max_vel, *p->L3_pos_cmd);
    // - If input signal is headed out of bounds, or headed in bounds
    //   but no danger of overshooting, the limit is the goal
    if ((vel_match_in_pos < *p->L3_min_pos) // Input below min limit
        || (*p->L3_pos_cmd <= *p->L3_min_pos && vel_match_in_pos < vel_match_out_pos)) {
        if (VALID_NEXT(*p->L3_min_pos,0))
            SET_NEXT_STATE(p, *p->L3_min_pos, 0, *p->L3_pos_cmd);   // - Park at min limit
        else
            SET_NEXT_STATE(p, min_pos, min_vel, *p->L3_pos_cmd);    // - Head toward min limit
    }
    if ((vel_match_in_pos > *p->L3_max_pos)                         // Input above max limit
        || (*p->L3_pos_cmd >= *p->L3_max_pos && vel_match_in_pos > vel_match_out_pos)) {
        if (VALID_NEXT(*p->L3_max_pos,0))
            SET_NEXT_STATE(p, *p->L3_max_pos, 0, *p->L3_pos_cmd);   // - Park at max limit
        else
            SET_NEXT_STATE(p, max_pos, max_vel, *p->L3_pos_cmd);    // - Head toward min limit
    }

    // Follow input signal
    //
    // - Try to track input
    if (VALID_NEXT(*p->L3_pos_cmd, in_vel))
        SET_NEXT_STATE(p, *p->L3_pos_cmd, in_vel, *p->L3_pos_cmd);
    // - Try to match position and velocity without overshooting
    if (*p->L3_curr_pos > *p->L3_pos_cmd) {                         // Output > input:
        if (vel_match_in_pos < vel_match_out_pos)                   // - Not overshooting
            SET_NEXT_STATE(p, min_pos, min_vel, *p->L3_pos_cmd);    //   - Move closer
        else                                                        // - Overshooting
            if (*p->L3_disallow_backoff) {
                SET_NEXT_STATE(p, min_pos, min_vel, *p->L3_pos_cmd);// for eoffset_pid
            } else {
                SET_NEXT_STATE(p, max_pos, max_vel, *p->L3_pos_cmd);//   - Back off
            }
    } else {                                                        // Output < input
        if (vel_match_in_pos > vel_match_out_pos)                   // - Not overshooting
            SET_NEXT_STATE(p, max_pos, max_vel, *p->L3_pos_cmd);    //   - Move closer
        else                                                        // - Overshooting
            if (*p->L3_disallow_backoff) {
                SET_NEXT_STATE(p, max_pos, max_vel, *p->L3_pos_cmd);// for eoffset_pid
            } else {
                SET_NEXT_STATE(p, min_pos, min_vel, *p->L3_pos_cmd);//   - Back off
            }
    }
}
