/* Aegis - UAV Failsafe State Machine (DRC-SFW-05)
 * The monitoring loop itself, extracted out of main() so it can be
 * driven programmatically (by tests, or any future caller) instead
 * of only via interactive stdin.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "failsafe.h"

int run_simulation(const SimConfig *cfg, State *final_state)
{
    Sim sim;

    srand(cfg->have_seed ? cfg->seed : (unsigned int)time(NULL));

    sim.batt       = cfg->batt;
    sim.alt        = cfg->alt;
    sim.cruise_alt = cfg->alt;
    sim.alt_limit  = cfg->alt_limit;
    sim.fence_r    = cfg->fence_r;

    float start_dist = cfg->start_dist;
    if (start_dist < 0.0f)
        start_dist = 0.0f;
    if (start_dist >= sim.fence_r) {
        if (!cfg->quiet) {
            printf("Note: that's outside (or right on) the fence radius - placing the\n");
            printf("      vehicle at 80%% of the radius instead, so the sim starts legal.\n");
        }
        start_dist = sim.fence_r * 0.8f;
    }

    float angle = frand(0.0f, 2.0f * PI_F);
    sim.x = cosf(angle) * start_dist;
    sim.y = sinf(angle) * start_dist;
    sim.vx = 0.0f;
    sim.vy = 0.0f;
    sim.rc_drop  = 0;
    sim.gps_drop = 0;

    if (!cfg->quiet) {
        printf("Placed at (%.1f, %.1f) m, %.1f m from home (%.0f%% of a %.1f m fence)\n",
               sim.x, sim.y, start_dist,
               (start_dist / sim.fence_r) * 100.0f, sim.fence_r);
    }

    State state = NORMAL;

    int bw_cnt = 0, bc_cnt = 0, rc_cnt = 0;
    int alt_cnt = 0, gps_cnt = 0, sens_cnt = 0;
    int rc_lost_for = 0;
    int just_landed = 0;

    for (int cycle = 1; cycle <= cfg->cycles; cycle++) {

        Reading r = read_data(&sim, state);

        rc_lost_for = r.rc_ok ? 0 : rc_lost_for + 1;

        int batt_warn    = debounce(r.batt < BATT_WARN_PCT, &bw_cnt);
        int batt_crit    = debounce(r.batt < BATT_CRIT_PCT, &bc_cnt);
        int rc_lost      = debounce(rc_lost_for >= RC_TIMEOUT_S, &rc_cnt);
        int alt_high     = debounce(r.alt > sim.alt_limit, &alt_cnt);
        int gps_lost     = debounce(!r.gps_ok, &gps_cnt);
        int sensor_bad   = debounce(!r.sensor_ok, &sens_cnt);
        int fence_breach = !r.fence_ok;
        int home_reached = (state == RETURN_TO_HOME && r.dist <= HOME_RADIUS_M);

        State prev = state;

        state = next_state(state, batt_warn, batt_crit, rc_lost,
                            alt_high, gps_lost, sensor_bad, fence_breach,
                            home_reached);

        if (home_reached) {
            sim.x = 0.0f;
            sim.y = 0.0f;
            r.x    = 0.0f;
            r.y    = 0.0f;
            r.dist = 0.0f;
            r.pct  = 0.0f;
        }

        just_landed = (state == AUTO_LAND && r.alt <= LANDED_ALT_M);
        if (just_landed) {
            sim.alt = 0.0f;
            r.alt   = 0.0f;
        }

        if (!cfg->quiet) {
            printf("\n----- Cycle %d -----\n", cycle);
            printf("Battery   : %.1f %%\n", r.batt);
            printf("Altitude  : %.1f m (limit %.1f m)\n", r.alt, sim.alt_limit);
            printf("Position  : (%.1f, %.1f) m -> %.1f m from home (%.0f%% of %.1f m fence) [%s]\n",
                   r.x, r.y, r.dist, r.pct, sim.fence_r, fence_zone(r.pct));
            printf("RC Signal : %s\n", r.rc_ok ? "AVAILABLE" : "LOST");
            printf("GPS       : %s\n", r.gps_ok ? "OK" : "FAILED");
            printf("Sensors   : %s\n", r.sensor_ok ? "HEALTHY" : "FAILED");
            printf("Geofence  : %s\n", r.fence_ok ? "INSIDE" : "BREACH");

            print_alert(prev, state);
            printf("Current State: %s\n", state_name(state));

            if (batt_warn)    printf("ALERT: Low Battery Warning\n");
            if (batt_crit)    printf("ALERT: Critical Battery - Auto Land\n");
            if (rc_lost)      printf("ALERT: RC Signal Lost - Returning Home\n");
            if (gps_lost)     printf("ALERT: GPS Failure Detected\n");
            if (alt_high)     printf("ALERT: Altitude Limit Exceeded\n");
            if (sensor_bad)   printf("ALERT: Sensor Failure Detected\n");
            if (fence_breach) printf("ALERT: Geofence Breach Detected\n");
            else if (r.pct >= FENCE_CAUTION_PCT)
                printf("ALERT: Approaching geofence boundary (%.0f%% of radius)\n", r.pct);
            if (home_reached) printf("INFO : Home reached - position is exactly (0.0, 0.0) m, 0.0 m from home\n");
            if (just_landed)  printf("INFO : Touchdown - altitude is exactly 0.0 m\n");
        }

        if (state == EMERGENCY_FAILSAFE) {
            if (!cfg->quiet) printf("\nSimulation halted: sensor data can no longer be trusted.\n");
            break;
        }
        if (just_landed) {
            if (!cfg->quiet) printf("\nVehicle has landed and disarmed.\n");
            break;
        }
    }

    if (!cfg->quiet) {
        printf("\n========================================\n");
        printf("Simulation ended. Final state: %s\n", state_name(state));
        printf("Final altitude: %.1f m\n", sim.alt);
        printf("Final position: (%.1f, %.1f) m, %.1f m from home\n",
               sim.x, sim.y, dist2d(sim.x, sim.y));
        if (state == AUTO_LAND && sim.alt == 0.0f)
            printf("VERIFIED: vehicle landed, altitude is exactly 0 m.\n");
        if (sim.x == 0.0f && sim.y == 0.0f)
            printf("VERIFIED: vehicle is at home, position is exactly (0, 0), 0 m from home.\n");
        printf("========================================\n");
    }

    if (final_state) *final_state = state;

    if (state == EMERGENCY_FAILSAFE) return SIM_EXIT_EMERGENCY;
    if (just_landed)                 return SIM_EXIT_LANDED;
    return SIM_EXIT_INCOMPLETE;
}
