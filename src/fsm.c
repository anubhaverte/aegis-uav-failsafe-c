#include <stdio.h>
#include "failsafe.h"

const char *state_name(State s)
{
    switch (s) {
        case NORMAL:              return "NORMAL";
        case WARNING:             return "WARNING";
        case RETURN_TO_HOME:      return "RETURN_TO_HOME";
        case AUTO_LAND:           return "AUTO_LAND";
        case EMERGENCY_FAILSAFE:  return "EMERGENCY_FAILSAFE";
        default:                  return "UNKNOWN";
    }
}

const char *fence_zone(float pct)
{
    if (pct >= 100.0f) return "BREACH";
    if (pct >= FENCE_CAUTION_PCT) return "CAUTION";
    return "SAFE";
}

int debounce(int condition_active, int *count)
{
    if (condition_active) {
        if (*count < DEBOUNCE_N)
            (*count)++;
    } else {
        *count = 0;
    }
    return (*count >= DEBOUNCE_N);
}

/* Pure decision function. Priority order, highest severity first:
 *   1. sensor_bad        -> EMERGENCY_FAILSAFE (overrides everything)
 *   2. already EMERGENCY -> stays (terminal/latched)
 *   3. already AUTO_LAND -> stays (latched; landing is never aborted)
 *   4. batt_crit         -> AUTO_LAND (overrides RTH/fence/RC/GPS)
 *   5. already RTH       -> stays until home_reached, then AUTO_LAND
 *   6. rc_lost/fence_breach/gps_lost -> RETURN_TO_HOME
 *   7. alt_high          -> WARNING
 *   8. otherwise         -> WARNING if batt_warn, else NORMAL
 *
 * Note: fence_breach is NOT debounced (raw !fence_ok each cycle); every
 * other condition is debounced over DEBOUNCE_N cycles by the caller.
 */
State next_state(State current,
                  int batt_warn,
                  int batt_crit,
                  int rc_lost,
                  int alt_high,
                  int gps_lost,
                  int sensor_bad,
                  int fence_breach,
                  int home_reached)
{
    if (sensor_bad)
        return EMERGENCY_FAILSAFE;

    if (current == EMERGENCY_FAILSAFE)
        return EMERGENCY_FAILSAFE;
    if (current == AUTO_LAND)
        return AUTO_LAND;

    if (batt_crit)
        return AUTO_LAND;

    if (current == RETURN_TO_HOME) {
        if (home_reached)
            return AUTO_LAND;
        return RETURN_TO_HOME;
    }

    if (rc_lost || fence_breach || gps_lost)
        return RETURN_TO_HOME;

    if (alt_high)
        return WARNING;

    return batt_warn ? WARNING : NORMAL;
}

void print_alert(State prev, State curr)
{
    if (prev != curr) {
        printf("\n=================================\n");
        printf("STATE CHANGE: %s --> %s\n", state_name(prev), state_name(curr));
        printf("=================================\n");
    }
}
