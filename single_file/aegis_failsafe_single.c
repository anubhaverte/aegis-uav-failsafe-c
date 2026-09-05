/* Aegis - UAV Failsafe State Machine (DRC-SFW-05)
 * Single-file build for online compilers/IDEs (Ideone, OnlineGDB, college
 * judges, etc.) that only accept one source file. Functionally identical
 * to the multi-file project in include/ and src/ - that version is the
 * maintained one, with tests and CI. This file is interactive-mode only
 * (safest for judges that just compile-and-run and pipe stdin): it will
 * prompt for 6 numbers, same as the original program.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define BATT_WARN_PCT       20.0f
#define BATT_CRIT_PCT       10.0f
#define RC_TIMEOUT_S          3
#define DEBOUNCE_N            3
#define DRAIN_PER_CYCLE       0.30f
#define HOME_RADIUS_M          2.0f
#define LANDED_ALT_M           0.5f
#define FENCE_CAUTION_PCT     70.0f
#define PI_F                   3.14159265f

typedef enum {
    NORMAL = 0,
    WARNING,
    RETURN_TO_HOME,
    AUTO_LAND,
    EMERGENCY_FAILSAFE
} State;

/* one cycle's worth of readings, handed to the state machine */
typedef struct {
    float batt;
    float alt;
    float x;
    float y;
    float dist;
    float pct;
    int   rc_ok;
    int   gps_ok;
    int   sensor_ok;
    int   fence_ok;
} Reading;

/* ground-truth simulation state, mutated every cycle */
typedef struct {
    float batt;
    float alt;
    float x;
    float y;
    float vx;
    float vy;
    float fence_r;
    float alt_limit;
    float cruise_alt;
    int   rc_drop;
    int   gps_drop;
} Sim;

static const char *state_name(State s)
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

static const char *fence_zone(float pct)
{
    if (pct >= 100.0f) return "BREACH";
    if (pct >= FENCE_CAUTION_PCT) return "CAUTION";
    return "SAFE";
}

static int debounce(int condition_active, int *count)
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
 */
static State next_state(State current,
                         int batt_warn, int batt_crit, int rc_lost,
                         int alt_high, int gps_lost, int sensor_bad,
                         int fence_breach, int home_reached)
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

static void print_alert(State prev, State curr)
{
    if (prev != curr) {
        printf("\n=================================\n");
        printf("STATE CHANGE: %s --> %s\n", state_name(prev), state_name(curr));
        printf("=================================\n");
    }
}

/* random float in [lo, hi) */
static float frand(float lo, float hi)
{
    return lo + ((float)rand() / (float)RAND_MAX) * (hi - lo);
}

/* true about 1 time in n */
static int chance(int n)
{
    return (rand() % n) == 0;
}

/* straight-line distance from home (0,0) to (x, y) */
static float dist2d(float x, float y)
{
    return sqrtf(x * x + y * y);
}

static float drain_batt(float batt)
{
    float rate = DRAIN_PER_CYCLE + frand(-0.05f, 0.10f);
    if (batt < 25.0f)
        rate *= 1.6f;
    if (chance(12))
        rate += frand(0.3f, 0.8f);
    float next = batt - rate;
    if (next < 0.0f) next = 0.0f;
    return next;
}

static float wander_alt(float alt, float cruise_alt)
{
    float next = alt + frand(-4.0f, 4.0f);
    next += (cruise_alt - next) * 0.05f;
    if (chance(15))
        next += frand(10.0f, 40.0f);
    if (next < 0.0f) next = 0.0f;
    return next;
}

static void wander_pos(Sim *s)
{
    s->vx += frand(-1.5f, 1.5f);
    s->vy += frand(-1.5f, 1.5f);
    s->vx *= 0.85f;
    s->vy *= 0.85f;
    if (chance(20)) {
        float angle = frand(0.0f, 2.0f * PI_F);
        float push  = frand(8.0f, 20.0f);
        s->vx += cosf(angle) * push;
        s->vy += sinf(angle) * push;
    }
    s->x += s->vx;
    s->y += s->vy;
}

static void fly_home(Sim *s)
{
    float d = dist2d(s->x, s->y);
    if (d < 0.01f) {
        s->vx = 0.0f;
        s->vy = 0.0f;
        return;
    }
    float speed = frand(5.0f, 12.0f);
    float ux = -s->x / d;
    float uy = -s->y / d;
    s->x += ux * speed;
    s->y += uy * speed;
}

static float descend(float alt)
{
    float next = alt - frand(8.0f, 16.0f);
    if (next < 0.0f) next = 0.0f;
    return next;
}

static Reading read_data(Sim *s, State behaviour)
{
    Reading r;

    if (behaviour == EMERGENCY_FAILSAFE) {
        r.batt      = s->batt;
        r.alt       = s->alt;
        r.x         = s->x;
        r.y         = s->y;
        r.dist      = dist2d(s->x, s->y);
        r.pct       = (r.dist / s->fence_r) * 100.0f;
        r.rc_ok     = (s->rc_drop == 0);
        r.gps_ok    = (s->gps_drop == 0);
        r.sensor_ok = 0;
        r.fence_ok  = (r.dist <= s->fence_r);
        return r;
    }

    s->batt = drain_batt(s->batt);

    if (behaviour == AUTO_LAND) {
        s->alt = descend(s->alt);
    } else if (behaviour == RETURN_TO_HOME) {
        s->alt = wander_alt(s->alt, s->cruise_alt);
        fly_home(s);
    } else {
        s->alt = wander_alt(s->alt, s->cruise_alt);
        wander_pos(s);
    }

    if (s->rc_drop == 0 && chance(25))
        s->rc_drop = 1 + (rand() % 5);
    if (s->rc_drop > 0)
        s->rc_drop--;

    if (s->gps_drop == 0 && chance(40))
        s->gps_drop = 1 + (rand() % 3);
    if (s->gps_drop > 0)
        s->gps_drop--;

    r.batt      = s->batt;
    r.alt       = s->alt;
    r.x         = s->x;
    r.y         = s->y;
    r.dist      = dist2d(s->x, s->y);
    r.pct       = (r.dist / s->fence_r) * 100.0f;
    r.rc_ok     = (s->rc_drop == 0);
    r.gps_ok    = (s->gps_drop == 0);
    r.sensor_ok = chance(60) ? 0 : 1;
    r.fence_ok  = (r.dist <= s->fence_r);

    return r;
}

int main(void)
{
    Sim sim;
    int cycles;
    float start_dist;

    srand((unsigned int)time(NULL));

    printf("========================================\n");
    printf(" UAV FAILSAFE STATE MACHINE SIMULATOR\n");
    printf("========================================\n\n");

    printf("Starting battery (%%): ");
    if (scanf("%f", &sim.batt) != 1) return 1;
    printf("Cruise altitude (m): ");
    if (scanf("%f", &sim.alt) != 1) return 1;
    sim.cruise_alt = sim.alt;
    printf("Altitude limit (m): ");
    if (scanf("%f", &sim.alt_limit) != 1) return 1;
    printf("Geofence radius (m): ");
    if (scanf("%f", &sim.fence_r) != 1) return 1;
    printf("Starting point (m): ");
    if (scanf("%f", &start_dist) != 1) return 1;

    if (start_dist < 0.0f) start_dist = 0.0f;
    if (start_dist >= sim.fence_r) {
        printf("Note: that's outside (or right on) the fence radius - placing the\n");
        printf("      vehicle at 80%% of the radius instead, so the sim starts legal.\n");
        start_dist = sim.fence_r * 0.8f;
    }

    float angle = frand(0.0f, 2.0f * PI_F);
    sim.x = cosf(angle) * start_dist;
    sim.y = sinf(angle) * start_dist;
    sim.vx = 0.0f; sim.vy = 0.0f;
    sim.rc_drop = 0; sim.gps_drop = 0;

    printf("Placed at (%.1f, %.1f) m, %.1f m from home (%.0f%% of a %.1f m fence)\n",
           sim.x, sim.y, start_dist, (start_dist / sim.fence_r) * 100.0f, sim.fence_r);

    printf("Number of monitoring cycles to run: ");
    if (scanf("%d", &cycles) != 1) return 1;

    State state = NORMAL;
    int bw_cnt = 0, bc_cnt = 0, rc_cnt = 0, alt_cnt = 0, gps_cnt = 0, sens_cnt = 0;
    int rc_lost_for = 0;

    for (int cycle = 1; cycle <= cycles; cycle++) {
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
        state = next_state(state, batt_warn, batt_crit, rc_lost, alt_high,
                            gps_lost, sensor_bad, fence_breach, home_reached);

        if (home_reached) {
            sim.x = 0.0f; sim.y = 0.0f;
            r.x = 0.0f; r.y = 0.0f; r.dist = 0.0f; r.pct = 0.0f;
        }
        int just_landed = (state == AUTO_LAND && r.alt <= LANDED_ALT_M);
        if (just_landed) {
            sim.alt = 0.0f;
            r.alt = 0.0f;
        }

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

        if (state == EMERGENCY_FAILSAFE) {
            printf("\nSimulation halted: sensor data can no longer be trusted.\n");
            break;
        }
        if (just_landed) {
            printf("\nVehicle has landed and disarmed.\n");
            break;
        }
    }

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

    return 0;
}
