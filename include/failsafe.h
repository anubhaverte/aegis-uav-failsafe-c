#ifndef FAILSAFE_H
#define FAILSAFE_H

/* Aegis - UAV Failsafe State Machine (originally DRC-SFW-05)
 * Shared types and tunable constants for the simulator.
 */

#define BATT_WARN_PCT       20.0f
#define BATT_CRIT_PCT       10.0f
#define RC_TIMEOUT_S          3
#define DEBOUNCE_N            3
#define DRAIN_PER_CYCLE       0.30f
#define HOME_RADIUS_M          2.0f
#define LANDED_ALT_M           0.5f
#define FENCE_CAUTION_PCT     70.0f
#define PI_F                   3.14159265f  /* was a hardcoded 3.14f; full float
                                                precision costs nothing here and
                                                removes a needless inaccuracy */

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

/* Exit codes for run_simulation() - lets the simulator be scripted/CI'd
 * instead of only judged by eyeballing console output. */
enum {
    SIM_EXIT_LANDED    = 0,  /* reached AUTO_LAND and touched down cleanly */
    SIM_EXIT_EMERGENCY = 1,  /* escalated to EMERGENCY_FAILSAFE */
    SIM_EXIT_INCOMPLETE = 2  /* cycle budget exhausted, still flying */
};

/* All inputs the original program collected via scanf, now a single
 * struct so the run can be driven from argv, from a test, or from
 * an interactive prompt without three copies of the same logic. */
typedef struct {
    float batt;
    float alt;
    float alt_limit;
    float fence_r;
    float start_dist;
    int   cycles;
    unsigned int seed;
    int   have_seed;   /* 0 => seed from time(NULL) */
    int   quiet;       /* 0 => print full per-cycle telemetry (default) */
} SimConfig;

/* Runs the full monitoring loop against cfg and prints telemetry
 * (unless cfg.quiet). Returns one of the SIM_EXIT_* codes above.
 * final_state, if non-NULL, receives the terminal FSM state. */
int run_simulation(const SimConfig *cfg, State *final_state);

const char *state_name(State s);
const char *fence_zone(float pct);

float frand(float lo, float hi);
int   chance(int n);
float dist2d(float x, float y);
int   debounce(int condition_active, int *count);

float drain_batt(float batt);
float wander_alt(float alt, float cruise_alt);
void  wander_pos(Sim *s);
void  fly_home(Sim *s);
float descend(float alt);

Reading read_data(Sim *s, State behaviour);

State next_state(State current,
                  int batt_warn,
                  int batt_crit,
                  int rc_lost,
                  int alt_high,
                  int gps_lost,
                  int sensor_bad,
                  int fence_breach,
                  int home_reached);

void print_alert(State prev, State curr);

#endif /* FAILSAFE_H */
