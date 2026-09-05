/* Unit tests for the pure decision logic in fsm.c.
 * Deliberately dependency-free (plain assert) so `make test` needs
 * nothing beyond a C compiler. Covers the priority ordering documented
 * in next_state()'s comment block and the debounce counter behavior.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "failsafe.h"

static void test_sensor_bad_overrides_everything(void)
{
    /* sensor_bad must win even from AUTO_LAND / EMERGENCY_FAILSAFE */
    assert(next_state(NORMAL, 0, 0, 0, 0, 0, 1, 0, 0) == EMERGENCY_FAILSAFE);
    assert(next_state(AUTO_LAND, 0, 0, 0, 0, 0, 1, 0, 0) == EMERGENCY_FAILSAFE);
    assert(next_state(EMERGENCY_FAILSAFE, 0, 0, 0, 0, 0, 1, 0, 0) == EMERGENCY_FAILSAFE);
}

static void test_emergency_is_latched(void)
{
    /* once in EMERGENCY_FAILSAFE, nothing but sensor_bad matters, and
     * with sensor_bad=0 it should still stay latched */
    assert(next_state(EMERGENCY_FAILSAFE, 1, 1, 1, 1, 1, 0, 1, 0) == EMERGENCY_FAILSAFE);
}

static void test_auto_land_is_latched(void)
{
    /* AUTO_LAND never reverses except via sensor_bad */
    assert(next_state(AUTO_LAND, 1, 0, 1, 1, 1, 0, 1, 0) == AUTO_LAND);
}

static void test_batt_crit_overrides_rth_and_faults(void)
{
    assert(next_state(NORMAL, 0, 1, 1, 0, 1, 0, 1, 0) == AUTO_LAND);
    assert(next_state(RETURN_TO_HOME, 0, 1, 0, 0, 0, 0, 0, 0) == AUTO_LAND);
}

static void test_rth_stays_until_home_reached(void)
{
    assert(next_state(RETURN_TO_HOME, 0, 0, 0, 0, 0, 0, 0, 0) == RETURN_TO_HOME);
    assert(next_state(RETURN_TO_HOME, 0, 0, 0, 0, 0, 0, 0, 1) == AUTO_LAND);
}

static void test_rc_gps_fence_trigger_rth(void)
{
    assert(next_state(NORMAL, 0, 0, 1, 0, 0, 0, 0, 0) == RETURN_TO_HOME);
    assert(next_state(NORMAL, 0, 0, 0, 0, 1, 0, 0, 0) == RETURN_TO_HOME);
    assert(next_state(NORMAL, 0, 0, 0, 0, 0, 0, 1, 0) == RETURN_TO_HOME);
}

static void test_alt_high_triggers_warning(void)
{
    assert(next_state(NORMAL, 0, 0, 0, 1, 0, 0, 0, 0) == WARNING);
}

static void test_batt_warn_triggers_warning_else_normal(void)
{
    assert(next_state(NORMAL, 1, 0, 0, 0, 0, 0, 0, 0) == WARNING);
    assert(next_state(NORMAL, 0, 0, 0, 0, 0, 0, 0, 0) == NORMAL);
}

static void test_debounce_requires_n_consecutive_cycles(void)
{
    int count = 0;
    assert(debounce(1, &count) == 0); /* 1 */
    assert(debounce(1, &count) == 0); /* 2 */
    assert(debounce(1, &count) == 1); /* 3 -> DEBOUNCE_N, confirmed */
    assert(debounce(1, &count) == 1); /* stays confirmed, capped */
}

static void test_debounce_resets_on_condition_clear(void)
{
    int count = 0;
    debounce(1, &count);
    debounce(1, &count);
    assert(count == 2);
    assert(debounce(0, &count) == 0);
    assert(count == 0);
}

static void test_fence_zone_thresholds(void)
{
    assert(strcmp(fence_zone(0.0f), "SAFE") == 0);
    assert(strcmp(fence_zone(69.9f), "SAFE") == 0);
    assert(strcmp(fence_zone(70.0f), "CAUTION") == 0);
    assert(strcmp(fence_zone(99.9f), "CAUTION") == 0);
    assert(strcmp(fence_zone(100.0f), "BREACH") == 0);
}

/* Full-loop regression tests, enabled by run_simulation() being callable
 * without stdin. Fixed seeds make these deterministic. */

static void test_critical_battery_start_forces_auto_land(void)
{
    SimConfig cfg = {
        .batt = 9.0f, .alt = 50.0f, .alt_limit = 120.0f,
        .fence_r = 100.0f, .start_dist = 10.0f, .cycles = 40,
        .seed = 1, .have_seed = 1, .quiet = 1
    };
    State final_state;
    int rc = run_simulation(&cfg, &final_state);
    /* battery starts below BATT_CRIT_PCT, so within DEBOUNCE_N cycles
     * the FSM must be in AUTO_LAND and stay there until it lands. */
    assert(final_state == AUTO_LAND);
    assert(rc == SIM_EXIT_LANDED || rc == SIM_EXIT_INCOMPLETE);
}

static void test_same_seed_is_reproducible(void)
{
    SimConfig cfg = {
        .batt = 60.0f, .alt = 50.0f, .alt_limit = 120.0f,
        .fence_r = 100.0f, .start_dist = 10.0f, .cycles = 25,
        .seed = 7, .have_seed = 1, .quiet = 1
    };
    State s1, s2;
    int rc1 = run_simulation(&cfg, &s1);
    int rc2 = run_simulation(&cfg, &s2);
    assert(rc1 == rc2);
    assert(s1 == s2);
}

static void test_zero_cycles_ends_incomplete_at_normal(void)
{
    SimConfig cfg = {
        .batt = 90.0f, .alt = 50.0f, .alt_limit = 120.0f,
        .fence_r = 100.0f, .start_dist = 10.0f, .cycles = 0,
        .seed = 3, .have_seed = 1, .quiet = 1
    };
    State final_state;
    int rc = run_simulation(&cfg, &final_state);
    assert(rc == SIM_EXIT_INCOMPLETE);
    assert(final_state == NORMAL);
}

int main(void)
{
    test_sensor_bad_overrides_everything();
    test_emergency_is_latched();
    test_auto_land_is_latched();
    test_batt_crit_overrides_rth_and_faults();
    test_rth_stays_until_home_reached();
    test_rc_gps_fence_trigger_rth();
    test_alt_high_triggers_warning();
    test_batt_warn_triggers_warning_else_normal();
    test_debounce_requires_n_consecutive_cycles();
    test_debounce_resets_on_condition_clear();
    test_fence_zone_thresholds();
    test_critical_battery_start_forces_auto_land();
    test_same_seed_is_reproducible();
    test_zero_cycles_ends_incomplete_at_normal();

    printf("All tests passed.\n");
    return 0;
}
