/* Aegis - UAV Failsafe State Machine (DRC-SFW-05)
 * CLI entry point. Two modes:
 *   - interactive (no args): prompts for each parameter, as before.
 *   - scripted: all five run parameters + cycle count given as flags,
 *     optionally with --seed for a reproducible run. This is what
 *     makes the simulator usable from scripts/CI instead of only by
 *     hand-typing numbers at a terminal.
 *
 * Usage:
 *   ./aegis_failsafe
 *   ./aegis_failsafe --batt 15 --alt 50 --alt-limit 120 --fence 100 \
 *                     --start-dist 90 --cycles 30 --seed 42
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "failsafe.h"

static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("  --batt N         starting battery %%\n");
    printf("  --alt N          cruise altitude (m)\n");
    printf("  --alt-limit N    altitude ceiling (m)\n");
    printf("  --fence N        geofence radius (m)\n");
    printf("  --start-dist N   starting distance from home (m)\n");
    printf("  --cycles N       number of monitoring cycles to run\n");
    printf("  --seed N         RNG seed, for a reproducible run\n");
    printf("  --quiet          suppress per-cycle telemetry\n");
    printf("  -h, --help       show this message\n");
    printf("With no options, prompts interactively for each value (original behavior).\n");
}

static int prompt_float(const char *label, float *out)
{
    printf("%s: ", label);
    return scanf("%f", out) == 1;
}

int main(int argc, char **argv)
{
    SimConfig cfg;
    int have_batt = 0, have_alt = 0, have_alt_limit = 0;
    int have_fence = 0, have_start_dist = 0, have_cycles = 0;

    cfg.have_seed = 0;
    cfg.quiet = 0;

    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-h") == 0) || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--quiet") == 0) {
            cfg.quiet = 1;
        } else if (i + 1 < argc && strcmp(argv[i], "--batt") == 0) {
            cfg.batt = strtof(argv[++i], NULL); have_batt = 1;
        } else if (i + 1 < argc && strcmp(argv[i], "--alt") == 0) {
            cfg.alt = strtof(argv[++i], NULL); have_alt = 1;
        } else if (i + 1 < argc && strcmp(argv[i], "--alt-limit") == 0) {
            cfg.alt_limit = strtof(argv[++i], NULL); have_alt_limit = 1;
        } else if (i + 1 < argc && strcmp(argv[i], "--fence") == 0) {
            cfg.fence_r = strtof(argv[++i], NULL); have_fence = 1;
        } else if (i + 1 < argc && strcmp(argv[i], "--start-dist") == 0) {
            cfg.start_dist = strtof(argv[++i], NULL); have_start_dist = 1;
        } else if (i + 1 < argc && strcmp(argv[i], "--cycles") == 0) {
            cfg.cycles = atoi(argv[++i]); have_cycles = 1;
        } else if (i + 1 < argc && strcmp(argv[i], "--seed") == 0) {
            cfg.seed = (unsigned int)strtoul(argv[++i], NULL, 10);
            cfg.have_seed = 1;
        } else {
            fprintf(stderr, "Unknown or incomplete option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    int any_flag_given = have_batt || have_alt || have_alt_limit ||
                          have_fence || have_start_dist || have_cycles;
    int all_flags_given = have_batt && have_alt && have_alt_limit &&
                           have_fence && have_start_dist && have_cycles;

    if (any_flag_given && !all_flags_given) {
        fprintf(stderr, "Scripted mode needs all of --batt --alt --alt-limit "
                         "--fence --start-dist --cycles together.\n");
        return 1;
    }

    if (!all_flags_given) {
        /* Interactive mode: same prompts the original program used. */
        printf("========================================\n");
        printf(" UAV FAILSAFE STATE MACHINE SIMULATOR\n");
        printf("========================================\n\n");

        if (!prompt_float("Starting battery (%)", &cfg.batt)) return 1;
        if (!prompt_float("Cruise altitude (m)", &cfg.alt)) return 1;
        if (!prompt_float("Altitude limit (m)", &cfg.alt_limit)) return 1;
        if (!prompt_float("Geofence radius (m)", &cfg.fence_r)) return 1;
        if (!prompt_float("Starting point (m)", &cfg.start_dist)) return 1;

        printf("Number of monitoring cycles to run: ");
        if (scanf("%d", &cfg.cycles) != 1) return 1;
    }

    State final_state;
    int exit_code = run_simulation(&cfg, &final_state);
    return exit_code;
}
