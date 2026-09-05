#include <stdlib.h>
#include <math.h>
#include "failsafe.h"

/* random float in [lo, hi) */
float frand(float lo, float hi)
{
    return lo + ((float)rand() / (float)RAND_MAX) * (hi - lo);
}

/* true about 1 time in n */
int chance(int n)
{
    return (rand() % n) == 0;
}

/* straight-line distance from home (0,0) to (x, y) */
float dist2d(float x, float y)
{
    return sqrtf(x * x + y * y);
}

float drain_batt(float batt)
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

float wander_alt(float alt, float cruise_alt)
{
    float next = alt + frand(-4.0f, 4.0f);
    next += (cruise_alt - next) * 0.05f;

    if (chance(15))
        next += frand(10.0f, 40.0f);   /* gust / climb */

    if (next < 0.0f) next = 0.0f;
    return next;
}

void wander_pos(Sim *s)
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

void fly_home(Sim *s)
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

float descend(float alt)
{
    float next = alt - frand(8.0f, 16.0f);
    if (next < 0.0f) next = 0.0f;
    return next;
}

/* Core per-cycle sensor-generation function. Behavior branches on the
 * current FSM state:
 *   EMERGENCY_FAILSAFE - freezes all dynamics, forces every health flag false
 *   AUTO_LAND          - only altitude descends, position frozen
 *   RETURN_TO_HOME     - altitude wanders, position flies toward home
 *   otherwise          - altitude and position both wander freely
 */
Reading read_data(Sim *s, State behaviour)
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
