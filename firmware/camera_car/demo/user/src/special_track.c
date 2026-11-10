/* Candidate hysteresis, classification and diagnostic lifecycle. GPL-3.0-or-later. */
#include "special_track_scan.h"
#include <string.h>

void special_track_config_default(special_track_config_t *cfg)
{
    *cfg = (special_track_config_t){
        .scan_top = 40, .scan_bottom = 106, .monitor_rows = 6, .min_run_width = 6,
        .min_channel_width = 16, .min_gap_width = 8, .center_tolerance = 14,
        .max_row_shift = 18, .min_curve_step = 2, .min_contrast = 16,
        .min_cross_rows = 4, .min_cross_width_percent = 75, .min_ring_rows = 5, .min_arc_rows = 2,
        .candidate_on = 30, .candidate_off = 14, .min_confidence = 70,
        .vote_window = 5, .vote_required = 3, .recover_frames = 3,
        .cooldown_ms = 400, .candidate_timeout_ms = 300, .approach_window_ms = 650,
        .loss_timeout_ms = 180, .active_timeout_ms = 8000
    };
}

void special_track_init(special_track_state_t *state)
{
    memset(state, 0, sizeof *state);
}

static int config_valid(const special_track_config_t *c)
{
    return c->scan_bottom < CAR_H && c->scan_bottom >= c->scan_top + SPECIAL_SCAN_ROWS - 1 &&
           c->monitor_rows >= 4 && c->monitor_rows <= SPECIAL_MONITOR_ROWS &&
           c->min_run_width > 0 && c->min_channel_width >= c->min_run_width &&
           c->min_gap_width > 0 && 2 * c->min_channel_width + c->min_gap_width <= CAR_W &&
           c->center_tolerance > 0 && c->center_tolerance < CAR_W / 2 &&
           c->max_row_shift > 0 && c->min_curve_step > 0 && c->min_contrast > 0 &&
           c->min_cross_rows >= 3 && c->min_cross_rows <= SPECIAL_SCAN_ROWS - 2 &&
           c->min_cross_width_percent > 0 && c->min_cross_width_percent <= 100 &&
           c->min_ring_rows >= 4 && c->min_ring_rows <= SPECIAL_SCAN_ROWS - 1 &&
           c->min_arc_rows >= 2 && c->min_arc_rows <= c->min_ring_rows - 2 &&
           c->candidate_off < c->candidate_on && c->candidate_on <= 100 &&
           c->min_confidence > 0 && c->min_confidence <= 100 &&
           c->vote_window >= 2 && c->vote_window <= SPECIAL_VOTE_WINDOW &&
           c->vote_required > c->vote_window / 2 && c->vote_required <= c->vote_window &&
           c->recover_frames >= 2 && c->cooldown_ms && c->candidate_timeout_ms &&
           c->approach_window_ms && c->loss_timeout_ms && c->active_timeout_ms > c->loss_timeout_ms;
}

static int ordinary_road(const special_scan_result_t *s, const vision_result_t *normal)
{
    return normal->valid && normal->scanned_rows > 0 &&
           3u * normal->paired_rows >= 2u * normal->scanned_rows &&
           s->multi_run_rows == 0 && s->clipped_bank_rows < 2 &&
           s->usable_rows == s->row_count && !s->overflow;
}

static void classify(const special_scan_result_t *s, const special_track_config_t *c,
                     const vision_result_t *normal, int approach, special_track_output_t *out)
{
    out->frame_type = SPECIAL_UNKNOWN;
    if (s->overflow) out->diagnostic_flags |= SPECIAL_DIAG_OVERFLOW;
    if ((out->diagnostic_flags & SPECIAL_DIAG_LOW_CONTRAST) || s->overflow) return;
    if (2u * s->usable_rows < s->row_count) {
        out->diagnostic_flags |= SPECIAL_DIAG_WEAK;
        return;
    }
    out->full_scan_valid = 1;
    int cross = s->cross_rows >= c->min_cross_rows;
    int ring = s->ring_rows >= c->min_ring_rows;
    if (s->ambiguous || s->ring_conflict || (cross && ring)) {
        out->diagnostic_flags |= SPECIAL_DIAG_CONFLICT;
        return;
    }
    if (ring && !approach) {
        out->diagnostic_flags |= SPECIAL_DIAG_NO_APPROACH;
        return;
    }
    if (cross || ring) {
        unsigned span = cross ? s->cross_rows : s->ring_rows;
        unsigned confidence = 60 + 4 * span + (ring ? 8 : 0);
        out->confidence = (uint8_t)(confidence > 100 ? 100 : confidence);
        if (out->confidence >= c->min_confidence) {
            out->frame_type = cross ? SPECIAL_CROSS_ISLAND : SPECIAL_ROUNDABOUT;
            out->island_side = ring ? s->island_side : 0;
            return;
        }
    }
    if (ordinary_road(s, normal)) out->frame_type = SPECIAL_NONE;
    else out->diagnostic_flags |= SPECIAL_DIAG_WEAK;
}

static void transition(special_track_state_t *s, special_track_state_id_t next, uint32_t now)
{
    s->state = next;
    s->state_since_ms = now;
    if (next == SPECIAL_IDLE || next == SPECIAL_COOLDOWN) {
        s->vote_pos = s->vote_count = s->recovery = 0;
        s->confirmed_side = 0;
        s->cross = (cross_island_t){0};
        s->round = (roundabout_t){0};
    }
}

static int voted(special_track_state_t *s, const special_track_config_t *c,
                 const special_track_output_t *out)
{
    special_vote_t vote = {out->frame_type, out->island_side};
    if (s->vote_window != c->vote_window) {
        s->vote_window = c->vote_window;
        s->vote_pos = s->vote_count = 0;
    }
    s->votes[s->vote_pos] = vote;
    s->vote_pos = (uint8_t)((s->vote_pos + 1u) % c->vote_window);
    if (s->vote_count < c->vote_window) ++s->vote_count;
    if (vote.type != SPECIAL_CROSS_ISLAND && vote.type != SPECIAL_ROUNDABOUT) return 0;
    unsigned count = 0;
    for (unsigned i = 0; i < s->vote_count && i < c->vote_window; ++i)
        if (s->votes[i].type == vote.type && s->votes[i].island_side == vote.island_side) ++count;
    return count >= c->vote_required;
}

static void active_update(special_track_state_t *s, const special_track_config_t *c,
                          special_track_output_t *out, uint32_t now, int ordinary)
{
    static const special_route_config_t disabled_route = {0};
    const special_fsm_config_t fsm_cfg = {c->recover_frames, c->loss_timeout_ms, c->active_timeout_ms};
    int cross = s->state == SPECIAL_ACTIVE_CROSS;
    special_track_type_t locked = cross ? SPECIAL_CROSS_ISLAND : SPECIAL_ROUNDABOUT;
    int matching = out->frame_type == locked && out->island_side == s->confirmed_side;
    special_route_observation_t observation = {0};
    observation.path_current = (uint8_t)(matching || ordinary);
    out->type = SPECIAL_UNKNOWN;
    out->diagnostic_flags |= SPECIAL_DIAG_ROUTE_DISABLED;
    if (cross) cross_island_update(&s->cross, &disabled_route, &fsm_cfg, &observation, now);
    else roundabout_update(&s->round, &disabled_route, &fsm_cfg, &observation, now);
    if (now - s->last_support_ms >= c->loss_timeout_ms ||
        now - s->state_since_ms >= c->active_timeout_ms) {
        out->diagnostic_flags |= SPECIAL_DIAG_TIMEOUT;
        transition(s, SPECIAL_FAULT, now);
        s->recovery = 0;
        return;
    }
    if (matching || ordinary) s->last_support_ms = now;
    if (matching) out->type = locked;
    else if (out->frame_type == SPECIAL_CROSS_ISLAND || out->frame_type == SPECIAL_ROUNDABOUT)
        out->diagnostic_flags |= SPECIAL_DIAG_CONFLICT;
    if (!ordinary) s->recovery = 0;
    else if (++s->recovery >= c->recover_frames) transition(s, SPECIAL_COOLDOWN, now);
}

void special_track_update_rows(special_track_state_t *s, const special_track_config_t *c,
                               const uint8_t *pixels, const uint8_t *row_thresholds,
                               const vision_result_t *normal, uint32_t now, int force_full,
                               special_track_output_t *out)
{
    *out = (special_track_output_t){0};
    if (!config_valid(c)) {
        special_track_init(s);
        out->state = s->state = SPECIAL_FAULT;
        out->type = out->frame_type = SPECIAL_UNKNOWN;
        out->diagnostic_flags = SPECIAL_DIAG_CONFIG;
        return;
    }

    if (s->frame_seen && now - s->frame_ms >= c->loss_timeout_ms &&
        (s->state == SPECIAL_CANDIDATE || s->state == SPECIAL_UNKNOWN_STATE)) {
        transition(s, SPECIAL_IDLE, now);
        s->candidate = 0;
    }
    s->frame_ms = now;
    s->frame_seen = 1;

    special_track_scan(pixels, row_thresholds, c, normal, c->monitor_rows, &s->scan);
    out->candidate_score = special_monitor_score(&s->scan, normal);
    if (normal->contrast < c->min_contrast) out->diagnostic_flags |= SPECIAL_DIAG_LOW_CONTRAST;
    if (force_full || out->candidate_score >= c->candidate_on) s->candidate = 1;
    else if (out->candidate_score <= c->candidate_off) s->candidate = 0;
    if (s->state == SPECIAL_COOLDOWN && now - s->state_since_ms >= c->cooldown_ms)
        transition(s, SPECIAL_IDLE, now);

    int active = s->state == SPECIAL_ACTIVE_CROSS || s->state == SPECIAL_ACTIVE_ROUNDABOUT;
    if (force_full || active ||
        (s->candidate && s->state != SPECIAL_COOLDOWN && s->state != SPECIAL_FAULT)) {
        out->full_scan_attempted = 1;
        special_track_scan(pixels, row_thresholds, c, normal, SPECIAL_SCAN_ROWS, &s->scan);
        int approach = s->state == SPECIAL_ACTIVE_ROUNDABOUT ||
                       (s->approach_seen && now - s->approach_ms <= c->approach_window_ms);
        classify(&s->scan, c, normal, approach, out);
    }
    int ordinary = !(out->diagnostic_flags & SPECIAL_DIAG_LOW_CONTRAST) &&
                   ordinary_road(&s->scan, normal);
    if (!active && ordinary) {
        s->approach_seen = 1;
        s->approach_ms = now;
    }

    if (active) {
        active_update(s, c, out, now, ordinary);
    } else if (s->state == SPECIAL_FAULT) {
        out->type = SPECIAL_UNKNOWN;
        out->diagnostic_flags |= SPECIAL_DIAG_TIMEOUT;
        if (!ordinary) s->recovery = 0;
        else if (++s->recovery >= c->recover_frames) transition(s, SPECIAL_COOLDOWN, now);
    } else if (s->state != SPECIAL_COOLDOWN) {
        if (!s->candidate) {
            transition(s, SPECIAL_IDLE, now);
        } else {
            if (s->state == SPECIAL_IDLE) transition(s, SPECIAL_CANDIDATE, now);
            out->type = out->frame_type == SPECIAL_NONE ? SPECIAL_NONE : SPECIAL_UNKNOWN;
            if (voted(s, c, out)) {
                out->type = out->frame_type;
                s->confirmed_side = out->island_side;
                s->last_support_ms = now;
                if (out->type == SPECIAL_CROSS_ISLAND) {
                    cross_island_begin(&s->cross, now);
                    transition(s, SPECIAL_ACTIVE_CROSS, now);
                } else {
                    roundabout_begin(&s->round, now);
                    transition(s, SPECIAL_ACTIVE_ROUNDABOUT, now);
                }
                out->diagnostic_flags |= SPECIAL_DIAG_ROUTE_DISABLED;
            } else if (s->state == SPECIAL_CANDIDATE &&
                       now - s->state_since_ms >= c->candidate_timeout_ms) {
                transition(s, SPECIAL_UNKNOWN_STATE, now);
            }
        }
    }
    if (s->state == SPECIAL_COOLDOWN) {
        out->type = SPECIAL_NONE;
        s->candidate = 0;
    }
    if (out->diagnostic_flags & SPECIAL_DIAG_LOW_CONTRAST) {
        out->type = out->frame_type = SPECIAL_UNKNOWN;
    }
    out->candidate = s->candidate;
    out->state = s->state;
    out->cross_state = s->cross.phase;
    out->round_state = s->round.phase;
}

void special_track_update(special_track_state_t *s, const special_track_config_t *c,
                          const uint8_t *binary, const vision_result_t *normal,
                          uint32_t now, special_track_output_t *out)
{
    special_track_update_rows(s, c, binary, NULL, normal, now, 0, out);
}
