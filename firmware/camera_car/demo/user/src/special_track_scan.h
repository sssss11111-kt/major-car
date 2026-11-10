/* Internal interface between scan extraction and diagnostic classification. */
#ifndef SPECIAL_TRACK_SCAN_H
#define SPECIAL_TRACK_SCAN_H

#include "special_track.h"

/* vision_process normalizes the selected road polarity to 255. */
void special_track_scan(const uint8_t *binary, const special_track_config_t *cfg,
                        const vision_result_t *normal, unsigned count, special_scan_result_t *out);
uint8_t special_monitor_score(const special_scan_result_t *scan, const vision_result_t *normal);

#endif
