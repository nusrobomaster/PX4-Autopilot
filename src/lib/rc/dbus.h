#pragma once

#include <stdint.h>
#include <stdbool.h>


__BEGIN_DECLS

__EXPORT bool	dbus_parse(uint64_t now, uint8_t *frame, unsigned len, uint16_t *values,
			   uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, unsigned *frame_drops, uint16_t max_channels);


__END_DECLS
