# Dropbox message data

uint64 timestamp		# time since system start (microseconds)

uint8 dropbox_state			# State of Dropbox (see enum below)
uint8 winch_state			# State of Winch (see enum below)
uint8 cutter_state			# State of Cutter (see enum below)
float32 winch_mass 			# current winch mass

# Dropbox states
uint8 DP_CLOSED_BOX = 0
uint8 DP_OPENED_BOX = 1
uint8 DP_CLOSING_BOX = 2
uint8 DP_OPENING_BOX = 3

# Winch states
uint8 WINCH_TOP = 0
uint8 WINCH_BOTTOM = 1		# TODO: Implement payload mass detection logic
uint8 WINCH_WINDING_UP = 2
uint8 WINCH_WINDING_DOWN = 3
uint8 WINCH_MID_AIR = 4

# Cutter states
uint8 CUTTER_IDLE = 0
uint8 CUTTER_CUTLINE = 1
