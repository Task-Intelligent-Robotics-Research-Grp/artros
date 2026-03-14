uint8 g_sid  # slave ID of the device

# Byte 0
uint8 g_act  # activation status (0: gripper reset, 1: gripper activation)
uint8 g_mod  # mode of EPick      (0: auto, 1: advanced)
             # mode of 3F-gripper (0: basic, 1: pinch, 2: wide, 3: scissor)
uint8 g_gto  # action status (0: stopped, 1: go to position request)
uint8 g_sta  # gripper status (0: is reset, 1: activating, 3: active)
uint8 g_obj  # - object detection for two-finger grippers
             #     0: moving
             #     1: outer grip
             #     2: inner grip,
             #     3: no object at rest)
             # - motion status for 3F-gripper
	     #     0: moving toward requested position
             #     1: one or two fingers stopped before requested position
             #     2: all fingers stopped before requested position
             #     3: all fingers reached requested position

# Byte 1
uint8 g_vas  # - EPick vacuum actuator status
             #     0: standby
             #     1: gripping
             #     2: passive releasing
             #     3: active releasing
             # - finger A status for 3F-gripper (gDTA)
             #     0: moving
             #     1: stopped due to contact while opening
             #     2: stopped due to contact while closing
             #     3: at the requested position
uint8 g_dtb  # - finger B status for 3F-gripper
uint8 g_dtc  # - finger C status for 3F-gripper
uint8 g_dts  # - scissor status for 3F-gripper

# Byte 2
uint8 g_flt  # fault (0: ok, other: see manual for errors)

# Byte 3
uint8 g_pr   # position request (echo of last commanded position)

# Byte 4
uint8 g_po   # actual position (0-255, 0 = open)

# Byte 5
uint8 g_cou  # actual current (0-255, proportional to actual force)

# Byte 6
uint8 g_prb  # position request for finger B (echo of last commanded position)

# Byte 7
uint8 g_pob  # actual position for finger B (0-255, 0 = open)

# Byte 8
uint8 g_cub  # actual current for finger B (0-255)

# Byte 9
uint8 g_prc  # position request for finger C (echo of last commanded position)

# Byte 10
uint8 g_poc  # actual position for finger C (0-255, 0 = open)

# Byte 11
uint8 g_cuc  # actual current for finger C (0-255)

# Byte 12
uint8 g_prs  # position request for scissor (echo of last commanded position)

# Byte 13
uint8 g_pos  # actual position for scissor (0-255, 0 = open)

# Byte 14
uint8 g_cus  # actual current for scissor (0-255)
