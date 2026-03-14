uint8 r_sid  # slave ID

# Byte 0
uint8 r_act  # activate (0: deactivate, 1: activate)
uint8 r_mod  # mode of EPick      (0: auto,  1: advanced)
             # mode of 3F-gripper (0: basic, 1: pinch, 2: wide, 3: scissor)
uint8 r_gto  # go-to (0: stop, 1: move with values set in rPR, rSP and rFR)
uint8 r_atr  # auto-release (0: normal, 1: emergency slow move)
uint8 r_ard  # auto-release direction (0: toward closing, 1: toward opening)

# Byte 1
uint8 r_icf  # individual finger control (0: disable, 1: enable)
uint8 r_ics  # individual scissor control (0: disable, 1: enable)

# Byte 3
uint8 r_pr   # position request (0-255, 0 = open), for finger A if 3F-gripper

# Byte 4
uint8 r_sp   # speed request (0-255), for finger A if 3F-gripper

# Byte 5
uint8 r_fr   # force request (0-255), for finger A if 3F-gripper

# Byte 6
uint8 r_prb  # position request for finger B (0-255, 0 = open)

# Byte 7
uint8 r_spb  # speed request for finger B (0-255)

# Byte 8
uint8 r_frb  # force request for finger B (0-255)

# Byte 9
uint8 r_prc  # position request for finger C (0-255, 0 = open)

# Byte 10
uint8 r_spc  # speed request for finger C (0-255)

# Byte 11
uint8 r_frc  # force request for finger C (0-255)

# Byte 12
uint8 r_prs  # position request for scissor (0-255, 0 = open)

# Byte 13
uint8 r_sps  # speed request for scissor (0-255)

# Byte 14
uint8 r_frs  # force request for scissor (0-255)
