#ifndef IR_CODES_DNT_RC11_H

const unsigned long IR_DNT_RC11_CMD_TIMEOUT_MS = 150;

enum class ir_codes_dnt_rc11 : unsigned long {
          INVALID                = 0x00000000,
          KEY_PLAY               = 0x00FF20DF,
          KEY_STOP               = 0x00FFA05F,
          KEY_POWER              = 0x00FFB24D,
          KEY_VOLUMEUP           = 0x00FF32CD,
          KEY_VOLUMEDOWN         = 0x00FF12ED,
          KEY_MUTE               = 0x00FFF00F,
          KEY_UP                 = 0x00FF708F,
          KEY_DOWN               = 0x00FF58A7,
          KEY_0                  = 0x00FFE817,
          KEY_1                  = 0x00FF08F7,
          KEY_2                  = 0x00FFC03F,
          KEY_3                  = 0x00FF807F,
          KEY_4                  = 0x00FF906F
  };

#endif
