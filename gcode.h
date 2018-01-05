#include<stdint.h>
#include "motion.h"
#include "config_pins.h"

#define DEBUG
#define X 0
#define Y 1
#define Z 2
#define E 3
#define E0 3
#define E1 4

#define DECFLOAT_EXP_MAX 4
typedef struct {
	uint32_t	mantissa;		///< the actual digits of our floating point number
	uint8_t	exponent	:7;	///< scale mantissa by \f$10^{-exponent}\f$
	uint8_t	sign			:1; ///< positive or negative?
} decfloat;

typedef struct {
  float axis[NUMBUFFER];
  float  F;

  float  e_multiplier;
  float  f_multiplier;
  uint8_t   e_relative        :1; ///< bool: e axis relative? Overrides all_relative
} TARGET;
/// this holds all the possible data from a received command
typedef struct {
    uint8_t					seen_G	:1;
    uint8_t					seen_M	:1;
    uint8_t					seen_X	:1;
    uint8_t					seen_Y	:1;
    uint8_t					seen_Z	:1;
    uint8_t					seen_E	:1;
    uint8_t					seen_F	:1;
    uint8_t					seen_S	:1;
    uint8_t					seen_P	:1;
    uint8_t					seen_T	:1;
    uint8_t					seen_N	:1;
#ifdef ARC_SUPPORT        
    uint8_t					seen_I	:1;
    uint8_t					seen_J	:1;
    uint8_t					seen_R	:1;
#endif        
    uint8_t					seen_checksum				:1; ///< seen a checksum?
    uint8_t					seen_semi_comment		:1; ///< seen a semicolon?
    uint8_t					seen_parens_comment	:1; ///< seen an open parenthesis
    uint8_t         read_string         :1; ///< Currently reading a string.
    uint8_t					option_all_relative	:1; ///< relative or absolute coordinates?
    uint8_t					option_e_relative		:1; ///< same for e axis (M82/M83)

  uint32_t          N;          ///< line number
  uint32_t          N_expected; ///< expected line number

  float            S;          ///< S word (various uses)
  uint16_t          P;          ///< P word (various uses)

	uint16_t					G;				///< G command number
	uint16_t					M;				///< M command number
	TARGET						target;		///< target position: X, Y, Z, E and F
#ifdef ARC_SUPPORT
    float  I;
    float  J;
    float  R;
#endif
	uint8_t						T;				///< T word (tool index)

	uint8_t						checksum_read;				///< checksum in gcode command
	uint8_t						checksum_calculated;	///< checksum we calculated
} GCODE_COMMAND;

extern void process_gcode_command();
extern uint8_t gcode_parse_char(uint8_t c);
extern void init_gcode();

