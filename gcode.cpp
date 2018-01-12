#include "gcode.h"
#include "timer.h"
#include "common.h"
#include "temp.h"

#include<stdint.h>
#if defined(__AVR__) || defined(ESP8266)
#include<arduino.h>
#endif
#include "eprom.h"
/// crude crc macro
#define crc(a, b)		(a ^ b)
decfloat read_digit;
GCODE_COMMAND next_target;
uint16_t last_field = 0;
/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const uint32_t powers[] = {1, 1, 10, 100, 1000, 10000, 100000, 1000000};




static float decfloat_to_float(void) {
  float r = read_digit.mantissa;
  uint8_t	e = read_digit.exponent;

  // e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero

  if (e) r = (r /*+ powers[e-1] / 2*/) / powers[e];

  return read_digit.sign ? -r : r;
}

uint8_t gcode_parse_char(uint8_t c) {
  uint8_t checksum_char = c;
  //Serial.write(c);
  // uppercase
  if (c >= 'a' && c <= 'z')
    c &= ~32;

  // An asterisk is a quasi-EOL and always ends all fields.
  if (c == '*') {
    next_target.seen_semi_comment = next_target.seen_parens_comment =
                                      next_target.read_string = 0;
  }

  // Skip comments and strings.
  if (next_target.seen_semi_comment == 0 &&
      next_target.seen_parens_comment == 0 &&
      next_target.read_string == 0
     ) {
    // Check if the field has ended. Either by a new field, space or EOL.
    if (last_field && (c < '0' || c > '9') && c != '.') {
      switch (last_field) {
        case 'G':
          next_target.G = read_digit.mantissa;
          break;
        case 'M':
          next_target.M = read_digit.mantissa;

          break;
        case 'X':

          next_target.target.axis[X] = decfloat_to_float();
          break;
        case 'Y':
          next_target.target.axis[Y] = decfloat_to_float();
          break;
        case 'Z':
          next_target.target.axis[Z] = decfloat_to_float();
          break;
#ifdef ARC_SUPPORT
        case 'I':
          next_target.I = decfloat_to_float();
          break;
        case 'J':
          next_target.J = decfloat_to_float();
          break;
        case 'R':
          next_target.R = decfloat_to_float();
          break;
#endif
        case 'E':
          next_target.target.axis[E] = decfloat_to_float();
          break;
        case 'F':
          // just use raw integer, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
          next_target.target.F = decfloat_to_float() / 60;
          break;
        case 'S':
          // if this is temperature, multiply by 4 to convert to quarter-degree units
          // cosmetically this should be done in the temperature section,
          // but it takes less code, less memory and loses no precision if we do it here instead
          if ((next_target.M == 104) || (next_target.M == 109) || (next_target.M == 140))
            next_target.S = decfloat_to_float();
          // if this is heater PID stuff, multiply by PID_SCALE because we divide by PID_SCALE later on
          else if ((next_target.M == 206))
            next_target.S = decfloat_to_float();
          else if ((next_target.M >= 130) && (next_target.M <= 132))
            next_target.S = decfloat_to_float();
          else
            next_target.S = decfloat_to_float();
          break;
        case 'P':
          next_target.P = decfloat_to_float();
          break;
        case 'T':
          next_target.T = read_digit.mantissa;
          break;
        case 'N':
          next_target.N = decfloat_to_float();
          break;
        case '*':
          next_target.checksum_read = decfloat_to_float();
          break;
      }
    }

    // new field?
    if ((c >= 'A' && c <= 'Z') || c == '*') {
      last_field = c;
      read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;
    }

    // process character
    // Can't do ranges in switch..case, so process actual digits here.
    // Do it early, as there are many more digits than characters expected.
    if (c >= '0' && c <= '9') {
      if (read_digit.exponent < DECFLOAT_EXP_MAX + 1) {
        // this is simply mantissa = (mantissa * 10) + atoi(c) in different clothes
        read_digit.mantissa = (read_digit.mantissa << 3) +
                              (read_digit.mantissa << 1) + (c - '0');
        if (read_digit.exponent)
          read_digit.exponent++;
      }
    }
    else {
      switch (c) {
        // Each currently known command is either G or M, so preserve
        // previous G/M unless a new one has appeared.
        // FIXME: same for T command
        case 'G':
          next_target.seen_G = 1;
          next_target.seen_M = 0;
          next_target.M = 0;
          break;
        case 'M':
          next_target.seen_M = 1;
          next_target.seen_G = 0;
          next_target.G = 0;
          break;
#ifdef ARC_SUPPORT
        case 'I':
          next_target.seen_I = 1;
          break;
        case 'J':
          next_target.seen_J = 1;
          break;
        case 'R':
          next_target.seen_R = 1;
          break;
#endif
        case 'X':
          next_target.seen_X = 1;
          break;
        case 'Y':
          next_target.seen_Y = 1;
          break;
        case 'Z':
          next_target.seen_Z = 1;
          break;
        case 'E':
          next_target.seen_E = 1;
          break;
        case 'F':
          next_target.seen_F = 1;
          break;
        case 'S':
          next_target.seen_S = 1;
          break;
        case 'P':
          next_target.seen_P = 1;
          break;
        case 'T':
          next_target.seen_T = 1;
          break;
        case 'N':
          next_target.seen_N = 1;
          break;
        case '*':
          next_target.seen_checksum = 0;//1;
          break;

        // comments
        case ';':
          next_target.seen_semi_comment = 1;    // Reset by EOL.
          break;
        case '(':
          next_target.seen_parens_comment = 1;  // Reset by ')' or EOL.
          break;

        // now for some numeracy
        case '-':
          read_digit.sign = 1;
          // force sign to be at start of number, so 1-2 = -2 instead of -12
          read_digit.exponent = 0;
          read_digit.mantissa = 0;
          break;
        case '.':
          if (read_digit.exponent == 0)
            read_digit.exponent = 1;
          break;
#ifdef	DEBUG
        case ' ':
        case '\t':
        case 10:
        case 13:
          // ignore
          break;
#endif

        default:
#ifdef	DEBUG
          // invalid
          zprintf(PSTR("?%d\n"), c);
#endif
          break;
      }
    }
  } else if ( next_target.seen_parens_comment == 1 && c == ')')
    next_target.seen_parens_comment = 0; // recognize stuff after a (comment)

  if (next_target.seen_checksum == 0)
    next_target.checksum_calculated =
      crc(next_target.checksum_calculated, checksum_char);

  // end of line
  if ((c == 10) || (c == 13)) {

    // Assume G1 for unspecified movements.
    if ( ! next_target.seen_G && ! next_target.seen_M && ! next_target.seen_T &&
         (next_target.seen_X || next_target.seen_Y || next_target.seen_Z ||
          next_target.seen_E || next_target.seen_F)) {
      next_target.seen_G = 1;
      next_target.G = 1;
    }

    if (1) {
      if (((next_target.checksum_calculated == next_target.checksum_read) || (next_target.seen_checksum == 0))

         ) {
        // process
        process_gcode_command();

      }
      else {
        zprintf(PSTR("rs N%ld Expected checksum %d\n"), next_target.N_expected, next_target.checksum_calculated);
        // 				request_resend();
      }
    }
    else {
      zprintf(PSTR("rs N%ld Expected line number %ld\n"), next_target.N_expected, next_target.N_expected);
      // 			request_resend();
    }

    // reset variables
    uint8_t ok = next_target.seen_G || next_target.seen_M || next_target.seen_T;
    next_target.seen_X = next_target.seen_Y = next_target.seen_Z = \
                         next_target.seen_E = next_target.seen_F = next_target.seen_S = \
                             next_target.seen_P = next_target.seen_T = next_target.seen_N = \
                                 next_target.seen_G = next_target.seen_M = next_target.seen_checksum = \
                                     next_target.seen_semi_comment = next_target.seen_parens_comment = \
                                         next_target.read_string = next_target.checksum_read = \
                                             next_target.checksum_calculated = 0;
#ifdef ARC_SUPPORT
    next_target.seen_R = next_target.seen_I = next_target.seen_J = 0;
#endif
    last_field = 0;
    read_digit.sign = read_digit.mantissa = read_digit.exponent = 0;

    if (next_target.option_all_relative) {
      next_target.target.axis[X] = next_target.target.axis[Y] = next_target.target.axis[Z] = 0;
    }
    if (next_target.option_all_relative || next_target.option_e_relative) {
      next_target.target.axis[E] = 0;
    }
    if (ok) return 2;
    return 1;
  }

#ifdef SD
  // Handle string reading. After checking for EOL.
  if (next_target.read_string) {
    if (c == ' ') {
      if (str_buf_ptr)
        next_target.read_string = 0;
    }
    else if (str_buf_ptr < STR_BUF_LEN) {
      gcode_str_buf[str_buf_ptr] = c;
      str_buf_ptr++;
      gcode_str_buf[str_buf_ptr] = '\0';
    }
  }
#endif /* SD */

  return 0;
}


// implement minimalis code to match teacup

float lastE, lastZ;

void printposition() {
  zprintf(PSTR("X:%f Y:%f Z:%f E:%f\n"),
          ff(cx1), ff(cy1),
          ff(cz1), ff(ce01));

}

#define queue_wait() needbuffer()

void delay_ms(uint32_t d) {
#if defined(__AVR__) || defined(ESP8266)
  delayMicroseconds(d * 1000);
#else
  // delay on pc,

#endif

}
void doclock() {
  feedthedog();
  motionloop();
}
void temp_wait(void) {
  wait_for_temp = 1;
  int32_t c = 0;
  while (wait_for_temp && !temp_achieved()) {
    doclock();
    //delayMicroseconds(1000);
    if (c++ > 200000) {
      c = 0;
      zprintf(PSTR("T:%f\n"), ff(Input));
      //zprintf(PSTR("Heating\n"));
    }
  }
  wait_for_temp = 0;
}
int32_t mvc = 0;
void enqueue_home(TARGET *t, uint8_t endstop_check, uint8_t endstop_stop_cond, int g0 = 1)
{
  checkendstop = 0;
  addmove(t->F * t->f_multiplier, t->axis[X]
          , t->axis[Y]
          , t->axis[Z]
          , t->axis[E]
          , g0);
  //waitbufferempty();


}
static void enqueue(TARGET *) __attribute__ ((always_inline));
inline void enqueue(TARGET *t, int g0 = 1) {
  enqueue_home(t, 0, 0, g0);
}

void process_gcode_command() {
  uint32_t	backup_f;

  // convert relative to absolute
  if (next_target.option_all_relative) {
    next_target.target.axis[X] += cx1;//startpoint.axis[X];
    next_target.target.axis[Y] += cy1;//startpoint.axis[Y];
    next_target.target.axis[Z] += cz1;//startpoint.axis[Z];
    next_target.target.axis[E] += ce01;//startpoint.axis[Z];
  }

  // E relative movement.
  // Matches Sprinter's behaviour as of March 2012.
  if (next_target.option_all_relative || next_target.option_e_relative)
    next_target.target.e_relative = 1;
  else
    next_target.target.e_relative = 0;

  // The GCode documentation was taken from http://reprap.org/wiki/Gcode .

  if (next_target.seen_T) {
    //? --- T: Select Tool ---
    //?
    //? Example: T1
    //?
    //? Select extruder number 1 to build with.  Extruder numbering starts at 0.

    //next_tool = next_target.T;
  }
  // check if buffer is near full
  /*
    int bl=bufflen();
    float spd=1;
    if (bl>NUMBUFFER/2) {
      spd=(float)NUMBUFFER/(bl*4+2);
    }
  */
  if (next_target.seen_G) {
    uint8_t axisSelected = 0;
    //zprintf(PSTR("Gcode %su \n"),next_target.G);
    switch (next_target.G) {
      case 0:
        //? G0: Rapid Linear Motion
        //?
        //? Example: G0 X12
        //?
        //? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)
        //?
        if (!next_target.seen_F) {
          backup_f = next_target.target.F;
          next_target.target.F = 500;
          enqueue(&next_target.target, 1);
          next_target.target.F = backup_f;
        } else
          //next_target.target.F*=spd;
          enqueue(&next_target.target, 1);
        break;

      case 1:
        //? --- G1: Linear Motion at Feed Rate ---
        //?
        //? Example: G1 X90.6 Y13.8 E22.4
        //?
        //? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
        //?
        //next_target.target.axis[E]=0;
        // auto retraction change
        enqueue(&next_target.target, 0);
        break;

      //	G2 - Arc Clockwise
      //	G3 - Arc anti Clockwise
      case 2:
      case 3:
        // we havent immplement R
        ///*
        //*/
        break;

      case 4:
        //? --- G4: Dwell ---
        //?
        //? Example: G4 P200
        //?
        //? In this case sit still doing nothing for 200 milliseconds.  During delays the state of the machine (for example the temperatures of its extruders) will still be preserved and controlled.
        //?
        queue_wait();
        // delay
        if (next_target.seen_P) {
          for (; next_target.P > 0; next_target.P--) {
            doclock();
            delay_ms(1);
          }
        }
        break;
      case 28:
        homing(0, 0, 0, 0);
        next_target.target.axis[X] = cx1;
        next_target.target.axis[Y] = cy1;
        next_target.target.axis[Z] = cz1;
        next_target.target.axis[E] = ce01;
        printposition();
        break;

      case 90:
        //? --- G90: Set to Absolute Positioning ---
        //?
        //? Example: G90
        //?
        //? All coordinates from now on are absolute relative to the origin
        //? of the machine. This is the RepRap default.
        //?
        //? If you ever want to switch back and forth between relative and
        //? absolute movement keep in mind, X, Y and Z follow the machine's
        //? coordinate system while E doesn't change it's position in the
        //? coordinate system on relative movements.
        //?

        // No wait_queue() needed.
        next_target.option_all_relative = 0;
        break;

      case 91:
        //? --- G91: Set to Relative Positioning ---
        //?
        //? Example: G91
        //?
        //? All coordinates from now on are relative to the last position.
        //?

        // No wait_queue() needed.
        next_target.option_all_relative = 1;
        break;

      case 92:
        //? --- G92: Set Position ---
        //?
        //? Example: G92 X10 E90
        //?
        //? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.
        //?

        queue_wait();

        if (next_target.seen_X) {
          cx1 = next_target.target.axis[X];
          axisSelected = 1;
        };
        if (next_target.seen_Y) {
          cy1 = next_target.target.axis[Y];
          axisSelected = 1;
        };
        if (next_target.seen_Z) {
          cz1 = next_target.target.axis[Z];
          axisSelected = 1;
        };
        if (next_target.seen_E) {
          lastE = ce01 = next_target.target.axis[E];
          axisSelected = 1;
        };

        if (axisSelected == 0) {
          cx1 = next_target.target.axis[X] =
                                 cy1 = next_target.target.axis[Y] =
                                       cz1 = next_target.target.axis[Z] =
                                             ce01 = next_target.target.axis[E] = 0;
        }

        
        break;

      // unknown gcode: spit an error
      default:
        zprintf(PSTR("E: Bad G-code %d\nok\n"), next_target.G);
        return;
    }
  }
  else if (next_target.seen_M) {
    //uint8_t i;

    switch (next_target.M) {
      case 0:
      //? --- M0: machine stop ---
      //?
      //? Example: M0
      //?
      //? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
      //? Unimplemented, especially the restart after the stop. Fall trough to M2.
      //?

      case 2:
      case 84: // For compatibility with slic3rs default end G-code.
        //? --- M2: program end ---
        //?
        //? Example: M2
        //?
        //? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
        //?
        queue_wait();
        //for (i = 0; i < NUM_HEATERS; i++)temp_set(i, 0);
        power_off();

        zprintf(PSTR("\nstop\n"));
        break;

      case 6:
        //? --- M6: tool change ---
        //?
        //? Undocumented.
        //tool = next_tool;
        break;

      // M3/M101- extruder on
      case 3:
      case 101:
        //? --- M101: extruder on ---
        //?
        //? Undocumented.
        temp_wait();
        // enable the laser or spindle
        break;

      // M5/M103- extruder off
      case 5:
      case 103:
        //? --- M103: extruder off ---
        //?
        //? Undocumented.
        // disable laser/spindle
        break;
      case 104:
        set_temp(next_target.S);
        break;
      case 105:
        zprintf(PSTR("T:%f\n"), ff(Input));
        //zprintf(PSTR("B:%d/%d\n"), fi(bufflen()),fi(NUMBUFFER));
        break;
      case 109:
        set_temp(next_target.S);
        temp_wait();
        break;
      case 7:
      case 107:
        // set laser pwm off
        break;
      case 106:
        // set laser pwm on

        break;

      case 112:
        //? --- M112: Emergency Stop ---
        //?
        //? Example: M112
        //?
        //? Any moves in progress are immediately terminated, then the printer
        //? shuts down. All motors and heaters are turned off. Only way to
        //? restart is to press the reset button on the master microcontroller.
        //? See also M0.
        //?

        break;

      case 114:
        //? --- M114: Get Current Position ---
        //?
        //? Example: M114
        //?
        //? This causes the RepRap machine to report its current X, Y, Z and E coordinates to the host.
        //?
        //? For example, the machine returns a string such as:
        //?
        //? <tt>ok C: X:0.00 Y:0.00 Z:0.00 E:0.00</tt>
        //?
#ifdef ENFORCE_ORDER
        // wait for all moves to complete
        queue_wait();
#endif
        printposition();
        break;

      case 115:
        zprintf(PSTR("FIRMWARE_NAME:Repetier_1.9 FIRMWARE_URL:null PROTOCOL_VERSION:1.0 MACHINE_TYPE:teacup EXTRUDER_COUNT:1 REPETIER_PROTOCOL:\n"));

        break;


      case 119:
        //? --- M119: report endstop status ---
        //? Report the current status of the endstops configured in the
        //? firmware to the host.
        docheckendstop();
        zprintf(PSTR("Endstop:"));
        for (int e = 0; e < 3; e++) {
          zprintf(endstopstatus[e] < 0 ? PSTR("ON ") : PSTR("OFF "));
        }
        zprintf(PSTR("\n"));

        break;

        // unknown mcode: spit an error
#ifdef USE_EEPROM
      case 502:
        reset_eeprom();
      case 205:
        reload_eeprom();
      case 503:
        zprintf(PSTR("EPR:3 145 %f Xmax\n"), ff(ax_max[0]));
        zprintf(PSTR("EPR:3 149 %f Ymax\n"), ff(ax_max[1]));
        zprintf(PSTR("EPR:3 153 %f Zmax\n"), ff(ax_max[2]));

        zprintf(PSTR("EPR:3 3 %f StepX\n"), ff(stepmmx[0]));
        zprintf(PSTR("EPR:3 7 %f StepY\n"), ff(stepmmx[1]));
        zprintf(PSTR("EPR:3 11 %f StepZ\n"), ff(stepmmx[2]));
        zprintf(PSTR("EPR:3 0 %f StepE\n"), ff(stepmmx[3]));

        zprintf(PSTR("EPR:2 15 %d MFX\n"), fi(maxf[0]));
        zprintf(PSTR("EPR:2 19 %d MFY\n"), fi(maxf[1]));
        zprintf(PSTR("EPR:2 23 %d MFZ\n"), fi(maxf[2]));
        zprintf(PSTR("EPR:2 27 %d MFE\n"), fi(maxf[3]));


        zprintf(PSTR("EPR:3 51 %d AccelX\n"), fi(accel[0]));
        zprintf(PSTR("EPR:3 55 %d AccelY\n"), fi(accel[1]));
        zprintf(PSTR("EPR:3 59 %d AccelZ\n"), fi(accel[2]));
        zprintf(PSTR("EPR:3 63 %d AccelE\n"), fi(accel[3]));
        zprintf(PSTR("EPR:3 67 %d Mv AccelX\n"), fi(mvaccel[0]));
        zprintf(PSTR("EPR:3 71 %d Mv AccelY\n"), fi(mvaccel[1]));
        zprintf(PSTR("EPR:3 75 %d Mv AccelZ\n"), fi(mvaccel[2]));

        zprintf(PSTR("EPR:2 35 %d XJerk\n"), fi(jerk[0]));
        zprintf(PSTR("EPR:2 39 %d YJerk\n"), fi(jerk[1]));
        zprintf(PSTR("EPR:2 43 %d Zjerk\n"), fi(jerk[2]));
        zprintf(PSTR("EPR:2 47 %d Ejerk\n"), fi(jerk[3]));
#ifdef USE_BACKLASH
        zprintf(PSTR("EPR:2 80 %d Xbacklash\n"), fi(xback[0]));
        zprintf(PSTR("EPR:2 84 %d Ybacklash\n"), fi(xback[1]));
        zprintf(PSTR("EPR:2 88 %d Zbacklash\n"), fi(xback[2]));
        zprintf(PSTR("EPR:2 92 %d Ebacklash\n"), fi(xback[3]));
#endif

        break;
      case 206:
        if (next_target.seen_X)next_target.S = next_target.target.axis[X];
        int32_t S_F;
        S_F = (next_target.S * 1000);
        int32_t S_I;
        S_I = (next_target.S);
        if (next_target.seen_P)
          switch (next_target.P) {
#define eprom_wr(id,pos,val){\
  case id:\
    eeprom_write_dword((uint32_t *) &pos, val);\
    break;\
  }
              eprom_wr(153, EE_zmax, S_F);
              eprom_wr(0, EE_estepmm, S_F);
              eprom_wr(3, EE_xstepmm, S_F);
              eprom_wr(7, EE_ystepmm, S_F);
              eprom_wr(11, EE_zstepmm, S_F);

              eprom_wr(15, EE_max_x_feedrate, S_I);
              eprom_wr(19, EE_max_y_feedrate, S_I);
              eprom_wr(23, EE_max_z_feedrate, S_I);
              eprom_wr(27, EE_max_e_feedrate, S_I);

              eprom_wr(35, EE_xjerk, S_I);
              eprom_wr(39, EE_yjerk, S_I);
              eprom_wr(43, EE_zjerk, S_I);
              eprom_wr(47, EE_ejerk, S_I);

              eprom_wr(51, EE_accelx, S_I);
              eprom_wr(55, EE_accely, S_I);
              eprom_wr(59, EE_accelz, S_I);
              eprom_wr(63, EE_accele, S_I);

              eprom_wr(67, EE_mvaccelx, S_I);
              eprom_wr(71, EE_mvaccely, S_I);
              eprom_wr(75, EE_mvaccelz, S_I);

#ifdef USE_BACKLASH
              eprom_wr(80, EE_xbacklash, S_I);
              eprom_wr(84, EE_ybacklash, S_I);
              eprom_wr(88, EE_zbacklash, S_I);
              eprom_wr(92, EE_ebacklash, S_I);
#endif
              /*            //case 153:
                            //eeprom_write_dword((uint32_t *) &EE_zmax, S_F);
                            //break;
                          case 0:
                            eeprom_write_dword((uint32_t *) &EE_estepmm, S_F);
                            break;
                          case 3:
                            eeprom_write_dword((uint32_t *) &EE_xstepmm, S_F);
                            break;
                          case 7:
                            eeprom_write_dword((uint32_t *) &EE_ystepmm, S_F);
                            break;
                          case 11:
                            eeprom_write_dword((uint32_t *) &EE_zstepmm, S_F);
                            break;
                          case 15:
                            eeprom_write_dword((uint32_t *) &EE_max_x_feedrate, S_I);
                            break;
                          case 19:
                            eeprom_write_dword((uint32_t *) &EE_max_y_feedrate, S_I);
                            break;
                          case 23:
                            eeprom_write_dword((uint32_t *) &EE_max_z_feedrate, S_I);
                            break;
                          case 27:
                            eeprom_write_dword((uint32_t *) &EE_max_e_feedrate, S_I);
                            break;
                          case 35:
                            eeprom_write_dword((uint32_t *) &EE_xjerk, S_I);
                            break;
                          case 39:
                            eeprom_write_dword((uint32_t *) &EE_yjerk, S_I);
                            break;
                          case 43:
                            eeprom_write_dword((uint32_t *) &EE_zjerk, S_I);
                            break;
                          case 47:
                            eeprom_write_dword((uint32_t *) &EE_ejerk, S_I);
                            break;
                          case 51:
                            eeprom_write_dword((uint32_t *) &EE_accelx, S_I);
                            break;
                          case 55:
                            eeprom_write_dword((uint32_t *) &EE_accely, S_I);
                            break;
                          case 59:
                            eeprom_write_dword((uint32_t *) &EE_accelz, S_I);
                            break;
                          case 63:
                            eeprom_write_dword((uint32_t *) &EE_accele, S_I);
                            break;
                          case 67:
                            eeprom_write_dword((uint32_t *) &EE_mvaccelx, S_I);
                            break;
                          case 71:
                            eeprom_write_dword((uint32_t *) &EE_mvaccely, S_I);
                            break;
                          case 75:
                            eeprom_write_dword((uint32_t *) &EE_mvaccelz, S_I);
                            break;*/
          }
        reload_eeprom();
        break;
#endif
      case 220:
        //? --- M220: Set speed factor override percentage ---
        if ( ! next_target.seen_S)
          break;
        // Scale 100% = 256
        next_target.target.f_multiplier = next_target.S / 100;
        break;

      case 221:
        //? --- M220: Set speed factor override percentage ---
        if ( ! next_target.seen_S)
          break;
        // Scale 100% = 256
        e_multiplier = next_target.S / 100;
        break;

      default:;
        //zprintf(PSTR("E: Bad M-code %d\nok\n"), next_target.M);
    } // switch (next_target.M)
  } // else if (next_target.seen_M)
} // process_gcode_command()

void init_gcode() {
  next_target.target.F = 100;
  next_target.target.f_multiplier = 1;
  e_multiplier = 1;
  next_target.option_all_relative = 0;

}



