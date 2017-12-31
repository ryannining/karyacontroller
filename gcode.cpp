#include "gcode.h"
#include "common.h"
#include<stdint.h>
#if defined(__AVR__) || defined(ESP8266)
    #include<arduino.h>
#else
#include <stdio.h>
#endif
/// crude crc macro
#define crc(a, b)		(a ^ b)
decfloat read_digit;
GCODE_COMMAND next_target;
uint16_t last_field = 0;
/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const uint32_t powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

static int32_t decfloat_to_int(void) {
	uint32_t r = read_digit.mantissa;
	uint8_t	e = read_digit.exponent;

	// e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero

	if (e) r = (r + powers[e] / 2) / powers[e];

	return read_digit.sign ? -r : r;
}

static double decfloat_to_double(void) {
	double r = read_digit.mantissa;
	uint8_t	e = read_digit.exponent;

	// e=1 means we've seen a decimal point but no digits after it, and e=2 means we've seen a decimal point with one digit so it's too high by one if not zero

	if (e) r = (r + powers[e] / 2) / powers[e];

	return read_digit.sign ? -r : r;
}

uint8_t gcode_parse_char(uint8_t c) {
	uint8_t checksum_char = c;

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
          #ifdef SD
            if (next_target.M == 23) {
              // SD card command with a filename.
              next_target.read_string = 1;  // Reset by string handler or EOL.
              str_buf_ptr = 0;
              last_field = 0;
            }
          #endif
					break;
				case 'X':
                    
            next_target.target.axis[X] = decfloat_to_double();
					break;
				case 'Y':
            next_target.target.axis[Y] = decfloat_to_double();
					break;
				case 'Z':
            next_target.target.axis[Z] = decfloat_to_double();
					break;
#ifdef ARC_SUPPORT                    
				case 'I':
            next_target.I = decfloat_to_double();
					break;
				case 'J':
            next_target.J = decfloat_to_double();
					break;
				case 'R':
            next_target.R = decfloat_to_double();
					break;
 #endif                   
				case 'E':
                        next_target.target.axis[E] = decfloat_to_double();
					break;
				case 'F':
					// just use raw integer, we need move distance and n_steps to convert it to a useful value, so wait until we have those to convert it
						next_target.target.F = decfloat_to_double()/60;
					break;
				case 'S':
					// if this is temperature, multiply by 4 to convert to quarter-degree units
					// cosmetically this should be done in the temperature section,
					// but it takes less code, less memory and loses no precision if we do it here instead
					if ((next_target.M == 104) || (next_target.M == 109) || (next_target.M == 140))
						next_target.S = decfloat_to_double();
					// if this is heater PID stuff, multiply by PID_SCALE because we divide by PID_SCALE later on
					else if ((next_target.M == 206))
						next_target.S = decfloat_to_double();
					else if ((next_target.M >= 130) && (next_target.M <= 132))
						next_target.S = decfloat_to_double();
					else
						next_target.S = decfloat_to_double();
					break;
				case 'P':
					next_target.P = decfloat_to_int();
					break;
				case 'T':
					next_target.T = read_digit.mantissa;
					break;
				case 'N':
					next_target.N = decfloat_to_int();
					break;
				case '*':
					next_target.checksum_read = decfloat_to_int();
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
            #if defined(__AVR__)
                Serial.print('?');
                Serial.print(c);
                Serial.print('\n');
            #elif defined(ESP8266)
                Serial.print('?');
                Serial.print(c);
                Serial.print('\n');
            #else
                xprintf("?%d\n",c);
            #endif
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

		if (
		#ifdef	REQUIRE_LINENUMBER
			((next_target.N >= next_target.N_expected) && (next_target.seen_N == 1)) ||
			(next_target.seen_M && (next_target.M == 110))
		#else
			1
		#endif
			) {
			if (
				#ifdef	REQUIRE_CHECKSUM
				((next_target.checksum_calculated == next_target.checksum_read) && (next_target.seen_checksum == 1))
				#else
				((next_target.checksum_calculated == next_target.checksum_read) || (next_target.seen_checksum == 0))
				#endif
				) {
				// process
				process_gcode_command();

        // Acknowledgement ("ok") is sent in the main loop, in mendel.c.

				// expect next line number
				if (next_target.seen_N == 1)
					next_target.N_expected = next_target.N + 1;
			}
			else {
				xprintf(PSTR("rs N%ld Expected checksum %d\n"), next_target.N_expected, next_target.checksum_calculated);
// 				request_resend();
			}
		}
		else {
			xprintf(PSTR("rs N%ld Expected line number %ld\n"), next_target.N_expected, next_target.N_expected);
// 			request_resend();
		}

		// reset variables
        uint8_t ok=next_target.seen_G || next_target.seen_M || next_target.seen_T;
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


void process_gcode_command() {}

