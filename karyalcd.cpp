#include "karyalcd.h"


void KaryaLCD::end() {
    if (this->buffer) {
        free(this->buffer);
        this->buffer = NULL;
    }
}



void KaryaLCD::drawString(int16_t xMove, int16_t yMove, String strUser) {
    uint16_t lineHeight = pgm_read_byte(fontData + HEIGHT_POS);
    char* text = utf8ascii(strUser);
    uint16_t textLength = strlen(text);
    uint16_t textWidth = getStringWidth(text, textLength);
    drawStringInternal(xMove, yMove, text, textLength, textWidth);
    free(text);
}

void KaryaLCD::setFont(const uint8_t* font) {
    fontData = font;
}


bool KaryaLCD::allocateBuffer() {

  
  this->end();
	
  if(this->buffer==NULL) {
    this->buffer = (uint8_t*) malloc((sizeof(uint8_t) * displayBufferSize) + getBufferOffset());
    this->buffer += getBufferOffset();

  
  }

  if(this->buffer_back==NULL) {
    this->buffer_back = (uint8_t*) malloc((sizeof(uint8_t) * (displayBufferSize / BACK_BLOCK)  ) + getBufferOffset());
    this->buffer_back += getBufferOffset();
  
   
  }

  return true;
}
void inline KaryaLCD::drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const uint8_t *data, uint16_t offset, uint16_t bytesInData) {
  if (width < 0 || height < 0) return;
  if (yMove + height < 0 || yMove > this->displayHeight)  return;
  if (xMove + width  < 0 || xMove > this->displayWidth)   return;
  uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
  int8_t   yOffset      = yMove & 7;

  bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

  int16_t initYMove   = yMove;
  int8_t  initYOffset = yOffset;


  for (uint16_t i = 0; i < bytesInData; i++) {

    // Reset if next horizontal drawing phase is started.
    if ( i % rasterHeight == 0) {
      yMove   = initYMove;
      yOffset = initYOffset;
    }

    uint8_t currentByte = pgm_read_byte(data + offset + i);

    int16_t xPos = xMove + (i / rasterHeight);
    int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * this->displayWidth;

//  int16_t yScreenPos = yMove + yOffset;
    int16_t dataPos    = xPos  + yPos;

    if (dataPos >=  0  && dataPos < displayBufferSize &&
        xPos    >=  0  && xPos    < this->displayWidth ) {

      if (yOffset >= 0) {
        switch (this->color) {
          case WHITE:   buffer[dataPos] |= currentByte << yOffset; break;
          case BLACK:   buffer[dataPos] &= ~(currentByte << yOffset); break;
          case INVERSE: buffer[dataPos] ^= currentByte << yOffset; break;
        }

        if (dataPos < (displayBufferSize - this->displayWidth)) {
          switch (this->color) {
            case WHITE:   buffer[dataPos + this->displayWidth] |= currentByte >> (8 - yOffset); break;
            case BLACK:   buffer[dataPos + this->displayWidth] &= ~(currentByte >> (8 - yOffset)); break;
            case INVERSE: buffer[dataPos + this->displayWidth] ^= currentByte >> (8 - yOffset); break;
          }
        }
      } else {
        // Make new offset position
        yOffset = -yOffset;

        switch (this->color) {
          case WHITE:   buffer[dataPos] |= currentByte >> yOffset; break;
          case BLACK:   buffer[dataPos] &= ~(currentByte >> yOffset); break;
          case INVERSE: buffer[dataPos] ^= currentByte >> yOffset; break;
        }

        // Prepare for next iteration by moving one block up
        yMove -= 8;

        // and setting the new yOffset
        yOffset = 8 - yOffset;
      }

  motionloop();
    }
  }
}
void KaryaLCD::drawStringInternal(int16_t xMove, int16_t yMove, char* text, uint16_t textLength, uint16_t textWidth) {
    uint8_t textHeight = pgm_read_byte(fontData + HEIGHT_POS);
    uint8_t firstChar = pgm_read_byte(fontData + FIRST_CHAR_POS);
    uint16_t sizeOfJumpTable = pgm_read_byte(fontData + CHAR_NUM_POS) * JUMPTABLE_BYTES;
  uint16_t cursorX         = 0;
  uint16_t cursorY         = 0;
    if (xMove + textWidth  < 0 || xMove > this->displayWidth ) {return;}
    if (yMove + textHeight < 0 || yMove > this->displayHeight ) {return;}

  for (uint16_t j = 0; j < textLength; j++) {
    int16_t xPos = xMove + cursorX;
    int16_t yPos = yMove + cursorY;

    uint8_t code = text[j];
    if (code >= firstChar) {
      uint8_t charCode = code - firstChar;

      // 4 Bytes per char code
      uint8_t msbJumpToChar    = pgm_read_byte( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES );                  // MSB  \ JumpAddress
      uint8_t lsbJumpToChar    = pgm_read_byte( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB);   // LSB /
      uint8_t charByteSize     = pgm_read_byte( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE);  // Size
      uint8_t currentCharWidth = pgm_read_byte( fontData + JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH); // Width

      // Test if the char is drawable
      if (!(msbJumpToChar == 255 && lsbJumpToChar == 255)) {
        // Get the position of the char data
        uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
        drawInternal(xPos, yPos, currentCharWidth, textHeight, fontData, charDataPosition, charByteSize);
      }

      cursorX += currentCharWidth;
    }
  }
}

uint16_t KaryaLCD::getStringWidth(const char* text, uint16_t length) {
    uint16_t firstChar = pgm_read_byte(fontData + FIRST_CHAR_POS);
    uint16_t stringWidth = 0;
    uint16_t maxWidth = 0;

    while (length--) {
        stringWidth += pgm_read_byte(fontData + JUMPTABLE_START + (text[length] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH);
        if (text[length] == 10) {
            maxWidth = max(maxWidth, stringWidth);
            stringWidth = 0;
        }
    }

    return max(maxWidth, stringWidth);
}
uint16_t KaryaLCD::getStringWidth(String strUser) {
  char* text = utf8ascii(strUser);
  uint16_t length = strlen(text);
  uint16_t width = getStringWidth(text, length);
  free(text);
  return width;
}

char* KaryaLCD::utf8ascii(String str) {
    uint16_t k = 0;
    uint16_t length = str.length() + 1;
    char* s = (char*)malloc(length * sizeof(char));
    if (!s) {
        return NULL;
    }
    str.toCharArray(s, length);
    length--;

    for (uint16_t i = 0; i < length; i++) {
        char c = s[i];
        if (c < 128) {
            s[k++] = c;
        }
    }

    s[k] = 0;
    return s;
}

void KaryaLCD::drawRect(int16_t x, int16_t y, int16_t width, int16_t height) {
  drawHorizontalLine(x, y, width);
  drawVerticalLine(x, y, height);
  drawVerticalLine(x + width - 1, y, height);

  drawHorizontalLine(x, y + height - 1, width);
}

void KaryaLCD::fillRect(int16_t xMove, int16_t yMove, int16_t width, int16_t height) {
  for (int16_t x = xMove; x < xMove + width; x++) {
    drawVerticalLine(x, yMove, height);
  }
}
void KaryaLCD::drawHorizontalLine(int16_t x, int16_t y, int16_t length) {
  if (y < 0 || y >= displayHeight) { return; }

  if (x < 0) {
    length += x;
    x = 0;
  }

  if ( (x + length) > displayWidth) {
    length = (displayWidth - x);
  }

  if (length <= 0) { return; }
  motionloop();

  uint8_t * bufferPtr = buffer;
  bufferPtr += (y >> 3) * displayWidth;
  bufferPtr += x;

  uint8_t drawBit = 1 << (y & 7);

  switch (color) {
    case WHITE:   while (length--) {
        *bufferPtr++ |= drawBit;
      }; break;
    case BLACK:   drawBit = ~drawBit;   while (length--) {
        *bufferPtr++ &= drawBit;
      }; break;
    case INVERSE: while (length--) {
        *bufferPtr++ ^= drawBit;
      }; break;
  }
}

void KaryaLCD::drawVerticalLine(int16_t x, int16_t y, int16_t length) {
  if (x < 0 || x >= displayWidth) return;
  
  if (y < 0) {
    length += y;
    y = 0;
  }

  if ( (y + length) > displayHeight) {
    length = (displayHeight - y);
  }

  if (length <= 0) return;


  
  uint8_t yOffset = y & 7;
  uint8_t drawBit;
  uint8_t *bufferPtr = buffer;

  bufferPtr += (y >> 3) * displayWidth;
  bufferPtr += x;

  if (yOffset) {
    yOffset = 8 - yOffset;
    drawBit = ~(0xFF >> (yOffset));

    if (length < yOffset) {
      drawBit &= (0xFF >> (yOffset - length));
    }

    switch (color) {
      case WHITE:   *bufferPtr |=  drawBit; break;
      case BLACK:   *bufferPtr &= ~drawBit; break;
      case INVERSE: *bufferPtr ^=  drawBit; break;
    }

    if (length < yOffset) return;

    length -= yOffset;
    bufferPtr += displayWidth;
  }

  if (length >= 8) {
    switch (color) {
      case WHITE:
      case BLACK:
        drawBit = (color == WHITE) ? 0xFF : 0x00;
        do {
          *bufferPtr = drawBit;
          bufferPtr += displayWidth;
          length -= 8;
        } while (length >= 8);
        break;
      case INVERSE:
        do {
          *bufferPtr = ~(*bufferPtr);
          bufferPtr += displayWidth;
          length -= 8;
        } while (length >= 8);
        break;
    }
  }

  if (length > 0) {
    drawBit = (1 << (length & 7)) - 1;
    switch (color) {
      case WHITE:   *bufferPtr |=  drawBit; break;
      case BLACK:   *bufferPtr &= ~drawBit; break;
      case INVERSE: *bufferPtr ^=  drawBit; break;
    }
  }
  motionloop();
}