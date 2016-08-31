# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from __future__ import division
import logging
import time

import Adafruit_GPIO as GPIO
import Adafruit_GPIO.SPI as SPI


# Constants
SSD1331_I2C_ADDRESS = 0x3C    # 011110+SA0+RW - 0x3C or 0x3D
SSD1331_DRAWLINE = 0x21
SSD1331_DRAWRECT = 0x22
SSD1331_FILL = 0x26
SSD1331_COLUMNADDR = 0x15
SSD1331_ROWADDR = 0x75
SSD1331_SETCONTRASTA = 0x81
SSD1331_SETCONTRASTB = 0x82
SSD1331_SETCONTRASTC = 0x83
SSD1331_SETMASTERCURRENT = 0x87
SSD1331_SETREMAP = 0xA0
SSD1331_SETSTARTLINE = 0xA1
SSD1331_SETDISPLAYOFFSET = 0xA2
SSD1331_NORMALDISPLAY = 0xA4
SSD1331_DISPLAYALLON = 0xA5
SSD1331_DISPLAYALLOFF = 0xA6
SSD1331_INVERTDISPLAY = 0xA7
SSD1331_SETMULTIPLEX = 0xA8
SSD1331_SETMASTER = 0xAD
SSD1331_DISPLAYOFF = 0xAE
SSD1331_DISPLAYON = 0xAF
SSD1331_POWERMODE = 0xB0
SSD1331_SETPRECHARGE = 0xB1
SSD1331_SETDISPLAYCLOCKDIV = 0xB3
SSD1331_SETPRECHARGEA = 0x8A
SSD1331_SETPRECHARGEB = 0x8B
SSD1331_SETPRECHARGEC = 0x8B
SSD1331_SETPRECHARGELEVEL = 0xBB
SSD1331_SETVCOMH = 0xBE

# Scrolling constants
SSD1331_ACTIVATE_SCROLL = 0x2F
SSD1331_DEACTIVATE_SCROLL = 0x2E
SSD1331_SET_VERTICAL_SCROLL_AREA = 0x27
SSD1331_RIGHT_HORIZONTAL_SCROLL = 0x26
SSD1331_LEFT_HORIZONTAL_SCROLL = 0x27
SSD1331_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL = 0x29
SSD1331_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL = 0x2A

class SSD1331Base(object):
    """Base class for SSD1331-based OLED displays.  Implementors should subclass
    and provide an implementation for the _initialize function.
    """

    def __init__(self, width, height, rst, dc=None, sclk=None, din=None, cs=None,
                 gpio=None, spi=None, i2c_bus=None, i2c_address=SSD1331_I2C_ADDRESS,
                 i2c=None):
        self._log = logging.getLogger('Adafruit_SSD1331.SSD1331Base')
        self._spi = None
        self._i2c = None
        self.width = width
        self.height = height
        self._pages = height//8
        self._buffer = [0]*(width*height*2)
        # Default to platform GPIO if not provided.
        self._gpio = gpio
        if self._gpio is None:
            self._gpio = GPIO.get_platform_gpio()
        # Setup reset pin.
        self._rst = rst
        self._gpio.setup(self._rst, GPIO.OUT)
        # Handle hardware SPI
        if spi is not None:
            self._log.debug('Using hardware SPI')
            self._spi = spi
            self._spi.set_clock_hz(8000000)
        # Handle software SPI
        elif sclk is not None and din is not None and cs is not None:
            self._log.debug('Using software SPI')
            self._spi = SPI.BitBang(self._gpio, sclk, din, None, cs)
        # Handle hardware I2C
        elif i2c is not None:
            self._log.debug('Using hardware I2C with custom I2C provider.')
            self._i2c = i2c.get_i2c_device(i2c_address)
        else:
            self._log.debug('Using hardware I2C with platform I2C provider.')
            import Adafruit_GPIO.I2C as I2C
            if i2c_bus is None:
                self._i2c = I2C.get_i2c_device(i2c_address)
            else:
                self._i2c = I2C.get_i2c_device(i2c_address, busnum=i2c_bus)
        # Initialize DC pin if using SPI.
        if self._spi is not None:
            if dc is None:
                raise ValueError('DC pin must be provided when using SPI.')
            self._dc = dc
            self._gpio.setup(self._dc, GPIO.OUT)

    def _initialize(self):
        raise NotImplementedError

    def command(self, c):
        """Send command byte to display."""
        if self._spi is not None:
            # SPI write.
            self._gpio.set_low(self._dc)
            self._spi.write([c])
        else:
            # I2C write.
            control = 0x00   # Co = 0, DC = 0
            self._i2c.write8(control, c)

    def data(self, c):
        """Send byte of data to display."""
        if self._spi is not None:
            # SPI write.
            self._gpio.set_high(self._dc)
            self._spi.write([c])
        else:
            # I2C write.
            control = 0x40   # Co = 0, DC = 0
            self._i2c.write8(control, c)

    def begin(self):
        """Initialize display."""
        # Reset and initialize display.
        self.reset()
        self._initialize()
        # Turn on the display.
        self.command(SSD1331_DISPLAYON)

    def reset(self):
        """Reset the display."""
        # Set reset high for a millisecond.
        self._gpio.set_high(self._rst)
        time.sleep(0.001)
        # Set reset low for 10 milliseconds.
        self._gpio.set_low(self._rst)
        time.sleep(0.010)
        # Set reset high again.
        self._gpio.set_high(self._rst)

    def color565(self, r, g, b):
        return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
        
    def display(self):
        """Write display buffer to physical display."""
        # Write buffer data.
        if self._spi is not None:
            for y in range(0, self.height):
                self.command(SSD1331_COLUMNADDR)
                self.command(0)              # Column start address. (0 = reset)
                self.command(self.width-1)   # Column end address.
                self.command(SSD1331_ROWADDR)
                self.command(y)              # Page start address. (0 = reset)
                self.command(self.height-1)  # height end address.
                # Set DC high for data.
                self._gpio.set_high(self._dc)
                # Write buffer.
                self._spi.write(self._buffer[2*y*self.width:2*(y+1)*self.width])
        else:
            self.command(SSD1331_COLUMNADDR)
            self.command(0)              # Column start address. (0 = reset)
            self.command(self.width-1)   # Column end address.
            self.command(SSD1331_ROWADDR)
            self.command(0)              # Page start address. (0 = reset)
            self.command(self.height-1)  # height end address.
            for i in range(0, len(self._buffer), 16):
                control = 0x40   # Co = 0, DC = 0
                self._i2c.writeList(control, self._buffer[i:i+16])

    def image(self, image):
        """Set buffer to value of Python Imaging Library image.  The image should
        be in 1 bit mode and a size equal to the display size.
        """
        if image.mode != '1' and image.mode != 'RGB' and image.mode != 'RGBA':
            raise ValueError('Image must be in mode 1, RGB or RGBA.')
        imwidth, imheight = image.size
        if imwidth != self.width or imheight != self.height:
            raise ValueError('Image must be same dimensions as display ({0}x{1}).' \
                .format(self.width, self.height))
        # Grab all the pixels from the image, faster than getpixel.
        pix = image.load()
        # Iterate through the memory pages
        index = 0
        for y in range(self.height):
            # Iterate through all x axis columns.
            for x in range(self.width):
                # Set the bits for the column of pixels at the current position.
                bits = 0
                if image.mode == '1':
                    bits = 0 if pix[(x, y)] == 0 else 0x7FFF
                else:
                    p = pix[(x,y)]
                    bits = self.color565(p[0], p[1], p[2])
                # Update buffer byte and increment to next byte.
                self._buffer[index] = (bits >> 8) & 0xFF
                self._buffer[index+1] = bits & 0xFF
                index += 2

    def clear(self):
        """Clear contents of image buffer."""
        self._buffer = [0]*(self.width*self.height*2)

    def set_contrast(self, contrast):
        """Sets the contrast of the display.  Contrast should be a value between
        0 and 255."""
        if contrast < 0 or contrast > 255:
            raise ValueError('Contrast must be a value from 0 to 255 (inclusive).')
        self.command(SSD1331_SETCONTRASTA)
        self.command(contrast)
        self.command(SSD1331_SETCONTRASTB)
        self.command(contrast)
        self.command(SSD1331_SETCONTRASTC)
        self.command(contrast)

    def dim(self, dim):
        """Adjusts contrast to dim the display if dim is True, otherwise sets the
        contrast to normal brightness if dim is False.
        """
        # Assume dim display.
        contrast = 0
        # Adjust contrast based on VCC if not dimming.
        if not dim:
            contrast = 0xCF


class SSD1331_96_64(SSD1331Base):
    def __init__(self, rst, dc=None, sclk=None, din=None, cs=None, gpio=None,
                 spi=None, i2c_bus=None, i2c_address=SSD1331_I2C_ADDRESS,
                 i2c=None):
        # Call base class constructor.
        super(SSD1331_96_64, self).__init__(96, 64, rst, dc, sclk, din, cs,
                                             gpio, spi, i2c_bus, i2c_address, i2c)

    def _initialize(self):
        # 96x64 pixel specific initialization.
        self.command(SSD1331_DISPLAYOFF)                    # 0xAE
        self.command(SSD1331_SETREMAP)
        self.command(0x72)	# RGB color, and 0x76 for BGR
        self.command(SSD1331_SETSTARTLINE)                  # line #1
        self.command(0x0)
        self.command(SSD1331_SETDISPLAYOFFSET)              # line #2
        self.command(0x0)
        self.command(SSD1331_NORMALDISPLAY)                 # 0xA6
        self.command(SSD1331_SETMULTIPLEX)                  # 0xA8
        self.command(0x3F)
        self.command(SSD1331_SETMASTER)                     # 0xAD
        self.command(0x8E)
        self.command(SSD1331_POWERMODE)                     # 0xB0
        self.command(0x0B)                                  #
        self.command(SSD1331_SETPRECHARGE)                  # 0xB1
        self.command(0x31)                                  #
        self.command(SSD1331_SETDISPLAYCLOCKDIV)            # 0xB3
        self.command(0xF0)      # 7:4=Osc. Freq., 3:0=CLK Div Ratiio (A[3:0]+1=1..16)
        self.command(SSD1331_SETPRECHARGEA)                 # 0x8A
        self.command(0x64)                                  #
        self.command(SSD1331_SETPRECHARGEB)                 # 0x8B
        self.command(0x78)                                  #
        self.command(SSD1331_SETPRECHARGEC)                 # 0x8C
        self.command(0x64)                                  #
        self.command(SSD1331_SETPRECHARGELEVEL)             # 0xBB
        self.command(0x3A)                                  #
        self.command(SSD1331_SETVCOMH)                      # 0xBE
        self.command(0x3E)                                  #
        self.command(SSD1331_SETMASTERCURRENT)              # 0x87
        self.command(0x06)                                  #
        self.command(SSD1331_SETCONTRASTA)                  # 0x81
        self.command(0x91)
        self.command(SSD1331_SETCONTRASTB)                  # 0x82
        self.command(0x50)
        self.command(SSD1331_SETCONTRASTC)                  # 0x83
        self.command(0x7D)

