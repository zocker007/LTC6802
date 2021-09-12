/**
 * Copyright 2017, 2019 Dipl.-Inform. Kai Hofmann
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef LTC6802_H_INCLUDED_
  #define LTC6802_H_INCLUDED_

  #include <Arduino.h>

  #include <array>
  #include <bitset>

  // 57./58./59. namespace?
  // 72./73./74./75. exceptions
  // 68. assert

  /**
   * Bitsets for DCC and MCI bits. One bit for each cell from bit 0 (Cell1) to 11 (Cell12)
   */
  using DCCBitset = std::bitset<12>;
  using MCIBitset = std::bitset<12>;

  /**
   * LTC6802-2 multicell addressable battery stack monitor value class.
   *
   * https://cds.linear.com/docs/en/datasheet/68022fa.pdf
   */
  class LTC6802
   {
    public:
      /**
       * Enum for Comparator Duty Cycle settings
       */
      enum CDCValues : byte {
        STANDBY = 0,
        COMP_OFF_MEAS_13MS = 1,
        COMP_13MS_MEAS_13MS = 2,
        COMP_130MS_MEAS_13MS = 3,
        COMP_500MS_MEAS_13MS = 4,
        COMP_130MS_MEAS_21MS = 5,
        COMP_500MS_MEAS_21MS = 6,
        COMP_2000MS_MEAS_21MS = 7,
      };

      /**
       * Number of maximum cells connected to LTC6802.
       */
      static constexpr byte maxCells = 12;
      
      /**
       * Init SPI bus for LTC6802 chips.
       *
       * Call only one time, and not for each chip!
       *
       * @param pinMOSI Pin for master out slave in
       * @param pinMISO Pin for master in slave out
       * @param pinCLK Pin for clock
       */
      static void initSPI(byte pinMOSI = MOSI, byte pinMISO = MISO, byte pinCLK = SCK);

      /**
       * Destroy SPI bus.
       *
       * Call only one time, and not for each chip!
       */
      static void destroySPI();


      /**
       * Constructor.
       *
       * @param address Chip address on SPI bus
       * @param csPin Chip select pin
       */
      explicit LTC6802(byte address, byte csPin);

      // ~LTC6802();

      /**
       * Read configuration from chip registers.
       */
      void cfgRead(const bool broadcast);

      /**
       * Write configuration to chip registers.
       *
       * @param broadcast Send as broadcast
       */
      void cfgWrite(bool broadcast) const;

      /**
       * Write configuration to serial.
       *
       * @Deprecated  Don't use anymore
       */
      void cfgDebugOutput() const;

      /**
       * Get Watchdog timer flag from configuration.
       *
       * @return 0 : WDTB pin low; 1 : WDTB pin high
       */
      bool cfgGetWDT() const;

      /**
       * Get GPIO1 pin control from configuration.
       *
       * @return 0 : GPIO1 pin low; 1 : GPIO1 pin high
       */
      bool cfgGetGPIO1() const;

      /**
       * Set GPIO1 pin control in configuration.
       *
       * @param gpio 0 : GPIO1 pin pull down on; 1 : GPIO1 pin pull down off (default)
       */
      void cfgSetGPIO1(bool gpio);

      /**
       * Get GPIO2 pin control from configuration.
       *
       * @return 0 : GPIO2 pin low; 1 : GPIO2 pin high
       */
      bool cfgGetGPIO2() const;

      /**
       * Set GPIO2 pin control in configuration.
       *
       * @param gpio 0 : GPIO2 pin pull down on; 1 : GPIO2 pin pull down off (default)
       */
      void cfgSetGPIO2(bool gpio);

      /**
       * Get CELL10 bit from configuration.
       *
       * @return 0 : 12-cell mode (default); 1 :10-cell mode
       */
      bool cfgGetCELL10() const;

      /**
       * Set cell10 bit: If the CELL10 bit is high, then the inputs for cells 11 and 12 are automatically masked.
       *
       * @param cell10 1 : Cell10 on; 0 : cell10 off (default)
       */
      void cfgSetCELL10(bool cell10);

      /**
       * Get level polling mode from configuration.
       *
       * @return 0 : toggle polling mode (default); 1 : level polling
       */
      bool cfgGetLVLPL() const;

      /**
       * Set level polling mode in configuration.
       *
       * @param lvlpl 0 : toggle polling mode (default); 1 : level polling
       */
      void cfgSetLVLPL(bool lvlpl);

      /**
       * Get comparator duty cycle from configuration.
       *
       * @return 0 : Standby mode; 1 : Measure mode comparator off; 2 : Comparator 13ms; 3 : Comparator 130ms; 4 : Comparator 500ms; 5: Comparator 130ms with power down; 6 : Comparator 500ms with power down; 7 : Comparator 2000ms with power down
       */
      CDCValues cfgGetCDC() const;

      /**
       * Set comparator duty cycle in configuration.
       *
       * @param cdc 0 : Standby mode; 1 : Measure mode comparator off; 2 : Comparator 13ms; 3 : Comparator 130ms; 4 : Comparator 500ms; 5: Comparator 130ms with power down; 6 : Comparator 500ms with power down; 7 : Comparator 2000ms with power down
       */
      void cfgSetCDC(CDCValues cdc);

      /**
       * Get discharge cell flags from configuration.
       *
       * @return bit 0-11: 0: turn off shorting switch for cell (default); 1: turn on shorting switch
       */
      std::bitset<12> cfgGetDCC() const;

      /**
       * Set discharge cell flags in configuration.
       *
       * @param dcc bit 0-11: 0: turn off shorting switch for cell (default); 1: turn on shorting switch
       */
      void cfgSetDCC(std::bitset<12> dcc);

      /**
       * Get mask cell interrupt flags from configuration.
       *
       * @return bit 0-11: 0: enable interrupt value for cell (default); 1: turn off interrupts and clear flags for cell
       */
      std::bitset<12> cfgGetMCI() const;

      /**
       * Set mask cell interrupt flags in configuration.
       *
       * @param mci bit 0-11: 0: enable interrupt value for cell (default); 1: turn off interrupts and clear flags for cell
       */
      void cfgSetMCI(std::bitset<12> mci);

      /**
       * Get undervoltage comparison voltage from configuration.
       *
       * @return Voltage (default: 0 or factory programmed)
       */
      float cfgGetVUV() const;

      /**
       * Set undervoltage comparison voltage in configuration.
       *
       * @param vuv Voltage
       */
      void cfgSetVUV(float vuv);

      /**
       * Get overvoltage comparison voltage from configuration.
       *
       * @return Voltage (default: 0 or factory programmed)
       */
      float cfgGetVOV() const;

      /**
       * Set overvoltage comparison voltage in configuration.
       *
       * @param vov Voltage
       */
      void cfgSetVOV(float vov);

      /**
       * Measure temperatures on chip.
       */
      void temperatureMeasure(const bool broadcast);

      /**
       * Read temperatures from chip.
       */
      void temperatureRead(const bool broadcast);

      /**
       * Write temperatures to serial.
       *
       * @Deprecated Don't use anymore
       */
      void temperatureDebugOutput() const;

      /**
       * Measure cell voltages on chip.
       */
      void cellsMeasure(const bool broadcast);

      /**
       * Read cell voltages from chip.
       */
      void cellsRead(const bool broadcast);

      /**
       * Write cell voltages to serial.
       *
       * @Deprecated Don't use anymore
       */
      void cellsDebugOutput() const;

      /**
       * Read flag register group from chip.
       */
      void flagsRead(const bool broadcast);

      /**
       * Write flag register group to serial.
       *
       * @Deprecated Don't use anymore
       */
      void flagsDebugOutput();

      /**
       * Poll for ADC operation complete 
       * @return busy status, returns false when complete.
       */
      bool pollADC(const bool broadcast) const;

      /**
       * expose array of cell voltage data
       * 
       * @param cellvolts
       */
      void getVolts(std::array<word, maxCells> &cellvolts) const;
      
      // bool operator==(const LTC6802& obj1, const LTC6802& obj2);
      // bool operator!=(const LTC6802& obj1, const LTC6802& obj2);

    protected:
      // Disable heap allocation
      static void *operator new (size_t) throw() {return (0);}
      static void operator delete (void *) throw() {}

    private:

      enum BitMasks {
        /**
         * Configuration register 0 watchdog timer bitmask.
         */
        CFG0_WDT_MSK    = 0x80,

        /**
         * Configuration register 0 GPIO2 bitmask.
         */
        CFG0_GPIO2_MSK  = 0x40,

        /**
         * Configuration register 0 GPIO1 bitmask.
         */
        CFG0_GPIO1_MSK  = 0x20,

        /**
         * Configuration register 0 level polling bitmask.
         */
        CFG0_LVLPL_MSK  = 0x10,

        /**
         * Configuration register 0 10-cell mode bitmask.
         */
        CFG0_CELL10_MSK = 0x08,

        /**
         * Configuration register 0 comparator duty cycle bitmask.
         */
        CFG0_CDC_MSK    = 0x07,

        /**
         * Configuration register 1 discharge cell bitmask.
         */
        CFG1_DCC_MSK    = 0xff,

        /**
         * Configuration register 2 discharge cell bitmask.
         */
        CFG2_DCC_MSK    = 0x0f,

        /**
         * Configuration register 2 mask cell interrupts bitmask.
         */
        CFG2_MCI_MSK    = 0xf0,

        /**
         * Configuration register 3 mask cell interrupts bitmask.
         */
        CFG3_MCI_MSK    = 0xff,

        /**
         * Configuration register 0 watchdog timer inverse bitmask.
         */
        CFG0_WDT_INVMSK    = 0x7f,

        /**
         * Configuration register 0 GPIO2 inverse bitmask.
         */
        CFG0_GPIO2_INVMSK  = 0xbf,

        /**
         * Configuration register 0 GPIO1 inverse bitmask.
         */
        CFG0_GPIO1_INVMSK  = 0xdf,

        /**
         * Configuration register 0 level polling mode inverse bitmask.
         */
        CFG0_LVLPL_INVMSK  = 0xef,

        /**
         * Configuration register 0 10-cell mode inverse bitmask.
         */
        CFG0_CELL10_INVMSK = 0xf7,

        /**
         * Configuration register 0 comparator duty cycle inverse bitmask.
         */
        CFG0_CDC_INVMSK    = 0xf8,

        /**
         * Configuration register 1 discharge cell inverse bitmask.
         */
        CFG1_DCC_INVMSK    = 0x00,

        /**
         * Configuration register 2 discharge cell inverse bitmask.
         */
        CFG2_DCC_INVMSK    = 0xf0,

        /**
         * Configuration register 2 mask cell interrupts inverse bitmask.
         */
        CFG2_MCI_INVMSK    = 0x0f,

        /**
         * Configuration register 3 mask cell interrupts inverse bitmask.
         */
        CFG3_MCI_INVMSK    = 0x00,
      };

      enum Commands : byte {
        /**
         * Write configuration register group.
         */
        WRCFG   = 0x01,

        /**
         * Read configuration register group.
         */
        RDCFG   = 0x02,

        /**
         * Read cellvoltage register group.
         */
        RDCV    = 0x04,

        /**
         * Read flag register group.
         */
        RDFLG   = 0x06,

        /**
         * Read temperature register group.
         */
        RDTMP   = 0x08,

        /**
         * Start cell voltage A/D conversion and poll status.
         *
         * | 0x00 : all cell voltage inputs
         * | 0x01 : cell 1 only
         * | 0x02 : cell 2 only
         * | 0x03 : cell 3 only
         * | 0x04 : cell 4 only
         * | 0x05 : cell 5 only
         * | 0x06 : cell 6 only
         * | 0x07 : cell 7 only
         * | 0x08 : cell 8 only
         * | 0x09 : cell 9 only
         * | 0x0a : cell 10 only
         * | 0x0b : cell 11 only, if CELL10 bit=0
         * | 0x0c : cell 12 only, if CELL10 bit=0
         * | 0x0e : cell self test 1; all CV=0x555
         * | 0x0f : cell self test 2; all CV=0xaaa
         */
        STCVAD  = 0x10,

        /**
         * Start Open-Wire A/D conversion and poll status.
         *
         * | 0x00 : all cell voltage inputs
         * | 0x01 : cell 1 only
         * | 0x02 : cell 2 only
         * | 0x03 : cell 3 only
         * | 0x04 : cell 4 only
         * | 0x05 : cell 5 only
         * | 0x06 : cell 6 only
         * | 0x07 : cell 7 only
         * | 0x08 : cell 8 only
         * | 0x09 : cell 9 only
         * | 0x0a : cell 10 only
         * | 0x0b : cell 11 only, if CELL10 bit=0
         * | 0x0c : cell 12 only, if CELL10 bit=0
         * | 0x0e : cell self test 1; all CV=0x555
         * | 0x0f : cell self test 2; all CV=0xaaa
         */
        STOWAD  = 0x20,

        /**
         * Start temperature A/D conversion and poll status.
         *
         * | 0x00 : all temperature inputs
         * | 0x01 : external temp 1 only
         * | 0x02 : external temp 2 only
         * | 0x03 : internal temp only
         * | 0x0e : temp self test 1; all TMP=0x555
         * | 0x0f : temp self test 2; all TMP=0xaaa
         */
        STTMPAD = 0x30,

        /**
         * Poll A/D convertr status.
         */
        PLADC   = 0x40,

        /**
         * Poll interrupt status.
         */
        PLINT   = 0x50,

        /**
         * Start cell voltage A/D conversion and poll status, with discharge permitted.
         *
         * | 0x00 : all cell voltage inputs
         * | 0x01 : cell 1 only
         * | 0x02 : cell 2 only
         * | 0x03 : cell 3 only
         * | 0x04 : cell 4 only
         * | 0x05 : cell 5 only
         * | 0x06 : cell 6 only
         * | 0x07 : cell 7 only
         * | 0x08 : cell 8 only
         * | 0x09 : cell 9 only
         * | 0x0a : cell 10 only
         * | 0x0b : cell 11 only, if CELL10 bit=0
         * | 0x0c : cell 12 only, if CELL10 bit=0
         * | 0x0e : cell self test 1; all CV=0x555
         * | 0x0f : cell self test 2; all CV=0xaaa
         */
        STCDC   = 0x60,

        /**
         * Start open-Wire A/D conversions and poll status, with Discharge Permitted.
         *
         * | 0x00 : all cell voltage inputs
         * | 0x01 : cell 1 only
         * | 0x02 : cell 2 only
         * | 0x03 : cell 3 only
         * | 0x04 : cell 4 only
         * | 0x05 : cell 5 only
         * | 0x06 : cell 6 only
         * | 0x07 : cell 7 only
         * | 0x08 : cell 8 only
         * | 0x09 : cell 9 only
         * | 0x0a : cell 10 only
         * | 0x0b : cell 11 only, if CELL10 bit=0
         * | 0x0c : cell 12 only, if CELL10 bit=0
         * | 0x0e : cell self test 1; all CV=0x555
         * | 0x0f : cell self test 2; all CV=0xaaa
         */
        STOWDC  = 0x70,
      };

      enum CfgBits {
        /**
         * Configuration register 0 watchdog timer bit.
         */
        CFG0_WDT_Pos    = 7,

        /**
         * Configuration register 0 GPIO2 bit.
         */
        CFG0_GPIO2_Pos  = 6,

        /**
         * Configuration register 0 GPIO1 bit.
         */
        CFG0_GPIO1_Pos  = 5,

        /**
         * Configuration register 0 level polling mode bit.
         */
        CFG0_LVLPL_Pos  = 4,

        /**
         * Configuration register 0 10-cell mode bit.
         */
        CFG0_CELL10_Pos = 3,

        // CFG0_CDC_BITS   = 0-2,
        // CFG1_DCC_BITS   = 0-7,
        // CFG2_DCC_BITS   = 0-3,
        // CFG2_MCI_BITS   = 4-7,
        // CFG3_MCI_BITS   = 0-7,
      };

      /**
       * Enum for register indices in register map arrays
       */
      enum RegNames {
        CFGR0 = 0,
        CFGR1 = 1,
        CFGR2 = 2,
        CFGR3 = 3,
        CFGR4 = 4,
        CFGR5 = 5,

        TMPR0 = 0,
        TMPR1 = 1,
        TMPR2 = 2,
        TMPR3 = 3,
        TMPR4 = 4,

        CVR00 = 0,
        CVR01 = 1,
        CVR02 = 2,
        CVR03 = 3,
        CVR04 = 4,
        CVR05 = 5,
        CVR06 = 6,
        CVR07 = 7,
        CVR08 = 8,
        CVR09 = 9,
        CVR10 = 10,
        CVR11 = 11,
        CVR12 = 12,
        CVR13 = 13,
        CVR14 = 14,
        CVR15 = 15,
        CVR16 = 16,
        CVR17 = 17,

        FLGR0 = 0,
        FLGR1 = 1,
        FLGR2 = 2,
      };

      enum NumberOfRegs {
        /**
         * Number of LTC6802 configuration registers.
         */
        NUMCFGR = 6,

        /**
         * Number of LTC6802 temperature registers.
         */
        NUMTMPR = 5,

        /**
         * Number of LTC6802 flag registers.
         */
        NUMFLGR = 3,

        /**
         * Number of LTC6802 cell registers.
         */
        NUMCVR  = 18
      };

      struct Registers {
        /**
         * Configuration register group.
         */
        std::array<byte, NUMCFGR> CFGRx;

        /**
         * Tempertaure register group.
         */
        std::array<byte, NUMTMPR> TMPRx;

        /**
         * Cell voltage register group.
         */
        std::array<byte, NUMCVR> CVRxx;

        /**
         * Flag register group.
         */
        std::array<byte, NUMFLGR> FLGRx;

        /**
         * Packet Error Code register.
         */
        byte PEC;
      };

      /**
       * Chip SPI address.
       */
      byte address;

      /**
       * Chip select pin.
       */
      byte csPin;

      /**
       * Register map
       */
      Registers regs;

      // Disable array heap allocation
      static void *operator new[] (size_t);
      static void operator delete[] (void *);

      /**
       * Read registers from chip.
       *
       * @param cmd Read command.
       * @param numOfRegisters Number of registers to read
       * @param arr Array for register values
       */
      template<std::size_t N>
      void read(const Commands cmd, std::array<byte, N> &arr, const bool broadcast);

      /**
       * Send measure command to chip.
       *
       * @param cmd Chip command
       * @param broadcast Send as broadcast to multiple chips
       */
      void measure(const Commands cmd, bool broadcast) const;

      /**
       * Read register values from chip.
       *
       * @param cmd Read command.
       * @param numOfRegisters Number of registers to read
       * @param arr Array for register values
       */
      template<std::size_t N>
      void readValues(const Commands cmd, std::array<byte, N> &arr, const bool broadcast);

   };

#endif
