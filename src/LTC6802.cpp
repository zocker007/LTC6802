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
#include <LTC6802.h>
#include <SPI.h>

void LTC6802::initSPI(const byte pinMOSI, const byte pinMISO, const byte pinCLK)
 {
  // TODO parameters for different arduinos (pins, clock)
  pinMode(pinMOSI, OUTPUT);
  pinMode(pinMISO, INPUT);
  pinMode(pinCLK, OUTPUT);

  // SPI.setBitOrder(MSBFIRST);
  // SPI.setDataMode(SPI_MODE3);
  // SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();

  // SPI.begin(ETHERNET_SHIELD_SPI_CS);
  // SPI.setClockDivider(ETHERNET_SHIELD_SPI_CS, SPI_CLOCK_DIV16);
  // SPI.setDataMode(ETHERNET_SHIELD_SPI_CS, SPI_MODE3);
 }


void LTC6802::destroySPI()
 {
  SPI.end();
 }


LTC6802::LTC6802(const byte address, const byte csPin)
 : address(address), csPin(csPin), regs({})
 {
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
 }


/*
LTC6802::~LTC6802()
 {
 }
*/


void LTC6802::measure(const LTC6802::Commands cmd, const bool broadcast) const
 {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(csPin, LOW);
  if (!broadcast)
   {
    SPI.transfer(this->address);
   }
  SPI.transfer(static_cast<byte>(cmd));

  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  // check SDO for measure finished
 }

template<std::size_t N>
void LTC6802::read(const LTC6802::Commands cmd, std::array<byte, N> &arr, const bool broadcast) // TODO eliminate buffer overflow risk
  {
   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
   digitalWrite(csPin, LOW);
   if (!broadcast)
    {
     SPI.transfer(this->address);
    }
    
   SPI.transfer(static_cast<byte>(cmd));

   for (auto &element : arr)
    {
     element = SPI.transfer(static_cast<byte>(cmd));
    }
    
   /* byte pec = */ SPI.transfer(static_cast<byte>(cmd));

   digitalWrite(csPin, HIGH);
   SPI.endTransaction();
  }

void LTC6802::flagsRead(const bool broadcast)
  {
   read(RDFLG, this->regs.FLGRx, broadcast);
  }


 void LTC6802::flagsDebugOutput()
  {
   for (const auto &flag : this->regs.FLGRx)
    {
     Serial.print(flag, HEX);
     Serial.print(" ");
    }
   Serial.println();
  }


void LTC6802::cfgRead(const bool broadcast)
 {
  read(RDCFG, this->regs.CFGRx, broadcast);
 }

bool LTC6802::pollADC(const bool broadcast) const
{
  bool busy;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(csPin, LOW);
  if (!broadcast)
   {
    SPI.transfer(this->address);
   }
  SPI.transfer(static_cast<byte>(PLADC));
  busy = SPI.transfer(static_cast<byte>(PLADC));
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return busy;
}

void LTC6802::cfgWrite(const bool broadcast) const
 {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(csPin, LOW);
  if (!broadcast)
   {
    SPI.transfer(this->address);
   }
  SPI.transfer(WRCFG);
  for (const auto &reg : this->regs.CFGRx)
   {
    SPI.transfer(reg);
   }
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
 }


void LTC6802::cfgDebugOutput() const
 {
  Serial.print("WDT GPIO2/1 LVLPL Cell10 CDC: ");
  Serial.println(regs.CFGRx[CFGR0], BIN); // WDT  GPIO2 GPIO1  LVLPL  Cell10  CDC[2] CDC[1] CDC[0]

  word dcc = regs.CFGRx[CFGR1]; // DCC8 DCC7 DCC6 DCC5 DCC4 DCC3 DCC DCC1
  dcc |= (regs.CFGRx[CFGR2] & 0x000F) << 8; // MC4I MC3I MC2I MC1I   DCC12 DCC11 DCC10 DCC9
  Serial.print("DCC: ");
  Serial.println(dcc, HEX);

  word mci = regs.CFGRx[CFGR3] << 4; // MC12I MC11IMC10I MC9I MC8I MC7I MC6I MC5I
  mci |= ((regs.CFGRx[CFGR2] & 0xF0) >> 4); // MC4I MC3I MC2I MC1I   DCC12 DCC11 DCC10 DCC9
  Serial.print("MC I: ");
  Serial.println(mci, HEX);

  Serial.print("VUV: ");
  Serial.println(regs.CFGRx[CFGR4], HEX); // VUV 7-0

  Serial.print("VOV: ");
  Serial.println(regs.CFGRx[CFGR5], HEX); // VOV 7-0
 }


bool LTC6802::cfgGetWDT() const
 {
  return (regs.CFGRx[CFGR0] & CFG0_WDT_MSK);
 }


bool LTC6802::cfgGetGPIO1() const
 {
  return (regs.CFGRx[CFGR0] & CFG0_GPIO1_MSK);
 }


void LTC6802::cfgSetGPIO1(const bool gpio)
 {
  regs.CFGRx[CFGR0] = (regs.CFGRx[CFGR0] & CFG0_GPIO1_INVMSK) | (gpio << CFG0_GPIO1_Pos);
 }


bool LTC6802::cfgGetGPIO2() const
 {
  return (regs.CFGRx[CFGR0] & CFG0_GPIO2_MSK);
 }


void LTC6802::cfgSetGPIO2(const bool gpio)
 {
  regs.CFGRx[CFGR0] = (regs.CFGRx[CFGR0] & CFG0_GPIO2_INVMSK) | (gpio << CFG0_GPIO2_Pos);
 }

bool LTC6802::cfgGetCELL10() const
 {
  return (regs.CFGRx[CFGR0] & CFG0_CELL10_MSK);
 }


void LTC6802::cfgSetCELL10(const bool cell10)
 {
  regs.CFGRx[CFGR0] = (regs.CFGRx[CFGR0] & CFG0_CELL10_INVMSK) | (cell10 << CFG0_CELL10_Pos);
 }

bool LTC6802::cfgGetLVLPL() const
 {
  return (regs.CFGRx[CFGR0] & CFG0_LVLPL_MSK);
 }


void LTC6802::cfgSetLVLPL(const bool lvlpl)
 {
  regs.CFGRx[CFGR0] = (regs.CFGRx[CFGR0] & CFG0_LVLPL_INVMSK) | (lvlpl << CFG0_LVLPL_Pos);
 }


LTC6802::CDCValues LTC6802::cfgGetCDC() const
 {
  return static_cast<CDCValues>(regs.CFGRx[CFGR0] & CFG0_CDC_MSK);
 }


void LTC6802::cfgSetCDC(const LTC6802::CDCValues cdc)
 {
  // assert cdc 0-7
  regs.CFGRx[CFGR0] = (regs.CFGRx[CFGR0] & CFG0_CDC_INVMSK) | static_cast<byte>(cdc);
 }


DCCBitset LTC6802::cfgGetDCC() const
 {
  return DCCBitset{static_cast<unsigned long long>((regs.CFGRx[CFGR1] & CFG1_DCC_MSK) | ((regs.CFGRx[CFGR2] & CFG2_DCC_MSK) << 8))};
 }


void LTC6802::cfgSetDCC(const DCCBitset dcc)
 {
  // assert 0x0fff
  regs.CFGRx[CFGR1] = (dcc.to_ulong() & 0x00ff); // (regs.CFGRx[CFGR1] & CFG1_DCC_INVMSK) |
  regs.CFGRx[CFGR2] = (regs.CFGRx[CFGR2] & CFG2_DCC_INVMSK) | ((dcc.to_ulong() & 0x0f00) >> 8);
 }


MCIBitset LTC6802::cfgGetMCI() const
 {
  return MCIBitset{static_cast<unsigned long long>((regs.CFGRx[CFGR3] << 4) | ((regs.CFGRx[CFGR2] & CFG2_MCI_MSK) >> 4))};
 }


void LTC6802::cfgSetMCI(const MCIBitset mci)
 {
  // assert 0x0fff
  regs.CFGRx[CFGR2] = (regs.CFGRx[CFGR2] & CFG2_MCI_INVMSK) | ((mci.to_ulong() & 0x0f) << 4);
  regs.CFGRx[CFGR3] = (mci.to_ulong() & 0x0ff0) >> 4;
 }


float LTC6802::cfgGetVUV() const // TODO float vs double
 {
  return (regs.CFGRx[CFGR4] * 16.0f * 0.0015f);
 }


void LTC6802::cfgSetVUV(const float vuv) // TODO float vs double
 {
  regs.CFGRx[CFGR4] = static_cast<byte>(vuv / (0.0015f * 16.0f));
 }


float LTC6802::cfgGetVOV() const // TODO float vs double
 {
  return (regs.CFGRx[CFGR5] * 16.0f * 0.0015f);
 }


void LTC6802::cfgSetVOV(const float vov) // TODO float vs double
 {
  regs.CFGRx[CFGR5] = static_cast<byte>(vov / (0.0015f * 16.0f));
 }


void LTC6802::temperatureMeasure(const bool broadcast)
 {
  measure(STTMPAD, broadcast);
 }


void LTC6802::temperatureRead(const bool broadcast)
 {
  read(RDTMP, this->regs.TMPRx, broadcast);
 }


void LTC6802::temperatureDebugOutput() const
 {
  /*
  word etmp1 = regs.TMPRx[TMPR0]; // ETMP1
  etmp1 |= (regs.TMPRx[TMPR1] & 0x0f) << 8; // ETMP2 ETMP1
  Serial.print("ETMP1: ");
  Serial.println(etmp1, HEX);
  */

  /*
  word etmp2 = regs.TMPRx[TMPR2] << 4; // ETMP2
  etmp2 |= (etmp1 & 0xf0) >> 4; // ETMP2 ETMP1
  Serial.print("ETMP2: ");
  Serial.println(etmp2, HEX);
  */

  word itmp = regs.TMPRx[TMPR3];
  itmp |= (regs.TMPRx[TMPR4] & 0x0f) << 8; // REV  THSD  ITMP

  int cel = (itmp * 1.5 / 8) - 273;
  Serial.print("iCelsius: ");
  Serial.println(cel);

  Serial.print("THSD: ");
  Serial.println((regs.TMPRx[TMPR4] >> 4) & 0x01, HEX); // REV  THSD  ITMP

  Serial.print("REV: ");
  Serial.println(regs.TMPRx[TMPR4] >> 5, HEX); // REV  THSD  ITMP
 }


void LTC6802::cellsMeasure(const bool broadcast)
 {
  measure(STCVAD, broadcast);
 }


void LTC6802::cellsRead(const bool broadcast)
 {
  read(RDCV, this->regs.CVRxx, broadcast);
 }

void LTC6802::getVolts(std::array<word, maxCells> &cellvolts) const
 {
  cellvolts[0] = regs.CVRxx[CVR00] | ((regs.CVRxx[CVR01] & 0x0F) << 8);
  cellvolts[1] = ((regs.CVRxx[CVR01] & 0xf0) >> 4) | (regs.CVRxx[CVR02] << 4);

  cellvolts[2] = regs.CVRxx[CVR03] | ((regs.CVRxx[CVR04] & 0x0F) << 8);
  cellvolts[3] = ((regs.CVRxx[CVR04] & 0xf0) >> 4) | (regs.CVRxx[CVR05] << 4);

  cellvolts[4] = regs.CVRxx[CVR06] | ((regs.CVRxx[CVR07] & 0x0F) << 8);
  cellvolts[5] = ((regs.CVRxx[CVR07] & 0xf0) >> 4) | (regs.CVRxx[CVR08] << 4);

  cellvolts[6] = regs.CVRxx[CVR09] | ((regs.CVRxx[CVR10] & 0x0F) << 8);
  cellvolts[7] = ((regs.CVRxx[CVR10] & 0xf0) >> 4) | (regs.CVRxx[CVR11] << 4);

  cellvolts[8] = regs.CVRxx[CVR12] | ((regs.CVRxx[CVR13] & 0x0F) << 8);
  cellvolts[9] = ((regs.CVRxx[CVR13] & 0xf0) >> 4) | (regs.CVRxx[CVR14] << 4);

  cellvolts[10] = regs.CVRxx[CVR15] | ((regs.CVRxx[CVR16] & 0x0F) << 8);
  cellvolts[11] = ((regs.CVRxx[CVR16] & 0xf0) >> 4) | (regs.CVRxx[CVR17] << 4);
 }

 void LTC6802::getTemps(std::array<word, 3> &temp) const
 {
   temp[0] = regs.TMPRx[TMPR0] |((regs.TMPRx[TMPR1] & 0x0f) << 8); // external temp1
   temp[1] = ((regs.TMPRx[TMPR1] & 0xf0) >> 4) | (regs.TMPRx[TMPR2] << 4); // external temp2
   temp[2] = regs.TMPRx[TMPR3] |((regs.TMPRx[TMPR4] & 0x0f) << 8); // internal temp
 }

 bool LTC6802::getTHSD() const
 {
   return (regs.TMPRx[TMPR4] >> 4) & 0x01;
 }

 word LTC6802::getChipRevision() const
 {
   return regs.TMPRx[TMPR4] >> 5;
 }

void LTC6802::cellsDebugOutput() const
 {
  word cellvolts[maxCells];
  cellvolts[0] = regs.CVRxx[CVR00] | ((regs.CVRxx[CVR01] & 0x0F) << 8);
  cellvolts[1] = ((regs.CVRxx[CVR01] & 0xf0) >> 4) | (regs.CVRxx[CVR02] << 4);

  cellvolts[2] = regs.CVRxx[CVR03] | ((regs.CVRxx[CVR04] & 0x0F) << 8);
  cellvolts[3] = ((regs.CVRxx[CVR04] & 0xf0) >> 4) | (regs.CVRxx[CVR05] << 4);

  cellvolts[4] = regs.CVRxx[CVR06] | ((regs.CVRxx[CVR07] & 0x0F) << 8);
  cellvolts[5] = ((regs.CVRxx[CVR07] & 0xf0) >> 4) | (regs.CVRxx[CVR08] << 4);

  cellvolts[6] = regs.CVRxx[CVR09] | ((regs.CVRxx[CVR10] & 0x0F) << 8);
  cellvolts[7] = ((regs.CVRxx[CVR10] & 0xf0) >> 4) | (regs.CVRxx[CVR11] << 4);

  cellvolts[8] = regs.CVRxx[CVR12] | ((regs.CVRxx[CVR13] & 0x0F) << 8);
  cellvolts[9] = ((regs.CVRxx[CVR13] & 0xf0) >> 4) | (regs.CVRxx[CVR14] << 4);

  cellvolts[10] = regs.CVRxx[CVR15] | ((regs.CVRxx[CVR16] & 0x0F) << 8);
  cellvolts[11] = ((regs.CVRxx[CVR16] & 0xf0) >> 4) | (regs.CVRxx[CVR17] << 4);

  Serial.print(cellvolts[0] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[1] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[2] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[3] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[4] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[5] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[6] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[7] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[8] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[9] * 1.5 / 1000);
  Serial.print(", ");
  Serial.print(cellvolts[10] * 1.5 / 1000);
  Serial.print(", ");
  Serial.println(cellvolts[11] * 1.5 / 1000);
 }
