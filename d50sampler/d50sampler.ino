#include <driver/i2s.h>
#include <MIDI.h>
#include "samples.h"
//#include <MIDIUSB.h>

//----------------MIDI SETUP BEGIN-----------
struct Serial2MIDISettings : public midi::DefaultSettings {
  static const long BaudRate = 31250;
  static const int8_t RxPin = 16;
  static const int8_t TxPin = 17;
};

MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial2, MIDI2, Serial2MIDISettings);
byte midichan = 1;
byte commandByte;
byte noteByte;
byte velocityByte;
byte localParameterByte;
//----------------MIDI SETUP END--------------

//----------------LCD Majd később-------------
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

//----------------DAC SETUP BEGIN-------------
// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0
#define I2S_DOUT 25
#define I2S_BCLK 27
#define I2S_LRC 26
#define IR_PIN 34
size_t i2s_bytes_write = 0;

void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = IR_PIN
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

// Define input buffer length
#define bufferLen 1024
int16_t sBuffer[bufferLen];

void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 2,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}
//----------------DAC SETUP END------------

//----------------------PROGRAM VARIABLES-----------
const byte polyphony = 10;
uint32_t freqmutato[4][polyphony];
uint32_t pich[4][polyphony];
byte generatornumber = 1;
uint32_t wavefreq[4][polyphony];
byte wavebias[4][polyphony];
uint32_t noteertek[4][256];
byte oldnoteByte[polyphony];
bool noteoff[polyphony];
bool loopsample[4] = { false, false, false, false };
uint16_t samplebegin[4] = { 64, 64, 64, 64 };
uint16_t sampleend[4] = { 10190, 10190, 10190, 10190 };
byte opmenuoldal = 0;
uint16_t samplesize[4];
uint16_t GLOBAL_TUNE = 894;
byte COARSE[4] = { 36, 36, 36, 36 };
byte FINE[4] = { 50, 50, 50, 50 };
float c[4] = {1001, 1001, 1001, 1001};
byte szorzo[4] = {1, 1, 1, 1};
byte LKeyShift = 0;
byte UKeyShift = 0;
byte volume[4] { 60, 0, 0, 0 };
byte generatorvolume[4][polyphony];
//reverb variable
int32_t bufferbe[2];
uint16_t delaybuffersize = 8192;
int16_t delaybuffer[8192];
int16_t delaybuffer2[8192];
uint16_t delaybufferindex = 0;
uint16_t delaybufferindex2 = 0;
byte delaystep = 0;
byte delay2step = 0;
byte delaytime = 1;
byte delay2time = 1;
byte reverblevel = 60;
byte reverbdiffusion = 22;
uint16_t reverbtime = delaybuffersize;
uint16_t reverbtime2 = delaybuffersize;
byte chorusLevelLeft = 10;
byte chorusLevelRight = 49;
uint16_t chorusbuffersize = 511;
uint16_t chorusbuffersize2 = 511;
int16_t const* genstartadress[4];
const byte LFOnumber = 8;
uint16_t const* LFOadress[LFOnumber];
int32_t tempbuffer1;
int32_t tempbuffer2;
int32_t tempbuffer3;
int32_t tempbuffer4;
byte ENV_L0 = 0;
byte ENV_T1[4] = { 125, 125, 125, 125 };
byte ENV_L1[4] = { 125, 125, 125, 125 };
byte ENV_T2[4] = { 1, 1, 1, 1 };
byte ENV_L2[4] = { 125, 125, 125, 125 };
byte ENV_L3[4] = { 125, 125, 125, 125 };
byte ENV_T3[4] = { 1, 1, 1, 1 };
byte ENV_LSUS[4] = { 125, 125, 125, 125 };
byte ENV_T4[4] = { 1, 1, 1, 1 };
byte ENV_T5[4] = { 1, 1, 1, 1 };
byte ENV_LEND[4] = { 0, 0, 0, 0 };
byte generatorstatus[4][polyphony];
byte TVAvolume[4][polyphony];
byte TVA[4] = {0, 0, 0, 0};
byte KEYFollow[4] = { 11, 11, 11, 11 };
byte LFOMode[4] = { 0, 0, 0, 0 };
byte PENVMode[4] = { 0, 0, 0, 0 };
byte BENDERMode[4] = { 0, 0, 0, 0 };
byte Waveform[4] = { 0, 1, 0, 1 };
byte PCMWaveNo[4] = { 12, 12, 12, 12 };
byte BiasPoint[4] = {64, 64, 64, 64};
byte BiasLevel[4] = {0, 0, 0, 0};
byte Bias[4][256];
uint32_t lfoarrayindex[LFOnumber] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t lfovalue[LFOnumber];
byte lfofreq[LFOnumber] = {22, 34, 22, 22, 22, 22, 22, 22};

//---------------------------TUNE----------------------------------
/*
  void notevaluesarraytest() {
  for (int i = 0; i < 256; i++) {
    Serial.print(String(noteertek[0][i]) + " ");
  }
  Serial.println();
  }
*/
void notebias() {
  for (int j = 0; j < 4; j++) {
    if (BiasPoint[j] < 64) {
      for (int i = 0; i < 128; i++)
      {
        if (i < BiasPoint[j]) {
          Bias[j][i] = BiasLevel[j] ;
        } else {
          Bias[j][i] = 12;
        }
      }
    }
    if (BiasPoint[j] >= 64) {
      for (int i = 0; i < 128; i++)
      {
        if (i < BiasPoint[j]) {
          Bias[j][i] = 12 ;
        } else {
          Bias[j][i] = BiasLevel[j];
        }
      }
    }
  }
}
void notetune() {
  float szorzo2 = 2;
  for (int j = 0; j < 4; j++) {
    switch (KEYFollow[j]) {
      case 0:  szorzo2 = 0.12; break;
      case 1:  szorzo2 = 0.5; break;
      case 2:  szorzo2 = 0.25; break;
      case 3:  szorzo2 = 1; break;
      case 4:  szorzo2 = 1.125; break;
      case 5:  szorzo2 = 1.25; break;
      case 6:  szorzo2 = 1.375; break;
      case 7:  szorzo2 = 1.5; break;
      case 8:  szorzo2 = 1.625; break;
      case 9:  szorzo2 = 1.75; break;
      case 10: szorzo2 = 1.875; break;
      case 11: szorzo2 = 2; break;
      case 12: szorzo2 = 2.25; break;
      case 13: szorzo2 = 2.5; break;
      case 14: szorzo2 = 4; break;
      case 15: szorzo2 = 3; break;
      case 16: szorzo2 = 5; break;
    }
    float TUNE_NOW = GLOBAL_TUNE + FINE[j];
    TUNE_NOW = TUNE_NOW * pow(szorzo2, COARSE[j] / 12.0);
    TUNE_NOW += ((pow(2, 13) / pow(szorzo2, 13)) - 1) * COARSE[j];
    float BASIC_TUNE[12];
    for (int i = 0; i < 12; i++) {

      BASIC_TUNE[i] = TUNE_NOW  * pow(szorzo2, i / 12.0);
    }
    for (int i = 0; i < 12; i++) {
      //Serial.print(String(BASIC_TUNE[i]) + " ");
    }
    //Serial.println();

    float okt = 1;
    for (int i = 0; i < 14; i++) {
      for (int k = 0; k < 12; k++) {
        noteertek [j][i * 12 + k] = BASIC_TUNE[k] * okt;
      }
      okt = okt * szorzo2;
    }
  }

  //notevaluesarraytest();
}

uint16_t sizes[36];
void maxsize() {

  sizes[0] = sizeof(marimba) >> 1;
  sizes[1] = sizeof(vibraphone) >> 1;
  sizes[2] = sizeof(xilophone1) >> 1;
  sizes[3] = sizeof(xilophone2) >> 1;
  sizes[4] = sizeof(logbass) >> 1;
  sizes[5] = sizeof(hammer) >> 1;
  sizes[6] = sizeof(japanesedrum) >> 1;
  sizes[7] = sizeof(kalimba) >> 1;
  sizes[8] = sizeof(pluck1) >> 1;
  sizes[9] = sizeof(chink) >> 1;
  sizes[10] = sizeof(agogo) >> 1;

  sizes[11] = sizeof(triangle) >> 1;

  sizes[12] = sizeof(bells) >> 1;

  sizes[13] = sizeof(nailfile) >> 1;
  sizes[14] = sizeof(pick) >> 1;
  sizes[15] = sizeof(lowpiano) >> 1;
  sizes[16] = sizeof(midpiano) >> 1;
  sizes[17] = sizeof(highpiano) >> 1;
  sizes[18] = sizeof(hapsichord) >> 1;
  sizes[19] = sizeof(harp) >> 1;
  sizes[20] = sizeof(organpercus) >> 1;
  sizes[21] = sizeof(steelstrings) >> 1;
  sizes[22] = sizeof(nylonstrings) >> 1;
  sizes[23] = sizeof(electgitar1) >> 1;
  sizes[24] = sizeof(electgitar2) >> 1;
  sizes[25] = sizeof(dirtygitar) >> 1;
  sizes[26] = sizeof(pickbass) >> 1;
  sizes[27] = sizeof(popbass) >> 1;
  sizes[28] = sizeof(thump) >> 1;
  sizes[29] = sizeof(klarinet) >> 1;
  sizes[30] = sizeof(breath) >> 1;
  sizes[31] = sizeof(klarinet) >> 1;
  sizes[32] = sizeof(steamer) >> 1;
  sizes[47] = sizeof(drawbarsloop) >> 1;
  sizes[48] = sizeof(highorganloop) >> 1;
  sizes[49] = sizeof(loworganloop) >> 1;
  sizes[50] = sizeof(electpiano1loop) >> 1;
  sizes[51] = sizeof(electpiano2loop) >> 1;
  sizes[52] = sizeof(claviloop) >> 1;
  sizes[53] = sizeof(hapsichordloop) >> 1;
  sizes[54] = sizeof(electbassloop1) >> 1;
  sizes[55] = sizeof(acusticbassloop) >> 1;
  sizes[56] = sizeof(electbassloop2) >> 1;
  sizes[57] = sizeof(violinloop) >> 1;

}

void setsamplesize() {
  //Set up max sample size
  samplesize[opmenuoldal] = sizes[PCMWaveNo[opmenuoldal]];
  if (samplesize[opmenuoldal] < sampleend[opmenuoldal]) {
    sampleend[opmenuoldal] = samplesize[opmenuoldal];
  }
  Serial.println("Size of sample:");
  Serial.println(String(samplesize[0]));
  Serial.println(String(samplesize[1]));
  Serial.println(String(samplesize[2]));
  Serial.println(String(samplesize[3]));
}

void setPCMWave() {
  switch (PCMWaveNo[opmenuoldal]) {

    case 0: genstartadress[opmenuoldal] = marimba; break;
    case 1: genstartadress[opmenuoldal] = vibraphone; break;
    case 2: genstartadress[opmenuoldal] = xilophone1; break;
    case 3: genstartadress[opmenuoldal] = xilophone2; break;
    case 4: genstartadress[opmenuoldal] = logbass; break;
    case 5: genstartadress[opmenuoldal] = hammer; break;
    case 6: genstartadress[opmenuoldal] = japanesedrum; break;
    case 7: genstartadress[opmenuoldal] = kalimba; break;
    case 8: genstartadress[opmenuoldal] = pluck1; break;
    case 9: genstartadress[opmenuoldal] = chink; break;
    case 10: genstartadress[opmenuoldal] = agogo; break;

    case 11: genstartadress[opmenuoldal] = triangle; break;

    case 12: genstartadress[opmenuoldal] = bells; break;

    case 13: genstartadress[opmenuoldal] = nailfile; break;
    case 14: genstartadress[opmenuoldal] = pick; break;
    case 15: genstartadress[opmenuoldal] = lowpiano; break;
    case 16: genstartadress[opmenuoldal] = midpiano; break;
    case 17: genstartadress[opmenuoldal] = highpiano; break;
    case 18: genstartadress[opmenuoldal] = hapsichord; break;
    case 19: genstartadress[opmenuoldal] = harp; break;
    case 20: genstartadress[opmenuoldal] = organpercus; break;
    case 21: genstartadress[opmenuoldal] = steelstrings; break;
    case 22: genstartadress[opmenuoldal] = nylonstrings; break;
    case 23: genstartadress[opmenuoldal] = electgitar1; break;
    case 24: genstartadress[opmenuoldal] = electgitar2; break;
    case 25: genstartadress[opmenuoldal] = dirtygitar; break;
    case 26: genstartadress[opmenuoldal] = pickbass; break;
    case 27: genstartadress[opmenuoldal] = popbass; break;
    case 28: genstartadress[opmenuoldal] = thump; break;
    case 29: genstartadress[opmenuoldal] = klarinet; break;
    case 30: genstartadress[opmenuoldal] = breath; break;
    case 31: genstartadress[opmenuoldal] = popbass; break;
    case 32: genstartadress[opmenuoldal] = steamer; break;
    case 47: genstartadress[opmenuoldal] = drawbarsloop; break;
    case 48: genstartadress[opmenuoldal] = highorganloop; break;
    case 49: genstartadress[opmenuoldal] = loworganloop; break;
    case 50: genstartadress[opmenuoldal] = electpiano1loop; break;
    case 51: genstartadress[opmenuoldal] = electpiano2loop; break;
    case 52: genstartadress[opmenuoldal] = claviloop; break;
    case 53: genstartadress[opmenuoldal] = hapsichordloop; break;
    case 54: genstartadress[opmenuoldal] = electbassloop1; break;
    case 55: genstartadress[opmenuoldal] = acusticbassloop; break;
    case 56: genstartadress[opmenuoldal] = electbassloop2; break;
    case 57: genstartadress[opmenuoldal] = violinloop; break;

  }
  Serial.println("PCMWave" + String(opmenuoldal) + "generator: " + String(PCMWaveNo[opmenuoldal]));
  setsamplesize();
}

void setLFOWave() {
  LFOadress[0] = lfosine;
  LFOadress[1] = lfotriangle;
  LFOadress[2] = lfosine;
  LFOadress[3] = lfotriangle;
  LFOadress[4] = lfosine;
  LFOadress[5] = lfotriangle;
  LFOadress[6] = lfosine;
  LFOadress[7] = lfotriangle;
}

//--------------MIDI SYSEX PARAMETER CONTROL------
void parametersysexchanged() {
  byte step = 1;
  byte value = velocityByte;
  if (localParameterByte == 0)
    switch (noteByte) {
      case 0:
        //couarse u1
        COARSE[2] = value;
        Serial.println("COARSE U1: " + String(COARSE[2]));
        notetune();
        break;
      case 1:
        //couarse u1
        FINE[2] = value;
        Serial.println("COARSE U1: " + String(FINE[2]));
        notetune();
        break;
      case 2:
        KEYFollow[2] = value;
        Serial.println("KEYFollow U1: " + String(KEYFollow[2]));
        notetune();
        break;
      case 3:

        break;
      case 4:
        TVA[2] = value;
        Serial.println("TVA U1: " + String(TVA[1]));
        break;
      case 7:
        PCMWaveNo[2] = value;
        Serial.println("PCMWaveNo U1: " + String(PCMWaveNo[2]));
        opmenuoldal = 2;
        setPCMWave();
        break;
      case 35:
        volume[2] = value;
        Serial.println("Level U1: " + String(volume[2]));
        break;
      case 37:
        BiasPoint[2] = value;
        Serial.println("BiasPoint U1: " + String(BiasPoint[2]));
        notebias();
        break;
      case 38:
        BiasLevel[2] = value;
        Serial.println("bieasLevel U1: " + String(BiasLevel[2]));
        notebias();
        break;
      case 39:
        ENV_T1[2] = value;
        Serial.println("ENV_T1 U1: " + String(ENV_T1[2]));
        break;
      case 40:
        ENV_T2[2] = value;
        Serial.println("ENV_T2 U1:" + String(ENV_T2[2]));
        break;
      case 41:
        ENV_T3[2] = value;
        Serial.println("ENV_T3 U1:" + String(ENV_T3[2]));
        break;
      case 42:
        ENV_T4[2] = value;
        Serial.println("ENV_T4 U1:" + String(ENV_T4[2]));
        break;
      case 43:
        step = samplesize[2] / 100;
        sampleend[2] = value * step;
        Serial.println("SAMPLE END U1: " + String(sampleend[2]));
        break;
      case 44:
        ENV_L1[2] = value;
        Serial.println("ENV_L1 U1:" + String(ENV_L1[2]));
        break;
      case 45:
        ENV_L2[2] = value;
        Serial.println("ENV_L2 U1:" + String(ENV_L2[2]));
        break;
      case 46:
        step = samplesize[2] / 100;
        samplebegin[2] = value * step;
        Serial.println("SAMPLE BEGIN U1: " + String(samplebegin[2]));
        break;
      case 47:
        ENV_LSUS[2] = value;
        Serial.println("ENV_LSUS U1: " + String(ENV_LSUS[2]));
        break;
      case 48:
        ENV_LEND[2] = value;
        Serial.println("ENV_LEND U1: " + String(ENV_LEND[2]));
        break;
      case 49:
        opmenuoldal = 2;
        if (value == 0) {

          loopsample[opmenuoldal] = false;
          Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
        }
        if (value == 1) {
          loopsample[opmenuoldal] = true;
          Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
        }
        break;
      case 64:
        //couarse u2
        COARSE[3] = value;
        Serial.println("COARSE U2:" + String(COARSE[3]));
        notetune();
        break;
      case 65:
        //couarse u2
        FINE[3] = value;
        Serial.println("COARSE U2:" + String(FINE[3]));
        notetune();
        break;
      case 66:
        KEYFollow[3] = value;
        Serial.println("KEYFollow U2:" + String(KEYFollow[3]));
        notetune();
        break;
      case 67:

        break;
      case 68:
        TVA[3] = value;
        Serial.println("TVA U2: " + String(TVA[3]));
        break;
      case 71:
        PCMWaveNo[3] = value;
        Serial.println("PCMWaveNo U2: " + String(PCMWaveNo[3]));
        opmenuoldal = 3;
        setPCMWave();
        break;
      case 99:
        volume[3] = value;
        Serial.println("Level U2: " + String(volume[3]));
        break;
      case 101:
        BiasPoint[3] = value;
        Serial.println("BiasPoint L2: " + String(BiasPoint[3]));
        notebias();
        break;
      case 102:
        BiasLevel[3] = value;
        Serial.println("bieasLevel L1: " + String(BiasLevel[3]));
        notebias();
        break;
      case 103:
        ENV_T1[3] = value;
        Serial.println("ENV_T1 U2: " + String(ENV_T1[3]));
        break;
      case 104:
        ENV_T2[3] = value;
        Serial.println("ENV_T2 U2:" + String(ENV_T2[3]));
        break;
      case 105:
        ENV_T3[3] = value;
        Serial.println("ENV_T3 U2:" + String(ENV_T3[3]));
        break;
      case 106:
        ENV_T4[3] = value;
        Serial.println("ENV_T4 U2:" + String(ENV_T4[3]));
        break;
      case 107:
        step = samplesize[3] / 100;
        sampleend[3] = value * step;
        Serial.println("SAMPLE END U2: " + String(sampleend[3]));
        break;
      case 108:
        ENV_L1[3] = value;
        Serial.println("ENV_L1 U2: " + String(ENV_L1[3]));
        break;
      case 109:
        ENV_L2[3] = value;
        Serial.println("ENV_L2 U2: " + String(ENV_L2[3]));
        break;
      case 110:
        step = samplesize[3] / 100;
        samplebegin[3] = value * step;
        Serial.println("SAMPLE BEGIN L1: " + String(samplebegin[3]));
        break;
      case 111:
        ENV_LSUS[3] = value;
        Serial.println("ENV_LSUS U2" + String(ENV_LSUS[3]));
        break;
      case 112:
        ENV_LEND[3] = value;
        Serial.println("ENV_LEND U2: " + String(ENV_LEND[3]));
        break;
      case 113:
        opmenuoldal = 3;
        if (value == 0) {

          loopsample[opmenuoldal] = false;
          Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
        }
        if (value == 1) {
          loopsample[opmenuoldal] = true;
          Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
        }
        break;
    }

  if (localParameterByte == 1)
    switch (noteByte) {

      case 1:

        break;

      case 3:
        LFOMode[opmenuoldal] = value;
        break;
      case 4:
        PENVMode[opmenuoldal] = value;
        break;
      case 5:
        BENDERMode[opmenuoldal] = value;
        break;
      case 6:

        break;
      case 43:
        lfofreq[1] = value;
        break;
      case 44:
        chorusLevelRight = value;
        break;
      case 64:
        COARSE[0] = value;
        Serial.println("COARSE L1: " + String(COARSE[0]));
        notetune();
        break;
      case 65:
        FINE[0] = value;
        Serial.println("FINE L1: " + String(FINE[0]));
        notetune();
        break;
      case 66:
        KEYFollow[0] = value;
        Serial.println("KEYFollow L1: " + String(KEYFollow[0]));
        notetune();

        break;
      case 67:
        TVA[0] = value;
        Serial.println("TVA L1: " + String(TVA[0]));
        break;
      case 71:
        PCMWaveNo[0] = value;
        Serial.println("PCMWaveNo" + String(0) + ": " + String(PCMWaveNo[0]));
        opmenuoldal = 0;
        setPCMWave();
        break;
      case 99:
        volume[0] = value;
        Serial.println("Level L1: " + String(volume[0]));
        break;
      case 101:
        BiasPoint[0] = value;
        Serial.println("BiasPoint L1: " + String(BiasPoint[0]));
        notebias();
        break;
      case 102:
        BiasLevel[0] = value;
        Serial.println("bieasLevel L1: " + String(BiasLevel[0]));
        notebias();
        break;
      case 103:
        ENV_T1[0] = value;
        Serial.println("ENV_T1 L1: " + String(ENV_T1[0]));
        break;
      case 104:
        ENV_T2[0] = value;
        Serial.println("ENV_T2 L1:" + String(ENV_T2[0]));
        break;
      case 105:
        ENV_T3[0] = value;
        Serial.println("ENV_T3 L1:" + String(ENV_T3[0]));
        break;
      case 106:
        ENV_T4[0] = value;
        Serial.println("ENV_T4 L1" + String(ENV_T4[0]));
        break;
      case 107:
        step = samplesize[0] / 100;
        sampleend[0] = value * step;
        Serial.println("SAMPLE END L1: " + String(sampleend[0]));
        break;
      case 108:
        ENV_L1[0] = value;
        Serial.println("ENV_L1 L1: " + String(ENV_L1[0]));
        break;
      case 109:
        ENV_L2[0] = value;
        Serial.println("ENV_L2 L1: " + String(ENV_L2[0]));
        break;
      case 110:
        step = samplesize[0] / 100;
        samplebegin[0] = value * step;
        Serial.println("SAMPLE BEGIN L1: " + String(samplebegin[0]));
        break;
      case 111:
        ENV_LSUS[0] = value;
        Serial.println("ENV_LSUS L1" + String(ENV_LSUS[0]));
        break;
      case 112:
        ENV_LEND[0] = value;
        Serial.println("ENV_LEND L1" + String(ENV_LEND[0]));
        break;
      case 113:
        opmenuoldal = 0;
        if (value == 0) {

          loopsample[opmenuoldal] = false;
          Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
        }
        if (value == 1) {
          loopsample[opmenuoldal] = true;
          Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
        }
        break;
      case 114:
        if (value == 4) {
          sampleend[0]++;
          if (samplesize[0] < sampleend[0]) {
            sampleend[0] = samplesize[0];
          }
        }

        Serial.println("SAMPLE END L1: " + String(sampleend[0]));
        if (value == 1) {
          samplebegin[0]++;
          if (samplesize[0] < samplebegin[0]) {
            samplebegin[0] = samplesize[0];
          }
        }
        Serial.println("SAMPLE BEGIN L1: " + String(samplebegin[0]));

        break;
    }

  if (localParameterByte == 2) {
    switch (noteByte) {
      case 0:
        COARSE[1] = value;
        Serial.println("COARSE L2: " + String(COARSE[1]));
        notetune();
        break;
      case 1:
        FINE[1] = value;
        Serial.println("FINE L2: " + String(FINE[1]));
        notetune();
        break;
      case 2:
        KEYFollow[1] = value;
        Serial.println("KEYFollow L2: " + String(KEYFollow[1]));
        notetune();
        break;
      case 4:
        TVA[1] = value;
        Serial.println("TVA L2: " + String(TVA[1]));
        break;
      case 7:
        PCMWaveNo[1] = value;
        Serial.println("PCMWaveNo L2: " + String(PCMWaveNo[1]));
        opmenuoldal = 1;
        setPCMWave();
        break;
      case 35:
        volume[1] = value;
        Serial.println("Level L2: " + String(volume[1]));
        break;
      case 37:
        BiasPoint[1] = value;
        Serial.println("BiasPoint L2: " + String(BiasPoint[1]));
        notebias();
        break;
      case 38:
        BiasLevel[1] = value;
        Serial.println("bieasLevel L2: " + String(BiasLevel[1]));
        notebias();
        break;
      case 39:
        ENV_T1[1] = value;
        Serial.println("ENV_T1 L2: " + String(ENV_T1[1]));
        break;
      case 40:
        ENV_T2[1] = value;
        Serial.println("ENV_T2 L2" + String(ENV_T2[1]));
        break;
      case 41:
        ENV_T3[1] = value;
        Serial.println("ENV_T3 L2" + String(ENV_T3[1]));
        break;
      case 42:
        ENV_T4[1] = value;
        Serial.println("ENV_T4 L2" + String(ENV_T4[1]));
        break;
      case 43:
        step = samplesize[1] / 100;
        sampleend[1] = value * step;
        Serial.println("SAMPLE END L2: " + String(sampleend[1]));
        break;
      case 44:
        ENV_L1[1] = value;
        Serial.println("ENV_L1 L2: " + String(ENV_L1[1]));
        break;
      case 45:
        ENV_L2[1] = value;
        Serial.println("ENV_L2 L2: " + String(ENV_L2[1]));
        break;
      case 46:
        step = samplesize[1] / 100;
        samplebegin[1] = value * step;
        Serial.println("SAMPLE BEGIN L2: " + String(samplebegin[1]));
        break;
      case 47:
        ENV_LSUS[1] = value;
        Serial.println("ENV_LSUS L2: " + String(ENV_LSUS[1]));
        break;
      case 48:
        ENV_LEND[1] = value;
        Serial.println("ENV_LEND L2: " + String(ENV_LEND[1]));
        break;
      case 49:
        opmenuoldal = 1;
        if (value == 0) {

          loopsample[opmenuoldal] = false;
          Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
        }
        if (value == 1) {
          loopsample[opmenuoldal] = true;
          Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
        }
        break;
      case 50:
        if (value == 4) {
          sampleend[1]++;
          if (samplesize[1] < sampleend[1]) {
            sampleend[1] = samplesize[1];
          }
        }

        Serial.println("SAMPLE END L1: " + String(sampleend[1]));
        if (value == 1) {
          samplebegin[1]++;
          if (samplesize[1] < samplebegin[1]) {
            samplebegin[1] = samplesize[1];
          }
        }
        Serial.println("SAMPLE BEGIN L2: " + String(samplebegin[1]));

        break;

      case 106:
        switch (value) {
          case 1:
            //chorus1
            chorusbuffersize = 255;
            LFOadress[0] = lfotriangle;
            LFOadress[1] = lfotriangle;
            break;
          case 2:
            //chorus2
            chorusbuffersize = 386;
            LFOadress[0] = lfotriangle;
            LFOadress[1] = lfosine;
            break;
          case 3:
            //chorus3
            chorusbuffersize = 511;
            LFOadress[0] = lfotriangle;
            LFOadress[1] = lfotriangle;
            break;
          case 4:
            //chorus4
            chorusbuffersize = 255;
            LFOadress[0] = lfosine;
            LFOadress[1] = lfotriangle;
            break;
          case 5:
            //chorus5
            chorusbuffersize = 386;
            LFOadress[0] = lfosine;
            LFOadress[1] = lfosine;
            break;
          case 6:
            //chorus6
            chorusbuffersize = 511;
            LFOadress[0] = lfosine;
            LFOadress[1] = lfosine;
            break;

        }
        break;
      case 107:
        // chorusRate=value;
        lfofreq[0] = value;
        break;
      case 108:
        chorusLevelLeft = value;
        break;

    }
  }
  if (localParameterByte == 3) {
    switch (noteByte) {
      case 30:
        switch (value) {
          case 0:
            delaybuffersize = 2048;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            Serial.println("SmallHall");
            break;
          case 1:
            delaybuffersize = 4096;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            break;
          case 2:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            break;
          case 3:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            break;
          case 4:
            delaybuffersize = 4096;
            delaytime = 1;
            delay2time = 2;
            reverblevel = 127;
            break;
          case 5:
            delaybuffersize = 8192;
            delaytime = 2;
            delay2time = 1;
            reverblevel = 127;
            break;
          case 6:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 2;
            reverblevel = 127;
            break;
          case 7:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 2;
            reverblevel = 127;
            break;
          case 8:
            delaybuffersize = 8192;
            delaytime = 2;
            delay2time = 2;
            reverblevel = 127;
            break;
          case 9:
            delaybuffersize = 8192;
            delaytime = 3;
            delay2time = 2;
            reverblevel = 127;
            break;
          case 10:
            delaybuffersize = 8192;
            delaytime = 3;
            delay2time = 3;
            reverblevel = 127;
            break;
          case 11:
            delaybuffersize = 8192;
            delaytime = 3;
            delay2time = 4;
            reverblevel = 127;
            break;
          case 12:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 4;
            reverblevel = 127;
            break;
          case 13:
            delaybuffersize = 8192;
            delaytime = 2;
            delay2time = 4;
            reverblevel = 127;
            break;
          case 14:
            Serial.println("Crossdelay 148-256msec");
            delaybuffersize = 8192;
            delaytime = 4;
            delay2time = 4;
            reverblevel = 127;
            break;
        }
        break;
      case 31:
        reverbdiffusion = value;
        break;

    }
  }
}

//--------------MIDI PARAMETER CONTROL-------------
void parameterchange2() {

  byte value = velocityByte;
  switch (noteByte) {
    case 0:
      KEYFollow[opmenuoldal] = value;
      break;
    case 5:
      volume[opmenuoldal] = value;

      Serial.println("generatorvolume" + String(opmenuoldal) + ": " + String(volume[opmenuoldal]));
      break;
    case 6:
      if (value < 32) {
        PCMWaveNo[opmenuoldal] = value;
        setPCMWave();
      }
      if (value == 32) {
        loopsample[opmenuoldal] = false;
        Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
      }
      if (value == 33) {
        loopsample[opmenuoldal] = true;
        Serial.println("loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]));
      }
      //opmenuincrent
      if (value == 52) {
        if (opmenuoldal < 3) {
          opmenuoldal++;
        } else {
          opmenuoldal = 0;
        }
      }
      //opmenudecrement
      if (value == 53) {
        if (opmenuoldal > 0) {
          opmenuoldal--;
        } else {
          opmenuoldal = 3;
        }
      }
      //opmenuset
      if (value == 54) {
        opmenuoldal = 0;
      }
      if (value == 55) {
        opmenuoldal = 1;
      }
      if (value == 56) {
        opmenuoldal = 2;
      }
      if (value == 57) {
        opmenuoldal = 3;
      }
      Serial.println("opmenuoldal: " + String(opmenuoldal));
      break;
    case 7:

      switch (opmenuoldal) {
        case 0: reverblevel = value + 1; break;
        case 1: reverbdiffusion = value; break;
        case 2: delaytime = value; break;
        case 3: delay2time = value; break;
      }
      Serial.println("Reverblevel: " + String(reverblevel));
      Serial.println("Reverbdiffusion: " + String(reverbdiffusion));
      Serial.println("delaytime: " + String(delaytime));
      Serial.println("delay2time: " + String(delay2time));
      break;
    case 44:
      COARSE[opmenuoldal] = value;
      Serial.println("GENERATOR FREQ: " + String(opmenuoldal) + " :" + String(COARSE[opmenuoldal]));
      break;
    case 108:
      ENV_L1[opmenuoldal] = value;
      break;
    case 109:
      ENV_L2[opmenuoldal] = value;
      break;
    case 110:
      ENV_LSUS[opmenuoldal] = value;
      break;
    case 113:

      break;
    case 114:
      ENV_T1[opmenuoldal] = value;
      break;

    case 115:
      ENV_T2[opmenuoldal] = value;

      samplebegin[opmenuoldal] = value << 7;

      if  (samplesize[opmenuoldal] < samplebegin[opmenuoldal])
      {
        samplebegin[opmenuoldal] = samplesize[opmenuoldal];
      }

      Serial.println("SAMPLE BEGIN" + String(opmenuoldal) + " :" + String(samplebegin[opmenuoldal]));

      break;
    case 116:
      sampleend[opmenuoldal] = value << 7;
      if (samplesize[opmenuoldal] < sampleend[opmenuoldal]) {
        sampleend[opmenuoldal] = samplesize[opmenuoldal];
      }
      Serial.println("SAMPLE END: " + String(opmenuoldal) + " :" + String(sampleend[opmenuoldal]));
      break;
    case 117:
      ENV_T4[opmenuoldal] = value;
      break;
  }
}

//-------------------------------REVERB-DELAY EFFECT LEFT----------------------------------------
//bufferbe: actual sample, delaybuffer array, reverblecel, reverbdiffusion, delaytime,
int32_t atlag = 0;
int32_t atlag2 = 0;
void reverbleft() {
  bufferbe[0] = bufferbe[0] + delaybuffer[delaybufferindex];
  atlag += (reverblevel - 1) * bufferbe[0] / reverblevel;
  delaystep++;
  if (delaystep >= delaytime) {
    delaybuffer[delaybufferindex] = ((atlag / delaystep) << 4) / reverbdiffusion;
    atlag = 0;
    delaybufferindex++;
    delaystep = 0;
  }
  delaybufferindex &= (reverbtime - 1);
}

//------------------------------REVERB-DELAY EFFECT RIGHT-----------------------------------------
//bufferbe: actual sample right, delaybuffer2 array, reverblevel, reverbdiffusion, delaytime2, delaystep2
void reverbright() {
  bufferbe[1] = bufferbe[1] + delaybuffer2[delaybufferindex2];
  atlag2 += (reverblevel - 1) * bufferbe[1] / reverblevel;
  delay2step++;
  if (delay2step >= delay2time) {
    delaybuffer2[delaybufferindex2] = ((atlag2 / delay2step) << 4) / reverbdiffusion;
    atlag2 = 0;
    delaybufferindex2++;
    delay2step = 0;
  }

  delaybufferindex2 &= (reverbtime2 - 1);
  // Serial.println("delaybufferindex2 : " + String(delaybufferindex2));
}

//--------------------------CHORUS LEFT------------------------------


int16_t chorusbufferleft[512];
uint16_t chorusbufferindex = 0;
uint16_t chorusindex;
void chorusleft() {
  chorusbufferleft[chorusbufferindex] = bufferbe[0];
  chorusbufferindex++;
  chorusbufferindex &= (chorusbuffersize);
  chorusindex = (lfovalue[0] + chorusbufferindex) % chorusbuffersize;
  bufferbe[0] = (bufferbe[0] + ((chorusbufferleft[chorusindex] * chorusLevelLeft) >> 7));
}

//--------------------------CHORUS RIGHT------------------------------

int16_t chorusbufferright[512];
uint16_t chorusbufferindex2 = 0;
uint16_t chorusindex2;
void chorusright() {
  chorusbufferright[chorusbufferindex2] = bufferbe[1];
  chorusbufferindex2++;
  chorusbufferindex2 &= (chorusbuffersize2);


  // Serial.println("chorusbufferindex2 : " + String(chorusbufferindex2 ));

  chorusindex2 = (lfovalue[1] + chorusbufferindex2) % chorusbuffersize2;
  bufferbe[1] = (bufferbe[1] + ((chorusbufferright[chorusindex2] * chorusLevelRight) >> 7));
}


//-----------------------LOWPASSFILTER LEFT---------------------------
//lowpassfilter in delaybuffer!!!
//delaybuffer actual sample, x: delaybuffer prev sample
int16_t x = 0;
void lowpassfilterleft() {
  delaybuffer[delaybufferindex] = (delaybuffer[delaybufferindex] + x) >> 1;
  x = delaybuffer[delaybufferindex];
}

//LOWPASSFILTER RIGHT
//lowpassfilter in delaybuffer!!!
//delaybuffer actual sample, x2: delaybuffer prev sample
int16_t x2 = 0;
void lowpassfilterright() {
  delaybuffer2[delaybufferindex2] = (delaybuffer2[delaybufferindex2] + x2) >> 1;
  x2 = delaybuffer2[delaybufferindex2];
}


//-------------------------------MIDI INPUT COMMAND-------------------------------------
void serialEvent() {
  if (MIDI2.read(midichan)) {
    switch (MIDI2.getType()) {
      case midi::NoteOn:
        //  Serial.println("NoteOn: " + String(noteByte) + " " + String(velocityByte) + " ");
        noteByte = MIDI2.getData1();
        velocityByte = MIDI2.getData2();

        wavefreq[0][generatornumber] = noteertek[0][noteByte  + LKeyShift];
        wavebias[0][generatornumber] = Bias[0][noteByte  + LKeyShift];
        //Serial.println("wavefreq0: " + String(wavefreq[0][generatornumber]));
        wavefreq[1][generatornumber] = noteertek[1][noteByte  + UKeyShift];
        wavebias[1][generatornumber] = Bias[1][noteByte  + LKeyShift];
        wavefreq[2][generatornumber] = noteertek[2][noteByte  + LKeyShift];
        wavebias[2][generatornumber] = Bias[2][noteByte  + LKeyShift];
        wavefreq[3][generatornumber] = noteertek[3][noteByte  + UKeyShift];
        wavebias[3][generatornumber] = Bias[3][noteByte  + LKeyShift];
        oldnoteByte[generatornumber] = noteByte;
        pich[0][generatornumber] = wavefreq[0][generatornumber];
        pich[1][generatornumber] = wavefreq[1][generatornumber];
        pich[2][generatornumber] = wavefreq[2][generatornumber];
        pich[3][generatornumber] = wavefreq[3][generatornumber];
        // Serial.println(String(pich[generatornumber]));
        freqmutato[0][generatornumber] = samplebegin[0] << 18;
        freqmutato[1][generatornumber] = samplebegin[1] << 18;
        freqmutato[2][generatornumber] = samplebegin[2] << 18;
        freqmutato[3][generatornumber] = samplebegin[3] << 18;
        noteoff[generatornumber] = false;
        TVAvolume[0][generatornumber] = ENV_L0;
        TVAvolume[1][generatornumber] = ENV_L0;
        TVAvolume[2][generatornumber] = ENV_L0;
        TVAvolume[3][generatornumber] = ENV_L0;
        generatorstatus[0][generatornumber] = 0;
        generatorstatus[1][generatornumber] = 0;
        generatorstatus[2][generatornumber] = 0;
        generatorstatus[3][generatornumber] = 0;
        generatornumber++;
        if (generatornumber == polyphony) {
          generatornumber = 0;
        }
        break;
      case midi::NoteOff:
        //  Serial.println("Noteoff: " + String(noteByte) + " " + String(velocityByte) + " ");
        noteByte = MIDI2.getData1();
        //  velocityByte = MIDI2.getData2();

        for (int i = 0; i < polyphony; i++) {
          if (noteByte == oldnoteByte[i]) {
            oldnoteByte[i] = 0;
            //  noteoff[i] = true;
            generatorstatus[0][i] = 3;
            generatorstatus[1][i] = 3;
            generatorstatus[2][i] = 3;
            generatorstatus[3][i] = 3;
          }
        }
        break;
      case midi::ProgramChange:
        Serial.println("Programchange: " + String(noteByte) + " " + String(velocityByte) + " ");
        break;
      case midi::AfterTouchPoly:
        Serial.println("AfterTouch: " + String(noteByte) + " " + String(velocityByte) + " ");
        break;
      case midi::PitchBend:

        break;
      case midi::ControlChange:
        noteByte = MIDI2.getData1();
        velocityByte = MIDI2.getData2();
        parameterchange2();

        break;
      case midi::Clock:

        break;
      case midi::SystemExclusive:
        Serial.println("SysexDATA: ");
        for (int i = 0; i < MIDI2.getSysExArrayLength(); i++) {
          Serial.print(String(MIDI2.getSysExArray()[i]) + " ");
        }
        if (MIDI2.getSysExArray()[0] == 240 && MIDI2.getSysExArray()[1] == 65 && MIDI2.getSysExArray()[2] == 0 && MIDI2.getSysExArray()[3] == 20 && MIDI2.getSysExArray()[4] == 18 && MIDI2.getSysExArray()[5] == 0) {
          localParameterByte = MIDI2.getSysExArray()[6];
          noteByte = MIDI2.getSysExArray()[7];
          velocityByte = MIDI2.getSysExArray()[8];
          parametersysexchanged();
        }
        break;
    }
  }
}


void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  Serial.println("i2s Setup begin for Roland D50Sampler....");
  delay(1000);
  // Set up I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
  //Set up LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  D50-Sampler   ");
  delay(600);
  lcd.setCursor(0, 1);
  lcd.print("  DigitalSynth  ");
  delay(600);
  lcd.setCursor(0, 0);
  lcd.print(" Firmvare: 0.2 ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  delay(10);


  // Set up MIDI
  MIDI2.begin(MIDI_CHANNEL_OMNI);
  //Set up NOTE TUNE
  notetune();
  notebias();
  maxsize();
  setLFOWave();
  for (int i = 0; i < 4; i++) {
    opmenuoldal = i;
    setPCMWave();
    setsamplesize();
  }
  opmenuoldal = 0;
}


const uint32_t expgains128[128] = { 0, 1, 2, 3, 4, 5, 6, 11, 16, 23, 32, 42, 55, 70, 88, 108, 132, 158, 188, 221, 258, 298, 343, 392, 445, 504, 566, 634, 708, 786, 870, 961, 1057, 1159, 1267, 1383, 1505, 1633, 1770, 1913, 2064, 2223, 2389, 2564, 2747, 2939, 3139, 3349, 3567, 3795, 4032, 4279, 4535, 4802, 5079, 5366, 5665, 5973, 6293, 6625, 6967, 7321, 7688, 8066, 8456, 8858, 9274, 9702, 10142, 10597, 11064, 11545, 12040, 12548, 13071, 13608, 14160, 14726, 15308, 15904, 16516, 17143, 17786, 18444, 19119, 19810, 20517, 21242, 21982, 22740, 23516, 24308, 25118, 25947, 26793, 27657, 28539, 29441, 30361, 31299, 32258, 33235, 34232, 35249, 36285, 37342, 38419, 39517, 40635, 41775, 42935, 44117, 45320, 46545, 47791, 49060, 50351, 51664, 53001, 54359, 55741, 57147, 58575, 60027, 61504, 63004, 64430, 65535 };

/*
  uint32_t egyopgenA(uint32_t freqmutato1, long op1level, byte lep)
  {
  return generator1[freqmutato1 >> lep] * op1level;
  }
*/

void loop() {
  //--MIDI input--
  serialEvent();
  //LFO
  for (int i = 0; i < LFOnumber; i++) {
    lfovalue[i] = *(LFOadress[i] + (lfoarrayindex[i] >> 23));
    lfoarrayindex[i] += (lfofreq[i] << 19);
  }
  // Serial.println("lfo1value : " + String(lfo1value ));
  //TVA ENVELOPE

  for (int i = 0; i < 4; i++) {
    if (TVA[i] == 1) {
      for (int j = 0; j < polyphony; j++) {
        switch (generatorstatus[i][j]) {
          case 0:
            if (TVAvolume[i][j] < ENV_L1[i]) {
              TVAvolume[i][j] = TVAvolume[i][j] + ENV_T1[i];
            } else {
              generatorstatus[i][j]++;
            }
            break;
          case 1:
            if (ENV_L2[i] < ENV_L1[i] && TVAvolume[i][j] < ENV_L2[i]) {
              TVAvolume[i][j] = TVAvolume[i][j] - ENV_T2[i];
            } else if (ENV_L2[i] > ENV_L1[i] && TVAvolume[i][j] > ENV_L2[i]) {
              TVAvolume[i][j] = TVAvolume[i][j] + ENV_T2[i];
            } else {
              generatorstatus[i][j]++;
            }
            break;
          case 2:
            TVAvolume[i][j] = ENV_LSUS[i];
            break;
          case 3:
            if (TVAvolume[i][j] > ENV_LEND[i]) {
              TVAvolume[i][j] = TVAvolume[i][j] - ENV_T4[i];
            } else {
              generatorstatus[i][j]++;
            }
            break;

          case 4:
            TVAvolume[i][j] = 0;
            break;
        }
        generatorvolume[i][j] = (TVAvolume[i][j] * volume[i] * wavebias[i][j]) >> 10;
      }
      // Serial.println("Generator" + String(i) + "statusz: " + String(generatorstatus[i][0])+" "+String(generatorstatus[i][1])+" "+String(generatorstatus[i][2])+" "+String(generatorstatus[i][3])+" "+String(generatorstatus[i][4])+" "+String(generatorstatus[i][5]));
    } else
    {
      for (int j = 0; j < polyphony; j++) {
        generatorvolume[i][j] =  (volume[i] * wavebias[i][j]) >> 4;
      }
    }
  }
  //release


  //--2 SOUND LEFT, 2 SOUND RIGHT, 6  POLYFONI!!!!--
  for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
    bufferbe[0] = 0;
    bufferbe[1] = 0;
    for (int j = 0; j < polyphony; j++) {

      if ((freqmutato[0][j] >> 18) < sampleend[0] - 1) {
        tempbuffer1 = *(genstartadress[0] + (freqmutato[0][j] >> 18));
        bufferbe[0] += (tempbuffer1 * generatorvolume[0][j]) >> 5;
        freqmutato[0][j] += pich[0][j];
      } else if (loopsample[0]) {
        // if(j==0){Serial.println(String(freqmutato[0][0] >> 18));}
        freqmutato[0][j] = samplebegin[0] << 18;
        // if(j==0){Serial.println(String(freqmutato[0][0] >> 18));}
      }

      if ((freqmutato[1][j] >> 18) < sampleend[1]) {
        tempbuffer2 = *(genstartadress[1] + (freqmutato[1][j] >> 18));
        //    bufferbe[0] += (((tempbuffer2 * generatorvolume[1][j]) >> 5)*((tempbuffer1 * generatorvolume[0][j]) >> 5))>>10; //ringmod
        bufferbe[0] += ((tempbuffer2 * generatorvolume[1][j]) >> 5);
        freqmutato[1][j] += pich[1][j];
      } else if (loopsample[1]) {
        freqmutato[1][j] = samplebegin[0] << 18;
      }

      if ((freqmutato[2][j] >> 18) < sampleend[2]) {
        tempbuffer3 = *(genstartadress[2] + (freqmutato[2][j] >> 18));
        bufferbe[1] += (tempbuffer3 * generatorvolume[2][j]) >> 5;
        freqmutato[2][j] += pich[2][j];
      } else if (loopsample[2]) {
        freqmutato[2][j] = samplebegin[2] << 18;
      }

      if ((freqmutato[3][j] >> 18) < sampleend[3]) {
        tempbuffer4 = *(genstartadress[3] + (freqmutato[3][j] >> 18));
        bufferbe[1] += (tempbuffer4 * generatorvolume[3][j]) >> 5;
        freqmutato[3][j] += pich[3][j];
      } else if (loopsample[3]) {
        freqmutato[3][j] = samplebegin[3] << 18;
      }
    }
    bufferbe[0] = bufferbe[0] >> 3;
    bufferbe[1] = bufferbe[1] >> 3;
    chorusleft();
    chorusright();
    reverbleft();
    reverbright();
    lowpassfilterleft();
    lowpassfilterright();
    sBuffer[i] = bufferbe[0];
    sBuffer[i + 1] = bufferbe[1];
  }
  //BUFFER WRITE DAC
  i2s_write(I2S_PORT, &sBuffer, bufferLen, &i2s_bytes_write, portMAX_DELAY);
}
