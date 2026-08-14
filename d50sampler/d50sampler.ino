#pragma GCC optimize ("Ofast")
#include <driver/i2s.h>
#include <MIDI.h>
#include "samples.h"

//#include <MIDIUSB.h>

//----------------MIDI SETUP BEGIN-----------
struct Serial2MIDISettings : public midi::DefaultSettings {
  static const long BaudRate = 31250;
  static const int8_t RxPin = 16;
  static const int8_t TxPin = 17;
  static const unsigned SysExMaxSize = 64;
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
#define bufferLen 256
int16_t sBuffer[bufferLen];

//---16-bit
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

/*
  ///32bit ????
  void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // Átállítva 32-re
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S), // Tisztább konfiguráció
    .intr_alloc_flags = 0,
    .dma_buf_count = 2,
    .dma_buf_len = bufferLen,
    .use_apll = true // 32-bitnél javasolt az APLL a pontosabb órajelért
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  }

*/

//----------------DAC SETUP END------------

//----------------------PROGRAM VARIABLES-----------
const byte polyphony = 8;
uint32_t freqmutato[4][polyphony];
uint32_t pich[4][polyphony];
uint32_t currentPitch[4][polyphony];
uint32_t pichcount[4][polyphony];
byte generatornumber = 1;
uint32_t wavefreq[4][polyphony];
byte wavebias[4][polyphony];
uint32_t noteertek[4][256];
byte oldnoteByte[polyphony];
bool noteoff[polyphony];
bool loopsample[4] = { true, true, true, true };
uint16_t samplebegin[4] = { 0, 0, 0, 0 };
uint16_t sampleend[4] = { 10190, 10190, 10190, 10190 };
byte opmenuoldal = 0;
uint16_t samplesize[4];
bool LCD_ON = true;
//int step = 16;
//uint16_t GLOBAL_TUNE = 472;

int step = 22;
uint16_t GLOBAL_TUNE = 5040;
byte prognumber = 0;
byte COARSE[4] = { 48, 48, 48, 48 };
byte FINE[4] = { 50, 50, 50, 50 };
byte szorzo[4] = {1, 1, 1, 1};
byte LKeyShift = 0;
byte UKeyShift = 0;
byte volume[4] { 20, 20, 20, 20 };
uint16_t generatorvolume[4][polyphony];
//reverb variable
int32_t bufferbe[8];
uint16_t delaybuffersize = 8192;
//int16_t delaybuffer[8192];
//int16_t delaybuffer2[8192];
int16_t* delaybuffer = NULL; // Fontos a NULL, hogy lássuk, ha nincs kész
int16_t* delaybuffer2 = NULL;
uint16_t delaybufferindex = 0;
uint16_t delaybufferindex2 = 0;
byte delaystep = 0;
byte delay2step = 0;
byte delaytime = 1;
byte delay2time = 1;
byte reverblevel = 20;
byte reverbdiffusion = 1; // 1 = éles/pattogós, 8 = nagyon sűrű/puha

uint16_t reverbtime = delaybuffersize;
uint16_t reverbtime2 = delaybuffersize;
byte chorusLevelLeft = 56;
byte chorusLevelRight = 37;
uint16_t chorusbuffersize = 511;
uint16_t chorusbuffersize2 = 511;
int16_t const* genstartadress[4];
const byte LFOnumber = 8;
uint16_t const* LFOadress[LFOnumber];
// 0: Sinus, 1: Triangle, 2: Saw, 3: Random
int32_t tempbuffer0;
int32_t tempbuffer1;
int32_t tempbuffer2;
int32_t tempbuffer3;
int32_t tempbuffer[4] = {0, 0, 0, 0};
byte ENV_L0 = 0;
// 0-3: TVA (Hangerő), 4-7: TVF (Szűrő)
byte ENV_T1[8]   = { 100, 100, 100, 100, 100, 100, 100, 100 };
byte ENV_L1[8]   = { 100, 100, 100, 100, 100, 100, 100, 100 };
byte ENV_T2[8]   = { 1, 1, 1, 1, 1, 1, 1, 1 };
byte ENV_L2[8]   = { 80, 80, 80, 80, 80, 80, 80, 80 };
byte ENV_L3[8]   = { 80, 80, 80, 80, 80, 80, 80, 80 };
byte ENV_T3[8]   = { 1, 1, 1, 1, 1, 1, 1, 1 };
byte ENV_LSUS[8] = { 50, 50, 50, 50, 50, 50, 50, 50 };
byte ENV_T4[8]   = { 1, 1, 1, 1, 1, 1, 1, 1 };
byte ENV_T5[8]   = { 1, 1, 1, 1, 1, 1, 1, 1 };
byte ENV_LEND[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
byte TVA_Slide = 19;
#define L_SCALE 2500 // 2.5 * 1000 a fixpontos matekhoz
byte generatorstatus[8][polyphony];
uint32_t TVAvolume[8][polyphony];
byte TVFlevel[4][polyphony];
byte TVA[4] = {1, 1, 1, 1};
byte KEYFollow[4] = { 11, 11, 11, 11 };
byte LFOMode[4] = { 0, 0, 0, 0 };
byte PENVMode[4] = { 0, 0, 0, 0 };
byte BENDERMode[4] = { 0, 0, 0, 0 };
byte Waveform[4] = { 2, 2, 2, 2 };
byte PCMWaveNo[4] = { 1, 1, 1, 1 };
byte BiasPoint[4] = {64, 64, 64, 64};
byte BiasLevel[4] = {12, 12, 12, 12};
byte Bias[4][256];
byte STRUCTURE_U = 5;
byte STRUCTURE_L = 5;
byte STRUCTURE = 55;
uint32_t lfoarrayindex[LFOnumber] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t lfovalue[LFOnumber];
byte LFO_Delay[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte LFO_Delay_Counter[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte lfofreq[LFOnumber] = {7, 10, 22, 22, 22, 22, 22, 22};
uint8_t LFO_Wave_Select[8] = {0, 0, 0, 0, 0, 0, 4, 4};
float f0 = 100;
float f0orig = 100;
float Q = 2;
float f02 = 100;
float f02orig = 100;
float Q2 = 2;
byte lfolevel[6];//using effect
byte LFOSYNC[6] = {2, 2, 2, 2, 2, 2};
byte lfo2sync = 2;
byte lfo3sync = 2;
byte lfo4sync = 2;
byte lfo5sync = 2;
byte CHASE_TIME = 0;
byte CHASE_LEVEL = 4;
uint32_t ido;
long elozoido;
byte lastchase = 0;
byte CaseArray[32];
byte chaseindex = 0;
byte MIDI_SYNC = 1;
byte sixteen = 0;
byte PW[4] = {15, 15, 15, 15};
String pachname = "Initial Patch";
uint32_t PWcount[4][polyphony];
float filter_f[4][polyphony] = {0.8f, 0.8f, 0.8f, 0.8f};
float filter_q[4] = {0.5f, 0.5f, 0.5f, 0.5f}; // Semleges rezonancia
uint16_t portamento_time[4] = {20, 20, 20, 20}; // 0 = azonnali, nagyobb = lassabb csúszás
// A D-50 stílusú 0-100 értékek (ezeket majd te tekered)
byte tvf_cutoff[4] = {80, 80, 80, 80};
byte tvf_reso[4] = {0, 0, 0, 0};
float v_lp[4][polyphony];
float v_bp[4][polyphony];
byte PICHLFO[4] = {0, 0, 0, 0};
byte TWFLFO[4] = {2, 3, 4, 5};
byte PWMLFO[4] = {2, 3, 4, 5};
byte PICH_LFO_level[4] = {0, 0, 0, 0};
byte TVF_LFO_level[4] = {0, 0, 0, 0};
byte PWMLFODepth[4];
byte masterVolume = 5;
volatile int masterTick = 0;
byte OFFSET = 2;
byte modulationWheel = 0;
int32_t pitchBendValue = 0;
float lastOut[4][polyphony];
uint32_t tvapointer[4][polyphony];
byte ropi = 10;
byte Lpoly = 8;
byte Upoly = 8;
byte oscMode[4] = {0, 0, 0, 0};
uint32_t lastTargetPitch[4] = {0, 0, 0, 0};
byte voiceStack[polyphony];
uint16_t smoothedVol[4][polyphony];
const float* waveformLookup[3] = { sqrTable, sawTable, sinTable };
uint8_t modulationMatrix[4][3];
float tvf_env_mod[4][polyphony];
uint8_t tvf_env_depth[8] = {100, 100, 100, 100, 50, 50, 50, 50}; // 0-100 közötti értékek
float ceiling_val = 0.95f;
float stretch_val = 0.00015f;
// 1. MEMÓRIA LEFOGLALÁS (Fixen a legnagyobb szoba méretére: 2048)
int16_t chorusbufferleft[512];
int16_t chorusbufferright[512];


uint16_t chorusbufferindex = 0;
uint16_t chorusbufferindex2 = 0;

// Ezt a változót állítod a case 106-ban (pl. 127, 255, 511, 1023, 2047)
uint16_t currentChorusMask = 511;

int16_t atlagchorus0 = 0;
int16_t atlagchorus1 = 0;
uint16_t maskLeft = 511;
uint16_t maskRight = 511;

//----------------------------PARAMETRIC EQ LEFT-------------------------------------------------
/* cut-off (or center) frequency in Hz */
/* filter Q */
/*
  https://ethanwiner.com/eq-dsp.htm
*/
float Fs = 44100;
float Pi = 3.141592;
byte eqlevel = 10;
byte eqlevel2 = 10;
//parametric eq left default:
float w0 = 2 * Pi * f0 / Fs;
float alpha = sin(w0) / (2 * Q);
float a0 = (1 + alpha);
float a1 = (-2 * cos(w0));
float a2 = (1 - alpha) ;
float b0 = ((1 + cos(w0)) / 2);
float b1 = (-(1 + cos(w0))) ;
float b2 = ((1 + cos(w0)) / 2) ;


//parametric eq left init:
void eqkiszamol() {
  float cosw0 = cos(w0);
  w0 = 2 * Pi * f0 / Fs;
  alpha = sin(w0) / (2 * Q);
  a0 = (1 + alpha) * 100 ;
  a1 = (-2 * cosw0) * 100;
  a2 = (1 - alpha) * 100;

  b0 = ((1 + cosw0) / 2) * 100;
  b1 = (-(1 + cosw0)) * 100;
  b2 = ((1 + cosw0) / 2) * 100;
}

//parametric eq left counts actual value:
int32_t PrevSample[4];
int32_t lastbuffer[3];
int32_t paraeqleftbuffer;
//parametric eq left function
void parametereqleft() {
  PrevSample[3] = PrevSample[2];
  PrevSample[2] = PrevSample[1];
  PrevSample[1] = PrevSample[0];
  PrevSample[0] = bufferbe[0];
  //bufferbe[0] = ( b0 / a0 * PrevSample[0]) +(b1 / a0 * PrevSample[1]) +(b2 / a0 * PrevSample[2]) -(a1 / a0 * lastbuffer[0]) - (a2 / a0 * lastlastbuffer);
  paraeqleftbuffer = (b0 / a0 * PrevSample[0]) + (b1 / a0 * PrevSample[1]) + (b2 / a0 * PrevSample[2])  - (a1 / a0 * lastbuffer[0]) - (a2 / a0 * lastbuffer[1]);
  lastbuffer[2] = lastbuffer[1];
  lastbuffer[1] = lastbuffer[0];
  lastbuffer[0] =  paraeqleftbuffer;
}

//parametric eq2 right default:
float w02 = 2 * Pi * f02 / Fs;
float alpha2 = sin(w02) / (2 * Q2);
float a02 = (1 + alpha2);
float a12 = (-2 * cos(w02));
float a22 = (1 - alpha2) ;
float b02 = ((1 + cos(w02)) / 2);
float b12 = (-(1 + cos(w02))) ;
float b22 = ((1 + cos(w02)) / 2) ;

//parametric eq2 right init:
void eqkiszamol2() {
  float cosw02 = cos(w02);
  w02 = 2 * Pi * f02 / Fs;
  alpha2 = sin(w02) / (2 * Q2);
  a02 = (1 + alpha2) * 100 ;
  a12 = (-2 * cosw02) * 100;
  a22 = (1 - alpha2) * 100;
  b02 = ((1 + cosw02) / 2) * 100;
  b12 = (-(1 + cosw02)) * 100;
  b22 = ((1 + cosw02) / 2) * 100;
}
//parametric eq right counts actual value:
int32_t PrevSample2[4];
int32_t lastbuffer2[3];
int32_t paraeqrightbuffer;
//parametric eq left function
void parametereqright() {
  PrevSample2[3] = PrevSample2[2];
  PrevSample2[2] = PrevSample2[1];
  PrevSample2[1] = PrevSample2[0];
  PrevSample2[0] = bufferbe[1];
  //bufferbe[0] = ( b0 / a0 * PrevSample2[0]) +(b1 / a0 * PrevSample2[1]) +(b2 / a0 * PrevSample2[2]) -(a1 / a0 * lastbuffer2[0]) - (a2 / a0 * lastlastbuffer2);
  paraeqrightbuffer = (b02 / a02 * PrevSample2[0]) + (b12 / a02 * PrevSample2[1]) + (b22 / a02 * PrevSample2[2])  - (a12 / a02 * lastbuffer2[0]) - (a22 / a02 * lastbuffer2[1]);
  lastbuffer2[2] = lastbuffer2[1];
  lastbuffer2[1] = lastbuffer2[0];
  lastbuffer2[0] =  paraeqrightbuffer;
}

//---------------------------TUNE----------------------------------
void notevaluesarraytest() {
  for (int i = 0; i < 256; i++) {
    Serial.print(String(noteertek[0][i]) + " ");
  }
  Serial.println();
}

void notebias() {
  for (int j = 0; j < 4; j++) {
    int point = BiasPoint[j];
    int level = BiasLevel[j];

    for (int i = 0; i < 128; i++) {
      int result = 12;
      int diff = 0;

      // 1. Távolság meghatározása (D-50 irányultság szerint)
      if (point < 64) {
        if (i < point) diff = point - i;
      } else {
        if (i > point) diff = i - point;
      }

      // 2. Görbe kiszámítása
      if (diff > 0) {
        if (level == 12) {
          result = 12; // Teljesen lapos, nincs hatás
        } else if (level == 0) {
          result = 0;  // Azonnali némítás (Hard Split)
        } else {
          // A szorzót (slope) a kívánt meredekséghez igazítjuk
          // Level 11-nél a slope = 4.
          // A számítás: (diff * 4) / 32 -> minden 8. billentyűnél veszítünk 1 egység hangerőt.
          // Ez azt jelenti, hogy 96 billentyű (8 oktáv!) kell a teljes elnémuláshoz.
          int slope = (12 - level) * 4;

          result = 12 - ((diff * slope) >> 5);
        }
      }

      // 3. Biztonsági korlátok
      if (result < 0) result = 0;
      if (result > 12) result = 12;

      Bias[j][i] = (byte)result;
    }
  }
}

void notetune() {
  for (int j = 0; j < 4; j++) {
    float szorzo2 = 1.0;
    switch (KEYFollow[j]) {
      case 0:  szorzo2 = 0.5;    break;
      case 1:  szorzo2 = 0.7071; break;
      case 2:  szorzo2 = 0.8409; break;
      case 3:  szorzo2 = 1.0;    break;
      case 4:  szorzo2 = 1.125;  break;
      case 5:  szorzo2 = 1.25;   break;
      case 6:  szorzo2 = 1.375;  break;
      case 7:  szorzo2 = 1.5;    break;
      case 8:  szorzo2 = 1.625;  break;
      case 9:  szorzo2 = 1.75;   break;
      case 10: szorzo2 = 1.875;  break;
      case 11: szorzo2 = 2.0;    break; // Normal
      case 12: szorzo2 = 2.25;   break;
      case 13: szorzo2 = 2.5;    break;
      case 14: szorzo2 = 4.0;    break;
      case 15: szorzo2 = 3.0;    break;
      case 16: szorzo2 = 5.0;    break;
    }

    // A referencia hangolás (Középső C = 60)
    float TUNE_NOW = (GLOBAL_TUNE + (FINE[j] / 8.0)) * 16;
    TUNE_NOW = TUNE_NOW * pow(2.0, COARSE[j] / 12.0);

    for (int idx = 0; idx < 168; idx++) {
      // Kiszámoljuk, hány félhangra van az adott billentyű a 60-astól
      float tavolsag = (idx - 60) / 12.0;

      // Közvetlenül a 60-as hangból indulunk ki minden hangnál!
      // f = f60 * (szorzo2 ^ tavolsag)
      noteertek[j][idx] = round(TUNE_NOW * pow(szorzo2, tavolsag));
    }
  }
}

uint16_t sizes[128];
void maxsize() {
  sizes[0] = sizeof(marimba) >> 1 ;
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
  sizes[29] = sizeof(uprightbass) >> 1;
  sizes[30] = sizeof(klarinet) >> 1;
  sizes[31] = sizeof(breath) >> 1;
  sizes[32] = sizeof(steamer) >> 1;
  sizes[33] = sizeof(hightflute) >> 1;
  sizes[34] = sizeof(lowflute) >> 1;
  sizes[35] = sizeof(guiro) >> 1;
  sizes[36] = sizeof(indianflute) >> 1;
  sizes[37] = sizeof(fluteharmonics) >> 1;
  sizes[38] = sizeof(lips1) >> 1;
  sizes[39] = sizeof(lips2) >> 1;
  sizes[40] = sizeof(trumpet) >> 1;
  sizes[41] = sizeof(trombones) >> 1;
  sizes[42] = sizeof(contrabass) >> 1;
  sizes[43] = sizeof(cello) >> 1;
  sizes[44] = sizeof(violinbow) >> 1;
  sizes[45] = sizeof(violins) >> 1;
  sizes[46] = sizeof(pizzicart) >> 1;
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
  sizes[57] = sizeof(electbassloop3) >> 1;
  sizes[58] = sizeof(electgitarloop) >> 1;
  sizes[59] = sizeof(chelloloop) >> 1;
  sizes[60] = sizeof(violinloop) >> 1;
  sizes[61] = sizeof(reedloop) >> 1;
  sizes[62] = sizeof(saxloop1) >> 1;
  sizes[63] = sizeof(saxloop2) >> 1;
  sizes[64] = sizeof(aahloop) >> 1;
  sizes[65] = sizeof(oohloop) >> 1;
  sizes[66] = sizeof(maleloop) >> 1;
  sizes[67] = sizeof(spectrum1loop) >> 1;
  sizes[68] = sizeof(spectrum2loop) >> 1;
  sizes[69] = sizeof(spectrum3loop) >> 1;
  sizes[70] = sizeof(spectrum4loop) >> 1;
  sizes[71] = sizeof(spectrum5loop) >> 1;
  sizes[72] = sizeof(spectrum6loop) >> 1;
  sizes[73] = sizeof(spectrum7loop) >> 1;
  sizes[74] = sizeof(male) >> 1;
  sizes[75] = sizeof(noise) >> 1;
  sizes[76] = (sizeof(hammer) + sizeof(japanesedrum) + sizeof(kalimba) + sizeof(pluck1)) >> 1;
  sizes[77] = (+sizeof(japanesedrum) + sizeof(kalimba)) >> 1;
  sizes[78] = (+sizeof(japanesedrum) + sizeof(kalimba) + sizeof(pluck1)) >> 1;
  sizes[79] = (+sizeof(japanesedrum) + sizeof(kalimba) + sizeof(pluck1) + sizeof(chink)) >> 1;
  sizes[80] = (sizeof(kalimba) + sizeof(pluck1)) >> 1;
  sizes[81] = (sizeof(kalimba) + sizeof(pluck1) + sizeof(chink)) >> 1;
  sizes[82] = (sizeof(kalimba) + sizeof(pluck1) + sizeof(chink) + sizeof(agogo)) >> 1;
  sizes[83] = (sizeof(pluck1) + sizeof(chink)) >> 1;
  sizes[84] = (sizeof(pluck1) + sizeof(chink) + sizeof(agogo)) >> 1;
  sizes[85] = (sizeof(pluck1) + sizeof(chink) + sizeof(agogo) + sizeof(triangle)) >> 1;
  sizes[86] = (sizeof(chink) + sizeof(agogo)) >> 1;
  sizes[87] = (sizeof(chink) + sizeof(agogo) + sizeof(triangle)) >> 1;
  sizes[88] = (sizeof(chink) + sizeof(agogo) + sizeof(triangle) + sizeof(bells)) >> 1;
  sizes[89] = (sizeof(agogo) + sizeof(triangle) + sizeof(bells) + sizeof(nailfile)) >> 1;
  sizes[90] = (sizeof(agogo) + sizeof(triangle) + sizeof(bells) + sizeof(nailfile)) >> 1;
  sizes[91] = (sizeof(agogo) + sizeof(triangle) + sizeof(bells) + sizeof(nailfile)) >> 1;
  sizes[92] = (sizeof(triangle) + sizeof(bells)) >> 1;
  sizes[93] = (sizeof(triangle) + sizeof(bells) + sizeof(nailfile)) >> 1;
  sizes[94] = (sizeof(triangle) + sizeof(bells) + sizeof(nailfile) + sizeof(pick)) >> 1;
  sizes[95] = (sizeof(bells) + sizeof(nailfile)) >> 1;
  sizes[96] = (sizeof(bells) + sizeof(nailfile) + sizeof(pick)) >> 1;
  sizes[97] = (sizeof(bells) + sizeof(nailfile) + sizeof(pick) + sizeof(lowpiano)) >> 1;
  sizes[98] = (sizeof(nailfile) + sizeof(pick)) >> 1;
  sizes[99] = (sizeof(nailfile) + sizeof(pick) + sizeof(lowpiano)) >> 1;
  //sizes[100] = (sizeof(nailfile)+sizeof(pick)+sizeof(lowpiano)+sizeof(midpiano)) >> 1;
}

void setsamplesize() {
  //Set up max sample size
  samplesize[opmenuoldal] = sizes[PCMWaveNo[opmenuoldal]];
  sampleend[opmenuoldal] = samplesize[opmenuoldal];
}

//"marimba","vibraphone","xilophone1","xilophone2","logbass","hammer","japanesedrum","kalimba","pluck1","chink","agogo","triangle","bells","pick","lowpiano","pianosample","highpiano","hapsichord","harp","organpercus",
//"steelstrings","nylonstrings","electgitar1","electgitar2","dirtygitar","pickbass","popbass","thump","klarinet","breath","popbass","steamer","steamer","steamer","steamer","steamer","steamer","steamer","steamer","steamer","steamer","steamer","steamer","steamer",
DRAM_ATTR int16_t fast_samples[4][8192];
void setPCMWave() {
  const int16_t* source_ptr = NULL;
  uint32_t source_size = 0;
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
    case 29: genstartadress[opmenuoldal] = uprightbass; break;
    case 30: genstartadress[opmenuoldal] = klarinet; break;
    case 31: genstartadress[opmenuoldal] = breath; break;
    case 32: genstartadress[opmenuoldal] = steamer; break;
    case 33: genstartadress[opmenuoldal] = hightflute; break;
    case 34: genstartadress[opmenuoldal] = lowflute; break;
    case 35: genstartadress[opmenuoldal] = guiro; break;
    case 36: genstartadress[opmenuoldal] = indianflute; break;
    case 37: genstartadress[opmenuoldal] = fluteharmonics; break;
    case 38: genstartadress[opmenuoldal] = lips1; break;
    case 39: genstartadress[opmenuoldal] = lips2; break;
    case 40: genstartadress[opmenuoldal] = trumpet; break;
    case 41: genstartadress[opmenuoldal] = trombones; break;
    case 42: genstartadress[opmenuoldal] = contrabass; break;
    case 43: genstartadress[opmenuoldal] = cello; break;
    case 44: genstartadress[opmenuoldal] = violinbow; break;
    case 45: genstartadress[opmenuoldal] = violins; break;
    case 46: genstartadress[opmenuoldal] = pizzicart; break;
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
    case 57: genstartadress[opmenuoldal] = electbassloop3; break;
    case 58: genstartadress[opmenuoldal] = electgitarloop; break;
    case 59: genstartadress[opmenuoldal] = chelloloop; break;
    case 60: genstartadress[opmenuoldal] = violinloop; break;
    case 61: genstartadress[opmenuoldal] = reedloop; break;
    case 62: genstartadress[opmenuoldal] = saxloop1; break;
    case 63: genstartadress[opmenuoldal] = saxloop2; break;
    case 64: genstartadress[opmenuoldal] = aahloop; break;
    case 65: genstartadress[opmenuoldal] = oohloop; break;
    case 66: genstartadress[opmenuoldal] = maleloop; break;
    case 67: genstartadress[opmenuoldal] = spectrum1loop; break;
    case 68: genstartadress[opmenuoldal] = spectrum2loop; break;
    case 69: genstartadress[opmenuoldal] = spectrum3loop; break;
    case 70: genstartadress[opmenuoldal] = spectrum4loop; break;
    case 71: genstartadress[opmenuoldal] = spectrum5loop; break;
    case 72: genstartadress[opmenuoldal] = spectrum6loop; break;
    case 73: genstartadress[opmenuoldal] = spectrum7loop; break;
    case 74: genstartadress[opmenuoldal] = male; break;
    case 75: genstartadress[opmenuoldal] = noise; break;
    //loop
    case 76: genstartadress[opmenuoldal] = hammer; break;
    case 77: genstartadress[opmenuoldal] = japanesedrum; break;
    case 78: genstartadress[opmenuoldal] = japanesedrum; break;
    case 79: genstartadress[opmenuoldal] = japanesedrum; break;
    case 80: genstartadress[opmenuoldal] = kalimba; break;
    case 81: genstartadress[opmenuoldal] = kalimba; break;
    case 82: genstartadress[opmenuoldal] = kalimba; break;
    case 83: genstartadress[opmenuoldal] = pluck1; break;
    case 84: genstartadress[opmenuoldal] = pluck1; break;
    case 85: genstartadress[opmenuoldal] = pluck1; break;
    case 86: genstartadress[opmenuoldal] = chink; break;
    case 87: genstartadress[opmenuoldal] = chink; break;
    case 88: genstartadress[opmenuoldal] = chink; break;
    case 89: genstartadress[opmenuoldal] = agogo; break;
    case 90: genstartadress[opmenuoldal] = agogo; break;
    case 91: genstartadress[opmenuoldal] = agogo; break;
    case 92: genstartadress[opmenuoldal] = triangle; break;
    case 93: genstartadress[opmenuoldal] = triangle; break;
    case 94: genstartadress[opmenuoldal] = triangle; break;
    case 95: genstartadress[opmenuoldal] = bells; break;
    case 96: genstartadress[opmenuoldal] = bells; break;
    case 97: genstartadress[opmenuoldal] = bells; break;
    case 98: genstartadress[opmenuoldal] = nailfile; break;
    case 99: genstartadress[opmenuoldal] = nailfile; break;
  }
  // Kimentjük a Flash címet és az EREDETI méretet
  const int16_t* flash_ptr = genstartadress[opmenuoldal];
  uint32_t original_sample_count = sizes[PCMWaveNo[opmenuoldal]];
  uint32_t flash_size_bytes = original_sample_count << 1;

  if (flash_ptr != NULL) {
    uint32_t max_bytes = sizeof(fast_samples[0]);
    uint32_t copy_bytes = (flash_size_bytes > max_bytes) ? max_bytes : flash_size_bytes;

    // SRAM törlés és MÁSOLÁS
    memset(fast_samples[opmenuoldal], 0, max_bytes);
    memcpy(fast_samples[opmenuoldal], flash_ptr, copy_bytes);

    // Átirányítjuk a generátort az SRAM-ra
    genstartadress[opmenuoldal] = fast_samples[opmenuoldal];

    // Itt a trükk: a 'samplesize' és 'sampleend' változókat állítjuk be a másolt méretre,
    // de az eredeti 'sizes' tömböt NEM bántjuk, hogy megmaradjon az információ.
    samplesize[opmenuoldal] = copy_bytes >> 1;
    sampleend[opmenuoldal] = samplesize[opmenuoldal];
  }

  // A setsamplesize() hívás már nem is kell, mert fentebb beállítottuk kézzel a RAM-hoz

}

//-----------LFO-Waveform-----------------------
void updateLFOAdresses() {
  for (int i = 0; i < LFOnumber; i++) {
    switch (LFO_Wave_Select[i]) {
      case 0: LFOadress[i] = lfosine;   break;
      case 1: LFOadress[i] = lfotriangle; break;
      case 2: LFOadress[i] = lfosaw;      break;
      case 3: LFOadress[i] = lforandom;   break;
      case 4: LFOadress[i] = lfosine;   break;
      default: LFOadress[i] = lfosine;  break;
    }
  }
}

//--------------LCD-------------------------------
String lcdBuffer[2] = {"                ", "                "};

void lcdprint(String ujSzoveg, byte sor) {
  if (!LCD_ON) return;

  // 1. Kiegészítjük a bejövő szöveget szóközökkel 16 karakterre
  while (ujSzoveg.length() < 16) {
    ujSzoveg += " ";
  }
  // Ha hosszabb lenne, levágjuk (biztonság kedvéért)
  if (ujSzoveg.length() > 16) {
    ujSzoveg = ujSzoveg.substring(0, 16);
  }

  // 2. Karakterenkénti összehasonlítás
  for (int i = 0; i < 16; i++) {
    // Csak akkor küldjük ki, ha az adott pozíción lévő karakter megváltozott
    if (ujSzoveg[i] != lcdBuffer[sor][i]) {
      lcd.setCursor(i, sor);
      lcd.write(ujSzoveg[i]); // A .write() gyorsabb, mint a .print() egy karakternél
      lcdBuffer[sor][i] = ujSzoveg[i]; // Frissítjük a belső buffert
    }
  }
}

String lcdprint2(int cc)
{
  if (LCD_ON)
  {
    String eredmeny = "";
    eredmeny += (cc % 100) / 10;
    eredmeny += cc % 10;
    return eredmeny;
  }
}
String lcdprint3(int cc)
{
  if (LCD_ON)
  {
    String eredmeny = "";
    eredmeny += cc / 100;
    eredmeny += (cc % 100) / 10;
    eredmeny += cc % 10;
    return eredmeny;
  }
}

//--------------MIDI SYSEX PARAMETER CONTROL------
int debugCounter = 0;
String line = "";
void parametersysexchanged() {
  //byte step = 1;
  byte value = velocityByte;
  // debugCounter++;
  //Serial.print("Param betöltés: "); Serial.println(debugCounter);
  if (localParameterByte == 0)
    switch (noteByte) {
      case 0:
        //couarse u1
        COARSE[2] = value;
        line = "U1: COARSE=" + lcdprint3(COARSE[2]) + "  ";
        notetune();
        break;
      case 1:
        //couarse u1
        FINE[2] = value;
        line = "U1: COARSE=" + lcdprint3(FINE[2]) + "  ";
        notetune();
        break;
      case 2:
        KEYFollow[2] = value;
        line = "U1: KEYFollow=" + lcdprint2(KEYFollow[2]);
        notetune();
        break;
      case 3:

        break;
      case 4:
        TVA[2] = value;
        line = "U1: TVA=" + lcdprint3(TVA[2]) + "     ";
        break;
      case 6:
        Waveform[2] = value;
        if (value == 0) {
          line = "U1: Waveform=Sqr";
        }
        if (value == 1) {
          line = "U1: Waveform=Saw";
        }
        break;
      case 7:
        PCMWaveNo[2] = value;
        line = "U1: PCMWavNo=" + lcdprint3(PCMWaveNo[2]);
        opmenuoldal = 2;
        setPCMWave();
        break;
      case 8:
        PW[2] = value;
        line = "U1: PW=" + lcdprint3(PW[2]) + "      ";
        break;
      case 10:
        PWMLFO[2] = value;
        line = "U1: PWMLFO=" + lcdprint3(PWMLFO[2]) + "  ";
        break;
      case 11:
        PWMLFODepth[2] = value;
        line = "U1: PLFODPT=" + lcdprint3(PWMLFODepth[2]) + " ";
        break;
      case 13:
        tvf_cutoff[2] = value;
        line = "U1: TVFCUTOF=" + lcdprint3(tvf_cutoff[2]);
        break;
      case 14:
        tvf_reso[2] = value;
        line = "U1: TVF RESO=" + lcdprint3(tvf_reso[2]);
        break;
      case 18:
        // Tároljuk az eredeti MIDI értéket (0-127), hogy ne vesszen el a felbontás
        tvf_env_depth[6] = value;
        line = "U1: TVF Dpth=" + lcdprint3(value);
        break;
      case 22: ENV_T1[6] = 100 - value; line = "U1: TVF_T1=" + lcdprint3(ENV_T1[6]); break;
      case 23: ENV_T2[6] = 100 - value; line = "U1: TVF_T2=" + lcdprint3(ENV_T2[6]); break; // Figyeld az L2-t, ha ez maradt a címke!
      case 24: ENV_T3[6] = 100 - value; line = "U1: TVF_T3=" + lcdprint3(ENV_T3[6]); break;
      case 25: ENV_T4[6] = 100 - value; line = "U1: TVF_T4=" + lcdprint3(ENV_T4[6]); break;
      case 26: ENV_T5[6] = 100 - value; line = "U1: TVF_T5=" + lcdprint3(ENV_T5[6]); break;
      case 27: ENV_L1[6] = value; line = "U1: TVF_L1=" + lcdprint3(ENV_L1[6]); break;
      case 28: ENV_L2[6] = value; line = "U1: TVF_L2=" + lcdprint3(ENV_L2[6]); break;
      case 29: ENV_L3[6] = value; line = "U1: TVF_L3=" + lcdprint3(ENV_L3[6]); break;
      case 30: ENV_LSUS[6] = value; line = "U1: TVF_L4=" + lcdprint3(ENV_LSUS[6]); break;
      case 31: ENV_LEND[6] = value; line = "U1: TVF_L5=" + lcdprint3(ENV_LEND[6]); break;


      case 32:
        TWFLFO[2] = value;
        line = "U1: TWFLFO=" + lcdprint3(TWFLFO[2]) + "   ";
        break;
      case 33:
        TVF_LFO_level[2] = value;
        line = "U1: TVFLFOL=" + lcdprint3(TVF_LFO_level[2]) + " ";
        break;
      case 35:
        volume[2] = value;
        line = "U1: Level=" + lcdprint3(volume[2]) + "   ";
        break;
      case 37:
        BiasPoint[2] = value;
        line = "U1: BisPont=" + lcdprint3(BiasPoint[2]);
        notebias();
        break;
      case 38:
        BiasLevel[2] = value;
        line = "U1: BieasLev=" + lcdprint3(BiasLevel[2]);
        notebias();
        break;
      case 39:
        ENV_T1[2] = 100 - value;
        line = "U1: ENV T1=" + lcdprint3(ENV_T1[2]) + "  ";
        break;
      case 40:
        ENV_T2[2] = 100 - value;
        line = "U1: ENV_T2=" + lcdprint3(ENV_T2[2]) + "  ";
        break;
      case 41:
        ENV_T3[2] = 100 - value;
        line = "U1: ENV_T3=" + lcdprint3(ENV_T3[2]) + "  ";
        break;
      case 42:
        ENV_T4[2] = 100 - value;
        line = "U1: ENV_T4=" + lcdprint3(ENV_T4[2]) + "  ";
        break;
      case 43:
        /*
          step = samplesize[2] / 100;
          sampleend[2] = value * step;
          Serial.println("SAMPLE END U1: " + String(sampleend[2]));
        */
        ENV_T5[2] = 100 - value;
        line = "U1: ENV_T5=" + lcdprint3(ENV_T5[2]) + "  ";
        break;
      case 44:
        ENV_L1[2] = value;
        line = "U1: ENV_L1" + lcdprint3(ENV_L1[2]) + "  ";
        break;
      case 45:
        ENV_L2[2] = value;
        line = "U1: ENV_L2=" + String(ENV_L2[2]) + "  ";
        break;
      case 46:
        /*
          step = samplesize[2] / 100;
          samplebegin[2] = value * step;
          Serial.println("SAMPLE BEGIN U1: " + String(samplebegin[2]));
        */
        ENV_L3[2] = value;
        line = "U1: ENV_L3=" + lcdprint3(ENV_L3[2]) + "  ";
        break;
      case 47:
        ENV_LSUS[2] = value;
        line = "U1: ENV_LSUS=" + lcdprint3(ENV_LSUS[2]) + "  ";
        break;
      case 48:
        ENV_LEND[2] = value;
        line = "U1: ENV_LEND=" + lcdprint3(ENV_LEND[2]) + "  ";
        break;
      case 49:
        opmenuoldal = 2;
        if (value == 0) {

          loopsample[opmenuoldal] = false;
          line = "loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]);
        }
        if (value == 1) {
          loopsample[opmenuoldal] = true;
          line = "loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]);
        }
        break;
      case 51:
        PICHLFO[2] = value;
        line = "U1: PICH LFO=" + lcdprint3(PICHLFO[2]);
        break;
      case 52:
        PICH_LFO_level[2] = value;
        line = "U1: PLFO LEV=" + lcdprint3(PICH_LFO_level[2]);
        break;
      case 64:
        //couarse u2
        COARSE[3] = value;
        line = "U2: COARSE=" + lcdprint3(COARSE[3]) + "  ";
        notetune();
        break;
      case 65:
        //couarse u2
        FINE[3] = value;
        line = "U2: COARSE=" + lcdprint3(FINE[3]) + "  ";
        notetune();
        break;
      case 66:
        KEYFollow[3] = value;
        line = "U2: KEYFolow=" + lcdprint2(KEYFollow[3]);
        notetune();
        break;
      case 67:
        break;
      case 68:
        TVA[3] = value;
        line = "U2: TVA=" + lcdprint3(TVA[3]) + "   ";
        break;
      case 70:
        Waveform[3] = value;
        if (value == 0) {
          line = "U2: Waveform=Square";
        }
        if (value == 1) {
          line = "U2: Waveform=Sawtooth";
        }
        break;
      case 71:
        PCMWaveNo[3] = value;
        line = "U2: PCMWaveNo=" + lcdprint3(PCMWaveNo[3]);
        opmenuoldal = 3;
        setPCMWave();
        //lcd
        break;
      case 72:
        PW[3] = value;
        line = "U2: PW=" + lcdprint3(PW[3]);
        break;
      case 74:
        PWMLFO[3] = value;
        line = "U2: PWMLFO=" + lcdprint3(PWMLFO[3]);
        break;
      case 75:
        PWMLFODepth[3] = value;
        line = "U2: PLFODPT=" + lcdprint3(PWMLFODepth[3]);
        break;
      case 77:
        tvf_cutoff[3] = value;
        line = "U2: TVF_CUTOFF=" + lcdprint3(tvf_cutoff[3]);
        break;
      case 78:
        tvf_reso[3] = value;
        line = "U2: TVF_RESO=" + lcdprint3(tvf_reso[3]);
        break;
      case 82:
        tvf_env_depth[7] = value; // pl. 0-100 közötti érték
        line = "U2 TVF Depth: " + String(value);
        break;
      // --- TVF 4 (index 7) ---
      case 86: ENV_T1[7] = 100 - value; line = "U2: TVF_T1=" + lcdprint3(ENV_T1[7]); break;
      case 87: ENV_T2[7] = 100 - value; line = "U2: TVF_T2=" + lcdprint3(ENV_T2[7]); break;
      case 88: ENV_T3[7] = 100 - value; line = "U2: TVF_T3=" + lcdprint3(ENV_T3[7]); break;
      case 89: ENV_T4[7] = 100 - value; line = "U2: TVF_T4=" + lcdprint3(ENV_T4[7]); break;
      case 90: ENV_T5[7] = 100 - value; line = "U2: TVF_T5=" + lcdprint3(ENV_T5[7]); break;
      case 91: ENV_L1[7] = value; line = "U2: TVF_L1=" + lcdprint3(ENV_L1[7]); break;
      case 92: ENV_L2[7] = value; line = "U2: TVF_L2=" + lcdprint3(ENV_L2[7]); break;
      case 93: ENV_L3[7] = value; line = "U2: TVF_L3=" + lcdprint3(ENV_L3[7]); break;
      case 94: ENV_LSUS[7] = value; line = "U2: TVF_L4=" + lcdprint3(ENV_LSUS[7]); break;
      case 95: ENV_LEND[7] = value; line = "U2: TVF_L5=" + lcdprint3(ENV_LEND[7]); break;



      case 96:
        TWFLFO[3] = value;
        line = "U2: TWFLFO=" + lcdprint3(TWFLFO[3]);
        break;
      case 97:
        TVF_LFO_level[3] = value;
        line = "U2 TVFLFOL=" + lcdprint3(TVF_LFO_level[3]);
        break;
      case 99:
        volume[3] = value;
        line = "U2: Level=" + lcdprint3(volume[3]);
        break;
      case 100:
        //velocity[3]=ropi;

        ropi = value;

        line = "U2: Velocity=" + lcdprint3(ropi);
        notebias();
        break;

      case 101:
        BiasPoint[3] = value;
        line = "L2: BiasPoint=" + lcdprint3(BiasPoint[3]);
        notebias();
        break;
      case 102:
        BiasLevel[3] = value;
        line = "L1: bieasLevel=" + lcdprint3(BiasLevel[3]);
        notebias();
        break;
      case 103:
        ENV_T1[3] = 100 - value;
        line = "U2: ENV_T1=" + lcdprint3(ENV_T1[3]);
        break;
      case 104:
        ENV_T2[3] = 100 - value;
        line = "U2: ENV_T2=" + lcdprint3(ENV_T2[3]);
        break;
      case 105:
        ENV_T3[3] = 100 - value;
        line = "U2: ENV_T3=" + lcdprint3(ENV_T3[3]);
        break;
      case 106:
        ENV_T4[3] = 100 - value;
        line = "U2: ENV_T4=" + lcdprint3(ENV_T4[3]);
        break;
      case 107:
        /*
          step = samplesize[3] / 100;
          sampleend[3] = value * step;
          Serial.println("SAMPLE END U2: " + String(sampleend[3]));
        */
        ENV_T5[3] = 100 - value;
        line = "U2: ENV_T5=" + lcdprint3(ENV_T5[3]);
        break;
      case 108:
        ENV_L1[3] = value;
        line = "U2: ENV_L1=" + lcdprint3(ENV_L1[3]);
        break;
      case 109:
        ENV_L2[3] = value;
        line = "U2: ENV_L2=" + lcdprint3(ENV_L2[3]);
        break;
      case 110:
        /*
          step = samplesize[3] / 100;
          samplebegin[3] = value * step;
          Serial.println("SAMPLE BEGIN L1: " + String(samplebegin[3]));
        */
        ENV_L3[3] = value;
        line = "U2: ENV_L3=" + lcdprint3(ENV_L3[3]);
        break;
      case 111:
        ENV_LSUS[3] = value;
        line = "U2: ENV_LSUS=" + lcdprint3(ENV_LSUS[3]);
        break;
      case 112:
        ENV_LEND[3] = value;
        line = "U2: ENV_LEND=" + lcdprint3(ENV_LEND[3]);
        break;
      case 113:
        opmenuoldal = 3;
        if (value == 0) {

          loopsample[opmenuoldal] = false;
          line = "loopsample" + lcdprint3(opmenuoldal) + ": " + String(loopsample[opmenuoldal]);
        }
        if (value == 1) {
          loopsample[opmenuoldal] = true;
          line = "loopsample" + lcdprint3(opmenuoldal) + ": " + String(loopsample[opmenuoldal]);
        }
        break;
      case 115:
        PICHLFO[3] = value;
        line = "U2: PICH_LFO=" + lcdprint3(PICHLFO[3]);
        break;
      case 116:
        PICH_LFO_level[3] = value;
        line = "U2: PICH_LFOL=" + lcdprint3(PICH_LFO_level[3]);
        break;
      default:
        line = "No implemented" + String(localParameterByte) + " " + String(noteByte);
        return;
        break;
    }

  if (localParameterByte == 1)
    switch (noteByte) {
      case 1:
        break;
      case 3:
        LFOMode[0] = value;
        line = "L1: LFOMode=" + lcdprint3(LFOMode[0]);
        break;
      case 4:
        PENVMode[0] = value;
        line = "L1: LFOMode=" + lcdprint3(PENVMode[0]);
        break;
      case 5:
        BENDERMode[0] = value;
        line = "L1:  BENDERMode=" + lcdprint3(BENDERMode[0] );
        break;
      case 6:
        break;
      case 10:
        STRUCTURE_U = value;
        STRUCTURE = (STRUCTURE_L * 10) + STRUCTURE_U;
        line = "STRUCTURE_U: " + lcdprint3(STRUCTURE_U );
        break;
      case 22:
        f02orig = expgains128[value] >> 1 + 1;
        f02 = f02orig;
        eqkiszamol2();
        // Serial.println("f02orig: " + String(f02orig));
        break;
      case 23:
        Q2 = value / 10.0;
        eqkiszamol2();
        // Serial.println("Q2: " + String(Q2));
        break;
      case 24:
        eqlevel2 = value;
        //  Serial.println("eqlevel2: " + String(eqlevel2));
        break;
      case 25:
        LFO_Wave_Select[3] = value;
        line = "LFO4_Wave=" + lcdprint3(LFO_Wave_Select[3]);
        updateLFOAdresses();
        break;
      case 26:
        lfofreq[3] = value;
        line = " lfofreq3: " + lcdprint3(lfofreq[3]);
        break;
      case 27:
        lfolevel[3] = value;
        line = " lfofreq3: " + lcdprint3(lfolevel[3]);
        break;
      case 28:
        lfo3sync = value;
        line = " lfofreq3: " + lcdprint3(lfo3sync);
        break;
      case 29:
        LFO_Wave_Select[4] = value;
        line = "LFO5_Wave=" + lcdprint3(LFO_Wave_Select[4]);
        updateLFOAdresses();
        break;
      case 30:
        lfofreq[4] = value;
        line = " lfofreq4: " + lcdprint3(lfofreq[4]);
        break;
      case 31:
        LFO_Delay[4] = value;
        line = " LFO5_DELAY: " + lcdprint3(LFO_Delay[4]);
        break;
      case 33:
        LFO_Wave_Select[5] = value;
        line = "LFO6_Wave=" + lcdprint3(LFO_Wave_Select[5]);
        updateLFOAdresses();
        break;
      case 34:
        lfofreq[5] = value;
        line = " lfofreq5: " + lcdprint3(lfofreq[5]);
        break;
      case 35:
        LFO_Delay[5] = value;
        line = " LFO6_DELAY: " + lcdprint3(LFO_Delay[5]);
        break;


      case 42:
        switch (value) {
          case 0: maskRight = 255;  LFOadress[7] = lfosine;     break;
          case 1: maskRight = 511;  LFOadress[7] = lfotriangle; break;
          case 2: maskRight = 383;  LFOadress[7] = lfosine;     break;
          case 3: maskRight = 383;  LFOadress[7] = lfotriangle; break;
          case 4: maskRight = 255;  LFOadress[7] = lfosine;     break;
          case 5: maskRight = 255;  LFOadress[7] = lfotriangle; break;
          case 6: maskRight = 127;  LFOadress[7] = lfosine;     break;
          case 7: maskRight = 127;  LFOadress[7] = lfotriangle; break;
          default: return;
        }
        memset(chorusbufferright, 0, sizeof(chorusbufferright));
        chorusbufferindex2 = 0;
        break;
      case 43:
        lfofreq[7] = value;
        line = "U: CHORUSFREQ=" + lcdprint3(lfofreq[7] );
        break;
      case 44:
        chorusLevelRight = value;
        line = "U: CHORUSLEVEL=" + lcdprint3(chorusLevelRight );
        break;
      case 45:
        reverbdiffusion = value;
        line = "Reverbdiff=" + lcdprint3(reverbdiffusion);
        break;
      case 64:
        COARSE[0] = value;
        line = "L1: COARSE=" + lcdprint3(COARSE[0]);
        notetune();
        break;
      case 65:
        FINE[0] = value;
        line = "L1: FINE=" + lcdprint3(FINE[0]);
        notetune();
        break;
      case 66:
        KEYFollow[0] = value;
        line = "L1: KEYFollow=" + lcdprint2(KEYFollow[0]);
        notetune();
        break;
      case 68:
        TVA[0] = value;
        line = "L1: TVA ON-OFF=" + lcdprint3(TVA[0]);
        break;
      case 70:
        Waveform[0] = value;
        if (value == 0) {
          line = "L1: Waveform=Square";
        }
        if (value == 1) {
          line = "L1: Waveform=Sawtooth";
        }
        //lcd
        break;
      case 71:

        PCMWaveNo[0] = value;
        line = "L1: PCMWaveNo=" + lcdprint3(PCMWaveNo[0]);
        opmenuoldal = 0;
        setPCMWave();
        //lcd
        break;
      case 72:
        PW[0] = value;
        line = "L1: PW=" + lcdprint3(PW[0]);
        break;
      case 74:
        PWMLFO[0] = value;
        line = "L1: PWMLFO=" + lcdprint3(PWMLFO[0]);
        break;
      case 75:
        PWMLFODepth[0] = value;
        line = "L1: PLFODPT=" + lcdprint3(PWMLFODepth[0]);
        break;
      case 77:
        tvf_cutoff[0] = value;
        line = "L1: TVF_CUTEOFF=" + lcdprint3(tvf_cutoff[0]);
        break;
      case 78:
        tvf_reso[0] = value;
        line = "L1: TVF_RESO=" + lcdprint3(tvf_reso[0]);
        break;
      case 82:
        // Itt nincs float, nincs szorzás, csak nyers érték (0-127)
        tvf_env_depth[4] = value;
        line = "L1 TVF Depth: " + String(value);
        break;

      case 86:
        ENV_T1[4] = 100 - value;
        line = "L1: TVF_T1=" + lcdprint3(ENV_T1[4]);
        break;
      case 87:
        ENV_T2[4] = 100 - value;
        line = "L1: TVF_T2=" + lcdprint3(ENV_T2[4]);
        break;
      case 88:
        ENV_T3[4] = 100 - value;
        line = "L1: TVF_T3=" + lcdprint3(ENV_T3[4]);
        break;
      case 89:
        ENV_T4[4] = 100 - value;
        line = "L1: TVF_T4=" + lcdprint3(ENV_T4[4]);
        break;
      case 90:
        ENV_T5[4] = 100 - value;
        line = "L1: TVF_T5=" + lcdprint3(ENV_T5[4]);
        break;
      case 91: ENV_L1[4] = value; line = "L1: TVF_L1=" + lcdprint3(ENV_L1[4]); break;
      case 92: ENV_L2[4] = value; line = "L1: TVF_L2=" + lcdprint3(ENV_L2[4]); break;
      case 93: ENV_L3[4] = value; line = "L1: TVF_L3=" + lcdprint3(ENV_L3[4]); break;
      case 94: ENV_LSUS[4] = value; line = "L1: TVF_L4=" + lcdprint3(ENV_LSUS[4]); break;
      case 95: ENV_LEND[4] = value; line = "L1: TVF_L5=" + lcdprint3(ENV_LEND[4]); break;



      case 96:
        TWFLFO[0] = value;
        line = "L1: TWFLFO=" + lcdprint3(TWFLFO[0]);
        break;
      case 97:
        TVF_LFO_level[0] = value;
        line = "L1 TVFLFOL=" + lcdprint3(TVF_LFO_level[0]);
        break;
      case 99:
        volume[0] = value;
        line = "L1: Level=" + lcdprint3(volume[0]);
        break;
      case 101:
        BiasPoint[0] = value;
        line = "L1: BiasPoint=" + lcdprint3(BiasPoint[0]);
        notebias();
        break;
      case 102:
        BiasLevel[0] = value;
        line = "L1: BieasLevel=" + lcdprint3(BiasLevel[0]);
        notebias();
        break;
      case 103:
        ENV_T1[0] = 100 - value;
        line = "L1: ENV_T1=" + lcdprint3(ENV_T1[0]);
        break;
      case 104:
        ENV_T2[0] = 100 - value;
        line = "L1: ENV_T2=" + lcdprint3(ENV_T2[0]);
        break;
      case 105:
        ENV_T3[0] = 100 - value;
        line = "L1: ENV_T3=" + lcdprint3(ENV_T3[0]);
        break;
      case 106:
        ENV_T4[0] = 100 - value;
        line = "L1 ENV_T4=" + lcdprint3(ENV_T4[0]);
        break;
      case 107:
        ENV_T5[0] = 100 - value;
        line = "L1 ENV_T5=" + lcdprint3(ENV_T5[0]);
        break;
      case 108:
        ENV_L1[0] = value;
        line = "L1: ENV_L1=" + lcdprint3(ENV_L1[0]);
        break;
      case 109:
        ENV_L2[0] = value;
        line = "L1: ENV_L2=" + lcdprint3(ENV_L2[0]);
        break;
      case 110:
        ENV_L3[0] = value;
        line = "L1: ENV_L3=" + lcdprint3(ENV_L3[0]);
        break;
      case 111:
        ENV_LSUS[0] = value;
        line = "L1: ENV_LSUS=" + lcdprint3(ENV_LSUS[0]);
        break;
      case 112:
        ENV_LEND[0] = value;
        line = "L1: ENV_LEND=" + lcdprint3(ENV_LEND[0]);
        break;
      case 113:
        opmenuoldal = 0;
        if (value == 0) {

          loopsample[opmenuoldal] = false;
          line = "loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]);
        }
        if (value == 1) {
          loopsample[opmenuoldal] = true;
          line = "loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]);
        }
        break;
      case 114:
        if (value == 0) {
          TVA_Slide = 16;
        }
        if (value == 1) {
          TVA_Slide = 17;
        }
        if (value == 2) {
          TVA_Slide = 18;
        }
        if (value == 3) {
          TVA_Slide = 19;
        }
        if (value == 4) {
          TVA_Slide = 20;
        }
        line = "G: TVA_SLIDE=" + lcdprint3(TVA_Slide);
        break;
      case 115:
        PICHLFO[0] = value;
        line = "L1: PICH_LFO=" + lcdprint3(PICHLFO[0]);
        break;
      case 116:
        PICH_LFO_level[0] = value;
        line = "L1: PICH_LFOL=" + lcdprint3(PICH_LFO_level[0]);
        break;
      default:
        line = "No implemented" + String(localParameterByte) + " " + String(noteByte);
        return;
        break;
    }

  if (localParameterByte == 2) {
    switch (noteByte) {
      case 0:
        COARSE[1] = value;
        line = "L2: COARSE=" + lcdprint3(COARSE[1]);
        notetune();
        break;
      case 1:
        FINE[1] = value;
        line = "L2:  FINE=" + lcdprint3(FINE[1]);
        notetune();
        break;
      case 2:
        KEYFollow[1] = value;
        line = "L2: KEYFollow=" + lcdprint2(KEYFollow[1]);
        notetune();
        break;
      case 4:
        TVA[1] = value;
        line = "L2: TVA=" + lcdprint3(TVA[1]);
        break;
      case 6:
        Waveform[1] = value;
        if (value == 0) {
          line = "L2: Waveform=Square";
        }
        if (value == 1) {
          line = "L2: Waveform=Sawtooth";
        }
        break;
      case 7:
        opmenuoldal = 1;
        PCMWaveNo[1] = value;
        line = "L2: PCMWaveNo=" + lcdprint3(PCMWaveNo[1]);
        setPCMWave();
        break;
      case 8:
        PW[1] = value;
        line = "L2: PW=" + lcdprint3(PW[1]);
        break;
      case 10:
        PWMLFO[1] = value;
        line = "L2: PWMLFO=" + lcdprint3(PWMLFO[1]);
        break;
      case 11:
        PWMLFODepth[1] = value;
        line = "L2: PLFODPT=" + lcdprint3(PWMLFODepth[1]);
        break;
      case 13:
        tvf_cutoff[1] = value;
        line = "L1: TVF_CUTOFF=" + lcdprint3(tvf_cutoff[1]);
        break;
      case 14:
        tvf_reso[1] = value;
        line = "L2: TVF_RESO=" + lcdprint3(tvf_reso[1]);
        break;
      case 18:
        // Itt nincs float, nincs szorzás, csak nyers érték (0-127)
        tvf_env_depth[5] = value;
        line = "L2 TVF Depth: " + String(value);
        break;
      case 22: ENV_T1[5] = 100 - value; line = "L2: TVF_T1=" + lcdprint3(ENV_T1[5]); break;
      case 23: ENV_T2[5] = 100 - value; line = "L2: TVF_T2=" + lcdprint3(ENV_T2[5]); break;
      case 24: ENV_T3[5] = 100 - value; line = "L2: TVF_T3=" + lcdprint3(ENV_T3[5]); break;
      case 25: ENV_T4[5] = 100 - value; line = "L2: TVF_T4=" + lcdprint3(ENV_T4[5]); break;
      case 26: ENV_T5[5] = 100 - value; line = "L2: TVF_T5=" + lcdprint3(ENV_T5[5]); break;
      case 27: ENV_L1[5] = value; line = "L2: TVF_L1=" + lcdprint3(ENV_L1[5]); break;
      case 28: ENV_L2[5] = value; line = "L2: TVF_L2=" + lcdprint3(ENV_L2[5]); break;
      case 29: ENV_L3[5] = value; line = "L2: TVF_L3=" + lcdprint3(ENV_L3[5]); break;
      case 30: ENV_LSUS[5] = value; line = "L2: TVF_L4=" + lcdprint3(ENV_LSUS[5]); break;
      case 31: ENV_LEND[5] = value; line = "L2: TVF_L5=" + lcdprint3(ENV_LEND[5]); break;



      case 32:
        TWFLFO[1] = value;
        line = "L2: TWFLFO=" + lcdprint3(TWFLFO[1]);
        break;
      case 33:
        TVF_LFO_level[1] = value;
        line = "L2 TVFLFOL=" + lcdprint3(TVF_LFO_level[1]);
        break;
      case 35:
        volume[1] = value;
        line = "L2: Level=" + lcdprint3(volume[1]);
        break;
      case 37:
        BiasPoint[1] = value;
        line = "BiasPoint L2: " + lcdprint3(BiasPoint[1]);
        notebias();
        break;
      case 38:
        BiasLevel[1] = value;
        line = "L2: bieasLevel=" + lcdprint3(BiasLevel[1]);
        notebias();
        break;
      case 39:
        ENV_T1[1] = 100 - value;
        line = "L2: ENV_T1=" + lcdprint3(ENV_T1[1]);
        break;
      case 40:
        ENV_T2[1] = 100 - value;
        line = "L2: ENV_T2=" + lcdprint3(ENV_T2[1]);
        break;
      case 41:
        ENV_T3[1] = 100 - value;
        line = "L2: ENV_T3=" + lcdprint3(ENV_T3[1]);
        break;
      case 42:
        ENV_T4[1] = 100 - value;
        line = "L2: ENV_T4=" + lcdprint3(ENV_T4[1]);
        break;
      case 43:
        /*
          step = samplesize[1] / 100;
          sampleend[1] = value * step;
          Serial.println("SAMPLE END L2: " + String(sampleend[1]));
        */
        ENV_T5[1] = 100 - value;
        line = "L2: ENV_T5=" + lcdprint3(ENV_T5[1]);
        break;
      case 44:
        ENV_L1[1] = value;
        line = "L2: ENV_L1=" + lcdprint3(ENV_L1[1]);
        break;
      case 45:
        ENV_L2[1] = value;
        line = "L2: ENV_L2=" + lcdprint3(ENV_L2[1]);
        break;
      case 46:
        /*
          step = samplesize[1] / 100;
          samplebegin[1] = value * step;
          Serial.println("SAMPLE BEGIN L2: " + String(samplebegin[1]));
        */
        ENV_L3[1] = value;
        line = "L2: ENV_L3=" + lcdprint3(ENV_L3[1]);
        break;
      case 47:
        ENV_LSUS[1] = value;
        line = "L2: ENV_LSUS=" + lcdprint3(ENV_LSUS[1]);
        break;
      case 48:
        ENV_LEND[1] = value;
        line = "L2: ENV_LEND=" + lcdprint3(ENV_LEND[1]);
        break;
      case 49:
        opmenuoldal = 1;
        if (value == 0) {

          loopsample[opmenuoldal] = false;
          line = "loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]);
        }
        if (value == 1) {
          loopsample[opmenuoldal] = true;
          line = "loopsample" + String(opmenuoldal) + ": " + String(loopsample[opmenuoldal]);
        }
        break;
      case 50:
        /*
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
        */
        break;
      case 51:
        PICHLFO[1] = value;
        line = "L2: PICH_LFO=" + lcdprint3(PICHLFO[1]);
        break;
      case 52:
        PICH_LFO_level[1] = value;
        line = "L2: PICH_LFOL=" + lcdprint3(PICH_LFO_level[1]);
        break;
      case 74:
        STRUCTURE_L = value;
        STRUCTURE = (STRUCTURE_L * 10) + STRUCTURE_U;
        line = "STRUCTURE_L: " + lcdprint3(STRUCTURE_L);
        break;
      case 86:
        f0orig = expgains128[value] >> 1 + 1;
        f0 = f0orig;
        eqkiszamol();
        // Serial.println("f0orig: " + String(f0orig));
        break;
      case 87:
        Q = value / 10.0;
        eqkiszamol();
        //Serial.println("Q: " + String(Q));
        break;
      case 88:
        eqlevel = value;
        //Serial.println("eqlevel: " + String(eqlevel));
        break;
      case 89:
        LFO_Wave_Select[0] = value;
        line = "LFO1_Wave=" + lcdprint3(LFO_Wave_Select[0]);
        updateLFOAdresses();
        break;
      case 90:
        lfofreq[0] = value;
        line = " lfofreq0: " + lcdprint3(lfofreq[0]);
        break;
      case 91:
        lfolevel[0] = value;
        line = " lfofreq0: " + lcdprint3(lfolevel[0]);
        break;
      case 92:
        lfo2sync = value;
        line = " lfo2sync: " + lcdprint3(lfo2sync);
        break;
      case 93:
        LFO_Wave_Select[1] = value;
        line = "LFO2_Wave=" + lcdprint3(LFO_Wave_Select[1]);
        updateLFOAdresses();
        break;
      case 94:
        lfofreq[1] = value;
        line = " lfofreq1: " + lcdprint3(lfofreq[1]);
        break;
      case 95:
        LFO_Delay[1] = value;
        line = " LFO2_DELAY: " + lcdprint3(LFO_Delay[1]);
        break;
      case 97:
        LFO_Wave_Select[2] = value;
        line = "LFO3_Wave=" + lcdprint3(LFO_Wave_Select[2]);
        updateLFOAdresses();
        break;
      case 98:
        lfofreq[2] = value;
        line = " lfofreq2: " + lcdprint3(lfofreq[2]);
        break;
      case 99:
        LFO_Delay[2] = value;
        line = " LFO3_DELAY: " + lcdprint3(LFO_Delay[2]);
        break;
      case 101: // "Ceiling" - 0-tól 15-ig
        {
          float norm = value / 15.0f; // 15-nél lesz pontosan 1.0
          ceiling_val = (norm * norm) * 0.98f;

          if (ceiling_val < 0.015f) ceiling_val = 0.015f;
        }
        break;

      case 102: // "Stretch" - 0-tól 24-ig
        {
          // 24 / 24.0f = 1.0 -> a négyzete is 1.0
          float norm = value / 24.0f;

          // A 0.00025f szorzóval a 24-es állásnál pont a
          // stabilitási határ szélén fog táncolni (brutál vonyítás)
          stretch_val = (norm * norm) * 0.00025f;

          if (stretch_val < 0.00001f) stretch_val = 0.00001f;
        }
        break;
      case 106:
        switch (value) {
          case 0: maskLeft = 255;  LFOadress[6] = lfosine;     break; // A régi kedvenc (Standard)
          case 1: maskLeft = 511;  LFOadress[6] = lfotriangle; break; // A régi kedvenc (Tri)
          case 2: maskLeft = 383;  LFOadress[6] = lfosine;     break; // Egyedi méret (Stabil marad)
          case 3: maskLeft = 383;  LFOadress[6] = lfotriangle; break;
          case 4: maskLeft = 255;  LFOadress[6] = lfosine;     break; // Ez az 5-ösöd, ami most jó
          case 5: maskLeft = 255;  LFOadress[6] = lfotriangle; break;
          case 6: maskLeft = 127;  LFOadress[6] = lfosine;     break; // Ez a 7-esed, ami most jó
          case 7: maskLeft = 127;  LFOadress[6] = lfotriangle; break;
        }
        memset(chorusbufferleft, 0, sizeof(chorusbufferleft));
        chorusbufferindex = 0;
        break;


      case 107:
        // chorusRate=value;
        lfofreq[6] = value;
        line = "L: Chorus RATE=" + lcdprint3(lfofreq[6]);
        break;
      case 108:
        chorusLevelLeft = value;
        line = "U: Chorus LEVEL=" + lcdprint3( chorusLevelLeft);
        break;
      default:
        line = "No implemented" + String(localParameterByte) + " " + String(value);
        // Serial.println(line);
        return;
        break;
    }
  }
  if (localParameterByte == 3) {
    if (noteByte >= 0 && noteByte <= 13) {
      int charIndex = noteByte; // 1-es noteByte -> 0-s index, 13-as noteByte -> 12-es index

      char c = ' ';
      if (value == 0) {
        c = ' '; // Szóköz
      } else if (value >= 1 && value <= 26) {
        c = 'A' + (value - 1);  // Nagybetűk
      } else if (value >= 27 && value <= 52) {
        c = 'a' + (value - 27); // Kisbetűk
      } else if (value >= 52 && value <= 61) {
        c = '0' + (value - 52); // Számok (0-9)
      }

      pachname[charIndex] = c;
    }
    switch (noteByte) {
      case 18:
        if (value == 0) {
          // 1. MINDEN POLI (Standard szinti)
          oscMode[0] = 0; oscMode[1] = 0; oscMode[2] = 0; oscMode[3] = 0;
        }
        else if (value == 1) {
          // 2. LOWER POLI (0,1) + UPPER MONO (2,3) -> Szóló a jobb kézbe
          oscMode[0] = 0; oscMode[1] = 0; oscMode[2] = 1; oscMode[3] = 1;
        }
        else if (value == 2) {
          // 3. LOWER MONO (0,1) + UPPER POLI (2,3) -> Basszus a bal kézbe
          oscMode[0] = 1; oscMode[1] = 1; oscMode[2] = 0; oscMode[3] = 0;
        }
        else if (value == 3) {
          // 4. MINDEN MONO (Vastag Unisono-szerű mono)
          oscMode[0] = 1; oscMode[1] = 1; oscMode[2] = 1; oscMode[3] = 1;
        }
        else if (value == 4) {
          // 5. LOWER MIX (0=Poli, 1=Mono) + UPPER POLI (2,3)
          oscMode[0] = 0; oscMode[1] = 1; oscMode[2] = 0; oscMode[3] = 0;
        }
        else if (value == 5) {
          // 6. LOWER POLI (0,1) + UPPER MIX (2=Poli, 3=Mono)
          oscMode[0] = 0; oscMode[1] = 0; oscMode[2] = 0; oscMode[3] = 1;
        }
        else if (value == 6) {
          // 7. PÁRATLANOK MONO (0,2), PÁROSOK POLI (1,3)
          oscMode[0] = 1; oscMode[1] = 0; oscMode[2] = 1; oscMode[3] = 0;
        }
        else if (value == 7) {
          // 8. CSAK AZ ELSŐ OSC POLI (0), TÖBBI MONO (1,2,3)
          oscMode[0] = 0; oscMode[1] = 1; oscMode[2] = 1; oscMode[3] = 1;
        }
        break;
      case 22:
        UKeyShift = value;
        line = "U: KeyShift=" + lcdprint3(UKeyShift);
        break;
      case 23:
        LKeyShift = value;
        line = "L: KeyShift=" + lcdprint3(LKeyShift);
        break;
      case 25:
        GLOBAL_TUNE = 300 + value;
        notetune();
        Serial.println(String(GLOBAL_TUNE));
        break;
      case 27:
        step = value;
        switch (step) {
          case 7: GLOBAL_TUNE = 1; break;
          case 8: GLOBAL_TUNE = 2; break;
          case 9: GLOBAL_TUNE = 4; break;
          case 10: GLOBAL_TUNE = 7; break;
          case 11: GLOBAL_TUNE = 14; break;
          case 12: GLOBAL_TUNE = 28; break;
          case 13: GLOBAL_TUNE = 59; break;
          case 14: GLOBAL_TUNE = 118; break;
          case 15: GLOBAL_TUNE = 236; break;
          case 16: GLOBAL_TUNE = 333; break;//módosított korrekció 472-rol
          case 17: GLOBAL_TUNE = 944; break;
          case 18: GLOBAL_TUNE = 1888; break;
          case 19: GLOBAL_TUNE = 3776; break;
          case 20: GLOBAL_TUNE = 7552; break;
          case 21: GLOBAL_TUNE = 15104; break;
          case 22: GLOBAL_TUNE = 30208; break;
          case 23: GLOBAL_TUNE = 60416; break;
          case 24: GLOBAL_TUNE = 120832; break;
          default:
            return;
            break;
        }//
        notetune();
        line = "STEP WAVE PART=" + String(step);
        line = "GLOBAL_TUNE=" + String(GLOBAL_TUNE);
        break;
      case 28:
        portamento_time[0] = value;
        portamento_time[1] = value;
        portamento_time[2] = value;
        portamento_time[3] = value;
        break;
      case 29:
        //sync ofset!!! in oder controller!!!
        //OFFSET = value;
        //line = "SYNC_OFFCET=" + String(OFFSET);
        break;
      case 30:
        switch (value) {
          case 0:
            delaybuffersize = 337;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 45;
            reverbdiffusion = 2;
            line = "1. Small Hall";
            break;
          case 1:
            delaybuffersize = 1583;
            delaytime = 3;
            delay2time = 2;
            reverblevel = 50;
            line = "2. Medium Hall";
            break;
          case 2:
            delaybuffersize = 1024;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            line = "3. Large Hall";
            break;
          case 3:
            delaybuffersize = 2048;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            line = "4. Chapel";
            break;
          case 4:
            delaybuffersize = 4127;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 40;
            line = "5. Box";
            break;
          case 5:
            delaybuffersize = 211;
            delaytime = 2;
            delay2time = 1;
            reverblevel = 40;
            line = "6. Small Metal Room";
            break;
          case 6:
            delaybuffersize = 8191;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 40;
            line = "7. Small Room ";
            break;
          case 7:
            delaybuffersize = 8191;
            delaytime = 1;
            delay2time = 2;
            reverblevel = 40;
            line = "8. Small Room";
            break;
          case 8:
            delaybuffersize = 8192;
            delaytime = 2;
            delay2time = 2;
            reverblevel = 40;
            line = "9. Room";
            break;
          case 9:
            delaybuffersize = 8192;
            delaytime = 3;
            delay2time = 2;
            reverblevel = 40;
            line = "8. Medium Room";
            break;
          case 10:
            reverbtime = 8192;  // Bal oldal: hosszú út
            reverbtime2 = 2048; // Jobb oldal: rövid út (azonnali válasz)
            delaytime = 1;
            delay2time = 2;     // A jobb oldal legyen kicsit tompább (több átlagolás)
            reverblevel = 45;
            line = "9. Medium Large Room";
            break;
          case 11:
            delaybuffersize = 8192;
            delaytime = 3;
            delay2time = 4;
            reverblevel = 40;
            line = "10. Large Room";
            break;
          case 12:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 4;
            reverblevel = 40;
            line = "11. Single Delay 102ms";
            break;
          case 13:
            delaybuffersize = 8192;
            delaytime = 2;
            delay2time = 4;
            reverblevel = 40;
            line = "12. Cross Delay 180ms";
            break;
          case 14:
            delaybuffersize = 8192;
            delaytime = 4;
            delay2time = 4;
            reverblevel = 40;
            line = "13. Cross Delay 148-256msec";
            break;
          case 15:
            delaybuffersize = 8192;
            delaytime = 5;
            delay2time = 6;
            reverblevel = 40;
            line = "14. Short Gate";
            break;
          case 16:
            delaybuffersize = 8192;
            delaytime = 6;
            delay2time = 7;
            reverblevel = 40;
            line = "15. Long Gate";
            break;
          case 17:
            delaybuffersize = 8192;
            delaytime = 7;
            delay2time = 8;
            reverblevel = 40;
            line = "Cross Delay 148-256msec";
            break;
          case 18:
            delaybuffersize = 8192;
            delaytime = 8;
            delay2time = 9;
            reverblevel = 40;
            line = "Cross Delay 148-256msec";
            break;
          case 19:
            delaybuffersize = 8192;
            delaytime = 9;
            delay2time = 10;
            reverblevel = 40;
            line = "Cross Delay 148-256msec";
            break;
          case 20:
            delaybuffersize = 8192;
            delaytime = 10;
            delay2time = 10;
            reverblevel = 40;
            line = "Cross Delay 148-256msec";
            break;
          case 21:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 40;
            line = "Cross Delay 148-256msec";
            break;
          default:
            return;
            break;
        }
        break;
      case 31:
        reverblevel = value;
        line = "ReverbLevel: " + lcdprint3(reverblevel);
        break;
      case 32:
        masterVolume = 7 - (value >> 4);
        line = "MasterVol: " + lcdprint3(masterVolume);
        break;
      case 34:
        MIDI_SYNC = value;
        line = "MIDI_SYNC=" + lcdprint3(MIDI_SYNC);
        break;
      case 35:
        CHASE_LEVEL = value;
        line = "CHASE LEVEL=" + lcdprint3(CHASE_LEVEL);
        break;
      case 36:
        CHASE_TIME = value;
        line = "CHASE TIME=" + lcdprint3(CHASE_TIME);
        break;
      case 37:
        if (value > 0) {
          midichan = value;
        }
        line = "MIDI CH=" + String(midichan);
        break;
      default:
        line = "No implemented" + String(localParameterByte) + " " + String(noteByte);
        return;
        break;
    }
  }
  //serial
  //Serial.println(line);
  //lcd
  if (LCD_ON) {
    lcdprint(line, 1);
  }

}

//-------------------------------REVERB-DELAY EFFECT LEFT----------------------------------------

int32_t atlag = 0;
int16_t x = 0;
int32_t atlag2 = 0;
int16_t x2 = 0;

void processingStereoReverb() {
  // 1. Kiolvasás a bufferekből
  int16_t delayedL = delaybuffer[delaybufferindex];
  int16_t delayedR = delaybuffer2[delaybufferindex2];

  // 2. Bemenet + Kereszt-visszacsatolás (itt történik a sztereó varázslat)
  int16_t inputL = bufferbe[0];
  int16_t inputR = bufferbe[1];

  // A delayedR >> 2 és delayedL >> 2 azt jelenti, hogy 25% átszivárog a másik oldalra
  bufferbe[0] = inputL + delayedL + (delayedR >> 2);
  bufferbe[1] = inputR + delayedR + (delayedL >> 2);

  // --- BAL OLDAL SZÁMÍTÁSA ---
  atlag += (bufferbe[0] * reverblevel) >> 6;
  delaystep++;

  if (delaystep >= delaytime) {
    int16_t resL = atlag / delaystep;

    // 1. LIMITER (Hogy ne gerjedjen be a kereszt-feedback miatt sem)
    if (resL > 16384) resL = 16384 + (resL - 16384) / 2;
    else if (resL < -16384) resL = -16384 + (resL + 16384) / 2;

    // 2. REVERB DIFFUSION (A régi fix (3*új+régi)/4 helyett)
    x = ((resL * (8 - reverbdiffusion)) + (x * reverbdiffusion)) >> 3;

    delaybuffer[delaybufferindex] = x;
    atlag = 0;
    delaybufferindex++;
    delaybufferindex &= (reverbtime - 1);
    delaystep = 0;
  }

  // --- JOBB OLDAL SZÁMÍTÁSA ---
  atlag2 += (bufferbe[1] * reverblevel) >> 6;
  delay2step++;

  if (delay2step >= delay2time) {
    int16_t resR = atlag2 / delay2step;

    // 1. LIMITER JOBB
    if (resR > 16384) resR = 16384 + (resR - 16384) / 2;
    else if (resR < -16384) resR = -16384 + (resR + 16384) / 2;

    // 2. REVERB DIFFUSION JOBB
    x2 = ((resR * (8 - reverbdiffusion)) + (x2 * reverbdiffusion)) >> 3;

    delaybuffer2[delaybufferindex2] = x2;
    atlag2 = 0;
    delaybufferindex2++;
    delaybufferindex2 &= (reverbtime2 - 1);
    delay2step = 0;
  }
}

void reverbleft() {
  int16_t delayedSample = delaybuffer[delaybufferindex];
  bufferbe[0] += delayedSample;
  atlag += (bufferbe[0] * reverblevel) >> 6;
  delaystep++;

  if (delaystep >= delaytime) {
    int16_t newSample = atlag / delaystep;

    // 1. LIMITER (Puha vágás, hogy ne gerjedjen be)
    if (newSample > 16384) newSample = 16384 + (newSample - 16384) / 2;
    else if (newSample < -16384) newSample = -16384 + (newSample + 16384) / 2;

    // 2. DIFFUSION (Ez simítja el a limiter esetleges éleit is)
    x = ((newSample * (8 - reverbdiffusion)) + (x * reverbdiffusion)) >> 3;

    delaybuffer[delaybufferindex] = x;
    atlag = 0;
    delaybufferindex++;
    delaystep = 0;
  }
  delaybufferindex &= (reverbtime - 1);
}

void reverbright() {
  int16_t delayedSample2 = delaybuffer2[delaybufferindex2];
  bufferbe[1] = bufferbe[1] + delayedSample2;
  atlag2 += (bufferbe[1] * reverblevel) >> 6;
  delay2step++;

  if (delay2step >= delay2time) {
    int16_t newSample2 = atlag2 / delay2step;

    // 1. LIMITER jobb oldal
    if (newSample2 > 16384) newSample2 = 16384 + (newSample2 - 16384) / 2;
    else if (newSample2 < -16384) newSample2 = -16384 + (newSample2 + 16384) / 2;

    // 2. DIFFUSION jobb oldal
    x2 = ((newSample2 * (8 - reverbdiffusion)) + (x2 * reverbdiffusion)) >> 3;

    delaybuffer2[delaybufferindex2] = x2;
    atlag2 = 0;
    delaybufferindex2++;
    delay2step = 0;
  }
  delaybufferindex2 &= (reverbtime2 - 1);
}


//--------------------------CHORUS LEFT (OPTIMALIZÁLT)------------------------------
void chorusleft() {
  //  if (maskLeft == 0) return;

  uint32_t indexLarge = lfoarrayindex[6];
  uint16_t i1 = (indexLarge >> 23) & 511;
  uint16_t i2 = (i1 + 1) & 511;
  uint16_t lfoFraction = (indexLarge >> 15) & 0xFF;

  int32_t v1 = (int32_t)(*(LFOadress[6] + i1));
  int32_t v2 = (int32_t)(*(LFOadress[6] + i2));
  uint32_t rawLFO = v1 + (((v2 - v1) * lfoFraction) >> 8);

  lfoarrayindex[6] += (lfofreq[6] << 14);

  // --- A "RÉGI JÓ" RECEPT SKÁLÁZÁSA ---
  // A maszk 75%-át engedjük csak bejárni (mint a régi 384/512 arányod)
  uint32_t depthLimit = (maskLeft * 3) >> 2;
  uint32_t depthLFO = (rawLFO * depthLimit) >> 16;
  uint8_t fraction = (rawLFO & 0xFF);

  chorusbufferleft[chorusbufferindex] = bufferbe[0];

  uint16_t idx1 = (chorusbufferindex - depthLFO) & maskLeft;
  uint16_t idx2 = (idx1 + 1) & maskLeft;

  int16_t s1 = chorusbufferleft[idx1];
  int16_t s2 = chorusbufferleft[idx2];

  int16_t interpolated = s1 + (((s2 - s1) * fraction) >> 8);

  // Kimenet: Közvetlen az interpolált jelet használjuk a harapáshoz
  int32_t chorusPart = (interpolated * chorusLevelLeft) >> 8;
  int32_t out = (int32_t)bufferbe[0] + chorusPart;

  atlagchorus0 = (interpolated + atlagchorus0) >> 1;

  if (out > 32767) out = 32767;
  else if (out < -32768) out = -32768;

  bufferbe[0] = (int16_t)out;
  chorusbufferindex = (chorusbufferindex + 1) & maskLeft;
}

//--------------------------CHORUS RIGHT (OPTIMALIZÁLT)------------------------------
void chorusright() {
  // if (maskRight == 0) return;

  uint32_t indexLarge = lfoarrayindex[7];
  uint16_t i1 = (indexLarge >> 23) & 511;
  uint16_t i2 = (i1 + 1) & 511;
  uint16_t lfoFraction = (indexLarge >> 15) & 0xFF;

  int32_t v1 = (int32_t)(*(LFOadress[7] + i1));
  int32_t v2 = (int32_t)(*(LFOadress[7] + i2));
  uint32_t rawLFO = v1 + (((v2 - v1) * lfoFraction) >> 8);

  lfoarrayindex[7] += (lfofreq[7] << 14);

  // --- A "RÉGI JÓ" RECEPT SKÁLÁZÁSA ---
  uint32_t depthLimit = (maskRight * 3) >> 2;
  uint32_t depthLFO = (rawLFO * depthLimit) >> 16;
  uint8_t fraction = (rawLFO & 0xFF);

  chorusbufferright[chorusbufferindex2] = bufferbe[1];

  uint16_t idx1 = (chorusbufferindex2 - depthLFO) & maskRight;
  uint16_t idx2 = (idx1 + 1) & maskRight;

  int16_t s1 = chorusbufferright[idx1];
  int16_t s2 = chorusbufferright[idx2];

  int16_t interpolated = s1 + (((s2 - s1) * fraction) >> 8);

  int32_t chorusPart = (interpolated * chorusLevelRight) >> 8;
  int32_t out = (int32_t)bufferbe[1] + chorusPart;

  atlagchorus1 = (interpolated + atlagchorus1) >> 1;

  if (out > 32767) out = 32767;
  else if (out < -32768) out = -32768;

  bufferbe[1] = (int16_t)out;
  chorusbufferindex2 = (chorusbufferindex2 + 1) & maskRight;
}


//---------------------------Flanger-------------------------------------
/*
  void flangerright() {
  // --- 1. LFO ÉS SEBESSÉG (lfofreq[1]) ---
  uint32_t currentPhase = lfoarrayindex[1];
  // Az LFO sebességét az lfofreq[1] határozza meg
  lfoarrayindex[1] += (lfofreq[1] << 13);

  lfovalue[1] = *(LFOadress[1] + (currentPhase >> 23));
  uint8_t fraction = (currentPhase >> 15) & 0xFF; // Törtrész az interpolációhoz

  // --- 2. INDEXEK KISZÁMÍTÁSA ---
  // A Flangerhez rövid eltolás kell (lfovalue-t oszthatod, ha túl mély)
  int32_t baseIdx = (int32_t)chorusbufferindex2 - (lfovalue[1] >> 1);

  // Biztonságos körbefordulás (modulo vagy if helyett)
  while (baseIdx < 0) baseIdx += chorusbuffersize2;
  uint16_t idx1 = (uint16_t)baseIdx % chorusbuffersize2;
  uint16_t idx2 = (idx1 + 1) % chorusbuffersize2;

  // --- 3. INTERPOLÁCIÓ ---
  int16_t s1 = chorusbufferright[idx1];
  int16_t s2 = chorusbufferright[idx2];
  int16_t delayedSample = s1 + (((s2 - s1) * fraction) >> 8);

  // --- 4. FEEDBACK (flangeFeedback) ---
  // flangeFeedback értéke: 0 - 200 (255 felett begerjedhet!)
  int32_t feedbackSignal = (delayedSample * flangeFeedback) >> 8;
  int32_t inputToBuffer = bufferbe[1] + feedbackSignal;

  // Limiter (védelem a recsegés ellen)
  if (inputToBuffer > 32767) inputToBuffer = 32767;
  else if (inputToBuffer < -32768) inputToBuffer = -32768;

  // Beírás a pufferbe
  chorusbufferright[chorusbufferindex2] = (int16_t)inputToBuffer;

  // Puffer index léptetése (Vigyázz a -1-re, ha nem 2 hatványa a méret!)
  chorusbufferindex2++;
  if (chorusbufferindex2 >= chorusbuffersize2) chorusbufferindex2 = 0;

  // --- 5. MIX SZINT (flangeLevel) ÉS SZŰRÉS ---
  // flangeLevel értéke: 0 (tiszta hang) - 255 (csak az effekt)
  // Megtartottam a te kedvenc atlag-szűrődet is:
  atlagchorus1 += (((delayedSample * flangeLevel) >> 8) + atlagchorus1) >> 1;

  // Végső kimenet: Eredeti jel + Szűrt effektelt jel
  bufferbe[1] = bufferbe[1] + atlagchorus1;
  atlagchorus1 = atlagchorus1 >> 1;
  }
*/

//-----------------------LOWPASSFILTER LEFT---------------------------
//lowpassfilter in delaybuffer!!!
//delaybuffer actual sample, x: delaybuffer prev sample


/*
  void lowpassfilterleft() {
  //delaybuffer[delaybufferindex] = (delaybuffer[delaybufferindex] + x) >> 1;
  //x = delaybuffer[delaybufferindex];
  }
*/
//LOWPASSFILTER RIGHT
//lowpassfilter in delaybuffer!!!
//delaybuffer actual sample, x2: delaybuffer prev sample
/*
  void lowpassfilterright() {
  //delaybuffer2[delaybufferindex2] = (delaybuffer2[delaybufferindex2] + x2) >> 1;
  //x2 = delaybuffer2[delaybufferindex2];
  }
*/

//-------------------------------MIDI INPUT COMMAND-------------------------------------
//keylogic

inline int findBestSlot() {
  int foundIdx = 0;
  // 1. Keresés az első szabadra (status 5)
  for (int i = 0; i < polyphony; i++) {
    if (generatorstatus[0][voiceStack[i]] == 5) {
      foundIdx = i;
      break;
    }
  }

  int bestS = voiceStack[foundIdx];

  // 2. Csak akkor mozgatunk, ha szükséges.
  // Ha kicsi a polyphony (pl. <= 8), egy sima for ciklus gyorsabb is lehet, mint a memmove
  if (foundIdx < (polyphony - 1)) {
    int count = polyphony - 1 - foundIdx;
    memmove(&voiceStack[foundIdx], &voiceStack[foundIdx + 1], count); // sizeof(byte) elhagyható, ha byte
  }
  voiceStack[polyphony - 1] = bestS;
  return bestS;
}
byte noteToSlot[128]; // Inicializáld 255-tel a setupban!
byte monoNote = 0;
void keyon(byte noteByte) {
  bool isAnyPoly = false;
  for (int i = 0; i < 4; i++) if (oscMode[i] == 0) isAnyPoly = true;
  int g;
  if (isAnyPoly) {
    g = findBestSlot();
  } else {
    g = 0;
  }
  int m = 0;
  for (int i = 0; i < 4; i++) {
    int targetS = (oscMode[i] == 0) ? g : m;
    int shift = (i < 2) ? LKeyShift : UKeyShift;
    wavefreq[i][targetS] = noteertek[i][noteByte + shift];
    wavebias[i][targetS] = Bias[i][noteByte + shift];
    pich[i][targetS] = wavefreq[i][targetS];
    // Portamento
    if (portamento_time[i] == 0) {
      currentPitch[i][targetS] = pich[i][targetS];
    } else {
      currentPitch[i][targetS] = lastTargetPitch[i];
    }
    lastTargetPitch[i] = pich[i][targetS];
  }
  // --- 3. INDÍTÁSI LOGIKA OSC-NKÉNT ---
  for (int i = 0; i < 4; i++) {
    if (oscMode[i] == 0) {
      // --- POLIFÓN ÁG (LRU slot 'g') ---
      freqmutato[i][g] = samplebegin[i] << step;
      v_lp[i][g] = 0.0f;
      v_bp[i][g] = 0.0f;
      TVAvolume[i][g] = ENV_L0;
      generatorstatus[i][g] = 0; // ATTACK
      // --- EZ A KIEGÉSZÍTÉS A TVF-NEK (i+4) ---
      generatorstatus[i + 4][g] = 0;
      TVAvolume[i + 4][g] = 0; // Vagy ENV_L0, ha a TVF is innen indul
    }
    else {
      //monofon
      if (generatorstatus[i][m] == 5) {
        freqmutato[i][m] = (uint32_t)samplebegin[i] << step;
        smoothedVol[i][m] = 0; // A simítót nulláról indítjuk
        TVAvolume[i][m] = 0;   // A burkolót is nulláról indítjuk
        generatorstatus[i][m] = 0; // Indul az Attack
        generatorstatus[i + 4][m] = 0;
        TVAvolume[i + 4][m] = 0;
      }
      // 2. Ha már szól valami (Legato):
      else {
        generatorstatus[i][m] = 0;
        generatorstatus[i + 4][m] = 0;
      }
    }
  }
  // --- 4. ADMINISZTRÁCIÓ ---
  oldnoteByte[g] = noteByte;
  noteoff[g] = false;
  noteToSlot[noteByte] = (byte)g; // <--- EZT ADD HOZZÁ! Ekkor fogja tudni a keyoff, hova nyúljon.
  // LFO Sync
  for (int i = 0; i < 6; i++) {
    if (LFOSYNC[i] == 2) LFO_Delay_Counter[i] = 0;
  }
  monoNote = noteByte; // Megjegyezzük, mi indította a monofont
}




void keyoff(byte noteByte) {
  int g = noteToSlot[noteByte];

  if (g < polyphony && oldnoteByte[g] == noteByte) {
    oldnoteByte[g] = 255;      // Jelezzük, hogy ez a slot már nem tartozik semmilyen billentyűhöz
    noteToSlot[noteByte] = 255; // Kitakarítjuk a táblát is

    for (int i = 0; i < 4; i++) {
      if (oscMode[i] == 0) {
        generatorstatus[i][g] = 4;
        generatorstatus[i + 4][g] = 4;
      }
    }
  }

  // 3. MONOFÓN ELLENŐRZÉS (Ha ugyanaz a hang, ami a monofont tartja)
  if (noteByte == monoNote) {
    for (int i = 0; i < 4; i++) {
      if (oscMode[i] > 0) {
        generatorstatus[i][0] = 4;
        generatorstatus[i + 4][0] = 4;
      }
    }
  }
}
//--------------CHASE---------------------------



void chasearpeggiomidiclock() {
  // 1. Csak akkor fut, ha van külső START (MIDI_SYNC) és van tempó osztás
  if (MIDI_SYNC == 1 && CHASE_TIME > 0) {

    int shiftedTick = (masterTick + OFFSET);

    // 2. Időzítés a MIDI Tickek alapján
    if (shiftedTick % CHASE_TIME == 0) {
      static int lastProcessedTick = -1;
      if (shiftedTick == lastProcessedTick) return;
      lastProcessedTick = shiftedTick;

      int talaltHang = 255;

      // 3. KERESÉS: Végigpörgetjük a slotokat a következő lefogott hangért
      // Pontosan ugyanúgy, ahogy a sima Arpeggiónál csináltuk!
      for (int i = 0; i < polyphony; i++) {
        chaseindex++;
        if (chaseindex >= polyphony) chaseindex = 0;

        if (oldnoteByte[chaseindex] != 255) {
          talaltHang = oldnoteByte[chaseindex];
          break;
        }
      }

      // 4. MEGSZÓLALTATÁS
      if (talaltHang != 255) {
        // Frissítjük a lastchase-t, hogy tudjuk, mit kell majd leállítani
        lastchase = talaltHang;

        // Újraindítjuk a hangot (keyoff-keyon az LRU miatt)
        keyoff(lastchase);
        keyon(lastchase);
      } else {
        lastchase = 255;
      }
    }
  }
}


void chasearpeggio() {
  if (CHASE_TIME > 0) {
    ido = micros();
    if (ido - elozoido > (uint32_t)CHASE_TIME << 13) {

      int talaltHang = 255; // 255 = "Nincs mit lejátszani"

      // 1. Keresünk egy élő hangot a slotokban
      for (int i = 0; i < polyphony; i++) {
        chaseindex++;
        if (chaseindex >= polyphony) chaseindex = 0;

        // Csak olyan slotot fogadunk el, ami NINCS felengedve (255)
        // ÉS nem 0 (ha a 0-ás hangjegy nálad a "nincs hang")
        if (oldnoteByte[chaseindex] != 255 && oldnoteByte[chaseindex] > 0) {
          talaltHang = oldnoteByte[chaseindex];
          break;
        }
      }

      // 2. KRITIKUS PONT: Csak akkor indítunk hangot, ha találtunk VALÓDIT
      if (talaltHang != 255) {
        keyoff(talaltHang);
        keyon(talaltHang);
      }
      // Ha talaltHang == 255, nem hívunk keyon-t -> nincs kopogás!

      elozoido = ido;
    }
  }
}

void patchbeginscreen() {
  LCD_ON = true;
  line = lcdprint2(prognumber);
  line += ":";
  line += pachname;
  lcdprint(line, 0);
  line = "                ";
  switch (STRUCTURE) {
    case 0:
      line = "LA+LA LA+LA";
      break;
    case 1:
      line = "LA+LA LA*LA";
      break;
    case 2:
      line = "LA+LA PCM+LA";
      break;
    case 10:
      line = "LA+LA LA*LA";
      break;
    case 11:
      line = "LA*LA LA*LA";
      break;
    case 20:
      line = "PCM+LA LA+LA";
      break;
    case 22:
      line = "PCM+LA PCM+LA";
      break;
    case 23:
      line = "PCM+LA PCM*LA";
      break;
    case 32:
      line = "PCM*LA PCM+LA";
      break;
    case 33:
      line = "PCM*LA PCM*LA";
      break;
    case 55:
      line = "PCM+PCM PCM+PCM";
      break;
    case 56:
      line = "PCM+PCM PCM*PCM";
      break;
    case 57:
      line = "PCM+PCM FM-2OP";
      break;
    case 65:
      line = "PCM+PCM PCM*PCM";
      break;
    case 66:
      line = "PCM*PCM PCM*PCM";
      break;
    case 67:
      line = "FM-2OP FM-2OP";
      break;
    case 76:
      line = "FM Y structura";
      break;
    case 77:
      line = "FM-2OP FM-2OP";
      break;
    default:
      line = "  UNKNOWN  ";
      break;
  }
  //lcdprint(line, 1);
}


void handleNoteOn(byte channel, byte note, byte velocity) {
  if (channel == midichan) { // <--- A szűrő kapuja
    if (velocity > 0 && note >= 12) {
      keyon(note);
    } else {
      keyoff(note);
    }
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  if (channel == midichan) { // <--- Ide is kell!
    keyoff(note);
  }
}
void handleClock() {
  masterTick++;
  if (masterTick >= 24) masterTick = 0; // Egy negyed lefutott, kezdjük újra
  chasearpeggiomidiclock();
}

void handleStart() {
  for (int i = 0; i < CHASE_LEVEL; i++) CaseArray[i] = 0;
  sixteen = 0;
  chaseindex = 0;
  lastchase = 255;
  MIDI_SYNC = 1;
  masterTick = 0;
}

void handleContinue() {
  MIDI_SYNC = 1;
  sixteen = 0;
}

void handleStop() {
  MIDI_SYNC = 0;
  if (lastchase != 255) keyoff(lastchase);
  lastchase = 255;
}

void handleControlChange(byte channel, byte number, byte value) {
  if (number == 1) { // CC#1 a Mod Wheel
    modulationWheel = value;
  }
}
void handlePitchBend(byte channel, int bend) {
  // A bend 0-16383 között jön, eltoljuk, hogy -8192 és +8191 között legyen
  pitchBendValue = bend - 8192;
}

void handleSysEx(byte* data, unsigned size) {
  // CSAK AKKOR printelj, ha nagyon muszáj, mert a Serial.print lassú!
  // A legjobb, ha csak a feldolgozás marad:
  // prefix ellenőrzés: 240, 65, 0, 20, 18, 0
  //Sysex:240,65,0,20,18,0,---local-3,  notebayt-0, velocitybyte-0,    0,0,0,0,0,0,27,38,32,27,48,35,38,38,31,0,0,0,0,0,0,0,24,50,50,2,16,0,0,5,39,18,50,0,0,40,0,0,0,29,247
  if (size >= 6 && data[0] == 240 && data[1] == 65 && data[2] == 0 && data[3] == 20 && data[4] == 18 && data[5] == 0) {
    if (size <= 11) {
      localParameterByte = data[6];
      noteByte = data[7];
      velocityByte = data[8];
      parametersysexchanged();
    }
    else {
      LCD_ON = false;
      localParameterByte = data[6];
      noteByte = data[7];
      velocityByte = data[8];
      parametersysexchanged();
      for (unsigned i = 9; i < size - 2; i++) {
        noteByte++;
        velocityByte = data[i];
        parametersysexchanged();
      }
      //Mivel csoportos sysex volt így sokminden történt. A legbiztonságosabb ha a Pach kezdőképernyőt írjuk ki!
      patchbeginscreen();
    }
  }
}
//----------------------------------PACH------------------------

struct Section {
  byte localByte;
  byte startNote;
  byte length;
  const char* name;
};

// A 7 szekció definíciója pontosan a te sorrendedben
Section sections[] = {
  {1, 64, 54, "U1"}, // Local 1, Note 64-től, 54 byte
  {2, 0,  54, "U2"}, // Local 2, Note 0-tól,  54 byte
  {0, 0,  54, "L1"}, // Local 0, Note 0-tól,  54 byte
  {0, 64, 54, "L2"}, // Local 0, Note 64-től, 54 byte
  {3, 0,  40, "P"},  // Patch, Local 3, 40 byte
  {1, 0,  48, "CU"}, // Common Upper, Local 1, 48 byte
  {2, 65, 56, "CL"}  // Common Lower, Local 2, 48 byte
};

void LoadPatch(const byte* storedPatch) {
  LCD_ON = false;
  int globalIdx = 0;
  for (int s = 0; s < 7; s++) {
    //Serial.print("--- Section: "); Serial.println(sections[s].name);
    localParameterByte = sections[s].localByte;
    for (int i = 0; i < sections[s].length; i++) {
      vTaskDelay(pdMS_TO_TICKS(1));
      noteByte = sections[s].startNote + i;
      velocityByte = storedPatch[globalIdx];
      parametersysexchanged();
      globalIdx++;
    }
  }
  LCD_ON = true;
}

void handleProgramChange(byte channel, byte number) {
  //Serial.print("Program Change érkezett: "); Serial.println(number);
  prognumber = number;
  switch (prognumber) {
    case 0:
      LoadPatch(storedpach1);
      break;
    case 1:
      LoadPatch(storedpach2);
      break;
    case 2:
      LoadPatch(storedpach3);
      break;
    case 3:
      LoadPatch(storedpach4);
      break;
    case 4:
      LoadPatch(storedpach5);
      break;
    case 5:
      LoadPatch(storedpach6);
      break;
    case 6:
      LoadPatch(storedpach7);
      break;
    case 7:
      LoadPatch(storedpach8);
      break;
    case 8:
      LoadPatch(storedpach9);
      break;
    case 9:
      LoadPatch(storedpach10);
      break;
    case 10:
      LoadPatch(storedpach11);
      break;
    case 11:
      LoadPatch(storedpach12);
      break;
    case 12:
      LoadPatch(storedpach13);
      break;
    case 13:
      LoadPatch(storedpach14);
      break;
    case 14:
      LoadPatch(storedpach15);
      break;
    case 15:
      LoadPatch(storedpach16);
      break;
    case 16:
      LoadPatch(storedpach17);
      break;
    case 17:
      LoadPatch(storedpach18);
      break;
    case 18:
      LoadPatch(storedpach19);
      break;
    case 19:
      LoadPatch(storedpach20);
      break;
    case 20:
      LoadPatch(storedpach21);
      break;
    default:
      Serial.println("Nincs ilyen tárolt patch!");
      break;
  }
  patchbeginscreen();
}

//----------------------------------------setup--------------------------------
void setup() {
  // Ha a DSP a loop-ban van, adj neki prioritást
  // Ez megakadályozza, hogy a háttérfolyamatok "ellopják" az időt
  vTaskPrioritySet(NULL, 1);
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
  Wire.setClock(400000);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(" Firmvare: 0.2 ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("  D50-Sampler   ");
  delay(600);
  lcd.setCursor(0, 1);
  lcd.print("  DigitalSynth  ");
  delay(600);
  delay(10);
  MIDI2.setHandleSystemExclusive(handleSysEx);
  MIDI2.setHandleNoteOn(handleNoteOn);
  MIDI2.setHandleNoteOff(handleNoteOff);
  MIDI2.setHandleClock(handleClock);
  MIDI2.setHandleStart(handleStart);
  MIDI2.setHandleContinue(handleContinue);
  MIDI2.setHandleStop(handleStop);
  MIDI2.setHandleControlChange(handleControlChange);
  MIDI2.setHandlePitchBend(handlePitchBend);
  MIDI2.setHandleProgramChange(handleProgramChange);
  MIDI2.begin(MIDI_CHANNEL_OMNI);
  delaybuffer = (int16_t*) heap_caps_malloc(delaybuffersize * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  delaybuffer2 = (int16_t*) heap_caps_malloc(delaybuffersize * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (delaybuffer != NULL) memset(delaybuffer, 0, delaybuffersize * sizeof(int16_t));
  if (delaybuffer2 != NULL) memset(delaybuffer2, 0, delaybuffersize * sizeof(int16_t));
  //Set up NOTE TUNE
  notetune();
  notebias();
  maxsize();
  // Minden LFO kapjon egy létező címet, hogy ne legyen 0 (NULL)
  updateLFOAdresses();
  for (int i = 0; i < 4; i++) {
    opmenuoldal = i;
    setPCMWave();
    setsamplesize();
  }
  opmenuoldal = 0;
  eqkiszamol();
  //Serial.println("Start");

  LoadPatch(storedpach1);

  Serial.println("--- Szinti Init ---");
  //modmatrix az lfokhoz
  for (int o = 0; o < 4; o++) for (int t = 0; t < 3; t++) modulationMatrix[o][t] = 1;

  for (int s = 0; s < polyphony; s++) {
    voiceStack[s] = s; // Feltöltjük: 0, 1, 2, 3...
    for (int i = 0; i < 4; i++) {
      generatorstatus[i][s] = 5; // Minden oszcillátor alapból OFF
      TVAvolume[i][s] = 0;
      v_lp[i][s] = 0;
      v_bp[i][s] = 0;
    }
    oldnoteByte[s] = 0;
  }
  Serial.print("Polyphony: "); Serial.println(polyphony);

}

void loop() {
  MIDI2.read();
  // Serial.print("-");
  if (MIDI_SYNC == 2)
  {
    chasearpeggio();
  }
  //--MIDI input--
  //serialEvent();
  //LFOVALUES
  static uint8_t lfoPrescaler = 0;
  lfoPrescaler++;
  auto pLfoIdx  = &lfoarrayindex[0];
  auto pLfoFreq = &lfofreq[0];
  auto pLfoVal  = &lfovalue[0];
  auto pCounter = &LFO_Delay_Counter[0];
  for (int i = 0; i < 6; i++) {
    if (*pCounter < LFO_Delay[i]) {
      // Csak minden 8. körben (buffernél) növeljük a számlálót
      if ((lfoPrescaler & 0x0F) == 0) {
        (*pCounter)++;
      }
      *pLfoVal = 0;
    }
    else {
      *pLfoVal = *(LFOadress[i] + (*pLfoIdx >> 23));
      *pLfoIdx += ((uint32_t) * pLfoFreq << 19);
    }
    pLfoIdx++; pLfoFreq++; pLfoVal++; pCounter++;
  }

  // TVA ENVELOPE OPTIMIZED
  for (int i = 0; i < 8; i++) {
    // Előszámítások (csak 8x futnak le)
    const uint32_t L1 = (uint32_t)ENV_L1[i] << 17;
    const uint32_t L2 = (uint32_t)ENV_L2[i] << 17;
    const uint32_t L3 = (uint32_t)ENV_L3[i] << 17;
    const uint32_t LS = (uint32_t)ENV_LSUS[i] << 17;
    const uint32_t LE = (uint32_t)(ENV_LEND[i] * 100) << 17;

    const uint32_t sT1 = ((uint32_t)speedTable[ENV_T1[i]] << 12) + 100U;
    const uint32_t sT2 = ((uint32_t)speedTable[ENV_T2[i]] << 12) + 100U;
    const uint32_t sT3 = ((uint32_t)speedTable[ENV_T3[i]] << 12) + 100U;
    const uint32_t sT4 = ((uint32_t)speedTable[ENV_T4[i]] << 12) + 100U;
    const uint32_t sT5 = ((uint32_t)speedTable[ENV_T5[i]] << 12) + 100U;

    uint8_t* pStat = &generatorstatus[i][0];
    uint32_t* pVol  = &TVAvolume[i][0];

    // Előkészítjük a TVA specifikus adatokat, hogy a ciklusban ne kelljen IF
    const bool isTVA = (i < 4);
    const uint16_t base = isTVA ? volume[i] : 0;
    const uint8_t tvaMode = isTVA ? TVA[i] : 0;
    uint16_t* pGVol = isTVA ? &generatorvolume[i][0] : nullptr;

    for (int j = 0; j < polyphony; j++) {
      uint32_t v = *pVol;
      uint8_t s = *pStat;

      // --- Tömörített Switch (kevesebb elágazás) ---
      switch (s) {
        case 0: v += sT1; if (v >= L1) {
            v = L1;
            s = 1;
          } break;
        case 1: if (v > L2) {
            v = (v > L2 + sT2) ? v - sT2 : L2;
            if (v == L2) s = 2;
          }
          else {
            v = (v + sT2 < L2) ? v + sT2 : L2;
            if (v == L2) s = 2;
          } break;
        case 2: if (v > L3) {
            v = (v > L3 + sT3) ? v - sT3 : L3;
            if (v == L3) s = 3;
          }
          else {
            v = (v + sT3 < L3) ? v + sT3 : L3;
            if (v == L3) s = 3;
          } break;
        case 3: if (v > LS) v = (v > LS + sT4) ? v - sT4 : LS;
          else if (v < LS) v = (v + sT4 < LS) ? v + sT4 : LS; break;
        case 4: if (v > LE + sT5) v -= sT5;
          else if (v < LE) {
            v += sT5;
            if (v > LE) v = LE;
          } else v = LE;
          if (v == LE) s = (LE == 0) ? 5 : 4; break;
        case 5: v = 0; break;
      }
      *pVol = v; *pStat = s;

      // --- KIMENETI LOGIKA (IF NÉLKÜL, BIT-MASZKKAL) ---
      // A ciklus előtt számold ki a "módosított" alaphangerőt
      uint32_t biasedBase = ((uint32_t)base * wavebias[i][j] * 21) >> 8;

      if (isTVA) {
        if (tvaMode > 0) {
          uint8_t idx = (v >> 16);
          if (tvaMode != 1) idx = 255 - idx;

          // A belső loopban már csak a biasedBase-t használod
          *pGVol = (uint16_t)((uint32_t)logTable16_S[idx] * biasedBase >> 14);
          pGVol++;
        } else {
          // Ha nincs TVA, a biasedBase adja a fix hangerőt
          *pGVol = (uint16_t)(biasedBase >> 2);
          pGVol++;
        }
      }

      else {
        // TVF: Csak akkor konvertálunk float-ra, ha muszáj
        tvf_env_mod[i - 4][j] = (float)(v >> 16) * 0.00390625f;
      }
      pVol++; pStat++;
    }
  }

  // BEND SZÁMÍTÁSA KÍVÜL ???---
  // LFO working area
  for (int osc = 0; osc < 4; osc++) {
    int lfoBaseIndex = (osc < 2) ? 0 : 3;
    int selectedLFO = lfoBaseIndex + (PWMLFO[osc] >> 1);
    // --- 1. PWM MODULÁCIÓ ---
    // [osc][0] -> PWM Mátrix
    int32_t currentPWMLFODepth = PWMLFODepth[osc];
    if (modulationMatrix[osc][0]) {
      currentPWMLFODepth += (modulationWheel >> 1);
    }
    int32_t lfoMod = (lfovalue[selectedLFO] * currentPWMLFODepth) >> 5;
    if (PWMLFO[osc] & 1) lfoMod = -lfoMod;
    int32_t finalPW = ((PW[osc] + 1) << 5) + lfoMod;
    finalPW = (finalPW > 1023) ? 1023 : (finalPW < 1 ? 1 : finalPW);
    // --- 2. TVF (SZŰRŐ) MODULÁCIÓ ---
    // [osc][1] -> TVF (Szűrő) Mátrix
    float lfo_part = (tvf_cutoff[osc] * 0.01f);
    if (modulationMatrix[osc][1]) {
      int32_t totalTVFLfoLevel = TVF_LFO_level[osc] + (modulationWheel >> 2);
      lfo_part += ((lfovalue[TWFLFO[osc]] - 128.0f) * (totalTVFLfoLevel * 0.000039f));
    }
    filter_q[osc] = fmaxf(0.05f, 1.0f - (tvf_reso[osc] * 0.03f));
    // --- 3. PITCH MODULÁCIÓ ---
    // [osc][2] -> Pitch Mátrix
    int32_t pitchMod = 0;
    if (modulationMatrix[osc][2]) {
      int16_t bipolarLFO = (int16_t)lfovalue[PICHLFO[osc]] - 127;
      int32_t totalPichLfoDepth = (PICH_LFO_level[osc] + (modulationWheel >> 2)) >> 2;
      pitchMod = bipolarLFO * totalPichLfoDepth;
    }
    // A pitchMod-ot itt adod hozzá a frekvencia számításhoz
    // Portamento sebesség skálázása (Próbáld a << 14 vagy << 15 értéket, ha lassú/gyors)
    uint32_t portamentoSpeed = (uint32_t)portamento_time[osc] << 15;
    for (int j = 0; j < polyphony; j++) {
      // --- 4. PORTAMENTO (GLIDE) LÉPTETÉS - GYORS ÉS PONTOS ---
      if (currentPitch[osc][j] != pich[osc][j]) {
        if (portamento_time[osc] == 0) {
          currentPitch[osc][j] = pich[osc][j];
        } else {
          uint32_t distance;
          uint8_t shift = 1 + (portamento_time[osc] >> 3);
          if (currentPitch[osc][j] < pich[osc][j]) {
            // FELFELÉ
            distance = pich[osc][j] - currentPitch[osc][j];
            uint32_t step = distance >> shift;
            if (step < 2) step = 2;
            currentPitch[osc][j] += step;
            // Ha túlszaladtunk (felfelé), korrigálunk
            if (currentPitch[osc][j] > pich[osc][j]) currentPitch[osc][j] = pich[osc][j];
          } else {
            // LEFELÉ
            distance = currentPitch[osc][j] - pich[osc][j];
            uint32_t step = distance >> shift;
            if (step < 2) step = 2;
            currentPitch[osc][j] -= step;
            // JAVÍTÁS: Ha túlszaladtunk lefelé (kisebb lett), korrigálunk
            if (currentPitch[osc][j] < pich[osc][j]) currentPitch[osc][j] = pich[osc][j];
          }
        }
      }
      // --- 5. SZŰRŐ ÉS PWM FRISSÍTÉS ---
      // 1. Olvasd ki a bájt mélységet (0-100)
      float env_mod = tvf_env_mod[osc][j];
      uint8_t depthByte = tvf_env_depth[osc + 4];
      // 2. Gyors skálázás float-ra (mivel env_mod float, ezt a proci nagyon szereti)
      // A 0.01f konstans szorzás a leggyorsabb módja a 0-100 -> 0.0-1.0 konverziónak
      float current_env_depth = (float)depthByte * 0.01f;
      // 3. Összevonás (lfo_part már float, ez maradhat)
      float total_norm = lfo_part + (env_mod * current_env_depth);
      // 4. Biztonsági korlát
      total_norm = fmaxf(0.0f, fminf(1.0f, total_norm));
      float cutoffHz = 20.0f + (total_norm * total_norm * 12000.0f);
      //filter_f[osc][j] = fmaxf(0.005f, fminf(0.95f, 2.0f * sinf(cutoffHz * 0.00015f)));
      filter_f[osc][j] = fmaxf(0.005f, fminf(ceiling_val, 2.0f * sinf(cutoffHz * stretch_val)));

      PWcount[osc][j] = finalPW;
      // --- 6. VÉGLEGES PITCH (GLIDE + LFO) ---
      //pichcount[osc][j] = currentPitch[osc][j] + (bipolarLFO * totalPichLfoDepth);
      pichcount[osc][j] = currentPitch[osc][j] + pitchMod;
    }
  }

  if (true) {
    f0 = f0orig + (lfovalue[0] * lfolevel[0]);
    eqkiszamol();
  }
  if (true) {
    f02 = f02orig + (lfovalue[3] * lfolevel[3]);
    eqkiszamol2();
  }

  //STRUCTURES

  //------------------0-------------------0---------------------LA+LA LA+LA------------------------------------
  /*
    ============================================================================
            ALGORITHM 00: DUAL SYNTH MIX (LA0+LA1  |  LA2+LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      |  (LA 0)  |   |  (LA 1)  |        |  (LA 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Synth 0)    | (Synth 1)         | (Synth 2)    | (Synth 3)
           |              |                   |              |
           v              v                   v              v
        ( Sum: LA0 + LA1 )                 ( Sum: LA2 + LA3 )
           |              |                   |              |
           +--------------+---------+---------+--------------+
                                    |
                                    v
                              [ AUDIO OUT ]

  */
  if (STRUCTURE == 0) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer Linear variable
      uint8_t idx = Waveform[0];
      if (idx > 2) idx = 2;
      const float* currentWave = waveformLookup[idx];
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint32_t* pW0  = &PWcount[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint8_t idx1 = Waveform[1];
      if (idx1 > 2) idx1 = 2; // Biztonsági korlát az OSC 1-nek
      const float* currentWave1 = waveformLookup[idx1];
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      // --- OSC 2 Előkészítés ---
      uint8_t idx2 = Waveform[2];
      if (idx2 > 2) idx2 = 2; // Biztonsági korlát az OSC 2-nek
      const float* currentWave2 = waveformLookup[idx2];
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2     = &v_lp[2][0];
      float* pB2     = &v_bp[2][0];
      uint32_t* pW2  = &PWcount[2][0];
      uint16_t* pV2  = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      // --- OSC 3 Előkészítés ---
      uint8_t idx3 = Waveform[3];
      if (idx3 > 2) idx3 = 2; // Biztonsági korlát az OSC 3-nak
      const float* currentWave3 = waveformLookup[idx3];
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3     = &v_lp[3][0];
      float* pB3     = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0];
      uint16_t* pV3  = &generatorvolume[3][0];
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (Tábla alapú + Optimalizált) ---
        *pF0 += *pP0;
        uint32_t ph0 = (*pF0 >> step) & 1023;
        uint32_t ph_shifted = (ph0 + *pW0) & 1023;
        float in0_sub = currentWave[ph0] - currentWave[ph_shifted];
        float mod = (*pW0 / 1024.0f) * 128.0f;
        int32_t offset = (int32_t)((sinTable[ph0] / 32768.0f) * mod);
        uint32_t warpedPh = (ph0 + offset) & 1023;
        // Itt az 'in0_sin'-t a sinTable-ből olvassuk, hogy a PD hatás tiszta maradjon
        float in0_sin = sinTable[warpedPh];
        float in0 = (Waveform[0] == 2) ? in0_sin : in0_sub;
        float resFB0 = fmaxf(-20000.0f, fminf(20000.0f, *pB0));
        float hp0 = in0 - *pL0 - (filter_q[0] * resFB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        *pL0 = fmaxf(-32768.0f, fminf(32767.0f, *pL0));
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 6;
        pF0++; pP0++; pL0++; pB0++; pW0++; pV0++; pFF0++;

        // --- OSC 1 ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        uint32_t ph1_shifted = (ph1 + *pW1) & 1023;
        float in1_sub = currentWave1[ph1] - currentWave1[ph1_shifted];
        float mod1 = (*pW1 / 1024.0f) * 128.0f;
        int32_t offset1 = (int32_t)((sinTable[ph1] / 32768.0f) * mod1);
        uint32_t warpedPh1 = (ph1 + offset1) & 1023;
        float in1_sin = sinTable[warpedPh1];
        float in1 = (Waveform[1] == 2) ? in1_sin : in1_sub;
        float resFB1 = fmaxf(-20000.0f, fminf(20000.0f, *pB1));
        float hp1 = in1 - *pL1 - (filter_q[1] * resFB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        *pL1 = fmaxf(-32768.0f, fminf(32767.0f, *pL1));
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 ---
        *pF2 += *pP2; // JAVÍTVA: pP2 kell, hogy a saját hangolása működjön!
        uint32_t ph2 = (*pF2 >> step) & 1023;
        uint32_t ph2_shifted = (ph2 + *pW2) & 1023;
        float in2_sub = currentWave2[ph2] - currentWave2[ph2_shifted];
        float mod2 = (*pW2 / 1024.0f) * 128.0f;
        int32_t offset2 = (int32_t)((sinTable[ph2] / 32768.0f) * mod2);
        uint32_t warpedPh2 = (ph2 + offset2) & 1023;
        float in2_sin = sinTable[warpedPh2];
        float in2 = (Waveform[2] == 2) ? in2_sin : in2_sub;
        float resFB2 = fmaxf(-20000.0f, fminf(20000.0f, *pB2));
        float hp2 = in2 - *pL2 - (filter_q[2] * resFB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2; // JAVÍTVA: pL2, pFF2 és pB2 kell ide!
        *pL2 = fmaxf(-32768.0f, fminf(32767.0f, *pL2));
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 6;
        pF2++; pP2++; pL2++; pB2++; pW2++; pV2++; pFF2++;

        // --- OSC 3 (Tábla alapú + Optimalizált) ---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        uint32_t ph3_shifted = (ph3 + *pW3) & 1023;
        float in3_sub = currentWave3[ph3] - currentWave3[ph3_shifted];
        float mod3 = (*pW3 / 1024.0f) * 128.0f;
        int32_t offset3 = (int32_t)((sinTable[ph3] / 32768.0f) * mod3);
        uint32_t warpedPh3 = (ph3 + offset3) & 1023;
        float in3_sin = sinTable[warpedPh3];
        float in3 = (Waveform[3] == 2) ? in3_sin : in3_sub;
        float resFB3 = fmaxf(-20000.0f, fminf(20000.0f, *pB3));
        float hp3 = in3 - *pL3 - (filter_q[3] * resFB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3; // JAVÍTVA az indexelés!
        *pL3 = fmaxf(-32768.0f, fminf(32767.0f, *pL3));
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] + osc_out[3]);
      }
      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = (totalUpper + (totalLower >> 2)) >> masterVolume;
      bufferbe[1] = (totalLower + (totalUpper >> 2)) >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------1-------------------0---------------------LA*LA LA+LA------------------------------------
  /*
    ============================================================================
            ALGORITHM 10: LA RINGMOD / MIX (LA0*LA1  |  LA2+LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      |  (LA 0)  |   |  (LA 1)  |        |  (LA 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Synth 0)    | (Synth 1)         | (Synth 2)    | (Synth 3)
           |              |                   |              |
           +------+  +----+                   v              v
                  |  |                     ( Sum: LA2 + LA3 )
                  v  v                        |              |
                +------+                      |              |
                |  (X) |  <-- RingMod         |              |
                +--+---+     (LA0 * LA1)      |              |
                   |                          |              |
                   +--------------------------+--------------+
                                              |
                                              v
                                        [ AUDIO OUT ]
  */
  if (STRUCTURE == 10) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer Linear variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint32_t* pW0  = &PWcount[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer Linear variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2  = &v_lp[2][0];
      float* pB2  = &v_bp[2][0];
      uint32_t* pW2  = &PWcount[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (Linear Engine pointer) ---
        *pF0 += *pP0;
        uint32_t ph0 = (*pF0 >> step) & 1023;
        // Waveform[0]-at használjuk
        float in0 = (Waveform[0] == 1) ? (float)((int32_t)(ph0 << 6) - 32768) : (ph0 < *pW0 ? 32767.0f : -32768.0f);
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 6;
        // Mutatók léptetése a polifónia körben
        pF0++; pP0++; pL0++; pB0++; pW0++; pV0++; pFF0++;



        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 LINEAR Engine pointer---
        *pF2 += *pP2;
        uint32_t ph2 = (*pF2 >> step) & 1023;
        float in2 = (Waveform[2] == 1) ? (float)((int32_t)(ph2 << 6) - 32768) : (ph2 < *pW2 ? 32767.0f : -32768.0f);
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 6;
        pF2++; pP2++; pL2++; pB2++; pW2++; pV2++; pFF2++;

        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += (osc_out[0] * (osc_out[1] >> 12)) >> 3;
        //totalUpper += (osc_out[0] + osc_out[1]);
        //totalLower += (osc_out[2] * (osc_out[3] >> 12)) >> 3;
        totalLower += (osc_out[2] + osc_out[3]);

      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------0-------------------1---------------------LA+LA LA*LA------------------------------------
  /*
    ============================================================================
            ALGORITHM 01: LA MIX / RINGMOD (LA0+LA1  |  LA2*LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      |  (LA 0)  |   |  (LA 1)  |        |  (LA 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Synth 0)    | (Synth 1)         | (Synth 2)    | (Synth 3)
           |              |                   |              |
           v              v                   +------+  +---+
         ( Sum: LA0 + LA1 )                         |  |
           |              |                         v  v
           |              |                       +------+
           |              |                       |  (X) |  <-- RingMod (LA2 * LA3)
           |              |                       +--+---+
           |              |                          |
           +--------------+--------------------------+
                          |
                          v
                    [ AUDIO OUT ]

  */
  if (STRUCTURE == 1) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer Linear variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint32_t* pW0  = &PWcount[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer Linear variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2  = &v_lp[2][0];
      float* pB2  = &v_bp[2][0];
      uint32_t* pW2  = &PWcount[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (Linear Engine pointer) ---
        *pF0 += *pP0;
        uint32_t ph0 = (*pF0 >> step) & 1023;
        // Waveform[0]-at használjuk
        float in0 = (Waveform[0] == 1) ? (float)((int32_t)(ph0 << 6) - 32768) : (ph0 < *pW0 ? 32767.0f : -32768.0f);
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 6;
        // Mutatók léptetése a polifónia körben
        pF0++; pP0++; pL0++; pB0++; pW0++; pV0++; pFF0++;



        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 LINEAR Engine pointer---
        *pF2 += *pP2;
        uint32_t ph2 = (*pF2 >> step) & 1023;
        float in2 = (Waveform[2] == 1) ? (float)((int32_t)(ph2 << 6) - 32768) : (ph2 < *pW2 ? 32767.0f : -32768.0f);
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 6;
        pF2++; pP2++; pL2++; pB2++; pW2++; pV2++; pFF2++;

        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        //totalLower += (osc_out[0] * (osc_out[1] >> 12)) >> 3;
        totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] * (osc_out[3] >> 12)) >> 3;
        //totalLower += (osc_out[2] + osc_out[3]);

      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //------------------1-------------------1---------------------LA*LA LA*LA------------------------------------
  /*
    ============================================================================
            ALGORITHM 11: DUAL RINGMOD SYNTH (LA0*LA1  |  LA2*LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      |  (LA 0)  |   |  (LA 1)  |        |  (LA 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Synth 0)    | (Synth 1)         | (Synth 2)    | (Synth 3)
           |              |                   |              |
           +------+  +----+                   +------+  +----+
                  |  |                               |  |
                  v  v                               v  v
                +------+                           +------+
                |  (X) |  <-- RingMod              |  (X) |  <-- RingMod
                +--+---+     (LA0 * LA1)           +--+---+     (LA2 * LA3)
                   |                                  |
                   +-----------------+----------------+
                                     |
                                     v
                               [ AUDIO OUT ]
  */
  if (STRUCTURE == 11) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer Linear variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint32_t* pW0  = &PWcount[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer Linear variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2  = &v_lp[2][0];
      float* pB2  = &v_bp[2][0];
      uint32_t* pW2  = &PWcount[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (Linear Engine pointer) ---
        *pF0 += *pP0;
        uint32_t ph0 = (*pF0 >> step) & 1023;
        // Waveform[0]-at használjuk
        float in0 = (Waveform[0] == 1) ? (float)((int32_t)(ph0 << 6) - 32768) : (ph0 < *pW0 ? 32767.0f : -32768.0f);
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 6;
        // Mutatók léptetése a polifónia körben
        pF0++; pP0++; pL0++; pB0++; pW0++; pV0++; pFF0++;



        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 LINEAR Engine pointer---
        *pF2 += *pP2;
        uint32_t ph2 = (*pF2 >> step) & 1023;
        float in2 = (Waveform[2] == 1) ? (float)((int32_t)(ph2 << 6) - 32768) : (ph2 < *pW2 ? 32767.0f : -32768.0f);
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 6;
        pF2++; pP2++; pL2++; pB2++; pW2++; pV2++; pFF2++;

        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += (osc_out[0] * (osc_out[1] >> 12)) >> 3;
        //totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] * (osc_out[3] >> 12)) >> 3;
        //totalLower += (osc_out[2] + osc_out[3]);

      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //------------------0-------------------2---------------------LINEAR+LINEAR PCM+LINEAR------------------------------------
  /*
    ============================================================================
            ALGORITHM 02: HYBRID MIX (LA0+LA1  |  PCM2+LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      |  (LA 0)  |   |  (LA 1)  |        | (PCM 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Synth 0)    | (Synth 1)         | (Sample 2)   | (Synth 3)
           |              |                   |              |
           v              v                   v              v
        ( Sum: LA0 + LA1 )                 ( Sum: PCM2 + LA3 )
           |              |                   |              |
           +--------------+---------+---------+--------------+
                                    |
                                    v
                              [ AUDIO OUT ]

  */
  if (STRUCTURE == 2) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer Linear variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint32_t* pW0  = &PWcount[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer PCM variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2     = &v_lp[2][0];
      float* pB2     = &v_bp[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (Linear Engine pointer) ---
        *pF0 += *pP0;
        uint32_t ph0 = (*pF0 >> step) & 1023;
        // Waveform[0]-at használjuk
        float in0 = (Waveform[0] == 1) ? (float)((int32_t)(ph0 << 6) - 32768) : (ph0 < *pW0 ? 32767.0f : -32768.0f);
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 6;
        // Mutatók léptetése a polifónia körben
        pF0++; pP0++; pL0++; pB0++; pW0++; pV0++; pFF0++;


        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 PCM Engine pointer---
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        uint32_t frac2 = pos2 & ((1 << step) - 1);
        if (idx2 < sampleend[2] - 1) {
          *pF2 += *pP2;
        } else if (loopsample[2]) {
          *pF2 = (uint32_t)samplebegin[2] << step;
        }
        int16_t s1_2 = *(genstartadress[2] + idx2);
        int16_t s2_2 = *(genstartadress[2] + idx2 + 1);
        float in2 = (float)(s1_2 + (((int32_t)(s2_2 - s1_2) * (int32_t)frac2) >> step));
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;

        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] + osc_out[3]);
      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //------------------2-------------------0---------------------PCM+LINEAR LINEAR+LINEAR------------------------------------
  /*
    ============================================================================
            ALGORITHM 20: HYBRID MIX (PCM0+LA1  |  LA2+LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      | (PCM 0)  |   |  (LA 1)  |        |  (LA 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Sample 0)   | (Synth 1)         | (Synth 2)    | (Synth 3)
           |              |                   |              |
           v              v                   v              v
        ( Sum: PCM0 + LA1 )                ( Sum: LA2 + LA3 )
           |              |                   |              |
           +--------------+---------+---------+--------------+
                                    |
                                    v
                              [ AUDIO OUT ]

  */
  if (STRUCTURE == 20) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer PCM variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer Linear variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2  = &v_lp[2][0];
      float* pB2  = &v_bp[2][0];
      uint32_t* pW2  = &PWcount[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (PCM Sample Engine pointer) ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        uint32_t frac0 = pos0 & ((1 << step) - 1);
        if (idx0 < sampleend[0] - 1) {
          *pF0 += *pP0;
        } else if (loopsample[0]) {
          *pF0 = (uint32_t)samplebegin[0] << step;
        }
        int16_t s1_0 = *(genstartadress[0] + idx0);
        int16_t s2_0 = *(genstartadress[0] + idx0 + 1);
        float in0 = (float)(s1_0 + (((int32_t)(s2_0 - s1_0) * (int32_t)frac0) >> step));
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;

        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 LINEAR Engine pointer---
        *pF2 += *pP2;
        uint32_t ph2 = (*pF2 >> step) & 1023;
        float in2 = (Waveform[2] == 1) ? (float)((int32_t)(ph2 << 6) - 32768) : (ph2 < *pW2 ? 32767.0f : -32768.0f);
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 6;
        pF2++; pP2++; pL2++; pB2++; pW2++; pV2++; pFF2++;

        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] + osc_out[3]);
      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------2-------------------2---------------------PCM+LINEAR PCM+LINEAR------------------------------------
  /*
    ============================================================================
            ALGORITHM 22: PARALLEL DUAL MIX (PCM0+LA1  |  PCM2+LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      | (PCM 0)  |   |  (LA 1)  |        | (PCM 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Sample 0)   | (Synth 1)         | (Sample 2)   | (Synth 3)
           |              |                   |              |
           v              v                   v              v
        ( Sum: PCM0 + LA1 )                ( Sum: PCM2 + LA3 )
           |              |                   |              |
           +--------------+---------+---------+--------------+
                                    |
                                    v
                              [ AUDIO OUT ]

  */
  if (STRUCTURE == 22) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer PCM variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer PCM variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2     = &v_lp[2][0];
      float* pB2     = &v_bp[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];


        // --- OSC 0 PCM ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        uint32_t frac0 = pos0 & ((1 << step) - 1);
        const int16_t* pSample0 = genstartadress[0];
        int16_t s1_0, s2_0;
        if (idx0 < sampleend[0] - 1) {
          // NORMÁL LEJÁTSZÁS: Benne vagyunk a mintában
          s1_0 = pSample0[idx0];
          s2_0 = pSample0[idx0 + 1];
          *pF0 += *pP0; // Csak akkor lépünk, ha nem értük el a végét
        }
        else if (idx0 >= sampleend[0] - 1) {
          // HATÁR ESET: Elértük az utolsó mintát vagy túlfutottunk
          if (loopsample[0]) {
            // LOOP MÓD: Visszarántjuk az elejére
            *pF0 = (uint32_t)samplebegin[0] << step;
            idx0 = samplebegin[0];
            s1_0 = pSample0[idx0];
            s2_0 = pSample0[idx0 + 1];
            // Itt nem növelünk újra, mert a következő körben a pos0 már az eleje lesz
          } else {
            // ONE-SHOT MÓD: Megállítjuk a fázist és elnémítjuk a bemenetet
            s1_0 = 0;
            s2_0 = 0;
            // A *pF0-t NEM növeljük tovább, így ott marad a minta végén.
          }
        }
        // 3. Interpoláció (Már a tiszta s1_0, s2_0 értékekkel)
        float in0 = (float)s1_0 + (float)(s2_0 - s1_0) * (float)frac0 * (1.0f / (float)(1 << step));
        // 4. SZŰRŐ (Változatlanul gyors)
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        // Anti-pop / Limiter
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        // 5. Kimenet és Pointer léptetés
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;





        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;


        // --- OSC 2 PCM Engine pointer ---
        // --- OSC 2 (Végleges, optimalizált PCM - One-shot védelemmel) ---
        // 1. Alapadatok
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        uint32_t frac2 = pos2 & ((1 << step) - 1);
        const int16_t* pSample2 = genstartadress[2];

        int16_t s1_2, s2_2;

        // 2. Loop és határkezelés (Szigorú kontroll)
        if (idx2 < sampleend[2] - 1) {
          // NORMÁL LEJÁTSZÁS: Benne vagyunk a mintában
          s1_2 = pSample2[idx2];
          s2_2 = pSample2[idx2 + 1];
          *pF2 += *pP2; // Csak akkor lépünk, ha nem értük el a végét
        }
        else if (idx2 >= sampleend[2] - 1) {
          // HATÁR ESET: Elértük az utolsó mintát vagy túlfutottunk
          if (loopsample[2]) {
            // LOOP MÓD: Visszarántjuk az elejére
            *pF2 = (uint32_t)samplebegin[2] << step;
            idx2 = samplebegin[2];
            s1_2 = pSample2[idx2];
            s2_2 = pSample2[idx2 + 1];
          } else {
            // ONE-SHOT MÓD: Megállítjuk a fázist és elnémítjuk a bemenetet
            s1_2 = 0;
            s2_2 = 0;
            // A *pF2-t NEM növeljük tovább, így ott marad a minta végén.
          }
        }

        // 3. Interpoláció
        float in2 = (float)s1_2 + (float)(s2_2 - s1_2) * (float)frac2 * (1.0f / (float)(1 << step));

        // 4. SZŰRŐ (Kompakt és gyors)
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;

        // Anti-pop / Limiter (Biztonsági korlát)
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;

        // 5. Kimenet és Pointer léptetés
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;


        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] + osc_out[3]);
      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }



  //------------------3-------------------2---------------------PCM*LINEAR---PCM+LINEAR------------------------------------
  /*
    ============================================================================
            ALGORITHM 32: HYBRID RINGMOD/MIX (PCM0*LA1  |  PCM2+LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      | (PCM 0)  |   |  (LA 1)  |        | (PCM 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Sample 0)   | (Synth 1)         | (Sample 2)   | (Synth 3)
           |              |                   |              |
           +------+  +----+                   v              v
                  |  |                     ( Sum: PCM2 + LA3 )
                  v  v                        |              |
                +------+                      |              |
                |  (X) |                      |              |
                +--+---+                      |              |
                   |                          |              |
                   | (PCM0 * LA1)             |              |
                   +--------------------------+--------------+
                                              |
                                              v
                                        [ AUDIO OUT ]

  */
  if (STRUCTURE == 32) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer PCM variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer PCM variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2     = &v_lp[2][0];
      float* pB2     = &v_bp[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (PCM Sample Engine pointer) ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        uint32_t frac0 = pos0 & ((1 << step) - 1);
        if (idx0 < sampleend[0] - 1) {
          *pF0 += *pP0;
        } else if (loopsample[0]) {
          *pF0 = (uint32_t)samplebegin[0] << step;
        }
        int16_t s1_0 = *(genstartadress[0] + idx0);
        int16_t s2_0 = *(genstartadress[0] + idx0 + 1);
        float in0 = (float)(s1_0 + (((int32_t)(s2_0 - s1_0) * (int32_t)frac0) >> step));
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;

        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 PCM Engine pointer---
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        uint32_t frac2 = pos2 & ((1 << step) - 1);
        if (idx2 < sampleend[2] - 1) {
          *pF2 += *pP2;
        } else if (loopsample[2]) {
          *pF2 = (uint32_t)samplebegin[2] << step;
        }
        int16_t s1_2 = *(genstartadress[2] + idx2);
        int16_t s2_2 = *(genstartadress[2] + idx2 + 1);
        float in2 = (float)(s1_2 + (((int32_t)(s2_2 - s1_2) * (int32_t)frac2) >> step));
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;

        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += (osc_out[0] * (osc_out[1] >> 12)) >> 3;
        totalLower += (osc_out[2] + osc_out[3]);
      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------2-------------------3---------------------PCM+LINEAR---PCM*LINEAR------------------------------------
  /*
    ============================================================================
            ALGORITHM 23: HYBRID MIX/RINGMOD (PCM0+LA1  |  PCM2*LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      | (PCM 0)  |   |  (LA 1)  |        | (PCM 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Sample 0)   | (Synth 1)         | (Sample 2)   | (Synth 3)
           |              |                   |              |
           v              v                   +------+  +---+
        ( Sum: PCM0 + LA1 )                         |  |
           |              |                         v  v
           |              |                       +------+
           |              |                       |  (X) |  <-- RingMod (PCM2 * LA3)
           |              |                       +--+---+
           |              |                          |
           +--------------+--------------------------+
                          |
                          v
                    [ AUDIO OUT ]

  */
  if (STRUCTURE == 23) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer PCM variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer PCM variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2     = &v_lp[2][0];
      float* pB2     = &v_bp[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (PCM Sample Engine pointer) ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        uint32_t frac0 = pos0 & ((1 << step) - 1);
        if (idx0 < sampleend[0] - 1) {
          *pF0 += *pP0;
        } else if (loopsample[0]) {
          *pF0 = (uint32_t)samplebegin[0] << step;
        }
        int16_t s1_0 = *(genstartadress[0] + idx0);
        int16_t s2_0 = *(genstartadress[0] + idx0 + 1);
        float in0 = (float)(s1_0 + (((int32_t)(s2_0 - s1_0) * (int32_t)frac0) >> step));
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;

        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 PCM Engine pointer---
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        uint32_t frac2 = pos2 & ((1 << step) - 1);
        if (idx2 < sampleend[2] - 1) {
          *pF2 += *pP2;
        } else if (loopsample[2]) {
          *pF2 = (uint32_t)samplebegin[2] << step;
        }
        int16_t s1_2 = *(genstartadress[2] + idx2);
        int16_t s2_2 = *(genstartadress[2] + idx2 + 1);
        float in2 = (float)(s1_2 + (((int32_t)(s2_2 - s1_2) * (int32_t)frac2) >> step));
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;

        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += osc_out[0] + osc_out[1] ;
        totalLower += (osc_out[2] * (osc_out[3] >> 12)) >> 3;
      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------3-------------------3---------------------PWM*LINEAR------------------------------------
  /*
    ============================================================================
                  ALGORITHM 33: DUAL RINGMOD (PCM0*LA1 + PCM2*LA3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      | (PCM 0)  |   |  (LA 1)  |        | (PCM 2)  |   |  (LA 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Sample 0)   | (Synth 1)         | (Sample 2)   | (Synth 3)
           |              |                   |              |
           +------+  +----+                   +------+  +----+
                  |  |                               |  |
                  v  v                               v  v
                +------+                           +------+
                |  (X) |                           |  (X) |  <-- RingMod Processors
                +--+---+                           +--+---+
                   |                                  |
                   | (PCM0 * LA1)                     | (PCM2 * LA3)
                   +-----------------+----------------+
                                     |
                                     v
                               [ AUDIO OUT ]

  */
  if (STRUCTURE == 33 ) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      // Ezek gyűjtik a teljes polifóniát a két kimenetre
      int32_t totalUpper = 0; // Itt lesz a 0+1 mix
      int32_t totalLower = 0; // Itt lesz a 2*3 ring

      //osc0 pointer PCM variable
      uint32_t* pF0  = &freqmutato[0][0];
      uint32_t* pP0  = &pichcount[0][0];
      float* pL0     = &v_lp[0][0];
      float* pB0     = &v_bp[0][0];
      uint16_t* pV0      = &generatorvolume[0][0];
      float* pFF0    = &filter_f[0][0];

      //osc1 pointer Linear variable
      uint32_t* pF1  = &freqmutato[1][0];
      uint32_t* pP1  = &pichcount[1][0];
      float* pL1  = &v_lp[1][0];
      float* pB1  = &v_bp[1][0];
      uint32_t* pW1  = &PWcount[1][0];
      uint16_t* pV1  = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      //oc2 pointer PCM variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2     = &v_lp[2][0];
      float* pB2     = &v_bp[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer Linear variable
      uint32_t* pF3  = &freqmutato[3][0];
      uint32_t* pP3  = &pichcount[3][0];
      float* pL3  = &v_lp[3][0];
      float* pB3  = &v_bp[3][0];
      uint32_t* pW3  = &PWcount[3][0]; // uint32_t, ha az OSC 0-nál az vált be
      uint16_t* pV3      = &generatorvolume[3][0]; // byte-ra javítva!
      float* pFF3    = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (PCM Sample Engine pointer) ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        uint32_t frac0 = pos0 & ((1 << step) - 1);
        if (idx0 < sampleend[0] - 1) {
          *pF0 += *pP0;
        } else if (loopsample[0]) {
          *pF0 = (uint32_t)samplebegin[0] << step;
        }
        int16_t s1_0 = *(genstartadress[0] + idx0);
        int16_t s2_0 = *(genstartadress[0] + idx0 + 1);
        float in0 = (float)(s1_0 + (((int32_t)(s2_0 - s1_0) * (int32_t)frac0) >> step));
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f)  *pL0 = 32767.0f;
        if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;

        // --- OSC 1 LINEAR Engine pointer ---
        *pF1 += *pP1;
        uint32_t ph1 = (*pF1 >> step) & 1023;
        // Waveform[1]-et használjuk
        float in1 = (Waveform[1] == 1) ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f);
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 6;
        // Mutatók léptetése a j végén (az összesé!)
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;

        // --- OSC 2 PCM Engine pointer---
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        uint32_t frac2 = pos2 & ((1 << step) - 1);
        if (idx2 < sampleend[2] - 1) {
          *pF2 += *pP2;
        } else if (loopsample[2]) {
          *pF2 = (uint32_t)samplebegin[2] << step;
        }
        int16_t s1_2 = *(genstartadress[2] + idx2);
        int16_t s2_2 = *(genstartadress[2] + idx2 + 1);
        float in2 = (float)(s1_2 + (((int32_t)(s2_2 - s1_2) * (int32_t)frac2) >> step));
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f)  *pL2 = 32767.0f;
        if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;

        // --- OSC 3 LINEAR Engine pointer---
        *pF3 += *pP3;
        uint32_t ph3 = (*pF3 >> step) & 1023;
        float in3 = (Waveform[3] == 1) ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f);
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 6;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += (osc_out[0] * (osc_out[1] >> 12)) >> 3;
        totalLower += (osc_out[2] * (osc_out[3] >> 12)) >> 3;
      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //---------------------5-5-----------------PCM+PCM--PCM+PCM----------------------------
  /*
     ============================================================================
                    4-PCM QUAD PARALLEL MIXER (PCM 0..3)
     ============================================================================

       +----------+   +----------+   +----------+   +----------+
       |  OSC 0   |   |  OSC 1   |   |  OSC 2   |   |  OSC 3   |
       | (PCM 0)  |   | (PCM 1)  |   | (PCM 2)  |   | (PCM 3)  |
       +----+-----+   +----+-----+   +----+-----+   +----+-----+
            |              |              |              |
            | (Sample 0)   | (Sample 1)   | (Sample 2)   | (Sample 3)
            v              v              v              v
       +----+--------------+--------------+--------------+----+
       |                                                      |
       |                 PARALLEL SUM BUS                     |
       |                                                      |
       +--------------------------+---------------------------+
                                  |
                                  v
                            [ AUDIO OUT ]

     ============================================================================
     DSP Implementation note:
     int32_t total_out = (int32_t)sample0 + sample1 + sample2 + sample3;
     ============================================================================
  */
  if (STRUCTURE == 55) {
    // Kiszámoljuk előre az osztás reciprokát, így a ciklusban csak szorzunk
    const float invStep = 1.0f / (float)(1 << step);

    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      // Pointerek inicializálása - csak azokat hagyjuk meg, amik kellenek
      uint32_t *pF0 = &freqmutato[0][0], *pP0 = &pichcount[0][0];
      float *pL0 = &v_lp[0][0], *pB0 = &v_bp[0][0], *pFF0 = &filter_f[0][0];
      uint16_t *pV0 = &generatorvolume[0][0];

      uint32_t *pF1 = &freqmutato[1][0], *pP1 = &pichcount[1][0];
      float *pL1 = &v_lp[1][0], *pB1 = &v_bp[1][0], *pFF1 = &filter_f[1][0];
      uint16_t *pV1 = &generatorvolume[1][0];

      uint32_t *pF2 = &freqmutato[2][0], *pP2 = &pichcount[2][0];
      float *pL2 = &v_lp[2][0], *pB2 = &v_bp[2][0], *pFF2 = &filter_f[2][0];
      uint16_t *pV2 = &generatorvolume[2][0];

      uint32_t *pF3 = &freqmutato[3][0], *pP3 = &pichcount[3][0];
      float *pL3 = &v_lp[3][0], *pB3 = &v_bp[3][0], *pFF3 = &filter_f[3][0];
      uint16_t *pV3 = &generatorvolume[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4]; // Megtartva az öröklődéshez

        // --- OSC 0 PCM ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        const int16_t* pS0 = genstartadress[0];
        int16_t s1_0, s2_0;

        if (idx0 < sampleend[0] - 1) {
          s1_0 = pS0[idx0]; s2_0 = pS0[idx0 + 1];
          *pF0 += *pP0;
        } else if (loopsample[0]) {
          *pF0 = (uint32_t)samplebegin[0] << step;
          idx0 = samplebegin[0];
          s1_0 = pS0[idx0]; s2_0 = pS0[idx0 + 1];
        } else {
          s1_0 = s2_0 = 0;
        }

        float in0 = (float)s1_0 + (float)(s2_0 - s1_0) * (float)(pos0 & ((1 << step) - 1)) * invStep;
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0; *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f) *pL0 = 32767.0f; else if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;

        // --- OSC 1 PCM ---
        uint32_t pos1 = *pF1;
        uint32_t idx1 = pos1 >> step;
        const int16_t* pS1 = genstartadress[1];
        int16_t s1_1, s2_1;

        if (idx1 < sampleend[1] - 1) {
          s1_1 = pS1[idx1]; s2_1 = pS1[idx1 + 1];
          *pF1 += *pP1;
        } else if (loopsample[1]) {
          *pF1 = (uint32_t)samplebegin[1] << step;
          idx1 = samplebegin[1];
          s1_1 = pS1[idx1]; s2_1 = pS1[idx1 + 1];
        } else {
          s1_1 = s2_1 = 0;
        }

        float in1 = (float)s1_1 + (float)(s2_1 - s1_1) * (float)(pos1 & ((1 << step) - 1)) * invStep;
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1; *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f) *pL1 = 32767.0f; else if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 4;
        pF1++; pP1++; pL1++; pB1++; pV1++; pFF1++;

        // --- OSC 2 PCM ---
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        const int16_t* pS2 = genstartadress[2];
        int16_t s1_2, s2_2;

        if (idx2 < sampleend[2] - 1) {
          s1_2 = pS2[idx2]; s2_2 = pS2[idx2 + 1];
          *pF2 += *pP2;
        } else if (loopsample[2]) {
          *pF2 = (uint32_t)samplebegin[2] << step;
          idx2 = samplebegin[2];
          s1_2 = pS2[idx2]; s2_2 = pS2[idx2 + 1];
        } else {
          s1_2 = s2_2 = 0;
        }

        float in2 = (float)s1_2 + (float)(s2_2 - s1_2) * (float)(pos2 & ((1 << step) - 1)) * invStep;
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2; *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f) *pL2 = 32767.0f; else if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;

        // --- OSC 3 PCM ---
        uint32_t pos3 = *pF3;
        uint32_t idx3 = pos3 >> step;
        const int16_t* pS3 = genstartadress[3];
        int16_t s1_3, s2_3;

        if (idx3 < sampleend[3] - 1) {
          s1_3 = pS3[idx3]; s2_3 = pS3[idx3 + 1];
          *pF3 += *pP3;
        } else if (loopsample[3]) {
          *pF3 = (uint32_t)samplebegin[3] << step;
          idx3 = samplebegin[3];
          s1_3 = pS3[idx3]; s2_3 = pS3[idx3 + 1];
        } else {
          s1_3 = s2_3 = 0;
        }

        float in3 = (float)s1_3 + (float)(s2_3 - s1_3) * (float)(pos3 & ((1 << step) - 1)) * invStep;
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3; *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f) *pL3 = 32767.0f; else if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 4;
        pF3++; pP3++; pL3++; pB3++; pV3++; pFF3++;

        totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] + osc_out[3]);
      }
      // --- Stereo Mix & Final Processing ---
      bufferbe[0] = (totalUpper + (totalLower >> 2)) >> masterVolume;
      bufferbe[1] = (totalLower + (totalUpper >> 2)) >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //----------------------5-6-----PCM+PCM------PCM*PCM---------------------------------------------------
  /*
    ============================================================================
              4-PCM ENGINE: MIX (PCM0 + PCM1) & RINGMOD (PCM2 * PCM3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      | (PCM 0)  |   | (PCM 1)  |        | (PCM 2)  |   | (PCM 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           |              |                   |              |
           v              v                   +------+  +---+
        ( Sum: PCM0 + PCM1 )                        |  |
           |              |                        v  v
           |              |                      +------+
           |              |                      |  (X) |  <-- RingMod (PCM2 * PCM3)
           |              |                      +--+---+
           |              |                         |
           +--------------+-------------------------+
                          |
                          v
                    [ AUDIO OUT ]

    ============================================================================
  */

  if (STRUCTURE == 56) {
    // Kiszámoljuk előre az osztás reciprokát, így a ciklusban csak szorzunk
    const float invStep = 1.0f / (float)(1 << step);

    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      // Pointerek inicializálása - csak azokat hagyjuk meg, amik kellenek
      uint32_t *pF0 = &freqmutato[0][0], *pP0 = &pichcount[0][0];
      float *pL0 = &v_lp[0][0], *pB0 = &v_bp[0][0], *pFF0 = &filter_f[0][0];
      uint16_t *pV0 = &generatorvolume[0][0];

      uint32_t *pF1 = &freqmutato[1][0], *pP1 = &pichcount[1][0];
      float *pL1 = &v_lp[1][0], *pB1 = &v_bp[1][0], *pFF1 = &filter_f[1][0];
      uint16_t *pV1 = &generatorvolume[1][0];

      uint32_t *pF2 = &freqmutato[2][0], *pP2 = &pichcount[2][0];
      float *pL2 = &v_lp[2][0], *pB2 = &v_bp[2][0], *pFF2 = &filter_f[2][0];
      uint16_t *pV2 = &generatorvolume[2][0];

      uint32_t *pF3 = &freqmutato[3][0], *pP3 = &pichcount[3][0];
      float *pL3 = &v_lp[3][0], *pB3 = &v_bp[3][0], *pFF3 = &filter_f[3][0];
      uint16_t *pV3 = &generatorvolume[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4]; // Megtartva az öröklődéshez

        // --- OSC 0 PCM ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        const int16_t* pS0 = genstartadress[0];
        int16_t s1_0, s2_0;

        if (idx0 < sampleend[0] - 1) {
          s1_0 = pS0[idx0]; s2_0 = pS0[idx0 + 1];
          *pF0 += *pP0;
        } else if (loopsample[0]) {
          *pF0 = (uint32_t)samplebegin[0] << step;
          idx0 = samplebegin[0];
          s1_0 = pS0[idx0]; s2_0 = pS0[idx0 + 1];
        } else {
          s1_0 = s2_0 = 0;
        }

        float in0 = (float)s1_0 + (float)(s2_0 - s1_0) * (float)(pos0 & ((1 << step) - 1)) * invStep;
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0; *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f) *pL0 = 32767.0f; else if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;

        // --- OSC 1 PCM ---
        uint32_t pos1 = *pF1;
        uint32_t idx1 = pos1 >> step;
        const int16_t* pS1 = genstartadress[1];
        int16_t s1_1, s2_1;

        if (idx1 < sampleend[1] - 1) {
          s1_1 = pS1[idx1]; s2_1 = pS1[idx1 + 1];
          *pF1 += *pP1;
        } else if (loopsample[1]) {
          *pF1 = (uint32_t)samplebegin[1] << step;
          idx1 = samplebegin[1];
          s1_1 = pS1[idx1]; s2_1 = pS1[idx1 + 1];
        } else {
          s1_1 = s2_1 = 0;
        }

        float in1 = (float)s1_1 + (float)(s2_1 - s1_1) * (float)(pos1 & ((1 << step) - 1)) * invStep;
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1; *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f) *pL1 = 32767.0f; else if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 4;
        pF1++; pP1++; pL1++; pB1++; pV1++; pFF1++;

        // --- OSC 2 PCM ---
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        const int16_t* pS2 = genstartadress[2];
        int16_t s1_2, s2_2;

        if (idx2 < sampleend[2] - 1) {
          s1_2 = pS2[idx2]; s2_2 = pS2[idx2 + 1];
          *pF2 += *pP2;
        } else if (loopsample[2]) {
          *pF2 = (uint32_t)samplebegin[2] << step;
          idx2 = samplebegin[2];
          s1_2 = pS2[idx2]; s2_2 = pS2[idx2 + 1];
        } else {
          s1_2 = s2_2 = 0;
        }

        float in2 = (float)s1_2 + (float)(s2_2 - s1_2) * (float)(pos2 & ((1 << step) - 1)) * invStep;
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2; *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f) *pL2 = 32767.0f; else if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;

        // --- OSC 3 PCM ---
        uint32_t pos3 = *pF3;
        uint32_t idx3 = pos3 >> step;
        const int16_t* pS3 = genstartadress[3];
        int16_t s1_3, s2_3;

        if (idx3 < sampleend[3] - 1) {
          s1_3 = pS3[idx3]; s2_3 = pS3[idx3 + 1];
          *pF3 += *pP3;
        } else if (loopsample[3]) {
          *pF3 = (uint32_t)samplebegin[3] << step;
          idx3 = samplebegin[3];
          s1_3 = pS3[idx3]; s2_3 = pS3[idx3 + 1];
        } else {
          s1_3 = s2_3 = 0;
        }

        float in3 = (float)s1_3 + (float)(s2_3 - s1_3) * (float)(pos3 & ((1 << step) - 1)) * invStep;
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3; *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f) *pL3 = 32767.0f; else if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 4;
        pF3++; pP3++; pL3++; pB3++; pV3++; pFF3++;

        totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] * osc_out[3]) >> 12;
      }
      // --- Stereo Mix & Final Processing ---
      bufferbe[0] = (totalUpper + (totalLower >> 2)) >> masterVolume;
      bufferbe[1] = (totalLower + (totalUpper >> 2)) >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //----------------------5-7-----PCM+PCM------FM+FM---------------------------------------------------
  /*
    ============================================================================
                  ALGORITHM 57: DUAL PCM + 2-OP FM
    ============================================================================

                                               +----------+
                                            +->|  OSC 2   | (FM Modulator)
                                            |  |  (OP 2)  |
                                           (Self-+----+-----+
                                            FB)  |    |
                                                 |    | (mod = lastOut[2] * pSV2)
                                                 v    v
      +----------+        +----------+         +----------+
      |  OSC 0   |        |  OSC 1   |      +->|  OSC 3   | (FM Carrier)
      | (PCM 0)  |        | (PCM 1)  |      |  |  (OP 3)  |
      +----+-----+        +----+-----+     (Self-+----+-----+
           |                   |            FB)  |    |
           | (osc_out[0])      | (osc_out[1])    |    | (osc_out[3])
           |                   |                 |    |
           +-------------------+-----------------+----+
                               |
                               v
                         [ AUDIO OUT ]

    ============================================================================
    Features in Alg 57:
    - OSC 0 & OSC 1: Independent PCM Sample playback engines with SVF filters.
    - OSC 2 -> OSC 3: 2-Operator FM engine with dual feedback loops.
    ============================================================================
  */
  if (STRUCTURE == 57) {
    const float invStep = 1.0f / (float)(1 << step);

    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      // Pointerek inicializálása (Hozzáadva a pSV-k)
      uint32_t *pF0 = &freqmutato[0][0], *pP0 = &pichcount[0][0];
      float *pL0 = &v_lp[0][0], *pB0 = &v_bp[0][0], *pFF0 = &filter_f[0][0];
      uint16_t *pV0 = &generatorvolume[0][0], *pSV0 = &smoothedVol[0][0];

      uint32_t *pF1 = &freqmutato[1][0], *pP1 = &pichcount[1][0];
      float *pL1 = &v_lp[1][0], *pB1 = &v_bp[1][0], *pFF1 = &filter_f[1][0];
      uint16_t *pV1 = &generatorvolume[1][0], *pSV1 = &smoothedVol[1][0];

      uint32_t *pF2 = &freqmutato[2][0], *pP2 = &pichcount[2][0];
      float *pL2 = &v_lp[2][0], *pB2 = &v_bp[2][0], *pFF2 = &filter_f[2][0];
      uint16_t *pV2 = &generatorvolume[2][0], *pSV2 = &smoothedVol[2][0];
      uint32_t *pW2 = &PWcount[2][0];

      uint32_t *pF3 = &freqmutato[3][0], *pP3 = &pichcount[3][0];
      float *pL3 = &v_lp[3][0], *pB3 = &v_bp[3][0], *pFF3 = &filter_f[3][0];
      uint16_t *pV3 = &generatorvolume[3][0], *pSV3 = &smoothedVol[3][0];
      uint32_t *pW3 = &PWcount[3][0];
      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4];

        // --- OSC 0 (PCM 1) ---
        {
          uint32_t pos = *pF0;
          uint32_t idx = pos >> step;
          const int16_t* pS = genstartadress[0];
          int16_t s1, s2;
          if (idx < sampleend[0] - 1) {
            s1 = pS[idx]; s2 = pS[idx + 1];
            *pF0 += *pP0;
          } else if (loopsample[0]) {
            *pF0 = (uint32_t)samplebegin[0] << step;
            idx = samplebegin[0];
            s1 = pS[idx]; s2 = pS[idx + 1];
          } else {
            s1 = s2 = 0;
          }
          float in = (float)s1 + (float)(s2 - s1) * (float)(pos & ((1 << step) - 1)) * invStep;
          float hp = in - *pL0 - (filter_q[0] * *pB0);
          *pB0 += *pFF0 * hp; *pL0 += *pFF0 * *pB0;
          if (*pSV0 < *pV0) (*pSV0)++; else if (*pSV0 > *pV0) (*pSV0)--; // Simító
          osc_out[0] = ((int32_t) * pL0 * *pSV0) >> 4;

        }

        // --- OSC 1 (PCM 2) ---
        {
          uint32_t pos = *pF1;
          uint32_t idx = pos >> step;
          const int16_t* pS = genstartadress[1];
          int16_t s1, s2;
          if (idx < sampleend[1] - 1) {
            s1 = pS[idx]; s2 = pS[idx + 1];
            *pF1 += *pP1;
          } else if (loopsample[1]) {
            *pF1 = (uint32_t)samplebegin[1] << step;
            idx = samplebegin[1];
            s1 = pS[idx]; s2 = pS[idx + 1];
          } else {
            s1 = s2 = 0;
          }
          float in = (float)s1 + (float)(s2 - s1) * (float)(pos & ((1 << step) - 1)) * invStep;
          float hp = in - *pL1 - (filter_q[1] * *pB1);
          *pB1 += *pFF1 * hp; *pL1 += *pFF1 * *pB1;
          if (*pSV1 < *pV1) (*pSV1)++; else if (*pSV1 > *pV1) (*pSV1)--; // Simító
          osc_out[1] = ((int32_t) * pL1 * *pSV1) >> 4;
        }

        // --- OSC 2 (FM Modulator + Feedback) ---
        {
          *pF2 += *pP2;
          float feedback = lastOut[2][j] * (*pW2 * 0.00001f);
          if (*pW2 < 2) feedback = 0.0f;

          uint32_t ph = ((*pF2 >> step) + (int32_t)feedback) & 1023;
          float s = (Waveform[2] == 0) ? sinTable[ph] : (float)((int32_t)(ph << 6) - 32768);

          // Szűrés a modulátornak is (opcionális, de benne hagytam a konzisztencia miatt)
          float hp = s - *pL2 - (filter_q[2] * *pB2);
          *pB2 += *pFF2 * hp; *pL2 += *pFF2 * *pB2;
          lastOut[2][j] = *pL2;
          if (*pSV2 < *pV2) (*pSV2)++; else if (*pSV2 > *pV2) (*pSV2)--; // Simító
          osc_out[2] = ((int32_t) * pL2 * *pSV2) >> 6;
        }

        // --- OSC 3 (FM Carrier - Modulated by OSC 2) ---
        {
          *pF3 += *pP3;
          // Moduláció az OSC 2 kimenetéről
          // float mod = lastOut[2][j] * (*pV2 * 0.0002f);
          float mod = lastOut[2][j] * (*pSV2 * 0.0002f);
          float feedback = lastOut[3][j] * (*pW3 * 0.00001f);
          if (*pW3 < 2) feedback = 0.0f;

          uint32_t ph = ((*pF3 >> step) + (int32_t)mod + (int32_t)feedback) & 1023;
          float in = (Waveform[3] == 0) ? sinTable[ph] : (float)((int32_t)(ph << 6) - 32768);

          float hp = in - *pL3 - (filter_q[3] * *pB3);
          *pB3 += *pFF3 * (hp > 32767.0f ? 32767.0f : (hp < -32768.0f ? -32768.0f : hp));
          *pL3 += *pFF3 * *pB3;
          lastOut[3][j] = *pL3;
          if (*pSV3 < *pV3) (*pSV3)++; else if (*pSV3 > *pV3) (*pSV3)--; // Simító
          osc_out[3] = ((int32_t) * pL3 * *pSV3) >> 6;
        }


        // Összegzés: Upper = PCM-ek, Lower = FM-ek
        totalUpper += (osc_out[3] + osc_out[0]);
        totalLower += (osc_out[3]) + osc_out[1];

        pF0++; pP0++; pL0++; pB0++; pFF0++; pV0++; pSV0++;
        pF1++; pP1++; pL1++; pB1++; pFF1++; pV1++; pSV1++;
        pF2++; pP2++; pL2++; pB2++; pFF2++; pV2++; pSV2++; pW2++;
        pF3++; pP3++; pL3++; pB3++; pFF3++; pV3++; pSV3++; pW3++;
      }

      // Stereo Mix & Effekt lánc
      bufferbe[0] = (totalUpper + (totalLower >> 2)) >> masterVolume;
      bufferbe[1] = (totalLower + (totalUpper >> 2)) >> masterVolume;

      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;

      chorusleft(); chorusright();
      processingStereoReverb();

      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }



  //----------------------6-5-----PCM+PCM------PCM*PCM---------------------------------------------------
  if (STRUCTURE == 65) {
    // Kiszámoljuk előre az osztás reciprokát, így a ciklusban csak szorzunk
    const float invStep = 1.0f / (float)(1 << step);

    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      // Pointerek inicializálása - csak azokat hagyjuk meg, amik kellenek
      uint32_t *pF0 = &freqmutato[0][0], *pP0 = &pichcount[0][0];
      float *pL0 = &v_lp[0][0], *pB0 = &v_bp[0][0], *pFF0 = &filter_f[0][0];
      uint16_t *pV0 = &generatorvolume[0][0];

      uint32_t *pF1 = &freqmutato[1][0], *pP1 = &pichcount[1][0];
      float *pL1 = &v_lp[1][0], *pB1 = &v_bp[1][0], *pFF1 = &filter_f[1][0];
      uint16_t *pV1 = &generatorvolume[1][0];

      uint32_t *pF2 = &freqmutato[2][0], *pP2 = &pichcount[2][0];
      float *pL2 = &v_lp[2][0], *pB2 = &v_bp[2][0], *pFF2 = &filter_f[2][0];
      uint16_t *pV2 = &generatorvolume[2][0];

      uint32_t *pF3 = &freqmutato[3][0], *pP3 = &pichcount[3][0];
      float *pL3 = &v_lp[3][0], *pB3 = &v_bp[3][0], *pFF3 = &filter_f[3][0];
      uint16_t *pV3 = &generatorvolume[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4]; // Megtartva az öröklődéshez

        // --- OSC 0 PCM ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        const int16_t* pS0 = genstartadress[0];
        int16_t s1_0, s2_0;

        if (idx0 < sampleend[0] - 1) {
          s1_0 = pS0[idx0]; s2_0 = pS0[idx0 + 1];
          *pF0 += *pP0;
        } else if (loopsample[0]) {
          *pF0 = (uint32_t)samplebegin[0] << step;
          idx0 = samplebegin[0];
          s1_0 = pS0[idx0]; s2_0 = pS0[idx0 + 1];
        } else {
          s1_0 = s2_0 = 0;
        }

        float in0 = (float)s1_0 + (float)(s2_0 - s1_0) * (float)(pos0 & ((1 << step) - 1)) * invStep;
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0; *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f) *pL0 = 32767.0f; else if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;

        // --- OSC 1 PCM ---
        uint32_t pos1 = *pF1;
        uint32_t idx1 = pos1 >> step;
        const int16_t* pS1 = genstartadress[1];
        int16_t s1_1, s2_1;

        if (idx1 < sampleend[1] - 1) {
          s1_1 = pS1[idx1]; s2_1 = pS1[idx1 + 1];
          *pF1 += *pP1;
        } else if (loopsample[1]) {
          *pF1 = (uint32_t)samplebegin[1] << step;
          idx1 = samplebegin[1];
          s1_1 = pS1[idx1]; s2_1 = pS1[idx1 + 1];
        } else {
          s1_1 = s2_1 = 0;
        }

        float in1 = (float)s1_1 + (float)(s2_1 - s1_1) * (float)(pos1 & ((1 << step) - 1)) * invStep;
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1; *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f) *pL1 = 32767.0f; else if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 4;
        pF1++; pP1++; pL1++; pB1++; pV1++; pFF1++;

        // --- OSC 2 PCM ---
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        const int16_t* pS2 = genstartadress[2];
        int16_t s1_2, s2_2;

        if (idx2 < sampleend[2] - 1) {
          s1_2 = pS2[idx2]; s2_2 = pS2[idx2 + 1];
          *pF2 += *pP2;
        } else if (loopsample[2]) {
          *pF2 = (uint32_t)samplebegin[2] << step;
          idx2 = samplebegin[2];
          s1_2 = pS2[idx2]; s2_2 = pS2[idx2 + 1];
        } else {
          s1_2 = s2_2 = 0;
        }

        float in2 = (float)s1_2 + (float)(s2_2 - s1_2) * (float)(pos2 & ((1 << step) - 1)) * invStep;
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2; *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f) *pL2 = 32767.0f; else if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;

        // --- OSC 3 PCM ---
        uint32_t pos3 = *pF3;
        uint32_t idx3 = pos3 >> step;
        const int16_t* pS3 = genstartadress[3];
        int16_t s1_3, s2_3;

        if (idx3 < sampleend[3] - 1) {
          s1_3 = pS3[idx3]; s2_3 = pS3[idx3 + 1];
          *pF3 += *pP3;
        } else if (loopsample[3]) {
          *pF3 = (uint32_t)samplebegin[3] << step;
          idx3 = samplebegin[3];
          s1_3 = pS3[idx3]; s2_3 = pS3[idx3 + 1];
        } else {
          s1_3 = s2_3 = 0;
        }

        float in3 = (float)s1_3 + (float)(s2_3 - s1_3) * (float)(pos3 & ((1 << step) - 1)) * invStep;
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3; *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f) *pL3 = 32767.0f; else if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 4;
        pF3++; pP3++; pL3++; pB3++; pV3++; pFF3++;

        totalUpper += (osc_out[0] * osc_out[1]) >> 12;
        totalLower += (osc_out[2] + osc_out[3]);
      }

      // --- Stereo Mix & Final Processing ---
      bufferbe[0] = (totalUpper + (totalLower >> 2)) >> masterVolume;
      bufferbe[1] = (totalLower + (totalUpper >> 2)) >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }




  //----------------------6-6-----PCM*PCM------PCM*PCM----------------------------------------------------
  /*
    ============================================================================
            ALGORITHM 66: DUAL PCM RINGMOD (PCM0*PCM1  |  PCM2*PCM3)
    ============================================================================

      +----------+   +----------+        +----------+   +----------+
      |  OSC 0   |   |  OSC 1   |        |  OSC 2   |   |  OSC 3   |
      | (PCM 0)  |   | (PCM 1)  |        | (PCM 2)  |   | (PCM 3)  |
      +----+-----+   +----+-----+        +----+-----+   +----+-----+
           |              |                   |              |
           | (Sample 0)   | (Sample 1)        | (Sample 2)   | (Sample 3)
           |              |                   |              |
           +------+  +----+                   +------+  +----+
                  |  |                               |  |
                  v  v                               v  v
                +------+                           +------+
                |  (X) |  <-- RingMod              |  (X) |  <-- RingMod
                +--+---+     (PCM0 * PCM1)         +--+---+     (PCM2 * PCM3)
                   |                                  |
                   +-----------------+----------------+
                                     |
                                     v
                               [ AUDIO OUT ]

  */
  if (STRUCTURE == 66) {
    // Kiszámoljuk előre az osztás reciprokát, így a ciklusban csak szorzunk
    const float invStep = 1.0f / (float)(1 << step);

    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      // Pointerek inicializálása - csak azokat hagyjuk meg, amik kellenek
      uint32_t *pF0 = &freqmutato[0][0], *pP0 = &pichcount[0][0];
      float *pL0 = &v_lp[0][0], *pB0 = &v_bp[0][0], *pFF0 = &filter_f[0][0];
      uint16_t *pV0 = &generatorvolume[0][0];

      uint32_t *pF1 = &freqmutato[1][0], *pP1 = &pichcount[1][0];
      float *pL1 = &v_lp[1][0], *pB1 = &v_bp[1][0], *pFF1 = &filter_f[1][0];
      uint16_t *pV1 = &generatorvolume[1][0];

      uint32_t *pF2 = &freqmutato[2][0], *pP2 = &pichcount[2][0];
      float *pL2 = &v_lp[2][0], *pB2 = &v_bp[2][0], *pFF2 = &filter_f[2][0];
      uint16_t *pV2 = &generatorvolume[2][0];

      uint32_t *pF3 = &freqmutato[3][0], *pP3 = &pichcount[3][0];
      float *pL3 = &v_lp[3][0], *pB3 = &v_bp[3][0], *pFF3 = &filter_f[3][0];
      uint16_t *pV3 = &generatorvolume[3][0];

      for (int j = 0; j < polyphony; j++) {
        int32_t osc_out[4]; // Megtartva az öröklődéshez

        // --- OSC 0 PCM ---
        uint32_t pos0 = *pF0;
        uint32_t idx0 = pos0 >> step;
        const int16_t* pS0 = genstartadress[0];
        int16_t s1_0, s2_0;

        if (idx0 < sampleend[0] - 1) {
          s1_0 = pS0[idx0]; s2_0 = pS0[idx0 + 1];
          *pF0 += *pP0;
        } else if (loopsample[0]) {
          *pF0 = (uint32_t)samplebegin[0] << step;
          idx0 = samplebegin[0];
          s1_0 = pS0[idx0]; s2_0 = pS0[idx0 + 1];
        } else {
          s1_0 = s2_0 = 0;
        }

        float in0 = (float)s1_0 + (float)(s2_0 - s1_0) * (float)(pos0 & ((1 << step) - 1)) * invStep;
        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0; *pL0 += *pFF0 * *pB0;
        if (*pL0 > 32767.0f) *pL0 = 32767.0f; else if (*pL0 < -32768.0f) *pL0 = -32768.0f;
        osc_out[0] = ((int32_t) * pL0 * *pV0) >> 4;
        pF0++; pP0++; pL0++; pB0++; pV0++; pFF0++;

        // --- OSC 1 PCM ---
        uint32_t pos1 = *pF1;
        uint32_t idx1 = pos1 >> step;
        const int16_t* pS1 = genstartadress[1];
        int16_t s1_1, s2_1;

        if (idx1 < sampleend[1] - 1) {
          s1_1 = pS1[idx1]; s2_1 = pS1[idx1 + 1];
          *pF1 += *pP1;
        } else if (loopsample[1]) {
          *pF1 = (uint32_t)samplebegin[1] << step;
          idx1 = samplebegin[1];
          s1_1 = pS1[idx1]; s2_1 = pS1[idx1 + 1];
        } else {
          s1_1 = s2_1 = 0;
        }

        float in1 = (float)s1_1 + (float)(s2_1 - s1_1) * (float)(pos1 & ((1 << step) - 1)) * invStep;
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1; *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f) *pL1 = 32767.0f; else if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 4;
        pF1++; pP1++; pL1++; pB1++; pV1++; pFF1++;

        // --- OSC 2 PCM ---
        uint32_t pos2 = *pF2;
        uint32_t idx2 = pos2 >> step;
        const int16_t* pS2 = genstartadress[2];
        int16_t s1_2, s2_2;

        if (idx2 < sampleend[2] - 1) {
          s1_2 = pS2[idx2]; s2_2 = pS2[idx2 + 1];
          *pF2 += *pP2;
        } else if (loopsample[2]) {
          *pF2 = (uint32_t)samplebegin[2] << step;
          idx2 = samplebegin[2];
          s1_2 = pS2[idx2]; s2_2 = pS2[idx2 + 1];
        } else {
          s1_2 = s2_2 = 0;
        }

        float in2 = (float)s1_2 + (float)(s2_2 - s1_2) * (float)(pos2 & ((1 << step) - 1)) * invStep;
        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2; *pL2 += *pFF2 * *pB2;
        if (*pL2 > 32767.0f) *pL2 = 32767.0f; else if (*pL2 < -32768.0f) *pL2 = -32768.0f;
        osc_out[2] = ((int32_t) * pL2 * *pV2) >> 4;
        pF2++; pP2++; pL2++; pB2++; pV2++; pFF2++;

        // --- OSC 3 PCM ---
        uint32_t pos3 = *pF3;
        uint32_t idx3 = pos3 >> step;
        const int16_t* pS3 = genstartadress[3];
        int16_t s1_3, s2_3;

        if (idx3 < sampleend[3] - 1) {
          s1_3 = pS3[idx3]; s2_3 = pS3[idx3 + 1];
          *pF3 += *pP3;
        } else if (loopsample[3]) {
          *pF3 = (uint32_t)samplebegin[3] << step;
          idx3 = samplebegin[3];
          s1_3 = pS3[idx3]; s2_3 = pS3[idx3 + 1];
        } else {
          s1_3 = s2_3 = 0;
        }

        float in3 = (float)s1_3 + (float)(s2_3 - s1_3) * (float)(pos3 & ((1 << step) - 1)) * invStep;
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3; *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f) *pL3 = 32767.0f; else if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 4;
        pF3++; pP3++; pL3++; pB3++; pV3++; pFF3++;

        totalUpper += (osc_out[0] * osc_out[1]) >> 12;
        totalLower += (osc_out[2] * osc_out[3]) >> 12;
      }

      // --- Stereo Mix & Final Processing ---
      bufferbe[0] = (totalUpper + (totalLower >> 2)) >> masterVolume;
      bufferbe[1] = (totalLower + (totalUpper >> 2)) >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //----------------------6-7-----fm-------Low Compensed------------------------------------------------
  /*
     ============================================================================
                   ALGORITHM 67: LOW COMPENSATED (OPTIMIZED 4-OP)
     ============================================================================

           +--------------------------------------------------------+
           |                                                        |
           |       +------------------------------------+           |
           |       | (Cross-Mod: OSC 1 -> OSC 0)        |           |
           v       v                                    |           |
        +-------------+  (fb0_scale)                    |           |
     +->|    OSC 0    |<-------+                        |           |
     |  | (OP 2 /Mod1)|        |                        |           |
     |  +------+------+        |                        |           |
     | (Self-  |               |                        |           |
     |  FB)    | (in0*d0_scale)|                        |           |
     +---------+               |                        |           |
               v               |                        |           |
        +-------------+        |                        |           |
        |    OSC 1    |--------+------------------------+           |
        | (OP 1 /Car1)|                                             |
        +------+------+                                             |
               |                                                    |
               +-----------------------> [ AUDIO OUT ] <------+     |
                                              ^               |     |
                                              |               |     |
        +-------------+                        | (out3)        |     |
     +->|    OSC 2    |                        |               |     |
     |  | (OP 4 /Mod2)|                  +-----+-------+       |     |
     |  +------+------+                  |    OSC 3    |-------+-----+
     | (Self-  |                         | (OP 3 /Car2)|  (Link: OSC 3 -> OSC 1)
     |  FB)    | (in2*d2_scale)          +-------------+
     +---------+                               ^
               |                               |
               +-------------------------------+

     ============================================================================
     Features in Alg 67:
     - Pre-calculated quadratic scale factors (PWcount^2, generatorvolume^2).
     - Per-operator SVF filters with soft-clip limiting (+/- 20000.0f threshold).
     ============================================================================
  */


  if (STRUCTURE == 67) {
    // --- Konstansok és előkalkulált értékek a belső ciklus előtt ---
    const float inv10000_0015 = 0.0001f * 0.0015f;
    const float inv10000_05 = 0.0001f * 0.05f;
    const float inv100_01 = 0.01f * 0.01f;

    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      for (int j = 0; j < polyphony; j++) {
        // Előkészítjük a paramétereket, amik a hangon belül fixek (nem kell minden j-re pointerezni, ha konstansok)
        // Megjegyzés: Ha a paraméterek (pl. pW0) hangonként változnak, marad a pointer,
        // de a négyzetre emelést elvégezhetjük itt egyszer:
        float fb0_scale = (float)(PWcount[0][j] * PWcount[0][j]) * inv10000_0015;
        float fb1_scale = (float)(PWcount[1][j] * PWcount[1][j]) * inv10000_0015;
        float fb2_scale = (float)(PWcount[2][j] * PWcount[2][j]) * inv10000_0015;
        float fb3_scale = (float)(PWcount[3][j] * PWcount[3][j]) * inv10000_0015;

        float d0_scale = (float)(generatorvolume[0][j] * generatorvolume[0][j]) * inv10000_05;
        float d2_scale = (float)(generatorvolume[2][j] * generatorvolume[2][j]) * inv10000_05;
        float link1_val = (float)PWcount[3][j] * inv100_01;
        float cross0_val = (float)PWcount[1][j] * inv100_01;

        // --- OSC 0 ---
        freqmutato[0][j] += pichcount[0][j];
        float fb0 = (lastOut[0][j] * fb0_scale) + (lastOut[1][j] * cross0_val);
        uint32_t ph0 = ((freqmutato[0][j] >> step) + (int32_t)fb0) & 1023;

        // Gyorsított Waveform választó (csak szinuszra példázva, a többit is így tördeld be)
        float in0 = (Waveform[0] == 0) ? sinTable[ph0] : (Waveform[0] == 1 ? (float)((int32_t)(ph0 << 6) - 32768) : (ph0 < PWcount[0][j] ? 32767.0f : -32768.0f));

        // Filter (SVF) - marad a float, mert a stabilitás fontos
        float hp0 = in0 - v_lp[0][j] - (filter_q[0] * v_bp[0][j]);
        v_bp[0][j] += filter_f[0][j] * hp0;
        v_lp[0][j] += filter_f[0][j] * v_bp[0][j];

        // Soft clip (gyorsabb if-ekkel)
        if (v_lp[0][j] > 20000.0f) v_lp[0][j] = 20000.0f + (v_lp[0][j] - 20000.0f) * 0.2f;
        else if (v_lp[0][j] < -20000.0f) v_lp[0][j] = -20000.0f + (v_lp[0][j] + 20000.0f) * 0.2f;
        lastOut[0][j] = v_lp[0][j];

        // --- OSC 1 ---
        freqmutato[1][j] += pichcount[1][j];
        float fb1 = lastOut[1][j] * fb1_scale;
        float totalMod1 = (in0 * d0_scale) + (lastOut[3][j] * link1_val);
        uint32_t ph1 = ((freqmutato[1][j] >> step) + (int32_t)totalMod1 + (int32_t)fb1) & 1023;
        float in1 = (Waveform[1] == 0) ? sinTable[ph1] : (Waveform[1] == 1 ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < PWcount[1][j] ? 32767.0f : -32768.0f));

        // Filter 1 + Soft clip
        float hp1 = in1 - v_lp[1][j] - (filter_q[1] * v_bp[1][j]);
        v_bp[1][j] += filter_f[1][j] * hp1;
        v_lp[1][j] += filter_f[1][j] * v_bp[1][j];
        if (v_lp[1][j] > 20000.0f) v_lp[1][j] = 20000.0f + (v_lp[1][j] - 20000.0f) * 0.2f;
        else if (v_lp[1][j] < -20000.0f) v_lp[1][j] = -20000.0f + (v_lp[1][j] + 20000.0f) * 0.2f;
        lastOut[1][j] = v_lp[1][j];
        int32_t out1 = ((int32_t)v_lp[1][j] * generatorvolume[1][j]) >> 6;

        // --- OSC 2 ---
        freqmutato[2][j] += pichcount[2][j];
        float fb2 = lastOut[2][j] * fb2_scale;
        uint32_t ph2 = ((freqmutato[2][j] >> step) + (int32_t)fb2) & 1023;
        float in2 = (Waveform[2] == 0) ? sinTable[ph2] : (Waveform[2] == 1 ? (float)((int32_t)(ph2 << 6) - 32768) : (ph2 < PWcount[2][j] ? 32767.0f : -32768.0f));

        // Filter 2 + Soft clip
        float hp2 = in2 - v_lp[2][j] - (filter_q[2] * v_bp[2][j]);
        v_bp[2][j] += filter_f[2][j] * hp2;
        v_lp[2][j] += filter_f[2][j] * v_bp[2][j];
        if (v_lp[2][j] > 20000.0f) v_lp[2][j] = 20000.0f + (v_lp[2][j] - 20000.0f) * 0.2f;
        else if (v_lp[2][j] < -20000.0f) v_lp[2][j] = -20000.0f + (v_lp[2][j] + 20000.0f) * 0.2f;
        lastOut[2][j] = v_lp[2][j];

        // --- OSC 3 ---
        freqmutato[3][j] += pichcount[3][j];
        float fb3 = lastOut[3][j] * fb3_scale;
        uint32_t ph3 = ((freqmutato[3][j] >> step) + (int32_t)(in2 * d2_scale) + (int32_t)fb3) & 1023;
        float in3 = (Waveform[3] == 0) ? sinTable[ph3] : (Waveform[3] == 1 ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < PWcount[3][j] ? 32767.0f : -32768.0f));

        // Filter 3 + Soft clip
        float hp3 = in3 - v_lp[3][j] - (filter_q[3] * v_bp[3][j]);
        v_bp[3][j] += filter_f[3][j] * hp3;
        v_lp[3][j] += filter_f[3][j] * v_bp[3][j];
        if (v_lp[3][j] > 20000.0f) v_lp[3][j] = 20000.0f + (v_lp[3][j] - 20000.0f) * 0.2f;
        else if (v_lp[3][j] < -20000.0f) v_lp[3][j] = -20000.0f + (v_lp[3][j] + 20000.0f) * 0.2f;
        lastOut[3][j] = v_lp[3][j];
        int32_t out3 = ((int32_t)v_lp[3][j] * generatorvolume[3][j]) >> 6;

        totalUpper += (out1 + (out3 >> 2));
        totalLower += (out3 + (out1 >> 2));
      }
      // Itt töltsd az output buffert a totalUpper/Lower értékekkel...


      // --- Kimeneti lánc (Effektek, EQ, Pan) ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft(); parametereqright();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      // lowpassfilterleft(); lowpassfilterright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------------7-6-------Fm-Y-Struktúra---------------------------------
  /*
    ============================================================================
                     ALGORITHM 76: Y-STRUCTURE (3-OP CHAIN + SUB)
    ============================================================================

          +----------+
       +->|  OSC 0   | (OP 4 - Modulator 1)
       |  | (Mod 1)  |
       |  +----+-----+
      (Self-  |
       FB)    | (in0 * pV0)
              v
          +----------+
       +->|  OSC 1   | (OP 2 - Modulator 2)
       |  | (Mod 2)  |
       |  +----+-----+
      (Self-  |
       FB)    | (in1 * pV1)
              v
          +----------+                      +----------+
       +->|  OSC 2   |                   +->|  OSC 3   | (OP 3 - Foundation)
       |  |(MainCar) |                   |  |(SubCar)  |
       |  +----+-----+                   |  +----+-----+
      (Self-  |                         (Self-  |
       FB)    | (out2)                   FB)    | (out3)
              |                                 |
              +--------------> [ AUDIO OUT ] <---+

    ============================================================================
    Note:
    - OSC 0 -> OSC 1 -> OSC 2 forms a 3-operator serial FM chain.
    - OSC 3 runs independently as a solid sub/foundation carrier.
    - All 4 operators have independent self-feedback loops.
    ============================================================================
  */



  const float mDepth = 0.0005f;
  const float mLink = 0.0001f;

  if (STRUCTURE == 76) { // Vagy 78, amit a gombnál beállítottál
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      // --- POINTEREK INICIALIZÁLÁSA ---
      uint32_t* pF0 = &freqmutato[0][0]; uint32_t* pP0 = &pichcount[0][0];
      float* pL0 = &v_lp[0][0]; float* pB0 = &v_bp[0][0];
      uint32_t* pW0 = &PWcount[0][0]; uint16_t* pV0 = &generatorvolume[0][0];
      float* pFF0 = &filter_f[0][0];

      uint32_t* pF1 = &freqmutato[1][0]; uint32_t* pP1 = &pichcount[1][0];
      float* pL1 = &v_lp[1][0]; float* pB1 = &v_bp[1][0];
      uint32_t* pW1 = &PWcount[1][0]; uint16_t* pV1 = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      uint32_t* pF2 = &freqmutato[2][0]; uint32_t* pP2 = &pichcount[2][0];
      float* pL2 = &v_lp[2][0]; float* pB2 = &v_bp[2][0];
      uint32_t* pW2 = &PWcount[2][0]; uint16_t* pV2 = &generatorvolume[2][0];
      float* pFF2 = &filter_f[2][0];

      uint32_t* pF3 = &freqmutato[3][0]; uint32_t* pP3 = &pichcount[3][0];
      float* pL3 = &v_lp[3][0]; float* pB3 = &v_bp[3][0];
      uint32_t* pW3 = &PWcount[3][0]; uint16_t* pV3 = &generatorvolume[3][0];
      float* pFF3 = &filter_f[3][0];


      for (int j = 0; j < polyphony; j++) {
        // --- OSC 0 (Mod 1) ---
        *pF0 += *pP0;

        // A szorzót levesszük 0.00001f-re (tizedére), hogy a poti 100-ig zenei maradjon
        float selfFeedback0 = lastOut[0][j] * (*pW0 * 0.00001f);

        // Ha a poti 2 alatt van, kényszerítsük tiszta szinuszra
        if (*pW0 < 2) selfFeedback0 = 0.0f;

        uint32_t ph0 = ((*pF0 >> step) + (int32_t)selfFeedback0) & 1023;

        // Hullámforma választó OSC 0
        float s0 = (Waveform[0] == 0) ? sinTable[ph0] : (float)((int32_t)(ph0 << 6) - 32768);
        lastOut[0][j] = s0;
        int32_t out0 = ((int32_t)(s0) * *pV0) >> 9;



        // --- OSC 1 (Modulator 2 + Self Feedback) ---
        *pF1 += *pP1;
        uint32_t ph1_base = (*pF1 >> step) & 1023;

        // 1. Moduláció a 0-ástól (vV0 szorzó)
        float mod1 = lastOut[0][j] * (*pV0 * 0.0001f);

        // 2. ÚJ: Self-Feedback az 1-esnek (pW1 poti)
        float feedback1 = lastOut[1][j] * (*pW1 * 0.00001f);
        if (*pW1 < 2) feedback1 = 0.0f;

        // 3. Összegezzük a kettőt
        uint32_t ph1 = (ph1_base + (int32_t)mod1 + (int32_t)feedback1) & 1023;

        // Hullámforma választó OSC 1
        float s1 = (Waveform[1] == 0) ? sinTable[ph1] : (float)((int32_t)(ph1 << 6) - 32768);
        lastOut[1][j] = s1;
        int32_t out1 = ((int32_t)(s1) * *pV1) >> 8;


        // --- OSC 2 (Main Carrier + Self Feedback) ---
        *pF2 += *pP2;

        // 1. Ugyanaz a finom feedback skálázás, mint a 3-asnál
        float feedback2 = lastOut[2][j] * (*pW2 * 0.00001f);
        if (*pW2 < 2) feedback2 = 0.0f;

        // 2. Az FM moduláció az 1-estől (marad a régi, de hozzáadjuk a feedbacket)
        float mod2 = lastOut[1][j] * (*pV1 * 0.0002f);

        // 3. Összegezzük a kettőt a fázisban
        uint32_t ph2 = ((*pF2 >> step) + (int32_t)mod2 + (int32_t)feedback2) & 1023;
        // Hullámforma választó OSC 2
        float in2 = (Waveform[2] == 0) ? sinTable[ph2] : (float)((int32_t)(ph2 << 6) - 32768);

        // MIX és SZŰRŐ OSC 2
        float mixedIn2 = (in2 * 0.5f) + (out1 * 0.3f) + (out0 * 0.2f);
        float hp2 = mixedIn2 - *pL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * (hp2 > 32767.0f ? 32767.0f : (hp2 < -32768.0f ? -32768.0f : hp2));
        *pL2 += *pFF2 * *pB2;
        lastOut[2][j] = *pL2;
        int32_t out2 = ((int32_t)(*pL2) * *pV2) >> 6;

        // --- OSC 3 (The Foundation) ---
        *pF3 += *pP3;

        // A szorzót leosztjuk 10-zel (0.0001f -> 0.00001f),
        // így a 100-as poti állásnál lesz annyi a feedback, mint régen a 10-esnél.
        float feedbackAmount = *pW3 * 0.00001f;

        // Biztonsági nulla: ha a poti 2 alatt van, legyen tökéletesen tiszta szinusz
        if (*pW3 < 2) feedbackAmount = 0.0f;

        uint32_t ph3 = ((*pF3 >> step) + (int32_t)(lastOut[3][j] * feedbackAmount)) & 1023;

        // Hullámforma választó OSC 3 (Itt volt a hiány!)
        float in3 = (Waveform[3] == 0) ? sinTable[ph3] : (float)((int32_t)(ph3 << 6) - 32768);

        // SZŰRŐ OSC 3
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * (hp3 > 32767.0f ? 32767.0f : (hp3 < -32768.0f ? -32768.0f : hp3));
        *pL3 += *pFF3 * *pB3;
        lastOut[3][j] = *pL3;
        int32_t out3 = ((int32_t)(*pL3) * *pV3) >> 6;

        // Összegzés

        totalUpper += (out2 + (out3 >> 2));
        totalLower += (out3 + (out2 >> 2));

        // Pointer léptetések
        pF0++; pP0++; pL0++; pB0++; pW0++; pV0++; pFF0++;
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;
        pF2++; pP2++; pL2++; pB2++; pW2++; pV2++; pFF2++;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;
      }

      // --- KIMENET ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      //processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //-----------------------7-7-Two Fm Osci (Optimized Filtered Feedback)----------------------------------
  /*
     ============================================================================
                             4-OPERATOR FM ALGORITHM
     ============================================================================

           +--------------------------------------------------------+
           |                                                        |
           |       +------------------------------------+           |
           |       | (Cross-Mod: OSC 1 -> OSC 0)        |           |
           v       v                                    |           |
        +-------------+  (pW0)                          |           |
     +->|    OSC 0    |<-------+                        |           |
     |  | (Modulator) |        |                        |           |
     |  +------+------+        |                        |           |
     | (Self-  |               |                        |           |
     |  FB)    | (in0 * pV0)   |                        |           |
     +---------+               |                        |           |
               v               |                        |           |
        +-------------+        |                        |           |
        |    OSC 1    |--------+------------------------+           |
        |  (Carrier)  |                                             |
        +------+------+                                             |
               |                                                    |
               +-----------------------> [ AUDIO OUT ] <------+     |
                                              ^               |     |
                                              |               |     |
        +-------------+                        | (out3)        |     |
     +->|    OSC 2    |                        |               |     |
     |  | (Modulator) |                  +-----+-------+       |     |
     |  +------+------+                  |    OSC 3    |-------+-----+
     | (Self-  |                         |  (Carrier)  |  (Link: OSC 3 -> OSC 1)
     |  FB)    | (in2 * pV2)             +-------------+
     +---------+                               ^
               |                               |
               +-------------------------------+

     ============================================================================
  */

  if (STRUCTURE_L == 7 && STRUCTURE_U == 7) {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      // --- POINTEREK INICIALIZÁLÁSA --- (Változatlan)
      uint32_t* pF0 = &freqmutato[0][0]; uint32_t* pP0 = &pichcount[0][0];
      float* pL0 = &v_lp[0][0]; float* pB0 = &v_bp[0][0];
      uint32_t* pW0 = &PWcount[0][0]; uint16_t* pV0 = &generatorvolume[0][0];
      float* pFF0 = &filter_f[0][0];

      uint32_t* pF1 = &freqmutato[1][0]; uint32_t* pP1 = &pichcount[1][0];
      float* pL1 = &v_lp[1][0]; float* pB1 = &v_bp[1][0];
      uint32_t* pW1 = &PWcount[1][0]; uint16_t* pV1 = &generatorvolume[1][0];
      float* pFF1 = &filter_f[1][0];

      // ... (OSC 2 & 3 pointerek ugyanúgy) ...
      uint32_t* pF2 = &freqmutato[2][0]; uint32_t* pP2 = &pichcount[2][0];
      float* pL2 = &v_lp[2][0]; float* pB2 = &v_bp[2][0];
      uint32_t* pW2 = &PWcount[2][0]; uint16_t* pV2 = &generatorvolume[2][0];
      float* pFF2 = &filter_f[2][0];

      uint32_t* pF3 = &freqmutato[3][0]; uint32_t* pP3 = &pichcount[3][0];
      float* pL3 = &v_lp[3][0]; float* pB3 = &v_bp[3][0];
      uint32_t* pW3 = &PWcount[3][0]; uint16_t* pV3 = &generatorvolume[3][0];
      float* pFF3 = &filter_f[3][0];

      for (int j = 0; j < polyphony; j++) {
        // --- OSC 0 (Modulátor 1 + Feedback + Cross-mod az OSC 1-től) ---
        *pF0 += *pP0;
        float feedbackAmount0 = (float)(*pW0) * 0.00005f;
        float crossMod0 = (float)(*pW1) * 0.00005f; // pW1 vezérli a visszacsatolást
        float fb0 = (lastOut[0][j] * feedbackAmount0) + (lastOut[1][j] * crossMod0);

        uint32_t ph0 = ((*pF0 >> step) + (int32_t)fb0) & 1023;
        float valL0 = *pL0;
        float in0;
        if (Waveform[0] == 0) in0 = sinTable[ph0];
        else if (Waveform[0] == 1) in0 = (float)((int32_t)(ph0 << 6) - 32768);
        else in0 = (ph0 < *pW0 ? 32767.0f : -32768.0f);

        float hp0 = in0 - valL0 - (filter_q[0] * *pB0);
        *pB0 += *pFF0 * hp0;
        valL0 += *pFF0 * *pB0;
        if (valL0 > 32767.0f) valL0 = 32767.0f; else if (valL0 < -32768.0f) valL0 = -32768.0f;
        lastOut[0][j] = valL0;
        *pL0 = valL0;
        float depth0 = (float)(*pV0) * 0.00005f;

        // --- OSC 1 (Carrier 1 + FM az OSC 0-tól + Link az OSC 3-tól) ---
        *pF1 += *pP1;
        float linkMod = (float)(*pW3) * 0.00005f; // pW3 összeköti a két párt
        float totalMod1 = (in0 * depth0) + (lastOut[3][j] * linkMod);

        uint32_t ph1 = ((*pF1 >> step) + (int32_t)totalMod1) & 1023;
        float valL1 = *pL1;
        float in1;
        if (Waveform[1] == 0) in1 = sinTable[ph1];
        else if (Waveform[1] == 1) in1 = (float)((int32_t)(ph1 << 6) - 32768);
        else in1 = (ph1 < *pW1 ? 32767.0f : -32768.0f);

        float hp1 = in1 - valL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        valL1 += *pFF1 * *pB1;
        if (valL1 > 32767.0f) valL1 = 32767.0f; else if (valL1 < -32768.0f) valL1 = -32768.0f;
        lastOut[1][j] = valL1;
        *pL1 = valL1;
        int32_t out1 = ((int32_t)valL1 * *pV1) >> 6;

        // --- OSC 2 (Modulátor 2 + Feedback) ---
        *pF2 += *pP2;
        float feedbackAmount2 = (float)(*pW2) * 0.00005f;
        float fb2 = lastOut[2][j] * feedbackAmount2;

        uint32_t ph2 = ((*pF2 >> step) + (int32_t)fb2) & 1023;
        float valL2 = *pL2;
        float in2;
        if (Waveform[2] == 0) in2 = sinTable[ph2];
        else if (Waveform[2] == 1) in2 = (float)((int32_t)(ph2 << 6) - 32768);
        else in2 = (ph2 < *pW2 ? 32767.0f : -32768.0f);

        float hp2 = in2 - valL2 - (filter_q[2] * *pB2);
        *pB2 += *pFF2 * hp2;
        valL2 += *pFF2 * *pB2;
        if (valL2 > 32767.0f) valL2 = 32767.0f; else if (valL2 < -32768.0f) valL2 = -32768.0f;
        lastOut[2][j] = valL2; // Itt a hiányolt sor!
        *pL2 = valL2;
        float depth2 = (float)(*pV2) * 0.00005f;

        // --- OSC 3 (Carrier 2 + FM az OSC 2-től) ---
        *pF3 += *pP3;
        uint32_t ph3 = ((*pF3 >> step) + (int32_t)(in2 * depth2)) & 1023;
        float valL3 = *pL3;
        float in3;
        if (Waveform[3] == 0) in3 = sinTable[ph3];
        else if (Waveform[3] == 1) in3 = (float)((int32_t)(ph3 << 6) - 32768);
        else in3 = (ph3 < *pW3 ? 32767.0f : -32768.0f);

        float hp3 = in3 - valL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        valL3 += *pFF3 * *pB3;
        if (valL3 > 32767.0f) valL3 = 32767.0f; else if (valL3 < -32768.0f) valL3 = -32768.0f;
        lastOut[3][j] = valL3; // Ezt is mentsük el a linkeléshez
        *pL3 = valL3;
        int32_t out3 = ((int32_t)valL3 * *pV3) >> 6;

        // --- Pointer léptetések ---
        pF0++; pP0++; pL0++; pB0++; pW0++; pV0++; pFF0++;
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;
        pF2++; pP2++; pL2++; pB2++; pW2++; pV2++; pFF2++;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        // Összegzés
        totalUpper += (out1 + (out3 >> 2));
        totalLower += (out3 + (out1 >> 2));
      }
      // --- EFFEKTEK ÉS KIMENET (Változatlan) ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      //reverbleft(); reverbright();
      processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }
  //BUFFER WRITE DAC
  i2s_write(I2S_PORT, &sBuffer, bufferLen, &i2s_bytes_write, portMAX_DELAY);
}
