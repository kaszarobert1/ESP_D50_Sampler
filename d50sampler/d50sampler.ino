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
byte midichan = 16;
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
#define bufferLen 512
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
bool LCD_ON = false;
//int step = 16;
//uint16_t GLOBAL_TUNE = 472;

int step = 22;
uint16_t GLOBAL_TUNE = 7552;
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
int16_t delaybuffer[8192];
int16_t delaybuffer2[8192];
uint16_t delaybufferindex = 0;
uint16_t delaybufferindex2 = 0;
byte delaystep = 0;
byte delay2step = 0;
byte delaytime = 1;
byte delay2time = 1;
byte reverblevel = 20;
byte reverbdiffusion = 22;
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
byte ENV_T1[4] = { 100, 100, 100, 100 };
byte ENV_L1[4] = { 100, 100, 100, 100 };
byte ENV_T2[4] = { 1, 1, 1, 1 };
byte ENV_L2[4] = { 80, 80, 80, 80 };
byte ENV_L3[4] = { 80, 80, 80, 80 }; //not used more
byte ENV_T3[4] = { 1, 1, 1, 1 };
byte ENV_LSUS[4] = { 50, 50, 50, 50 };
byte ENV_T4[4] = { 1, 1, 1, 1 };
byte ENV_T5[4] = { 1, 1, 1, 1 };
byte ENV_LEND[4] = { 0, 0, 0, 0 };
byte TVA_Slide = 19;
#define L_SCALE 2500 // 2.5 * 1000 a fixpontos matekhoz
byte generatorstatus[4][polyphony];
uint32_t TVAvolume[4][polyphony];
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

  // Előbb szorzunk a mintával, és csak a végén osztunk le a normalizáló a0-val
  // Ez sokkal pontosabb marad és kevésbé hajlamos a gerjedésre
  paraeqrightbuffer = ( (b02 * PrevSample2[0]) + (b12 * PrevSample2[1]) + (b22 * PrevSample2[2]) 
                        - (a12 * lastbuffer2[0]) - (a22 * lastbuffer2[1]) ) / a02;

  lastbuffer2[2] = lastbuffer2[1];
  lastbuffer2[1] = lastbuffer2[0];
  lastbuffer2[0] = paraeqrightbuffer;
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
  float szorzo2 = 1;
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
      // Serial.print(String(BASIC_TUNE[i]) + " ");
    }
    //Serial.println();

    float okt = 1;
    for (int i = 0; i < 14; i++) {
      for (int k = 0; k < 12; k++) {
        noteertek [j][i * 12 + k] = round(BASIC_TUNE[k] * okt);
      }
      okt = okt * szorzo2;
    }
  }
  //notevaluesarraytest();
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
    //"drawbarsloop","highorganloop","loworganloop","electpiano1loop","electpiano2loop","claviloop","hapsichordloop","electbassloop1","acusticbassloop","electbassloop2","electbassloop3","electgitarloop","chelloloop","violinloop","reedloop","saxloop1","saxloop2","aahloop","oohloop","maleloop","spectrum1loop",""
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
      //    case 74: genstartadress[opmenuoldal] = maleloop2; break;
  }
  //Serial.println("PCMWave" + String(opmenuoldal) + "generator: " + String(PCMWaveNo[opmenuoldal]));
  setsamplesize();
}

void updateLFOAdresses() {
  for (int i = 0; i < LFOnumber; i++) {
    switch (LFO_Wave_Select[i]) {
      case 0: LFOadress[i] = lfotriangle;   break;
      case 1: LFOadress[i] = lfosaw;  break;
      case 2: LFOadress[i] = lfosquare;       break;
      case 3: LFOadress[i] = lforandom;   break;
      case 4: LFOadress[i] = lfosine;   break;
      default: LFOadress[i] = lfosine;  break;
    }
  }
}

//--------------LCD-------------------------------
void lcdprint(String szoveg) {
  if (LCD_ON)
  {
    lcd.setCursor(0, 1);
    int hiany = 16 - szoveg.length();
    for (int i = 0; i < hiany; i++)
    {
      szoveg += " ";
    }
    lcd.print(szoveg);
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


  debugCounter++;
  Serial.print("Param betöltés: "); Serial.println(debugCounter);
  if (localParameterByte == 0)
    switch (noteByte) {
      case 0:
        //couarse u1
        COARSE[2] = value;
        line = "COARSE U1: " + lcdprint3(COARSE[2]);
        notetune();
        break;
      case 1:
        //couarse u1
        FINE[2] = value;
        line = "U1: COARSE= " + lcdprint3(FINE[2]);
        notetune();
        break;
      case 2:
        KEYFollow[2] = value;
        line = " U1: KEYFollow=" + lcdprint2(KEYFollow[2]);
        notetune();
        break;
      case 3:

        break;
      case 4:
        TVA[2] = value;
        line = "U1: TVA=" + lcdprint3(TVA[2]);
        break;

      case 6:
        Waveform[2] = value;
        if (value == 0) {
          line = "U1: Waveform=Square";
        }
        if (value == 1) {
          line = "U1: Waveform=Sawtooth";
        }
        break;
      case 7:
        PCMWaveNo[2] = value;
        line = "U1: PCMWaveNo=" + lcdprint3(PCMWaveNo[2]);

        //
        opmenuoldal = 2;
        setPCMWave();
        break;
      case 8:
        PW[2] = value;
        line = "U1: PW=" + lcdprint3(PW[2]);
        break;
      case 10:
        PWMLFO[2] = value;
        line = "U1: PWMLFO=" + lcdprint3(PWMLFO[2]);
        break;
      case 11:
        PWMLFODepth[2] = value;
        line = "U1: PLFODPT=" + lcdprint3(PWMLFODepth[2]);
        break;
      case 13:
        tvf_cutoff[2] = value;
        line = "U1: TVF_CUTOFF=" + lcdprint3(tvf_cutoff[2]);
        break;
      case 14:
        tvf_reso[2] = value;
        line = "U1: TVF_RESO=" + lcdprint3(tvf_reso[2]);
        break;
      case 32:
        TWFLFO[2] = value;
        line = "U1: TWFLFO=" + lcdprint3(TWFLFO[2]);
        break;
      case 33:
        TVF_LFO_level[2] = value;
        line = "U1 TVFLFOL=" + lcdprint3(TVF_LFO_level[2]);
        break;

      case 35:
        volume[2] = value;
        line = "U1: Level=" + lcdprint3(volume[2]);
        break;
      case 37:
        BiasPoint[2] = value;
        line = "U1: BiasPoint=" + lcdprint3(BiasPoint[2]);
        notebias();
        break;
      case 38:
        BiasLevel[2] = value;
        line = "U1: bieasLevel=" + lcdprint3(BiasLevel[2]);
        notebias();
        break;
      case 39:
        ENV_T1[2] = 100 - value;
        line = "U1: ENV_T1=" + lcdprint3(ENV_T1[2]);
        break;
      case 40:
        ENV_T2[2] = 100 - value;
        line = "U1: ENV_T2=" + lcdprint3(ENV_T2[2]);
        break;
      case 41:
        ENV_T3[2] = 100 - value;
        line = "U1: ENV_T3=" + lcdprint3(ENV_T3[2]);
        break;
      case 42:
        ENV_T4[2] = 100 - value;
        line = "U1: ENV_T4=" + lcdprint3(ENV_T4[2]);
        break;
      case 43:
        /*
          step = samplesize[2] / 100;
          sampleend[2] = value * step;
          Serial.println("SAMPLE END U1: " + String(sampleend[2]));
        */
        ENV_T5[2] = 100 - value;
        line = "U1: ENV_T5=" + lcdprint3(ENV_T5[2]);
        break;
      case 44:
        ENV_L1[2] = value;
        line = "U1: ENV_L1" + lcdprint3(ENV_L1[2]);
        break;
      case 45:
        ENV_L2[2] = value;
        line = "U1: ENV_L2=" + String(ENV_L2[2]);
        break;
      case 46:
        /*
          step = samplesize[2] / 100;
          samplebegin[2] = value * step;
          Serial.println("SAMPLE BEGIN U1: " + String(samplebegin[2]));
        */
        ENV_L3[2] = value;
        line = "U1: ENV_L3=" + lcdprint3(ENV_L3[2]);
        break;
      case 47:
        ENV_LSUS[2] = value;
        line = "U1: ENV_LSUS=" + lcdprint3(ENV_LSUS[2]);
        break;
      case 48:
        ENV_LEND[2] = value;
        line = "U1: ENV_LEND=" + lcdprint3(ENV_LEND[2]);
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
        line = "U1: PICH_LFO=" + lcdprint3(PICHLFO[2]);
        break;
      case 52:
        PICH_LFO_level[2] = value;
        line = "U1: PICH_LFOL=" + lcdprint3(PICH_LFO_level[2]);
        break;


      case 64:
        //couarse u2
        COARSE[3] = value;
        line = "U2: COARSE=" + lcdprint3(COARSE[3]);
        notetune();
        break;
      case 65:
        //couarse u2
        FINE[3] = value;
        line = "U2: COARSE=" + lcdprint3(FINE[3]);
        notetune();
        break;
      case 66:
        KEYFollow[3] = value;
        line = "U2: KEYFollow=" + lcdprint2(KEYFollow[3]);
        notetune();
        break;
      case 67:

        break;
      case 68:
        TVA[3] = value;
        line = "U2: TVA=" + lcdprint3(TVA[3]);
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
        line = "L1: PENVMode=" + lcdprint3(PENVMode[0]);
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
        LFO_Delay[3] = value;
        line = " LFO3_DELAY: " + lcdprint3(LFO_Delay[3]);
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
      case 43:
        lfofreq[7] = value;
        line = "U: CHORUSFREQ=" + lcdprint3(lfofreq[7] );
        break;
      case 44:
        chorusLevelRight = value;
        line = "U: CHORUSLEVEL=" + lcdprint3(chorusLevelRight );
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
        LFO_Delay[0] = value;
        line = " LFO0_DELAY: " + lcdprint3(LFO_Delay[0]);
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


      case 106:
        switch (value) {
          case 1:
            //chorus1
            chorusbuffersize = 2048;
            LFOadress[0] = lfotriangle;
            LFOadress[1] = lfotriangle;
            break;
          case 2:
            //chorus2
            chorusbuffersize = 2048;
            LFOadress[0] = lfotriangle;
            LFOadress[1] = lfosine;
            break;
          case 3:
            //chorus3
            chorusbuffersize = 1024;
            LFOadress[0] = lfotriangle;
            LFOadress[1] = lfotriangle;
            break;
          case 4:
            //chorus4
            chorusbuffersize = 1023;
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
          case 7:
            //chorus6
            chorusbuffersize = 255;
            LFOadress[0] = lfotriangle;
            LFOadress[1] = lfotriangle;
            break;
          case 8:
            //chorus6
            chorusbuffersize = 127;
            LFOadress[0] = lfosine;
            LFOadress[1] = lfosine;
            break;
          default:
            return;
            break;

        }
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
        Serial.println(line);
        return;
        break;
    }
  }
  if (localParameterByte == 3) {
    switch (noteByte) {
      case 22:
        UKeyShift = value;
        line = "U: KeyShift=" + lcdprint3(UKeyShift);
        break;
      case 23:
        LKeyShift = value;
        line = "L: KeyShift=" + lcdprint3(LKeyShift);
        break;
      case 25:
        GLOBAL_TUNE = 422 + value;
        notetune();
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
          case 16: GLOBAL_TUNE = 472; break;
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
  Serial.println(line);
  //lcd
  //lcdprint(line);
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

int32_t atlag = 0;
int16_t x = 0;
int32_t atlag2 = 0;
int16_t x2 = 0;

void processingStereoReverb() {
  // 1. Kiolvasás a bufferekből
  int16_t delayedL = delaybuffer[delaybufferindex];
  int16_t delayedR = delaybuffer2[delaybufferindex2];

  // 2. Bemenet + Kereszt-visszacsatolás
  // Eltároljuk az eredeti bemenetet, hogy ne adjuk hozzá kétszer
  int16_t inputL = bufferbe[0];
  int16_t inputR = bufferbe[1];

  bufferbe[0] = inputL + delayedL + (delayedR >> 2);
  bufferbe[1] = inputR + delayedR + (delayedL >> 2);

  // --- BAL OLDAL SZÁMÍTÁSA ---
  atlag += (bufferbe[0] * reverblevel) >> 6;
  delaystep++;

  if (delaystep >= delaytime) {
    int16_t resL = atlag / delaystep; // Kiszámoljuk az új mintát

    // Fényesebb Lowpass: (3*új + 1*régi) / 4
    x = ((resL << 1) + resL + x) >> 2;

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
    int16_t resR = atlag2 / delay2step; // Kiszámoljuk az új mintát

    // Fényesebb Lowpass a jobb oldalon is
    x2 = ((resR << 1) + resR + x2) >> 2;

    delaybuffer2[delaybufferindex2] = x2;
    atlag2 = 0;
    delaybufferindex2++;
    delaybufferindex2 &= (reverbtime2 - 1);
    delay2step = 0;
  }
}

//bufferbe: actual sample, delaybuffer array, reverblecel, reverbdiffusion, delaytime,



void reverbleft() {
  int16_t delayedSample = delaybuffer[delaybufferindex];
  bufferbe[0] += delayedSample;
  atlag += (bufferbe[0] * reverblevel) >> 6;
  delaystep++;
  if (delaystep >= delaytime) {
    int16_t newSample = atlag / delaystep;
    x = (newSample + x) >> 1;
    delaybuffer[delaybufferindex] = x;
    atlag = 0;
    delaybufferindex++;
    delaystep = 0;
  }
  delaybufferindex &= (reverbtime - 1);
}

//------------------------------REVERB-DELAY EFFECT RIGHT-----------------------------------------
//bufferbe: actual sample right, delaybuffer2 array, reverblevel, reverbdiffusion, delaytime2, delaystep2



void reverbright() {
  int16_t delayedSample2 = delaybuffer2[delaybufferindex2];
  bufferbe[1] = bufferbe[1] + delayedSample2;
  atlag2 += (bufferbe[1] * reverblevel) >> 6;
  delay2step++;
  if (delay2step >= delay2time) {
    int16_t newSample2 = atlag2 / delay2step;
    x2 = (newSample2 + x2) >> 1;
    delaybuffer2[delaybufferindex2] = x2;
    atlag2 = 0;
    delaybufferindex2++;
    delay2step = 0;
  }
  delaybufferindex2 &= (reverbtime2 - 1);
}

//--------------------------CHORUS LEFT------------------------------

int16_t chorusbufferleft[512];
uint16_t chorusbufferindex = 0;
uint16_t chorusindex;
int16_t atlagchorus0 = 0;
int32_t lfoSmoothedLeft = 0;

void chorusleft() {
  // 1. Interpolált LFO kiolvasás (LFO 6-os csatorna)
  uint32_t indexLarge = lfoarrayindex[6];
  uint16_t i1 = (indexLarge >> 23) & 511;
  uint16_t i2 = (i1 + 1) & 511;
  uint16_t lfoFraction = (indexLarge >> 15) & 0xFF;

  int32_t v1 = (int32_t)(*(LFOadress[6] + i1));
  int32_t v2 = (int32_t)(*(LFOadress[6] + i2));

  // 16-bites nyers LFO érték kiszámítása
  uint32_t rawLFO = v1 + (((v2 - v1) * lfoFraction) >> 8);

  // SEBESSÉG: Itt is a bevált << 14 eltolást használjuk
  lfoarrayindex[6] += (lfofreq[6] << 14);

  // 2. SKÁLÁZÁS ÉS ERŐSÍTÉS (A "majdnem teljesen erős" recept)
  // A rawLFO-ból (0-65535) csinálunk egy mélyebb, 0-511 közötti eltolást
  uint32_t depthLFO = rawLFO >> 7;

  int32_t currentLFO = (depthLFO * 3) >> 2;
  uint8_t fraction = (rawLFO & 0xFF);  // A törtrész marad 0-255 a ketyegésmentességhez

  // 3. BUFFER MŰVELETEK
  chorusbufferleft[chorusbufferindex] = bufferbe[0];
  uint16_t mask = 511;

  uint16_t idx1 = (chorusbufferindex - currentLFO) & mask;
  uint16_t idx2 = (idx1 + 1) & mask;

  int16_t s1 = chorusbufferleft[idx1];
  int16_t s2 = chorusbufferleft[idx2];

  // Tiszta interpoláció (>> 8-cal, mert a fraction 255-ig megy)
  int16_t interpolated = s1 + (((s2 - s1) * fraction) >> 8);

  // 4. MIX ÉS KIMENET
  // Meghagytam az átlagolást (LPF), de ha túl halk, ezen még lazíthatunk
  atlagchorus0 = (interpolated + atlagchorus0) >> 1;
  int32_t chorusPart = (atlagchorus0 * chorusLevelLeft) >> 8;

  int32_t out = (int32_t)bufferbe[0] + chorusPart;

  // Limiter
  if (out > 32767) out = 32767;
  else if (out < -32768) out = -32768;

  bufferbe[0] = (int16_t)out;
  chorusbufferindex = (chorusbufferindex + 1) & mask;
}

//--------------------------CHORUS RIGHT------------------------------
#define DEBUG_SIZE 512
int16_t debug_lfo[DEBUG_SIZE];
int16_t debug_fract[DEBUG_SIZE];
uint16_t debug_ptr = 0;
bool debug_ready = false;

//debug end


int chorusbufferright[512];
uint16_t chorusbufferindex2 = 0;
uint16_t chorusindex2;
int16_t atlagchorus1 = 0;
int32_t lfoSmoothedRight = 0;
void chorusright() {
  uint32_t indexLarge = lfoarrayindex[7];
  uint16_t i1 = (indexLarge >> 23) & 511;
  uint16_t i2 = (i1 + 1) & 511;
  uint16_t lfoFraction = (indexLarge >> 15) & 0xFF;

  int32_t v1 = (int32_t)(*(LFOadress[7] + i1));
  int32_t v2 = (int32_t)(*(LFOadress[7] + i2));
  uint32_t rawLFO = v1 + (((v2 - v1) * lfoFraction) >> 8);

  lfoarrayindex[7] += (lfofreq[7] << 14); // A tempó, ami bevált

  // --- ERŐSÍTÉS KETYEGÉS NÉLKÜL ---
  // A rawLFO-t (0-65535) nem shifteljük le fixen 8-ra,
  // hanem csinálunk egy köztes értéket, ami mélyebb.
  uint32_t depthLFO = rawLFO >> 7; // Ez 0-511 közötti eltolás (erős!)

  int32_t currentLFO = (depthLFO * 3) >> 2;  // Az egész rész (visszahozzuk a skálát)
  // A trükk: A fraction-t mindig a rawLFO legaljából vesszük,
  // így az mindig 0-255 marad, függetlenül a mélységtől!
  uint8_t fraction = (rawLFO & 0xFF);

  chorusbufferright[chorusbufferindex2] = bufferbe[1];
  uint16_t mask = 511;

  uint16_t idx1 = (chorusbufferindex2 - currentLFO) & mask;
  uint16_t idx2 = (idx1 + 1) & mask;

  int16_t s1 = chorusbufferright[idx1];
  int16_t s2 = chorusbufferright[idx2];

  // Itt marad a >> 8, mert a fraction 0-255. Így tiszta lesz a hang!
  int16_t interpolated = s1 + (((s2 - s1) * fraction) >> 8);

  int32_t chorusPart = (interpolated * chorusLevelRight) >> 8;

  int32_t out = (int32_t)bufferbe[1] + chorusPart;

  if (out > 32767) out = 32767;
  if (out < -32768) out = -32768;

  bufferbe[1] = (int16_t)out;
  chorusbufferindex2 = (chorusbufferindex2 + 1) & mask;
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



void lowpassfilterleft() {
  //delaybuffer[delaybufferindex] = (delaybuffer[delaybufferindex] + x) >> 1;
  //x = delaybuffer[delaybufferindex];
}

//LOWPASSFILTER RIGHT
//lowpassfilter in delaybuffer!!!
//delaybuffer actual sample, x2: delaybuffer prev sample

void lowpassfilterright() {
  //delaybuffer2[delaybufferindex2] = (delaybuffer2[delaybufferindex2] + x2) >> 1;
  //x2 = delaybuffer2[delaybufferindex2];
}

uint32_t lastTargetPitch[4] = {0, 0, 0, 0};
//-------------------------------MIDI INPUT COMMAND-------------------------------------
void keyon(byte noteByte) {
  wavefreq[0][generatornumber] = noteertek[0][noteByte  + LKeyShift];
  wavebias[0][generatornumber] = Bias[0][noteByte  + LKeyShift];
  wavefreq[1][generatornumber] = noteertek[1][noteByte  + LKeyShift];
  wavebias[1][generatornumber] = Bias[1][noteByte  + LKeyShift];
  wavefreq[2][generatornumber] = noteertek[2][noteByte  + UKeyShift];
  wavebias[2][generatornumber] = Bias[2][noteByte  + UKeyShift];
  wavefreq[3][generatornumber] = noteertek[3][noteByte  + UKeyShift];
  wavebias[3][generatornumber] = Bias[3][noteByte  + UKeyShift];
  oldnoteByte[generatornumber] = noteByte;
  CaseArray[chaseindex] = noteByte;
  pich[0][generatornumber] = wavefreq[0][generatornumber];
  pich[1][generatornumber] = wavefreq[1][generatornumber];
  pich[2][generatornumber] = wavefreq[2][generatornumber];
  pich[3][generatornumber] = wavefreq[3][generatornumber];
  for (int osc = 0; osc < 4; osc++) {
    // 1. Ha a portamento ki van kapcsolva, azonnal odaállunk
    if (portamento_time[osc] == 0) {
      currentPitch[osc][generatornumber] = pich[osc][generatornumber];
    }
    // 2. Ha be van kapcsolva, akkor az ELŐZŐLEG leütött hangról indítjuk a csúszást,
    // nem pedig arról, ami véletlenül abban a slotban maradt.
    else {
      currentPitch[osc][generatornumber] = lastTargetPitch[osc];
    }

    // Elmentjük a mostani célpontot, hogy a következő hang erről indulhasson
    lastTargetPitch[osc] = pich[osc][generatornumber];
  }
  // Serial.println(String(pich[generatornumber]));
  freqmutato[0][generatornumber] = samplebegin[0] << step;
  freqmutato[1][generatornumber] = samplebegin[1] << step;
  freqmutato[2][generatornumber] = samplebegin[2] << step;
  freqmutato[3][generatornumber] = samplebegin[3] << step;
  // 2. SZŰRŐ RESET (Ezt add hozzá!)


  // CSAK ENNYI KELL: Töröljük a szűrő memóriáját az új hang indításakor
  v_lp[0][generatornumber] = 0.0f; v_bp[0][generatornumber] = 0.0f;
  v_lp[1][generatornumber] = 0.0f; v_bp[1][generatornumber] = 0.0f;
  v_lp[2][generatornumber] = 0.0f; v_bp[2][generatornumber] = 0.0f;
  v_lp[3][generatornumber] = 0.0f; v_bp[3][generatornumber] = 0.0f;

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
  //sync
  for (int i = 0; i < 6; i++) {
    if (LFOSYNC[i] == 2) {
      LFO_Delay_Counter[i] = 0;
    }
  }

}

void keyoff(byte noteByte) {
  for (int i = 0; i < polyphony; i++) {
    if (noteByte == oldnoteByte[i]) {
      oldnoteByte[i] = 0;
      //  noteoff[i] = true;
      generatorstatus[0][i] = 4;
      generatorstatus[1][i] = 4;
      generatorstatus[2][i] = 4;
      generatorstatus[3][i] = 4;
    }
  }

}

//--------------CHASE---------------------------


void chasearpeggiomidiclock() {

  if (MIDI_SYNC == 1 && CHASE_TIME > 0) {

    // A masterTick eltolása az offsettel (0-23 tartományban tartva)
    int shiftedTick = (masterTick + OFFSET);

    // A shiftedTick alapján nézzük az osztást
    if (shiftedTick % CHASE_TIME == 0) {

      // A statikus változó most a shiftedTick-et figyeli,
      // hogy ne fusson le többször ugyanaz a lépés
      static int lastProcessedTick = -1;

      if (shiftedTick == lastProcessedTick) return;
      lastProcessedTick = shiftedTick;

      // --- AZ ARPEGGIO LÉPTETÉSE ---
      if (lastchase != 255) {
        keyoff(lastchase);
      }

      chaseindex++;
      if (chaseindex >= CHASE_LEVEL) chaseindex = 0;

      if (CaseArray[chaseindex] > 0) {
        lastchase = CaseArray[chaseindex];
        keyon(lastchase);
      } else {
        lastchase = 255;
      }
    }
  }
}


/*
  void chasearpeggio() {
  if (CHASE_TIME > 0) {
    ido = micros();
    if (elozoido > ido)
    {
      elozoido = 0;
    }
    //Serial.println("Lastchase: " + String(elozoido)+" "+String(ido));
    if (ido - elozoido > (CHASE_TIME << 12)) {
      keyoff( CaseArray[chaseindex]);
      chaseindex++;
      if (chaseindex >= CHASE_LEVEL) {
        chaseindex = 0;
      }
      if (CaseArray[chaseindex] != 0)
      {
        lastchase = CaseArray[chaseindex];
        keyon(lastchase);

      }
      elozoido = ido;
    }
  }
  }
*/

void chasearpeggio() {
  if (CHASE_TIME > 0) {
    ido = micros();
    if (elozoido > ido) elozoido = 0;

    if (ido - elozoido > (uint32_t)(CHASE_TIME << 13)) {

      // 1. ELŐBB kikapcsoljuk az előzőt, ami tényleg szólt
      if (lastchase != 0) {
        keyoff(lastchase);
        lastchase = 0; // Biztonság kedvéért nullázzuk
      }

      // 2. Léptetünk
      chaseindex++;
      if (chaseindex >= CHASE_LEVEL) chaseindex = 0;

      // 3. Megnézzük az újat
      if (CaseArray[chaseindex] != 0) {
        lastchase = CaseArray[chaseindex];
        keyon(lastchase);
      }

      elozoido = ido;
    }
  }
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
      LCD_ON = true;
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
  int globalIdx = 0;

  for (int s = 0; s < 7; s++) {
    Serial.print("--- Section: "); Serial.println(sections[s].name);

    localParameterByte = sections[s].localByte;

    for (int i = 0; i < sections[s].length; i++) {
      vTaskDelay(pdMS_TO_TICKS(4));
      noteByte = sections[s].startNote + i;
      velocityByte = storedPatch[globalIdx];

      parametersysexchanged();

      globalIdx++;
    }
  }
}

void handleProgramChange(byte channel, byte number) {
  // Opcionális: csak egy adott MIDI csatornára figyeljen
  // if (channel != 1) return;

  Serial.print("Program Change érkezett: "); Serial.println(number);

  switch (number) {
    case 0:
      LoadPatch(storedpach1);
      Serial.println("Patch 1 betöltve");
      break;
    case 1:
      LoadPatch(storedpach2);
      Serial.println("Patch 2 betöltve");
      break;
    case 2:
      LoadPatch(storedpach3);
      Serial.println("Patch 3 betöltve");
      break;
    case 3:
      LoadPatch(storedpach4);
      Serial.println("Patch 4 betöltve");
      break;
    case 4:
      LoadPatch(storedpach5);
      Serial.println("Patch 5 betöltve");
      break;
    case 5:
      LoadPatch(storedpach6);
      Serial.println("Patch 6 betöltve");
      break;
    case 6:
      LoadPatch(storedpach7);
      Serial.println("Patch 7 betöltve");
      break;
    case 8:
      LoadPatch(storedpach8);
      Serial.println("Patch 8 betöltve");
      break;
    case 9:
      LoadPatch(storedpach9);
      Serial.println("Patch 9 betöltve");
      break;
    case 10:
      LoadPatch(storedpach10);
      Serial.println("Patch 10 betöltve");
      break;
    case 11:
      LoadPatch(storedpach11);
      Serial.println("Patch 11 betöltve");
      break;
    case 12:
      LoadPatch(storedpach12);
      Serial.println("Patch 12 betöltve");
      break;
    case 13:
      LoadPatch(storedpach13);
      Serial.println("Patch 13 betöltve");
      break;
    case 14:
      LoadPatch(storedpach14);
      Serial.println("Patch 14 betöltve");
      break;
    case 15:
      LoadPatch(storedpach15);
      Serial.println("Patch 15 betöltve");
      break;
    default:
      Serial.println("Nincs ilyen tárolt patch!");
      break;
  }
}


//----------------------------------------setup--------------------------------
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
  // Set up MIDI

  // Eseménykezelők regisztrálása
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

}

/*
  uint32_t egyopgenA(uint32_t freqmutato1, long op1level, byte lep)
  {
  return generator1[freqmutato1 >> lep] * op1level;
  }
*/
uint32_t tvapointer[4][polyphony];



void loop() {
  MIDI2.read();
  // Serial.print("-");
  if (MIDI_SYNC == 2)
  {
    chasearpeggio();
  }
  //--MIDI input--
  //serialEvent();

  // LFO LÉPTETÉS: TISZTA SZÜNET -> FINOM FELÚSZÁS
  static uint8_t lfoPrescaler = 0;
  lfoPrescaler++;

  for (int i = 0; i < 6; i++) {
    // 1. A fázis mindig menjen a háttérben
    uint8_t rawVal = *(LFOadress[i] + (lfoarrayindex[i] >> 23));
    lfoarrayindex[i] += ((uint32_t)lfofreq[i] << 19);

    // 2. Késleltetés kezelése
    // Legyen a potméter fele a "totál csend", a másik fele a "beúszás"
    uint16_t silenceThreshold = LFO_Delay[i] >> 1;

    if (LFO_Delay_Counter[i] < silenceThreshold) {
      // ELSŐ SZAKASZ: Teljes csend
      if ((lfoPrescaler & 0x1F) == 0) LFO_Delay_Counter[i]++;
      lfovalue[i] = 128; // Középérték, nincs eltolás
    }
    else if (LFO_Delay_Counter[i] < LFO_Delay[i]) {
      // MÁSODIK SZAKASZ: Felúszás (Precíz matekkal)
      if ((lfoPrescaler & 0x1F) == 0) LFO_Delay_Counter[i]++;

      uint32_t fadeProgress = LFO_Delay_Counter[i] - silenceThreshold;
      uint32_t fadeDuration = LFO_Delay[i] - silenceThreshold;

      // Kiszámoljuk a különbséget a középértéktől (128)
      // rawVal: 0-255, 128 a közép.
      int32_t diff = (int32_t)rawVal - 128;

      // A szorzást elvégezzük 32 biten, majd osztunk
      int32_t scaledDiff = (diff * (int32_t)fadeProgress) / (int32_t)fadeDuration;

      // Visszatoljuk a 128-as középvonalra
      lfovalue[i] = (uint8_t)(128 + scaledDiff);
    }
    else {
      // HARMADIK SZAKASZ: Teljes vibrato
      lfovalue[i] = rawVal;
    }
  }




  // TVA ENVELOPE OPTIMIZED
  for (int i = 0; i < 4; i++) {
    if (TVA[i] > 0) {
      // Előre kiszámolt célpontok az adott oszcillátorhoz (i)
      const uint32_t attackTarget = (uint32_t)ENV_L1[i] << 17;
      const uint32_t targetL2     = (uint32_t)ENV_L2[i] << 17;
      const uint32_t targetL3     = (uint32_t)ENV_L3[i] << 17;
      const uint32_t targetLSUS   = (uint32_t)ENV_LSUS[i] << 17;
      const uint32_t targetLEND   = (uint32_t)(ENV_LEND[i] * 100) << 17;

      // Sebességek előre kiszámolva (nem kell a switch-ben újra és újra)
      const uint32_t sT1 = ((uint32_t)speedTable[ENV_T1[i]] << 12) + 100U;
      const uint32_t sT2 = ((uint32_t)speedTable[ENV_T2[i]] << 12) + 100U;
      const uint32_t sT3 = ((uint32_t)speedTable[ENV_T3[i]] << 12) + 100U;
      const uint32_t sT4 = ((uint32_t)speedTable[ENV_T4[i]] << 12) + 100U;
      const uint32_t sT5 = ((uint32_t)speedTable[ENV_T5[i]] << 12) + 100U;

      // Pointerek az oszcillátor 'i' sorának elejére
      uint8_t* pStatus = &generatorstatus[i][0];
      uint32_t* pTVAvol = &TVAvolume[i][0];
      uint16_t* pGenVol = &generatorvolume[i][0];

      const uint16_t baseVol = volume[i];
      const uint8_t  tvaMode = TVA[i];

      for (int j = 0; j < polyphony; j++) {
        // Aktuális hang állapotának és hangerejének betöltése pointerről
        uint8_t  status = *pStatus;
        uint32_t vol    = *pTVAvol;

        switch (status) {
          case 0: // ATTACK
            if (vol < attackTarget) vol += sT1;
            if (vol >= attackTarget) {
              vol = attackTarget;
              status = 1;
            }
            break;

          case 1: // DECAY 1
            if (vol > targetL2) {
              vol = (vol > targetL2 + sT2) ? vol - sT2 : targetL2;
              if (vol == targetL2) status = 2;
            } else {
              vol = (vol + sT2 < targetL2) ? vol + sT2 : targetL2;
              if (vol == targetL2) status = 2;
            }
            break;

          case 2: // DECAY 2
            if (vol > targetL3) {
              vol = (vol > targetL3 + sT3) ? vol - sT3 : targetL3;
              if (vol == targetL3) status = 3;
            } else {
              vol = (vol + sT3 < targetL3) ? vol + sT3 : targetL3;
              if (vol == targetL3) status = 3;
            }
            break;

          case 3: // SUSTAIN SLIDE
            if (vol > targetLSUS) vol = (vol > targetLSUS + sT4) ? vol - sT4 : targetLSUS;
            else if (vol < targetLSUS) vol = (vol + sT4 < targetLSUS) ? vol + sT4 : targetLSUS;
            break;

          case 4: // RELEASE
            if (vol > targetLEND + sT5) vol -= sT5;
            else if (vol < targetLEND) {
              vol += sT5;
              if (vol > targetLEND) vol = targetLEND;
            } else vol = targetLEND;

            if (vol == targetLEND) {
              status = (targetLEND == 0) ? 5 : 4;
            }
            break;

          case 5:
            vol = 0;
            break;
        }

        // Értékek visszaírása pointeren keresztül
        *pStatus = status;
        *pTVAvol = vol;

        // --- KIMENETI SZÁMÍTÁS POINTEREKKEL ---
        uint8_t currentLevel = (vol >> 16) & 0xFF;
        uint8_t lookupIdx = (tvaMode == 1) ? currentLevel : (255 - currentLevel);

        // Logaritmikus tábla elérése és hangerő számítás
        uint64_t tempVolume = (uint64_t)logTable16_S[lookupIdx] * (uint64_t)baseVol;
        *pGenVol = (uint16_t)(tempVolume >> 14);

        // Pointerek léptetése a következő hangra (j++)
        pStatus++; pTVAvol++; pGenVol++;
      }
    } else {
      // Ha nincs TVA, egyszerűsített feltöltés pointerrel
      uint16_t* pGenVol = &generatorvolume[i][0];
      uint8_t* pBias   = &wavebias[i][0];
      uint16_t  v       = volume[i];
      for (int j = 0; j < polyphony; j++) {
        *pGenVol = (v * (*pBias)) >> 2;
        pGenVol++; pBias++;
      }
    }
  }

  // LFO working area
  if (true) {
    for (int osc = 0; osc < 4; osc++) {
      // 1. MEGHATÁROZZUK AZ LFO CSOPORTOT (Lower: LFO 0-2 | Upper: LFO 3-5)
      int lfoOffset = (osc < 2) ? 0 : 3;

      // --- A) PWM LFO KIVÁLASZTÁSA ÉS IRÁNYA ---
      int selectedPWM = lfoOffset + (PWMLFO[osc] >> 1);
      int32_t currentPWMLFODepth = PWMLFODepth[osc] + (modulationWheel >> 1);
      int32_t lfoModPWM = (lfovalue[selectedPWM] * currentPWMLFODepth) >> 5;
      if (PWMLFO[osc] & 1) lfoModPWM = -lfoModPWM; // Ha páratlan a MIDI érték, invertálunk

      int32_t finalPW = ((PW[osc] + 1) << 5) + lfoModPWM;
      if (finalPW > 1023) finalPW = 1023;
      if (finalPW < 1)    finalPW = 1;

      // --- B) TVF (SZŰRŐ) LFO KIVÁLASZTÁSA ÉS IRÁNYA ---
      int selectedTVF = lfoOffset + (TWFLFO[osc] >> 1);
      float rawLfoTVF = (lfovalue[selectedTVF] - 128.0f);
      if (TWFLFO[osc] & 1) rawLfoTVF = -rawLfoTVF; // Fázisfordítás

      int32_t totalTVFLfoLevel = TVF_LFO_level[osc] + (modulationWheel >> 2);
      float lfo_part = (tvf_cutoff[osc] * 0.01f) + (rawLfoTVF * (totalTVFLfoLevel * 0.000039f));
      filter_q[osc] = fmaxf(0.05f, 1.0f - (tvf_reso[osc] * 0.03f));

      // --- C) PITCH LFO KIVÁLASZTÁSA ÉS IRÁNYA ---
      int selectedPICH = lfoOffset + (PICHLFO[osc] >> 1);
      int16_t rawLfoPich = (int16_t)lfovalue[selectedPICH] - 127;
      if (PICHLFO[osc] & 1) rawLfoPich = -rawLfoPich; // Fázisfordítás

      int32_t totalPichLfoDepth = PICH_LFO_level[osc];

      // --- D) POLYPHONIC CIKLUS (Glide + Összegzés) ---
      for (int j = 0; j < polyphony; j++) {

        // PORTAMENTO (A korábban kikísérletezett +1-es shift)
        if (currentPitch[osc][j] != pich[osc][j]) {
          if (portamento_time[osc] == 0) {
            currentPitch[osc][j] = pich[osc][j];
          } else {
            uint32_t distance;
            uint8_t shift = 1 + (portamento_time[osc] >> 3);

            if (currentPitch[osc][j] < pich[osc][j]) {
              distance = pich[osc][j] - currentPitch[osc][j];
              uint32_t step = distance >> shift;
              if (step < 1) step = 1;
              currentPitch[osc][j] += step;
              if (currentPitch[osc][j] > pich[osc][j]) currentPitch[osc][j] = pich[osc][j];
            } else {
              distance = currentPitch[osc][j] - pich[osc][j];
              uint32_t step = distance >> shift;
              if (step < 1) step = 1;
              currentPitch[osc][j] -= step;
              if (currentPitch[osc][j] < pich[osc][j]) currentPitch[osc][j] = pich[osc][j];
            }
          }
        }

        // SZŰRŐ SZÁMÍTÁS
        float total_norm = lfo_part + (TVFlevel[osc][j] * 0.5f);
        total_norm = fmaxf(0.0f, fminf(1.0f, total_norm));
        float cutoffHz = 20.0f + (total_norm * total_norm * 12000.0f);
        filter_f[osc][j] = fmaxf(0.005f, fminf(0.45f, 2.0f * sinf(cutoffHz * 0.0000712f)));

        PWcount[osc][j] = finalPW;

        int32_t lfoShift = (int32_t)((currentPitch[osc][j] >> 10) * rawLfoPich * totalPichLfoDepth) >> 7;

        pichcount[osc][j] = currentPitch[osc][j] + lfoShift;
      }
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
  // Serial.println(String(f0));



  // Serial.println("Generator" + String(generatorstatus[0][0]) + "statusz: " + String(TVAvolume[0][0]) + " " + String(TVAvolume[0][1]) + " " + String(TVAvolume[0][2]) + " " + String(TVAvolume[0][3]) + " " + String(TVAvolume[0][4]) + " " + String(TVAvolume[0][5]));


  // Serial.print("Generatorstátusz" + String(generatorstatus[0][0]) + " " + String(TVAvolume[0][0]) + "\n");




  //--2 SOUND LEFT, 2 SOUND RIGHT, 6  POLYFONI!!!!--

  //STRUCTURES

  //------------------0-------------------0---------------------PWM+PWM PWM+PWM------------------------------------
  if (STRUCTURE == 0) {
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------1-------------------0---------------------PWM*PWM PWM+PWM------------------------------------
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------0-------------------1---------------------PWM+PWM PWM*PWM------------------------------------
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //------------------1-------------------1---------------------PWM*PWM PWM*PWM------------------------------------
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //------------------0-------------------2---------------------LINEAR+LINEAR PCM+LINEAR------------------------------------
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
      parametereqleft(); parametereqright();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //------------------2-------------------0---------------------PCM+LINEAR LINEAR+LINEAR------------------------------------
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------2-------------------2---------------------PCM+LINEAR PCM+LINEAR------------------------------------
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }



  //------------------3-------------------2---------------------PWM*LINEAR---PWM+LINEAR------------------------------------
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //------------------2-------------------3---------------------PWM+LINEAR---PWM*LINEAR------------------------------------
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }





  //------------------3-------------------3---------------------PWM*LINEAR------------------------------------
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
      reverbleft(); reverbright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }






  //------------------5-------------------5---------------------
  if (STRUCTURE == 55) {
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





        // --- OSC 1 PCM ---

        uint32_t pos1 = *pF1;

        uint32_t idx1 = pos1 >> step;

        uint32_t frac1 = pos1 & ((1 << step) - 1);

        const int16_t* pSample1 = genstartadress[1];

        int16_t s1_1, s2_1;


        if (idx1 < sampleend[1] - 1) {

          // NORMÁL LEJÁTSZÁS: Benne vagyunk a mintában

          s1_1 = pSample1[idx1];

          s2_1 = pSample1[idx1 + 1];

          *pF1 += *pP1; // Csak akkor lépünk, ha nem értük el a végét

        }

        else if (idx1 >= sampleend[1] - 1) {

          // HATÁR ESET: Elértük az utolsó mintát vagy túlfutottunk

          if (loopsample[1]) {

            // LOOP MÓD: Visszarántjuk az elejére

            *pF1 = (uint32_t)samplebegin[1] << step;

            idx1 = samplebegin[1];

            s1_1 = pSample1[idx1];

            s2_1 = pSample1[idx1 + 1];

          } else {

            // ONE-SHOT MÓD: Megállítjuk a fázist és elnémítjuk a bemenetet

            s1_1 = 0;

            s2_1 = 0;

            // A *pF1-et NEM növeljük tovább.

          }

        }


        // 3. Interpoláció

        float in1 = (float)s1_1 + (float)(s2_1 - s1_1) * (float)frac1 * (1.0f / (float)(1 << step));


        // 4. SZŰRŐ

        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);

        *pB1 += *pFF1 * hp1;

        *pL1 += *pFF1 * *pB1;


        // Anti-pop / Limiter

        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;

        if (*pL1 < -32768.0f) *pL1 = -32768.0f;


        // 5. Kimenet és Pointer léptetés

        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 4;


        // 6. Mutatók léptetése (pW1 is léptetve!)

        pF1++; pP1++; pL1++; pB1++; pV1++; pFF1++;

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


        // --- OSC 3 PCM ---
        uint32_t pos3 = *pF3;
        uint32_t idx3 = pos3 >> step;
        uint32_t frac3 = pos3 & ((1 << step) - 1);
        const int16_t* pSample3 = genstartadress[3];
        int16_t s1_3, s2_3;

        if (idx3 < sampleend[3] - 1) {
          s1_3 = pSample3[idx3];
          s2_3 = pSample3[idx3 + 1];
          *pF3 += *pP3;
        }
        else if (idx3 >= sampleend[3] - 1) {
          if (loopsample[3]) {
            *pF3 = (uint32_t)samplebegin[3] << step;
            idx3 = samplebegin[3]; // <--- JAVÍTVA: idx3-at frissítjük!
            s1_3 = pSample3[idx3];
            s2_3 = pSample3[idx3 + 1];
          } else {
            s1_3 = 0;
            s2_3 = 0;
          }
        }

        // 3. Interpoláció
        float in3 = (float)s1_3 + (float)(s2_3 - s1_3) * (float)frac3 * (1.0f / (float)(1 << step));

        // 4. SZŰRŐ
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;

        // Anti-pop / Limiter
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;

        // 5. Kimenet
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 4;

        // 6. Mutatók léptetése (Szinkronban a polifóniával)
        pF3++; pP3++; pL3++; pB3++; pV3++; pFF3++; pW3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        totalUpper += (osc_out[0] + osc_out[1]);
        totalLower += (osc_out[2] + osc_out[3]);

      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper + (totalLower >> 2) >> masterVolume;
      bufferbe[1] = totalLower + (totalUpper >> 2) >> masterVolume;
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

  //--------------------6-------------------5---------------------

  if (STRUCTURE == 65 ) {
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

      //osc1 pointer PCM variable
      uint32_t* pF1 = &freqmutato[1][0]; // JAVÍTVA: 1-es hangszín/oszci sor
      uint32_t* pP1 = &pichcount[1][0];
      float* pL1    = &v_lp[1][0];
      float* pB1    = &v_bp[1][0];
      uint16_t* pV1 = &generatorvolume[1][0];
      float* pFF1   = &filter_f[1][0];

      //oc2 pointer PCM variable
      uint32_t* pF2  = &freqmutato[2][0];
      uint32_t* pP2  = &pichcount[2][0];
      float* pL2     = &v_lp[2][0];
      float* pB2     = &v_bp[2][0];
      uint16_t* pV2      = &generatorvolume[2][0];
      float* pFF2    = &filter_f[2][0];

      //osc3 pointer PCM variable
      uint32_t* pF3 = &freqmutato[3][0]; // JAVÍTVA: 3-as hangszín/oszci sor
      uint32_t* pP3 = &pichcount[3][0];
      float* pL3    = &v_lp[3][0];
      float* pB3    = &v_bp[3][0];
      uint16_t* pV3 = &generatorvolume[3][0];
      float* pFF3   = &filter_f[3][0];

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
        uint32_t pos1 = *pF1;
        uint32_t idx1 = pos1 >> step;
        uint32_t frac1 = pos1 & ((1 << step) - 1);
        if (idx1 < sampleend[1] - 1) {
          *pF1 += *pP1;
        } else if (loopsample[1]) {
          *pF1 = (uint32_t)samplebegin[1] << step;
        }
        int16_t s1_1 = *(genstartadress[1] + idx1);
        int16_t s2_1 = *(genstartadress[1] + idx1 + 1);
        float in1 = (float)(s1_1 + (((int32_t)(s2_1 - s1_1) * (int32_t)frac1) >> step));
        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;
        if (*pL1 > 32767.0f)  *pL1 = 32767.0f;
        if (*pL1 < -32768.0f) *pL1 = -32768.0f;
        osc_out[1] = ((int32_t) * pL1 * *pV1) >> 4;
        pF1++; pP1++; pL1++; pB1++; pV1++; pFF1++;

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
        uint32_t pos3 = *pF3;
        uint32_t idx3 = pos3 >> step;
        uint32_t frac3 = pos3 & ((1 << step) - 1);
        if (idx3 < sampleend[3] - 1) {
          *pF3 += *pP3;
        } else if (loopsample[3]) {
          *pF3 = (uint32_t)samplebegin[3] << step;
        }
        int16_t s1_3 = *(genstartadress[3] + idx3);
        int16_t s2_3 = *(genstartadress[3] + idx3 + 1);
        float in3 = (float)(s1_3 + (((int32_t)(s2_3 - s1_3) * (int32_t)frac3) >> step));
        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;
        if (*pL3 > 32767.0f)  *pL3 = 32767.0f;
        if (*pL3 < -32768.0f) *pL3 = -32768.0f;
        osc_out[3] = ((int32_t) * pL3 * *pV3) >> 4;
        pF3++; pP3++; pL3++; pB3++; pV3++; pFF3++;

        // --- 4. STRUKTÚRA MATEK (Hangonkénti feldolgozás) ---
        //totalUpper += (osc_out[0] + osc_out[1]);
        totalUpper += (osc_out[0] * (osc_out[1] >> 12)) >> 3;
        totalLower += (osc_out[2] + osc_out[3]);

        //totalLower += (osc_out[2] * (osc_out[3] >> 12)) >> 3;
      }

      // --- 5. Kimeneti bufferbe töltés és effektezés ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      reverbleft(); reverbright();
      lowpassfilterleft(); lowpassfilterright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  if (STRUCTURE == 66) {
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
      reverbleft(); reverbright();
      lowpassfilterleft(); lowpassfilterright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //----------------------6-------fm-------7---------------------------------------------- Low Compensed----------------------------------

  if ((STRUCTURE == 67) ) {
    // Előre kiszámolt fixek a CPU-nak (nem a ciklusban!)
    const float fMult = 0.0005f;
    const float lMult = 0.0001f;

    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      int32_t totalUpper = 0;
      int32_t totalLower = 0;

      // --- POINTEREK INICIALIZÁLÁSA (Ezek hiányoztak) ---
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
        float fb0 = (lastOut[0][j] * (*pW0 * fMult)) + (lastOut[1][j] * (*pW1 * lMult));
        uint32_t ph0 = ((*pF0 >> step) + (int32_t)fb0) & 1023;

        float in0 = (Waveform[0] == 0) ? sinTable[ph0] : (Waveform[0] == 1 ? (float)((int32_t)(ph0 << 6) - 32768) : (ph0 < *pW0 ? 32767.0f : -32768.0f));

        float hp0 = in0 - *pL0 - (filter_q[0] * *pB0);
        if (hp0 > 32767.0f) hp0 = 32767.0f; else if (hp0 < -32768.0f) hp0 = -32768.0f;
        *pB0 += *pFF0 * hp0;
        *pL0 += *pFF0 * *pB0;

        if (*pL0 > 20000.0f) *pL0 = 20000.0f + (*pL0 - 20000.0f) * 0.2f;
        else if (*pL0 < -20000.0f) *pL0 = -20000.0f + (*pL0 + 20000.0f) * 0.2f;

        lastOut[0][j] = *pL0;
        float d0 = *pV0 * fMult;

        // --- OSC 1 (Carrier 1) ---
        *pF1 += *pP1;
        uint32_t ph1 = ((*pF1 >> step) + (int32_t)((in0 * d0) + (lastOut[3][j] * (*pW3 * lMult)))) & 1023;
        float in1 = (Waveform[1] == 0) ? sinTable[ph1] : (Waveform[1] == 1 ? (float)((int32_t)(ph1 << 6) - 32768) : (ph1 < *pW1 ? 32767.0f : -32768.0f));

        float hp1 = in1 - *pL1 - (filter_q[1] * *pB1);
        if (hp1 > 32767.0f) hp1 = 32767.0f; else if (hp1 < -32768.0f) hp1 = -32768.0f;
        *pB1 += *pFF1 * hp1;
        *pL1 += *pFF1 * *pB1;

        if (*pL1 > 20000.0f) *pL1 = 20000.0f + (*pL1 - 20000.0f) * 0.2f;
        else if (*pL1 < -20000.0f) *pL1 = -20000.0f + (*pL1 + 20000.0f) * 0.2f;

        lastOut[1][j] = *pL1;
        int32_t out1 = ((int32_t)(*pL1) * *pV1) >> 6;

        // --- OSC 2 (Mod 2) ---
        *pF2 += *pP2;
        uint32_t ph2 = ((*pF2 >> step) + (int32_t)(lastOut[2][j] * (*pW2 * fMult))) & 1023;
        float in2 = (Waveform[2] == 0) ? sinTable[ph2] : (Waveform[2] == 1 ? (float)((int32_t)(ph2 << 6) - 32768) : (ph2 < *pW2 ? 32767.0f : -32768.0f));

        float hp2 = in2 - *pL2 - (filter_q[2] * *pB2);
        if (hp2 > 32767.0f) hp2 = 32767.0f; else if (hp2 < -32768.0f) hp2 = -32768.0f;
        *pB2 += *pFF2 * hp2;
        *pL2 += *pFF2 * *pB2;

        if (*pL2 > 20000.0f) *pL2 = 20000.0f + (*pL2 - 20000.0f) * 0.2f;
        else if (*pL2 < -20000.0f) *pL2 = -20000.0f + (*pL2 + 20000.0f) * 0.2f;

        lastOut[2][j] = *pL2;
        float d2 = *pV2 * fMult;

        // --- OSC 3 (Carrier 2) ---
        *pF3 += *pP3;
        uint32_t ph3 = ((*pF3 >> step) + (int32_t)(in2 * d2)) & 1023;
        float in3 = (Waveform[3] == 0) ? sinTable[ph3] : (Waveform[3] == 1 ? (float)((int32_t)(ph3 << 6) - 32768) : (ph3 < *pW3 ? 32767.0f : -32768.0f));

        float hp3 = in3 - *pL3 - (filter_q[3] * *pB3);
        if (hp3 > 32767.0f) hp3 = 32767.0f; else if (hp3 < -32768.0f) hp3 = -32768.0f;
        *pB3 += *pFF3 * hp3;
        *pL3 += *pFF3 * *pB3;

        if (*pL3 > 20000.0f) *pL3 = 20000.0f + (*pL3 - 20000.0f) * 0.2f;
        else if (*pL3 < -20000.0f) *pL3 = -20000.0f + (*pL3 + 20000.0f) * 0.2f;

        lastOut[3][j] = *pL3;
        int32_t out3 = ((int32_t)(*pL3) * *pV3) >> 6;

        // Pointerek léptetése
        pF0++; pP0++; pL0++; pB0++; pW0++; pV0++; pFF0++;
        pF1++; pP1++; pL1++; pB1++; pW1++; pV1++; pFF1++;
        pF2++; pP2++; pL2++; pB2++; pW2++; pV2++; pFF2++;
        pF3++; pP3++; pL3++; pB3++; pW3++; pV3++; pFF3++;

        totalUpper += (out1 + (out3 >> 2));
        totalLower += (out3 + (out1 >> 2));
      }
      // --- Kimeneti lánc (Effektek, EQ, Pan) ---
      bufferbe[0] = totalUpper >> masterVolume;
      bufferbe[1] = totalLower >> masterVolume;
      parametereqleft(); parametereqright();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;
      chorusleft(); chorusright();
      reverbleft(); reverbright();
      lowpassfilterleft(); lowpassfilterright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }

  //----------------------- 76- Új Y-Struktúra -----------------------
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
      parametereqright();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;

      chorusleft(); chorusright();
      reverbleft(); reverbright();
      //processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }



  //-----------------------7-7-Two Fm Osci (Optimized Filtered Feedback)----------------------------------
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
      parametereqright();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7;
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7;

      chorusleft(); chorusright();
      reverbleft(); reverbright();
      //processingStereoReverb();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }








  //BUFFER WRITE DAC
  i2s_write(I2S_PORT, &sBuffer, bufferLen, &i2s_bytes_write, portMAX_DELAY);
}
