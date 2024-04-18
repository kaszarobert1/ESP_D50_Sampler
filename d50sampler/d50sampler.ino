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
#define bufferLen 512
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
const byte polyphony = 8;
uint32_t freqmutato[4][polyphony];
uint32_t pich[4][polyphony];
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
uint16_t GLOBAL_TUNE = 15104;
byte COARSE[4] = { 48, 48, 48, 48 };
byte FINE[4] = { 50, 50, 50, 50 };
byte szorzo[4] = {1, 1, 1, 1};
byte LKeyShift = 0;
byte UKeyShift = 0;
byte volume[4] { 20, 20, 20, 20 };
byte generatorvolume[4][polyphony];
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
int32_t tempbuffer0;
int32_t tempbuffer1;
int32_t tempbuffer2;
int32_t tempbuffer3;
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
byte generatorstatus[4][polyphony];
byte TVAvolume[4][polyphony];
byte TVA[4] = {1, 1, 1, 1};
byte KEYFollow[4] = { 11, 11, 11, 11 };
byte LFOMode[4] = { 0, 0, 0, 0 };
byte PENVMode[4] = { 0, 0, 0, 0 };
byte BENDERMode[4] = { 0, 0, 0, 0 };
byte Waveform[4] = { 0, 1, 0, 1 };
byte PCMWaveNo[4] = { 69, 68, 69, 32 };
byte BiasPoint[4] = {64, 64, 64, 64};
byte BiasLevel[4] = {12, 12, 12, 12};
byte Bias[4][256];
byte STRUCTURE_U = 5;
byte STRUCTURE_L = 5;
uint32_t lfoarrayindex[LFOnumber] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t lfovalue[LFOnumber];
byte lfofreq[LFOnumber] = {7, 10, 22, 22, 22, 22, 22, 22};
float f0 = 100;
float f0orig = 100;
float Q = 2;
float f02 = 100;
float f02orig = 100;
float Q2 = 2;
byte lfo2level = 2;
byte lfo2sync = 2;
byte lfo3level = 2;
byte lfo3sync = 2;
byte CHASE_TIME = 0;
byte CHASE_LEVEL = 4;
uint32_t ido;
long elozoido;
byte lastchase = 0;
byte CaseArray[32];
byte chaseindex = 0;
byte MIDI_SYNC = 1;
byte sixteen = 0;



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
  sizes[16] = sizeof(pianosample) >> 1;
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
  sizes[33] = sizeof(steamer) >> 1;
  sizes[34] = sizeof(steamer) >> 1;
  sizes[35] = sizeof(steamer) >> 1;
  sizes[36] = sizeof(steamer) >> 1;
  sizes[37] = sizeof(steamer) >> 1;
  sizes[38] = sizeof(steamer) >> 1;
  sizes[39] = sizeof(steamer) >> 1;
  sizes[40] = sizeof(steamer) >> 1;
  sizes[41] = sizeof(steamer) >> 1;
  sizes[42] = sizeof(steamer) >> 1;
  sizes[43] = sizeof(steamer) >> 1;
  sizes[44] = sizeof(steamer) >> 1;
  sizes[45] = sizeof(violins) >> 1;
  sizes[46] = sizeof(pidzicart) >> 1;
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
  //loop
  //sizes[68] = (sizeof(marimba) + sizeof(vibraphone)) >> 1 ;
  sizes[68] = (sizeof(vibraphone) + sizeof(xilophone1)) >> 1;
  // sizes[70] = (sizeof(vibraphone) + sizeof(xilophone1) + sizeof(xilophone2)) >> 1;
  // sizes[71] = (sizeof(vibraphone) + sizeof(xilophone1) + sizeof(xilophone2) + sizeof(logbass)) >> 1;
  sizes[69] = (sizeof(xilophone1) + sizeof(xilophone2)) >> 1;
  //sizes[73] = (sizeof(xilophone1) + sizeof(xilophone2) + sizeof(logbass)) >> 1;
  // sizes[74] = (sizeof(xilophone1) + sizeof(xilophone2) + sizeof(logbass) + sizeof(hammer)) >> 1;
  // sizes[75] = (sizeof(xilophone2) + sizeof(logbass)) >> 1;
  // sizes[76] = (sizeof(xilophone2) + sizeof(logbass) + sizeof(hammer)) >> 1;
  sizes[70] = (sizeof(xilophone2) + sizeof(logbass) + sizeof(hammer) + sizeof(japanesedrum)) >> 1;
  sizes[71] = (sizeof(logbass) + sizeof(hammer)) >> 1;
  sizes[72] = (sizeof(logbass) + sizeof(hammer) + sizeof(japanesedrum)) >> 1;
  sizes[73] = (sizeof(logbass) + sizeof(hammer) + sizeof(japanesedrum) + sizeof(kalimba)) >> 1;
  sizes[74] = (sizeof(hammer) + sizeof(japanesedrum)) >> 1;
  sizes[75] = (sizeof(hammer) + sizeof(japanesedrum) + sizeof(kalimba)) >> 1;
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






  //sizes[74] = sizeof(maleloop2) >> 1;
}

void setsamplesize() {
  //Set up max sample size
  samplesize[opmenuoldal] = sizes[PCMWaveNo[opmenuoldal]];
  sampleend[opmenuoldal] = samplesize[opmenuoldal];
  Serial.println("Size of sample:");
  Serial.println(String(samplesize[0]));
  Serial.println(String(samplesize[1]));
  Serial.println(String(samplesize[2]));
  Serial.println(String(samplesize[3]));
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
    case 16: genstartadress[opmenuoldal] = pianosample; break;
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
    case 33: genstartadress[opmenuoldal] = steamer; break;
    case 34: genstartadress[opmenuoldal] = steamer; break;
    case 35: genstartadress[opmenuoldal] = steamer; break;
    case 36: genstartadress[opmenuoldal] = steamer; break;
    case 37: genstartadress[opmenuoldal] = steamer; break;
    case 38: genstartadress[opmenuoldal] = steamer; break;
    case 39: genstartadress[opmenuoldal] = steamer; break;
    case 40: genstartadress[opmenuoldal] = steamer; break;
    case 41: genstartadress[opmenuoldal] = steamer; break;
    case 42: genstartadress[opmenuoldal] = steamer; break;
    case 43: genstartadress[opmenuoldal] = steamer; break;
    case 44: genstartadress[opmenuoldal] = steamer; break;
    case 45: genstartadress[opmenuoldal] = violins; break;
    case 46: genstartadress[opmenuoldal] = pidzicart; break;
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
    //loop
    //  case 68: genstartadress[opmenuoldal] = marimba; break;
    case 68: genstartadress[opmenuoldal] = vibraphone; break;
    //  case 70: genstartadress[opmenuoldal] = vibraphone; break;
    // case 71: genstartadress[opmenuoldal] = vibraphone; break;
    case 69: genstartadress[opmenuoldal] = xilophone1; break;
    // case 73: genstartadress[opmenuoldal] = xilophone1; break;
    // case 74: genstartadress[opmenuoldal] = xilophone1; break;
    // case 75: genstartadress[opmenuoldal] = xilophone2; break;
    // case 76: genstartadress[opmenuoldal] = xilophone2; break;
    case 70: genstartadress[opmenuoldal] = xilophone2; break;
    case 71: genstartadress[opmenuoldal] = logbass; break;
    case 72: genstartadress[opmenuoldal] = logbass; break;
    case 73: genstartadress[opmenuoldal] = logbass; break;
    case 74: genstartadress[opmenuoldal] = hammer; break;
    case 75: genstartadress[opmenuoldal] = hammer; break;
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

void setLFOWave() {
  LFOadress[0] = lfosine;
  LFOadress[1] = lfotriangle;
  LFOadress[2] = lfotriangle;
  LFOadress[3] = lfotriangle;
  LFOadress[4] = lfosine;
  LFOadress[5] = lfotriangle;
  LFOadress[6] = lfosine;
  LFOadress[7] = lfotriangle;
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
  String eredmeny = "";
  eredmeny += (cc % 100) / 10;
  eredmeny += cc % 10;
  return eredmeny;
}
String lcdprint3(int cc)
{
  String eredmeny = "";
  eredmeny += cc / 100;
  eredmeny += (cc % 100) / 10;
  eredmeny += cc % 10;
  return eredmeny;
}
//--------------MIDI SYSEX PARAMETER CONTROL------
void parametersysexchanged() {
  //byte step = 1;
  byte value = velocityByte;
  String line = "";
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
        line = "COARSE U1: " + lcdprint3(FINE[2]);
        notetune();
        break;
      case 2:
        KEYFollow[2] = value;
        line = "KEYFollow U1: " + lcdprint2(KEYFollow[2]);
        notetune();
        break;
      case 3:

        break;
      case 4:
        TVA[2] = value;
        line = "TVA U1: " + lcdprint3(TVA[1]);
        break;
      case 7:
        PCMWaveNo[2] = value;
        line = "PCMWaveNo U1: " + lcdprint3(PCMWaveNo[2]);

        //
        opmenuoldal = 2;
        setPCMWave();
        break;
      case 35:
        volume[2] = value;
        line = "Level U1: " + lcdprint3(volume[2]);
        break;
      case 37:
        BiasPoint[2] = value;
        line = "BiasPoint U1: " + lcdprint3(BiasPoint[2]);
        notebias();
        break;
      case 38:
        BiasLevel[2] = value;
        line = "bieasLevel U1: " + lcdprint3(BiasLevel[2]);
        notebias();
        break;
      case 39:
        ENV_T1[2] = 100 - value;
        line = "ENV_T1 U1: " + lcdprint3(ENV_T1[2]);
        break;
      case 40:
        ENV_T2[2] = 100 - value;
        line = "ENV_T2 U1:" + lcdprint3(ENV_T2[2]);
        break;
      case 41:
        ENV_T3[2] = 100 - value;
        line = "ENV_T3 U1:" + lcdprint3(ENV_T3[2]);
        break;
      case 42:
        ENV_T4[2] = 100 - value;
        line = "ENV_T4 U1:" + lcdprint3(ENV_T4[2]);
        break;
      case 43:
        /*
          step = samplesize[2] / 100;
          sampleend[2] = value * step;
          Serial.println("SAMPLE END U1: " + String(sampleend[2]));
        */
        ENV_T5[2] = 100 - value;
        line = "ENV_T5 U1:" + lcdprint3(ENV_T5[2]);
        break;
      case 44:
        ENV_L1[2] = value;
        line = "ENV_L1 U1:" + lcdprint3(ENV_L1[2]);
        break;
      case 45:
        ENV_L2[2] = value;
        line = "ENV_L2 U1:" + String(ENV_L2[2]);
        break;
      case 46:
        /*
          step = samplesize[2] / 100;
          samplebegin[2] = value * step;
          Serial.println("SAMPLE BEGIN U1: " + String(samplebegin[2]));
        */
        ENV_L3[2] = value;
        line = "ENV_L3 U1:" + lcdprint3(ENV_L3[2]);
        break;
      case 47:
        ENV_LSUS[2] = value;
        line = "ENV_LSUS U1: " + lcdprint3(ENV_LSUS[2]);
        break;
      case 48:
        ENV_LEND[2] = value;
        line = "ENV_LEND U1: " + lcdprint3(ENV_LEND[2]);
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
      case 64:
        //couarse u2
        COARSE[3] = value;
        line = "COARSE U2:" + lcdprint3(COARSE[3]);
        notetune();
        break;
      case 65:
        //couarse u2
        FINE[3] = value;
        line = "COARSE U2:" + lcdprint3(FINE[3]);
        notetune();
        break;
      case 66:
        KEYFollow[3] = value;
        line = "KEYFollow U2:" + lcdprint2(KEYFollow[3]);
        notetune();
        break;
      case 67:

        break;
      case 68:
        TVA[3] = value;
        line = "TVA U2: " + lcdprint3(TVA[3]);
        break;
      case 71:
        PCMWaveNo[3] = value;
        line = "PCMWaveNo U2: " + lcdprint3(PCMWaveNo[3]);
        opmenuoldal = 3;
        setPCMWave();
        //lcd

        break;
      case 99:
        volume[3] = value;
        line = "Level U2: " + lcdprint3(volume[3]);
        break;
      case 101:
        BiasPoint[3] = value;
        line = "BiasPoint L2: " + lcdprint3(BiasPoint[3]);
        notebias();
        break;
      case 102:
        BiasLevel[3] = value;
        line = "bieasLevel L1: " + lcdprint3(BiasLevel[3]);
        notebias();
        break;
      case 103:
        ENV_T1[3] = 100 - value;
        line = "ENV_T1 U2: " + lcdprint3(ENV_T1[3]);
        break;
      case 104:
        ENV_T2[3] = 100 - value;
        line = "ENV_T2 U2:" + lcdprint3(ENV_T2[3]);
        break;
      case 105:
        ENV_T3[3] = 100 - value;
        line = "ENV_T3 U2:" + lcdprint3(ENV_T3[3]);
        break;
      case 106:
        ENV_T4[3] = 100 - value;
        line = "ENV_T4 U2:" + lcdprint3(ENV_T4[3]);
        break;
      case 107:
        /*
          step = samplesize[3] / 100;
          sampleend[3] = value * step;
          Serial.println("SAMPLE END U2: " + String(sampleend[3]));
        */
        ENV_T5[3] = 100 - value;
        line = "ENV_T5 U2:" + lcdprint3(ENV_T5[3]);
        break;

      case 108:
        ENV_L1[3] = value;
        line = "ENV_L1 U2: " + lcdprint3(ENV_L1[3]);
        break;
      case 109:
        ENV_L2[3] = value;
        line = "ENV_L2 U2: " + lcdprint3(ENV_L2[3]);
        break;
      case 110:
        /*
          step = samplesize[3] / 100;
          samplebegin[3] = value * step;
          Serial.println("SAMPLE BEGIN L1: " + String(samplebegin[3]));
        */
        ENV_L3[3] = value;
        line = "ENV_L3 U2: " + lcdprint3(ENV_L3[3]);
        break;
      case 111:
        ENV_LSUS[3] = value;
        line = "ENV_LSUS U2" + lcdprint3(ENV_LSUS[3]);
        break;
      case 112:
        ENV_LEND[3] = value;
        line = "ENV_LEND U2: " + lcdprint3(ENV_LEND[3]);
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
    }

  if (localParameterByte == 1)
    switch (noteByte) {

      case 1:

        break;

      case 3:
        LFOMode[0] = value;
        line = "LFOMode L1: " + lcdprint3(LFOMode[0]);
        break;
      case 4:
        PENVMode[0] = value;
        line = "LFOMode L1: " + lcdprint3(PENVMode[0]);
        break;
      case 5:
        BENDERMode[0] = value;
        line = "BENDERMode L1: " + lcdprint3(BENDERMode[0] );
        break;
      case 6:
        break;
      case 10:
        STRUCTURE_U = value;
        line = "STRUCTURE_U: " + lcdprint3(STRUCTURE_U );
        break;
      case 43:
        lfofreq[1] = value;
        line = "CHORUSFREQ U: " + lcdprint3(lfofreq[1] );
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
      case 26:
        lfofreq[3] = value;
        line = " lfofreq2: " + lcdprint3(lfofreq[3]);
        break;
      case 27:
        lfo3level = value;
        line = " lfofreq2: " + lcdprint3(lfo3level);
        break;
      case 28:
        lfo3sync = value;
        line = " lfofreq3: " + lcdprint3(lfo3sync);
        break;
      case 44:
        chorusLevelRight = value;
        line = "CHORUSLEVEL U: " + lcdprint3(chorusLevelRight );
        break;
      case 64:
        COARSE[0] = value;
        line = "COARSE L1: " + lcdprint3(COARSE[0]);
        notetune();
        break;
      case 65:
        FINE[0] = value;
        line = "FINE L1: " + lcdprint3(FINE[0]);
        notetune();
        break;
      case 66:
        KEYFollow[0] = value;
        line = "KEYFollow L1: " + lcdprint2(KEYFollow[0]);
        notetune();

        break;
      case 67:
        TVA[0] = value;
        line = "TVA L1 ON-OFF: " + lcdprint3(TVA[0]);
        break;
      case 71:
        PCMWaveNo[0] = value;
        line = "PCMWaveNo L1: " + lcdprint3(PCMWaveNo[0]);
        opmenuoldal = 0;
        setPCMWave();
        //lcd


        break;
      case 99:
        volume[0] = value;
        line = "Level L1: " + lcdprint3(volume[0]);
        break;
      case 101:
        BiasPoint[0] = value;
        line = "BiasPoint L1: " + lcdprint3(BiasPoint[0]);
        notebias();
        break;
      case 102:
        BiasLevel[0] = value;
        line = "BieasLevel L1: " + lcdprint3(BiasLevel[0]);
        notebias();
        break;
      case 103:
        ENV_T1[0] = 100 - value;
        line = "ENV_T1 L1: " + lcdprint3(ENV_T1[0]);
        break;
      case 104:
        ENV_T2[0] = 100 - value;
        line = "ENV_T2 L1:" + lcdprint3(ENV_T2[0]);
        break;
      case 105:
        ENV_T3[0] = 100 - value;
        line = "ENV_T3 L1:" + lcdprint3(ENV_T3[0]);
        break;
      case 106:
        ENV_T4[0] = 100 - value;
        line = "ENV_T4 L1" + lcdprint3(ENV_T4[0]);
        break;
      case 107:
        ENV_T5[0] = 100 - value;
        line = "ENV_T5 L1" + lcdprint3(ENV_T5[0]);
        break;
      case 108:
        ENV_L1[0] = value;
        line = "ENV_L1 L1: " + lcdprint3(ENV_L1[0]);
        break;
      case 109:
        ENV_L2[0] = value;
        line = "ENV_L2 L1: " + lcdprint3(ENV_L2[0]);
        break;
      case 110:
        ENV_L3[0] = value;
        line = "ENV_L3 L1: " + lcdprint3(ENV_L3[0]);
        break;
      case 111:
        ENV_LSUS[0] = value;
        line = "ENV_LSUS L1" + lcdprint3(ENV_LSUS[0]);
        break;
      case 112:
        ENV_LEND[0] = value;
        line = "ENV_LEND L1" + lcdprint3(ENV_LEND[0]);
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
        if (value == 4) {
          sampleend[0]++;
          if (samplesize[0] < sampleend[0]) {
            sampleend[0] = samplesize[0];
          }
        }

        line = "SAMPLE END L1: " + String(sampleend[0]);
        if (value == 1) {
          samplebegin[0]++;
          if (samplesize[0] < samplebegin[0]) {
            samplebegin[0] = samplesize[0];
          }
        }
        line = "SAMPLE BEGIN L1: " + String(samplebegin[0]);

        break;
    }

  if (localParameterByte == 2) {
    switch (noteByte) {
      case 0:
        COARSE[1] = value;
        line = "COARSE L2: " + lcdprint3(COARSE[1]);
        notetune();
        break;
      case 1:
        FINE[1] = value;
        line = "FINE L2: " + lcdprint3(FINE[1]);
        notetune();
        break;
      case 2:
        KEYFollow[1] = value;
        line = "KEYFollow L2: " + lcdprint2(KEYFollow[1]);
        notetune();
        break;
      case 4:
        TVA[1] = value;
        line = "TVA L2: " + lcdprint3(TVA[1]);
        break;
      case 7:
        PCMWaveNo[1] = value;
        line = "PCMWaveNo L2: " + lcdprint3(PCMWaveNo[1]);
        opmenuoldal = 1;
        setPCMWave();
        //lcd

        break;

      case 35:
        volume[1] = value;
        line = "Level L2: " + lcdprint3(volume[1]);
        break;
      case 37:
        BiasPoint[1] = value;
        line = "BiasPoint L2: " + lcdprint3(BiasPoint[1]);
        notebias();
        break;
      case 38:
        BiasLevel[1] = value;
        line = "bieasLevel L2: " + lcdprint3(BiasLevel[1]);
        notebias();
        break;
      case 39:
        ENV_T1[1] = 100 - value;
        line = "ENV_T1 L2: " + lcdprint3(ENV_T1[1]);
        break;
      case 40:
        ENV_T2[1] = 100 - value;
        line = "ENV_T2 L2" + lcdprint3(ENV_T2[1]);
        break;
      case 41:
        ENV_T3[1] = 100 - value;
        line = "ENV_T3 L2" + lcdprint3(ENV_T3[1]);
        break;
      case 42:
        ENV_T4[1] = 100 - value;
        line = "ENV_T4 L2" + lcdprint3(ENV_T4[1]);
        break;
      case 43:
        /*
          step = samplesize[1] / 100;
          sampleend[1] = value * step;
          Serial.println("SAMPLE END L2: " + String(sampleend[1]));
        */
        ENV_T5[1] = 100 - value;
        line = "ENV_T5 L2" + lcdprint3(ENV_T5[1]);
        break;
      case 44:
        ENV_L1[1] = value;
        line = "ENV_L1 L2: " + lcdprint3(ENV_L1[1]);
        break;
      case 45:
        ENV_L2[1] = value;
        line = "ENV_L2 L2: " + lcdprint3(ENV_L2[1]);
        break;
      case 46:
        /*
          step = samplesize[1] / 100;
          samplebegin[1] = value * step;
          Serial.println("SAMPLE BEGIN L2: " + String(samplebegin[1]));
        */
        ENV_L3[1] = value;
        line = "ENV_L3 L2: " + lcdprint3(ENV_L3[1]);
        break;
      case 47:
        ENV_LSUS[1] = value;
        line = "ENV_LSUS L2: " + lcdprint3(ENV_LSUS[1]);
        break;
      case 48:
        ENV_LEND[1] = value;
        line = "ENV_LEND L2: " + lcdprint3(ENV_LEND[1]);
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
      case 74:
        STRUCTURE_L = value;
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
      case 90:
        lfofreq[2] = value;
        line = " lfofreq2: " + lcdprint3(lfofreq[2]);
        break;
      case 91:
        lfo2level = value;
        line = " lfofreq2: " + lcdprint3(lfo2level);
        break;
      case 92:
        lfo2sync = value;
        line = " lfo2sync: " + lcdprint3(lfo2sync);
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

        }
        break;
      case 107:
        // chorusRate=value;
        lfofreq[0] = value;
        line = " Chorus RATE L: " + lcdprint3(lfofreq[0]);
        break;
      case 108:
        chorusLevelLeft = value;
        line = " Chorus LEVEL U: " + lcdprint3( chorusLevelLeft);
        break;

    }
  }
  if (localParameterByte == 3) {
    switch (noteByte) {

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
        }//
        notetune();
        line = " Step: " + String(step);
        line = " GLOBAL_TUNE: " + String(GLOBAL_TUNE);
        break;
      case 30:
        switch (value) {
          case 0:
            delaybuffersize = 256;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            line = "Small Hall";
            break;
          case 1:
            delaybuffersize = 512;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            line = "1. Medium Hall";
            break;
          case 2:
            delaybuffersize = 1024;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            line = "2. Large Hall";
            break;
          case 3:
            delaybuffersize = 2048;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 60;
            line = "3. Chapel";
            break;
          case 4:
            delaybuffersize = 4096;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 40;
            line = "4. SmallHall";
            break;
          case 5:
            delaybuffersize = 128;
            delaytime = 2;
            delay2time = 1;
            reverblevel = 40;
            line = "5. Box";
            break;
          case 6:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 1;
            reverblevel = 40;
            line = "5. Small Metal Room";
            break;
          case 7:
            delaybuffersize = 8192;
            delaytime = 1;
            delay2time = 2;
            reverblevel = 40;
            line = "6. Small Room";
            break;
          case 8:
            delaybuffersize = 8192;
            delaytime = 2;
            delay2time = 2;
            reverblevel = 40;
            line = "7. Room";
            break;
          case 9:
            delaybuffersize = 8192;
            delaytime = 3;
            delay2time = 2;
            reverblevel = 40;
            line = "8. Medium Room";
            break;
          case 10:
            delaybuffersize = 8192;
            delaytime = 3;
            delay2time = 3;
            reverblevel = 40;
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
        }
        break;
      case 31:
        reverblevel = value;
        line = "ReverbLevel: " + lcdprint3(reverblevel);
        break;
      case 32:

        break;
      case 34:
        MIDI_SYNC = value;
        line = "MIDI_SYNC: " + lcdprint3(MIDI_SYNC);
        break;
      case 35:
        CHASE_LEVEL = value;
        line = "CHASE LEVEL: " + lcdprint3(CHASE_LEVEL);
        break;
      case 36:
        CHASE_TIME = value;
        line = "CHASE TIME: " + lcdprint3(CHASE_TIME);
        break;

    }
  }
  //serial
  Serial.println(line);
  //lcd
  lcdprint(line);
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
  atlag += (bufferbe[0] * reverblevel) >> 6 ;
  delaystep++;
  if (delaystep >= delaytime) {
    delaybuffer[delaybufferindex] = atlag / delaystep ;
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
  atlag2 += (bufferbe[1] * reverblevel) >> 6 ;
  delay2step++;
  if (delay2step >= delay2time) {
    delaybuffer2[delaybufferindex2] = atlag2 / delay2step ;
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
void chorusleft() {
  lfovalue[0] = *(LFOadress[0] + (lfoarrayindex[0] >> 23));
  lfoarrayindex[0] += (lfofreq[0] << 13);

  chorusbufferleft[chorusbufferindex] = bufferbe[0];
  chorusbufferindex++;
  chorusbufferindex &= (chorusbuffersize);
  chorusindex = (lfovalue[0] + chorusbufferindex) % chorusbuffersize;
  //     Serial.println("CHORUSINDEX: " + String( chorusindex ));



  atlagchorus0 += (((chorusbufferleft[chorusindex] * chorusLevelLeft) >> 7) + atlagchorus0) >> 1;
  bufferbe[0] = bufferbe[0] + atlagchorus0;
  atlagchorus0 = atlagchorus0 >> 1;

}

//--------------------------CHORUS RIGHT------------------------------

int chorusbufferright[512];
uint16_t chorusbufferindex2 = 0;
uint16_t chorusindex2;
int16_t atlagchorus1 = 0;

void chorusright() {
  lfovalue[1] = *(LFOadress[1] + (lfoarrayindex[1] >> 23));
  lfoarrayindex[1] += (lfofreq[1] << 13);
  chorusbufferright[chorusbufferindex2] = bufferbe[1];
  chorusbufferindex2++;
  chorusbufferindex2 &= (chorusbuffersize2);
  chorusindex2 = (lfovalue[1] + chorusbufferindex2) % chorusbuffersize2;
  //Serial.println("CHORUSINDEX2: " + String( chorusindex2 ));

  atlagchorus1 += (((chorusbufferright[chorusindex2] * chorusLevelRight) >> 7) + atlagchorus1) >> 1;
  bufferbe[1] = bufferbe[1] + atlagchorus1;
  atlagchorus1 = atlagchorus1 >> 1;


  //bufferbe[1] = (bufferbe[1] + ((chorusbufferright[chorusindex2] * chorusLevelRight) >> 7));
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
void keyon(byte noteByte) {
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
  CaseArray[chaseindex] = noteByte;
  pich[0][generatornumber] = wavefreq[0][generatornumber];
  pich[1][generatornumber] = wavefreq[1][generatornumber];
  pich[2][generatornumber] = wavefreq[2][generatornumber];
  pich[3][generatornumber] = wavefreq[3][generatornumber];
  // Serial.println(String(pich[generatornumber]));
  freqmutato[0][generatornumber] = samplebegin[0] << step;
  freqmutato[1][generatornumber] = samplebegin[1] << step;
  freqmutato[2][generatornumber] = samplebegin[2] << step;
  freqmutato[3][generatornumber] = samplebegin[3] << step;
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
  if (lfo2sync == 2) {
    lfoarrayindex[2] = 0;
  }
  if (lfo3sync == 2) {
    lfoarrayindex[3] = 0;
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
  if (CHASE_TIME > 0) {
    if (MIDI_SYNC == 1) {
      //  Serial.println("clock: ");
      sixteen++;
      if (sixteen % CHASE_TIME == 0) {
        keyoff(lastchase);
        chaseindex++;
        if (chaseindex >= CHASE_LEVEL) {
          chaseindex = 0;
        }
        if (CaseArray[chaseindex] != 0)
        {
          lastchase = CaseArray[chaseindex];
          keyon(lastchase);
          //   Serial.println("CHASE_TIME: " + String( MIDI_SYNC));
          // Serial.println("CHASE_TIME: " + String( CHASE_TIME));
        }
      }
    }
  }
}

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

void serialEvent() {
  if (MIDI2.read(midichan)) {
    switch (MIDI2.getType()) {
      case midi::NoteOn:
        //  Serial.println("NoteOn: " + String(noteByte) + " " + String(velocityByte) + " ");
        noteByte = MIDI2.getData1();
        velocityByte = MIDI2.getData2();
        keyon(noteByte);
        break;
      case midi::NoteOff:
        //  Serial.println("Noteoff: " + String(noteByte) + " " + String(velocityByte) + " ");
        noteByte = MIDI2.getData1();
        //  velocityByte = MIDI2.getData2();
        keyoff(noteByte);
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
        chasearpeggiomidiclock();
        break;
      case midi::SystemExclusive:
        Serial.print("SysexDATA: ");
        for (int i = 0; i < MIDI2.getSysExArrayLength(); i++) {
          Serial.print(String(MIDI2.getSysExArray()[i]) + " ");
        }
        Serial.println();
        //prefix: 240 65 0 20 18 0
        if (MIDI2.getSysExArray()[0] == 240 && MIDI2.getSysExArray()[1] == 65 && MIDI2.getSysExArray()[2] == 0 && MIDI2.getSysExArray()[3] == 20 && MIDI2.getSysExArray()[4] == 18 && MIDI2.getSysExArray()[5] == 0)
        {
          //single sysex: array[6] array[7] array[8]
          if (MIDI2.getSysExArrayLength() <= 11)
          {
            localParameterByte = MIDI2.getSysExArray()[6];
            noteByte = MIDI2.getSysExArray()[7];
            velocityByte = MIDI2.getSysExArray()[8];
            parametersysexchanged();
          }
          //not single sysex 3-3-3-3...-3to-->arraylength:
          else {
            LCD_ON = false;
            Serial.println("lcdki");
            localParameterByte = MIDI2.getSysExArray()[6];
            noteByte = MIDI2.getSysExArray()[7];
            velocityByte = MIDI2.getSysExArray()[8];
            Serial.println("Localvalue:" + String(localParameterByte) + " Commandsysex:" + String(noteByte) + " Datasysex:" + String(velocityByte));

            parametersysexchanged();

            for (int i = 9; i < MIDI2.getSysExArrayLength() - 2; i++) {
              noteByte++ ;
              velocityByte = MIDI2.getSysExArray()[i];
              Serial.println("Localvalue:" + String(localParameterByte) + " Commandsysex:" + String(noteByte) + " Datasysex:" + String(velocityByte));
              parametersysexchanged();
            }
            LCD_ON = true;
            Serial.println("lcdbe");
          }
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
  eqkiszamol();
}

/*
  uint32_t egyopgenA(uint32_t freqmutato1, long op1level, byte lep)
  {
  return generator1[freqmutato1 >> lep] * op1level;
  }
*/
uint32_t tvapointer[4][polyphony];
void loop() {
  if (MIDI_SYNC == 2)
  {
    chasearpeggio();
  }
  //--MIDI input--
  serialEvent();
  //LFO
  for (int i = 2; i < LFOnumber; i++) {
    lfovalue[i] = *(LFOadress[i] + (lfoarrayindex[i] >> 23));
    lfoarrayindex[i] += (lfofreq[i] << 19);
  }


  if (true) {
    f0 = f0orig + (lfovalue[2] * lfo2level);
    eqkiszamol();
  }
  if (true) {
    f02 = f02orig + (lfovalue[3] * lfo3level);
    eqkiszamol2();
  }
  // Serial.println(String(f0));

  // Serial.println("lfo1value : " + String(lfo1value ));

  //TVA ENVELOPE
  for (int i = 0; i < 4; i++) {
    if (TVA[i] > 0) {
      for (int j = 0; j < polyphony; j++) {
        switch (generatorstatus[i][j]) {
          case 0:
            TVAvolume[i][j] = linear[tvapointer[i][j] >> 18];
            tvapointer[i][j] += ENV_T1[i] << 16;
            if (TVAvolume[i][j] > ENV_L1[i] - ENV_T1[i]) {
              TVAvolume[i][j] = ENV_L1[i];
              tvapointer[i][j] = 0;
              generatorstatus[i][j]++;
            }
            break;
          case 1:
            /*
              if (ENV_L2[i] < ENV_L1[i]) {
              TVAvolume[i][j] = TVAvolume[i][j] - ENV_T2[i];
              if (TVAvolume[i][j] < ENV_L2[i])
              {
                TVAvolume[i][j] = ENV_L2[i];
                generatorstatus[i][j]++;
              }
              }
              if (ENV_L2[i] >= ENV_L1[i]) {
              TVAvolume[i][j] = TVAvolume[i][j] + ENV_T2[i];
              if (TVAvolume[i][j] > ENV_L2[i])
              {
                TVAvolume[i][j] = ENV_L2[i];
                generatorstatus[i][j]++;
              }
              }
            */
            TVAvolume[i][j] = ENV_L1[i] - linear[tvapointer[i][j] >> 18];
            tvapointer[i][j] += ENV_T2[i] << 14;
            if (TVAvolume[i][j] < ENV_L2[i] + ENV_T2[i])
            {
              TVAvolume[i][j] = ENV_L2[i];
              tvapointer[i][j] = 0;
              generatorstatus[i][j]++;
            }
            break;
          case 2:

            /*if (ENV_LSUS[i] < ENV_L2[i]) {
              TVAvolume[i][j] = TVAvolume[i][j] - ENV_T3[i];
              if (TVAvolume[i][j] < ENV_LSUS[i])
              {
                TVAvolume[i][j] = ENV_LSUS[i];
                generatorstatus[i][j]++;
              }
              }
              if (ENV_LSUS[i] >= ENV_L2[i]) {
              TVAvolume[i][j] = TVAvolume[i][j] + ENV_T3[i];
              if (TVAvolume[i][j] > ENV_LSUS[i])
              {
                TVAvolume[i][j] = ENV_LSUS[i];
                generatorstatus[i][j]++;
              }
              }
            */
            TVAvolume[i][j] = ENV_L2[i] - linear[tvapointer[i][j] >> 18];
            tvapointer[i][j] += ENV_T3[i] << 14;
            if (TVAvolume[i][j] < ENV_L3[i] + ENV_T3[i])
            {
              TVAvolume[i][j] = ENV_L3[i];
              tvapointer[i][j] = 0;
              generatorstatus[i][j]++;
            }
            break;
          case 3:
            // TVAvolume[i][j] = ENV_LSUS[i];
            TVAvolume[i][j] = ENV_L3[i] - linear[tvapointer[i][j] >> 18];
            tvapointer[i][j] += ENV_T4[i] << 14;
            if (TVAvolume[i][j] < ENV_LSUS[i] + ENV_T4[i])
            {
              TVAvolume[i][j] = ENV_LSUS[i];
              tvapointer[i][j] = 0;
              generatorstatus[i][j]++;

            }


            break;
          case 4:
            TVAvolume[i][j] = ENV_LSUS[i] - linear[tvapointer[i][j] >> 18];
            tvapointer[i][j] += ENV_T5[i] << 14;
            if (TVAvolume[i][j] < ENV_LEND[i] + ENV_T5[i]) 
            {
              TVAvolume[i][j] = ENV_LEND[i];
              tvapointer[i][j] = 0;
              generatorstatus[i][j]++;
            }
            /*
                       if (ENV_LEND[i] < ENV_LSUS[i]) {
                         TVAvolume[i][j] = TVAvolume[i][j] - ENV_T4[i];
                         if(TVAvolume[i][j]<=ENV_LEND[i])
                         {
                            TVAvolume[i][j]=ENV_LEND[i];
                            generatorstatus[i][j]++;
                         }
                       }
                       if (ENV_LEND[i] >= ENV_LSUS[i]) {
                         TVAvolume[i][j] = TVAvolume[i][j] + ENV_T4[i];
                         if(TVAvolume[i][j]>=ENV_LEND[i])
                         {
                            TVAvolume[i][j]=ENV_LEND[i];
                            generatorstatus[i][j]++;
                         }
                       }
            */
            break;
          case 5:
            TVAvolume[i][j] = 0;
            break;
        }
        if (TVA[i] == 1) {
          generatorvolume[i][j] = (TVAvolume[i][j] * volume[i] * wavebias[i][j]) >> 10;
          // generatorvolume[i][j] = ((volume[i] * wavebias[i][j])<<TVAvolume[i][j]) >> 5;
        }
        if (TVA[i] == 2) {
          generatorvolume[i][j] = ((volume[i] * wavebias[i][j]) >> TVAvolume[i][j] ) >> 5;
        }

      }
      //  Serial.println("Generator" + String(i) + "statusz: " + String(TVAvolume[i][0]) + " " + String(TVAvolume[i][1]) + " " + String(TVAvolume[i][2]) + " " + String(TVAvolume[i][3]) + " " + String(TVAvolume[i][4]) + " " + String(TVAvolume[i][5]));
    } else
    {
      for (int j = 0; j < polyphony; j++) {
        generatorvolume[i][j] =  (volume[i] * wavebias[i][j]) >> 4;
      }
    }
  }
  //release


  //--2 SOUND LEFT, 2 SOUND RIGHT, 6  POLYFONI!!!!--

  //STRUCTURES

  //------------------5-------------------5---------------------

  if (STRUCTURE_U == 5 && STRUCTURE_L == 5)
  {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      bufferbe[0] = 0;
      bufferbe[1] = 0;
      bufferbe[2] = 0;
      bufferbe[3] = 0;
      for (int j = 0; j < polyphony; j++) {
        //if end of sample
        if (((freqmutato[0][j] >> step) < sampleend[0] - 1 )) {
          freqmutato[0][j] += pich[0][j];
        } else if (loopsample[0]) {

          freqmutato[0][j] = samplebegin[0] << step;
        }
        if (((freqmutato[1][j] >> step) < sampleend[1] - 1 )  ) {
          freqmutato[1][j] += pich[1][j];
        } else if (loopsample[1]) {

          freqmutato[1][j] = samplebegin[1] << step;
        }
        if (((freqmutato[2][j] >> step) < sampleend[2] - 1 ) ) {
          freqmutato[2][j] += pich[2][j];
        } else if (loopsample[2]) {

          freqmutato[2][j] = samplebegin[2] << step;
        }
        if (((freqmutato[3][j] >> step) < sampleend[3] - 1 )  ) {
          freqmutato[3][j] += pich[3][j];
        } else if (loopsample[3]) {

          freqmutato[3][j] = samplebegin[3] << step;
        }
        tempbuffer0 = *(genstartadress[0] + (freqmutato[0][j] >> step));
        bufferbe[2] = (tempbuffer0 * generatorvolume[0][j]) >> 6;

        tempbuffer1 = *(genstartadress[1] + (freqmutato[1][j] >> step));
        bufferbe[0] += (((tempbuffer1 * generatorvolume[1][j]) >> 6) + bufferbe[2]) >> 3;
        //buffers right
        tempbuffer2 = *(genstartadress[2] + (freqmutato[2][j] >> step));
        bufferbe[3] = (tempbuffer2 * generatorvolume[2][j]) >> 6;

        tempbuffer3 = *(genstartadress[3] + (freqmutato[3][j] >> step));
        bufferbe[1] += (((tempbuffer3 * generatorvolume[3][j]) >> 6) + bufferbe[3]) >> 3;
      }
      //freqstep
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7 ;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7 ;
      //choruslfo
      chorusleft();
      chorusright();
      reverbleft();
      reverbright();
      lowpassfilterleft();
      lowpassfilterright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }
  //--------------------5-------------------6---------------------

  if (STRUCTURE_U == 5 && STRUCTURE_L == 6)
  {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      bufferbe[0] = 0;
      bufferbe[1] = 0;
      bufferbe[2] = 0;
      bufferbe[3] = 0;
      for (int j = 0; j < polyphony; j++) {
        //if end of sample
        if (((freqmutato[0][j] >> step) < sampleend[0] - 1 )) {
          freqmutato[0][j] += pich[0][j];
        } else if (loopsample[0]) {

          freqmutato[0][j] = samplebegin[0] << step;
        }
        if (((freqmutato[1][j] >> step) < sampleend[1] - 1 )  ) {
          freqmutato[1][j] += pich[1][j];
        } else if (loopsample[1]) {

          freqmutato[1][j] = samplebegin[1] << step;
        }
        if (((freqmutato[2][j] >> step) < sampleend[2] - 1 ) ) {
          freqmutato[2][j] += pich[2][j];
        } else if (loopsample[2]) {

          freqmutato[2][j] = samplebegin[2] << step;
        }
        if (((freqmutato[3][j] >> step) < sampleend[3] - 1 )  ) {
          freqmutato[3][j] += pich[3][j];
        } else if (loopsample[3]) {

          freqmutato[3][j] = samplebegin[3] << step;
        }
        tempbuffer0 = *(genstartadress[0] + (freqmutato[0][j] >> step));
        bufferbe[2] = (tempbuffer0 * generatorvolume[0][j]) >> 8;

        tempbuffer1 = *(genstartadress[1] + (freqmutato[1][j] >> step));
        bufferbe[0] += (((tempbuffer1 * generatorvolume[1][j]) >> 8) * bufferbe[2]) >> 9;
        //buffers right
        tempbuffer2 = *(genstartadress[2] + (freqmutato[2][j] >> step));
        bufferbe[3] = (tempbuffer2 * generatorvolume[2][j]) >> 6;

        tempbuffer3 = *(genstartadress[3] + (freqmutato[3][j] >> step));
        bufferbe[1] += (((tempbuffer3 * generatorvolume[3][j]) >> 6) + bufferbe[3]) >> 3;
      }
      //freqstep
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7 ;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7 ;
      //choruslfo
      chorusleft();
      chorusright();
      reverbleft();
      reverbright();
      lowpassfilterleft();
      lowpassfilterright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //--------------------6-------------------5---------------------

  if (STRUCTURE_U == 6 && STRUCTURE_L == 5)
  {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      bufferbe[0] = 0;
      bufferbe[1] = 0;
      bufferbe[2] = 0;
      bufferbe[3] = 0;
      for (int j = 0; j < polyphony; j++) {
        //if end of sample
        if (((freqmutato[0][j] >> step) < sampleend[0] - 1 )) {
          freqmutato[0][j] += pich[0][j];
        } else if (loopsample[0]) {

          freqmutato[0][j] = samplebegin[0] << step;
        }
        if (((freqmutato[1][j] >> step) < sampleend[1] - 1 )  ) {
          freqmutato[1][j] += pich[1][j];
        } else if (loopsample[1]) {

          freqmutato[1][j] = samplebegin[1] << step;
        }
        if (((freqmutato[2][j] >> step) < sampleend[2] - 1 ) ) {
          freqmutato[2][j] += pich[2][j];
        } else if (loopsample[2]) {

          freqmutato[2][j] = samplebegin[2] << step;
        }
        if (((freqmutato[3][j] >> step) < sampleend[3] - 1 )  ) {
          freqmutato[3][j] += pich[3][j];
        } else if (loopsample[3]) {

          freqmutato[3][j] = samplebegin[3] << step;
        }
        tempbuffer0 = *(genstartadress[0] + (freqmutato[0][j] >> step));
        bufferbe[2] = (tempbuffer0 * generatorvolume[0][j]) >> 6;

        tempbuffer1 = *(genstartadress[1] + (freqmutato[1][j] >> step));
        bufferbe[0] += (((tempbuffer1 * generatorvolume[1][j]) >> 6) + bufferbe[2]) >> 3;
        //buffers right
        tempbuffer2 = *(genstartadress[2] + (freqmutato[2][j] >> step));
        bufferbe[3] = (tempbuffer2 * generatorvolume[2][j]) >> 8;

        tempbuffer3 = *(genstartadress[3] + (freqmutato[3][j] >> step));
        bufferbe[1] += (((tempbuffer3 * generatorvolume[3][j]) >> 8) * bufferbe[3]) >> 9;
      }
      //freqstep
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7 ;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7 ;
      //choruslfo
      chorusleft();
      chorusright();
      reverbleft();
      reverbright();
      lowpassfilterleft();
      lowpassfilterright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }


  //-----------------6-------------------6------------------
  if (STRUCTURE_U == 6 && STRUCTURE_L == 6)
  {
    for (int i = 0; i < bufferLen / 2 - 1; i += 2) {
      bufferbe[0] = 0;
      bufferbe[1] = 0;
      bufferbe[2] = 0;
      bufferbe[3] = 0;
      for (int j = 0; j < polyphony; j++) {
        //if end of sample
        if (((freqmutato[0][j] >> step) < sampleend[0] - 1 )) {
          freqmutato[0][j] += pich[0][j];
        } else if (loopsample[0]) {
          freqmutato[0][j] = samplebegin[0] << step;
        }
        if (((freqmutato[1][j] >> step) < sampleend[1] - 1 )  ) {
          freqmutato[1][j] += pich[1][j];
        } else if (loopsample[1]) {
          freqmutato[1][j] = samplebegin[1] << step;
        }
        if (((freqmutato[2][j] >> step) < sampleend[2] - 1 ) ) {
          freqmutato[2][j] += pich[2][j];
        } else if (loopsample[2]) {
          freqmutato[2][j] = samplebegin[2] << step;
        }
        if (((freqmutato[3][j] >> step) < sampleend[3] - 1 )  ) {
          freqmutato[3][j] += pich[3][j];
        } else if (loopsample[3]) {
          freqmutato[3][j] = samplebegin[3] << step;
        }
        tempbuffer0 = *(genstartadress[0] + (freqmutato[0][j] >> step));
        bufferbe[2] = (tempbuffer0 * generatorvolume[0][j]) >> 8;

        tempbuffer1 = *(genstartadress[1] + (freqmutato[1][j] >> step));
        bufferbe[0] += (((tempbuffer1 * generatorvolume[1][j]) >> 8) * bufferbe[2]) >> 9;
        //buffers right
        tempbuffer2 = *(genstartadress[2] + (freqmutato[2][j] >> step));
        bufferbe[3] = (tempbuffer2 * generatorvolume[2][j]) >> 8;

        tempbuffer3 = *(genstartadress[3] + (freqmutato[3][j] >> step));
        bufferbe[1] += (((tempbuffer3 * generatorvolume[3][j]) >> 8) * bufferbe[3]) >> 9;
      }
      //freqstep
      parametereqleft();
      bufferbe[0] = (100 * bufferbe[0] - paraeqleftbuffer * eqlevel) >> 7 ;
      parametereqright();
      bufferbe[1] = (100 * bufferbe[1] - paraeqrightbuffer * eqlevel2) >> 7 ;
      //choruslfo
      chorusleft();
      chorusright();
      reverbleft();
      reverbright();
      lowpassfilterleft();
      lowpassfilterright();
      sBuffer[i] = bufferbe[0];
      sBuffer[i + 1] = bufferbe[1];
    }
  }



  //BUFFER WRITE DAC
  i2s_write(I2S_PORT, &sBuffer, bufferLen, &i2s_bytes_write, portMAX_DELAY);
}
