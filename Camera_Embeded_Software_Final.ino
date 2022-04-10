#include <Servo.h>
Servo SHTR_SRV;

// Clocking stuff
#define V1_CLK 2
#define V1_CLK_HIGH (GPIO9_DR |= (1<<4))
#define V1_CLK_LOW (GPIO9_DR &= ~(1<<4))

#define V2_CLK 3
#define V2_CLK_HIGH (GPIO9_DR |= (1<<5))
#define V2_CLK_LOW (GPIO9_DR &= ~(1<<5))

#define H_CLK 4
#define H_CLK_HIGH (GPIO9_DR |= (1<<6))
#define H_CLK_LOW (GPIO9_DR &= ~(1<<6))

#define RST_CTRL 5
#define RST_CTRL_HIGH (GPIO9_DR |= (1<<8))
#define RST_CTRL_LOW (GPIO9_DR &= ~(1<<8))

#define CDS2 8
#define CDS2_HIGH (GPIO7_DR |= (1<<16))
#define CDS2_LOW (GPIO7_DR &= ~(1<<16))

#define CDS1 9
#define CDS1_HIGH (GPIO7_DR |= (1<<11))
#define CDS1_LOW (GPIO7_DR &= ~(1<<11))

#define ADCLK 7
#define ADCLK_HIGH (GPIO7_DR |= (1<<17))
#define ADCLK_LOW (GPIO7_DR &= ~(1<<17))

// DAC Stuff
#define DAC_CS 10
#define SPI_MOSI 11
#define SPI_CLK 12
#define AD_CS 6

/************** SEPARATE GPIO PINS FROM PORT.BIT ************/
#define GPIO6_DATA    (*(volatile uint32_t *)0x42000000)  /****** MAGIC ****/

#define A0_TX3 14          // bit 18
#define A0_TX3_BM 18
#define A1_RX3 15          // bit 19
#define A1_RX3_BM  19
#define A2_RX4_SCL1 16     // bit 23
#define A2_RX4_SCL1_BM  23
#define A3_TX4_SDA1 17     // bit 22
#define A3_TX4_SDA1_BM  22
#define A4_SDA0 18         // bit 17
#define A4_SDA0_BM  17
#define A5_SCL0 19         // bit 16
#define A5_SCL0_BM  16
#define A6_TX5 20          // bit 26
#define A6_TX5_BM  26
#define A7_RX5 21          // bit 27
#define A7_RX5_BM  27

/********************** missing macro for temperature sensor **********/
// temperature sensor
#define TEMP_GPIO    23

// tokenizer variables
char msg[32];
char *tokens[10];
int stok = 0;
char tmp[32];

int close_shutter=0;
int open_shutter=0;
// camera commands
const int xframe=796;
const int yframe=520;
int xsec=0;
int xmsec=0;
int xsize=0;
int ysize=0;
int xoffset=0;
int yoffset=0;
int xbin=0;
int ybin=0;
double temp=0;
char *ver = "CSC214S20.1.0";
int test = 42;

/************* need to declare prototypes for procedure calls *******/
double readTemp();
  
void setup() {
  Serial.begin(115200);
  pinMode(V1_CLK,OUTPUT);
  pinMode(V2_CLK,OUTPUT);
  pinMode(H_CLK,OUTPUT);
  pinMode(RST_CTRL,OUTPUT);
  pinMode(CDS2,OUTPUT);
  pinMode(CDS1,OUTPUT);
  pinMode(ADCLK,OUTPUT);
  SHTR_SRV.attach(0);
  SHTR_SRV.write(close_shutter);

  // DAC Stuff
/********** USE GPIO PINS FOR SETTING INPUT/OUTPUT **********/
  pinMode(AD_CS,OUTPUT);
  pinMode(DAC_CS,OUTPUT);
  pinMode(SPI_MOSI,OUTPUT);
  pinMode(SPI_CLK,OUTPUT);
  pinMode(A0_TX3,INPUT);
  pinMode(A1_RX3,INPUT);
  pinMode(A2_RX4_SCL1,INPUT);
  pinMode(A3_TX4_SDA1,INPUT);
  pinMode(A4_SDA0,INPUT);
  pinMode(A5_SCL0,INPUT);
  pinMode(A6_TX5,INPUT);
  pinMode(A7_RX5,INPUT);

  pinMode(AD_CS, OUTPUT);
  pinMode(DAC_CS, OUTPUT);

  /********** DAC INITIALIZATION  ???????? **********/

   /********** ADC INITIALIZATION ???????? **********/ 
  
//  // may be embeded in code. Test without this code with nano second timing on scope
//  ARM_DEMCR |= ARM_DEMCR_TRCENA; // Nanosecond timing magic code, 
//  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; // Nanosecond timing magic code
  
} // END SETUP

void loop() {
  
  char c;
  char *tok;
  char ntok = 0;
  char *cmd;
  char *prm;
 
  while(1) {
    int ind=0;
    while(Serial.available() == 0); // waits for incoming command
    while ((c = Serial.read()) != '\n') {
      msg[ind++] = tolower(c);
      delay(1); 
    }
    msg[ind]  = '\0';
    ntok = 0;
    tok = strtok(msg, " ");
    while (tok != NULL) {
      tokens[ntok++] = tok;
      tok = strtok(NULL, " ");
    }
    cmd = tokens[0];
      
    if(strcmp(cmd, "xframe?") == 0)  {
      Serial.print(xframe);
      Serial.print(" OK\n");
    }
    
    else if(strcmp(cmd, "yframe?") == 0)  {
      Serial.print(yframe); 
      Serial.print(" OK\n");
    }

/********************** what?????????????*/
/* you don't set the version - it is compiled into the code */
//    // testing parsing commands
//    else if(strcmp(cmd, "*ver") == 0) {
//      prm = tokens[1];
//      *ver = atoi(prm);
//      Serial.print(" OK\n");
//    }

/* **************** duplicate ??????????*/
//    else if(strcmp(cmd, "yframe?") == 0)  {
//      Serial.print( yframe );
//      Serial.print(" OK\n");
//    }

/* you don't set the version - it is compiled into the code */
//    else if(strcmp(cmd, "*ver") == 0) {
//      prm = tokens[1];
//      *ver = atoi(prm);
//      Serial.print(" OK\n");
//    }
        
    else if(strcmp(cmd,"capture") == 0)  {
      V1_CLK_LOW;
      V2_CLK_LOW;
      SHTR_SRV.write(close_shutter);
      for (int y=0; y<yframe;++y) {
        vertical_shift();
        horizontal_flush();
      }
      SHTR_SRV.write(open_shutter);
      delay(1000*xsec+xmsec); // designated exposure time
      SHTR_SRV.write(close_shutter);
      noInterrupts();
      int blen=2*xframe;
      unsigned char *buf=new unsigned char[blen];
      for (int y=0; y<yframe; ++y)  {
        vertical_shift();
        horizontal_shift(buf);
      }
      delete[] buf;
/********************* NEED A SPACE BEFORE 'OK' **************/
      Serial.print("OK\n"); // returns after caputre complete
    }
      
    else if(strcmp(cmd, "xoffset") == 0)
    {
      prm = tokens[1];
      xoffset = atoi(prm);
      Serial.print(" OK\n");
      //Serial.print("Setting XOFFSET to "); Serial.print(xoffset); Serial.write("\n");
    }
    
    else if(strcmp(cmd, "xoffset?") == 0)
    {
      Serial.print( xoffset ); Serial.print(" OK\n");
    }
    
    else if(strcmp(cmd, "yoffset") == 0)
    {
      prm = tokens[1];
      yoffset = atoi(prm);
      Serial.print(" OK\n");
    }
    
    else if(strcmp(cmd, "yoffset?") == 0)
    {
      Serial.print( yoffset ); Serial.print(" OK\n");
    }   
    
    else if(strcmp(cmd, "xbin") == 0) // dont worry about binning or setting area of interest, assume full frame
    {
      prm = tokens[1];
      xbin = atoi(prm);
      Serial.print(" OK\n");
    }
    
    else if(strcmp(cmd, "xbin?") == 0)
    {
      Serial.print( xbin ); Serial.print(" OK\n");
    }     
    
    else if(strcmp(cmd, "ybin") == 0)
    {
      prm = tokens[1];
      ybin = atoi(prm);
      Serial.print(" OK\n");
    }
    
    else if(strcmp(cmd, "ybin?") == 0)
    {
      Serial.print( ybin ); Serial.print(" OK\n");
    }

    // testing parsing commands
    else if(strcmp(cmd, "xsec") == 0)
    {
      prm = tokens[1];
      xsec = atoi(prm);
      Serial.print(" OK\n");
    }
    
    else if(strcmp(cmd, "xsec?") == 0)
    {
      Serial.print( xsec ); 
      Serial.print(" OK\n");
    }
    
    else if(strcmp(cmd, "xmsec") == 0)
    {
      prm = tokens[1];
      xmsec = atoi(prm);
      Serial.print(" OK\n");
    }
    
    else if(strcmp(cmd, "xmsec?") == 0) {
      Serial.print( xmsec ); 
      Serial.print(" OK\n");
    }

/**************  cant set the temperature, just read it *******/
//    else if(strcmp(cmd, "temp") == 0) {
//      prm = tokens[1];
//      temp = atoi(prm);
//      Serial.print(" OK\n");
//    }
    
    else if(strcmp(cmd, "temp?") == 0)  {
/***** have to read the temperature sensor like in the lab **/
      temp = readTemp();
      Serial.print( temp ); 
      Serial.print(" OK\n");
    }

    else if(strcmp(cmd, "test?") == 0)  {
      Serial.print(test); 
      Serial.print(" OK\n");
    }
        
    else if(strcmp(cmd, "open") == 0) {
      SHTR_SRV.write(open_shutter);
      Serial.print("OK\n");      
    }
    
    else if(strcmp(cmd, "close") == 0)  {
      SHTR_SRV.write(close_shutter);
      Serial.print("OK\n");    
    }
    
    else if(strcmp(cmd, "flush") == 0)
/* ****************** call flush() routine *************/
    {
      SHTR_SRV.write(close_shutter);
      for (int y=0; y<yframe;++y) 
      {
        vertical_shift();
        horizontal_flush();
      }
    }
    
    return 0;
  } // End while(1) loop
} // END LOOP

void vertical_shift() {
  CDS1_LOW;
  CDS2_LOW;
  ADCLK_HIGH;
  H_CLK_HIGH;
  delayMicroseconds(4);
  V1_CLK_HIGH;
  V2_CLK_LOW;
  delayMicroseconds(4);
  V1_CLK_LOW;
  V2_CLK_HIGH;
  delayMicroseconds(4);
  V1_CLK_HIGH;
  V2_CLK_LOW;
  delayMicroseconds(4);
  V1_CLK_LOW;
  delayMicroseconds(1);
}

void horizontal_flush() {
  for (int k=0; k<xframe; ++k)  {
    RST_CTRL_HIGH;
    H_CLK_LOW;
    delayNanoseconds(150);
    RST_CTRL_LOW;
    delayNanoseconds(150);
    RST_CTRL_HIGH;
    delayNanoseconds(200);
  }
}

void horizontal_shift(unsigned char* buf) {
  unsigned char hbyte;
  unsigned char lbyte;

  int blen=2*xframe;  // byte length of scanline
  int ind=0;
  for (int k=-4; k<xframe;++k)  {
    RST_CTRL_HIGH;
    H_CLK_LOW;
    delayNanoseconds(150);
    RST_CTRL_LOW;
    delayNanoseconds(200);
    read9826();
    CDS1_HIGH;  // intilize correlated double sampling
    ADCLK_LOW;
    delayNanoseconds(300);
    CDS1_LOW;
    delayNanoseconds(150);
    H_CLK_HIGH;  // charge transfer
    delayNanoseconds(200);
    read9826();
    CDS2_HIGH;
    delayNanoseconds(150);
    ADCLK_HIGH;
    delayNanoseconds(150);
    CDS2_LOW;
    delayNanoseconds(300);
    if(k>=0)  {
      buf[ind++]=hbyte;
      buf[ind++]=lbyte;
    }
    else
      delayNanoseconds(250);
    Serial.write(buf,blen);
    Serial.write("OK\n");

/************ THIS GOES AT END OF FRAME CAPTURE, NOT END-OF-LINE **********/  
//    interrupts(); // turn interruputs back on, from arduino library
  }
}

void config9826(int cdsMode)  {
  ushort cval = 0;
  ushort adr = 0b000; // for config, addr = 0b000
  cval |= 128; //(1<<7); // 4v range
  cval |= 64; //(1<<6); // internal ref enabled
  if (cdsMode != 0)
  cval |= 16; //(1<<4); // enable CDS mode
  cval |= 8; //(1<<3); // clamp bias 4v (?)
  delay(3000);
  // single channel enabled, bit 5 = 0
  // Power down disabled, bit 2 = 0
  // 2 byte output enabled, bit 0 = 0
  // bit 1 & bit 8 = 0
  // for writing, bit 15 = 0
  set9826(adr, cval);
  // set the MUX
  // MUX defaults are appropriate for mono reading 
}

void set9826(ushort addr, ushort val) {
  val |= (addr << 12);
  //displayBits("A/DConfig: ", val);
  digitalWrite(SPI_CLK, LOW);
  digitalWrite(AD_CS, LOW);
  for (int k = 15; k >= 0; --k)
  {
  long b = (val >> k) & 1;
  //Serial.print(b);
  digitalWrite(SPI_MOSI, (b == 0) ? LOW : HIGH);
  digitalWrite(SPI_CLK, HIGH);
  digitalWrite(SPI_CLK, LOW);
  }
  digitalWrite(AD_CS, HIGH);
}

void dacOn()  {
  dacOutput(1,2.00);  // HH=6 -> 6/3 = 2.00
  dacOutput(2,1.00);  // RH=3 -> 3/3 = 1.00
  dacOutput(3,0.17);  // VH=0.5 -> 0.5/3 = 0.17
  dacOutput(4,2.67);  // VL=-8 -> 8/3 = 2.67
  dacOutput(5,1.33);  // HL=-4 -> 4/3 = 1.33
  dacOutput(6,0.67);  // RL=-2 -> 2/3 = 0.67
  dacOutput(7,2.33);  // GRD=7 -> 7/3 = 2.33
  dacOutput(8,3.33);  // VRD=10 -> 10/3 = 3.33
  dacOutput(9,1.00);  // VOG=3 -> 3/3 = 1.00
  dacOutput(10,0.23); // VSS=0.7 -> .7/3 = 0.23
}

void dacOutput(int chn, double volt)  { // program BH2221FV
  // this reverses the bits of the channel ID
  unsigned short c = ((chn & 1) << 3)
  | ((chn & 2) << 1)
  | ((chn & 4) >> 1)
  | ((chn & 8) >> 3);
  // this converts the desired voltage to the
  // appropriate value for the dac
  double vlt = ((double)volt / 5.0) * 256.0;
  unsigned short v = (unsigned short)(vlt);
  unsigned short t = (unsigned short)((c << 8) | v);
  digitalWrite(DAC_CS, LOW);
  for (long i = 15; i >= 0; i--)
  {
  long b = (t >> i) & 1;
  digitalWrite(SPI_MOSI, b);
  digitalWrite(SPI_CLK, HIGH);
  digitalWrite(SPI_CLK, LOW);
  }
  digitalWrite(DAC_CS, HIGH);
}

void dacsClear()  {
  for (int k = 0; k < 12; ++k)
  dacOutput(k, 0);
}

/************** USE BITMASKS FOR READING DATA FROM PORT ******/
inline unsigned char read9826() {
  register uint32_t data = GPIO6_DATA;
/***** NOPE *****  register uint32_t data = AD_CS;  **/
  register uint32_t val = (data & A0_TX3) >> 18
                        | (data & A1_RX3) >> (19-1)
                        | (data & A2_RX4_SCL1) >> (23-2)
                        | (data & A3_TX4_SDA1) >> (22-3)
                        | (data & A4_SDA0) >> (17-4)
                        | (data & A5_SCL0) >> (16-5)
                        | (data & A6_TX5) >> (26-6)
                        | (data & A7_RX5) >> (27-7);
  return (unsigned char)(val & 0xffff);
} 

/*************** NEED TEMP READING **************/
double readTemp()
{
  int tmp = 0;
  for (int i = 0; i < 15; ++i)
    tmp += analogRead(TEMP_GPIO);
  tmp /= 15;

  //  double dtmp = tmp / 1023.0 * 3.3; // convert analog value to a voltage
  //  dtmp -= 0.5;                      // offset the voltage by the V@0dg = 500mv
  //  dtmp /= .01;                      // convert to deg (10mv per degree)

  double degC = ((double)tmp / 1023 * 3.3 - 0.5) / 0.01;
  double degF = degC * 9.0 / 5.0 + 32.0;

  return degF;
}
