//Include SPI library
#include <SPI.h>

//SPI timout limit
#define timoutLimit 100

//SPI commands used by the AMT20
#define AMT_NOP 0x00    //no operation
#define AMT_READ 0x10   //read position
#define AMT_ZERO 0x70   //set zero point

//MEGA SPI Ports
#define SPI_SCK 52
#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_CSB 48

// Function Prototype
uint8_t SPIWrite(uint8_t sendByte);

//setting up speed, data order, and data mode
SPISettings Settings(500000, MSBFIRST, SPI_MODE0);

void setup() {
  //Initialize the UART serial connection
  Serial.begin(9600);

  //Set I/O mode of all SPI pins.
  pinMode(SPI_SCK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_CSB, OUTPUT);


  
  //Explicitly set the beginning state the CS pin
  digitalWrite(SPI_CSB, HIGH);
  //initializes spi
  SPI.begin();
}

void loop() {
  uint8_t data;               //this will hold returned data from the AMT20
  uint8_t timeoutCounter;     //timeout incrementer
  uint16_t currentPosition;   //this 16 bit variable will hold our 12-bit position

  while(true) {
    //reset the timoutCounter;
    timeoutCounter = 0;
    
    //send the rd_pos command to have the AMT20 begin obtaining the current position
    data = SPIWrite(AMT_READ);
  
    //we need to send nop commands while the encoder processes the current position. We
    //will keep sending them until the AMT20 echos the rd_pos command, or our timeout is reached.
    while ((data != AMT_READ) && (timeoutCounter < timoutLimit)) {
      Serial.println("Hello");
      data = SPIWrite(AMT_NOP);
      timeoutCounter++;
    }
  
  
    if (timeoutCounter < timoutLimit) {
      //We received the rd_pos echo which means the next two bytes are the current encoder position.
      //Since the AMT20 is a 12 bit encoder we will throw away the upper 4 bits by masking.
  
      //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
      //shift it left 8 bits to make room for the lower byte.
      currentPosition = (SPIWrite(AMT_NOP)& 0x0F) << 8;
  
      //OR the next byte with the current position
      currentPosition |= SPIWrite(AMT_NOP);
    } else {
      //This means we had a problem with the encoder, most likely a lost connection. For our
      //purposes we will alert the user via the serial connection, and then stay here forever.
  
      Serial.write("Error obtaining position.\n");
      Serial.write("Reset Arduino to restart program.\n");
      
      while(true);
    }

    Serial.write("Current position: ");
    Serial.print(currentPosition, DEC); //current position in decimal
    Serial.write("\t 0x");
    Serial.print(currentPosition, HEX); //current position in hexidecimal
    Serial.write("\t 0b");
    Serial.print(currentPosition, BIN); //current position in binary
    Serial.write("\n");
    
    
    //Since we are displaying our position over the serial monitor we don't need updates that fast
    delay(250);
  }
}

//We will use this function to handle transmitting SPI commands in order to keep our code clear and concise.
//It will return the byte received from SPI.transfer()
uint8_t SPIWrite(uint8_t sendByte) {
  //holder for the received over SPI
  uint8_t data;

  //Initialize SPI with a clock rate of 500kHz, MSB First, and SPI Mode 0
  SPI.beginTransaction(Settings);

  //the AMT20 requires the release of the CS line after each byte
  digitalWrite(SPI_CSB, LOW);
  data = SPI.transfer(sendByte);
  Serial.print("Hi");
  Serial.print(data);
  digitalWrite(SPI_CSB, HIGH);
  
  Serial.println("Bye");
  SPI.endTransaction();

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  delay(1);
  
  return data;
}
