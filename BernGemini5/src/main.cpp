#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include "driver/i2s.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <memory.h>


#define I2S_WS 41      // Word select (LRCLK)
#define I2S_SD 2      // Data (DOUT)
#define I2S_SCK 42      // Bit clock (BCLK)

// I2S pins for MAX98357 amplifier
#define I2S_BCLK 37 //black
#define I2S_LRC 36 //grey
#define I2S_DOUT 38 //white


#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE 12000
#define I2S_CHANNEL_NUM 1
#define I2S_SAMPLE_BITS 16
#define I2S_READ_LEN (16*1024)
#define RECORD_TIME 40 // Seconds
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)
//#define RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)
//#define WRITE_THRESHOLD 8192  // Write in larger chunks

File file;
const char filename[] = "/record.wav";
const int headerSize = 44;

const int buttonPin = 19;    //GPIO for button
unsigned long lastButtonPressTime = 0;
uint8_t buttonpress = 0;
volatile bool isWIFIConnected = false;
//volatile bool isRecording = true;



void deleteFile(fs::FS &fs, const char *path) ;
void wavHeader(byte *header, int wavSize);
void i2sInit_mic();
void i2s_adc(void *arg);
void SPIFFSInit();
void wifiConnect();
void i2sInit_dac();
void playWAV(const char *path);
void listSPIFFS();
void i2s_adc_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len);




void setup() 
{
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP); // Use pull-up resistor
  wifiConnect();// Connect to WiFi
  i2sInit_dac();
  playWAV("/gdm.wav"); 
}

void loop() {
  //playWAV("/hellokid.wav");
  //SPIFFSInit();
  //i2sInit_mic();
  //xTaskCreate(i2s_adc, "i2s_adc", 1024 * 8, NULL, 5, NULL); 
}

// Initialize SPIFFS
void SPIFFSInit() {
  /*
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed!");
    while (1) yield();
  }
  */
  //SPIFFS.format();
  
  //deleteFile(SPIFFS, "/kidkid.wav");
  deleteFile(SPIFFS, filename);
  //SPIFFS.remove(filename);
  delay(100);
  file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    //Serial.println("File is not available!");
    return;
  }

  byte header[headerSize];
  wavHeader(header, FLASH_RECORD_SIZE);
  if (file.write(header, headerSize) == headerSize) {
    Serial.println("WAV header successfully written.");
  } else {
    Serial.println("Failed to write WAV header!");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path)) {
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}

// Initialize I2S
void i2sInit_mic() {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = I2S_SAMPLE_RATE,
      .bits_per_sample = (i2s_bits_per_sample_t)I2S_SAMPLE_BITS,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_MSB,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 1024,
      .use_apll = 1,
      .tx_desc_auto_clear = true
      };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  i2s_set_pin(I2S_PORT, &pin_config);
}

void i2sInit_dac() {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = I2S_SAMPLE_RATE,
      .bits_per_sample = (i2s_bits_per_sample_t)I2S_SAMPLE_BITS,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_MSB,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 1024,
      .use_apll = 1,
      .tx_desc_auto_clear = true
      };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK,
      .ws_io_num = I2S_LRC,
      .data_out_num = I2S_DOUT,
      .data_in_num = -1};

  i2s_set_pin(I2S_PORT, &pin_config);
}



//Converts raw 12-bit ADC values from the microphone to 16-bit PCM audio
void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len) {
    uint32_t j = 0;//Tracks the index for the destination buffer.
    uint32_t dac_value = 0;//Temporary variable for scaled ADC values.
    for (int i = 0; i < len; i += 2) {//Processes every 2 bytes of the input buffer (s_buff), as the 12-bit ADC value is stored across two bytes.
        dac_value = ((((uint16_t)(s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 2048;
    }
  
}

void i2s_adc(void *arg) {
  size_t bytes_read;
  uint8_t flag = 0;
  uint8_t *i2s_read_buff = (uint8_t *)calloc(I2S_READ_LEN, sizeof(char));
  uint8_t *flash_write_buff = (uint8_t *)calloc(I2S_READ_LEN, sizeof(char));
  uint8_t voice_detected = 0;

  if (!i2s_read_buff || !flash_write_buff) {
    //Serial.println("Failed to allocate memory for buffers!");
    vTaskDelete(NULL);
    return;
  }

  Serial.println("*** Recording Start ***");

  // Silence detection variables
  const int SILENCE_THRESHOLD = 20;         // Set based on analysis of noise vs. speech levels
  const int SILENCE_DURATION_MS = 10000;   // 10 seconds for sleep mode
  const int SILENCE_DURATION_MS_RECORD = 3000; // 3 seconds for stopping recording
  unsigned long silence_start_time = millis(); // Start timer
  int flash_wr_size = 0;
  i2s_read(I2S_PORT, (void *)i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);

  while (flash_wr_size < FLASH_RECORD_SIZE) {
    // Read audio data from I2S
    i2s_read(I2S_PORT, (void *)i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
    
    i2s_adc_data_scale(flash_write_buff, (uint8_t *)i2s_read_buff, bytes_read);

    if (bytes_read > 0) {
      // Write audio data to file
      //file.write((const byte *)flash_write_buff, bytes_read);
      //flash_wr_size += bytes_read;
      //Serial.printf("Bytes written: %d, Total: %d/%d\n", bytes_read, flash_wr_size, FLASH_RECORD_SIZE);

      // Check if the buffer is silent
      bool is_silent = true;
      for (size_t i = 0; i < bytes_read; i++) {
        if ((flash_write_buff[i] > SILENCE_THRESHOLD) && (flash_write_buff[i] <= 200)) { // If talking
          is_silent = false;
          silence_start_time = millis(); // Reset silence timer
          voice_detected++;
          break;
        }
        else{

        }
      }

      if (is_silent) {
        unsigned long silence_duration = millis() - silence_start_time;

        for( size_t i = 0; i < bytes_read; i++)
          flash_write_buff[i] = 0;
        Serial.printf("Silence detected for %lu ms\n", silence_duration);
        if ((silence_duration >= SILENCE_DURATION_MS)  && (voice_detected <= 3)) {
          Serial.println("10 seconds of silence detected. Stopping recording...");
          Serial.println("Entering sleep mode...");
          esp_deep_sleep_start();
        } else if ((silence_duration >= SILENCE_DURATION_MS_RECORD) && (voice_detected >= 3)) 
        {
          Serial.println("Recording completed due to prolonged silence.");
          break;
        }
  
      }
      file.write((const byte *)flash_write_buff, bytes_read);
      flash_wr_size += bytes_read;
      Serial.printf("Bytes written: %d, Total: %d/%d\n", bytes_read, flash_wr_size, FLASH_RECORD_SIZE);

    }
  }

  // Close file and cleanup
  file.close();
  free(i2s_read_buff);
  free(flash_write_buff);
  i2s_driver_uninstall(I2S_PORT);

  Serial.println("*** Recording End ***");

  // List recorded files and start playback
  listSPIFFS();
  i2sInit_dac();
  playWAV(filename);
  vTaskDelete(NULL);
}


/*void i2s_adc(void *arg) 
{ 
  int flash_wr_size = 0; size_t bytes_read; size_t bytes_read2; 
  uint8_t *i2s_read_buff = (uint8_t *)calloc(I2S_READ_LEN, sizeof(char)); 
  uint8_t *flash_write_buff = (uint8_t *)calloc(I2S_READ_LEN, sizeof(char));
  
  if (!i2s_read_buff || !flash_write_buff) 
  { 
    Serial.println("Failed to allocate memory for buffers!"); 
    vTaskDelete(NULL); 
    return; } 
    Serial.println("*** Recording Start ***"); 
    i2s_read(I2S_PORT, (void *)i2s_read_buff, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
    
    while (flash_wr_size < FLASH_RECORD_SIZE) 
    { 
      i2s_read(I2S_PORT, (void *)i2s_read_buff,I2S_READ_LEN, &bytes_read, portMAX_DELAY); 
      /*for(int i=0;i<I2S_READ_LEN;i++)
      {
        Serial.printf("%d  ",i2s_read_buff[i]);
      }
      i2s_adc_data_scale(flash_write_buff, (uint8_t *)i2s_read_buff, bytes_read);
      
      /*for(int i=0;i<I2S_READ_LEN;i++)
      {
        Serial.printf("%d  ",flash_write_buff[i]);
      }

      if (bytes_read > 0) {
        file.write((const byte *)flash_write_buff, bytes_read); 
        flash_wr_size += bytes_read;
        Serial.printf("Bytes written: %d, Total: %d/%d\n", bytes_read, flash_wr_size, FLASH_RECORD_SIZE); 
      }

      
    }
   
    file.close(); 
    free(i2s_read_buff); 
    free(flash_write_buff);
    i2s_driver_uninstall(I2S_PORT); 
    Serial.println("*** Recording End ***"); 
    listSPIFFS(); 
    i2sInitTx(); 
    playWAV(filename); 
    vTaskDelete(NULL);
}*/

void listSPIFFS() {
  Serial.println("Listing SPIFFS files:");
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.printf(" %s\t%d\n", file.name(), file.size());
    file = root.openNextFile();
  }
}
// Generate WAV file header
void wavHeader(byte *header, int wavSize) {
  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  unsigned int fileSize = wavSize + headerSize - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 0x10; header[17] = 0x00; header[18] = 0x00; header[19] = 0x00;
  header[20] = 0x01; header[21] = 0x00;
  header[22] = I2S_CHANNEL_NUM;
  header[23] = 0x00;
  header[24] = (byte)(I2S_SAMPLE_RATE & 0xFF);
  header[25] = (byte)((I2S_SAMPLE_RATE >> 8) & 0xFF);
  header[26] = (byte)((I2S_SAMPLE_RATE >> 16) & 0xFF);
  header[27] = (byte)((I2S_SAMPLE_RATE >> 24) & 0xFF);
  unsigned int byteRate = I2S_SAMPLE_RATE * I2S_CHANNEL_NUM * I2S_SAMPLE_BITS / 8;
  header[28] = (byte)(byteRate & 0xFF);
  header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = I2S_CHANNEL_NUM * I2S_SAMPLE_BITS / 8;
  header[33] = 0x00;
  header[34] = I2S_SAMPLE_BITS;
  header[35] = 0x00;
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
}


void playWAV(const char *path) {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed!");
    while (1) yield();
  }
  File audioFile = SPIFFS.open(path, FILE_READ);
  if (!audioFile) {
    Serial.println("Failed to open WAV file for playback!");
    return;
  }

  // Skip the WAV header
  audioFile.seek(headerSize);

  
  size_t bytesRead;
  uint8_t *i2s_write_buff = (uint8_t *)calloc(I2S_READ_LEN, sizeof(uint8_t));
  if (!i2s_write_buff) {
    Serial.println("Failed to allocate memory for playback buffer!");
    audioFile.close();
    return;
  }
  Serial.println("*** Playback Start ***");
  while (audioFile.available()) {
    bytesRead = audioFile.read(i2s_write_buff, I2S_READ_LEN);
    
    size_t bytesWritten;
    i2s_write(I2S_PORT, i2s_write_buff, bytesRead, &bytesWritten, portMAX_DELAY);
    if (bytesWritten != bytesRead) {
      Serial.printf("Mismatch: bytesRead = %d, bytesWritten = %d\n", bytesRead, bytesWritten);
    }
  } 
  
  i2s_driver_uninstall(I2S_PORT);
  free(i2s_write_buff);
  audioFile.close();
  Serial.println("*** Playback End ***");
  SPIFFSInit();
  i2sInit_mic();
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 8, NULL, 5, NULL); 
}

void wifiConnect() {
    Serial.println("Press the button to start recording audio...");

    if (!isWIFIConnected) {
        while (true) {
            if (digitalRead(buttonPin) == LOW) {
                unsigned long currentTime = millis();
                if (currentTime - lastButtonPressTime > 300) {  // Debounce
                    lastButtonPressTime = currentTime;
                    buttonpress++;

                    if (buttonpress == 1) {
                        
                        WiFi.mode(WIFI_STA);
                        WiFi.disconnect(true);
                        delay(1000);
                        const char* ssid = "myssid";
                        const char* password = "mypassword";
                        WiFi.begin(ssid,password);

                        int attempts = 0;
                        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                            delay(500);
                            Serial.print(".");
                            attempts++;
                        }

                        if (WiFi.status() == WL_CONNECTED) {
                            Serial.println("\nConnected to Wi-Fi!");
                            isWIFIConnected = true;
                            buttonpress = 0;
                            break;
                        } else {
                            Serial.println("\nFailed to connect. Check credentials or hotspot settings.");
                        }
                    }
                }
            }
        }
    } else {
        Serial.println("Wi-Fi is already connected.");
    }

    //WiFi.printDiag(Serial);  // Print diagnostic information
}