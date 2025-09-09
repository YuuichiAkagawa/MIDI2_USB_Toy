/*
 * MIDI 2.0 UMP Device Example for Raspberry Pi Pico
 * Copyright (c) 2025 Yuuichi Akagawa
 * 
 * Licensed under the MIT License. See LICENSE file for details. 
 *
 * This project was developed with reference to kb-showhey's MIDI2_USB_Dev:
 * https://github.com/kb-showhey/MIDI2_USB_Dev
 *
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "ump_device.h"
#include "bytestreamToUMP.h"
#include "umpToBytestream.h"
#include "umpToMIDI2Protocol.h"
#include "umpMessageCreate.h"
#include "midiCIMessageCreate.h"
#include "midiCIProcessor.h"

//#define DEBUG 1
#define ADC0_PIN       (26)
#define ADC1_PIN       (27)
#define LED_PIN        (25)

static bytestreamToUMP bsToUMP;
static umpToBytestream umptoBS;
static umpToMIDI2Protocol umpToMIDI2;
static midiCIProcessor midiCi;
static bool isMountedusb = false;
static semaphore_t sem;  // Semaphore for inter-core synchronization

// Device Identity for MIDI-CI Discovery and UMP Device Identity Notification
struct DeviceIdentity {
    std::array<uint8_t, 3> manuId;
    std::array<uint8_t, 2> familyId;
    std::array<uint8_t, 2> modelId;
    std::array<uint8_t, 4> version;
    uint32_t localMUID;
};
DeviceIdentity deviceIdentity = {
    {0x00, 0x02, 0x22}, // Manufacturer ID
    {0x7f, 0x00},       // Family ID
    {0x8f, 0x00},       // Model ID
    {0x01, 0x00, 0x00, 0x00}, // Version
    0x00000000 // Local MUID, will be set later
};

#ifdef DEBUG
void UmpTxRxMonior(const char* dir, uint32_t* data, int size)
{
  if (size != 0) {
    printf("%s: ", dir);
    for (int i = 0; i < size; i++) {
      printf("0x%08X ", (unsigned int)data[i]);
    }
    printf("\r\n");
  }
}
#endif

void SendOutSysex(struct MIDICI ciDetails, uint8_t *sysexBuffer , uint16_t length  )
{
    uint8_t umpGroup = 0;
    std::array<uint8_t, 6> sx = {0,0,0,0,0,0};
    uint8_t sxPos = 0;

    for (int i = 0; i < length; i++) {
        sx[sxPos++]=sysexBuffer[i] & 0x7F;
        if(sxPos == 6){
            std::array<uint32_t, 2> ump = UMPMessage::mt3Sysex7(umpGroup, i < 6 ? 1 : i==length-1 ? 3 : 2, 6, sx);
            tud_ump_write(BOARD_TUD_RHPORT, ump.data(), ump.size());
#ifdef DEBUG
            UmpTxRxMonior("UMP TX", ump.data(),  ump.size());
#endif
            sxPos=0;
        }
    }
    if (sxPos) {
        std::array<uint32_t, 2> ump = UMPMessage::mt3Sysex7(umpGroup, length < 7 ? 0 : 3, sxPos, sx);
        tud_ump_write(BOARD_TUD_RHPORT, ump.data(), ump.size());
#ifdef DEBUG
        UmpTxRxMonior("UMP TX", ump.data(),  ump.size());
#endif
    }

}

//MIDI-CI UMID Callback
bool CheckMUIDCallback(uint8_t umpGroup, uint32_t muid){
#ifdef DEBUG
  printf("CheckMUIDCallback(), %02d: %08X\n", umpGroup, muid);
#endif
  return (deviceIdentity.localMUID==muid);
}

//MIDI-CI Discovery Callback
void MidiCiDiscoveryCallback(struct MIDICI ciDetails, std::array<uint8_t, 3> manuId, std::array<uint8_t, 2> srcFamilyId,
                   std::array<uint8_t, 2> srcModelId, std::array<uint8_t, 4> srcVersion, uint8_t srcCiSupport,
                   uint16_t srcMaxSysex, uint8_t outputPathId
){
  //All MIDI-CI Devices shall reply to a Discovery message
  //printf("Received Discover on Group %d remote MUID: %d\n", ciDetails.umpGroup, ciDetails.remoteMUID);
  uint8_t sysexBuffer[64];
  uint8_t midiCIVer = 2;    // MIDI-CI Message Version/Format 1.2
  uint8_t ciSupport = 0x00; // no MIDI-CI support
  uint32_t sysExMax = 64;
  uint8_t fbIdx = 0; // Function Block Index

  #ifdef DEBUG
  printf("MidiCiDiscoveryCallback() srcMUID:%08X\n", ciDetails.remoteMUID);
#endif
#if 1
  // For Discovery Reply
  int len = CIMessage::sendDiscoveryReply(sysexBuffer, midiCIVer, deviceIdentity.localMUID, ciDetails.remoteMUID,
                             deviceIdentity.manuId, deviceIdentity.familyId , deviceIdentity.modelId,
                             deviceIdentity.version, ciSupport,
                             sysExMax, outputPathId, fbIdx
    );
#else
  // For NAK
  uint8_t ackNakDetails[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  uint16_t len = CIMessage::sendNAK(sysexBuffer, midiCIVer, deviceIdentity.localMUID, ciDetails.remoteMUID, 0x7f,
                            0x70, 0x01, 0x00, ackNakDetails, 0, NULL);
#endif

#ifdef DEBUG
  printf("sysexBuffer: ");
  for(int i=0; i<len; i++){
    printf("%02X ", sysexBuffer[i]);
  }
  printf("\n");
#endif
  SendOutSysex(ciDetails, sysexBuffer ,len );
}

void EndpointInfoNotification()
{
  uint8_t state = 1;
  uint8_t numOfFuncBlock = 1;
  bool midi2 = true;
  bool midi1 = false;
  bool rxjr = false;
  bool txjr = false;

  std::array<uint32_t, 4> mess = UMPMessage::mtFMidiEndpointInfoNotify( (state<<7) | numOfFuncBlock , midi2 , midi1, rxjr , txjr);
  tud_ump_write(BOARD_TUD_RHPORT, mess.data(), mess.size());
#ifdef DEBUG
  UmpTxRxMonior("UMP TX", mess.data(),  mess.size());
#endif
}

void DeviceIdentityNotification()
{
  std::array<uint32_t, 4> mess = UMPMessage::mtFMidiEndpointDeviceInfoNotify(deviceIdentity.manuId, deviceIdentity.familyId, deviceIdentity.modelId, deviceIdentity.version);
  tud_ump_write(BOARD_TUD_RHPORT, mess.data(), mess.size());
}

void EndpointNameNotification()
{
  uint8_t text[] = "RPiPicoMIDI2.0";
  uint8_t offset = 0;
  uint8_t textLen = sizeof(text) - 1; // Exclude null terminator

  std::array<uint32_t, 4> mess = UMPMessage::mtFMidiEndpointTextNotify(MIDIENDPOINT_NAME_NOTIFICATION, offset, text, textLen);
  tud_ump_write(BOARD_TUD_RHPORT, mess.data(), mess.size());
}

#define PRODUCT_INSTANCE_ID_LENGTH 32
void ProductInstanceIdNotification()
{
  uint16_t _desc_str[PRODUCT_INSTANCE_ID_LENGTH];
  uint8_t text[PRODUCT_INSTANCE_ID_LENGTH]; 
  uint8_t offset = 0;
  uint8_t textLen = (uint8_t)board_usb_get_serial(_desc_str, PRODUCT_INSTANCE_ID_LENGTH);
  // Convert UTF-16LE to ASCII
  for (uint8_t i = 0; i < textLen; i++) {
    text[i] = (uint8_t)(_desc_str[i] & 0xFF); // Take only the LSB
  }

  while(offset < textLen){
    std::array<uint32_t, 4> mess = UMPMessage::mtFMidiEndpointTextNotify(MIDIENDPOINT_PRODID_NOTIFICATION, offset, text, textLen);
    tud_ump_write(BOARD_TUD_RHPORT, mess.data(), mess.size());
    offset += 14;
  }
}

void FunctionBlockInfoNotification()
{
  uint8_t fbIdx = 0;
  bool active = true;
  uint8_t direction = 2;//bidirectional
  bool sender = true;
  bool recv = false;
  uint8_t firstGroup = 0;
  uint8_t groupLength = 1;
  uint8_t midiCISupport = 0;
  uint8_t isMIDI1 = 0;
  uint8_t maxS8Streams = 1;

  std::array<uint32_t, 4> mess = UMPMessage::mtFFunctionBlockInfoNotify(fbIdx, active, direction, sender, recv, firstGroup, groupLength, midiCISupport, isMIDI1, maxS8Streams);
  tud_ump_write(BOARD_TUD_RHPORT, mess.data(), mess.size());
#ifdef DEBUG
  UmpTxRxMonior("UMP TX", mess.data(),  mess.size());
#endif
}

void FunctionBlockNameNotification()
{
  uint8_t fbIdx = 0;
  uint8_t offset = 0;
  uint8_t text[] = "RPiPicoMIDI2";
  uint8_t textLen = sizeof(text) - 1; // Exclude null terminator

  std::array<uint32_t, 4> mess = UMPMessage::mtFFunctionBlockNameNotify(fbIdx, offset, text, textLen);
  tud_ump_write(BOARD_TUD_RHPORT, mess.data(), mess.size());
}

void StreamConfigurationNotification()
{
  uint8_t protocol = 2; // MIDI 2.0
  bool jrrx = false;
  bool jrtx = false;

  std::array<uint32_t, 4> mess = UMPMessage::mtFNotifyProtocol(protocol, jrrx, jrtx);
  tud_ump_write(BOARD_TUD_RHPORT, mess.data(), mess.size());
#ifdef DEBUG
  UmpTxRxMonior("UMP TX", mess.data(),  mess.size());
#endif
}

// 
void ProcessUmpMessage(uint32_t* UMP, int size)
{
#ifdef DEBUG
  printf("ProcessUmpMessage: size:%d\n", size);
  UmpTxRxMonior("UMP RX", UMP, size);
#endif
  int umpindex = 0;
  while(umpindex < size){
    // Check if the first UMP message is an Endpoint Discovery message
    uint8_t messageType = ((UMP[umpindex] >> 28) & 0xF);
#ifdef DEBUG
    printf("MT: %X\n", messageType);
#endif
    if(messageType == 0xF){  // UMP Stream Message
      uint16_t status = (UMP[umpindex] >> 16) & 0x3FF;
    
      //Endpoint Discovery
      if( status == MIDIENDPOINT ){
        uint8_t filter = UMP[umpindex+1] & 0x1F;
#ifdef DEBUG
        printf("Endpoint Discovery Status:%03X Filter:%02X\n", status, filter);
#endif
        if( filter & 0x01 ) { //shall
          //Endpoint Info Notification
          EndpointInfoNotification();
        }
        if ( filter & 0x02 ){ //may
          //Device Identity Notification
          DeviceIdentityNotification();
        }
        if ( filter & 0x04 ){ //may
          //Endpoint Name Notification
          EndpointNameNotification();
        }
        if( filter & 0x08 ){ //may
          //Product Instance Id Notification
          // Note: Linux is working with this, but macOS and Windows 11 is not.
          //ProductInstanceIdNotification();
        }
        if( filter & 0x10 ){ //shall
          //Stream Configuration Notification
          StreamConfigurationNotification();
        }
      //Function Block Discovery
      }else if ( status == FUNCTIONBLOCK ){
        uint8_t filter = UMP[umpindex] & 0x03;
#ifdef DEBUG
        printf("Function Block Discovery Status:%03X Filter:%02X\n", status, filter);
#endif
        if( filter & 0x01 ) { //shall
          //Function Block Info Notification
          FunctionBlockInfoNotification();
        }
        if( filter & 0x02 ) { //may
          //Function Block Name Notification
          FunctionBlockNameNotification();
        }
      }else if ( status == MIDIENDPOINT_PROTOCOL_REQUEST ){
        StreamConfigurationNotification();
      }else{
      }
      umpindex += 4;  // move to next UMP message
    }
    else if( messageType == 0x3 ){ // MIDI 1.0 SysEx
#ifdef DEBUG
      printf("SysEx: %X %X\n", messageType, (UMP[umpindex] >> 16) & 0xff);
#endif
      uint8_t mt = messageType;
      umptoBS.resetBuffer();
      midiCi.startSysex7(0, 0x7e);
      while(mt == 0x03){
        if( ((UMP[umpindex] >> 16) & 0x0f) > 0) { // check Sysex bytes
          for(int i=0; i<2; i++){
            umptoBS.UMPStreamParse(UMP[umpindex]);  //UMPStreamParseはsysex7を変換する際にF0/F7を付与する
            while(umptoBS.availableBS()){
              uint8_t sx = umptoBS.readBS();
              //printf("BS:%02X ", sx);
              if(sx < 0xf0) midiCi.processMIDICI(sx); //midiCIProcessor::processMIDICI()はF0/F7を除外しないと動作しない。
            }
            umpindex++;
          }
        }else{
          umpindex += 2;
        }
        mt = (UMP[umpindex] >> 28) & 0xF;
      }
      midiCi.endSysex7();
    }
  }
}

void tud_ump_rx_cb(uint8_t itf)
{
  gpio_put(LED_PIN, 1);
  // Read UMP data from the interface
  uint32_t ump[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int size = tud_ump_n_available(itf);

  if(size > 0){
    gpio_put(LED_PIN, 1);
    tud_ump_read(itf, ump, size);
    ProcessUmpMessage(ump, size);
    gpio_put(LED_PIN, 0);
  }
}

void tud_mount_cb()
{
  isMountedusb = true;
}
void tud_unmount_cb()
{
  isMountedusb = false;
}

// core1
// Read ADC value and send it as UMP CC message
void core1_entry() {
  uint16_t adcValue[2];
  uint16_t lastADCvalue[2];
  uint8_t ccNo[2] = {30, 31};
  adc_select_input(0);
  lastADCvalue[0]=(adc_read() >> 3);
  adc_select_input(1);
  lastADCvalue[1]=(adc_read() >> 3);

  while (true) {
    sem_acquire_blocking(&sem);
    bool isMounted = tud_ump_n_mounted(BOARD_TUD_RHPORT);
    sem_release(&sem);
    if(!isMounted){
      sleep_ms(100);
      continue; // UMP interface is not mounted
    }
    for(int ch=0; ch<=1; ch++){
      adc_select_input(ch);
      sem_acquire_blocking(&sem);
      adcValue[ch] = (adc_read() >> 3);
      int16_t diff = (int16_t)lastADCvalue[ch]-(int16_t)adcValue[ch]; 
      if( diff > 1 || diff < -1) {
        if( adcValue[ch] <= 1 ) adcValue[ch] = 0;
        // Create UMP message and send it
        std::array<uint32_t, 2> msg = UMPMessage::mt4CC(0, 0, ccNo[ch], (uint32_t)adcValue[ch]<<23);
        tud_ump_write(BOARD_TUD_RHPORT, msg.data(), msg.size());
        lastADCvalue[ch] = adcValue[ch];
      }
      sem_release(&sem);
      sleep_ms(5);
    }
  }
}

// core0
int main()
{
    stdio_init_all();

    // initialize GPIO for Onboard LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    sem_init(&sem, 1, 1);

    // initialize the ADC
    adc_init();
    adc_gpio_init(ADC0_PIN);
    adc_gpio_init(ADC1_PIN);
    adc_select_input(0);

    // initialize the tinyusb stack
    board_init();
    tud_init(BOARD_TUD_RHPORT);

    // launch core1
    multicore_launch_core1(core1_entry);

    // initialize the MIDI-CI
    srand(0xFFFFEFF);
    // MUID FFFFF00 to FFFFFFF are reserved, so for simplicity's sake, Fxxxxxx is excluded.
    deviceIdentity.localMUID = ((uint32_t)rand()) & 0x0EFFFFFF; // 27bits  
    midiCi.setCheckMUID(CheckMUIDCallback);
    midiCi.setRecvDiscovery(MidiCiDiscoveryCallback);

    // main loop
    while (true) {
        sem_acquire_blocking(&sem);
        tud_task(); // tinyusb device task
        sem_release(&sem);
    }
}
