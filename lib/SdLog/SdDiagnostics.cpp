#include <Arduino.h>
#include <SPI.h>
#include <sd_diskio.h>
#include <ff.h>
#include <diskio.h>
#include <string.h>

namespace {
uint16_t le16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
uint32_t le32(const uint8_t *p) { return uint32_t(le16(p)) | (uint32_t(le16(p + 2)) << 16); }
uint32_t hashSector(const uint8_t *p) {
  uint32_t h = 2166136261u;
  for (unsigned i=0; i<512; ++i) h = (h ^ p[i]) * 16777619u;
  return h;
}
bool readChecked(uint8_t drive, uint32_t lba, uint8_t *data) {
  uint8_t repeat[512];
  if (!sd_read_raw(drive, data, lba)) {
    Serial.printf("SDDIAG: LBA %lu READ FAILED\n", (unsigned long)lba);
    return false;
  }
  Serial.printf("SDDIAG: LBA %lu hash=%08lX signature=%02X %02X first16=",
    (unsigned long)lba, (unsigned long)hashSector(data), data[510], data[511]);
  for (unsigned i=0; i<16; ++i) Serial.printf("%02X ", data[i]);
  Serial.println();
  for (unsigned i=0; i<2; ++i) {
    if (!sd_read_raw(drive, repeat, lba)) {
      Serial.println("SDDIAG: repeat READ FAILED"); return false;
    }
    if (memcmp(data, repeat, 512) != 0) {
      Serial.printf("SDDIAG: repeat MISMATCH hash=%08lX\n", (unsigned long)hashSector(repeat));
      return false;
    }
  }
  Serial.println("SDDIAG: all 3 reads identical (does not alone prove correct data)");
  return true;
}
void printBoot(const uint8_t *b) {
  Serial.printf("SDDIAG: BPB bytes/sector=%u sectors/cluster=%u reserved=%u FATs=%u\n",
    le16(b+11), b[13], le16(b+14), b[16]);
  Serial.printf("SDDIAG: BPB rootEntries=%u total16=%u total32=%lu hidden=%lu\n",
    le16(b+17), le16(b+19), (unsigned long)le32(b+32), (unsigned long)le32(b+28));
  Serial.printf("SDDIAG: BPB FATsize16=%u FATsize32=%lu rootCluster=%lu FSInfo=%u backupBoot=%u\n",
    le16(b+22), (unsigned long)le32(b+36), (unsigned long)le32(b+44), le16(b+48), le16(b+50));
}
// Direct SPI commands below bypass the library's card-type/address conversion.
// Only CMD58 (OCR), CMD9 (CSD), and CMD17 (read block) are issued.
uint8_t commandCrc(const uint8_t *data, unsigned length) {
  uint8_t crc=0;
  for (unsigned i=0; i<length; ++i) {
    uint8_t value=data[i];
    for (unsigned bit=0; bit<8; ++bit) {
      crc <<= 1;
      if ((value ^ crc) & 0x80) crc ^= 0x09;
      value <<= 1;
    }
  }
  return (crc << 1) | 1;
}
uint16_t dataCrc(const uint8_t *data, unsigned length) {
  uint16_t crc=0;
  for(unsigned i=0;i<length;++i) {
    crc ^= uint16_t(data[i]) << 8;
    for(unsigned bit=0;bit<8;++bit) crc=(crc & 0x8000) ? (crc << 1)^0x1021 : crc << 1;
  }
  return crc;
}
void endDirect(SPIClass &spi, int cs) {
  digitalWrite(cs,HIGH); spi.transfer(0xFF); spi.endTransaction();
}
uint8_t startDirect(SPIClass &spi, int cs, uint32_t hz, uint8_t cmd, uint32_t arg) {
  spi.beginTransaction(SPISettings(hz, MSBFIRST, SPI_MODE0));
  digitalWrite(cs,HIGH); spi.transfer(0xFF); digitalWrite(cs,LOW);
  uint32_t start=millis();
  while (spi.transfer(0xFF)!=0xFF) {
    if(uint32_t(millis()-start)>=250) return 0xFF;
    delay(1);
  }
  uint8_t packet[6]={uint8_t(0x40|cmd),uint8_t(arg>>24),uint8_t(arg>>16),uint8_t(arg>>8),uint8_t(arg),0};
  packet[5]=commandCrc(packet,5);
  for(uint8_t b:packet) spi.transfer(b);
  for(unsigned i=0;i<16;++i) {
    uint8_t response=spi.transfer(0xFF);
    if(!(response & 0x80)) return response;
  }
  return 0xFF;
}
bool directData(SPIClass &spi, int cs, uint32_t hz, uint8_t cmd, uint32_t arg, uint8_t *data, unsigned length) {
  uint8_t r1=startDirect(spi,cs,hz,cmd,arg);
  if(r1!=0) {
    Serial.printf("SDDIAG: direct CMD%u arg=%lu R1=%02X FAILED\n",cmd,(unsigned long)arg,r1);
    endDirect(spi,cs); return false;
  }
  uint32_t start=millis(); uint8_t token;
  do { token=spi.transfer(0xFF); if(token!=0xFF) break; delay(1); }
  while(uint32_t(millis()-start)<500);
  if(token!=0xFE) {
    Serial.printf("SDDIAG: direct CMD%u data token=%02X FAILED\n",cmd,token);
    endDirect(spi,cs); return false;
  }
  // Byte transfers deliberately avoid transfer16/transfer32 byte-order effects.
  for(unsigned i=0;i<length;++i) data[i]=spi.transfer(0xFF);
  uint16_t received=uint16_t(spi.transfer(0xFF))<<8;
  received |= spi.transfer(0xFF);
  endDirect(spi,cs);
  const uint16_t computed=dataCrc(data,length);
  Serial.printf("SDDIAG: direct CMD%u arg=%lu CRC received=%04X computed=%04X %s\n",
    cmd,(unsigned long)arg,received,computed,received==computed?"OK":"FAILED");
  return received==computed;
}
void diagnoseAddressing(SPIClass &spi, int cs, uint32_t hz, uint32_t bootLba) {
  Serial.println("SDDIAG: DIRECT capacity/addressing probe (no writes)");
  for(unsigned repeat=0;repeat<2;++repeat) {
    uint8_t r1=startDirect(spi,cs,hz,58,0);
    uint8_t b[4]={};
    if(r1==0) for(unsigned i=0;i<4;++i) b[i]=spi.transfer(0xFF);
    endDirect(spi,cs);
    uint32_t ocr=(uint32_t(b[0])<<24)|(uint32_t(b[1])<<16)|(uint32_t(b[2])<<8)|b[3];
    Serial.printf("SDDIAG: CMD58 bytewise R1=%02X bytes=%02X %02X %02X %02X OCR=%08lX ready=%u CCS=%u\n",
      r1,b[0],b[1],b[2],b[3],(unsigned long)ocr,unsigned(ocr>>31),unsigned((ocr>>30)&1));
  }
  uint8_t r1=startDirect(spi,cs,hz,58,0);
  uint32_t packed=0;
  if(r1==0) packed=spi.transfer32(0xFFFFFFFF);
  endDirect(spi,cs);
  Serial.printf("SDDIAG: CMD58 transfer32 R1=%02X value=%08lX CCS=%u (compare bytewise OCR)\n",
    r1,(unsigned long)packed,unsigned((packed>>30)&1));
  uint8_t csd[16];
  if(directData(spi,cs,hz,9,0,csd,16)) {
    Serial.printf("SDDIAG: CSD structure=%u bytes=",csd[0]>>6);
    for(uint8_t b:csd) Serial.printf("%02X ",b);
    Serial.println();
  }
  uint8_t data[512], repeat[512];
  // Test both command arguments without changing the driver's detected type.
  for(unsigned mode=0;mode<2;++mode) {
    const uint32_t argument=mode ? bootLba*512u : bootLba;
    Serial.printf("SDDIAG: testing %s argument=%lu\n",mode?"BYTE-ADDRESS":"SDHC BLOCK-ADDRESS",(unsigned long)argument);
    if(!directData(spi,cs,hz,17,argument,data,512)) continue;
    Serial.printf("SDDIAG: direct hash=%08lX signature=%02X %02X\n",(unsigned long)hashSector(data),data[510],data[511]);
    printBoot(data);
    if(directData(spi,cs,hz,17,argument,repeat,512))
      Serial.printf("SDDIAG: direct repeat %s\n",memcmp(data,repeat,512)==0?"IDENTICAL":"MISMATCH");
  }
}

}
// Startup-only, read-only probe after SD.begin has released its failed mount.
// Uses the existing SPI object and CS. No format, sector writes, or pin changes.
void diagnoseSdRaw(SPIClass &spi, int csPin, uint32_t frequency) {
  Serial.printf("SDDIAG: BEGIN read-only probe at %lu Hz\n", (unsigned long)frequency);
  uint8_t drive = sdcard_init(csPin, &spi, frequency);
  if (drive == 0xFF) { Serial.println("SDDIAG: no disk slot; END"); return; }
  const DSTATUS status = disk_initialize(drive);
  Serial.printf("SDDIAG: initialization status=0x%02X cardType=%u\n", status, (unsigned)sdcard_type(drive));
  if (!(status & (STA_NOINIT | STA_NODISK))) {
    uint8_t sector[512];
    uint32_t bootLba = 32; // Known PC-inspected card partition starts at byte 16384.
    if (readChecked(drive, 0, sector)) {
      if (sector[510] == 0x55 && sector[511] == 0xAA) {
        for (unsigned i=0; i<4; ++i) {
          const uint8_t *entry = sector + 446 + 16*i;
          Serial.printf("SDDIAG: partition %u type=0x%02X start=%lu sectors=%lu\n",
            i+1, entry[4], (unsigned long)le32(entry+8), (unsigned long)le32(entry+12));
        }
        const uint8_t type = sector[450];
        const uint32_t first = le32(sector+454);
        if ((type==0x0B || type==0x0C) && first>0 && first<=1048576) bootLba=first;
        if (le16(sector+11)==512 && sector[16]>0 && sector[16]<=2) {
          Serial.println("SDDIAG: sector 0 may itself be a volume boot sector"); printBoot(sector);
        }
      } else Serial.println("SDDIAG: sector 0 has no 55 AA signature");
    }
    Serial.printf("SDDIAG: checking boot LBA %lu (PC expected 32)\n", (unsigned long)bootLba);
    if (readChecked(drive, bootLba, sector)) printBoot(sector);
    diagnoseAddressing(spi, csPin, frequency, bootLba);
  }
  sdcard_uninit(drive);
  Serial.println("SDDIAG: END; logging remains disabled");
}
