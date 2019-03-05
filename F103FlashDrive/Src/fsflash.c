/*
 * fsflash.c
 *
 *  Created on: 5.03.2019 г.
 *      Author: Vladimir Kuzmin
 *      vladimir.vladimirovitch.kuzmin@ya.ru
 */
#include "fsflash.h"
#include "usbd_storage_if.h"
#include "cdebug.h"

 // функции чтения из flash
 uint8_t uint8_t_from_internal_flash(uint32_t address) {
 	return *((uint8_t *)address);
 }
 uint32_t uint32_t_from_internal_flash(uint32_t address) {
 	return *((uint32_t *)address);
 }
 uint16_t uint16_t_from_internal_flash(uint32_t address) {
 	return *((uint16_t *)address);
 }
 char *char_from_internal_flash(uint32_t address) {
 	return ((char *)address);
 }

 // функция сохранения соседнего сектора перед стиранием страницы
 void set_buf_before_erase(uint32_t page_addr) {
     for (int i=0; i<STORAGE_BLK_SIZ/4;i++) {
         blk_buff[i]=uint32_t_from_internal_flash(page_addr);
         page_addr+=4;
     }
 }
 // функция определения адреса страницы
 uint32_t get_erase_addr (uint32_t page_addr) {
     uint32_t cnt = page_addr-PAGE_ADDR;
     if (cnt % (STORAGE_BLK_SIZ*2) == 0)
         return page_addr;
     else
         return page_addr-STORAGE_BLK_SIZ;
 }

 uint8_t f12_read_data (
                         char *file_name,                // имя файла для поиска в формате 11 символов - "NAME    TXT"
                         char **file_data,               // здесь будут данные найденного файла
                         char *file_list,                // здесь будет список файлов
                         uint16_t file_list_size)  {     // размер для списка файлов
 // FAT12 - http://elm-chan.org/docs/fat_e.html
  uint16_t BPB_BytsPerSec;
  uint8_t  BPB_NumFATs;
  uint16_t BPB_RootEntCnt;
  uint16_t BPB_RsvdSecCnt;
  uint16_t BPB_FATSz16;
  uint8_t  BPB_SecPerClus;
  uint16_t BS_BootSign;
  uint8_t  found=0;
  uint16_t cluster_no;

  char     *fname;
  uint32_t SEEK_FAT12_NAMES_OFFSET;
  uint32_t SEEK_FAT12_NAMES_OFFSET_END;
  uint32_t SEEK_DATA_OFFSET;
  uint32_t fs;

  // Получим данные FAT12
  BPB_BytsPerSec=uint16_t_from_internal_flash(PAGE_ADDR+11);
  BPB_NumFATs=uint8_t_from_internal_flash(PAGE_ADDR+16);
  BPB_RootEntCnt=uint16_t_from_internal_flash(PAGE_ADDR+17);
  BPB_RsvdSecCnt=uint16_t_from_internal_flash(PAGE_ADDR+14);
  BPB_FATSz16=uint16_t_from_internal_flash(PAGE_ADDR+22);
  BPB_SecPerClus=uint8_t_from_internal_flash(PAGE_ADDR+13);
  BS_BootSign=uint16_t_from_internal_flash(PAGE_ADDR+510);

  if (BS_BootSign != 0xAA55) { // неверная сигнатура boot сектора FAT
      return FAT12_ERR_BAD_FAT;
  }

  SEEK_FAT12_NAMES_OFFSET = (BPB_RsvdSecCnt+(BPB_FATSz16 * BPB_NumFATs)) * BPB_BytsPerSec; // отступ на начало имен
  SEEK_FAT12_NAMES_OFFSET_END=SEEK_FAT12_NAMES_OFFSET +
                            (((32 * BPB_RootEntCnt + BPB_BytsPerSec - 1) / BPB_BytsPerSec) * BPB_BytsPerSec); // отступ на конец имен

  SEEK_DATA_OFFSET= BPB_BytsPerSec * BPB_RsvdSecCnt+ // Boot сектор + резерв
                     ((BPB_FATSz16 * BPB_NumFATs) * BPB_BytsPerSec) + // Размер FAT
                     BPB_RootEntCnt*32; // Размер имен и хар-к файлов

  if (file_list_size > 0) // обнулить строку списка файлов
             file_list[0]=0;

  while (SEEK_FAT12_NAMES_OFFSET < SEEK_FAT12_NAMES_OFFSET_END)  {

          fname = char_from_internal_flash(PAGE_ADDR+SEEK_FAT12_NAMES_OFFSET);
          cluster_no  = uint16_t_from_internal_flash(PAGE_ADDR + SEEK_FAT12_NAMES_OFFSET + 0x1A);
          fs = uint32_t_from_internal_flash(PAGE_ADDR + SEEK_FAT12_NAMES_OFFSET + 0x1C);

          if ((file_list_size > 0) && (strlen(file_list)+20 < file_list_size) && (cluster_no  > 0)) {
              sprintf(file_list,"%s%11s %06d bytes\r\n",file_list,fname, fs); // дополним полный список файлов
          }

          for (int i=0;i<9;i++) {
        	  if (fname[i] != file_name[i]) {
			  found=0;
			  break;
        	  }
        	found=1;
          }

          if (found==1) {
              found=1;
              break;
          }
          SEEK_FAT12_NAMES_OFFSET+=0x20;
  }

      if (found == 0) {
          return FAT12_ERR_FILE_NOT_FOUND;
      }
     if ( *file_data != 0 ) {
      *file_data=char_from_internal_flash(PAGE_ADDR+(cluster_no-2) *
              (BPB_SecPerClus * BPB_BytsPerSec) + // Размер кластера
              SEEK_DATA_OFFSET);
     }
      return 0;
 }

 void writeBuf (uint32_t page_addr, uint8_t *buf){

    uint32_t erase_addr=get_erase_addr(page_addr);
    uint32_t buf_erase_addr;
    uint32_t buf32;


    if (page_addr != erase_addr) { // addr стираем пред.страницу
        buf_erase_addr=erase_addr;
    }   else    {
        buf_erase_addr=erase_addr+STORAGE_BLK_SIZ;
      }
     HAL_FLASH_Unlock();

    set_buf_before_erase(buf_erase_addr);

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = erase_addr;
    EraseInitStruct.NbPages     = 1;

    HAL_FLASHEx_Erase(&EraseInitStruct,&PAGEError);

    // запишем сохраненный буфер
    for (int i=0; i<STORAGE_BLK_SIZ/4;i++) {
     HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,buf_erase_addr,blk_buff[i]);
     buf_erase_addr+=4;
     }
    // запишем данные
    for (int i=0; i<STORAGE_BLK_SIZ/4;i++) {
     buf32=*(uint32_t *)&buf[i*4];
     HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, page_addr,buf32);
     page_addr+=4;
     }

     HAL_FLASH_Lock();
 }
