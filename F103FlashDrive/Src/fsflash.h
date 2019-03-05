/*
 * fsflash.h
 *
 *  Created on: 2 мар. 2019 г.
 *      Author: homesrv
 */

#ifndef FSFLASH_H_
#define FSFLASH_H_

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define PAGE_ADDR ((uint32_t)0x08006000) //((uint32_t)0x08007800)
#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x80  // количество секторов
#define STORAGE_BLK_SIZ                  0x200 // размер сектора 0x200 * 0x80 = 65535 bytes
#define PAGE_ADDR ((uint32_t)0x08006000) //((uint32_t)0x08007800)

#define FAT12_ERR_BAD_FAT 2
#define FAT12_ERR_FILE_NOT_FOUND 1

uint32_t blk_buff[STORAGE_BLK_SIZ/4]; // 512 bytes for FLASH erase sector


uint8_t  uint8_t_from_internal_flash(uint32_t address);
uint32_t uint32_t_from_internal_flash(uint32_t address);
uint16_t uint16_t_from_internal_flash(uint32_t address);
char *char_from_internal_flash(uint32_t address);

// функция сохранения соседнего сектора перед стиранием страницы
void set_buf_before_erase(uint32_t page_addr);
// функция определения адреса страницы
uint32_t get_erase_addr (uint32_t page_addr);
// функция чтения данных FAT12
uint8_t  f12_read_data (
                        char *file_name,                // имя файла для поиска в формате 11 символов - "NAME    TXT"
                        char **file_data,               // здесь будут данные найденного файла
                        char *file_list,                // здесь будет список файлов
                        uint16_t file_list_size);       // размер для списка файлов



#ifdef __cplusplus
 }
#endif

#endif /* FSFLASH_H_ */
