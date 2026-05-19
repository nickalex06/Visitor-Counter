/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   User diskio driver for microSD over SPI.
 ******************************************************************************
  */
/* USER CODE END Header */

#include "user_diskio.h"
#include "ff_gen_drv.h"
#include "diskio.h"
#include "ff.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define SD_BLOCK_SIZE 512

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType = 0;

#define CT_MMC   0x01
#define CT_SD1   0x02
#define CT_SD2   0x04
#define CT_SDC   (CT_SD1 | CT_SD2)
#define CT_BLOCK 0x08

DSTATUS USER_initialize(BYTE pdrv);
DSTATUS USER_status(BYTE pdrv);
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef USER_Driver =
{
    USER_initialize,
    USER_status,
    USER_read,
#if _USE_WRITE == 1
    USER_write,
#endif
#if _USE_IOCTL == 1
    USER_ioctl,
#endif
};

static void sd_cs_high(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

static void sd_cs_low(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static uint8_t sd_spi_txrx(uint8_t data)
{
    uint8_t rx = 0xFF;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void sd_spi_send_multi(const uint8_t *buff, UINT len)
{
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buff, len, HAL_MAX_DELAY);
}

static void sd_spi_receive_multi(uint8_t *buff, UINT len)
{
    uint8_t tx = 0xFF;

    for (UINT i = 0; i < len; i++) {
        HAL_SPI_TransmitReceive(&hspi1, &tx, &buff[i], 1, HAL_MAX_DELAY);
    }
}

static int sd_wait_ready(UINT timeout_ms)
{
    uint8_t response;
    uint32_t start = HAL_GetTick();

    do {
        response = sd_spi_txrx(0xFF);

        if (response == 0xFF) {
            return 1;
        }
    } while ((HAL_GetTick() - start) < timeout_ms);

    return 0;
}

static void sd_deselect(void)
{
    sd_cs_high();
    sd_spi_txrx(0xFF);
}

static int sd_select(void)
{
    sd_cs_low();
    sd_spi_txrx(0xFF);

    if (sd_wait_ready(500)) {
        return 1;
    }

    sd_deselect();
    return 0;
}

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc;
    uint8_t response;

    if (cmd & 0x80) {
        cmd &= 0x7F;
        response = sd_send_cmd(55, 0);

        if (response > 1) {
            return response;
        }
    }

    sd_deselect();

    if (!sd_select()) {
        return 0xFF;
    }

    sd_spi_txrx(0x40 | cmd);
    sd_spi_txrx((uint8_t)(arg >> 24));
    sd_spi_txrx((uint8_t)(arg >> 16));
    sd_spi_txrx((uint8_t)(arg >> 8));
    sd_spi_txrx((uint8_t)arg);

    crc = 0x01;

    if (cmd == 0) {
        crc = 0x95;
    }

    if (cmd == 8) {
        crc = 0x87;
    }

    sd_spi_txrx(crc);

    for (int i = 0; i < 10; i++) {
        response = sd_spi_txrx(0xFF);

        if ((response & 0x80) == 0) {
            return response;
        }
    }

    return response;
}

static int sd_receive_block(uint8_t *buff, UINT btr)
{
    uint8_t token;
    uint32_t start = HAL_GetTick();

    do {
        token = sd_spi_txrx(0xFF);

        if (token == 0xFE) {
            break;
        }
    } while ((HAL_GetTick() - start) < 200);

    if (token != 0xFE) {
        return 0;
    }

    sd_spi_receive_multi(buff, btr);

    sd_spi_txrx(0xFF);
    sd_spi_txrx(0xFF);

    return 1;
}

static int sd_transmit_block(const uint8_t *buff, uint8_t token)
{
    uint8_t response;

    if (!sd_wait_ready(500)) {
        return 0;
    }

    sd_spi_txrx(token);

    if (token != 0xFD) {
        sd_spi_send_multi(buff, SD_BLOCK_SIZE);

        sd_spi_txrx(0xFF);
        sd_spi_txrx(0xFF);

        response = sd_spi_txrx(0xFF);

        if ((response & 0x1F) != 0x05) {
            return 0;
        }
    }

    return 1;
}

DSTATUS USER_initialize(BYTE pdrv)
{
    uint8_t n;
    uint8_t cmd;
    uint8_t ty;
    uint8_t ocr[4];
    uint32_t timer;

    if (pdrv != 0) {
        return STA_NOINIT;
    }

    sd_cs_high();

    for (n = 10; n; n--) {
        sd_spi_txrx(0xFF);
    }

    ty = 0;

    if (sd_send_cmd(0, 0) == 1) {
        if (sd_send_cmd(8, 0x1AA) == 1) {
            for (n = 0; n < 4; n++) {
                ocr[n] = sd_spi_txrx(0xFF);
            }

            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                timer = HAL_GetTick();

                do {
                    if (sd_send_cmd(0x80 | 41, 1UL << 30) == 0) {
                        break;
                    }
                } while ((HAL_GetTick() - timer) < 1000);

                if ((HAL_GetTick() - timer) < 1000 && sd_send_cmd(58, 0) == 0) {
                    for (n = 0; n < 4; n++) {
                        ocr[n] = sd_spi_txrx(0xFF);
                    }

                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        } else {
            if (sd_send_cmd(0x80 | 41, 0) <= 1) {
                ty = CT_SD1;
                cmd = 0x80 | 41;
            } else {
                ty = CT_MMC;
                cmd = 1;
            }

            timer = HAL_GetTick();

            do {
                if (sd_send_cmd(cmd, 0) == 0) {
                    break;
                }
            } while ((HAL_GetTick() - timer) < 1000);

            if ((HAL_GetTick() - timer) >= 1000 || sd_send_cmd(16, SD_BLOCK_SIZE) != 0) {
                ty = 0;
            }
        }
    }

    CardType = ty;
    sd_deselect();

    if (ty) {
        Stat &= ~STA_NOINIT;
    } else {
        Stat = STA_NOINIT;
    }

    return Stat;
}

DSTATUS USER_status(BYTE pdrv)
{
    if (pdrv != 0) {
        return STA_NOINIT;
    }

    return Stat;
}

DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0 || count == 0) {
        return RES_PARERR;
    }

    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    if (!(CardType & CT_BLOCK)) {
        sector *= SD_BLOCK_SIZE;
    }

    if (count == 1) {
        if (sd_send_cmd(17, sector) == 0) {
            if (sd_receive_block(buff, SD_BLOCK_SIZE)) {
                count = 0;
            }
        }
    } else {
        if (sd_send_cmd(18, sector) == 0) {
            do {
                if (!sd_receive_block(buff, SD_BLOCK_SIZE)) {
                    break;
                }

                buff += SD_BLOCK_SIZE;
            } while (--count);

            sd_send_cmd(12, 0);
        }
    }

    sd_deselect();

    return count ? RES_ERROR : RES_OK;
}

#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0 || count == 0) {
        return RES_PARERR;
    }

    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    if (!(CardType & CT_BLOCK)) {
        sector *= SD_BLOCK_SIZE;
    }

    if (count == 1) {
        if (sd_send_cmd(24, sector) == 0) {
            if (sd_transmit_block(buff, 0xFE)) {
                count = 0;
            }
        }
    } else {
        if (CardType & CT_SDC) {
            sd_send_cmd(0x80 | 23, count);
        }

        if (sd_send_cmd(25, sector) == 0) {
            do {
                if (!sd_transmit_block(buff, 0xFC)) {
                    break;
                }
            } while (--count);

            if (!sd_transmit_block(0, 0xFD)) {
                count = 1;
            }
        }
    }

    sd_deselect();

    return count ? RES_ERROR : RES_OK;
}
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    DRESULT res;
    BYTE n;
    BYTE csd[16];
    DWORD csize;

    if (pdrv != 0) {
        return RES_PARERR;
    }

    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    res = RES_ERROR;

    switch (cmd) {
    case CTRL_SYNC:
        if (sd_select()) {
            res = RES_OK;
        }
        break;

    case GET_SECTOR_COUNT:
        if ((sd_send_cmd(9, 0) == 0) && sd_receive_block(csd, 16)) {
            if ((csd[0] >> 6) == 1) {
                csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                *(DWORD *)buff = csize << 10;
            } else {
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                *(DWORD *)buff = csize << (n - 9);
            }

            res = RES_OK;
        }
        break;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 128;
        res = RES_OK;
        break;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = SD_BLOCK_SIZE;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
    }

    sd_deselect();

    return res;
}
#endif
