#include "flash.h"
#include "stm32f411xe.h"
#include <stdio.h>
#include "systime.h"
#include "watchdog.h"

static void flash_unlock();
static void flash_lock() __attribute__((unused)) ;
static flash_result_t flash_wait_for_idle();

// we'll store parameters in sector 23
flash_result_t flash_erase_sector(const uint_fast8_t sector)
{
  if (sector >= 24)
    return FLASH_FAIL; // bogus sector 
  flash_unlock();
  flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_PSIZE;
  FLASH->CR |=  FLASH_CR_PSIZE_1; // we'll do 32-bit erases at a time
  FLASH->CR &= ~0xf8; // wipe out the sector address bits
  // fill in the sector address bits
  if (sector < 12)
    FLASH->CR |= sector << 3;
  else
    FLASH->CR |= (sector + 4) << 3;
  FLASH->CR |= FLASH_CR_SER; // sector erase operation
  FLASH->CR |= FLASH_CR_STRT; // start the sector-erase operation
  flash_result_t result = flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_SER; // reset the sector-erase operation bit
  FLASH->CR &= ~0xf8; // and wipe out the sector address bits
  //flash_lock(); // lock the flash again
  return result; // we're done
}

flash_result_t flash_erase_block_by_addr(const uint32_t *p)
{
  uint32_t addr = (uint32_t)p;
  if (addr <  0x08020000 ||
      addr >  0x080fffff ||
      (addr & 0x1ffff) != 0)
    return FLASH_FAIL; // bad address
  int offset = addr - 0x08020000;
  int block = 5 + offset / 0x20000;
  return flash_erase_sector(block);
}

void flash_unlock()
{
  // magic sequence to unlock flash...
  if (FLASH->CR & FLASH_CR_LOCK)
  {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xcdef89ab;
  }
  if (FLASH->CR & FLASH_CR_LOCK)
    printf("couldn't unlock flash\r\n");
}

void flash_lock() 
{
  FLASH->CR |= FLASH_CR_LOCK;
}

flash_result_t flash_wait_for_idle()
{
  uint32_t t_start = SYSTIME;
  while (FLASH->SR & FLASH_SR_BSY)
  {
    // keep resetting the watchdog, since this may be a long-running erase op
    // give it 10 seconds of watchdog auto-resetting
    if (SYSTIME - t_start < 10000000)
      watchdog_reset_counter();
  }
  // todo: check all the error bits, etc.
  if (FLASH->SR & 0x1f3)
  {
    printf("unexpected FLASH_SR error bit: FLASH_SR = 0x%08lx\r\n",
           FLASH->SR);
    FLASH->SR |= (FLASH->SR & 0x1f3); // clear the error bit(s)
    return FLASH_FAIL;
  }
  return FLASH_SUCCESS;
}

flash_result_t flash_program_word(const uint32_t *address, const uint32_t data)
{
  flash_unlock();
  if (FLASH_FAIL == flash_wait_for_idle())
    return FLASH_FAIL;
  FLASH->CR |= FLASH_CR_PG; // set the programming bit
  FLASH->CR &= ~FLASH_CR_PSIZE; // wipe out PSIZE to get ready to set it
  FLASH->CR |=  FLASH_CR_PSIZE_1; // we'll do 32-bit erases at a time
  *((volatile uint32_t *)address) = data;
  flash_result_t result = flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_PG; // disable the programming bit
  //flash_lock();
  return result;
}

flash_result_t flash_program_byte(const uint8_t *address, const uint8_t data)
{
  //printf("writing byte 0x%02x to 0x%08lx\r\n", data, (uint32_t)address);
  if (FLASH_FAIL == flash_wait_for_idle())
    return FLASH_FAIL;
  flash_unlock();
  FLASH->CR &= ~FLASH_CR_PSIZE; // set PSIZE to 0 for byte writes
  FLASH->CR |= FLASH_CR_PG;
  //printf("  flash_cr = 0x%08lx\r\n", FLASH->CR);
  *((volatile uint8_t *)address) = data;
  flash_result_t result = flash_wait_for_idle();
  FLASH->CR &= ~FLASH_CR_PG; // disable the programming bit
  //flash_lock();
  return result;
}

flash_result_t flash_flush_d_cache()
{
  // todo
  return FLASH_FAIL;
}

