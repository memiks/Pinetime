#include <legacy/nrf_drv_clock.h>
#include <softdevice/common/nrf_sdh.h>
#include <drivers/SpiMaster.h>
#include <drivers/Spi.h>
#include <drivers/SpiNorFlash.h>
#include <libraries/log/nrf_log.h>
#include "bootloader/boot_graphics.h"
#include <FreeRTOS.h>
#include <task.h>
#include <legacy/nrf_drv_gpiote.h>
#include <libraries/gpiote/app_gpiote.h>
#include <hal/nrf_wdt.h>
#include <flash_map.h>
#include <cstring>
#include <Components/Gfx/Gfx.h>
#include <drivers/St7789.h>
#include <Components/Brightness/BrightnessController.h>

#if NRF_LOG_ENABLED
#include "Logging/NrfLogger.h"
Pinetime::Logging::NrfLogger logger;
#else
#include "Logging/DummyLogger.h"
Pinetime::Logging::DummyLogger logger;
#endif

static constexpr uint8_t pinSpiSck = 2;
static constexpr uint8_t pinSpiMosi = 3;
static constexpr uint8_t pinSpiMiso = 4;
static constexpr uint8_t pinSpiFlashCsn = 5;
static constexpr uint8_t pinLcdCsn = 25;
static constexpr uint8_t pinLcdDataCommand = 18;

Pinetime::Drivers::SpiMaster spi{Pinetime::Drivers::SpiMaster::SpiModule::SPI0, {
        Pinetime::Drivers::SpiMaster::BitOrder::Msb_Lsb,
        Pinetime::Drivers::SpiMaster::Modes::Mode3,
        Pinetime::Drivers::SpiMaster::Frequencies::Freq8Mhz,
        pinSpiSck,
        pinSpiMosi,
        pinSpiMiso
  }
};
Pinetime::Drivers::Spi flashSpi{spi, pinSpiFlashCsn};
Pinetime::Drivers::SpiNorFlash spiNorFlash{flashSpi};

Pinetime::Drivers::Spi lcdSpi {spi, pinLcdCsn};
Pinetime::Drivers::St7789 lcd {lcdSpi, pinLcdDataCommand};

Pinetime::Components::Gfx gfx{lcd};
Pinetime::Controllers::BrightnessController brightnessController;

extern "C" {
void vApplicationIdleHook(void) {

}

/// Vector Table will be relocated here.
#define RELOCATED_VECTOR_TABLE 0x7F00

/// Number of entries in the Vector Table.
#define NVIC_NUM_VECTORS (16 + 38)

/// Address of the VTOR Register in the System Control Block.
#define SCB_VTOR ((uint32_t *) 0xE000ED08)

/// Contains addresses of flash sections. Defined in bin/targets/nrf52_boot/generated/src/nrf52_boot-sysflash.c
extern const struct flash_area sysflash_map_dflt[];

static void relocate_vector_table(void *vector_table, void *relocated_vector_table);

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void) {
  if(((NRF_SPIM0->INTENSET & (1<<6)) != 0) && NRF_SPIM0->EVENTS_END == 1) {
    NRF_SPIM0->EVENTS_END = 0;
    spi.OnEndEvent();
  }

  if(((NRF_SPIM0->INTENSET & (1<<19)) != 0) && NRF_SPIM0->EVENTS_STARTED == 1) {
    NRF_SPIM0->EVENTS_STARTED = 0;
    spi.OnStartedEvent();
  }

  if(((NRF_SPIM0->INTENSET & (1<<1)) != 0) && NRF_SPIM0->EVENTS_STOPPED == 1) {
    NRF_SPIM0->EVENTS_STOPPED = 0;
  }
}
}

void Process(void* instance) {
  // Wait before erasing the memory to let the time to the SWD debugger to flash a new firmware before running this one.
  vTaskDelay(5000);

  APP_GPIOTE_INIT(2);

  NRF_LOG_INFO("Init...");
  spi.Init();
  spiNorFlash.Init();
  brightnessController.Init();
  lcd.Init();
  gfx.Init();
  NRF_LOG_INFO("Init Done!")

  NRF_LOG_INFO("Read memory and display the graphic...");
  static constexpr uint32_t screenWidth = 240;
  static constexpr uint32_t screenWidthInBytes = screenWidth*2; // LCD display 16bits color (1 pixel = 2 bytes)
  uint16_t displayLineBuffer[screenWidth];
  for(int line = 0; line < screenWidth; line++) {
    spiNorFlash.Read(line*screenWidthInBytes, reinterpret_cast<uint8_t *>(displayLineBuffer), screenWidth);
    spiNorFlash.Read((line*screenWidthInBytes)+screenWidth, reinterpret_cast<uint8_t *>(displayLineBuffer) + screenWidth, screenWidth);
    for(int col = 0; col < screenWidth; col++) {
      gfx.pixel_draw(col, line, displayLineBuffer[col]);
    }
  }

  NRF_LOG_INFO("Init Relocation...");
  //  Init the Board Support Package.
  //hal_bsp_init();

  //  Previously: flash_map_init();
  //  Previously: rc = boot_go(&rsp);

  //  vector_table points to the Arm Vector Table for the appplication...
  //  First word contains initial MSP value (estack = end of RAM)
  //  Second word contains address of entry point (Reset_Handler)
  void *vector_table = (void *) (
      sysflash_map_dflt[1].fa_off  //  Offset of FLASH_AREA_IMAGE_0 (application image): 0x8000
      + 0x20                       //  Size of MCUBoot image header
  );                               //  Equals 0x8020 (__isr_vector)

  //  Relocate the application vector table to a 0x100 page boundary in ROM.
  relocate_vector_table(  //  Relocate the vector table...
      vector_table,       //  From the non-aligned application address (0x8020)
      (void *) RELOCATED_VECTOR_TABLE  //  To the relocated address aligned to 0x100 page boundary
  );

  //  Jump to Reset_Handler of the application. Uses first word and second word of vector table at img_start.
  hal_system_start(vector_table);

  //  Should never come here.
  //return 0;

  NRF_LOG_INFO("Done!");

  while(1) {
    asm("nop" );
  }
}

int main(void) {
  TaskHandle_t taskHandle;

  logger.Init();
  nrf_drv_clock_init();

  if (pdPASS != xTaskCreate(Process, "MAIN", 512, nullptr, 0, &taskHandle))
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);

  vTaskStartScheduler();

  for (;;) {
    APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
  }
}

/// Relocate the Arm Vector Table from vector_table to relocated_vector_table.
/// relocated_vector_table must be aligned to 0x100 page boundary.
static void relocate_vector_table(void *vector_table, void *relocated_vector_table) {
    uint32_t *current_location = (uint32_t *) vector_table;
    uint32_t *new_location     = (uint32_t *) relocated_vector_table;
    if (new_location == current_location) { return; }  //  No need to relocate
    //  Check whether we need to copy the vectors.
    int vector_diff = 0;  //  Non-zero if a vector is different
    for (int i = 0; i < NVIC_NUM_VECTORS; i++) {
        if (new_location[i] != current_location[i]) {
            vector_diff = 1;
            break;
        }
    }
    //  If we need to copy the vectors, erase the flash ROM and write the vectors.
    if (vector_diff) {
      NRF_LOG_INFO("Erasing...");
      spiNorFlash.SectorErase((uint32_t) relocated_vector_table);
      NRF_LOG_INFO("Erase done!");
      
      spiNorFlash.Write((uint32_t) relocated_vector_table, new_location, 0x100);
    }
    //  Point VTOR Register in the System Control Block to the relocated vector table.
    *SCB_VTOR = (uint32_t) relocated_vector_table;
}