#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#define I2C_TXBUFFER_SIZE   10      // Max transmit buffer size for write cmds (bytes)
#define I2C_RXBUFFER_SIZE   10      // Max transmit buffer size for write cmds (bytes)

#define SI7021_ADDRESS          0x80            // 1000 0000 = full 8 bits I2C address (includes end R/nW bit)
#define SI7021_CMD_READ_FW_REV  {0x84, 0xB8}    // Read FW version (awaited response = 0x20 = 00100000 = FW ver. 2.0)
#define SI7021_CMD_READ_TEMP    {0xE3}          // Read temperature cmd (using master hold mode - clock stretching)

//////////////////////////////////////////////////////////////////////

//GLOBAL VARIABLES

// debug
// static volatile uint8_t rxbuffer;
// static volatile uint8_t txbuffer;
// static volatile I2C_TransferReturn_TypeDef debug_result;
// static volatile I2C_TransferSeq_TypeDef debug_i2cTransfer;

// Buffer definition (uninitialized)
uint8_t i2c_txBuffer[I2C_TXBUFFER_SIZE]; // Array of I2C_TXBUFFER_SIZE bytes
uint8_t i2c_rxBuffer[I2C_RXBUFFER_SIZE]; // Array of I2C_TXBUFFER_SIZE bytes

// Transmission flag ==1 if transmission on-going
volatile bool i2c_startTx;

//////////////////////////////////////////////////////////////////////

// CONF. FUNCTIONS

// Clock Management Unit initialization
void initCMU(void)
 {
   // Enable clocks to the I2C and GPIO
   // CMU_ClockEnable(cmuClock_I2C0, true); // (NOT mandatory on EFRE32xG21)
   // CMU_ClockEnable(cmuClock_GPIO, true); // (NOT mandatory on EFRE32xG21)
 }

void initGPIO(void)
{
  // Configure BUTTON0(PB0) as input and interrupt
  GPIO_PinModeSet(gpioPortB, 0, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(gpioPortB, 0, 0, false, true, true);

  // Configure LED0(PD02) and LED1(PD03) as output
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);

  // Enable interface of SI7021 sensor through PD04
  GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0);

  // Enable EVEN (thus including PB0) interrupt to catch button press that starts I2C transfer
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void initI2C(void)
{
  // Use default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

  // Using PC02 (SDA) and PC00 (SCL)
  GPIO_PinModeSet(gpioPortC, 2, gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);

  // Route I2C pins to GPIO
  GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
                        | (gpioPortC << _GPIO_I2C_SDAROUTE_PORT_SHIFT
                        | (2 << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
                        | (gpioPortC << _GPIO_I2C_SCLROUTE_PORT_SHIFT
                        | (0 << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;

  // Initialize the I2C
  I2C_Init(I2C0, &i2cInit);

  // Set the status flags and index
  i2c_startTx = false;

  // Enable automatic STOP on NACK
  I2C0->CTRL = I2C_CTRL_AUTOSN;
}

// Make request to slave and readback result (Hold Master Mode -- with clock stretch during measurement)
// Data stored in rxBuff using its pointer
// numBytes = number of bytes of the data received from slave sensor
// WARNING : slaveAddress is the full 8 bits I2C address including the READ/nWRITE bit (7bits address + R/nW bit)
// for Si7021:
//  -- slaveAddress = 1000 0000 = 0x80
//  -- Measure Relative Humidity, Hold Master Mode :    0xE5
//  -- Measure Temperature, Hold Master Mode :          0xE3
//  -- Reset :                                          0xFE

// I2C_RequestAndReadback()

void I2C_RequestAndReadback(uint8_t slaveAddress, uint8_t *requestCmd, uint8_t numBytesCmd, uint8_t *rxBuff, uint8_t numBytesRx)
{

  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Initialize I2C transfer
  i2cTransfer.addr          = slaveAddress;
  i2cTransfer.flags         = I2C_FLAG_WRITE_READ; // must write requestCmd before reading response
  i2cTransfer.buf[0].data   = requestCmd;   // must be a pointer
  i2cTransfer.buf[0].len    = numBytesCmd;  //bytes
  i2cTransfer.buf[1].data   = rxBuff;
  i2cTransfer.buf[1].len    = numBytesRx;

  // debug_i2cTransfer = i2cTransfer;

  result = I2C_TransferInit(I2C0, &i2cTransfer); // returns the status of on-going transfer, i2cTransferInProgress if transfer not finished

  // I2C_Transfer continues the transfer initiated by I2C_TransferInit
  // Returns TransferDone if transfer successful, i2cTransferInProgress if transfer not finished
  while (result == i2cTransferInProgress) {
    result = I2C_Transfer(I2C0);
  }

  // Error occured during transfer
  if (result != i2cTransferDone) {
    // LED1 ON to indicate I2C transmission problem
    GPIO_PinOutSet(gpioPortD, 3);
  }

  i2c_startTx = false;
//   rxbuffer = rxBuff;
//   debug_result = result;
}

// numBytes = number of bytes of the data written to slave sensor (must be < I2C_TXBUFFER_SIZE)
void I2C_Write(uint8_t slaveAddress, uint8_t writeCmd, uint8_t *txBuff, uint8_t numBytes)
{

  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  uint8_t txBuffer[I2C_TXBUFFER_SIZE + 1];  // 1 byte is added in order to add the writeCmd prior data to write

  txBuffer[0] = writeCmd;   // 1st byte of txBuffer is the writeCmd
  for(int i = 0; i < numBytes; i++)
  {
      txBuffer[i + 1] = txBuff[i];  // data in txBuff is added after the writeCmd (after the 1st byte)
  }

  // Initialize I2C transfer
  i2cTransfer.addr          = slaveAddress;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = txBuffer;     // txBuffer = writeCmd[1byte] + data[numBytes]
  i2cTransfer.buf[0].len    = numBytes + 1; // numBytes is < I2C_TXBUFFER_SIZE
  i2cTransfer.buf[1].data   = NULL;
  i2cTransfer.buf[1].len    = 0;

  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Send data
  while (result == i2cTransferInProgress) {
    result = I2C_Transfer(I2C0);
  }

  if (result != i2cTransferDone) {
    // LED1 ON and infinite while loop to indicate I2C transmission problem
    GPIO_PinOutSet(gpioPortD, 3);
  }
}

void GPIO_EVEN_IRQHandler(void)
{
  // Clear pending
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  // Re-enable I2C
  I2C_Enable(I2C0, true);

  i2c_startTx = true;
}

int main(void)
{
  // Chip errata
  CHIP_Init();

  // Initialize the I2C
  initCMU();
  initGPIO();
  initI2C();

  // Enable Si7021 sensor I2C interface by setting PD04
  GPIO_PinOutSet(gpioPortD, 4);

  // i2c_startTx = true if Interrupt on Button0 (IRQHandler)
  while (1) {

    if (i2c_startTx) {
      // Transmitting data

      //uint8_t cmd[2] = SI7021_CMD_READ_FW_REV;
      uint8_t cmd[1] = SI7021_CMD_READ_TEMP;

      uint8_t numBytesCmd = sizeof(cmd);
      I2C_RequestAndReadback(SI7021_ADDRESS, cmd, numBytesCmd, i2c_rxBuffer, 2);

      } else {
        // Toggle LED0 when transmission passed
        GPIO_PinOutToggle(gpioPortD, 2);
        // Transmission complete
        i2c_startTx = false;
      }
    }
}
