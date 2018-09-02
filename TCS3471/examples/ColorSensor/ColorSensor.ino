/*
  TCS3471.h - TCS3471 Color light-to-digital converter IC by TAOS/AMS library demo code
  Copyright (c) 2012, 2013 Raivis Rengelis (raivis [at] rrkb.lv). All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <driver/i2c.h>
#include "TCS3471.h"

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_EXAMPLE_SLAVE_SCL_IO           26               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_SLAVE_SDA_IO           25               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_SLAVE_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */

#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define BH1750_SENSOR_ADDR                 0x23             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */
#define ESP_SLAVE_ADDR                     0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_example_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t)ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, (i2c_ack_type_t)NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_example_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief test code to write esp-i2c-slave
 *
 * 1. set mode
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */


/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config((i2c_port_t)i2c_master_port, &conf);
    i2c_driver_install((i2c_port_t)i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief i2c slave initialization
 */
static void i2c_example_slave_init()
{
    int i2c_slave_port = I2C_EXAMPLE_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = (gpio_num_t)I2C_EXAMPLE_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = (gpio_num_t)I2C_EXAMPLE_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config((i2c_port_t)i2c_slave_port, &conf_slave);
    i2c_driver_install((i2c_port_t)i2c_slave_port, conf_slave.mode,
                       I2C_EXAMPLE_SLAVE_RX_BUF_LEN,
                       I2C_EXAMPLE_SLAVE_TX_BUF_LEN, 0);
}





// Since library stays away from hardware, user sketch has to take care about
// actual communications. Please note that functions need to be exactly like this,
// return void and accept 3 parameters, 1st - i2c address of slave chip,
// 2nd number of bytes to be written or read and 3rd - pointer to a byte array
// from where to read or where to write respective bytes
// Since TCS3471 chips have 12c fixed address, this can come in handy if you want to
// put multiple chips behind multiplexer or do something like that
void i2cWrite(byte address, byte count, byte* buffer);
void i2cRead(byte address, byte count, byte* buffer);

// This is where TCS3471 object gets created
// See how functions declared get passed to constructor as parameters
TCS3471 TCS3471(i2cWrite, i2cRead);

// Just a variable to tell main loop that interrupt was triggered
volatile bool colorAvailable = false;

// counter to check when to finish printing
int printXtimes = 0;

void setup()
{
  i2c_example_master_init();
  
  Serial.begin(115200);
  // starting i2c interface in master mode
  //Wire.begin();
  // TCS3471 can generate interrupt when color/light detection cycle finishes and/or is over or under set threshold
  // To free resources of Arduino connect INT pin of TCS3471 to pin 2 of Arduino and use interrupts!
  // NB! Open drain logic in use, which means high voltage = 0, low voltage = 1, thus FALLING edge trigger
  pinMode(12, INPUT_PULLUP);
  //attachInterrupt(0, TCS3471Int, FALLING);
  attachInterrupt(digitalPinToInterrupt(12), TCS3471Int, FALLING);
  // No need to pound the chip if there is no chip, detect() function checks if chip responds and returns true
  // if it is and false if it does not
  if (!TCS3471.detect())
  {
    Serial.println("Something is not right. Check your wiring, sensor needs at least VCC, GND, SCL and SDA.");
    while (1);
  }
  // tell TCS3471 to generate interrupts on INT pin
  TCS3471.enableInterrupt();
  // integration time is time chip takes to measure RGBC values, the longer, the higher precision
  // range is from 2.4ms up to 614.4ms, parameter is float and in milliseconds,
  // step is 2.4ms, but you can put in any number you like, library will take care of rounding
  TCS3471.setIntegrationTime(700.0);
  // set wait time between measurements, chip just sits there and waits for approximately that many
  // milliseconds and conserves power
  // range is from 2.4ms up to 7400ms, from 2.4ms up to 614.4ms step is 2.4ms, from 614.4ms up step is  28.8ms
  // library takes care of rounding in this case too
  // if set to anything less than 2.4ms, wait time is disabled
  TCS3471.setWaitTime(2000.0);
  // chip has 4 different analog gain settings - TCS3471_GAIN_1X, TCS3471_GAIN_4X, TCS3471_GAIN_16X and TCS3471_GAIN_60X
  // naked chip under regular ambient lighting works ok with 1x gain
  TCS3471.setGain(TCS3471_GAIN_1X);
  // if C(lear) channel goes above this value, interrupt will be generated
  // range is from 0-65535, 16 full bits
  // just for demonstration purposes, setting it right above half of full range
  TCS3471.interruptHighThreshold(32768);
  // similar as above, only C channel has to go below this value
  // again, range is from 0-65535
  // setting it right below half of full range
  // this will ensure that interrupt is generated all the time
  TCS3471.interruptLowThreshold(32767);
  // interrupt persistence determines how many times in row C channel has to go above high threshold
  // or below low threshold for interrupt to be generated
  // range is from 1 to 60, from 1-3 step is 1, from 5-60 step is 5
  // library takes care of finding closest valid value
  TCS3471.interruptPersistence(2);
  // once interrupt flag is set in chip, it must be cleared from host
  // in case it was left on from previous runs, we clear it here
  // otherwise we are not going to get new interrupts
  TCS3471.clearInterrupt();
  // and finally, when all the parameters are set, let's start measuring color and light
  TCS3471.enable();
}

void loop()
{
  if (colorAvailable)
  {
    // check if valid RGBC data is available
    // in this scenario this call is redundant
    // but for demonstration purposes, let's have it here
    if (TCS3471.rgbcValid())
    {
      // as mentioned previously, interrupt flag has to be cleared from host
      TCS3471.clearInterrupt();
      // reset Arduino flag too
      colorAvailable = false;
      // read C(lear) channel data
      // range from 0-65535
      word clearVal = TCS3471.readCData();
      // read R(ed) channel data
      // range 0-65535
      word redVal   = TCS3471.readRData();
      // read G(reen) channel data
      // range 0-65535
      word greenVal = TCS3471.readGData();
      // read B(lue) channel data
      // range 0-65535
      word blueVal  = TCS3471.readBData();
      // and print it all out
      Serial.print("Light is ");
      Serial.print(clearVal, HEX);
      Serial.print(" overall and red is ");
      Serial.print(redVal, HEX);
      Serial.print(" while green is ");
      Serial.print(greenVal, HEX);
      Serial.print(" and blue is ");
      Serial.println(blueVal, HEX);
      printXtimes++;
    }
  }
  if (printXtimes > 9)
  {
    // disable interrupt on Arduino
    detachInterrupt(0);
    // disable interrupt generation on TCS3471
    TCS3471.disableInterrupt();
    // and finally disable the chip itself, saves power!
    TCS3471.disable();
  }
}

// this is function that gets called on interrupt from TCS3471
void TCS3471Int()
{
  colorAvailable = true;
}

// implementation of i2cWrite and i2cRead functions for simplest case when there is only one
// TCS2471 chip attached to Arduino's two wire bus
void i2cWrite(byte address, byte count, byte* buffer)
{
  int ret;
  //Wire.beginTransmission(address);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
  i2c_master_write(cmd, buffer, count, (i2c_ack_type_t)ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin((i2c_port_t)I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
}

void i2cRead(byte address, byte count, byte* buffer)
{
  int ret;
  //Wire.beginTransmission(address);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
  if (count > 1) {
        i2c_master_read(cmd, buffer, count - 1, (i2c_ack_type_t)ACK_VAL);
    }
  i2c_master_read_byte(cmd, buffer + count - 1, (i2c_ack_type_t)NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin((i2c_port_t)I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
}
