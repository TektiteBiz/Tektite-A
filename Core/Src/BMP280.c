#include "BMP280.h"

#include <math.h>
#include <stdint.h>
#include "main.h"

SPI_HandleTypeDef *spiHandle;
int32_t t_fine = 0;

/* ---------------------------------------------------------------------------
 * SPI interface definitions -------------------------------------------------
 * (ADAPT THESE METHODS TO YOUR HARDWARE) ------------------------------------
 * ------------------------------------------------------------------------ */

/**
 * SPI transmit and receive one byte simultaneously
 * @param tx_message: Transmit byte.
 * @return Received byte.
 * */
uint8_t spiReadWrite(uint8_t tx_message)
{
	uint8_t rx_message = 255;
	HAL_SPI_TransmitReceive(spiHandle, &tx_message, &rx_message, 1,
			HAL_MAX_DELAY);
	return rx_message;
}

/** Pull chip select high (inactive) */
void spiCSNhigh()
{
	HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
}

/** Pull chip select low (active) */
void spiCSNlow()
{
	HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
}

/** Millisecond Delay */
void delay_ms(uint32_t milliseconds)
{
	HAL_Delay(milliseconds);
}

/**
 * Read a register
 * @param address: Register address.
 * @return Register value.
 * */
uint8_t readRegister(uint8_t address)
{
	spiCSNlow();
	spiReadWrite(address);
	uint8_t value = spiReadWrite(0);
	spiCSNhigh();
	return value;
}

/**
 * Write to a register
 * @param address: Register address.
 * @param value: Value to write.
 * */
void writeRegister(uint8_t address, uint8_t value)
{
	spiCSNlow();
	spiReadWrite(address & BMP280_SPI_MASK_WRITE);
	spiReadWrite(value);
	spiCSNhigh();
}

/**
 * Read a multi-byte register
 * @param address: Register address.
 * @param values: Array pointer to store values in.
 * @param length: Number of bytes to read.
 * */
void readMBRegister(uint8_t address, uint8_t *values, uint8_t length)
{
	spiCSNlow();
	spiReadWrite(address);
	while (length--)
	{
		*values++ = spiReadWrite(0);
	}
	spiCSNhigh();
}


struct CompensationParameters
{
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
} compensationParameters;



/** Perform power-on reset procedure */
void reset()
{
	writeRegister(BMP280_REG_RESET, BMP280_RESET_VALUE);
}

/**
 * Read chip identification number.
 * @return chip ID
 * */
uint8_t getID()
{
	return readRegister(BMP280_REG_ID);
}


/** Read calibration data from non-volatile sensor registers */
void readCompensationParameters()
{
	uint8_t buf[24];
	readMBRegister(BMP280_REG_CALIB, buf, 24);
	compensationParameters.dig_t1 = ((buf[1] << 8) | buf[0]);
	compensationParameters.dig_t2 = ((buf[3] << 8) | buf[2]);
	compensationParameters.dig_t3 = ((buf[5] << 8) | buf[4]);
	compensationParameters.dig_p1 = ((buf[7] << 8) | buf[6]);
	compensationParameters.dig_p2 = ((buf[9] << 8) | buf[8]);
	compensationParameters.dig_p3 = ((buf[11] << 8) | buf[10]);
	compensationParameters.dig_p4 = ((buf[13] << 8) | buf[12]);
	compensationParameters.dig_p5 = ((buf[15] << 8) | buf[14]);
	compensationParameters.dig_p6 = ((buf[17] << 8) | buf[16]);
	compensationParameters.dig_p7 = ((buf[19] << 8) | buf[18]);
	compensationParameters.dig_p8 = ((buf[21] << 8) | buf[20]);
	compensationParameters.dig_p9 = ((buf[23] << 8) | buf[22]);
}

/** Configure pressure oversampling */
void setPressureOversampling(enum Oversampling osrs_p)
{
	uint8_t ctrl = readRegister(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b11100011) | (osrs_p << 2);
	writeRegister(BMP280_REG_CTRL, ctrl);
}

/** Configure temperature oversampling */
void setTemperatureOversampling(enum Oversampling osrs_t)
{
	uint8_t ctrl = readRegister(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b00011111) | (osrs_t << 5);
	writeRegister(BMP280_REG_CTRL, ctrl);
}

/** Configure power mode */
void setPowerMode(enum PowerMode mode)
{
	uint8_t ctrl = readRegister(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b11111100) | mode;
	writeRegister(BMP280_REG_CTRL, ctrl);
}

/** Configure standby time */
void setStandbyTime(enum StandbyTime t_sb)
{
	uint8_t conf = readRegister(BMP280_REG_CONFIG);
	conf = (conf & 0b00011111) | (t_sb << 5);
	writeRegister(BMP280_REG_CONFIG, conf);
}

/** Configure IIR filter */
void setFilterCoefficient(enum FilterSetting filter)
{
	uint8_t conf = readRegister(BMP280_REG_CONFIG);
	conf = (conf & 0b11100011) | (filter << 2);
	writeRegister(BMP280_REG_CONFIG, conf);
}


uint8_t BMP280Initialize(SPI_HandleTypeDef *handle)
{
	spiHandle = handle;
	if (getID() != BMP280_CHIP_ID)
	{
		return 1;
	}

	// Reset device and wait
	reset();
	delay_ms(500);

	// BEGIN OF CONFIGURATION ----------------------------------
	setPressureOversampling(oversampling_x16);
	setTemperatureOversampling(oversampling_x2);

	setPowerMode(mode_normal);
	setFilterCoefficient(filter_coeff_16);
	setStandbyTime(standby_time_500us);
	// END OF CONFIGURATION --------------------------

	readCompensationParameters();

	return 0;
}

/**
 * Calculate sensor temperature from measurement and compensation parameters.
 * @param uncomp_temp: Raw temperature measurement.
 * @return Temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
 * */
int32_t compensate_temperature(int32_t uncomp_temp)
{
	int32_t var1, var2;
	var1 =
			((((uncomp_temp / 8)
					- ((int32_t) compensationParameters.dig_t1 << 1)))
					* ((int32_t) compensationParameters.dig_t2)) / 2048;
	var2 = (((((uncomp_temp / 16) - ((int32_t) compensationParameters.dig_t1))
			* ((uncomp_temp / 16) - ((int32_t) compensationParameters.dig_t1)))
			/ 4096) * ((int32_t) compensationParameters.dig_t3)) / 16384;
	t_fine = var1 + var2;
	return (t_fine * 5 + 128) / 256;
}

/**
 * Calculate pressure from measurement and compensation parameters.
 * @param uncomp_pres: Raw pressure measurement.
 * @return Pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 * */
uint32_t compensate_pressure(int32_t uncomp_pres)
{
	int64_t var1, var2, p;

	var1 = ((int64_t) (t_fine)) - 128000;
	var2 = var1 * var1 * (int64_t) compensationParameters.dig_p6;
	var2 = var2 + ((var1 * (int64_t) compensationParameters.dig_p5) * 131072);
	var2 = var2 + (((int64_t) compensationParameters.dig_p4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t) compensationParameters.dig_p3) / 256)
			+ ((var1 * (int64_t) compensationParameters.dig_p2) * 4096);
	var1 = ((INT64_C(0x800000000000) + var1)
			* ((int64_t) compensationParameters.dig_p1)) / 8589934592;
	if (var1 == 0)
	{
		return 0;
	}
	p = 1048576 - uncomp_pres;
	p = (((((p * 2147483648U)) - var2) * 3125) / var1);
	var1 = (((int64_t) compensationParameters.dig_p9) * (p / 8192) * (p / 8192))
			/ 33554432;
	var2 = (((int64_t) compensationParameters.dig_p8) * p) / 524288;
	p = ((p + var1 + var2) / 256)
			+ (((int64_t) compensationParameters.dig_p7) * 16);
	return (uint32_t) p;
}

/* ---------------------------------------------------------------------------
 * Measurements and compensation ---------------------------------------------
 * ------------------------------------------------------------------------ */

/**
 * Read latest measurement from sensor and execute compensation.
 * Stores the results in measurement member variable.
 * */
void BMP280Measure(struct Measurement* measurement)
{
	uint8_t data[6];
	readMBRegister(BMP280_REG_DATA, data, 6);

	int32_t adc_P = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	int32_t adc_T = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	measurement->temperature = (float) compensate_temperature(adc_T) / 100.0;
	measurement->pressure = (float) compensate_pressure(adc_P) / 256.0;
}
