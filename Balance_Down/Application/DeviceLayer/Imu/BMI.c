#include "BMI.h"

/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */


/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*! Macros to select the sensors                   */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi2_dev);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Gravity.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program.

	uint8_t	intf
	 * For I2C : BMI2_I2C_INTF
	 * For SPI : BMI2_SPI_INTF

*/
int8_t bmi_init(struct bmi2_dev *bmi2_dev,uint8_t intf)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Assign accel and gyro sensor to variable. */
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(bmi2_dev, intf);

    /* Initialize bmi270. */
    rslt = bmi270_init(bmi2_dev);

    if (rslt == BMI2_OK)
    {
        /* Accel and gyro configuration settings. */
        rslt = set_accel_gyro_config(bmi2_dev);

        if (rslt == BMI2_OK)
        {
            /* NOTE:
             * Accel and Gyro enable must be done after setting configurations
             */
            rslt = bmi270_sensor_enable(sensor_list, 2, bmi2_dev);

            /* Loop to print accel and gyro data when interrupt occurs. */

        }
    }


    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi2_dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(config, 2, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi270_set_sensor_config(config, 2, bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale))) * (val);
}

float inVSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void MPU_Read_all(uint8_t reg,uint8_t *buff,uint8_t len)
{
	BMI_CS_LOW();
	bmi2_delay_us(1, NULL);
	reg |= 0x80;
	HAL_SPI_Transmit(&hspi2, &reg,  1, 1000);
	HAL_SPI_Receive(&hspi2, buff, len+1, 1000);
	bmi2_delay_us(1, NULL);
	BMI_CS_HIG();
}

void BMI_Get_RawData(int16_t *ggx, int16_t *ggy, int16_t *ggz, int16_t *aax, int16_t *aay, int16_t *aaz)
{
	uint8_t data[13];
	int16_t buff[6];
	MPU_Read_all(ACCD_X_LSB, data, 13);
	
	buff[0] = (int16_t)data[1] | ( (int16_t)data[2] << 8);
	buff[1] = (int16_t)data[3] | ( (int16_t)data[4] << 8);
	buff[2] = (int16_t)data[5] | ( (int16_t)data[6] << 8);
	
	buff[3] = (int16_t)data[7] | ( (int16_t)data[8] << 8);
	buff[4] = (int16_t)data[9] | ( (int16_t)data[10] << 8);
	buff[5] = (int16_t)data[11] | ( (int16_t)data[12] << 8);
	
	*aax = buff[0];
	*aay = buff[1];
	*aaz = buff[2];
	*ggx = buff[3];
	*ggy = buff[4];
	*ggz = buff[5];	
}

extern struct bmi2_dev bmi270;

/**
    @param
    @Kp
        越大表示越信任加速度，但快速晃动时，yaw轴角度可能会变化或者快速漂移。Kp越大，初始化的时候数据稳定越快。
    @Ki
        越小积分误差越小
    @halfT
        解算周期的一半，比如1ms解算1次则halfT为0.0005f
*/
float Kp = 4.f;//4
float norm;
float halfT = 0.00025f;
float vx, vy, vz;
float ex, ey, ez;
float gx,gy,gz,ax,ay,az;
float q0_init = 1.0f, q1_init = 0.0f, q2_init = 0.0f, q3_init = 0.0f;  //将云台摆到yaw、pitch都为零时即初始值
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  //将云台摆到yaw、pitch都为零时即初始值
float q0temp,q1temp,q2temp,q3temp;
float gx_, gy_, gz_;
float q0_, q1_, q2_, q3_;
float q0temp_,q1temp_,q2temp_,q3temp_;
float a_sum;
float sintemp, sintemp_, costemp, costemp_;
/**
  * @brief  不带_的为涉及加速度计的，带_的为不涉及加速度计的，用于差分计算速度
  * @param  
  * @retval 
  */
uint8_t BMI_Get_EulerAngle(float *pitch,float *roll,float *yaw,\
                           float *pitch_,float *roll_,float *yaw_,\
													 float *ggx,float *ggy,float *ggz,\
													 float *aax,float *aay,float *aaz)
{
	/* 陀螺仪值赋值 */
	gx = *ggx;
	gy = *ggy;
	gz = *ggz;
	
	/* 加速度计值赋值 */
	ax = *aax;
	ay = *aay;
	az = *aaz;
	
	/* 陀螺仪数据单位转换（to度每秒） */
	gx = lsb_to_dps(gx,2000,bmi270.resolution);
	gy = lsb_to_dps(gy,2000,bmi270.resolution);
	gz = lsb_to_dps(gz,2000,bmi270.resolution);
	
	/* 陀螺仪数据单位转换（to弧度每秒） */
	gx = gx * (double)0.017453;
	gy = gy * (double)0.017453;
	gz = gz * (double)0.017453;
	
	/* 备份给速度解算用 */
	gx_ = gx;
	gy_ = gy;
	gz_ = gz;
	
	/* 角度解算start */
	/* 加速度计数据检查 */
	if(ax * ay *az != 0)
	{
		/* 加速度计数据单位转换（to米每二次方秒） */
		ax = lsb_to_mps2(ax,2,bmi270.resolution);
		ay = lsb_to_mps2(ay,2,bmi270.resolution);
		az = lsb_to_mps2(az,2,bmi270.resolution);

		norm = inVSqrt(ax*ax + ay*ay + az*az);
		ax = ax *norm;
		ay = ay *norm;
		az = az *norm;
		
		vx = 2*(q1*q3 - q0*q2);//-sin(Pitch) cos(K,i)
		vy = 2*(q0*q1 + q2*q3);//sin(Roll)cos(Pitch) cos(K,j)
		vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;//cos(Roll)cos(Pitch) cos(K,k)
		
		ex = (ay*vz - az*vy) ;
		ey = (az*vx - ax*vz) ;//切线方向加速度
		ez = (ax*vy - ay*vx) ;
		
		gx = gx + Kp*ex;
		gy = gy + Kp*ey;
		gz = gz + Kp*ez;
	}
	
	q0temp=q0;
  q1temp=q1;
  q2temp=q2;
  q3temp=q3;
	
	q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
	q1 = q1temp + ( q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
	q2 = q2temp + ( q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
	q3 = q3temp + ( q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
	
	norm = inVSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
//	*roll = atan2(2 * q2 * q3 + 2 * q0 * q1,q0*q0 - q1 * q1 -  q2 * q2 + q3 *q3)* 57.295773f;
	arm_atan2_f32(2 * q2 * q3 + 2 * q0 * q1, q0 * q0 - q1 * q1 -  q2 * q2 + q3 * q3, roll);
	*roll *= 57.295773f;
	
//	*pitch = -asin( 2 * q1 * q3 -2 * q0* q2)*57.295773f;
//  asin(x) = atan(x/sqrt(1-x*x))
	sintemp = 2 * q1 * q3 -2 * q0* q2;
	arm_sqrt_f32(1 - sintemp * sintemp, &costemp);
	arm_atan2_f32(sintemp, costemp, pitch);
	*pitch *= -57.295773f;
	
//	*yaw =  atan2(2*(q1*q2 + q0*q3),q0*q0 +q1*q1-q2*q2 -q3*q3)*57.295773f;
	arm_atan2_f32(2 * (q1*q2 + q0*q3), q0*q0 +q1*q1-q2*q2 -q3*q3, yaw);
	*yaw  *= 57.295773f;
	/* 角度解算end */

	/* 速度解算begin */
	q0temp_ = q0_init;
	q1temp_ = q1_init;
	q2temp_ = q2_init;
	q3temp_ = q3_init;
	
	q0_ = q0temp_ + (-q1temp_*gx_ - q2temp_*gy_ -q3temp_*gz_)*halfT;
	q1_ = q1temp_ + ( q0temp_*gx_ + q2temp_*gz_ -q3temp_*gy_)*halfT;
	q2_ = q2temp_ + ( q0temp_*gy_ - q1temp_*gz_ +q3temp_*gx_)*halfT;
	q3_ = q3temp_ + ( q0temp_*gz_ + q1temp_*gy_ -q2temp_*gx_)*halfT;
	
	norm = inVSqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
	q0_ = q0_ * norm;
	q1_ = q1_ * norm;
	q2_ = q2_ * norm;
	q3_ = q3_ * norm;
	
//	*roll_ = atan2(2 * q2_ * q3_ + 2 * q0_ * q1_,q0_*q0_ - q1_ * q1_ -  q2_ * q2_ + q3_ *q3_)* 57.295773f;
	arm_atan2_f32(2 * q2_ * q3_ + 2 * q0_ * q1_,q0_*q0_ - q1_ * q1_ -  q2_ * q2_ + q3_ *q3_, roll_);
	*roll_ *= 57.295773f;
	
//	*pitch_ = -asin( 2 * q1_ * q3_ -2 * q0_ * q2_)*57.295773f;
//  asin(x) = atan(x/sqrt(1-x*x))
	sintemp_ = 2 * q1_ * q3_ -2 * q0_ * q2_;
	arm_sqrt_f32(1 - sintemp_ * sintemp_, &costemp_);
	arm_atan2_f32(sintemp_, costemp_, pitch_);
	*pitch_ *= -57.295773f;
	
//	*yaw_ =  atan2(2*(q1_*q2_ + q0_*q3_),q0_*q0_ +q1_*q1_-q2_*q2_ -q3_*q3_)*57.295773f;
	arm_atan2_f32(2*(q1_*q2_ + q0_*q3_),q0_*q0_ +q1_*q1_-q2_*q2_ -q3_*q3_, yaw_);
	*yaw_  *= 57.295773f;
	
	/* 速度解算end */
	
	return 0;
}

void BMI_Get_Acceleration(float pitch, float roll, float yaw,\
             float ax, float ay, float az,\
             float *accx, float *accy, float *accz)
{
 float imu_accx, imu_accy, imu_accz;
 
 pitch *= (double)0.017453;
 yaw *= (double)0.017453;
 roll *= (double)0.017453;
 
 ax = lsb_to_mps2(ax,2,bmi270.resolution);
 ay = lsb_to_mps2(ay,2,bmi270.resolution);
 az = lsb_to_mps2(az,2,bmi270.resolution);
 
 imu_accx = ax + arm_sin_f32(pitch) * GRAVITY_EARTH;
 imu_accy = ay - arm_sin_f32(roll) * arm_cos_f32(pitch) * GRAVITY_EARTH;
 imu_accz = az - arm_cos_f32(roll) * arm_cos_f32(pitch) * GRAVITY_EARTH;
 
 *accx = imu_accx * arm_cos_f32(pitch) + imu_accz * arm_sin_f32(pitch);
 *accy = imu_accy * arm_cos_f32(roll) - imu_accz * arm_sin_f32(roll);
 *accz = imu_accz * arm_cos_f32(pitch) * arm_cos_f32(roll) - imu_accx * arm_sin_f32(pitch) * arm_cos_f32(roll) \
     + imu_accy * arm_sin_f32(roll) * arm_cos_f32(pitch);
}
