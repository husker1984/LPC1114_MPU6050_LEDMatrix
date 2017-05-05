//ヘッダファイル
#include "mbed.h"
#include "math.h"

//マクロ
#define SERIAL_BAUDRATE         115200
#define I2C_FRQ_STD             100000
#define I2C_FRQ_HIGH            400000
#define TIMER_INTERVAL_US       1000          //timer interval(micro sec)
// 加速度/ジャイロセンサーの制御定数
#define MPU6050_ADDR            (0x68 << 1)     // MPU-6050 device address
#define MPU6050_SMPLRT_DIV      0x19            // MPU-6050 register address
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_WHO_AM_I        0x75
//LED MATRIX
#define LEDMTX_SIZE_ROW         8
#define LEDMTX_SIZE_COL         7


//関数
void timer_IRQHandler(void);
void mpu6050_init(void);
void mpu6050_write(char addr, char data);
uint8_t mpu6050_read(char reg);
void ledmatrix_init(void);


//クラス
Ticker timer;
Serial pc(USBTX, USBRX);
I2C i2c(dip5, dip27);

DigitalOut  led_rows[LEDMTX_SIZE_ROW]  = {//ROW=アノード
    DigitalOut( dip14 ),    //ROW_1, LEDPIN_9   ok
    DigitalOut( dip26 ),    //ROW_2, LEDPIN_14  ok
    DigitalOut( dip13 ),    //ROW_3, LEDPIN_8   ok
    DigitalOut( dip24 ),    //ROW_4, LEDPIN_12  ok
    DigitalOut( dip1  ),    //ROW_5, LEDPIN_1   ok
    DigitalOut( dip11 ),    //ROW_6, LEDPIN_7   ok
    DigitalOut( dip2  ),    //ROW_7, LEDPIN_2   ok
    DigitalOut( dip9  ),    //ROW_8, LEDPIN_5   ok
};

DigitalOut  led_cols[LEDMTX_SIZE_COL]  = {//COL=カソード
    DigitalOut( dip25 ),    //COL_1, LEDPIN_13  ok
    DigitalOut( dip4  ),    //COL_3, LEDPIN_4   ok
    DigitalOut( dip6  ),    //COL_4, LEDPIN_10  ok
    DigitalOut( dip17 ),    //COL_5, LEDPIN_6   ok
    DigitalOut( dip10 ),    //COL_6, LEDPIN_11  ok
    DigitalOut( dip18 ),    //COL_7, LEDPIN_15  ok
    DigitalOut( dip28 ),    //COL_8, LEDPIN_16  ok
};


//グローバル変数
int flip = 0, flip_last = 0;
// Specify sensor full scale
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

int Gscale = GFS_250DPS;
int Ascale = AFS_4G;



/** メイン関数
 *
 *  @param なし
 *  @deprecated
 *      加速度センサを読み取り，その値でLEDマトリクスの点灯位置を変更する
 *      LEDはタイマー割込みでダイナミック点灯
 */
int main(){

    //変数
    int16_t data_2bytes[8] = {0};
    float temp;
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float degRoll, degPitch;

    //初期化
    pc.baud(SERIAL_BAUDRATE);
    i2c.frequency(I2C_FRQ_STD);
//    mpu6050_init();
    ledmatrix_init();

    //タイマー割込み開始
    timer.attach_us(&timer_IRQHandler, TIMER_INTERVAL_US);     //割込み処理関数, タイマー時間

    //LED pattern
    led_rows[0] = 1;
    led_rows[4] = 1;

    //データ取得
    pc.printf("Temp \t AccX \t AccY \t AccZ \t GyroX \t GyroY \t GyroZ \t DegRoll \t DegPitch \n");
    while(1){
        data_2bytes[0] = (int16_t)( ((int16_t)(mpu6050_read(MPU6050_TEMP_OUT_H))   << 8) | mpu6050_read(MPU6050_TEMP_OUT_L) );
        data_2bytes[1] = (int16_t)( ((int16_t)(mpu6050_read(MPU6050_ACCEL_XOUT_H)) << 8) | mpu6050_read(MPU6050_ACCEL_XOUT_L) );
        data_2bytes[2] = (int16_t)( ((int16_t)(mpu6050_read(MPU6050_ACCEL_YOUT_H)) << 8) | mpu6050_read(MPU6050_ACCEL_YOUT_L) );
        data_2bytes[3] = (int16_t)( ((int16_t)(mpu6050_read(MPU6050_ACCEL_ZOUT_H)) << 8) | mpu6050_read(MPU6050_ACCEL_ZOUT_L) );
        data_2bytes[4] = (int16_t)( ((int16_t)(mpu6050_read(MPU6050_GYRO_XOUT_H))  << 8) | mpu6050_read(MPU6050_GYRO_XOUT_L) );
        data_2bytes[5] = (int16_t)( ((int16_t)(mpu6050_read(MPU6050_GYRO_YOUT_H))  << 8) | mpu6050_read(MPU6050_GYRO_YOUT_L) );
        data_2bytes[6] = (int16_t)( ((int16_t)(mpu6050_read(MPU6050_GYRO_ZOUT_H))  << 8) | mpu6050_read(MPU6050_GYRO_ZOUT_L) );

        temp   = (float)( data_2bytes[0] )/340.0 + 36.53;
        acc_x  = (float)( data_2bytes[1] )/8192.0;
        acc_y  = (float)( data_2bytes[2] )/8192.0;
        acc_z  = (float)( data_2bytes[3] )/8192.0;
//        gyro_x = (float)( data_2bytes[4] )/131.0;
//        gyro_y = (float)( data_2bytes[5] )/131.0;
//        gyro_z = (float)( data_2bytes[6] )/131.0;

        degRoll = atan2(acc_x, acc_z) * 360 / 2.0 / M_PI *100.0;
//        degPitch = atan(-acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * 360 / 2.0 / M_PI;

        //serial output
        pc.printf("%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n", (int)temp, (int)acc_x, (int)acc_y, (int)acc_z, (int)gyro_x, (int)gyro_y, (int)gyro_z, (int)degRoll, (int)degPitch);

        wait(1);

    }

}

/** 加速度/ジャイロセンサーの初期化
 *  @param なし
 *  @deprecated
 *
 */
void mpu6050_init(void){

    // Check Device
    if (mpu6050_read(MPU6050_WHO_AM_I) != 0x68) {
        pc.printf("\nWHO_AM_I error.");
        while (true) ;
    }

    // Initialize MPU6050 device
    // wake up device
    mpu6050_write(MPU6050_PWR_MGMT_1, 0x00);                // Clear sleep mode bit (6), enable all sensors
    wait(0.1);                                              // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

    // get stable time source
    mpu6050_write(MPU6050_PWR_MGMT_1, 0x01);                // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
    mpu6050_write(MPU6050_CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    mpu6050_write(MPU6050_SMPLRT_DIV, 0x04);                // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
     uint8_t c =  mpu6050_read(MPU6050_GYRO_CONFIG);
     mpu6050_write(MPU6050_GYRO_CONFIG, c & ~0xE0);         // Clear self-test bits [7:5]
     mpu6050_write(MPU6050_GYRO_CONFIG, c & ~0x18);         // Clear AFS bits [4:3]
     mpu6050_write(MPU6050_GYRO_CONFIG, c | Gscale << 3);   // Set full scale range for the gyro

    // Set accelerometer configuration
     c =  mpu6050_read(MPU6050_ACCEL_CONFIG);
     mpu6050_write(MPU6050_ACCEL_CONFIG, c & ~0xE0);        // Clear self-test bits [7:5]
     mpu6050_write(MPU6050_ACCEL_CONFIG, c & ~0x18);        // Clear AFS bits [4:3]
     mpu6050_write(MPU6050_ACCEL_CONFIG, c | Ascale << 3);  // Set full scale range for the accelerometer



/*
    mpu6050_write(MPU6050_PWR_MGMT_1, 0x01);     // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    mpu6050_write(MPU6050_SMPLRT_DIV, 0x07);     // sample rate: 8kHz/(7+1) = 1kHz
    mpu6050_write(MPU6050_CONFIG, 0x00);         // disable DLPF, gyro output rate = 8kHz
    mpu6050_write(MPU6050_GYRO_CONFIG, 0x00);    // gyro range: ±250dps
    mpu6050_write(MPU6050_ACCEL_CONFIG, 0x00);   // accel range: ±2g
*/



    return;
}

/** 加速度/ジャイロセンサーへデータ書込み
 *  @param char reg  :書込み対象レジスタ
 *  @param char data :書込みデータ
 *  @deprecated
 *
 */
void mpu6050_write(char reg, char data){

    char write_data[2];
    char error;

    //set write data
    write_data[0] = reg;    //Resister Address
    write_data[1] = data;   //Data

    //write data
    error = i2c.write(MPU6050_ADDR, write_data, 2);

    //error check
    if(error != 0){
        pc.printf("ERROR: WRITE_MPU6050\n");
    }

    return;
}

/** 加速度/ジャイロセンサーからデータ読込み
 *  @param char reg :読込み対象レジスタ
 *  @deprecated
 *
 */
uint8_t mpu6050_read(char reg){

    char data;
    char error;

    //write register address to read
    error = i2c.write(MPU6050_ADDR, &reg, 1, true);

    //read data
    error = i2c.read(MPU6050_ADDR, &data, 1);

    //error check
    if(error != 0){
        pc.printf("ERROR: READ_MPU6050\n");
        return (uint8_t)error;
    }
    else{
        return (uint8_t)data;
    }
}

/** LEDマトリクスの初期化
 *  @param なし
 *  @deprecated
 *
 */
void ledmatrix_init(void){

    unsigned int i;

    //reset raws and cols
    for(i=1; i<=8; i++) led_rows[i-1] = 0;
    for(i=1; i<=8; i++) led_cols[i-1] = 1;

    return;
}

/** 加速度/ジャイロセンサーへのデータ書込み
 *  @param char reg :書込み対象レジスタ
 *  @param char reg :書込みデータ
 *  @deprecated
 *
 */
void timer_IRQHandler(void){

    static unsigned int current_col = 0;

    led_cols[current_col] = 0;

    if(current_col==0){
        led_cols[LEDMTX_SIZE_COL-1] = 1;
    }
    else{
        led_cols[current_col-1] = 1;
    }

    switch (current_col){
    case 0:
        led_rows[0] = 1;
        led_rows[1] = 0;
        led_rows[2] = 0;
        led_rows[3] = 0;
        led_rows[4] = 0;
        led_rows[5] = 0;
        led_rows[6] = 0;
        led_rows[7] = 0;
        break;
    case 1:
        led_rows[0] = 0;
        led_rows[1] = 1;
        led_rows[2] = 0;
        led_rows[3] = 0;
        led_rows[4] = 0;
        led_rows[5] = 0;
        led_rows[6] = 0;
        led_rows[7] = 0;
        break;
    case 2:
        led_rows[0] = 0;
        led_rows[1] = 0;
        led_rows[2] = 1;
        led_rows[3] = 0;
        led_rows[4] = 0;
        led_rows[5] = 0;
        led_rows[6] = 0;
        led_rows[7] = 0;
        break;
    case 3:
        led_rows[0] = 0;
        led_rows[1] = 0;
        led_rows[2] = 0;
        led_rows[3] = 1;
        led_rows[4] = 0;
        led_rows[5] = 0;
        led_rows[6] = 0;
        led_rows[7] = 0;
        break;
    case 4:
        led_rows[0] = 0;
        led_rows[1] = 0;
        led_rows[2] = 0;
        led_rows[3] = 0;
        led_rows[4] = 1;
        led_rows[5] = 0;
        led_rows[6] = 0;
        led_rows[7] = 0;
        break;
    case 5:
        led_rows[0] = 0;
        led_rows[1] = 0;
        led_rows[2] = 0;
        led_rows[3] = 0;
        led_rows[4] = 0;
        led_rows[5] = 1;
        led_rows[6] = 0;
        led_rows[7] = 0;
        break;
    case 6:
        led_rows[0] = 0;
        led_rows[1] = 0;
        led_rows[2] = 0;
        led_rows[3] = 0;
        led_rows[4] = 0;
        led_rows[5] = 0;
        led_rows[6] = 1;
        led_rows[7] = 0;
        break;
    case 7:
        led_rows[0] = 0;
        led_rows[1] = 0;
        led_rows[2] = 0;
        led_rows[3] = 0;
        led_rows[4] = 0;
        led_rows[5] = 0;
        led_rows[6] = 0;
        led_rows[7] = 1;
        break;
    default:
        led_rows[0] = 0;
        led_rows[1] = 0;
        led_rows[2] = 0;
        led_rows[3] = 0;
        led_rows[4] = 0;
        led_rows[5] = 0;
        led_rows[6] = 0;
        led_rows[7] = 0;
        break;
    }

    current_col++;
    if(current_col==LEDMTX_SIZE_COL){
        //return to 0
        current_col = 0;
    }
}
