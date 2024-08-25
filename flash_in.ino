#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
//在这里设置SDA和SCL的引脚，还有别忘记在下面更改i2c频道
#define SDA_GPIO 26
#define SCL_GPIO 27
//是否开启调试模式，调试模式用于检测指定地址上设备，用于检测程序是否烧入成功
#define DEBUGGING false

//这个是3116默认的地址，烧入配置之前的地址，默认0x37，你可以修改为你想烧录的地址
uint8_t default_address = 0x37;
//这个是调试设备的地址
uint8_t debug_address = 0x39;
//创建一个i2c设备对象，这个对象的地址是上面的默认地址
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(default_address,&Wire1);
//这个是3116的配置文件，根据你生产的文件来，使用Ezclick
const uint8_t config[128] = {
    0xFFu, 0x0Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x54u, 0x00u, 0x80u, 0x80u, 0x80u, 0x80u,
    0x80u, 0x80u, 0x80u, 0x80u, 0x80u, 0x80u, 0x80u, 0x80u,
    0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x01u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,
    0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x1Eu, 0x1Eu, 0x00u,
    0x00u, 0x1Eu, 0x1Eu, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u,
    0x00u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
    0xFFu, 0x00u, 0x00u, 0x00u, 0x00u, 0x03u, 0x01u, 0x48u,
    0x00u, 0x40u, 0x01u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x39u, 0xB2u
};

void debug(){
  Adafruit_I2CDevice debug_dev = Adafruit_I2CDevice(debug_address,&Wire1);
  while(!debug_dev.begin()){
    Serial.print("没有设备在 0x");
    Serial.println(debug_dev.address(), HEX);
  }
  Serial.print("发现设备在 0x");
  Serial.println(debug_dev.address(), HEX);
  while(1);
}
void step1_ackChip(){
  delay(500);
  //检测地址上设备是否返回收到的1指令,如果没有，则在串口内卡死
  //并且报错
  while(!i2c_dev.begin()){
    Serial.print("没有设备在 0x");
    Serial.println(i2c_dev.address(), HEX);
  }
  Serial.print("发现设备在 0x");
  Serial.println(i2c_dev.address(), HEX);

  //检测0x37地址上设备的0x51寄存器内容
  Adafruit_BusIO_Register id_reg = Adafruit_BusIO_Register(&i2c_dev, 0x51, 1, LSBFIRST);
  uint16_t add = 0;
  for(int i=0;i<128;i++){
    add = id_reg.read();
  }
  //检测寄存器地址和i2c地址是否相同，如果相同，则为3116芯片
  if(!default_address==add){
    Serial.print("该芯片不是cyc8cmbr3116!");
    while (1);
  }
}

void step2_checkSiliconID(){
  //暂时不做这一步，基本不会出错
}

void step3_proggram_flash(){
  //向着CONFIG_CRC寄存器写入128位配置，地址为0x7e
  
  for(int i =0;i<128;i++){
    while(!i2c_dev.begin()){}
    Adafruit_BusIO_Register cfg_reg = Adafruit_BusIO_Register(&i2c_dev, i, 1, LSBFIRST);
    cfg_reg.write(config[i]);
    delay(1);
  }
  //给CTRL_CMD发送命令，检查CRC并保存，地址0x86，写入2
  Adafruit_BusIO_Register cmd_reg = Adafruit_BusIO_Register(&i2c_dev, 0x86, 1, LSBFIRST);
  cmd_reg.write(2);
  delay(100);
  //检查刚刚执行的命令是否成功，不成功则报错，CTRL_CMD_ERR，地址0x89
  Adafruit_BusIO_Register cmdcheck_reg = Adafruit_BusIO_Register(&i2c_dev, 0x89, 1, LSBFIRST);
  int flag = 0;
  for(int i=0;i<128;i++){
    flag = cmdcheck_reg.read();
  }
  if(flag==253){
    Serial.println("在烧录程序中出错！写入闪存失败，错误代码253");
    while (1);
  }
    if(flag==254){
    Serial.println("在烧录程序中出错！校验失败，错误代码254");
    while (1);
  }
    if(flag==255){
    Serial.println("在烧录程序中出错！命令无效，错误代码254");
    while (1);
  }
  //一切烧录完成，给芯片软重启信号，向CTRL_CMD，地址0x86发送255
  cmd_reg.write(255);
  Serial.println("烧录已完成！");
  while(1);
}

void setup(){
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Wire1.setSDA(SDA_GPIO);
  Wire1.setSCL(SCL_GPIO);
  Wire1.setClock(400000);

  if(DEBUGGING){
    debug();
  }
  
  step1_ackChip();
  step2_checkSiliconID();
  step3_proggram_flash();
}

void loop(){

}