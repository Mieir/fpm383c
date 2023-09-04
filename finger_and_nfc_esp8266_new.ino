#include <EEPROM.h>
#include <SPI.h>
#define BLINKER_MIOT_OUTLET
#include <MFRC522.h>
#include <BLINKER_PMSX003ST.h>
#include <Servo.h>
#define BLINKER_WIFI
#include <Blinker.h>
#include "stdio.h"
#include "SoftwareSerial.h"  
bool oState=false;
char auth[] = "80ca5126109c";    //手机端的key
char ssid[] = "神TMD211";     //WiFi名
char pswd[] = "/*1206733450*/";     //WiFi密码
int duoji=0;

// 新建组件对象
//BlinkerButton Button1("btn-num");   //位置1 按钮 数据键名
BlinkerButton Button2("btn-max");   //位置2 按钮 数据键名
BlinkerButton Button3("btn-tja");   //按钮添加nfc卡
BlinkerSlider Slider1("duoji");
BlinkerNumber NUM1("shuju");
Servo myservo;  //创建电机对象 
#define RST_PIN 5 // 配置针脚
#define SS_PIN 4 
MFRC522 mfrc522(SS_PIN, RST_PIN); // 创建新的RFID实例
//RC模块使用了D1.D2.D5.D6.D7
/*************************IO配置**************************/                 
int btn = 15;                      //D8(io15)按钮，保存门禁卡ID到EEPROM
/*************************数据**************************/
SoftwareSerial mySerial(3,1);    //软串口引脚，RX：GPIO3     TX：GPIO1

BlinkerButton Button_OneEnroll("OneEnroll");    //单次注册按钮
BlinkerButton Button_Delete("Delete");          //删除指纹按钮
BlinkerButton Button_Identify("Identify");      //搜索模式按钮
BlinkerButton Button_Empty("Empty");            //清空指纹按钮
BlinkerButton Button_MultEnroll("MultEnroll");  //连接注册按钮
BlinkerButton Button_Reset("Reset");            //复位模块按钮
BlinkerButton Button_disconnect("disconnect");  //断开WiFi按钮
//BlinkerButton Button_ON("ON");                  //手动开启继电器按钮
//BlinkerButton Button_OFF("OFF");                //手动关闭继电器按钮

char str[20];    //用于sprint函数的临时数组
int SearchID,EnrollID;    //搜索指纹的ID号和注册指纹的ID号
uint16_t ScanState = 0,WiFi_Connected_State = 0,ErrorNum = 0,PageID = 0;   //状态标志变量；WiFi是否连接状态标志位；扫描指纹错误次数标志位；输入ID号变量
uint8_t PS_ReceiveBuffer[20];   //串口接收数据的临时缓冲数组


//休眠协议
uint8_t PS_SleepBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x33,0x00,0x37};

//清空指纹协议
uint8_t PS_EmptyBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x0D,0x00,0x11};

//获取图像协议
uint8_t PS_GetImageBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x01,0x00,0x05};

//取消命令协议
uint8_t PS_CancelBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x30,0x00,0x34};

//生成模块协议
uint8_t PS_GetChar1Buffer[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,0x01,0x00,0x08};
uint8_t PS_GetChar2Buffer[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,0x02,0x00,0x09};

//RGB颜色控制协议
uint8_t PS_BlueLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x03,0x01,0x01,0x00,0x00,0x49};
uint8_t PS_RedLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x02,0x04,0x04,0x02,0x00,0x50};
uint8_t PS_GreenLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x02,0x02,0x02,0x02,0x00,0x4C};

//搜索指纹协议
uint8_t PS_SearchMBBuffer[17] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x04,0x01,0x00,0x00,0xFF,0xFF,0x02,0x0C};

//自动注册指纹协议
uint8_t PS_AutoEnrollBuffer[17] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x31,'\0','\0',0x04,0x00,0x16,'\0','\0'}; //PageID: bit 10:11，SUM: bit 15:16

//删除指纹协议
uint8_t PS_DeleteBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x0C,'\0','\0',0x00,0x01,'\0','\0'}; //PageID: bit 10:11，SUM: bit 14:15

//使用union结构，合并4个byte数据，转换为1个long
union long_byte
{
  long long_data;
  byte byte_data[4];
};
long_byte lb;
long EEPROM_RFID1 = -1;//EEPROM中保存的门禁卡ID1
long EEPROM_RFID2 = -1;
long EEPROM_RFID3 = -1;
long EEPROM_RFID4 = -1;
long EEPROM_RFID5 = -1;
long EEPROM_RFID6 = -1;
long EEPROM_RFID7 = -1;
long read_RFID = -1;      //当前读取到的门禁卡ID
int num ;
/**
  * @file    FPM383C.cpp
  * @brief   用在中断里面的延时函数
  * @param   ms：需要延时的毫秒数
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void delay_ms(long int ms)
{
  for(int i=0;i<ms;i++)
  {
    delayMicroseconds(1000);
  }
}


/**
  * @file    FPM383C.cpp
  * @brief   串口发送函数
  * @param   len: 发送数组长度
  * @param   PS_Databuffer[]: 需要发送的功能协议数组，在上面已有定义
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void FPM383C_SendData(int len,uint8_t PS_Databuffer[])
{
  mySerial.write(PS_Databuffer,len);
  mySerial.flush();
}


/**
  * @file    FPM383C.cpp
  * @brief   串口接收函数
  * @param   Timeout：接收超时时间
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void FPM383C_ReceiveData(uint16_t Timeout)
{
  uint8_t i = 0;
  while(mySerial.available() == 0 && (--Timeout))
  {
    delay(1);
  }
  while(mySerial.available() > 0)
  {
    delay(2);
    PS_ReceiveBuffer[i++] = mySerial.read();
    if(i > 15) break; 
  }
}


/**
  * @file    FPM383C.cpp
  * @brief   休眠函数，只有发送休眠后，模块的TOUCHOUT引脚才会变成低电平
  * @param   None
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void PS_Sleep()
{
  FPM383C_SendData(12,PS_SleepBuffer);
}


/**
  * @file    FPM383C.cpp
  * @brief   模块LED灯控制函数
  * @param   PS_ControlLEDBuffer[]：需要设置颜色的协议，一般定义在上面
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void PS_ControlLED(uint8_t PS_ControlLEDBuffer[])
{
  FPM383C_SendData(16,PS_ControlLEDBuffer);
}


/**
  * @file    FPM383C.cpp
  * @brief   模块任务取消操作函数，如发送了注册指纹命令，但是不想注册了，需要发送此函数
  * @param   None
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_Cancel()
{
  FPM383C_SendData(12,PS_CancelBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   模块获取搜索指纹用的图像函数
  * @param   None
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_GetImage()
{
  FPM383C_SendData(12,PS_GetImageBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   模块获取图像后生成特征，存储到缓冲区1
  * @param   None
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_GetChar1()
{
  FPM383C_SendData(13,PS_GetChar1Buffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   生成特征，存储到缓冲区2
  * @param   None
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_GetChar2()
{
  FPM383C_SendData(13,PS_GetChar2Buffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   搜索指纹模板函数
  * @param   None
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_SearchMB()
{
  FPM383C_SendData(17,PS_SearchMBBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   清空指纹模板函数
  * @param   None
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_Empty()
{
  FPM383C_SendData(12,PS_EmptyBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   自动注册指纹模板函数
  * @param   PageID：注册指纹的ID号，取值0 - 59
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_AutoEnroll(uint16_t PageID)
{
  PS_AutoEnrollBuffer[10] = (PageID>>8);
  PS_AutoEnrollBuffer[11] = (PageID);
  PS_AutoEnrollBuffer[15] = (0x54+PS_AutoEnrollBuffer[10]+PS_AutoEnrollBuffer[11])>>8;
  PS_AutoEnrollBuffer[16] = (0x54+PS_AutoEnrollBuffer[10]+PS_AutoEnrollBuffer[11]);
  FPM383C_SendData(17,PS_AutoEnrollBuffer);
  FPM383C_ReceiveData(10000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   删除指定指纹模板函数
  * @param   PageID：需要删除的指纹ID号，取值0 - 59
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_Delete(uint16_t PageID)
{
  PS_DeleteBuffer[10] = (PageID>>8);
  PS_DeleteBuffer[11] = (PageID);
  PS_DeleteBuffer[14] = (0x15+PS_DeleteBuffer[10]+PS_DeleteBuffer[11])>>8;
  PS_DeleteBuffer[15] = (0x15+PS_DeleteBuffer[10]+PS_DeleteBuffer[11]);
  FPM383C_SendData(16,PS_DeleteBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   二次封装自动注册指纹函数，实现注册成功闪烁两次绿灯，失败闪烁两次红灯
  * @param   PageID：注册指纹的ID号，取值0 - 59
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
/*，返回应答包的位9确认码。*/
uint8_t PS_Enroll(uint16_t PageID)
{
  if(PS_AutoEnroll(PageID) == 0x00)
  {
    PS_ControlLED(PS_GreenLEDBuffer);
    return PS_ReceiveBuffer[9];
  }
  PS_ControlLED(PS_RedLEDBuffer);
  return 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   分步式命令搜索指纹函数
  * @param   None
  * @return  应答包第9位确认码或者无效值0xFF
  * @version v1.0.0
  * @date    2022-08-10
  */
uint8_t PS_Identify()
{
  if(PS_GetImage() == 0x00)
  {
    if(PS_GetChar1() == 0x00)
    {
      if(PS_SearchMB() == 0x00)
      {
        if(PS_ReceiveBuffer[8] == 0x07 && PS_ReceiveBuffer[9] == 0x00)
        {
          PS_ControlLED(PS_GreenLEDBuffer);
          return PS_ReceiveBuffer[9];
        }
      }
    }
  }
  ErrorNum++;
  PS_ControlLED(PS_RedLEDBuffer);
  return 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   搜索指纹后的应答包校验，在此执行相应的功能，如开关继电器、开关灯等等功能
  * @param   ACK：各个功能函数返回的应答包
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void SEARCH_ACK_CHECK(uint8_t ACK)
{
  if(PS_ReceiveBuffer[6] == 0x07)
  {
    switch (ACK)
    {
      case 0x00:                          //指令正确
        SearchID = (int)((PS_ReceiveBuffer[10] << 8) + PS_ReceiveBuffer[11]);
        sprintf(str,"Now Search ID: %d",(int)SearchID);
        Blinker.notify(str);
        if(SearchID == 0) WiFi_Connected_State = 0;
        //digitalWrite(12,!digitalRead(12));
        opendoor(duoji);
        if(ErrorNum < 5) ErrorNum = 0;
        break;
    }
  }
  for(int i=0;i<20;i++) PS_ReceiveBuffer[i] = 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   注册指纹后返回的应答包校验
  * @param   ACK：注册指纹函数返回的应答包
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void ENROLL_ACK_CHECK(uint8_t ACK)
{
  if(PS_ReceiveBuffer[6] == 0x07)
  {
    switch (ACK)
    {
      case 0x00:                          //指令正确
        EnrollID = (int)((PS_AutoEnrollBuffer[10] << 8) + PS_AutoEnrollBuffer[11]);
        sprintf(str,"Now Enroll ID: %d",(int)EnrollID);
        Blinker.notify(str);
        break;
    }
  }
  for(int i=0;i<20;i++) PS_ReceiveBuffer[i] = 0xFF;
}


/**
  * @file    FPM383C.cpp
  * @brief   外部中断函数，触发中断后开启模块的LED蓝灯（代表正在扫描指纹），接着由搜索指纹函数修改成功（闪烁绿灯）或失败（闪烁红灯）
  * @param   None
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void ICACHE_RAM_ATTR InterruptFun()
{
  detachInterrupt(digitalPinToInterrupt(0));
  PS_ControlLED(PS_BlueLEDBuffer);
  delay_ms(10);
  ScanState |= 1<<4;
}


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 单次注册 “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void OneEnroll_callback(const String & state)
{
  Blinker.vibrate(500);
  ScanState |= 1<<2;
  Blinker.notify("OneEnroll Fingerprint");
}


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 删除指纹 “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void Delete_callback(const String & state)
{
  Blinker.vibrate(500);
  ScanState |= 1<<3;
  Blinker.notify("Delete Fingerprint");
}


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 搜索模式 “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void Identify_callback(const String & state)
{
  Blinker.vibrate(500);
  ScanState &= ~(1<<0);
  Blinker.notify("MultSearch Fingerprint");
}


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 清空指纹 “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void Empty_callback(const String & state)
{
  PageID = 0;
  Blinker.vibrate(500);
  Blinker.notify("Empty Fingerprint");
  if(PS_Empty() == 0x00)
  {
    PS_ControlLED(PS_GreenLEDBuffer);
  }
  else
  {
    PS_ControlLED(PS_RedLEDBuffer);
  }
}


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 连续注册 “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void MultEnroll_callback(const String & state)
{
  Blinker.vibrate(500);
  ScanState |= 0x01;
  Blinker.notify("MultEnroll Fingerprint");
}


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 复位模块 “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void Reset_callback(const String & state)
{
  Blinker.vibrate(500);
  Blinker.notify("Reset Fingerprint");
  PS_Cancel();
  delay(500);
  PS_Sleep();
  attachInterrupt(digitalPinToInterrupt(0),InterruptFun,RISING);
}


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 断开WiFi “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
/*void disconnect_callback(const String & state)
{
  ErrorNum = 0;
  Blinker.vibrate(500);
  Blinker.notify("WiFi Disable");
  Blinker.delay(200);
  Blinker.run();
  WiFi_Connected_State = 1;
  Blinker.vibrate(500);
  Blinker.notify("WiFi Connected");
}*/


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 手动开启 “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
/*/void ON_callback(const String & state)
{
  Blinker.vibrate(500);
  Blinker.notify("ON Relay");
  digitalWrite(12,HIGH);
}*/


/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 手动关闭 “ 按键
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
/*void OFF_callback(const String & state)
{
  Blinker.vibrate(500);
  Blinker.notify("OFF Relay");
  digitalWrite(12,LOW);
}
*/

/**
  * @file    FPM383C.cpp
  * @brief   点灯科技APP里面的 “ 对话框 “ 
  * @param   Unknown
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void DataRead(const String & data)
{
  PageID = data.toInt();
  ScanState |= 1<<1;
}
void slider1_callback(int32_t value)
{
BLINKER_LOG("get slider value: ", value);
duoji=value;
NUM1.print(duoji);

} 

/**
  * @file    FPM383C.cpp
  * @brief   初始化主函数
  * @param   None
  * @return  None
  * @version v1.0.0
  * @date    2022-08-10
  */
void setup()
{
  //ESP.wdtEnable(10000);
  //Serial.begin(115200);
    Serial.println("");
    mySerial.begin(57600);                              //软串口波特率，默认FPM383C指纹模块的57600，所以不需要动它

   // pinMode(2,OUTPUT);                                  //ESP8266，Builtin LED内置的灯引脚模式
    //pinMode(12,OUTPUT);                                 //继电器输出引脚
    pinMode(0,INPUT);                                  //FPM383C的2脚TouchOUT引脚，用于外部中断
    BLINKER_DEBUG.stream(Serial);
    Blinker.begin(auth, ssid, pswd);
    Blinker.attachData(DataRead);
    // Button1.attach(button1_callback);
     Button2.attach(button2_callback);
     Button3.attach(button3_callback);
     
    Slider1.attach(slider1_callback);
    // Blinker.attachData(dataRead);
     myservo.attach(2);  //servo.attach():设置舵机数据引脚
     myservo.write(0);  //servo.write():设置转动角度
     BlinkerMIOT.attachQuery(miotQuery);
     BlinkerMIOT.attachPowerState(miotPowerState);
     Button_OneEnroll.attach(OneEnroll_callback);
     Button_Delete.attach(Delete_callback);
     Button_Identify.attach(Identify_callback);
     Button_Empty.attach(Empty_callback);
     Button_MultEnroll.attach(MultEnroll_callback);
     Button_Reset.attach(Reset_callback);
     //Button_disconnect.attach(disconnect_callback);
     //Button_ON.attach(ON_callback);
     //Button_OFF.attach(OFF_callback);
     

  delay_ms(200);                                      //用于FPM383C模块启动延时，不可去掉
  PS_Sleep();
  delay_ms(200);

  attachInterrupt(digitalPinToInterrupt(0),InterruptFun,RISING);     //外部中断初始化
  //读取EEPROM索引的值            我这里设置了七张卡，不够也可以再加
  for (int i = 0; i < 4; i++)
  {
    lb.byte_data[i] = EEPROM.read(i);   //卡1
  }
  EEPROM_RFID1 = lb.long_data;
  
  for (int i = 0  ; i<4 ; i++ )
  {
    lb.byte_data[i] = EEPROM.read(i+4);   //卡2
  }
  EEPROM_RFID2 = lb.long_data;
 
  for (int i = 0  ; i<4 ; i++ )
  {
    lb.byte_data[i] = EEPROM.read(i+8);   //卡3
  }
  EEPROM_RFID3 = lb.long_data;
 
  for (int i = 0  ; i<4 ; i++ )
  {
    lb.byte_data[i] = EEPROM.read(i+12);    //卡4
  }
  EEPROM_RFID4 = lb.long_data;
 
  for (int i = 0  ; i<4 ; i++ )
  {
    lb.byte_data[i] = EEPROM.read(i+16);   //卡5
  }
  EEPROM_RFID5 = lb.long_data;
 
  for (int i = 0  ; i<4 ; i++ )
  {
    lb.byte_data[i] = EEPROM.read(i+20);   //卡6
  }
  EEPROM_RFID6 = lb.long_data;
 
  for (int i = 0  ; i<4 ; i++ )
  {
    lb.byte_data[i] = EEPROM.read(i+24);    //卡7
  }
  EEPROM_RFID7 = lb.long_data;
 
  num = EEPROM.read(30);
 // IO_init(); //设置读卡器添加按钮的电平状态
  RFID_init();//初始化读卡器
}
 
void loop()
{
  //ESP.wdtFeed();
  Blinker.run();
 // Btn_Event();                 //按钮状态   添加nfc
  RFID_read();               //读卡并做分析处理
  switch (ScanState)
  {
    //第一步
    case 0x10:    //搜索指纹模式
        SEARCH_ACK_CHECK(PS_Identify());
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(0),InterruptFun,RISING);
    break;

    //第二步
    case 0x11:    //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        Blinker.notify("Please Enter ID First");
        PS_ControlLED(PS_RedLEDBuffer);
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(0),InterruptFun,RISING);
    break;
    
    //第三步
    case 0x12:    //指纹中断提醒按下功能按键，执行完毕返回搜索指纹模式
        Blinker.notify("Please Press Enroll or Delete Key");
        PS_ControlLED(PS_RedLEDBuffer);
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(0),InterruptFun,RISING);
    break;

    //第四步
    case 0x13:    //连续搜索指纹模式，每次搜索前都必须由APP发送指纹ID，由函数将ScanState bit1置位才进入下一次搜索，否则提醒输入指纹ID并返回搜索模式
        ENROLL_ACK_CHECK(PS_Enroll(PageID));
        delay(1000);
        PS_Sleep();
        ScanState = 0x01;
        attachInterrupt(digitalPinToInterrupt(0),InterruptFun,RISING);
    break;

    //第五步
    case 0x14:    //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        ScanState = 0x11;   //返回第二步，提示输入指纹ID
    break;

    //第六步
    case 0x16:    //单次指纹注册模式，必须同时满足按下单次注册按键且已输入ID情况下才会执行
        ENROLL_ACK_CHECK(PS_Enroll(PageID));
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(0),InterruptFun,RISING);
    break;

    //第七步
    case 0x08:    //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        ScanState = 0x11;   //返回第二步，提示输入指纹ID
    break;

    //第八步
    case 0x0A:    //单独指纹删除模式
        if(PS_Delete(PageID) == 0x00)
        {
          Blinker.notify("Delete Success");
          PS_ControlLED(PS_GreenLEDBuffer);
        }
        ScanState = 0x00;
    break; 
  }

}
 
//void button1_callback(const String & state) {    //位置1按钮
//   
//    //yunxing();
//    Blinker.vibrate();
//}
  
void button2_callback(const String & state) {   //位置2按钮
    //BLINKER_LOG("get button state: ", button2);
    Blinker.vibrate(500); 
    Blinker.notify("开门");
  opendoor(duoji);
}

/*void  button3_callback(const String & state) {  //blinker按钮添加nfc卡
        //BLINKER_LOG("get button state: ", state);
       button3_3();
           
 
}
int button3_3(void){
  
  
           if(state=="tap"){
          return 1;          
           }
           else{
           return 0;
}
  }
  */
void button3_callback(const String & state) { //blinker按钮添加nfc卡
    //BLINKER_LOG("get button state: ", state);
    Blinker.vibrate(500);
    Blinker.notify("添加nfc卡");
    EEPROM.begin(1024);
     delay(200);
    if (read_RFID == -1)
    {
      Serial.println("当前未读卡");
    }
    
    else{
      switch(num){
     
     case 0 :   
              lb.long_data = read_RFID;
              EEPROM_RFID1 = lb.long_data;
              for (int i = 0; i < 4; i++)
              {
                EEPROM.write(i, lb.byte_data[i]);
              }
              num = 1;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID1已保存");
              break;
    case 1 :
              lb.long_data = read_RFID;
              EEPROM_RFID2 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+4, lb.byte_data[i]);
              }
              num=2;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID2已保存");
              break;
 
    case 2 :
              lb.long_data = read_RFID;
              EEPROM_RFID3 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+8, lb.byte_data[i]);
              }
              num=3;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID3已保存");
              break;
    case 3 :
              lb.long_data = read_RFID;
              EEPROM_RFID4 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+12, lb.byte_data[i]);
              }
              num=4;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID4已保存");
              break;
 
    case 4 :
              lb.long_data = read_RFID;
              EEPROM_RFID5 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+16, lb.byte_data[i]);
              }
              num=5;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID5已保存");
              break;
    case 5 :
              lb.long_data = read_RFID;
              EEPROM_RFID6 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+20, lb.byte_data[i]);
              }
              num=6;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID6已保存");
              break;
    case 6 :
              lb.long_data = read_RFID;
              EEPROM_RFID7 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+24, lb.byte_data[i]);
              }
              num=0;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID7已保存");
              break;
    
      }
    }
 }
    

void opendoor(int duoji){
if(duoji==0){
  duoji=70;
}

  myservo.write(duoji);         //控制舵机旋转到180度
  delay(3000);                //延迟3000毫秒
  myservo.write(0);//控制舵机旋转到0度
  }
 
 
 

 
/*void dataRead(const String & data)
{
    BLINKER_LOG("Blinker readString: ", data);
 
    //Blinker.vibrate();
    
    //uint32_t BlinkerTime = millis();
    //Blinker.print(BlinkerTime);
   
}
 */
 
/*监听按钮状态，更换显示状态和值
  /**void Btn_Event()
{
  EEPROM.begin(1024);
  //io15默认的有下拉，所以要判断是否为高电平
  if (digitalRead(btn) == 1)
  {
    delay(200);
    if (read_RFID == -1)
    {
      Serial.println("当前未读卡");
    }
    
    else{
      switch(num){
     
     case 0 :   
              lb.long_data = read_RFID;
              EEPROM_RFID1 = lb.long_data;
              for (int i = 0; i < 4; i++)
              {
                EEPROM.write(i, lb.byte_data[i]);
              }
              num = 1;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID1已保存");
              break;
    case 1 :
              lb.long_data = read_RFID;
              EEPROM_RFID2 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+4, lb.byte_data[i]);
              }
              num=2;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID2已保存");
              break;
 
    case 2 :
              lb.long_data = read_RFID;
              EEPROM_RFID3 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+8, lb.byte_data[i]);
              }
              num=3;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID3已保存");
              break;
    case 3 :
              lb.long_data = read_RFID;
              EEPROM_RFID4 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+12, lb.byte_data[i]);
              }
              num=4;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID4已保存");
              break;
 
    case 4 :
              lb.long_data = read_RFID;
              EEPROM_RFID5 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+16, lb.byte_data[i]);
              }
              num=5;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID5已保存");
              break;
    case 5 :
              lb.long_data = read_RFID;
              EEPROM_RFID6 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+20, lb.byte_data[i]);
              }
              num=6;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID6已保存");
              break;
    case 6 :
              lb.long_data = read_RFID;
              EEPROM_RFID7 = lb.long_data;
              for (int i = 0 ; i < 4; i++ )
              {
                EEPROM.write(i+24, lb.byte_data[i]);
              }
              num=0;
              EEPROM.write(30, num);
              EEPROM.commit();//执行保存EEPROM
              Serial.println("门禁卡ID7已保存");
              break;
    
      }
    }
  }
}
 */

/***void IO_init()
{
  //io15默认有硬件下拉，不要使用软件上拉模式
  pinMode(btn, INPUT);
 // digitalWrite(button3, 0);
//  pinMode(Buzzer, OUTPUT);
//  digitalWrite(Buzzer, 0);
}
*/

/***********************RFID读卡自定义函数***********************/
//初始化读卡
void RFID_init()
{ 
  SPI.begin();        // SPI开始
  mfrc522.PCD_Init(); // 初始化
  Serial.println("初始化读卡");
}
 
//运行读卡
void RFID_read()
{
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial())
  {
    return;
  }
  else
  {
    read_RFID = RFID_toLong(mfrc522.uid.uidByte);
    RFID_Handler(read_RFID);
    mfrc522.PICC_HaltA();//停止 PICC
    mfrc522.PCD_StopCrypto1(); //停止加密PCD
    return;
  }
}
 
//用户ID转换类型
long RFID_toLong(byte *by)
{
  long data;
  for (int i = 0; i < 4; i++)
  {
    lb.byte_data[i] = by[i];
  }
  data = lb.long_data;
  return data;
}
 
//把当前读到卡的ID，对比EEPROM中的ID
void RFID_Handler(long data)
{
  Serial.println(data);
  if (EEPROM_RFID1 == -1&&EEPROM_RFID2 == -1&&EEPROM_RFID3 == -1&&EEPROM_RFID4 == -1&&
  EEPROM_RFID5 == -1&&EEPROM_RFID6 == -1&&EEPROM_RFID7 == -1)
  {
    Serial.println("当前未设置卡");
  }
  else
  {
    if (data == EEPROM_RFID1) {
      Serial.println("ID1正确，验证通过");
      opendoor(duoji);   //开门函数
    }
    else if(data == EEPROM_RFID2){
      Serial.println("ID2正确，验证通过");
       opendoor(duoji);
      }
 
    else if(data == EEPROM_RFID3){
      Serial.println("ID3正确，验证通过");
      opendoor(duoji);
      }
 
    else if(data == EEPROM_RFID4){
      Serial.println("ID4正确，验证通过");
       opendoor(duoji);
      }
    else if(data == EEPROM_RFID5){
      Serial.println("ID5正确，验证通过");
       opendoor(duoji);
      }
 
    else if(data == EEPROM_RFID6){
      Serial.println("ID6正确，验证通过");
       opendoor(duoji);
      }
 
    else if(data == EEPROM_RFID7){
      Serial.println("ID7正确，验证通过");
      opendoor(duoji);
      }  
    else
    {
      Serial.println("ID错误，验证失败");
    }
  }
}




void miotPowerState(const String & state)//
{
   
 BLINKER_LOG("get button state: ", state);
           
       

    if (state == BLINKER_CMD_ON) {
        myservo.attach(2);
        myservo.write(60);         //控制舵机旋转到180度
       delay(3000);                //延迟3000毫秒
       myservo.write(0);
        oState=true; 
        
   
    BlinkerMIOT.powerState("on");
        BlinkerMIOT.print();
    }
    else if(state == BLINKER_CMD_OFF){
      BlinkerMIOT.powerState("off");
      }
}
uint32_t os_time = 0;
void miotQuery(int32_t queryCode)//小爱状态反馈函数
{
    BLINKER_LOG("MIOT Query codes: ", queryCode);

    switch (queryCode)
    {
        case BLINKER_CMD_QUERY_ALL_NUMBER :
            BLINKER_LOG("MIOT Query ALL");
            BlinkerMIOT.powerState(oState ? "on" : "off");
            BlinkerMIOT.print();
            if(oState==true){
              oState=false;
              }
              else{
                oState==true;
                }
            break;
      

       
    }
}
