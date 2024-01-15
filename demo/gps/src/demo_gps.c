
#include <string.h>
#include <stdio.h>
#include <api_os.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_debug.h>
#include "buffer.h"
#include "gps_parse.h"
#include "math.h"
#include "gps.h"
#include <api_mqtt.h>
#include "api_network.h"
#include "api_socket.h"
#include "demo_mqtt.h"

#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "GPS Test Task"

static HANDLE gpsTaskHandle = NULL;
static HANDLE semMqttStart = NULL;
bool flag = true;

typedef enum{
    MQTT_EVENT_CONNECTED = 0,
    MQTT_EVENT_DISCONNECTED ,
    MQTT_EVENT_MAX
}MQTT_Event_ID_t;

typedef struct {
    MQTT_Event_ID_t id;
    MQTT_Client_t* client;
}MQTT_Event_t;

typedef enum{
    MQTT_STATUS_DISCONNECTED = 0,
    MQTT_STATUS_CONNECTED       ,
    MQTT_STATUS_MAX
}MQTT_Status_t;

MQTT_Status_t mqttStatus = MQTT_STATUS_DISCONNECTED;

GPIO_config_t gpioLedBlue = {
    .mode         = GPIO_MODE_OUTPUT,
    .pin          = GPIO_PIN27,
    .defaultLevel = GPIO_LEVEL_LOW
};

void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case API_EVENT_ID_NO_SIMCARD:
            Trace(1,"!!NO SIM CARD%d!!!!",pEvent->param1);
            break;

        case API_EVENT_ID_SYSTEM_READY:
            Trace(1,"system initialize complete");
            break;

        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
            Trace(1,"network register success");
            Network_StartAttach();
            break;

        case API_EVENT_ID_NETWORK_ATTACHED:
            Trace(1,"network attach success");
            Network_PDP_Context_t context = {
                .apn        ="internet",
                .userName   = ""    ,
                .userPasswd = ""
            };
            Network_StartActive(context);
            break;

        case API_EVENT_ID_NETWORK_ACTIVATED:
            Trace(1,"network activate success.."); 
            OS_ReleaseSemaphore(semMqttStart);
            break;
        
        case API_EVENT_ID_SOCKET_CONNECTED:
            Trace(1,"socket connected");
            break;
        
        case API_EVENT_ID_SOCKET_CLOSED:
            Trace(1,"socket closed");
                      
            break;

        case API_EVENT_ID_SIGNAL_QUALITY:
            Trace(1,"CSQ:%d",pEvent->param1);
            break;
        case API_EVENT_ID_GPS_UART_RECEIVED:
            GPS_Update(pEvent->pParam1,pEvent->param1);
            break;
        case API_EVENT_ID_UART_RECEIVED:
            if(pEvent->param1 == UART1)
            {
                uint8_t data[pEvent->param2+1];
                data[pEvent->param2] = 0;
                memcpy(data,pEvent->pParam1,pEvent->param2);
                Trace(1,"uart received data,length:%d,data:%s",pEvent->param2,data);
                if(strcmp(data,"close") == 0)
                {
                    Trace(1,"close gps");
                    GPS_Close();
                }
                else if(strcmp(data,"open") == 0)
                {
                    Trace(1,"open gps");
                    GPS_Open(NULL);
                }
            }
            break;
        default:
            break;
    }
}

void gps_testTask(void *pData)
{
    GPS_Info_t* gpsInfo = Gps_GetInfo();
    uint8_t buffer[150];
    char buff1[15],buff2[15];
    while(!flag)
    {
        Trace(1,"wait for gprs regiter complete");
        OS_Sleep(2000);
    }
    GPS_Init();
    GPS_Open(NULL);
    while(gpsInfo->rmc.latitude.value == 0)
        OS_Sleep(1000);
    
    for(uint8_t i = 0;i<3;++i)
    {
        bool ret = GPS_SetOutputInterval(10000);
        Trace(1,"set gps ret:%d",ret);
        if(ret)
            break;
        OS_Sleep(1000);
    }
    
    if(!GPS_GetVersion(buffer,150))
        Trace(1,"get gps firmware version fail");
    else
        Trace(1,"gps firmware version:%s",buffer);

    if(!GPS_SetOutputInterval(1000))
        Trace(1,"set nmea output interval fail");
    
    Trace(1,"init ok");

    while(1)
    {
        uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ?gpsInfo->gsa[0].fix_type:gpsInfo->gsa[1].fix_type;
        char* isFixedStr;            
        if(isFixed == 2)
            isFixedStr = "2D fix";
        else if(isFixed == 3)
        {
            if(gpsInfo->gga.fix_quality == 1)
                isFixedStr = "3D fix";
            else if(gpsInfo->gga.fix_quality == 2)
                isFixedStr = "3D/DGPS fix";
        }
        else
            isFixedStr = "no fix";

        int temp = (int)(gpsInfo->rmc.latitude.value/gpsInfo->rmc.latitude.scale/100);
        double latitude = temp+(double)(gpsInfo->rmc.latitude.value - temp*gpsInfo->rmc.latitude.scale*100)/gpsInfo->rmc.latitude.scale/60.0;
        temp = (int)(gpsInfo->rmc.longitude.value/gpsInfo->rmc.longitude.scale/100);
        double longitude = temp+(double)(gpsInfo->rmc.longitude.value - temp*gpsInfo->rmc.longitude.scale*100)/gpsInfo->rmc.longitude.scale/60.0;

        gcvt(latitude,6,buff1);
        gcvt(longitude,6,buff2);
        

        snprintf(buffer,sizeof(buffer),"GPS fix mode:%d, BDS fix mode:%d, fix quality:%d, is fixed:%s, coordinate:WGS84, Latitude:%s, Longitude:%s, unit:degree",gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,
                                                            gpsInfo->gga.fix_quality,isFixedStr, buff1,buff2);
        Trace(2,buffer);
        UART_Write(UART1,buffer,strlen(buffer));
        UART_Write(UART1,"\r\n\r\n",4);
        OS_Sleep(5000);
    }
}


void gps_MainTask(void *pData)
{
    API_Event_t* event=NULL;

    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity   = UART_PARITY_NONE,
        .rxCallback = NULL,
        .useEvent   = true
    };
    UART_Init(UART1,config);

    OS_CreateTask(gps_testTask,
            NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);

    while(1)
    {
        if(OS_WaitEvent(gpsTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}


void gps_Main(void)
{
    gpsTaskHandle = OS_CreateTask(gps_MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&gpsTaskHandle);
}

