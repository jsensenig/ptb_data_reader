/** Example application to periodically read the XAdc monitoring sensors
  
  Author: Nuno Barros <nfbarros@hep.upenn.edu>

**/

#include <stdbool.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>

#define MAX_PATH_SIZE 200
//#define MAX_NAME_SIZE 50
#define MAX_VALUE_SIZE  100
#define MAX_UNIT_NAME_SIZE 50

#define SYS_PATH_IIO   "/sys/bus/iio/devices"
#define DEVICE_NAME    "xadc"

// -- Return codes
enum EErrorCode
{
  RET_SUCCESS,
  RET_ERR_DEV_NOT_FOUND,
  RET_CANNOT_OPEN_FILE,
  RET_FILE_READ_FAILED,
  RET_FILE_WRITE_FAILED
};

// -- Conversion modes
enum EConvType
{
  EConvType_None,
  EConvType_Raw_to_Scale,
  EConvType_Scale_to_Raw,
  EConvType_Max
};

struct XadcParameter
{
  const char name[MAX_VALUE_SIZE];
  const char short_name[MAX_VALUE_SIZE];
  float value;
  float (* const conv_fn)(float,enum EConvType);
  const char unit[MAX_UNIT_NAME_SIZE];
  // mutex *lock;
};

// -- Sensor parameters
enum XADC_Param
{
  EParamVccInt,
    EParamVccPAux,
  EParamVccBRam,
  EParamVccAux,
  EParamVccPInt,
  EParamVccODdr,
  EParamVRefP,
  EParamVRefN,
  EParamTemp,
  EParamMax // -- Allows to get the total number of sensors from a ENUM
};

enum XADC_Init_Type
{
  EXADC_INIT_READ_ONLY,
  EXADC_INIT_FULL, // -- NOT USED
  EXADC_NOT_INITIALIZED
};



///
/// Forward declarations
///


// -- Forward declarations
int xadc_core_init(enum XADC_Init_Type type);
int xadc_core_deinit(void);
float xadc_get_value(enum XADC_Param parameter);
float xadc_touch(enum XADC_Param parameter);

// -- Function to convert temperature into ADC counts, and vice-versa
float conv_temperature(float nput, enum EConvType conv_direction);
float conv_voltage(float nput, enum EConvType conv_direction);
float conv_voltage_ext_ch(float input, enum EConvType conv_direction);

// Aux functions
static int get_iio_node(const char * deviceName);
static int line_from_file(char* filename, char*linebuf);
void sig_handler(int signo);


///
/// Global variables
///


// -- Global variables for state machines
static char gDevicePath[MAX_PATH_SIZE];
static char gNodeName[MAX_VALUE_SIZE];
static enum XADC_Init_Type gInit_state = EXADC_NOT_INITIALIZED;
/// -- Define the multiplier
static const int multiplier = 1<<12; // -- size limit of the ADC, in samples
static const int mV_mul = 1000;
static bool keep_running; // control the running time

struct XadcParameter gXadcData[EParamMax] = {
  [EParamVccInt]  = { "in_voltage0_vccint_raw",   "VccInt", 0, conv_voltage,"mV"},
  [EParamVccPAux] = { "in_voltage4_vccpaux_raw",  "VccAuxP",  0, conv_voltage,"mV"},      // todo: workaround
  [EParamVccBRam] = { "in_voltage2_vccbram_raw",  "VccBRAM",  0, conv_voltage,"mV"},
  [EParamVccAux]  = { "in_voltage1_vccaux_raw",   "VccAux", 0, conv_voltage,"mV"},
  [EParamVccPInt] = { "in_voltage3_vccpint_raw",  "VccIntP",  0, conv_voltage,"mV"},      // todo: workaround
  [EParamVccODdr] = { "in_voltage5_vccoddr_raw",  "VccODDR",  0, conv_voltage,"mV"},      // todo: workaround
  [EParamVRefP] = { "in_voltage6_vrefp_raw",  "VRefP",  0, conv_voltage,"mV"},      // todo: workaround
  [EParamVRefN] = { "in_voltage7_vrefn_raw",  "VRefN",  0, conv_voltage,"mV"},      // todo: workaround
  //  [EParamVAux0]  = { "in_voltage8_raw",     0, conv_voltage_ext_ch},
  [EParamTemp]   = { "in_temp0_raw",        "Temp",   0, conv_temperature,"C"}
};

static int get_iio_node(const char * deviceName)
{
  struct dirent **namelist;
  char file[MAX_PATH_SIZE];
  char name[MAX_VALUE_SIZE];
  int i,n;
  int flag = 0;

  printf("-- Finding device (%s)\n",deviceName);

  n = scandir(SYS_PATH_IIO, &namelist, 0, alphasort);
  if (n < 0)
    return RET_ERR_DEV_NOT_FOUND;

  for (i=0; i < n; i++)
  {
    sprintf(file, "%s/%s/name", SYS_PATH_IIO, namelist[i]->d_name);
    printf("Probing...[%s]\n",file);
    if ((line_from_file(file,name) == 0) && (strcmp(name,deviceName) ==  0))
    {
      flag =1;
      strcpy(gNodeName, namelist[i]->d_name);
      sprintf(gDevicePath, "%s/%s", SYS_PATH_IIO, gNodeName);
      break;
    }
  }

  if(flag == 0) return RET_ERR_DEV_NOT_FOUND;

  printf("Device found : (%s)[%s]\n",gNodeName,gDevicePath);
  return RET_SUCCESS;
}


int xadc_core_init(enum XADC_Init_Type init_type)
{
  int ret = 0;

  assert(gInit_state == EXADC_NOT_INITIALIZED); // Make sure it is only called once.
  assert(init_type >= 0 && init_type < EXADC_NOT_INITIALIZED);

  if (get_iio_node(DEVICE_NAME) != RET_SUCCESS)
  {
    perror(DEVICE_NAME " Device Not Found");
    return -1;
  }

  printf("\n--> Device found : %s \n\n",gDevicePath);
  gInit_state = init_type;

  return 0;
}

int xadc_core_deinit(void)
{
  assert(gInit_state != EXADC_NOT_INITIALIZED);

  gInit_state = EXADC_NOT_INITIALIZED;
  return 0;
}

static int line_from_file(char* filename, char* linebuf)
{
    char* s;
    int i;
    FILE* fp = fopen(filename, "r");
    if (!fp){
      printf("    Failed to open file %s\n",filename);
      return RET_CANNOT_OPEN_FILE;
    } 
    s = fgets(linebuf, MAX_VALUE_SIZE, fp);
    fclose(fp);
    if (!s) return RET_FILE_READ_FAILED;
    //printf("Read [%s]\n",s);
    for (i=0; (*s)&&(i<MAX_VALUE_SIZE); i++) {
        if (*s == '\n') *s = 0;
        s++;
    }
    return RET_SUCCESS;
}


//utility functions
float conv_voltage(float input, enum EConvType conv_direction)
{
  float result=0;

  switch(conv_direction)
  {
  case EConvType_Raw_to_Scale:
    result = ((input * 3.0 * mV_mul)/multiplier);
    break;
  case EConvType_Scale_to_Raw:
    result = (input/(3.0 * mV_mul))*multiplier;
    break;
  default:
    printf("Convertion type incorrect... Doing no conversion\n");
    //  intentional no break;
  case EConvType_None:
      result = input;
      break;
  }

  return result;
}

float conv_voltage_ext_ch(float input, enum EConvType conv_direction)
{
  float result=0;

  switch(conv_direction)
  {
  case EConvType_Raw_to_Scale:
    result = ((input * mV_mul)/multiplier);
    break;
  case EConvType_Scale_to_Raw:
    result = (input/mV_mul)*multiplier;
    break;
  default:
    printf("Convertion type incorrect... Doing no conversion\n");
    //  intentional no break;
  case EConvType_None:
      result = input;
      break;
  }

  return result;
}

float conv_temperature(float input, enum EConvType conv_direction)
{
  float result=0;

  switch(conv_direction)
  {
  case EConvType_Raw_to_Scale:
    result = ((input * 503.975)/multiplier) - 273.15;
    break;
  case EConvType_Scale_to_Raw:
    result = (input + 273.15)*multiplier/503.975;
    break;
  default:
    printf("Conversion type incorrect... Doing no conversion\n");
    //  intentional no break;
  case EConvType_None:
      result = input;
      break;
  }

  return result;
}



static int read_xadc_param(struct XadcParameter *param)
{
  char filename[MAX_PATH_SIZE];
  char read_value[MAX_VALUE_SIZE];

  memset(filename, 0, sizeof(filename) );

  sprintf(filename, "%s/%s", gDevicePath, param->name );

  if (line_from_file(filename,read_value) == RET_SUCCESS)
  {
    // -- Convert from the raw ADC counts to the appropriate values 
    param->value = param->conv_fn(atof(read_value), EConvType_Raw_to_Scale);
  }
  else
  {
    printf("\n***Error: reading file %s\n",filename);
    param->value = 0;
    return RET_FILE_READ_FAILED;
  }

  return RET_SUCCESS;
}


float xadc_touch(enum XADC_Param parameter)
{
  assert(gInit_state != EXADC_NOT_INITIALIZED);

  assert((parameter >= 0) && (parameter < EParamMax));
  if(read_xadc_param(gXadcData + parameter) != RET_SUCCESS)
  {
    perror("Error Updating the statistic \n");
    return 0;
  }
  return gXadcData[parameter].value;
}




void sig_handler(int signo)
{
  if (signo == SIGINT) {
    printf("\n\nreceived SIGINT\n\n");
  keep_running = false;
  }
}



int main(int argc, char *argv[])
{
  int i;

  if(xadc_core_init(EXADC_INIT_READ_ONLY) != 0)
  {
    perror("Couldn't Start the XADC Core\nExiting\n");
    xadc_core_deinit();
    return 0;
  }
  keep_running = true;
  if (signal(SIGINT, sig_handler) == SIG_ERR) {
      printf("\nCan't catch SIGINT\n");
      return 1;
  }
  printf("\n\n Press Ctrl+C to interrupt the polling.\n\n");
  // -- Print the header
  for (i=0; i < EParamMax; i++) {
    printf("%-10s  ",gXadcData[i].short_name);
  }
  printf("\n");
  while (keep_running) {

    for (i=0; i < EParamMax; i++)
    {
      //printf("%s %.2f %s\n",gXadcData[i].short_name,xadc_touch((enum XADC_Param)i), gXadcData[i].unit);
      printf("%*.2f %-2s  ",-7,xadc_touch((enum XADC_Param)i), gXadcData[i].unit);
    }
    printf("\n");
    sleep(2);
}

  xadc_core_deinit();
  return 0;
}

