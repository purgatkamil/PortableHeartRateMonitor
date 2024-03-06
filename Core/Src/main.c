/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
#include <math.h>
#include <string.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define GRAPH_BUFFOR_SIZE 160
#define Y_AXIS_OFFSET 1
#define X_AXIS_OFFSET 64

#define BACKGROUND_COLOR 	BLACK
#define AXIS_COLOR 			WHITE
#define CHART_COLOR			GREEN

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

FATFS fs;
FIL fil;



int free_clusters;

RTC_TimeTypeDef time;
RTC_DateTypeDef date;

int seconds = 0, minutes = 0, hours = 0;

long int SD_counter = 0;

typedef struct {
    int32_t data[GRAPH_BUFFOR_SIZE];
    int head;
} GraphBuffer;

GraphBuffer graphBuffer;
GraphBuffer ZoomedOutGraphBuffer;

uint32_t value = 0;
uint32_t rescaled_value = 0;

float filteredValue = 0;  // Holds the filtered value
const float alpha = 0.7;  // Filter coefficient, adjust as needed

float voltage;

#define QRS_BUFFOR_SIZE 500

typedef struct {
	int data[QRS_BUFFOR_SIZE];
	int head;
	int previous, current, next;
} QRSBuffer;

QRSBuffer derivativeBuffer;

int derivative = 0;
int median_filtered_value = 0;

#define MEDIAN_BUFFOR_SIZE 5	//musi być liczba nieparzysta

typedef struct {
    int data[MEDIAN_BUFFOR_SIZE];
    int head;
} MedianBuffer;

MedianBuffer mBuffer;

#define LOWPASS_FILTER_SIZE 491 //41
#define LOWPASS_FILTER2_SIZE 491

typedef struct {
	int data[LOWPASS_FILTER_SIZE];
	int head;
} LowPassFilter;

LowPassFilter LowPass;

int32_t LPvalue;

int filterTimeCounter = 0;
int filterAvarageTicks = 0;
int TimeCounter = 0;

double filter_coefficients2[41] = {

		-0.00000000000000000, -0.00084388399507143, 0.00172480322214108, -0.00231835925473623, 0.00196357837057156,
		-0.00000000000000001, -0.00360465196592370, 0.00771487906070590, -0.01004082432204096, 0.00796400731979479, -0.00000000000000002,
		-0.01272687129871608, 0.02582363511903904, -0.03239793500394676, 0.02528013244720928, -0.00000000000000003, -0.04268216706926069,
		 0.09588895516209603, -0.14801625740913227, 0.18611212629983057, 0.80031766663487980, 0.18611212629983057, -0.14801625740913227,
		 0.09588895516209603, -0.04268216706926069, -0.00000000000000003, 0.02528013244720928, -0.03239793500394676, 0.02582363511903904,
		 -0.01272687129871608, -0.00000000000000002, 0.00796400731979479, -0.01004082432204096, 0.00771487906070590, -0.00360465196592370,
		 -0.00000000000000001, 0.00196357837057156, -0.00231835925473623, 0.00172480322214108, -0.00084388399507143, -0.00000000000000000};

double filter_coefficients[491] = {

		-0.006724665374820101,
		  -0.0039592405521505425,
		  -0.004879842672090847,
		  -0.005739436212265738,
		  -0.006473414299707283,
		  -0.0070174509406664065,
		  -0.007313012324351356,
		  -0.007311454168248525,
		  -0.006979710764297932,
		  -0.006302835093303763,
		  -0.005287356556378177,
		  -0.003961723353658333,
		  -0.0023757965558372303,
		  -0.0005988687959034679,
		  0.0012846423368572202,
		  0.0031804194041918123,
		  0.004989989147426519,
		  0.006617906605274425,
		  0.007978144220548312,
		  0.009000370456116963,
		  0.00963561712424311,
		  0.009859823620655492,
		  0.009675922372876462,
		  0.009114240543608798,
		  0.008229889822016166,
		  0.007098660335239779,
		  0.0058114576331865534,
		  0.004466958691646338,
		  0.0031640614866178785,
		  0.001994754958797834,
		  0.0010372535697518407,
		  0.00035018832064850084,
		  -0.0000316878151375713,
		  -0.00009979985099640354,
		  0.0001279462620581436,
		  0.0006089181959532771,
		  0.0012803236653627956,
		  0.0020655007292555163,
		  0.0028808092925518745,
		  0.003643281437626687,
		  0.004277629983100943,
		  0.004721770266061771,
		  0.004932084620797877,
		  0.004887422588363258,
		  0.004591207302807786,
		  0.004071348980577778,
		  0.0033765519649118776,
		  0.0025699599995857585,
		  0.0017229187244479393,
		  0.0009088726991713141,
		  0.000198365239799021,
		  -0.00034681086809946164,
		  -0.0006818755743303907,
		  -0.0007858211828020191,
		  -0.0006611599558581819,
		  -0.00032840133644604515,
		  0.00017727870154062673,
		  0.0007987440562488426,
		  0.0014607127507710379,
		  0.002096780427785378,
		  0.002653637338425714,
		  0.0030485458836070206,
		  0.003262979591314785,
		  0.003259839052466486,
		  0.0030431766166477132,
		  0.0026304290591559827,
		  0.0020607199420756323,
		  0.0013874693776336233,
		  0.0006749311163228131,
		  -0.00000963016661013781,
		  -0.0006010655262245917,
		  -0.0010439535004921193,
		  -0.001297748856870107,
		  -0.0013405110851259089,
		  -0.0011714991943510562,
		  -0.000810879483457281,
		  -0.00029742430192708215,
		  0.0003146649172919567,
		  0.0009616688975564811,
		  0.0015764142387132863,
		  0.0020945573064679863,
		  0.0024615771594911993,
		  0.0026382054341945907,
		  0.0026040197751395593,
		  0.002360063382717492,
		  0.0019285413288921607,
		  0.0013502288762677727,
		  0.0006807841869889326,
		  -0.000014480375247198256,
		  -0.0006671131049351773,
		  -0.001212448893684446,
		  -0.001596269583429432,
		  -0.0017803135592194073,
		  -0.0017463361417471976,
		  -0.0014977962895742914,
		  -0.001059186745037265,
		  -0.0004742618964127832,
		  0.00019770396757928525,
		  0.000888040002417127,
		  0.001525689456974742,
		  0.002044653947339687,
		  0.0023908249025104606,
		  0.0025267086770198142,
		  0.002435174721928312,
		  0.002121905862725469,
		  0.0016154484421309916,
		  0.0009647222256839877,
		  0.000233563635124051,
		  -0.0005063531853019121,
		  -0.0011813298880232114,
		  -0.0017222632441453295,
		  -0.002072835626498741,
		  -0.0021966661307383064,
		  -0.002080080229624072,
		  -0.0017316884329294407,
		  -0.001183300533026521,
		  -0.0004903977566402818,
		  0.0002776975291079214,
		  0.0010459705289211738,
		  0.0017342987007244492,
		  0.002268932313180422,
		  0.002598334594491151,
		  0.002679463736630121,
		  0.0025046953842694387,
		  0.002084506654881727,
		  0.0014587960530604393,
		  0.0006866415726633514,
		  -0.00015578410529333046,
		  -0.0009850387611147508,
		  -0.001716475964921645,
		  -0.0022751481102348077,
		  -0.0026016183324591483,
		  -0.0026592322103422945,
		  -0.0024380894815064376,
		  -0.001955852355017696,
		  -0.0012571698284203788,
		  -0.00040970036530490995,
		  0.0005030366132415669,
		  0.0013889951855443744,
		  0.0021573580428614707,
		  0.0027280556827255858,
		  0.003039136013865625,
		  0.003053521103786466,
		  0.002763810546538518,
		  0.002193151835628833,
		  0.0013936931549081402,
		  0.0004420555157804875,
		  -0.0005682957821342262,
		  -0.0015363221411339113,
		  -0.0023631979727147816,
		  -0.002962018447783027,
		  -0.0032670132481304294,
		  -0.0032411431703225813,
		  -0.0028801596282385347,
		  -0.0022134906585668315,
		  -0.0013022175330133895,
		  -0.00023344257158772061,
		  0.0008877621092884505,
		  0.001948251056949869,
		  0.0028385216433642895,
		  0.0034640646473657285,
		  0.003754952085148802,
		  0.0036733777478027275,
		  0.0032180385336624713,
		  0.0024256319928504327,
		  0.0013685039871313138,
		  0.00014742315498719977,
		  -0.0011182060861168336,
		  -0.0023010017621534723,
		  -0.003278237649604041,
		  -0.003944975486449301,
		  -0.004225447724876906,
		  -0.004080471110798365,
		  -0.003512223671868546,
		  -0.002566804645981423,
		  -0.0013303407668937672,
		  0.00008083661809819983,
		  0.0015283011428720487,
		  0.0028647573653323715,
		  0.003951821725081272,
		  0.0046715158281848065,
		  0.004937056494208208,
		  0.004709339867739604,
		  0.0039929468318752985,
		  0.002846563055546284,
		  0.0013713053632263285,
		  -0.0002930863055740355,
		  -0.001984494759627223,
		  -0.0035309946691470698,
		  -0.0047707894977827734,
		  -0.005565436118432528,
		  -0.005818010832860896,
		  -0.005482124573820861,
		  -0.004569201490560631,
		  -0.003149980554713361,
		  -0.0013485107230243431,
		  0.0006675135833448967,
		  0.0027010726086147867,
		  0.004545813474137445,
		  0.006005545901987634,
		  0.006914038674769464,
		  0.007153828906109374,
		  0.006669593976789687,
		  0.005475923408668897,
		  0.003659906521809703,
		  0.0013758828686440234,
		  -0.0011670880119533842,
		  -0.003723465881098876,
		  -0.006033477794114198,
		  -0.007847911134326176,
		  -0.00895361528950788,
		  -0.00919654696182677,
		  -0.008499976682260126,
		  -0.00687683209351048,
		  -0.004433591159930186,
		  -0.0013650181362259298,
		  0.0020590529469560986,
		  0.005514953655964573,
		  0.008653130201617525,
		  0.011130081179609811,
		  0.012641896880767457,
		  0.012955860115718755,
		  0.011937431472223406,
		  0.00957094808924724,
		  0.005970361321244434,
		  0.001378649724826345,
		  -0.003843202803927939,
		  -0.009237945620379866,
		  -0.014284003731359221,
		  -0.018434636670428897,
		  -0.021161074404101234,
		  -0.021996111287966347,
		  -0.020576235774781685,
		  -0.0166771128822221,
		  -0.010238084024046364,
		  -0.0013766816948736296,
		  0.00960890053902094,
		  0.02225469804659545,
		  0.03595673277608928,
		  0.05000709575744708,
		  0.06364204656716094,
		  0.07609342946430755,
		  0.08663919655603505,
		  0.09465877281488098,
		  0.09967180301038488,
		  0.10137741882288678,
		  0.09967180301038488,
		  0.09465877281488098,
		  0.08663919655603505,
		  0.07609342946430755,
		  0.06364204656716094,
		  0.05000709575744708,
		  0.03595673277608928,
		  0.02225469804659545,
		  0.00960890053902094,
		  -0.0013766816948736296,
		  -0.010238084024046364,
		  -0.0166771128822221,
		  -0.020576235774781685,
		  -0.021996111287966347,
		  -0.021161074404101234,
		  -0.018434636670428897,
		  -0.014284003731359221,
		  -0.009237945620379866,
		  -0.003843202803927939,
		  0.001378649724826345,
		  0.005970361321244434,
		  0.00957094808924724,
		  0.011937431472223406,
		  0.012955860115718755,
		  0.012641896880767457,
		  0.011130081179609811,
		  0.008653130201617525,
		  0.005514953655964573,
		  0.0020590529469560986,
		  -0.0013650181362259298,
		  -0.004433591159930186,
		  -0.00687683209351048,
		  -0.008499976682260126,
		  -0.00919654696182677,
		  -0.00895361528950788,
		  -0.007847911134326176,
		  -0.006033477794114198,
		  -0.003723465881098876,
		  -0.0011670880119533842,
		  0.0013758828686440234,
		  0.003659906521809703,
		  0.005475923408668897,
		  0.006669593976789687,
		  0.007153828906109374,
		  0.006914038674769464,
		  0.006005545901987634,
		  0.004545813474137445,
		  0.0027010726086147867,
		  0.0006675135833448967,
		  -0.0013485107230243431,
		  -0.003149980554713361,
		  -0.004569201490560631,
		  -0.005482124573820861,
		  -0.005818010832860896,
		  -0.005565436118432528,
		  -0.0047707894977827734,
		  -0.0035309946691470698,
		  -0.001984494759627223,
		  -0.0002930863055740355,
		  0.0013713053632263285,
		  0.002846563055546284,
		  0.0039929468318752985,
		  0.004709339867739604,
		  0.004937056494208208,
		  0.0046715158281848065,
		  0.003951821725081272,
		  0.0028647573653323715,
		  0.0015283011428720487,
		  0.00008083661809819983,
		  -0.0013303407668937672,
		  -0.002566804645981423,
		  -0.003512223671868546,
		  -0.004080471110798365,
		  -0.004225447724876906,
		  -0.003944975486449301,
		  -0.003278237649604041,
		  -0.0023010017621534723,
		  -0.0011182060861168336,
		  0.00014742315498719977,
		  0.0013685039871313138,
		  0.0024256319928504327,
		  0.0032180385336624713,
		  0.0036733777478027275,
		  0.003754952085148802,
		  0.0034640646473657285,
		  0.0028385216433642895,
		  0.001948251056949869,
		  0.0008877621092884505,
		  -0.00023344257158772061,
		  -0.0013022175330133895,
		  -0.0022134906585668315,
		  -0.0028801596282385347,
		  -0.0032411431703225813,
		  -0.0032670132481304294,
		  -0.002962018447783027,
		  -0.0023631979727147816,
		  -0.0015363221411339113,
		  -0.0005682957821342262,
		  0.0004420555157804875,
		  0.0013936931549081402,
		  0.002193151835628833,
		  0.002763810546538518,
		  0.003053521103786466,
		  0.003039136013865625,
		  0.0027280556827255858,
		  0.0021573580428614707,
		  0.0013889951855443744,
		  0.0005030366132415669,
		  -0.00040970036530490995,
		  -0.0012571698284203788,
		  -0.001955852355017696,
		  -0.0024380894815064376,
		  -0.0026592322103422945,
		  -0.0026016183324591483,
		  -0.0022751481102348077,
		  -0.001716475964921645,
		  -0.0009850387611147508,
		  -0.00015578410529333046,
		  0.0006866415726633514,
		  0.0014587960530604393,
		  0.002084506654881727,
		  0.0025046953842694387,
		  0.002679463736630121,
		  0.002598334594491151,
		  0.002268932313180422,
		  0.0017342987007244492,
		  0.0010459705289211738,
		  0.0002776975291079214,
		  -0.0004903977566402818,
		  -0.001183300533026521,
		  -0.0017316884329294407,
		  -0.002080080229624072,
		  -0.0021966661307383064,
		  -0.002072835626498741,
		  -0.0017222632441453295,
		  -0.0011813298880232114,
		  -0.0005063531853019121,
		  0.000233563635124051,
		  0.0009647222256839877,
		  0.0016154484421309916,
		  0.002121905862725469,
		  0.002435174721928312,
		  0.0025267086770198142,
		  0.0023908249025104606,
		  0.002044653947339687,
		  0.001525689456974742,
		  0.000888040002417127,
		  0.00019770396757928525,
		  -0.0004742618964127832,
		  -0.001059186745037265,
		  -0.0014977962895742914,
		  -0.0017463361417471976,
		  -0.0017803135592194073,
		  -0.001596269583429432,
		  -0.001212448893684446,
		  -0.0006671131049351773,
		  -0.000014480375247198256,
		  0.0006807841869889326,
		  0.0013502288762677727,
		  0.0019285413288921607,
		  0.002360063382717492,
		  0.0026040197751395593,
		  0.0026382054341945907,
		  0.0024615771594911993,
		  0.0020945573064679863,
		  0.0015764142387132863,
		  0.0009616688975564811,
		  0.0003146649172919567,
		  -0.00029742430192708215,
		  -0.000810879483457281,
		  -0.0011714991943510562,
		  -0.0013405110851259089,
		  -0.001297748856870107,
		  -0.0010439535004921193,
		  -0.0006010655262245917,
		  -0.00000963016661013781,
		  0.0006749311163228131,
		  0.0013874693776336233,
		  0.0020607199420756323,
		  0.0026304290591559827,
		  0.0030431766166477132,
		  0.003259839052466486,
		  0.003262979591314785,
		  0.0030485458836070206,
		  0.002653637338425714,
		  0.002096780427785378,
		  0.0014607127507710379,
		  0.0007987440562488426,
		  0.00017727870154062673,
		  -0.00032840133644604515,
		  -0.0006611599558581819,
		  -0.0007858211828020191,
		  -0.0006818755743303907,
		  -0.00034681086809946164,
		  0.000198365239799021,
		  0.0009088726991713141,
		  0.0017229187244479393,
		  0.0025699599995857585,
		  0.0033765519649118776,
		  0.004071348980577778,
		  0.004591207302807786,
		  0.004887422588363258,
		  0.004932084620797877,
		  0.004721770266061771,
		  0.004277629983100943,
		  0.003643281437626687,
		  0.0028808092925518745,
		  0.0020655007292555163,
		  0.0012803236653627956,
		  0.0006089181959532771,
		  0.0001279462620581436,
		  -0.00009979985099640354,
		  -0.0000316878151375713,
		  0.00035018832064850084,
		  0.0010372535697518407,
		  0.001994754958797834,
		  0.0031640614866178785,
		  0.004466958691646338,
		  0.0058114576331865534,
		  0.007098660335239779,
		  0.008229889822016166,
		  0.009114240543608798,
		  0.009675922372876462,
		  0.009859823620655492,
		  0.00963561712424311,
		  0.009000370456116963,
		  0.007978144220548312,
		  0.006617906605274425,
		  0.004989989147426519,
		  0.0031804194041918123,
		  0.0012846423368572202,
		  -0.0005988687959034679,
		  -0.0023757965558372303,
		  -0.003961723353658333,
		  -0.005287356556378177,
		  -0.006302835093303763,
		  -0.006979710764297932,
		  -0.007311454168248525,
		  -0.007313012324351356,
		  -0.0070174509406664065,
		  -0.006473414299707283,
		  -0.005739436212265738,
		  -0.004879842672090847,
		  -0.0039592405521505425,
		  -0.006724665374820101
};

enum display_mode{ZOOM_IN, ZOOM_OUT};
enum display_mode display_mode = ZOOM_OUT;


int already_changed = 0;


//####################################### QRS detection #######################################
#define F_SAMPLE 500
#define TRESHOLD_MULT 0.7
#define WINDOW_SEARCH 40
#define SKIP_VALUE 80
#define QRS_SETUP 1000

#define QRS_INIT_ITERATIONS 12		//ilosc wykrytych zespolow qrs, po ktorych nastapi obliczenie tetna

#define WAITING 1000

typedef struct {
int M[QRS_INIT_ITERATIONS];
long M_value[QRS_INIT_ITERATIONS];
int S1[QRS_INIT_ITERATIONS];
long treshold;
int intervals[QRS_INIT_ITERATIONS];
int current_interval_len;
int current_interval;
} QRS_process_properties;

QRS_process_properties QRS_prop;

enum QRS_State {INIT, TRESHOLD_SEARCH, MAXIMUM_SEARCH, RELAX, WAIT_ON_START};
enum QRS_State Stan_algorytmu = WAIT_ON_START;

int waiting_time = 0;

int init_phase = 0;
int init_samples = 0;
int max_value = 0;
int m_value_head = 0;	//index do tablicy M_value
int s1_head = 0;

int RR = 0;

int sample_num = 0;

int intervals_head = 0;

int pulse = 0;

int sample_skip = 0;

int max_avarage = 0;
int intervals_avarage = 0;


int max_pulse = 0;
int min_pulse = -1;
char min_pulse_tab[22];
char max_pulse_tab[22];

//#################################################################################################

int xPos = 1;
int data_new = 0;
int data_old = 0;

char timer[8];
char pulse_tab[5];


int min_adc_value = 200;
int max_adc_value = 0;

//######################### filtr IIR  ####################################################

int TestValue = 0;
int TestValue2 = 0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void DrawAxis(uint16_t xAxis, uint16_t yAxis, uint16_t color) {

	for(int i = 0; i < LCD_HEIGHT; i++){
		lcd_put_pixel(yAxis, i, color);
	}
	for(int i = Y_AXIS_OFFSET; i <= LCD_WIDTH - Y_AXIS_OFFSET; i++){
		lcd_put_pixel(i, xAxis, color);
	}

}

void GraphBuffer_Init(GraphBuffer *buffer) {
    buffer->head = 0;
    int i;
    for (i = 0; i < GRAPH_BUFFOR_SIZE; i++) {
        buffer->data[i] = 64;
    }
}

void MedianBuffer_Init(MedianBuffer *buffer){
	buffer->head = 0;
	int i;
	for (i=0; i < MEDIAN_BUFFOR_SIZE; i++){
		buffer->data[i] = 0;
	}
}

void LowPassBuffer_Init(LowPassFilter *buffer){
	buffer->head = 0;
	int i;
	for (i=0; i < LOWPASS_FILTER_SIZE; i++){
		buffer->data[i] = 0;
	}
}


void GraphBuffer_Add(GraphBuffer *buffer, int value) {
    buffer->data[buffer->head] = value;
    buffer->head = (buffer->head + 1) % GRAPH_BUFFOR_SIZE;
}

void QRSBuffer_Add(QRSBuffer *buffer, int value) {
    buffer->data[buffer->head] = value;
    buffer->head = (buffer->head + 1) % QRS_BUFFOR_SIZE;
}

void MedianBuffer_Add(MedianBuffer *buffer, int value) {
    buffer->data[buffer->head] = value;
    buffer->head = (buffer->head + 1) % MEDIAN_BUFFOR_SIZE;
}

void LowPassBuffer_Add(LowPassFilter *buffer, int value) {
    buffer->data[buffer->head] = value;
    buffer->head = (buffer->head + 1) % LOWPASS_FILTER_SIZE;
}

void bubbleSort(int32_t *arr, int32_t size) {
    int i, j;
    for (i = 0; i < size-1; i++)
        for (j = 0; j < size-i-1; j++)
            if (arr[j] > arr[j+1]) {
                // Zamiana miejscami arr[j] i arr[j+1]
                int temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
}

int32_t get_median(int32_t *arr, int32_t size){ return arr[ (size/2) + 1 ]; }

int32_t filter_median(int32_t new_data){
	int32_t tmp_array[MEDIAN_BUFFOR_SIZE];
	int32_t out = 0;

	MedianBuffer_Add(&mBuffer, new_data);

	memcpy(tmp_array, mBuffer.data, sizeof(tmp_array));
	bubbleSort(tmp_array, MEDIAN_BUFFOR_SIZE);

	out = get_median(tmp_array, MEDIAN_BUFFOR_SIZE);

	return out;
}

int32_t LPFilter(int32_t new_data){
	LowPassBuffer_Add(&LowPass, new_data);

	double output = 0;
	int index = LowPass.head;
	for (int i = 0; i < LOWPASS_FILTER_SIZE; ++i) {
	    index = index != 0 ? index - 1 : LOWPASS_FILTER_SIZE - 1;
	    output += filter_coefficients[i] * LowPass.data[index];
	}

	    return output;

}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6) {

	  	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	  	  HAL_ADC_Start(&hadc1);

	  	  value = HAL_ADC_GetValue(&hadc1);	// 25 cycles





	  	  LPvalue = LPFilter(value);


	  	  filteredValue = (1 - alpha) * LPvalue + alpha * filteredValue;


	  	 // DWT->CYCCNT = 0;
	  	  median_filtered_value = filter_median(filteredValue);


	  	  derivativeBuffer.previous =  derivativeBuffer.current;
	  	  derivativeBuffer.current =  derivativeBuffer.next;
	  	  derivativeBuffer.next = median_filtered_value;


	  	  derivative = abs(derivativeBuffer.next - derivativeBuffer.previous);
	  	  if(derivative < 30)
	  		  derivative = 0;


	  	  TestValue = map(median_filtered_value, 0, 4096, 0, 128);
	  	  TestValue2 = map(value, 0, 4096, 0, 128);
	  	  printf("%ld,%ld\n", TestValue, TestValue2);


	  	  QRSBuffer_Add(&derivativeBuffer, derivative);

	  	  rescaled_value = map(median_filtered_value, 0, 4096, 0, 128);


	  	  //###################################### PROCES WYKRYWANIA QRS ######################################

	  	  switch(Stan_algorytmu){

	  	  case WAIT_ON_START:
	  		  if(waiting_time < WAITING){
	  			  waiting_time++;
	  		  }
	  		  else{
	  			  waiting_time = 0;
	  			  Stan_algorytmu = INIT;
	  		  }
	  		  break;

	  	  case INIT:

	  		  if(init_samples < QRS_SETUP){
		  		  if((max_value < derivativeBuffer.data[derivativeBuffer.head - 1]) && (derivativeBuffer.head != 0)){
		  			max_value = derivativeBuffer.data[derivativeBuffer.head - 1];
		  		  }


		  		  init_samples++;
	  		  }
	  		  else{
	  			  QRS_prop.treshold = max_value * TRESHOLD_MULT;
	  			  //resetujemy zmienna max_value i init_samples, aby były gotowe do ponownego procesu inicjalizacji w przypadku bledu
	  			  max_value = 0;
	  			  init_samples = 0;
	  			  init_phase = 1;
	  			  Stan_algorytmu = TRESHOLD_SEARCH;
	  		  }

	  		  break;

	  	  case TRESHOLD_SEARCH:

	  		  if((derivativeBuffer.data[derivativeBuffer.head - 1] >= QRS_prop.treshold) && (derivativeBuffer.head != 0)){
	  			QRS_prop.S1[s1_head] = derivativeBuffer.data[derivativeBuffer.head - 1];


	  			if(s1_head < QRS_INIT_ITERATIONS - 1){
	  			  s1_head++;
	  			}
	  			else{
	  			  s1_head = 0;
	  			}

	  			Stan_algorytmu = MAXIMUM_SEARCH;
	  		  }
	  		  else{

	  		    RR++;

	  		  }

	  		  break;

	  	  case MAXIMUM_SEARCH:

	  		  if(sample_num < WINDOW_SEARCH){
	  			  if((derivativeBuffer.data[derivativeBuffer.head - 1] >= QRS_prop.S1[s1_head]) && (derivativeBuffer.head != 0)){
	  				  if((RR < 700) || (RR >= 0)){		//zabezpieczenie przed przekraczaniem zbyt dużych wartosci
	  	  				QRS_prop.M[m_value_head] = RR;
	  	  				QRS_prop.M_value[m_value_head] = derivativeBuffer.data[derivativeBuffer.head - 1];
	  				  }
	  			  }

	  			  RR++;
	  			  sample_num++;
	  		  }
	  		  else{
	  			  QRS_prop.intervals[intervals_head] = QRS_prop.M[m_value_head];		//wpisanie znalezionej wartosci RR do tablicy intervals
	  			  if(intervals_head < QRS_INIT_ITERATIONS - 1){
	  				intervals_head++;
	  			  }
	  			  else{
	  				  intervals_head = 0;
	  			  }

	  			  sample_num = 0;
	  			  if(m_value_head < QRS_INIT_ITERATIONS - 1){
	  				m_value_head++;
	  			  }
	  			  else{
	  				  int i;
	  				  max_avarage = 0;

	  				  for(i = 0; i < QRS_INIT_ITERATIONS; i++){
	  					  max_avarage += QRS_prop.M_value[i];
	  				  }

	  				  max_avarage = max_avarage / QRS_INIT_ITERATIONS;
	  				  QRS_prop.treshold = max_avarage * TRESHOLD_MULT;


	  				  //zabezpieczenie przed ujemnymi wartosciami RR
	  				  if(RR - QRS_prop.M[m_value_head] < 0){
	  					  RR = 0;
	  				  }
	  				  else{
	  					RR = RR - QRS_prop.M[m_value_head];	//reset RR tak, żeby zawierał już próbki od maximum do końca Window search
	  				  }

	  				  intervals_avarage = 0;
	  				  for(i = 0; i < QRS_INIT_ITERATIONS; i++){
	  					intervals_avarage += QRS_prop.intervals[i];
	  				  }
	  				 // intervals_avarage = (intervals_avarage - min - max) / (QRS_INIT_ITERATIONS - 2);
	  				 intervals_avarage = intervals_avarage / QRS_INIT_ITERATIONS;

	  				 pulse = (60 * F_SAMPLE) / intervals_avarage;

	  				 //ustawianie max i min pulse
	  				 if(init_phase != 0){
		  				 if(pulse > max_pulse){
		  					 max_pulse = pulse;
		  				 }

		  				 if(pulse < min_pulse){
		  					 min_pulse = pulse;
		  				 }
		  				 else if(min_pulse == -1){
		  					 min_pulse = pulse;
		  				 }

	  				 }



	  				  m_value_head = 0;
	  				  Stan_algorytmu = RELAX;
	  			  }
	  		  }
	  		  break;

	  	  case RELAX:

	  		  if(sample_skip < SKIP_VALUE){
	  			  sample_skip++;
	  			  RR++;
	  		  }
	  		  else{
	  			  sample_skip = 0;
	  			  Stan_algorytmu = TRESHOLD_SEARCH;
	  		  }

	  		  break;
	  	  }

	  	  //###################################### KONIEC PROCESU WYKRYWANIA QRS ######################################


	  	  GraphBuffer_Add(&graphBuffer,  rescaled_value);
	  	  if(abs(rescaled_value - data_new) < 2){
	  		GraphBuffer_Add(&ZoomedOutGraphBuffer, data_new);

	  	  }
	  	  else{
	  		GraphBuffer_Add(&ZoomedOutGraphBuffer, rescaled_value);
	  		data_new = rescaled_value;
	  	  }



	  	  if(init_phase != 0){
		  	  if(rescaled_value > max_adc_value){
		  		  lcd_draw_horizontal_line(max_adc_value, BLACK);
		  		  max_adc_value = rescaled_value;
		  	  }

		  	  if(rescaled_value < min_adc_value && rescaled_value != 0){
		  		  lcd_draw_horizontal_line(min_adc_value, BLACK);
		  		  min_adc_value = rescaled_value;
		  	  }
	  	  }




	  	  char txtToSD[8] = {0,0,0,0,0,0,0,0};	// 21 cycles

	  	  sprintf(txtToSD, "%li", rescaled_value);


	  	  if(SD_counter != -1){
			  if(++SD_counter < 20000){

				f_open(&fil, "write.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
				f_lseek(&fil, f_size(&fil));
				int o =  f_puts(txtToSD, &fil);
				f_puts(";", &fil);
				f_close(&fil);
			  }
			  else{
				f_mount(&fs, "", 0);
				SD_counter = -1;
			  }							// 777 cycles

	  	  }


	  	__WFI();	//uśpienie mikrokontrolera


	  	  	  //264 322 cycles
  }
  else if(htim == &htim7){
	  HAL_TIM_Base_Stop_IT(&htim6);

  	  //fill_with(BACKGROUND_COLOR);
  	  DrawAxis(X_AXIS_OFFSET, Y_AXIS_OFFSET, AXIS_COLOR);

  	  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
  	  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
  	  seconds = time.Seconds;
  	  minutes = time.Minutes;
  	  hours = time.Hours;



  	  sprintf(timer, "0:%d:%d", minutes, seconds);//hours, minutes, seconds);
  	  sprintf(pulse_tab, "%d", pulse);
  	  sprintf(min_pulse_tab, "min pulse: %d /min", min_pulse);
  	  sprintf(max_pulse_tab, "max pulse: %d /min", max_pulse);

  	  lcd_fill_box(5, 5, LCD_WIDTH, 25, BACKGROUND_COLOR);
  	  LCD_DisplayString(5, 5, timer, WHITE);
  	  LCD_DisplayString(5, 15, pulse_tab, WHITE);
  	  LCD_DisplayString(40, 5, min_pulse_tab, WHITE);
  	  LCD_DisplayString(40, 15, max_pulse_tab, WHITE);
      lcd_draw_horizontal_line(min_adc_value, BLUE);
      lcd_draw_horizontal_line(max_adc_value, RED);



  	  switch(display_mode){

  	  	  case ZOOM_OUT:

  	  		  if (xPos >= 160){
  	  		  	  xPos = 0;
  	  		  	  fill_with(BACKGROUND_COLOR);
  	  	          lcd_draw_horizontal_line(min_adc_value, BLUE);
  	  	          lcd_draw_horizontal_line(max_adc_value, RED);
  	  		  }
  	  		  else{

  	  			  // increment the horizontal position:
  	  			  if(ZoomedOutGraphBuffer.head != 0){
  	  				  LCD_DrawLine(xPos - 1, LCD_HEIGHT - ZoomedOutGraphBuffer.data[ZoomedOutGraphBuffer.head - 1], xPos, LCD_HEIGHT - ZoomedOutGraphBuffer.data[ZoomedOutGraphBuffer.head], CHART_COLOR);
  	  			  }
  	  			  else{
  	  				  LCD_DrawLine(xPos - 1, LCD_HEIGHT - data_old, xPos, LCD_HEIGHT - data_new, CHART_COLOR);
  	  			  }
  	  	    	  LCD_DisplayString(40, 5, min_pulse_tab, WHITE);
  	  	    	  LCD_DisplayString(40, 15, max_pulse_tab, WHITE);


  	  		  	  data_old = data_new;
  	  		  	  xPos++;
  	  		  }

  	  		  break;

  	  	  case ZOOM_IN:

  	  		  fill_with(BACKGROUND_COLOR);
  	    	  LCD_DisplayString(5, 5, timer, WHITE);
  	    	  LCD_DisplayString(5, 15, pulse_tab, WHITE);
  	    	  LCD_DisplayString(40, 5, min_pulse_tab, WHITE);
  	    	  LCD_DisplayString(40, 15, max_pulse_tab, WHITE);
  	          lcd_draw_horizontal_line(min_adc_value, BLUE);
  	          lcd_draw_horizontal_line(max_adc_value, RED);
  	  		  DrawAxis(X_AXIS_OFFSET, Y_AXIS_OFFSET, AXIS_COLOR);
  	  		  for(int i = 0, j = Y_AXIS_OFFSET; i < GRAPH_BUFFOR_SIZE - Y_AXIS_OFFSET; i++, j++){
  	  			  lcd_put_pixel(j, LCD_HEIGHT - graphBuffer.data[i], CHART_COLOR);								//wykres moze być do góry nogami, zrobienie LCD_HEIGHT - graphbuffor powoduje hard faulta
  	  		  }

  	  		  break;
  	  }

  	  lcd_copy();//5216 cycles
  	  HAL_TIM_Base_Start_IT(&htim6);
  	  __WFI();	//uśpienie mikrokontrolera


  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	derivativeBuffer.previous = 0;
	derivativeBuffer.current =  0;
	derivativeBuffer.next =     0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	RTC_TimeTypeDef reset_time;
	reset_time.Hours = 0;
	reset_time.Minutes = 0;
	reset_time.Seconds = 0;
	HAL_RTC_SetTime(&hrtc, &reset_time, RTC_FORMAT_BIN);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_init();

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  lcd_copy();

  GraphBuffer_Init(&graphBuffer);
  GraphBuffer_Init(&ZoomedOutGraphBuffer);
  MedianBuffer_Init(&mBuffer);
  LowPassBuffer_Init(&LowPass);


  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  HAL_Delay(500);
  int x = f_mount(&fs, "", 0);
  int y = f_open(&fil, "write.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
  int z = f_lseek(&fil, f_size(&fil));
  int v = f_puts("Hello from Kamil\n", &fil);

  int b = f_close(&fil);
  int n = f_mount(&fs, "", 0);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET) {
		  if(display_mode == ZOOM_OUT && already_changed == 0){

  	  		  fill_with(BACKGROUND_COLOR);
  	    	  LCD_DisplayString(5, 5, timer, WHITE);
  	    	  LCD_DisplayString(5, 15, pulse_tab, WHITE);

  	  		  DrawAxis(X_AXIS_OFFSET, Y_AXIS_OFFSET, AXIS_COLOR);
  	  		  lcd_copy();
  	  		  xPos = 1;

			  display_mode = ZOOM_IN;
			  already_changed = 1;
		  }
		  else if(display_mode == ZOOM_IN && already_changed == 0){

  	  		  fill_with(BACKGROUND_COLOR);
  	    	  LCD_DisplayString(5, 5, timer, WHITE);
  	    	  LCD_DisplayString(5, 15, pulse_tab, WHITE);
  	  		  DrawAxis(X_AXIS_OFFSET, Y_AXIS_OFFSET, AXIS_COLOR);
  	  		  lcd_copy();

			  display_mode = ZOOM_OUT;
			  already_changed = 1;
		  }
	  }
	  else{
		  already_changed = 0;
	  }
	  __WFI();


	  //HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi2)
	{
		lcd_transfer_done();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
