/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for Hello World Example using HAL APIs.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "cy_pdl.h"
#include "cy_crypto_core_hw.h"
#include "cy_crypto_core_hw_v2.h"
#include "string.h"
#include "stdlib.h"
#include "stdint.h"
#include "limits.h"


/*******************************************************************************
* Macros
*******************************************************************************/

//size of zeta
#define N (uint32_t)(256u)

//size of data and result 
#define SIZE (uint32_t)(64u)

#define DILITHIUM_N (uint32_t)(256u)

#define DILITHIUM_QINV          58728449

#define DILITHIUM_Q                     0x7fe001
//Polynomials for programmable.
#define CY_CONFIG_TR_GARO_CTL      0x6C740B8DuL //Galois ring oscillator.
#define CY_CONFIG_TR_FIRO_CTL      0x52D246E1uL // Fibonacci ring oscillator.
#define CY_TR_MAX_BITS             32U

#define INT64_MAX_VALUE INT64_MAX
#define UINT64_MAX_VALUE UINT64_MAX



#define SHIFT_13 (uint32_t)(13u)
#define SHIFT_23 (uint32_t)(23u)
#define SHIFT_26 (uint32_t)(26u)
#define SHIFT_32 (uint32_t)(32u)


/*******************************************************************************
* Global Variables
*******************************************************************************/

//96bit data type [Note: but the size is 128bit in total as the 32bits gets aligned to 64bit]
typedef struct{
    int64_t data;
    int64_t carry;
} int128_t;


CY_ALIGN(4) int128_t data[DILITHIUM_N]; // used to store the DATA
CY_ALIGN(4) int128_t data1[DILITHIUM_N]; // used to store the DATA
CY_ALIGN(4) static const int32_t zetas[N] = {
         0,    25847, -2608894,  -518909,   237124,  -777960,  -876248,   466468,
   1826347,  2353451,  -359251, -2091905,  3119733, -2884855,  3111497,  2680103,
   2725464,  1024112, -1079900,  3585928,  -549488, -1119584,  2619752, -2108549,
  -2118186, -3859737, -1399561, -3277672,  1757237,   -19422,  4010497,   280005,
   2706023,    95776,  3077325,  3530437, -1661693, -3592148, -2537516,  3915439,
  -3861115, -3043716,  3574422, -2867647,  3539968,  -300467,  2348700,  -539299,
  -1699267, -1643818,  3505694, -3821735,  3507263, -2140649, -1600420,  3699596,
    811944,   531354,   954230,  3881043,  3900724, -2556880,  2071892, -2797779,
  -3930395, -1528703, -3677745, -3041255, -1452451,  3475950,  2176455, -1585221,
  -1257611,  1939314, -4083598, -1000202, -3190144, -3157330, -3632928,   126922,
   3412210,  -983419,  2147896,  2715295, -2967645, -3693493,  -411027, -2477047,
   -671102, -1228525,   -22981, -1308169,  -381987,  1349076,  1852771, -1430430,
  -3343383,   264944,   508951,  3097992,    44288, -1100098,   904516,  3958618,
  -3724342,    -8578,  1653064, -3249728,  2389356,  -210977,   759969, -1316856,
    189548, -3553272,  3159746, -1851402, -2409325,  -177440,  1315589,  1341330,
   1285669, -1584928,  -812732, -1439742, -3019102, -3881060, -3628969,  3839961,
   2091667,  3407706,  2316500,  3817976, -3342478,  2244091, -2446433, -3562462,
    266997,  2434439, -1235728,  3513181, -3520352, -3759364, -1197226, -3193378,
    900702,  1859098,   909542,   819034,   495491, -1613174,   -43260,  -522500,
   -655327, -3122442,  2031748,  3207046, -3556995,  -525098,  -768622, -3595838,
    342297,   286988, -2437823,  4108315,  3437287, -3342277,  1735879,   203044,
   2842341,  2691481, -2590150,  1265009,  4055324,  1247620,  2486353,  1595974,
  -3767016,  1250494,  2635921, -3548272, -2994039,  1869119,  1903435, -1050970,
  -1333058,  1237275, -3318210, -1430225,  -451100,  1312455,  3306115, -1962642,
  -1279661,  1917081, -2546312, -1374803,  1500165,   777191,  2235880,  3406031,
   -542412, -2831860, -1671176, -1846953, -2584293, -3724270,   594136, -3776993,
  -2013608,  2432395,  2454455,  -164721,  1957272,  3369112,   185531, -1207385,
  -3183426,   162844,  1616392,  3014001,   810149,  1652634, -3694233, -1799107,
  -3038916,  3523897,  3866901,   269760,  2213111,  -975884,  1717735,   472078,
   -426683,  1723600, -1803090,  1910376, -1667432, -1104333,  -260646, -3833893,
  -2939036, -2235985,  -420899, -2286327,   183443,  -976891,  1612842, -3545687,
   -554416,  3919660,   -48306, -1362209,  3937738,  1400424,  -846154,  1976782
}; 
CY_ALIGN(4) int128_t temp1[DILITHIUM_N];// used to store intermediate operation data
CY_ALIGN(4) int128_t temp2[DILITHIUM_N];// used to store intermediate operation data
CY_ALIGN(4) int128_t temp3[DILITHIUM_N];// used to tamper the final result in montgomery reduction
CY_ALIGN(4) int128_t result_data[DILITHIUM_N];// used to store the FINAL RESULT
CY_ALIGN(4) int128_t final_result_data[DILITHIUM_N];// used to store the final result `t`
CY_ALIGN(4) uint32_t value;
CY_ALIGN(4) int flag;
CY_ALIGN(4) uint8_t shift_value;

CY_ALIGN(4) int128_t sample[4], sample1[4];
/*******************************************************************************
* Function Prototypes 
*******************************************************************************/

void waitForFifo(CRYPTO_Type *base);

void dataMultiplyZeta(CRYPTO_Type *base, int64_t zeta);
void dataMultiplyZetaBatch(CRYPTO_Type *base, int128_t *r, int64_t zeta, int startIdx, int size);
void dilithium_ntt(CRYPTO_Type *base, int128_t *r);
void dilithium_ntt_orginal(int128_t *r);
void mont_red(CRYPTO_Type *base, int128_t *rs, int startIdx, int size);
static int32_t dilithium_mont_red(int64_t a);
/*******************************************************************************
*      waitForFifo Implementation
*******************************************************************************/

void waitForFifo(CRYPTO_Type *base){
    while(Cy_Crypto_Core_GetFIFOUsed(base) == Cy_Crypto_Core_GetFIFODepth(base)){   
    }
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. 
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif /* #if defined (CY_DEVICE_SECURE) */

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CYBSP_DEBUG_UART_CTS,CYBSP_DEBUG_UART_RTS,CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Crypto core enable*/
    if(Cy_Crypto_Core_Enable(CRYPTO)!=CY_CRYPTO_SUCCESS)
        CY_ASSERT(0);

    // reseting the memory and the registers
    Cy_Crypto_Core_ClearVuRegisters(CRYPTO);
    memset((void *)0x40108000, 0, 1024 * 4);

    for (int i = 0; i < 256; i++) {
        // Map generated random uint32_t range [0, 2^32-1] to signed range [-2^31, 2^31-1]
        Cy_Crypto_Core_Trng(CRYPTO, CY_CONFIG_TR_GARO_CTL, CY_CONFIG_TR_FIRO_CTL, MAX_TRNG_BIT_SIZE, &value);
        //data[i].data = (int32_t)(value - 2147483648U); // 2^31
        data[i].data = (int32_t)value;
        data1[i].data = (int32_t)value;
    }

    // sample[0].data = 0x2;
    // sample[1].data = 0x3;
    // sample[2].data = 0x4;
    // sample[3].data = 0x5;

    // // sample1[3].data = 0x2;
    // // sample1[2].data = 0x3;
    // // sample1[1].data = 0x4;
    // // sample1[0].data = 0x5;
    // // mont_red(CRYPTO, sample, 0, 4);
    // // Multiplying the Data and zeta values
    // mont_red(CRYPTO, sample,0,4);
    // Cy_Crypto_Core_ClearVuRegisters(CRYPTO);
    // memset((void *)0x40108000, 0, 1024 * 4);


    // //checking the result
    // flag = 0;
    // for(int i=0;i<64;i++){
    //     temp2[i].data = data[i].data*(int64_t)zetas[1];
    //     flag = i;
    //     result_data[i].carry = 0x0;
    //     if(temp2[i].data!=result_data[i].data){
    //         break;
    //     }
    //     temp2[i].data = 0;
    // }

    dilithium_ntt(CRYPTO, data);

    Cy_Crypto_Core_ClearVuRegisters(CRYPTO);
    memset((void *)0x40108000, 0, 1024 * 4);
    
    dilithium_ntt_orginal(data1);
    int i;
    for(i=0;i<DILITHIUM_N; i++){
        if(data[i].data != data1[i].data){
            break;
        }
    }

    // mont_red(CRYPTO);

    //checlking the final result
    // flag = 0;
    // for(int i=0;i<64;i++){
    //     temp2[i].data = result_data[i].data - (result_data[i].data * 58728449) * 8380417;
    //     flag = i;
    //     final_result_data[i].carry = 0x0;
    //     if(temp2[i].data!=final_result_data[i].data){
    //         break;
    //     }
    //     temp2[i].data = 0;
    // }
    while(1);

}


/*******************************************************************************
*      dataMultiplyZeta Implementation
*******************************************************************************/
void dataMultiplyZeta(CRYPTO_Type *base, int64_t zeta){
    //Allocating the memory to the register 
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG0, sizeof(data)*8); // register `a` => 64 * 96  => 64 elements each of 96bit of size, but due alignment each element size is 128bit in memory region
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG1, sizeof(zeta)*8); // register `b` => 64 bit value 
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG2, sizeof(result_data)*8);//register `c` => 64 * 96  => 64 elements each of 96bit of size, but due alignment each element size is 128bit in memory region
    //wait for FIFO instruction set to be free
    waitForFifo(base);
    //loading the data to register `0` and zeta value to register `1`
    Cy_Crypto_Core_Vu_SetMemValue(base, CY_CRYPTO_VU_HW_REG0, (uint8_t *)&data, sizeof(data)*8);// register `a` ==> data
    Cy_Crypto_Core_Vu_SetMemValue(base, CY_CRYPTO_VU_HW_REG1, (uint8_t *)&zeta, sizeof(zeta)*8);// register `b` ==> zeta
    CY_CRYPTO_VU_UMUL(base, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG0, CY_CRYPTO_VU_HW_REG1);
    //wait for FIFO instruction set to be free
    waitForFifo(base);
    //Using UMUL on `REG0` and `REG1` and storing the result into `REG2`
    //copying the result from `REG2` to result variable
    Cy_Crypto_Core_Vu_GetMemValue(base, (uint8_t *)&result_data, CY_CRYPTO_VU_HW_REG2, sizeof(result_data)*8);
}


void dataMultiplyZetaBatch(CRYPTO_Type *base, int128_t *r, int64_t zeta, int startIdx, int size) 
{
    uint32_t alloc_mask = CY_CRYPTO_VU_REG_BIT(CY_CRYPTO_VU_HW_REG0) | CY_CRYPTO_VU_REG_BIT(CY_CRYPTO_VU_HW_REG1) | CY_CRYPTO_VU_REG_BIT(CY_CRYPTO_VU_HW_REG2);
    // Allocate memory for processing a batch of elements
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG0, sizeof(int128_t) * size * 8);  // Multiplicands (r)
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG1, sizeof(int64_t) * 8);  // Multiplier (zeta)
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG2, sizeof(int128_t) * size * 8);  // Result storage

    // Load the `r` values (data) into hardware memory
    waitForFifo(base);
    Cy_Crypto_Core_WaitForReady(base);
    Cy_Crypto_Core_Vu_SetMemValue(base, CY_CRYPTO_VU_HW_REG0, (uint8_t *)&r[startIdx], sizeof(int128_t) * size * 8);
    
    // Load the `zeta` value into hardware memory
    Cy_Crypto_Core_Vu_SetMemValue(base, CY_CRYPTO_VU_HW_REG1, (uint8_t *)&zeta, sizeof(zeta) * 8);

    // Perform the multiplication in parallel using the hardware accelerator
    CY_CRYPTO_VU_UMUL(base, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG0, CY_CRYPTO_VU_HW_REG1);
    
    waitForFifo(base);
    Cy_Crypto_Core_WaitForReady(base);
    
    // Retrieve the results into `result_data`
    Cy_Crypto_Core_Vu_GetMemValue(base, (uint8_t *)&result_data[startIdx], CY_CRYPTO_VU_HW_REG2, sizeof(int128_t) * size * 8);
    for(int i=startIdx; i< startIdx + size;i++ ){
        result_data[i].carry = 0;
    }

    CY_CRYPTO_VU_FREE_MEM(base, alloc_mask);
}


/*******************************************************************************
*      mont_red Implementation
*******************************************************************************/
void mont_red(CRYPTO_Type *base, int128_t *rs, int startIdx, int size){
    uint32_t InputData_addr, Temp2_addr, Reg3_addr;
    uint32_t alloc_mask = CY_CRYPTO_VU_REG_BIT(CY_CRYPTO_VU_HW_REG0) | CY_CRYPTO_VU_REG_BIT(CY_CRYPTO_VU_HW_REG1) | CY_CRYPTO_VU_REG_BIT(CY_CRYPTO_VU_HW_REG2) | CY_CRYPTO_VU_REG_BIT(CY_CRYPTO_VU_HW_REG3) | CY_CRYPTO_VU_REG_BIT(CY_CRYPTO_VU_HW_REG5);
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG0,  sizeof(int128_t) * size * 8);
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG1,  sizeof(int128_t) * size * 8); // used to store the intermediate result of the calculation
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG2,  sizeof(int128_t) * size * 8); // used to store the intermediate result of the calculation
    CY_CRYPTO_VU_ALLOC_MEM(base, CY_CRYPTO_VU_HW_REG3,  sizeof(int128_t) * size * 8); // used to store the intermediate result of the calculation [`t` value is used to store]
    InputData_addr = Cy_Crypto_Core_Vu_RegMemPointer(base, CY_CRYPTO_VU_HW_REG0);
    Temp2_addr = Cy_Crypto_Core_Vu_RegMemPointer(base, CY_CRYPTO_VU_HW_REG2);
    Reg3_addr = Cy_Crypto_Core_Vu_RegMemPointer(base, CY_CRYPTO_VU_HW_REG3);
    waitForFifo(base);
    for(int i = 0; i<0+size; i++){
        *(int32_t *)(InputData_addr + (i)*sizeof(int128_t)) = rs[i+startIdx].data;
    }
    Cy_Crypto_Core_WaitForReady(base);
    waitForFifo(base);
    CY_CRYPTO_VU_SET_REG(base, CY_CRYPTO_VU_HW_REG5, SHIFT_13, sizeof(SHIFT_13));
    CY_CRYPTO_VU_LSL(base, CY_CRYPTO_VU_HW_REG1, CY_CRYPTO_VU_HW_REG0, CY_CRYPTO_VU_HW_REG5); // temp1 = Input_data << 13
    CY_CRYPTO_VU_ADD(base, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG1, CY_CRYPTO_VU_HW_REG0); // temp2 = temp1 + Input_data
    Cy_Crypto_Core_WaitForReady(base);
    waitForFifo(base);
    CY_CRYPTO_VU_SET_REG(base, CY_CRYPTO_VU_HW_REG5, SHIFT_26, sizeof(SHIFT_26));
    CY_CRYPTO_VU_LSL(base, CY_CRYPTO_VU_HW_REG1, CY_CRYPTO_VU_HW_REG0, CY_CRYPTO_VU_HW_REG5); // temp1 = Input_data << 26
    CY_CRYPTO_VU_ADD(base, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG1); // temp2 = temp2 + temp1
    Cy_Crypto_Core_WaitForReady(base);
    waitForFifo(base);
    CY_CRYPTO_VU_SET_REG(base, CY_CRYPTO_VU_HW_REG5, SHIFT_23, sizeof(SHIFT_23));
    CY_CRYPTO_VU_LSL(base, CY_CRYPTO_VU_HW_REG1, CY_CRYPTO_VU_HW_REG0, CY_CRYPTO_VU_HW_REG5); // temp1 = Input_data << 23
    CY_CRYPTO_VU_SUB(base, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG1); // temp2 = temp2 - temp1
    Cy_Crypto_Core_WaitForReady(base);
    waitForFifo(base);
    for(int i = 0; i<size; i++){
        *(int64_t *)(Temp2_addr + (i)*sizeof(int128_t)) = (int32_t)(*(uint64_t *)(Temp2_addr + (i)*sizeof(int128_t)) & 0xffffffff);
    }
    CY_CRYPTO_VU_SET_REG(base, CY_CRYPTO_VU_HW_REG5, SHIFT_23, sizeof(SHIFT_23));
    CY_CRYPTO_VU_LSL(base, CY_CRYPTO_VU_HW_REG1, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG5); // temp1 = temp2 << 23
    CY_CRYPTO_VU_ADD(base, CY_CRYPTO_VU_HW_REG3, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG1); // Output_data = temp2 - temp1
    Cy_Crypto_Core_WaitForReady(base);
    waitForFifo(base);
    CY_CRYPTO_VU_SET_REG(base, CY_CRYPTO_VU_HW_REG5, SHIFT_13, sizeof(SHIFT_13));
    CY_CRYPTO_VU_LSL(base, CY_CRYPTO_VU_HW_REG1, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG5); // temp1 = temp2 << 13
    CY_CRYPTO_VU_SUB(base, CY_CRYPTO_VU_HW_REG3, CY_CRYPTO_VU_HW_REG3, CY_CRYPTO_VU_HW_REG1); // Output_data = Output_data - temp1
    Cy_Crypto_Core_WaitForReady(base);
    waitForFifo(base);
    for(int i = 0; i<size; i++){
        *(int64_t *)(InputData_addr + (i)*sizeof(int128_t)) = (int64_t)rs[i+startIdx].data;
        *(int64_t *)(Reg3_addr + (i)*sizeof(int128_t)+8) = 0;
    }
    CY_CRYPTO_VU_SUB(base, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG0, CY_CRYPTO_VU_HW_REG3); // temp2 = Input_data - Output_data
    Cy_Crypto_Core_WaitForReady(base);
    for(int i = 0; i<size; i++){
        if((*(int32_t *)(Temp2_addr + (i)*sizeof(int128_t))) == 0xffffffff){
            *(int64_t *)(Temp2_addr + (i)*sizeof(int128_t)) += 1;
        }
        *(int64_t *)(Temp2_addr + (i)*sizeof(int128_t)+8) = 0;
    }
    CY_CRYPTO_VU_SET_REG(base, CY_CRYPTO_VU_HW_REG5, SHIFT_32, sizeof(SHIFT_32));
    CY_CRYPTO_VU_LSR(base, CY_CRYPTO_VU_HW_REG3, CY_CRYPTO_VU_HW_REG2, CY_CRYPTO_VU_HW_REG5); // Output_data = temp2 >> 32
    
    Cy_Crypto_Core_Vu_GetMemValue(base, (uint8_t *)&final_result_data[startIdx], CY_CRYPTO_VU_HW_REG3, sizeof(int128_t) * size * 8);

    CY_CRYPTO_VU_FREE_MEM(base, alloc_mask);
}


void dilithium_ntt(CRYPTO_Type *base, int128_t *r)
{
    unsigned int len;
    unsigned int k;
    unsigned int j;
    k = 0;
    flag = 0;
    for (len = DILITHIUM_N / 2; len >= 1; len >>= 1) {
        unsigned int start;
        for (start = 0; start < DILITHIUM_N; start = j + len) {
            int32_t zeta = zetas[++k];  // Get the zeta value
            // Parallel multiplication with zeta for r[j + len]
            if(len > 64){
                //sending elements in chunks 
                dataMultiplyZetaBatch(base, r, zeta, start+len, len/2);
                dataMultiplyZetaBatch(base, r, zeta, (start+len)+(len)/2, len/2);
                // Cy_Crypto_Core_ClearVuRegisters(base);
                // memset((void *)0x40108000, 0, 1024 * 4);
                mont_red(base, result_data, start+len, ((len)/2));
                // Cy_Crypto_Core_ClearVuRegisters(base);
                // memset((void *)0x40108000, 0, 1024 * 4);
                mont_red(base, result_data, (start+len)+(len)/2, ((len)/2));
            }else{
                dataMultiplyZetaBatch(base, r, zeta, start + len, len);
                // Cy_Crypto_Core_ClearVuRegisters(base);
                // memset((void *)0x40108000, 0, 1024 * 4);
                mont_red(base, result_data, start + len, len);
            }
            for (j = start; j < start + len; ++j) {

                // The dilithium_mont_red function seems to be part of the original multiplication,
                // but we're now directly using the result from dataMultiplyZeta. We should consider
                // if any further operations are needed with the results.
                // int32_t t = dilithium_mont_red((int64_t)zeta * r[j + len].data);
                // if(t!=((int32_t)(final_result_data[j+len].data))){
                //     printf("%ld %ld",t,j);
                //     break;
                // }
                int32_t t = (int32_t)final_result_data[j+len].data;
                int128_t rj = r[j];
                r[j + len].data = rj.data - t;
                r[j].data = rj.data + t;
            }
            memset((void *)&final_result_data, 0, sizeof(final_result_data));
            memset((void *)&result_data, 0, sizeof(result_data));
            // Cy_Crypto_Core_ClearVuRegisters(base);
            // memset((void *)0x40108000, 0, 1024 * 4);
        }
    }

}

void dilithium_ntt_orginal(int128_t *r)
{
    unsigned int len;
    unsigned int k;
    unsigned int j;
    k = 0;
    flag = 0;
    for (len = DILITHIUM_N / 2; len >= 1; len >>= 1) {
        unsigned int start;
        for (start = 0; start < DILITHIUM_N; start = j + len) {
            int32_t zeta = zetas[++k];  // Get the zeta value
            for (j = start; j < start + len; ++j) {
                int32_t t = dilithium_mont_red((int64_t)zeta * r[j + len].data);
                int128_t rj = r[j];
                r[j + len].data = rj.data - t;
                r[j].data = rj.data + t;
            }
        }
    }

}

static int32_t dilithium_mont_red(int64_t a)
{
    int64_t t = (int32_t)((int32_t)a * (int32_t)DILITHIUM_QINV);
    return (int32_t)((a - ((int32_t)t * (int64_t)DILITHIUM_Q)) >> 32);

}
/* [] END OF FILE */

