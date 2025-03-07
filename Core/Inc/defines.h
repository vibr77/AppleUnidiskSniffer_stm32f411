

/*
DATA            MNEMONIC  DESCRIPTION                           TIME
(SmartPort Bus)           (Relative)
_____________________________________________________________________________       Offset 
FF              SYNC      SELF SYNCHRONIZING BYTES              0                   0
3F              :         :                                     32 micro Sec.       1
CF              :         :                                     32 micro Sec.       2
F3              :         :                                     32 micro Sec.       3
FC              :         :                                     32 micro Sec.       4
FF              :         :                                     32 micro Sec.       5
C3              PBEGIN    MARKS BEGINNING OF PACKET             32 micro Sec.       6   00
81              DEST      DESTINATION UNIT NUMBER               32 micro Sec.       7   01
80              SRC       SOURCE UNIT NUMBER                    32 micro Sec.       8   02
80              TYPE      PACKET TYPE FIELD                     32 micro Sec.       9   03
80              AUX       PACKET AUXILLIARY TYPE FIELD          32 micro Sec.      10   04
80              STAT      DATA STATUS FIELD                     32 micro Sec.      11   05
82              ODDCNT    ODD BYTES COUNT                       32 micro Sec.      12   06
81              GRP7CNT   GROUP OF 7 BYTES COUNT                32 micro Sec.      13   07
80              ODDMSB    ODD BYTES MSB's                       32 micro Sec.      14   08
81              COMMAND   1ST ODD BYTE = Command Byte           32 micro Sec.      15   09
83              PARMCNT   2ND ODD BYTE = Parameter Count        32 micro Sec.      16   10
80              GRP7MSB   MSB's FOR 1ST GROUP OF 7              32 micro Sec.      17   11
80              G7BYTE1   BYTE 1 FOR 1ST GROUP OF 7             32 micro Sec.      18   12
98              G7BYTE2   BYTE 2 FOR 1ST GROUP OF 7             32 micro Sec.      19   13
82              G7BYTE3   BYTE 3 FOR 1ST GROUP OF 7             32 micro Sec.      20   14
80              G7BYTE4   BYTE 4 FOR 1ST GROUP OF 7             32 micro Sec.      21   15
80              G7BYTE5   BYTE 5 FOR 1ST GROUP OF 7             32 micro Sec.      22   16
80              G7BYTE6   BYTE 6 FOR 1ST GROUP OF 7             32 micro Sec.      23   17
80              G7BYTE7   BYTE 7 FOR 1ST GROUP OF 7             32 micro Sec.      24   18
BB              CHKSUM1   1ST BYTE OF CHECKSUM                  32 micro Sec.      25   19
EE              CHKSUM2   2ND BYTE OF CHECKSUM                  32 micro Sec.      26   20
C8              PEND      PACKET END BYTE                       32 micro Sec.      27   21
00              FALSE     FALSE IWM WRITE TO CLEAR REGISTER     32 micro Sec.      28   22
_____________________________________________________________________________

Ex of Data packet :ADC_CLOCKPRESCALER_PCLK_DIV1
0000: C3 81 80 82 80 80 86 80 86 80 80 80 A4 D9 E7 BF - ..�.��.�.���....
0010: EF C8 

C3                  0   PBEGIN
81                  1   DEST
80                  2   SRC
82                  3   TYPE 
80                  4   AUX
80                  5   STAT
86                  6   ODDCNT
80                  7   GRPCNT
86                  8   ODD MSB
80 80 80 A4 D9 E7 
BF EF               CHKSUM
C8                  PEND



*/


#define SP_PKT_SIZE                 604

#define SP_PBEGIN   0
#define SP_DEST     1
#define SP_SRC      2
#define SP_TYPE     3
#define SP_AUX      4
#define SP_STAT     5
#define SP_ODDCNT   6
#define SP_GRP7CNT  7
#define SP_ODDMSB   8
#define SP_COMMAND  9
#define SP_PARMCNT  10
#define SP_GRP7MSB  11
#define SP_G7BYTE1  12
#define SP_G7BYTE2  13
#define SP_G7BYTE3  14
#define SP_G7BYTE4  15
#define SP_G7BYTE5  16
#define SP_G7BYTE6  17
#define SP_G7BYTE7  18