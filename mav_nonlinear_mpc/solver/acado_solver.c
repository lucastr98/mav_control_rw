/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 135 */
real_t state[ 135 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 9];
state[1] = acadoVariables.x[lRun1 * 9 + 1];
state[2] = acadoVariables.x[lRun1 * 9 + 2];
state[3] = acadoVariables.x[lRun1 * 9 + 3];
state[4] = acadoVariables.x[lRun1 * 9 + 4];
state[5] = acadoVariables.x[lRun1 * 9 + 5];
state[6] = acadoVariables.x[lRun1 * 9 + 6];
state[7] = acadoVariables.x[lRun1 * 9 + 7];
state[8] = acadoVariables.x[lRun1 * 9 + 8];

state[117] = acadoVariables.u[lRun1 * 3];
state[118] = acadoVariables.u[lRun1 * 3 + 1];
state[119] = acadoVariables.u[lRun1 * 3 + 2];
state[120] = acadoVariables.od[lRun1 * 15];
state[121] = acadoVariables.od[lRun1 * 15 + 1];
state[122] = acadoVariables.od[lRun1 * 15 + 2];
state[123] = acadoVariables.od[lRun1 * 15 + 3];
state[124] = acadoVariables.od[lRun1 * 15 + 4];
state[125] = acadoVariables.od[lRun1 * 15 + 5];
state[126] = acadoVariables.od[lRun1 * 15 + 6];
state[127] = acadoVariables.od[lRun1 * 15 + 7];
state[128] = acadoVariables.od[lRun1 * 15 + 8];
state[129] = acadoVariables.od[lRun1 * 15 + 9];
state[130] = acadoVariables.od[lRun1 * 15 + 10];
state[131] = acadoVariables.od[lRun1 * 15 + 11];
state[132] = acadoVariables.od[lRun1 * 15 + 12];
state[133] = acadoVariables.od[lRun1 * 15 + 13];
state[134] = acadoVariables.od[lRun1 * 15 + 14];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 9] = state[0] - acadoVariables.x[lRun1 * 9 + 9];
acadoWorkspace.d[lRun1 * 9 + 1] = state[1] - acadoVariables.x[lRun1 * 9 + 10];
acadoWorkspace.d[lRun1 * 9 + 2] = state[2] - acadoVariables.x[lRun1 * 9 + 11];
acadoWorkspace.d[lRun1 * 9 + 3] = state[3] - acadoVariables.x[lRun1 * 9 + 12];
acadoWorkspace.d[lRun1 * 9 + 4] = state[4] - acadoVariables.x[lRun1 * 9 + 13];
acadoWorkspace.d[lRun1 * 9 + 5] = state[5] - acadoVariables.x[lRun1 * 9 + 14];
acadoWorkspace.d[lRun1 * 9 + 6] = state[6] - acadoVariables.x[lRun1 * 9 + 15];
acadoWorkspace.d[lRun1 * 9 + 7] = state[7] - acadoVariables.x[lRun1 * 9 + 16];
acadoWorkspace.d[lRun1 * 9 + 8] = state[8] - acadoVariables.x[lRun1 * 9 + 17];

acadoWorkspace.evGx[lRun1 * 81] = state[9];
acadoWorkspace.evGx[lRun1 * 81 + 1] = state[10];
acadoWorkspace.evGx[lRun1 * 81 + 2] = state[11];
acadoWorkspace.evGx[lRun1 * 81 + 3] = state[12];
acadoWorkspace.evGx[lRun1 * 81 + 4] = state[13];
acadoWorkspace.evGx[lRun1 * 81 + 5] = state[14];
acadoWorkspace.evGx[lRun1 * 81 + 6] = state[15];
acadoWorkspace.evGx[lRun1 * 81 + 7] = state[16];
acadoWorkspace.evGx[lRun1 * 81 + 8] = state[17];
acadoWorkspace.evGx[lRun1 * 81 + 9] = state[18];
acadoWorkspace.evGx[lRun1 * 81 + 10] = state[19];
acadoWorkspace.evGx[lRun1 * 81 + 11] = state[20];
acadoWorkspace.evGx[lRun1 * 81 + 12] = state[21];
acadoWorkspace.evGx[lRun1 * 81 + 13] = state[22];
acadoWorkspace.evGx[lRun1 * 81 + 14] = state[23];
acadoWorkspace.evGx[lRun1 * 81 + 15] = state[24];
acadoWorkspace.evGx[lRun1 * 81 + 16] = state[25];
acadoWorkspace.evGx[lRun1 * 81 + 17] = state[26];
acadoWorkspace.evGx[lRun1 * 81 + 18] = state[27];
acadoWorkspace.evGx[lRun1 * 81 + 19] = state[28];
acadoWorkspace.evGx[lRun1 * 81 + 20] = state[29];
acadoWorkspace.evGx[lRun1 * 81 + 21] = state[30];
acadoWorkspace.evGx[lRun1 * 81 + 22] = state[31];
acadoWorkspace.evGx[lRun1 * 81 + 23] = state[32];
acadoWorkspace.evGx[lRun1 * 81 + 24] = state[33];
acadoWorkspace.evGx[lRun1 * 81 + 25] = state[34];
acadoWorkspace.evGx[lRun1 * 81 + 26] = state[35];
acadoWorkspace.evGx[lRun1 * 81 + 27] = state[36];
acadoWorkspace.evGx[lRun1 * 81 + 28] = state[37];
acadoWorkspace.evGx[lRun1 * 81 + 29] = state[38];
acadoWorkspace.evGx[lRun1 * 81 + 30] = state[39];
acadoWorkspace.evGx[lRun1 * 81 + 31] = state[40];
acadoWorkspace.evGx[lRun1 * 81 + 32] = state[41];
acadoWorkspace.evGx[lRun1 * 81 + 33] = state[42];
acadoWorkspace.evGx[lRun1 * 81 + 34] = state[43];
acadoWorkspace.evGx[lRun1 * 81 + 35] = state[44];
acadoWorkspace.evGx[lRun1 * 81 + 36] = state[45];
acadoWorkspace.evGx[lRun1 * 81 + 37] = state[46];
acadoWorkspace.evGx[lRun1 * 81 + 38] = state[47];
acadoWorkspace.evGx[lRun1 * 81 + 39] = state[48];
acadoWorkspace.evGx[lRun1 * 81 + 40] = state[49];
acadoWorkspace.evGx[lRun1 * 81 + 41] = state[50];
acadoWorkspace.evGx[lRun1 * 81 + 42] = state[51];
acadoWorkspace.evGx[lRun1 * 81 + 43] = state[52];
acadoWorkspace.evGx[lRun1 * 81 + 44] = state[53];
acadoWorkspace.evGx[lRun1 * 81 + 45] = state[54];
acadoWorkspace.evGx[lRun1 * 81 + 46] = state[55];
acadoWorkspace.evGx[lRun1 * 81 + 47] = state[56];
acadoWorkspace.evGx[lRun1 * 81 + 48] = state[57];
acadoWorkspace.evGx[lRun1 * 81 + 49] = state[58];
acadoWorkspace.evGx[lRun1 * 81 + 50] = state[59];
acadoWorkspace.evGx[lRun1 * 81 + 51] = state[60];
acadoWorkspace.evGx[lRun1 * 81 + 52] = state[61];
acadoWorkspace.evGx[lRun1 * 81 + 53] = state[62];
acadoWorkspace.evGx[lRun1 * 81 + 54] = state[63];
acadoWorkspace.evGx[lRun1 * 81 + 55] = state[64];
acadoWorkspace.evGx[lRun1 * 81 + 56] = state[65];
acadoWorkspace.evGx[lRun1 * 81 + 57] = state[66];
acadoWorkspace.evGx[lRun1 * 81 + 58] = state[67];
acadoWorkspace.evGx[lRun1 * 81 + 59] = state[68];
acadoWorkspace.evGx[lRun1 * 81 + 60] = state[69];
acadoWorkspace.evGx[lRun1 * 81 + 61] = state[70];
acadoWorkspace.evGx[lRun1 * 81 + 62] = state[71];
acadoWorkspace.evGx[lRun1 * 81 + 63] = state[72];
acadoWorkspace.evGx[lRun1 * 81 + 64] = state[73];
acadoWorkspace.evGx[lRun1 * 81 + 65] = state[74];
acadoWorkspace.evGx[lRun1 * 81 + 66] = state[75];
acadoWorkspace.evGx[lRun1 * 81 + 67] = state[76];
acadoWorkspace.evGx[lRun1 * 81 + 68] = state[77];
acadoWorkspace.evGx[lRun1 * 81 + 69] = state[78];
acadoWorkspace.evGx[lRun1 * 81 + 70] = state[79];
acadoWorkspace.evGx[lRun1 * 81 + 71] = state[80];
acadoWorkspace.evGx[lRun1 * 81 + 72] = state[81];
acadoWorkspace.evGx[lRun1 * 81 + 73] = state[82];
acadoWorkspace.evGx[lRun1 * 81 + 74] = state[83];
acadoWorkspace.evGx[lRun1 * 81 + 75] = state[84];
acadoWorkspace.evGx[lRun1 * 81 + 76] = state[85];
acadoWorkspace.evGx[lRun1 * 81 + 77] = state[86];
acadoWorkspace.evGx[lRun1 * 81 + 78] = state[87];
acadoWorkspace.evGx[lRun1 * 81 + 79] = state[88];
acadoWorkspace.evGx[lRun1 * 81 + 80] = state[89];

acadoWorkspace.evGu[lRun1 * 27] = state[90];
acadoWorkspace.evGu[lRun1 * 27 + 1] = state[91];
acadoWorkspace.evGu[lRun1 * 27 + 2] = state[92];
acadoWorkspace.evGu[lRun1 * 27 + 3] = state[93];
acadoWorkspace.evGu[lRun1 * 27 + 4] = state[94];
acadoWorkspace.evGu[lRun1 * 27 + 5] = state[95];
acadoWorkspace.evGu[lRun1 * 27 + 6] = state[96];
acadoWorkspace.evGu[lRun1 * 27 + 7] = state[97];
acadoWorkspace.evGu[lRun1 * 27 + 8] = state[98];
acadoWorkspace.evGu[lRun1 * 27 + 9] = state[99];
acadoWorkspace.evGu[lRun1 * 27 + 10] = state[100];
acadoWorkspace.evGu[lRun1 * 27 + 11] = state[101];
acadoWorkspace.evGu[lRun1 * 27 + 12] = state[102];
acadoWorkspace.evGu[lRun1 * 27 + 13] = state[103];
acadoWorkspace.evGu[lRun1 * 27 + 14] = state[104];
acadoWorkspace.evGu[lRun1 * 27 + 15] = state[105];
acadoWorkspace.evGu[lRun1 * 27 + 16] = state[106];
acadoWorkspace.evGu[lRun1 * 27 + 17] = state[107];
acadoWorkspace.evGu[lRun1 * 27 + 18] = state[108];
acadoWorkspace.evGu[lRun1 * 27 + 19] = state[109];
acadoWorkspace.evGu[lRun1 * 27 + 20] = state[110];
acadoWorkspace.evGu[lRun1 * 27 + 21] = state[111];
acadoWorkspace.evGu[lRun1 * 27 + 22] = state[112];
acadoWorkspace.evGu[lRun1 * 27 + 23] = state[113];
acadoWorkspace.evGu[lRun1 * 27 + 24] = state[114];
acadoWorkspace.evGu[lRun1 * 27 + 25] = state[115];
acadoWorkspace.evGu[lRun1 * 27 + 26] = state[116];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;
const real_t* od = in + 12;
/* Vector of auxiliary variables; number of elements: 63. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[4]));
a[1] = (cos(xd[3]));
a[2] = (cos(xd[3]));
a[3] = (sin(xd[5]));
a[4] = (cos(xd[5]));
a[5] = (sin(xd[4]));
a[6] = (sin(xd[3]));
a[7] = (cos(xd[3]));
a[8] = (cos(xd[5]));
a[9] = (sin(xd[4]));
a[10] = (sin(xd[3]));
a[11] = (sin(xd[5]));
a[12] = (cos(xd[4]));
a[13] = (sin(xd[3]));
a[14] = (pow((((((((real_t)(0.0000000000000000e+00)-a[2])*a[3])+((a[4]*a[5])*a[6]))*(xd[6]-od[9]))+(((a[7]*a[8])+((a[9]*a[10])*a[11]))*(xd[7]-od[10])))+(((xd[8]-od[11])*a[12])*a[13])),2));
a[15] = (sin(xd[4]));
a[16] = (cos(xd[4]));
a[17] = (cos(xd[5]));
a[18] = (cos(xd[4]));
a[19] = (sin(xd[5]));
a[20] = (pow((((((real_t)(0.0000000000000000e+00)-a[15])*(xd[8]-od[11]))+(((xd[6]-od[9])*a[16])*a[17]))+(((xd[7]-od[10])*a[18])*a[19])),2));
a[21] = (pow((real_t)(2.7180000000000000e+00),(((a[14]+a[20])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))));
a[22] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[23] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[24] = ((real_t)(2.0000000000000000e+00)*(((((((real_t)(0.0000000000000000e+00)-a[2])*a[3])+((a[4]*a[5])*a[6]))*(xd[6]-od[9]))+(((a[7]*a[8])+((a[9]*a[10])*a[11]))*(xd[7]-od[10])))+(((xd[8]-od[11])*a[12])*a[13])));
a[25] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[26] = (cos(xd[3]));
a[27] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[28] = (cos(xd[3]));
a[29] = (cos(xd[3]));
a[30] = (a[24]*(((((((real_t)(0.0000000000000000e+00)-a[25])*a[3])+((a[4]*a[5])*a[26]))*(xd[6]-od[9]))+(((a[27]*a[8])+((a[9]*a[28])*a[11]))*(xd[7]-od[10])))+(((xd[8]-od[11])*a[12])*a[29])));
a[31] = (log((real_t)(2.7180000000000000e+00)));
a[32] = ((a[30]*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[14]+a[20])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[31]));
a[33] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+a[21]));
a[34] = (a[33]*a[33]);
a[35] = (cos(xd[4]));
a[36] = (cos(xd[4]));
a[37] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[38] = (a[24]*(((((a[4]*a[35])*a[6])*(xd[6]-od[9]))+(((a[36]*a[10])*a[11])*(xd[7]-od[10])))+(((xd[8]-od[11])*a[37])*a[13])));
a[39] = ((real_t)(2.0000000000000000e+00)*(((((real_t)(0.0000000000000000e+00)-a[15])*(xd[8]-od[11]))+(((xd[6]-od[9])*a[16])*a[17]))+(((xd[7]-od[10])*a[18])*a[19])));
a[40] = (cos(xd[4]));
a[41] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[42] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[43] = (a[39]*(((((real_t)(0.0000000000000000e+00)-a[40])*(xd[8]-od[11]))+(((xd[6]-od[9])*a[41])*a[17]))+(((xd[7]-od[10])*a[42])*a[19])));
a[44] = (((a[38]+a[43])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[14]+a[20])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[31]));
a[45] = (cos(xd[5]));
a[46] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[47] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[48] = (cos(xd[5]));
a[49] = (a[24]*((((((real_t)(0.0000000000000000e+00)-a[2])*a[45])+((a[46]*a[5])*a[6]))*(xd[6]-od[9]))+(((a[7]*a[47])+((a[9]*a[10])*a[48]))*(xd[7]-od[10]))));
a[50] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[51] = (cos(xd[5]));
a[52] = (a[39]*((((xd[6]-od[9])*a[16])*a[50])+(((xd[7]-od[10])*a[18])*a[51])));
a[53] = (((a[49]+a[52])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[14]+a[20])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[31]));
a[54] = (a[24]*((((real_t)(0.0000000000000000e+00)-a[2])*a[3])+((a[4]*a[5])*a[6])));
a[55] = (a[39]*(a[16]*a[17]));
a[56] = (((a[54]+a[55])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[14]+a[20])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[31]));
a[57] = (a[24]*((a[7]*a[8])+((a[9]*a[10])*a[11])));
a[58] = (a[39]*(a[18]*a[19]));
a[59] = (((a[57]+a[58])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[14]+a[20])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[31]));
a[60] = (a[24]*(a[12]*a[13]));
a[61] = (a[39]*((real_t)(0.0000000000000000e+00)-a[15]));
a[62] = (((a[60]+a[61])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[14]+a[20])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[31]));

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[0];
out[4] = xd[1];
out[5] = xd[2];
out[6] = xd[3];
out[7] = xd[4];
out[8] = u[0];
out[9] = u[1];
out[10] = ((real_t)(-9.8065999999999995e+00)+((a[0]*a[1])*u[2]));
out[11] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+a[21]));
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(1.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(1.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(1.0000000000000000e+00);
out[39] = (real_t)(1.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(1.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(1.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(1.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(1.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = ((a[0]*a[22])*u[2]);
out[106] = ((a[23]*a[1])*u[2]);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = ((real_t)(0.0000000000000000e+00)-(a[32]*a[34]));
out[115] = ((real_t)(0.0000000000000000e+00)-(a[44]*a[34]));
out[116] = ((real_t)(0.0000000000000000e+00)-(a[53]*a[34]));
out[117] = ((real_t)(0.0000000000000000e+00)-(a[56]*a[34]));
out[118] = ((real_t)(0.0000000000000000e+00)-(a[59]*a[34]));
out[119] = ((real_t)(0.0000000000000000e+00)-(a[62]*a[34]));
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(1.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(1.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (a[0]*a[1]);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 9;
/* Vector of auxiliary variables; number of elements: 59. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[3]));
a[1] = (sin(xd[5]));
a[2] = (cos(xd[5]));
a[3] = (sin(xd[4]));
a[4] = (sin(xd[3]));
a[5] = (cos(xd[3]));
a[6] = (cos(xd[5]));
a[7] = (sin(xd[4]));
a[8] = (sin(xd[3]));
a[9] = (sin(xd[5]));
a[10] = (cos(xd[4]));
a[11] = (sin(xd[3]));
a[12] = (pow((((((((real_t)(0.0000000000000000e+00)-a[0])*a[1])+((a[2]*a[3])*a[4]))*(xd[6]-od[9]))+(((a[5]*a[6])+((a[7]*a[8])*a[9]))*(xd[7]-od[10])))+(((xd[8]-od[11])*a[10])*a[11])),2));
a[13] = (sin(xd[4]));
a[14] = (cos(xd[4]));
a[15] = (cos(xd[5]));
a[16] = (cos(xd[4]));
a[17] = (sin(xd[5]));
a[18] = (pow((((((real_t)(0.0000000000000000e+00)-a[13])*(xd[8]-od[11]))+(((xd[6]-od[9])*a[14])*a[15]))+(((xd[7]-od[10])*a[16])*a[17])),2));
a[19] = (pow((real_t)(2.7180000000000000e+00),(((a[12]+a[18])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))));
a[20] = ((real_t)(2.0000000000000000e+00)*(((((((real_t)(0.0000000000000000e+00)-a[0])*a[1])+((a[2]*a[3])*a[4]))*(xd[6]-od[9]))+(((a[5]*a[6])+((a[7]*a[8])*a[9]))*(xd[7]-od[10])))+(((xd[8]-od[11])*a[10])*a[11])));
a[21] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[22] = (cos(xd[3]));
a[23] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[3])));
a[24] = (cos(xd[3]));
a[25] = (cos(xd[3]));
a[26] = (a[20]*(((((((real_t)(0.0000000000000000e+00)-a[21])*a[1])+((a[2]*a[3])*a[22]))*(xd[6]-od[9]))+(((a[23]*a[6])+((a[7]*a[24])*a[9]))*(xd[7]-od[10])))+(((xd[8]-od[11])*a[10])*a[25])));
a[27] = (log((real_t)(2.7180000000000000e+00)));
a[28] = ((a[26]*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[12]+a[18])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[27]));
a[29] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+a[19]));
a[30] = (a[29]*a[29]);
a[31] = (cos(xd[4]));
a[32] = (cos(xd[4]));
a[33] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[34] = (a[20]*(((((a[2]*a[31])*a[4])*(xd[6]-od[9]))+(((a[32]*a[8])*a[9])*(xd[7]-od[10])))+(((xd[8]-od[11])*a[33])*a[11])));
a[35] = ((real_t)(2.0000000000000000e+00)*(((((real_t)(0.0000000000000000e+00)-a[13])*(xd[8]-od[11]))+(((xd[6]-od[9])*a[14])*a[15]))+(((xd[7]-od[10])*a[16])*a[17])));
a[36] = (cos(xd[4]));
a[37] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[38] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[4])));
a[39] = (a[35]*(((((real_t)(0.0000000000000000e+00)-a[36])*(xd[8]-od[11]))+(((xd[6]-od[9])*a[37])*a[15]))+(((xd[7]-od[10])*a[38])*a[17])));
a[40] = (((a[34]+a[39])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[12]+a[18])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[27]));
a[41] = (cos(xd[5]));
a[42] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[43] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[44] = (cos(xd[5]));
a[45] = (a[20]*((((((real_t)(0.0000000000000000e+00)-a[0])*a[41])+((a[42]*a[3])*a[4]))*(xd[6]-od[9]))+(((a[5]*a[43])+((a[7]*a[8])*a[44]))*(xd[7]-od[10]))));
a[46] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[5])));
a[47] = (cos(xd[5]));
a[48] = (a[35]*((((xd[6]-od[9])*a[14])*a[46])+(((xd[7]-od[10])*a[16])*a[47])));
a[49] = (((a[45]+a[48])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[12]+a[18])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[27]));
a[50] = (a[20]*((((real_t)(0.0000000000000000e+00)-a[0])*a[1])+((a[2]*a[3])*a[4])));
a[51] = (a[35]*(a[14]*a[15]));
a[52] = (((a[50]+a[51])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[12]+a[18])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[27]));
a[53] = (a[20]*((a[5]*a[6])+((a[7]*a[8])*a[9])));
a[54] = (a[35]*(a[16]*a[17]));
a[55] = (((a[53]+a[54])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[12]+a[18])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[27]));
a[56] = (a[20]*(a[10]*a[11]));
a[57] = (a[35]*((real_t)(0.0000000000000000e+00)-a[13]));
a[58] = (((a[56]+a[57])*(real_t)(1.1100000000000000e+02))*((pow((real_t)(2.7180000000000000e+00),(((a[12]+a[18])*(real_t)(1.1100000000000000e+02))-(real_t)(5.0999999999999996e+00))))*a[27]));

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[0];
out[4] = xd[1];
out[5] = xd[2];
out[6] = ((real_t)(1.0000000000000000e+00)/((real_t)(1.0000000000000000e+00)+a[19]));
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(1.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(1.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(1.0000000000000000e+00);
out[34] = (real_t)(1.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(1.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(1.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = ((real_t)(0.0000000000000000e+00)-(a[28]*a[30]));
out[65] = ((real_t)(0.0000000000000000e+00)-(a[40]*a[30]));
out[66] = ((real_t)(0.0000000000000000e+00)-(a[49]*a[30]));
out[67] = ((real_t)(0.0000000000000000e+00)-(a[52]*a[30]));
out[68] = ((real_t)(0.0000000000000000e+00)-(a[55]*a[30]));
out[69] = ((real_t)(0.0000000000000000e+00)-(a[58]*a[30]));
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[9]*tmpObjS[12] + tmpFx[18]*tmpObjS[24] + tmpFx[27]*tmpObjS[36] + tmpFx[36]*tmpObjS[48] + tmpFx[45]*tmpObjS[60] + tmpFx[54]*tmpObjS[72] + tmpFx[63]*tmpObjS[84] + tmpFx[72]*tmpObjS[96] + tmpFx[81]*tmpObjS[108] + tmpFx[90]*tmpObjS[120] + tmpFx[99]*tmpObjS[132];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[9]*tmpObjS[13] + tmpFx[18]*tmpObjS[25] + tmpFx[27]*tmpObjS[37] + tmpFx[36]*tmpObjS[49] + tmpFx[45]*tmpObjS[61] + tmpFx[54]*tmpObjS[73] + tmpFx[63]*tmpObjS[85] + tmpFx[72]*tmpObjS[97] + tmpFx[81]*tmpObjS[109] + tmpFx[90]*tmpObjS[121] + tmpFx[99]*tmpObjS[133];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[9]*tmpObjS[14] + tmpFx[18]*tmpObjS[26] + tmpFx[27]*tmpObjS[38] + tmpFx[36]*tmpObjS[50] + tmpFx[45]*tmpObjS[62] + tmpFx[54]*tmpObjS[74] + tmpFx[63]*tmpObjS[86] + tmpFx[72]*tmpObjS[98] + tmpFx[81]*tmpObjS[110] + tmpFx[90]*tmpObjS[122] + tmpFx[99]*tmpObjS[134];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[9]*tmpObjS[15] + tmpFx[18]*tmpObjS[27] + tmpFx[27]*tmpObjS[39] + tmpFx[36]*tmpObjS[51] + tmpFx[45]*tmpObjS[63] + tmpFx[54]*tmpObjS[75] + tmpFx[63]*tmpObjS[87] + tmpFx[72]*tmpObjS[99] + tmpFx[81]*tmpObjS[111] + tmpFx[90]*tmpObjS[123] + tmpFx[99]*tmpObjS[135];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[9]*tmpObjS[16] + tmpFx[18]*tmpObjS[28] + tmpFx[27]*tmpObjS[40] + tmpFx[36]*tmpObjS[52] + tmpFx[45]*tmpObjS[64] + tmpFx[54]*tmpObjS[76] + tmpFx[63]*tmpObjS[88] + tmpFx[72]*tmpObjS[100] + tmpFx[81]*tmpObjS[112] + tmpFx[90]*tmpObjS[124] + tmpFx[99]*tmpObjS[136];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[9]*tmpObjS[17] + tmpFx[18]*tmpObjS[29] + tmpFx[27]*tmpObjS[41] + tmpFx[36]*tmpObjS[53] + tmpFx[45]*tmpObjS[65] + tmpFx[54]*tmpObjS[77] + tmpFx[63]*tmpObjS[89] + tmpFx[72]*tmpObjS[101] + tmpFx[81]*tmpObjS[113] + tmpFx[90]*tmpObjS[125] + tmpFx[99]*tmpObjS[137];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[9]*tmpObjS[18] + tmpFx[18]*tmpObjS[30] + tmpFx[27]*tmpObjS[42] + tmpFx[36]*tmpObjS[54] + tmpFx[45]*tmpObjS[66] + tmpFx[54]*tmpObjS[78] + tmpFx[63]*tmpObjS[90] + tmpFx[72]*tmpObjS[102] + tmpFx[81]*tmpObjS[114] + tmpFx[90]*tmpObjS[126] + tmpFx[99]*tmpObjS[138];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[9]*tmpObjS[19] + tmpFx[18]*tmpObjS[31] + tmpFx[27]*tmpObjS[43] + tmpFx[36]*tmpObjS[55] + tmpFx[45]*tmpObjS[67] + tmpFx[54]*tmpObjS[79] + tmpFx[63]*tmpObjS[91] + tmpFx[72]*tmpObjS[103] + tmpFx[81]*tmpObjS[115] + tmpFx[90]*tmpObjS[127] + tmpFx[99]*tmpObjS[139];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[9]*tmpObjS[20] + tmpFx[18]*tmpObjS[32] + tmpFx[27]*tmpObjS[44] + tmpFx[36]*tmpObjS[56] + tmpFx[45]*tmpObjS[68] + tmpFx[54]*tmpObjS[80] + tmpFx[63]*tmpObjS[92] + tmpFx[72]*tmpObjS[104] + tmpFx[81]*tmpObjS[116] + tmpFx[90]*tmpObjS[128] + tmpFx[99]*tmpObjS[140];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[9]*tmpObjS[21] + tmpFx[18]*tmpObjS[33] + tmpFx[27]*tmpObjS[45] + tmpFx[36]*tmpObjS[57] + tmpFx[45]*tmpObjS[69] + tmpFx[54]*tmpObjS[81] + tmpFx[63]*tmpObjS[93] + tmpFx[72]*tmpObjS[105] + tmpFx[81]*tmpObjS[117] + tmpFx[90]*tmpObjS[129] + tmpFx[99]*tmpObjS[141];
tmpQ2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[9]*tmpObjS[22] + tmpFx[18]*tmpObjS[34] + tmpFx[27]*tmpObjS[46] + tmpFx[36]*tmpObjS[58] + tmpFx[45]*tmpObjS[70] + tmpFx[54]*tmpObjS[82] + tmpFx[63]*tmpObjS[94] + tmpFx[72]*tmpObjS[106] + tmpFx[81]*tmpObjS[118] + tmpFx[90]*tmpObjS[130] + tmpFx[99]*tmpObjS[142];
tmpQ2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[9]*tmpObjS[23] + tmpFx[18]*tmpObjS[35] + tmpFx[27]*tmpObjS[47] + tmpFx[36]*tmpObjS[59] + tmpFx[45]*tmpObjS[71] + tmpFx[54]*tmpObjS[83] + tmpFx[63]*tmpObjS[95] + tmpFx[72]*tmpObjS[107] + tmpFx[81]*tmpObjS[119] + tmpFx[90]*tmpObjS[131] + tmpFx[99]*tmpObjS[143];
tmpQ2[12] = + tmpFx[1]*tmpObjS[0] + tmpFx[10]*tmpObjS[12] + tmpFx[19]*tmpObjS[24] + tmpFx[28]*tmpObjS[36] + tmpFx[37]*tmpObjS[48] + tmpFx[46]*tmpObjS[60] + tmpFx[55]*tmpObjS[72] + tmpFx[64]*tmpObjS[84] + tmpFx[73]*tmpObjS[96] + tmpFx[82]*tmpObjS[108] + tmpFx[91]*tmpObjS[120] + tmpFx[100]*tmpObjS[132];
tmpQ2[13] = + tmpFx[1]*tmpObjS[1] + tmpFx[10]*tmpObjS[13] + tmpFx[19]*tmpObjS[25] + tmpFx[28]*tmpObjS[37] + tmpFx[37]*tmpObjS[49] + tmpFx[46]*tmpObjS[61] + tmpFx[55]*tmpObjS[73] + tmpFx[64]*tmpObjS[85] + tmpFx[73]*tmpObjS[97] + tmpFx[82]*tmpObjS[109] + tmpFx[91]*tmpObjS[121] + tmpFx[100]*tmpObjS[133];
tmpQ2[14] = + tmpFx[1]*tmpObjS[2] + tmpFx[10]*tmpObjS[14] + tmpFx[19]*tmpObjS[26] + tmpFx[28]*tmpObjS[38] + tmpFx[37]*tmpObjS[50] + tmpFx[46]*tmpObjS[62] + tmpFx[55]*tmpObjS[74] + tmpFx[64]*tmpObjS[86] + tmpFx[73]*tmpObjS[98] + tmpFx[82]*tmpObjS[110] + tmpFx[91]*tmpObjS[122] + tmpFx[100]*tmpObjS[134];
tmpQ2[15] = + tmpFx[1]*tmpObjS[3] + tmpFx[10]*tmpObjS[15] + tmpFx[19]*tmpObjS[27] + tmpFx[28]*tmpObjS[39] + tmpFx[37]*tmpObjS[51] + tmpFx[46]*tmpObjS[63] + tmpFx[55]*tmpObjS[75] + tmpFx[64]*tmpObjS[87] + tmpFx[73]*tmpObjS[99] + tmpFx[82]*tmpObjS[111] + tmpFx[91]*tmpObjS[123] + tmpFx[100]*tmpObjS[135];
tmpQ2[16] = + tmpFx[1]*tmpObjS[4] + tmpFx[10]*tmpObjS[16] + tmpFx[19]*tmpObjS[28] + tmpFx[28]*tmpObjS[40] + tmpFx[37]*tmpObjS[52] + tmpFx[46]*tmpObjS[64] + tmpFx[55]*tmpObjS[76] + tmpFx[64]*tmpObjS[88] + tmpFx[73]*tmpObjS[100] + tmpFx[82]*tmpObjS[112] + tmpFx[91]*tmpObjS[124] + tmpFx[100]*tmpObjS[136];
tmpQ2[17] = + tmpFx[1]*tmpObjS[5] + tmpFx[10]*tmpObjS[17] + tmpFx[19]*tmpObjS[29] + tmpFx[28]*tmpObjS[41] + tmpFx[37]*tmpObjS[53] + tmpFx[46]*tmpObjS[65] + tmpFx[55]*tmpObjS[77] + tmpFx[64]*tmpObjS[89] + tmpFx[73]*tmpObjS[101] + tmpFx[82]*tmpObjS[113] + tmpFx[91]*tmpObjS[125] + tmpFx[100]*tmpObjS[137];
tmpQ2[18] = + tmpFx[1]*tmpObjS[6] + tmpFx[10]*tmpObjS[18] + tmpFx[19]*tmpObjS[30] + tmpFx[28]*tmpObjS[42] + tmpFx[37]*tmpObjS[54] + tmpFx[46]*tmpObjS[66] + tmpFx[55]*tmpObjS[78] + tmpFx[64]*tmpObjS[90] + tmpFx[73]*tmpObjS[102] + tmpFx[82]*tmpObjS[114] + tmpFx[91]*tmpObjS[126] + tmpFx[100]*tmpObjS[138];
tmpQ2[19] = + tmpFx[1]*tmpObjS[7] + tmpFx[10]*tmpObjS[19] + tmpFx[19]*tmpObjS[31] + tmpFx[28]*tmpObjS[43] + tmpFx[37]*tmpObjS[55] + tmpFx[46]*tmpObjS[67] + tmpFx[55]*tmpObjS[79] + tmpFx[64]*tmpObjS[91] + tmpFx[73]*tmpObjS[103] + tmpFx[82]*tmpObjS[115] + tmpFx[91]*tmpObjS[127] + tmpFx[100]*tmpObjS[139];
tmpQ2[20] = + tmpFx[1]*tmpObjS[8] + tmpFx[10]*tmpObjS[20] + tmpFx[19]*tmpObjS[32] + tmpFx[28]*tmpObjS[44] + tmpFx[37]*tmpObjS[56] + tmpFx[46]*tmpObjS[68] + tmpFx[55]*tmpObjS[80] + tmpFx[64]*tmpObjS[92] + tmpFx[73]*tmpObjS[104] + tmpFx[82]*tmpObjS[116] + tmpFx[91]*tmpObjS[128] + tmpFx[100]*tmpObjS[140];
tmpQ2[21] = + tmpFx[1]*tmpObjS[9] + tmpFx[10]*tmpObjS[21] + tmpFx[19]*tmpObjS[33] + tmpFx[28]*tmpObjS[45] + tmpFx[37]*tmpObjS[57] + tmpFx[46]*tmpObjS[69] + tmpFx[55]*tmpObjS[81] + tmpFx[64]*tmpObjS[93] + tmpFx[73]*tmpObjS[105] + tmpFx[82]*tmpObjS[117] + tmpFx[91]*tmpObjS[129] + tmpFx[100]*tmpObjS[141];
tmpQ2[22] = + tmpFx[1]*tmpObjS[10] + tmpFx[10]*tmpObjS[22] + tmpFx[19]*tmpObjS[34] + tmpFx[28]*tmpObjS[46] + tmpFx[37]*tmpObjS[58] + tmpFx[46]*tmpObjS[70] + tmpFx[55]*tmpObjS[82] + tmpFx[64]*tmpObjS[94] + tmpFx[73]*tmpObjS[106] + tmpFx[82]*tmpObjS[118] + tmpFx[91]*tmpObjS[130] + tmpFx[100]*tmpObjS[142];
tmpQ2[23] = + tmpFx[1]*tmpObjS[11] + tmpFx[10]*tmpObjS[23] + tmpFx[19]*tmpObjS[35] + tmpFx[28]*tmpObjS[47] + tmpFx[37]*tmpObjS[59] + tmpFx[46]*tmpObjS[71] + tmpFx[55]*tmpObjS[83] + tmpFx[64]*tmpObjS[95] + tmpFx[73]*tmpObjS[107] + tmpFx[82]*tmpObjS[119] + tmpFx[91]*tmpObjS[131] + tmpFx[100]*tmpObjS[143];
tmpQ2[24] = + tmpFx[2]*tmpObjS[0] + tmpFx[11]*tmpObjS[12] + tmpFx[20]*tmpObjS[24] + tmpFx[29]*tmpObjS[36] + tmpFx[38]*tmpObjS[48] + tmpFx[47]*tmpObjS[60] + tmpFx[56]*tmpObjS[72] + tmpFx[65]*tmpObjS[84] + tmpFx[74]*tmpObjS[96] + tmpFx[83]*tmpObjS[108] + tmpFx[92]*tmpObjS[120] + tmpFx[101]*tmpObjS[132];
tmpQ2[25] = + tmpFx[2]*tmpObjS[1] + tmpFx[11]*tmpObjS[13] + tmpFx[20]*tmpObjS[25] + tmpFx[29]*tmpObjS[37] + tmpFx[38]*tmpObjS[49] + tmpFx[47]*tmpObjS[61] + tmpFx[56]*tmpObjS[73] + tmpFx[65]*tmpObjS[85] + tmpFx[74]*tmpObjS[97] + tmpFx[83]*tmpObjS[109] + tmpFx[92]*tmpObjS[121] + tmpFx[101]*tmpObjS[133];
tmpQ2[26] = + tmpFx[2]*tmpObjS[2] + tmpFx[11]*tmpObjS[14] + tmpFx[20]*tmpObjS[26] + tmpFx[29]*tmpObjS[38] + tmpFx[38]*tmpObjS[50] + tmpFx[47]*tmpObjS[62] + tmpFx[56]*tmpObjS[74] + tmpFx[65]*tmpObjS[86] + tmpFx[74]*tmpObjS[98] + tmpFx[83]*tmpObjS[110] + tmpFx[92]*tmpObjS[122] + tmpFx[101]*tmpObjS[134];
tmpQ2[27] = + tmpFx[2]*tmpObjS[3] + tmpFx[11]*tmpObjS[15] + tmpFx[20]*tmpObjS[27] + tmpFx[29]*tmpObjS[39] + tmpFx[38]*tmpObjS[51] + tmpFx[47]*tmpObjS[63] + tmpFx[56]*tmpObjS[75] + tmpFx[65]*tmpObjS[87] + tmpFx[74]*tmpObjS[99] + tmpFx[83]*tmpObjS[111] + tmpFx[92]*tmpObjS[123] + tmpFx[101]*tmpObjS[135];
tmpQ2[28] = + tmpFx[2]*tmpObjS[4] + tmpFx[11]*tmpObjS[16] + tmpFx[20]*tmpObjS[28] + tmpFx[29]*tmpObjS[40] + tmpFx[38]*tmpObjS[52] + tmpFx[47]*tmpObjS[64] + tmpFx[56]*tmpObjS[76] + tmpFx[65]*tmpObjS[88] + tmpFx[74]*tmpObjS[100] + tmpFx[83]*tmpObjS[112] + tmpFx[92]*tmpObjS[124] + tmpFx[101]*tmpObjS[136];
tmpQ2[29] = + tmpFx[2]*tmpObjS[5] + tmpFx[11]*tmpObjS[17] + tmpFx[20]*tmpObjS[29] + tmpFx[29]*tmpObjS[41] + tmpFx[38]*tmpObjS[53] + tmpFx[47]*tmpObjS[65] + tmpFx[56]*tmpObjS[77] + tmpFx[65]*tmpObjS[89] + tmpFx[74]*tmpObjS[101] + tmpFx[83]*tmpObjS[113] + tmpFx[92]*tmpObjS[125] + tmpFx[101]*tmpObjS[137];
tmpQ2[30] = + tmpFx[2]*tmpObjS[6] + tmpFx[11]*tmpObjS[18] + tmpFx[20]*tmpObjS[30] + tmpFx[29]*tmpObjS[42] + tmpFx[38]*tmpObjS[54] + tmpFx[47]*tmpObjS[66] + tmpFx[56]*tmpObjS[78] + tmpFx[65]*tmpObjS[90] + tmpFx[74]*tmpObjS[102] + tmpFx[83]*tmpObjS[114] + tmpFx[92]*tmpObjS[126] + tmpFx[101]*tmpObjS[138];
tmpQ2[31] = + tmpFx[2]*tmpObjS[7] + tmpFx[11]*tmpObjS[19] + tmpFx[20]*tmpObjS[31] + tmpFx[29]*tmpObjS[43] + tmpFx[38]*tmpObjS[55] + tmpFx[47]*tmpObjS[67] + tmpFx[56]*tmpObjS[79] + tmpFx[65]*tmpObjS[91] + tmpFx[74]*tmpObjS[103] + tmpFx[83]*tmpObjS[115] + tmpFx[92]*tmpObjS[127] + tmpFx[101]*tmpObjS[139];
tmpQ2[32] = + tmpFx[2]*tmpObjS[8] + tmpFx[11]*tmpObjS[20] + tmpFx[20]*tmpObjS[32] + tmpFx[29]*tmpObjS[44] + tmpFx[38]*tmpObjS[56] + tmpFx[47]*tmpObjS[68] + tmpFx[56]*tmpObjS[80] + tmpFx[65]*tmpObjS[92] + tmpFx[74]*tmpObjS[104] + tmpFx[83]*tmpObjS[116] + tmpFx[92]*tmpObjS[128] + tmpFx[101]*tmpObjS[140];
tmpQ2[33] = + tmpFx[2]*tmpObjS[9] + tmpFx[11]*tmpObjS[21] + tmpFx[20]*tmpObjS[33] + tmpFx[29]*tmpObjS[45] + tmpFx[38]*tmpObjS[57] + tmpFx[47]*tmpObjS[69] + tmpFx[56]*tmpObjS[81] + tmpFx[65]*tmpObjS[93] + tmpFx[74]*tmpObjS[105] + tmpFx[83]*tmpObjS[117] + tmpFx[92]*tmpObjS[129] + tmpFx[101]*tmpObjS[141];
tmpQ2[34] = + tmpFx[2]*tmpObjS[10] + tmpFx[11]*tmpObjS[22] + tmpFx[20]*tmpObjS[34] + tmpFx[29]*tmpObjS[46] + tmpFx[38]*tmpObjS[58] + tmpFx[47]*tmpObjS[70] + tmpFx[56]*tmpObjS[82] + tmpFx[65]*tmpObjS[94] + tmpFx[74]*tmpObjS[106] + tmpFx[83]*tmpObjS[118] + tmpFx[92]*tmpObjS[130] + tmpFx[101]*tmpObjS[142];
tmpQ2[35] = + tmpFx[2]*tmpObjS[11] + tmpFx[11]*tmpObjS[23] + tmpFx[20]*tmpObjS[35] + tmpFx[29]*tmpObjS[47] + tmpFx[38]*tmpObjS[59] + tmpFx[47]*tmpObjS[71] + tmpFx[56]*tmpObjS[83] + tmpFx[65]*tmpObjS[95] + tmpFx[74]*tmpObjS[107] + tmpFx[83]*tmpObjS[119] + tmpFx[92]*tmpObjS[131] + tmpFx[101]*tmpObjS[143];
tmpQ2[36] = + tmpFx[3]*tmpObjS[0] + tmpFx[12]*tmpObjS[12] + tmpFx[21]*tmpObjS[24] + tmpFx[30]*tmpObjS[36] + tmpFx[39]*tmpObjS[48] + tmpFx[48]*tmpObjS[60] + tmpFx[57]*tmpObjS[72] + tmpFx[66]*tmpObjS[84] + tmpFx[75]*tmpObjS[96] + tmpFx[84]*tmpObjS[108] + tmpFx[93]*tmpObjS[120] + tmpFx[102]*tmpObjS[132];
tmpQ2[37] = + tmpFx[3]*tmpObjS[1] + tmpFx[12]*tmpObjS[13] + tmpFx[21]*tmpObjS[25] + tmpFx[30]*tmpObjS[37] + tmpFx[39]*tmpObjS[49] + tmpFx[48]*tmpObjS[61] + tmpFx[57]*tmpObjS[73] + tmpFx[66]*tmpObjS[85] + tmpFx[75]*tmpObjS[97] + tmpFx[84]*tmpObjS[109] + tmpFx[93]*tmpObjS[121] + tmpFx[102]*tmpObjS[133];
tmpQ2[38] = + tmpFx[3]*tmpObjS[2] + tmpFx[12]*tmpObjS[14] + tmpFx[21]*tmpObjS[26] + tmpFx[30]*tmpObjS[38] + tmpFx[39]*tmpObjS[50] + tmpFx[48]*tmpObjS[62] + tmpFx[57]*tmpObjS[74] + tmpFx[66]*tmpObjS[86] + tmpFx[75]*tmpObjS[98] + tmpFx[84]*tmpObjS[110] + tmpFx[93]*tmpObjS[122] + tmpFx[102]*tmpObjS[134];
tmpQ2[39] = + tmpFx[3]*tmpObjS[3] + tmpFx[12]*tmpObjS[15] + tmpFx[21]*tmpObjS[27] + tmpFx[30]*tmpObjS[39] + tmpFx[39]*tmpObjS[51] + tmpFx[48]*tmpObjS[63] + tmpFx[57]*tmpObjS[75] + tmpFx[66]*tmpObjS[87] + tmpFx[75]*tmpObjS[99] + tmpFx[84]*tmpObjS[111] + tmpFx[93]*tmpObjS[123] + tmpFx[102]*tmpObjS[135];
tmpQ2[40] = + tmpFx[3]*tmpObjS[4] + tmpFx[12]*tmpObjS[16] + tmpFx[21]*tmpObjS[28] + tmpFx[30]*tmpObjS[40] + tmpFx[39]*tmpObjS[52] + tmpFx[48]*tmpObjS[64] + tmpFx[57]*tmpObjS[76] + tmpFx[66]*tmpObjS[88] + tmpFx[75]*tmpObjS[100] + tmpFx[84]*tmpObjS[112] + tmpFx[93]*tmpObjS[124] + tmpFx[102]*tmpObjS[136];
tmpQ2[41] = + tmpFx[3]*tmpObjS[5] + tmpFx[12]*tmpObjS[17] + tmpFx[21]*tmpObjS[29] + tmpFx[30]*tmpObjS[41] + tmpFx[39]*tmpObjS[53] + tmpFx[48]*tmpObjS[65] + tmpFx[57]*tmpObjS[77] + tmpFx[66]*tmpObjS[89] + tmpFx[75]*tmpObjS[101] + tmpFx[84]*tmpObjS[113] + tmpFx[93]*tmpObjS[125] + tmpFx[102]*tmpObjS[137];
tmpQ2[42] = + tmpFx[3]*tmpObjS[6] + tmpFx[12]*tmpObjS[18] + tmpFx[21]*tmpObjS[30] + tmpFx[30]*tmpObjS[42] + tmpFx[39]*tmpObjS[54] + tmpFx[48]*tmpObjS[66] + tmpFx[57]*tmpObjS[78] + tmpFx[66]*tmpObjS[90] + tmpFx[75]*tmpObjS[102] + tmpFx[84]*tmpObjS[114] + tmpFx[93]*tmpObjS[126] + tmpFx[102]*tmpObjS[138];
tmpQ2[43] = + tmpFx[3]*tmpObjS[7] + tmpFx[12]*tmpObjS[19] + tmpFx[21]*tmpObjS[31] + tmpFx[30]*tmpObjS[43] + tmpFx[39]*tmpObjS[55] + tmpFx[48]*tmpObjS[67] + tmpFx[57]*tmpObjS[79] + tmpFx[66]*tmpObjS[91] + tmpFx[75]*tmpObjS[103] + tmpFx[84]*tmpObjS[115] + tmpFx[93]*tmpObjS[127] + tmpFx[102]*tmpObjS[139];
tmpQ2[44] = + tmpFx[3]*tmpObjS[8] + tmpFx[12]*tmpObjS[20] + tmpFx[21]*tmpObjS[32] + tmpFx[30]*tmpObjS[44] + tmpFx[39]*tmpObjS[56] + tmpFx[48]*tmpObjS[68] + tmpFx[57]*tmpObjS[80] + tmpFx[66]*tmpObjS[92] + tmpFx[75]*tmpObjS[104] + tmpFx[84]*tmpObjS[116] + tmpFx[93]*tmpObjS[128] + tmpFx[102]*tmpObjS[140];
tmpQ2[45] = + tmpFx[3]*tmpObjS[9] + tmpFx[12]*tmpObjS[21] + tmpFx[21]*tmpObjS[33] + tmpFx[30]*tmpObjS[45] + tmpFx[39]*tmpObjS[57] + tmpFx[48]*tmpObjS[69] + tmpFx[57]*tmpObjS[81] + tmpFx[66]*tmpObjS[93] + tmpFx[75]*tmpObjS[105] + tmpFx[84]*tmpObjS[117] + tmpFx[93]*tmpObjS[129] + tmpFx[102]*tmpObjS[141];
tmpQ2[46] = + tmpFx[3]*tmpObjS[10] + tmpFx[12]*tmpObjS[22] + tmpFx[21]*tmpObjS[34] + tmpFx[30]*tmpObjS[46] + tmpFx[39]*tmpObjS[58] + tmpFx[48]*tmpObjS[70] + tmpFx[57]*tmpObjS[82] + tmpFx[66]*tmpObjS[94] + tmpFx[75]*tmpObjS[106] + tmpFx[84]*tmpObjS[118] + tmpFx[93]*tmpObjS[130] + tmpFx[102]*tmpObjS[142];
tmpQ2[47] = + tmpFx[3]*tmpObjS[11] + tmpFx[12]*tmpObjS[23] + tmpFx[21]*tmpObjS[35] + tmpFx[30]*tmpObjS[47] + tmpFx[39]*tmpObjS[59] + tmpFx[48]*tmpObjS[71] + tmpFx[57]*tmpObjS[83] + tmpFx[66]*tmpObjS[95] + tmpFx[75]*tmpObjS[107] + tmpFx[84]*tmpObjS[119] + tmpFx[93]*tmpObjS[131] + tmpFx[102]*tmpObjS[143];
tmpQ2[48] = + tmpFx[4]*tmpObjS[0] + tmpFx[13]*tmpObjS[12] + tmpFx[22]*tmpObjS[24] + tmpFx[31]*tmpObjS[36] + tmpFx[40]*tmpObjS[48] + tmpFx[49]*tmpObjS[60] + tmpFx[58]*tmpObjS[72] + tmpFx[67]*tmpObjS[84] + tmpFx[76]*tmpObjS[96] + tmpFx[85]*tmpObjS[108] + tmpFx[94]*tmpObjS[120] + tmpFx[103]*tmpObjS[132];
tmpQ2[49] = + tmpFx[4]*tmpObjS[1] + tmpFx[13]*tmpObjS[13] + tmpFx[22]*tmpObjS[25] + tmpFx[31]*tmpObjS[37] + tmpFx[40]*tmpObjS[49] + tmpFx[49]*tmpObjS[61] + tmpFx[58]*tmpObjS[73] + tmpFx[67]*tmpObjS[85] + tmpFx[76]*tmpObjS[97] + tmpFx[85]*tmpObjS[109] + tmpFx[94]*tmpObjS[121] + tmpFx[103]*tmpObjS[133];
tmpQ2[50] = + tmpFx[4]*tmpObjS[2] + tmpFx[13]*tmpObjS[14] + tmpFx[22]*tmpObjS[26] + tmpFx[31]*tmpObjS[38] + tmpFx[40]*tmpObjS[50] + tmpFx[49]*tmpObjS[62] + tmpFx[58]*tmpObjS[74] + tmpFx[67]*tmpObjS[86] + tmpFx[76]*tmpObjS[98] + tmpFx[85]*tmpObjS[110] + tmpFx[94]*tmpObjS[122] + tmpFx[103]*tmpObjS[134];
tmpQ2[51] = + tmpFx[4]*tmpObjS[3] + tmpFx[13]*tmpObjS[15] + tmpFx[22]*tmpObjS[27] + tmpFx[31]*tmpObjS[39] + tmpFx[40]*tmpObjS[51] + tmpFx[49]*tmpObjS[63] + tmpFx[58]*tmpObjS[75] + tmpFx[67]*tmpObjS[87] + tmpFx[76]*tmpObjS[99] + tmpFx[85]*tmpObjS[111] + tmpFx[94]*tmpObjS[123] + tmpFx[103]*tmpObjS[135];
tmpQ2[52] = + tmpFx[4]*tmpObjS[4] + tmpFx[13]*tmpObjS[16] + tmpFx[22]*tmpObjS[28] + tmpFx[31]*tmpObjS[40] + tmpFx[40]*tmpObjS[52] + tmpFx[49]*tmpObjS[64] + tmpFx[58]*tmpObjS[76] + tmpFx[67]*tmpObjS[88] + tmpFx[76]*tmpObjS[100] + tmpFx[85]*tmpObjS[112] + tmpFx[94]*tmpObjS[124] + tmpFx[103]*tmpObjS[136];
tmpQ2[53] = + tmpFx[4]*tmpObjS[5] + tmpFx[13]*tmpObjS[17] + tmpFx[22]*tmpObjS[29] + tmpFx[31]*tmpObjS[41] + tmpFx[40]*tmpObjS[53] + tmpFx[49]*tmpObjS[65] + tmpFx[58]*tmpObjS[77] + tmpFx[67]*tmpObjS[89] + tmpFx[76]*tmpObjS[101] + tmpFx[85]*tmpObjS[113] + tmpFx[94]*tmpObjS[125] + tmpFx[103]*tmpObjS[137];
tmpQ2[54] = + tmpFx[4]*tmpObjS[6] + tmpFx[13]*tmpObjS[18] + tmpFx[22]*tmpObjS[30] + tmpFx[31]*tmpObjS[42] + tmpFx[40]*tmpObjS[54] + tmpFx[49]*tmpObjS[66] + tmpFx[58]*tmpObjS[78] + tmpFx[67]*tmpObjS[90] + tmpFx[76]*tmpObjS[102] + tmpFx[85]*tmpObjS[114] + tmpFx[94]*tmpObjS[126] + tmpFx[103]*tmpObjS[138];
tmpQ2[55] = + tmpFx[4]*tmpObjS[7] + tmpFx[13]*tmpObjS[19] + tmpFx[22]*tmpObjS[31] + tmpFx[31]*tmpObjS[43] + tmpFx[40]*tmpObjS[55] + tmpFx[49]*tmpObjS[67] + tmpFx[58]*tmpObjS[79] + tmpFx[67]*tmpObjS[91] + tmpFx[76]*tmpObjS[103] + tmpFx[85]*tmpObjS[115] + tmpFx[94]*tmpObjS[127] + tmpFx[103]*tmpObjS[139];
tmpQ2[56] = + tmpFx[4]*tmpObjS[8] + tmpFx[13]*tmpObjS[20] + tmpFx[22]*tmpObjS[32] + tmpFx[31]*tmpObjS[44] + tmpFx[40]*tmpObjS[56] + tmpFx[49]*tmpObjS[68] + tmpFx[58]*tmpObjS[80] + tmpFx[67]*tmpObjS[92] + tmpFx[76]*tmpObjS[104] + tmpFx[85]*tmpObjS[116] + tmpFx[94]*tmpObjS[128] + tmpFx[103]*tmpObjS[140];
tmpQ2[57] = + tmpFx[4]*tmpObjS[9] + tmpFx[13]*tmpObjS[21] + tmpFx[22]*tmpObjS[33] + tmpFx[31]*tmpObjS[45] + tmpFx[40]*tmpObjS[57] + tmpFx[49]*tmpObjS[69] + tmpFx[58]*tmpObjS[81] + tmpFx[67]*tmpObjS[93] + tmpFx[76]*tmpObjS[105] + tmpFx[85]*tmpObjS[117] + tmpFx[94]*tmpObjS[129] + tmpFx[103]*tmpObjS[141];
tmpQ2[58] = + tmpFx[4]*tmpObjS[10] + tmpFx[13]*tmpObjS[22] + tmpFx[22]*tmpObjS[34] + tmpFx[31]*tmpObjS[46] + tmpFx[40]*tmpObjS[58] + tmpFx[49]*tmpObjS[70] + tmpFx[58]*tmpObjS[82] + tmpFx[67]*tmpObjS[94] + tmpFx[76]*tmpObjS[106] + tmpFx[85]*tmpObjS[118] + tmpFx[94]*tmpObjS[130] + tmpFx[103]*tmpObjS[142];
tmpQ2[59] = + tmpFx[4]*tmpObjS[11] + tmpFx[13]*tmpObjS[23] + tmpFx[22]*tmpObjS[35] + tmpFx[31]*tmpObjS[47] + tmpFx[40]*tmpObjS[59] + tmpFx[49]*tmpObjS[71] + tmpFx[58]*tmpObjS[83] + tmpFx[67]*tmpObjS[95] + tmpFx[76]*tmpObjS[107] + tmpFx[85]*tmpObjS[119] + tmpFx[94]*tmpObjS[131] + tmpFx[103]*tmpObjS[143];
tmpQ2[60] = + tmpFx[5]*tmpObjS[0] + tmpFx[14]*tmpObjS[12] + tmpFx[23]*tmpObjS[24] + tmpFx[32]*tmpObjS[36] + tmpFx[41]*tmpObjS[48] + tmpFx[50]*tmpObjS[60] + tmpFx[59]*tmpObjS[72] + tmpFx[68]*tmpObjS[84] + tmpFx[77]*tmpObjS[96] + tmpFx[86]*tmpObjS[108] + tmpFx[95]*tmpObjS[120] + tmpFx[104]*tmpObjS[132];
tmpQ2[61] = + tmpFx[5]*tmpObjS[1] + tmpFx[14]*tmpObjS[13] + tmpFx[23]*tmpObjS[25] + tmpFx[32]*tmpObjS[37] + tmpFx[41]*tmpObjS[49] + tmpFx[50]*tmpObjS[61] + tmpFx[59]*tmpObjS[73] + tmpFx[68]*tmpObjS[85] + tmpFx[77]*tmpObjS[97] + tmpFx[86]*tmpObjS[109] + tmpFx[95]*tmpObjS[121] + tmpFx[104]*tmpObjS[133];
tmpQ2[62] = + tmpFx[5]*tmpObjS[2] + tmpFx[14]*tmpObjS[14] + tmpFx[23]*tmpObjS[26] + tmpFx[32]*tmpObjS[38] + tmpFx[41]*tmpObjS[50] + tmpFx[50]*tmpObjS[62] + tmpFx[59]*tmpObjS[74] + tmpFx[68]*tmpObjS[86] + tmpFx[77]*tmpObjS[98] + tmpFx[86]*tmpObjS[110] + tmpFx[95]*tmpObjS[122] + tmpFx[104]*tmpObjS[134];
tmpQ2[63] = + tmpFx[5]*tmpObjS[3] + tmpFx[14]*tmpObjS[15] + tmpFx[23]*tmpObjS[27] + tmpFx[32]*tmpObjS[39] + tmpFx[41]*tmpObjS[51] + tmpFx[50]*tmpObjS[63] + tmpFx[59]*tmpObjS[75] + tmpFx[68]*tmpObjS[87] + tmpFx[77]*tmpObjS[99] + tmpFx[86]*tmpObjS[111] + tmpFx[95]*tmpObjS[123] + tmpFx[104]*tmpObjS[135];
tmpQ2[64] = + tmpFx[5]*tmpObjS[4] + tmpFx[14]*tmpObjS[16] + tmpFx[23]*tmpObjS[28] + tmpFx[32]*tmpObjS[40] + tmpFx[41]*tmpObjS[52] + tmpFx[50]*tmpObjS[64] + tmpFx[59]*tmpObjS[76] + tmpFx[68]*tmpObjS[88] + tmpFx[77]*tmpObjS[100] + tmpFx[86]*tmpObjS[112] + tmpFx[95]*tmpObjS[124] + tmpFx[104]*tmpObjS[136];
tmpQ2[65] = + tmpFx[5]*tmpObjS[5] + tmpFx[14]*tmpObjS[17] + tmpFx[23]*tmpObjS[29] + tmpFx[32]*tmpObjS[41] + tmpFx[41]*tmpObjS[53] + tmpFx[50]*tmpObjS[65] + tmpFx[59]*tmpObjS[77] + tmpFx[68]*tmpObjS[89] + tmpFx[77]*tmpObjS[101] + tmpFx[86]*tmpObjS[113] + tmpFx[95]*tmpObjS[125] + tmpFx[104]*tmpObjS[137];
tmpQ2[66] = + tmpFx[5]*tmpObjS[6] + tmpFx[14]*tmpObjS[18] + tmpFx[23]*tmpObjS[30] + tmpFx[32]*tmpObjS[42] + tmpFx[41]*tmpObjS[54] + tmpFx[50]*tmpObjS[66] + tmpFx[59]*tmpObjS[78] + tmpFx[68]*tmpObjS[90] + tmpFx[77]*tmpObjS[102] + tmpFx[86]*tmpObjS[114] + tmpFx[95]*tmpObjS[126] + tmpFx[104]*tmpObjS[138];
tmpQ2[67] = + tmpFx[5]*tmpObjS[7] + tmpFx[14]*tmpObjS[19] + tmpFx[23]*tmpObjS[31] + tmpFx[32]*tmpObjS[43] + tmpFx[41]*tmpObjS[55] + tmpFx[50]*tmpObjS[67] + tmpFx[59]*tmpObjS[79] + tmpFx[68]*tmpObjS[91] + tmpFx[77]*tmpObjS[103] + tmpFx[86]*tmpObjS[115] + tmpFx[95]*tmpObjS[127] + tmpFx[104]*tmpObjS[139];
tmpQ2[68] = + tmpFx[5]*tmpObjS[8] + tmpFx[14]*tmpObjS[20] + tmpFx[23]*tmpObjS[32] + tmpFx[32]*tmpObjS[44] + tmpFx[41]*tmpObjS[56] + tmpFx[50]*tmpObjS[68] + tmpFx[59]*tmpObjS[80] + tmpFx[68]*tmpObjS[92] + tmpFx[77]*tmpObjS[104] + tmpFx[86]*tmpObjS[116] + tmpFx[95]*tmpObjS[128] + tmpFx[104]*tmpObjS[140];
tmpQ2[69] = + tmpFx[5]*tmpObjS[9] + tmpFx[14]*tmpObjS[21] + tmpFx[23]*tmpObjS[33] + tmpFx[32]*tmpObjS[45] + tmpFx[41]*tmpObjS[57] + tmpFx[50]*tmpObjS[69] + tmpFx[59]*tmpObjS[81] + tmpFx[68]*tmpObjS[93] + tmpFx[77]*tmpObjS[105] + tmpFx[86]*tmpObjS[117] + tmpFx[95]*tmpObjS[129] + tmpFx[104]*tmpObjS[141];
tmpQ2[70] = + tmpFx[5]*tmpObjS[10] + tmpFx[14]*tmpObjS[22] + tmpFx[23]*tmpObjS[34] + tmpFx[32]*tmpObjS[46] + tmpFx[41]*tmpObjS[58] + tmpFx[50]*tmpObjS[70] + tmpFx[59]*tmpObjS[82] + tmpFx[68]*tmpObjS[94] + tmpFx[77]*tmpObjS[106] + tmpFx[86]*tmpObjS[118] + tmpFx[95]*tmpObjS[130] + tmpFx[104]*tmpObjS[142];
tmpQ2[71] = + tmpFx[5]*tmpObjS[11] + tmpFx[14]*tmpObjS[23] + tmpFx[23]*tmpObjS[35] + tmpFx[32]*tmpObjS[47] + tmpFx[41]*tmpObjS[59] + tmpFx[50]*tmpObjS[71] + tmpFx[59]*tmpObjS[83] + tmpFx[68]*tmpObjS[95] + tmpFx[77]*tmpObjS[107] + tmpFx[86]*tmpObjS[119] + tmpFx[95]*tmpObjS[131] + tmpFx[104]*tmpObjS[143];
tmpQ2[72] = + tmpFx[6]*tmpObjS[0] + tmpFx[15]*tmpObjS[12] + tmpFx[24]*tmpObjS[24] + tmpFx[33]*tmpObjS[36] + tmpFx[42]*tmpObjS[48] + tmpFx[51]*tmpObjS[60] + tmpFx[60]*tmpObjS[72] + tmpFx[69]*tmpObjS[84] + tmpFx[78]*tmpObjS[96] + tmpFx[87]*tmpObjS[108] + tmpFx[96]*tmpObjS[120] + tmpFx[105]*tmpObjS[132];
tmpQ2[73] = + tmpFx[6]*tmpObjS[1] + tmpFx[15]*tmpObjS[13] + tmpFx[24]*tmpObjS[25] + tmpFx[33]*tmpObjS[37] + tmpFx[42]*tmpObjS[49] + tmpFx[51]*tmpObjS[61] + tmpFx[60]*tmpObjS[73] + tmpFx[69]*tmpObjS[85] + tmpFx[78]*tmpObjS[97] + tmpFx[87]*tmpObjS[109] + tmpFx[96]*tmpObjS[121] + tmpFx[105]*tmpObjS[133];
tmpQ2[74] = + tmpFx[6]*tmpObjS[2] + tmpFx[15]*tmpObjS[14] + tmpFx[24]*tmpObjS[26] + tmpFx[33]*tmpObjS[38] + tmpFx[42]*tmpObjS[50] + tmpFx[51]*tmpObjS[62] + tmpFx[60]*tmpObjS[74] + tmpFx[69]*tmpObjS[86] + tmpFx[78]*tmpObjS[98] + tmpFx[87]*tmpObjS[110] + tmpFx[96]*tmpObjS[122] + tmpFx[105]*tmpObjS[134];
tmpQ2[75] = + tmpFx[6]*tmpObjS[3] + tmpFx[15]*tmpObjS[15] + tmpFx[24]*tmpObjS[27] + tmpFx[33]*tmpObjS[39] + tmpFx[42]*tmpObjS[51] + tmpFx[51]*tmpObjS[63] + tmpFx[60]*tmpObjS[75] + tmpFx[69]*tmpObjS[87] + tmpFx[78]*tmpObjS[99] + tmpFx[87]*tmpObjS[111] + tmpFx[96]*tmpObjS[123] + tmpFx[105]*tmpObjS[135];
tmpQ2[76] = + tmpFx[6]*tmpObjS[4] + tmpFx[15]*tmpObjS[16] + tmpFx[24]*tmpObjS[28] + tmpFx[33]*tmpObjS[40] + tmpFx[42]*tmpObjS[52] + tmpFx[51]*tmpObjS[64] + tmpFx[60]*tmpObjS[76] + tmpFx[69]*tmpObjS[88] + tmpFx[78]*tmpObjS[100] + tmpFx[87]*tmpObjS[112] + tmpFx[96]*tmpObjS[124] + tmpFx[105]*tmpObjS[136];
tmpQ2[77] = + tmpFx[6]*tmpObjS[5] + tmpFx[15]*tmpObjS[17] + tmpFx[24]*tmpObjS[29] + tmpFx[33]*tmpObjS[41] + tmpFx[42]*tmpObjS[53] + tmpFx[51]*tmpObjS[65] + tmpFx[60]*tmpObjS[77] + tmpFx[69]*tmpObjS[89] + tmpFx[78]*tmpObjS[101] + tmpFx[87]*tmpObjS[113] + tmpFx[96]*tmpObjS[125] + tmpFx[105]*tmpObjS[137];
tmpQ2[78] = + tmpFx[6]*tmpObjS[6] + tmpFx[15]*tmpObjS[18] + tmpFx[24]*tmpObjS[30] + tmpFx[33]*tmpObjS[42] + tmpFx[42]*tmpObjS[54] + tmpFx[51]*tmpObjS[66] + tmpFx[60]*tmpObjS[78] + tmpFx[69]*tmpObjS[90] + tmpFx[78]*tmpObjS[102] + tmpFx[87]*tmpObjS[114] + tmpFx[96]*tmpObjS[126] + tmpFx[105]*tmpObjS[138];
tmpQ2[79] = + tmpFx[6]*tmpObjS[7] + tmpFx[15]*tmpObjS[19] + tmpFx[24]*tmpObjS[31] + tmpFx[33]*tmpObjS[43] + tmpFx[42]*tmpObjS[55] + tmpFx[51]*tmpObjS[67] + tmpFx[60]*tmpObjS[79] + tmpFx[69]*tmpObjS[91] + tmpFx[78]*tmpObjS[103] + tmpFx[87]*tmpObjS[115] + tmpFx[96]*tmpObjS[127] + tmpFx[105]*tmpObjS[139];
tmpQ2[80] = + tmpFx[6]*tmpObjS[8] + tmpFx[15]*tmpObjS[20] + tmpFx[24]*tmpObjS[32] + tmpFx[33]*tmpObjS[44] + tmpFx[42]*tmpObjS[56] + tmpFx[51]*tmpObjS[68] + tmpFx[60]*tmpObjS[80] + tmpFx[69]*tmpObjS[92] + tmpFx[78]*tmpObjS[104] + tmpFx[87]*tmpObjS[116] + tmpFx[96]*tmpObjS[128] + tmpFx[105]*tmpObjS[140];
tmpQ2[81] = + tmpFx[6]*tmpObjS[9] + tmpFx[15]*tmpObjS[21] + tmpFx[24]*tmpObjS[33] + tmpFx[33]*tmpObjS[45] + tmpFx[42]*tmpObjS[57] + tmpFx[51]*tmpObjS[69] + tmpFx[60]*tmpObjS[81] + tmpFx[69]*tmpObjS[93] + tmpFx[78]*tmpObjS[105] + tmpFx[87]*tmpObjS[117] + tmpFx[96]*tmpObjS[129] + tmpFx[105]*tmpObjS[141];
tmpQ2[82] = + tmpFx[6]*tmpObjS[10] + tmpFx[15]*tmpObjS[22] + tmpFx[24]*tmpObjS[34] + tmpFx[33]*tmpObjS[46] + tmpFx[42]*tmpObjS[58] + tmpFx[51]*tmpObjS[70] + tmpFx[60]*tmpObjS[82] + tmpFx[69]*tmpObjS[94] + tmpFx[78]*tmpObjS[106] + tmpFx[87]*tmpObjS[118] + tmpFx[96]*tmpObjS[130] + tmpFx[105]*tmpObjS[142];
tmpQ2[83] = + tmpFx[6]*tmpObjS[11] + tmpFx[15]*tmpObjS[23] + tmpFx[24]*tmpObjS[35] + tmpFx[33]*tmpObjS[47] + tmpFx[42]*tmpObjS[59] + tmpFx[51]*tmpObjS[71] + tmpFx[60]*tmpObjS[83] + tmpFx[69]*tmpObjS[95] + tmpFx[78]*tmpObjS[107] + tmpFx[87]*tmpObjS[119] + tmpFx[96]*tmpObjS[131] + tmpFx[105]*tmpObjS[143];
tmpQ2[84] = + tmpFx[7]*tmpObjS[0] + tmpFx[16]*tmpObjS[12] + tmpFx[25]*tmpObjS[24] + tmpFx[34]*tmpObjS[36] + tmpFx[43]*tmpObjS[48] + tmpFx[52]*tmpObjS[60] + tmpFx[61]*tmpObjS[72] + tmpFx[70]*tmpObjS[84] + tmpFx[79]*tmpObjS[96] + tmpFx[88]*tmpObjS[108] + tmpFx[97]*tmpObjS[120] + tmpFx[106]*tmpObjS[132];
tmpQ2[85] = + tmpFx[7]*tmpObjS[1] + tmpFx[16]*tmpObjS[13] + tmpFx[25]*tmpObjS[25] + tmpFx[34]*tmpObjS[37] + tmpFx[43]*tmpObjS[49] + tmpFx[52]*tmpObjS[61] + tmpFx[61]*tmpObjS[73] + tmpFx[70]*tmpObjS[85] + tmpFx[79]*tmpObjS[97] + tmpFx[88]*tmpObjS[109] + tmpFx[97]*tmpObjS[121] + tmpFx[106]*tmpObjS[133];
tmpQ2[86] = + tmpFx[7]*tmpObjS[2] + tmpFx[16]*tmpObjS[14] + tmpFx[25]*tmpObjS[26] + tmpFx[34]*tmpObjS[38] + tmpFx[43]*tmpObjS[50] + tmpFx[52]*tmpObjS[62] + tmpFx[61]*tmpObjS[74] + tmpFx[70]*tmpObjS[86] + tmpFx[79]*tmpObjS[98] + tmpFx[88]*tmpObjS[110] + tmpFx[97]*tmpObjS[122] + tmpFx[106]*tmpObjS[134];
tmpQ2[87] = + tmpFx[7]*tmpObjS[3] + tmpFx[16]*tmpObjS[15] + tmpFx[25]*tmpObjS[27] + tmpFx[34]*tmpObjS[39] + tmpFx[43]*tmpObjS[51] + tmpFx[52]*tmpObjS[63] + tmpFx[61]*tmpObjS[75] + tmpFx[70]*tmpObjS[87] + tmpFx[79]*tmpObjS[99] + tmpFx[88]*tmpObjS[111] + tmpFx[97]*tmpObjS[123] + tmpFx[106]*tmpObjS[135];
tmpQ2[88] = + tmpFx[7]*tmpObjS[4] + tmpFx[16]*tmpObjS[16] + tmpFx[25]*tmpObjS[28] + tmpFx[34]*tmpObjS[40] + tmpFx[43]*tmpObjS[52] + tmpFx[52]*tmpObjS[64] + tmpFx[61]*tmpObjS[76] + tmpFx[70]*tmpObjS[88] + tmpFx[79]*tmpObjS[100] + tmpFx[88]*tmpObjS[112] + tmpFx[97]*tmpObjS[124] + tmpFx[106]*tmpObjS[136];
tmpQ2[89] = + tmpFx[7]*tmpObjS[5] + tmpFx[16]*tmpObjS[17] + tmpFx[25]*tmpObjS[29] + tmpFx[34]*tmpObjS[41] + tmpFx[43]*tmpObjS[53] + tmpFx[52]*tmpObjS[65] + tmpFx[61]*tmpObjS[77] + tmpFx[70]*tmpObjS[89] + tmpFx[79]*tmpObjS[101] + tmpFx[88]*tmpObjS[113] + tmpFx[97]*tmpObjS[125] + tmpFx[106]*tmpObjS[137];
tmpQ2[90] = + tmpFx[7]*tmpObjS[6] + tmpFx[16]*tmpObjS[18] + tmpFx[25]*tmpObjS[30] + tmpFx[34]*tmpObjS[42] + tmpFx[43]*tmpObjS[54] + tmpFx[52]*tmpObjS[66] + tmpFx[61]*tmpObjS[78] + tmpFx[70]*tmpObjS[90] + tmpFx[79]*tmpObjS[102] + tmpFx[88]*tmpObjS[114] + tmpFx[97]*tmpObjS[126] + tmpFx[106]*tmpObjS[138];
tmpQ2[91] = + tmpFx[7]*tmpObjS[7] + tmpFx[16]*tmpObjS[19] + tmpFx[25]*tmpObjS[31] + tmpFx[34]*tmpObjS[43] + tmpFx[43]*tmpObjS[55] + tmpFx[52]*tmpObjS[67] + tmpFx[61]*tmpObjS[79] + tmpFx[70]*tmpObjS[91] + tmpFx[79]*tmpObjS[103] + tmpFx[88]*tmpObjS[115] + tmpFx[97]*tmpObjS[127] + tmpFx[106]*tmpObjS[139];
tmpQ2[92] = + tmpFx[7]*tmpObjS[8] + tmpFx[16]*tmpObjS[20] + tmpFx[25]*tmpObjS[32] + tmpFx[34]*tmpObjS[44] + tmpFx[43]*tmpObjS[56] + tmpFx[52]*tmpObjS[68] + tmpFx[61]*tmpObjS[80] + tmpFx[70]*tmpObjS[92] + tmpFx[79]*tmpObjS[104] + tmpFx[88]*tmpObjS[116] + tmpFx[97]*tmpObjS[128] + tmpFx[106]*tmpObjS[140];
tmpQ2[93] = + tmpFx[7]*tmpObjS[9] + tmpFx[16]*tmpObjS[21] + tmpFx[25]*tmpObjS[33] + tmpFx[34]*tmpObjS[45] + tmpFx[43]*tmpObjS[57] + tmpFx[52]*tmpObjS[69] + tmpFx[61]*tmpObjS[81] + tmpFx[70]*tmpObjS[93] + tmpFx[79]*tmpObjS[105] + tmpFx[88]*tmpObjS[117] + tmpFx[97]*tmpObjS[129] + tmpFx[106]*tmpObjS[141];
tmpQ2[94] = + tmpFx[7]*tmpObjS[10] + tmpFx[16]*tmpObjS[22] + tmpFx[25]*tmpObjS[34] + tmpFx[34]*tmpObjS[46] + tmpFx[43]*tmpObjS[58] + tmpFx[52]*tmpObjS[70] + tmpFx[61]*tmpObjS[82] + tmpFx[70]*tmpObjS[94] + tmpFx[79]*tmpObjS[106] + tmpFx[88]*tmpObjS[118] + tmpFx[97]*tmpObjS[130] + tmpFx[106]*tmpObjS[142];
tmpQ2[95] = + tmpFx[7]*tmpObjS[11] + tmpFx[16]*tmpObjS[23] + tmpFx[25]*tmpObjS[35] + tmpFx[34]*tmpObjS[47] + tmpFx[43]*tmpObjS[59] + tmpFx[52]*tmpObjS[71] + tmpFx[61]*tmpObjS[83] + tmpFx[70]*tmpObjS[95] + tmpFx[79]*tmpObjS[107] + tmpFx[88]*tmpObjS[119] + tmpFx[97]*tmpObjS[131] + tmpFx[106]*tmpObjS[143];
tmpQ2[96] = + tmpFx[8]*tmpObjS[0] + tmpFx[17]*tmpObjS[12] + tmpFx[26]*tmpObjS[24] + tmpFx[35]*tmpObjS[36] + tmpFx[44]*tmpObjS[48] + tmpFx[53]*tmpObjS[60] + tmpFx[62]*tmpObjS[72] + tmpFx[71]*tmpObjS[84] + tmpFx[80]*tmpObjS[96] + tmpFx[89]*tmpObjS[108] + tmpFx[98]*tmpObjS[120] + tmpFx[107]*tmpObjS[132];
tmpQ2[97] = + tmpFx[8]*tmpObjS[1] + tmpFx[17]*tmpObjS[13] + tmpFx[26]*tmpObjS[25] + tmpFx[35]*tmpObjS[37] + tmpFx[44]*tmpObjS[49] + tmpFx[53]*tmpObjS[61] + tmpFx[62]*tmpObjS[73] + tmpFx[71]*tmpObjS[85] + tmpFx[80]*tmpObjS[97] + tmpFx[89]*tmpObjS[109] + tmpFx[98]*tmpObjS[121] + tmpFx[107]*tmpObjS[133];
tmpQ2[98] = + tmpFx[8]*tmpObjS[2] + tmpFx[17]*tmpObjS[14] + tmpFx[26]*tmpObjS[26] + tmpFx[35]*tmpObjS[38] + tmpFx[44]*tmpObjS[50] + tmpFx[53]*tmpObjS[62] + tmpFx[62]*tmpObjS[74] + tmpFx[71]*tmpObjS[86] + tmpFx[80]*tmpObjS[98] + tmpFx[89]*tmpObjS[110] + tmpFx[98]*tmpObjS[122] + tmpFx[107]*tmpObjS[134];
tmpQ2[99] = + tmpFx[8]*tmpObjS[3] + tmpFx[17]*tmpObjS[15] + tmpFx[26]*tmpObjS[27] + tmpFx[35]*tmpObjS[39] + tmpFx[44]*tmpObjS[51] + tmpFx[53]*tmpObjS[63] + tmpFx[62]*tmpObjS[75] + tmpFx[71]*tmpObjS[87] + tmpFx[80]*tmpObjS[99] + tmpFx[89]*tmpObjS[111] + tmpFx[98]*tmpObjS[123] + tmpFx[107]*tmpObjS[135];
tmpQ2[100] = + tmpFx[8]*tmpObjS[4] + tmpFx[17]*tmpObjS[16] + tmpFx[26]*tmpObjS[28] + tmpFx[35]*tmpObjS[40] + tmpFx[44]*tmpObjS[52] + tmpFx[53]*tmpObjS[64] + tmpFx[62]*tmpObjS[76] + tmpFx[71]*tmpObjS[88] + tmpFx[80]*tmpObjS[100] + tmpFx[89]*tmpObjS[112] + tmpFx[98]*tmpObjS[124] + tmpFx[107]*tmpObjS[136];
tmpQ2[101] = + tmpFx[8]*tmpObjS[5] + tmpFx[17]*tmpObjS[17] + tmpFx[26]*tmpObjS[29] + tmpFx[35]*tmpObjS[41] + tmpFx[44]*tmpObjS[53] + tmpFx[53]*tmpObjS[65] + tmpFx[62]*tmpObjS[77] + tmpFx[71]*tmpObjS[89] + tmpFx[80]*tmpObjS[101] + tmpFx[89]*tmpObjS[113] + tmpFx[98]*tmpObjS[125] + tmpFx[107]*tmpObjS[137];
tmpQ2[102] = + tmpFx[8]*tmpObjS[6] + tmpFx[17]*tmpObjS[18] + tmpFx[26]*tmpObjS[30] + tmpFx[35]*tmpObjS[42] + tmpFx[44]*tmpObjS[54] + tmpFx[53]*tmpObjS[66] + tmpFx[62]*tmpObjS[78] + tmpFx[71]*tmpObjS[90] + tmpFx[80]*tmpObjS[102] + tmpFx[89]*tmpObjS[114] + tmpFx[98]*tmpObjS[126] + tmpFx[107]*tmpObjS[138];
tmpQ2[103] = + tmpFx[8]*tmpObjS[7] + tmpFx[17]*tmpObjS[19] + tmpFx[26]*tmpObjS[31] + tmpFx[35]*tmpObjS[43] + tmpFx[44]*tmpObjS[55] + tmpFx[53]*tmpObjS[67] + tmpFx[62]*tmpObjS[79] + tmpFx[71]*tmpObjS[91] + tmpFx[80]*tmpObjS[103] + tmpFx[89]*tmpObjS[115] + tmpFx[98]*tmpObjS[127] + tmpFx[107]*tmpObjS[139];
tmpQ2[104] = + tmpFx[8]*tmpObjS[8] + tmpFx[17]*tmpObjS[20] + tmpFx[26]*tmpObjS[32] + tmpFx[35]*tmpObjS[44] + tmpFx[44]*tmpObjS[56] + tmpFx[53]*tmpObjS[68] + tmpFx[62]*tmpObjS[80] + tmpFx[71]*tmpObjS[92] + tmpFx[80]*tmpObjS[104] + tmpFx[89]*tmpObjS[116] + tmpFx[98]*tmpObjS[128] + tmpFx[107]*tmpObjS[140];
tmpQ2[105] = + tmpFx[8]*tmpObjS[9] + tmpFx[17]*tmpObjS[21] + tmpFx[26]*tmpObjS[33] + tmpFx[35]*tmpObjS[45] + tmpFx[44]*tmpObjS[57] + tmpFx[53]*tmpObjS[69] + tmpFx[62]*tmpObjS[81] + tmpFx[71]*tmpObjS[93] + tmpFx[80]*tmpObjS[105] + tmpFx[89]*tmpObjS[117] + tmpFx[98]*tmpObjS[129] + tmpFx[107]*tmpObjS[141];
tmpQ2[106] = + tmpFx[8]*tmpObjS[10] + tmpFx[17]*tmpObjS[22] + tmpFx[26]*tmpObjS[34] + tmpFx[35]*tmpObjS[46] + tmpFx[44]*tmpObjS[58] + tmpFx[53]*tmpObjS[70] + tmpFx[62]*tmpObjS[82] + tmpFx[71]*tmpObjS[94] + tmpFx[80]*tmpObjS[106] + tmpFx[89]*tmpObjS[118] + tmpFx[98]*tmpObjS[130] + tmpFx[107]*tmpObjS[142];
tmpQ2[107] = + tmpFx[8]*tmpObjS[11] + tmpFx[17]*tmpObjS[23] + tmpFx[26]*tmpObjS[35] + tmpFx[35]*tmpObjS[47] + tmpFx[44]*tmpObjS[59] + tmpFx[53]*tmpObjS[71] + tmpFx[62]*tmpObjS[83] + tmpFx[71]*tmpObjS[95] + tmpFx[80]*tmpObjS[107] + tmpFx[89]*tmpObjS[119] + tmpFx[98]*tmpObjS[131] + tmpFx[107]*tmpObjS[143];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[9] + tmpQ2[2]*tmpFx[18] + tmpQ2[3]*tmpFx[27] + tmpQ2[4]*tmpFx[36] + tmpQ2[5]*tmpFx[45] + tmpQ2[6]*tmpFx[54] + tmpQ2[7]*tmpFx[63] + tmpQ2[8]*tmpFx[72] + tmpQ2[9]*tmpFx[81] + tmpQ2[10]*tmpFx[90] + tmpQ2[11]*tmpFx[99];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[10] + tmpQ2[2]*tmpFx[19] + tmpQ2[3]*tmpFx[28] + tmpQ2[4]*tmpFx[37] + tmpQ2[5]*tmpFx[46] + tmpQ2[6]*tmpFx[55] + tmpQ2[7]*tmpFx[64] + tmpQ2[8]*tmpFx[73] + tmpQ2[9]*tmpFx[82] + tmpQ2[10]*tmpFx[91] + tmpQ2[11]*tmpFx[100];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[11] + tmpQ2[2]*tmpFx[20] + tmpQ2[3]*tmpFx[29] + tmpQ2[4]*tmpFx[38] + tmpQ2[5]*tmpFx[47] + tmpQ2[6]*tmpFx[56] + tmpQ2[7]*tmpFx[65] + tmpQ2[8]*tmpFx[74] + tmpQ2[9]*tmpFx[83] + tmpQ2[10]*tmpFx[92] + tmpQ2[11]*tmpFx[101];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[12] + tmpQ2[2]*tmpFx[21] + tmpQ2[3]*tmpFx[30] + tmpQ2[4]*tmpFx[39] + tmpQ2[5]*tmpFx[48] + tmpQ2[6]*tmpFx[57] + tmpQ2[7]*tmpFx[66] + tmpQ2[8]*tmpFx[75] + tmpQ2[9]*tmpFx[84] + tmpQ2[10]*tmpFx[93] + tmpQ2[11]*tmpFx[102];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[13] + tmpQ2[2]*tmpFx[22] + tmpQ2[3]*tmpFx[31] + tmpQ2[4]*tmpFx[40] + tmpQ2[5]*tmpFx[49] + tmpQ2[6]*tmpFx[58] + tmpQ2[7]*tmpFx[67] + tmpQ2[8]*tmpFx[76] + tmpQ2[9]*tmpFx[85] + tmpQ2[10]*tmpFx[94] + tmpQ2[11]*tmpFx[103];
tmpQ1[5] = + tmpQ2[0]*tmpFx[5] + tmpQ2[1]*tmpFx[14] + tmpQ2[2]*tmpFx[23] + tmpQ2[3]*tmpFx[32] + tmpQ2[4]*tmpFx[41] + tmpQ2[5]*tmpFx[50] + tmpQ2[6]*tmpFx[59] + tmpQ2[7]*tmpFx[68] + tmpQ2[8]*tmpFx[77] + tmpQ2[9]*tmpFx[86] + tmpQ2[10]*tmpFx[95] + tmpQ2[11]*tmpFx[104];
tmpQ1[6] = + tmpQ2[0]*tmpFx[6] + tmpQ2[1]*tmpFx[15] + tmpQ2[2]*tmpFx[24] + tmpQ2[3]*tmpFx[33] + tmpQ2[4]*tmpFx[42] + tmpQ2[5]*tmpFx[51] + tmpQ2[6]*tmpFx[60] + tmpQ2[7]*tmpFx[69] + tmpQ2[8]*tmpFx[78] + tmpQ2[9]*tmpFx[87] + tmpQ2[10]*tmpFx[96] + tmpQ2[11]*tmpFx[105];
tmpQ1[7] = + tmpQ2[0]*tmpFx[7] + tmpQ2[1]*tmpFx[16] + tmpQ2[2]*tmpFx[25] + tmpQ2[3]*tmpFx[34] + tmpQ2[4]*tmpFx[43] + tmpQ2[5]*tmpFx[52] + tmpQ2[6]*tmpFx[61] + tmpQ2[7]*tmpFx[70] + tmpQ2[8]*tmpFx[79] + tmpQ2[9]*tmpFx[88] + tmpQ2[10]*tmpFx[97] + tmpQ2[11]*tmpFx[106];
tmpQ1[8] = + tmpQ2[0]*tmpFx[8] + tmpQ2[1]*tmpFx[17] + tmpQ2[2]*tmpFx[26] + tmpQ2[3]*tmpFx[35] + tmpQ2[4]*tmpFx[44] + tmpQ2[5]*tmpFx[53] + tmpQ2[6]*tmpFx[62] + tmpQ2[7]*tmpFx[71] + tmpQ2[8]*tmpFx[80] + tmpQ2[9]*tmpFx[89] + tmpQ2[10]*tmpFx[98] + tmpQ2[11]*tmpFx[107];
tmpQ1[9] = + tmpQ2[12]*tmpFx[0] + tmpQ2[13]*tmpFx[9] + tmpQ2[14]*tmpFx[18] + tmpQ2[15]*tmpFx[27] + tmpQ2[16]*tmpFx[36] + tmpQ2[17]*tmpFx[45] + tmpQ2[18]*tmpFx[54] + tmpQ2[19]*tmpFx[63] + tmpQ2[20]*tmpFx[72] + tmpQ2[21]*tmpFx[81] + tmpQ2[22]*tmpFx[90] + tmpQ2[23]*tmpFx[99];
tmpQ1[10] = + tmpQ2[12]*tmpFx[1] + tmpQ2[13]*tmpFx[10] + tmpQ2[14]*tmpFx[19] + tmpQ2[15]*tmpFx[28] + tmpQ2[16]*tmpFx[37] + tmpQ2[17]*tmpFx[46] + tmpQ2[18]*tmpFx[55] + tmpQ2[19]*tmpFx[64] + tmpQ2[20]*tmpFx[73] + tmpQ2[21]*tmpFx[82] + tmpQ2[22]*tmpFx[91] + tmpQ2[23]*tmpFx[100];
tmpQ1[11] = + tmpQ2[12]*tmpFx[2] + tmpQ2[13]*tmpFx[11] + tmpQ2[14]*tmpFx[20] + tmpQ2[15]*tmpFx[29] + tmpQ2[16]*tmpFx[38] + tmpQ2[17]*tmpFx[47] + tmpQ2[18]*tmpFx[56] + tmpQ2[19]*tmpFx[65] + tmpQ2[20]*tmpFx[74] + tmpQ2[21]*tmpFx[83] + tmpQ2[22]*tmpFx[92] + tmpQ2[23]*tmpFx[101];
tmpQ1[12] = + tmpQ2[12]*tmpFx[3] + tmpQ2[13]*tmpFx[12] + tmpQ2[14]*tmpFx[21] + tmpQ2[15]*tmpFx[30] + tmpQ2[16]*tmpFx[39] + tmpQ2[17]*tmpFx[48] + tmpQ2[18]*tmpFx[57] + tmpQ2[19]*tmpFx[66] + tmpQ2[20]*tmpFx[75] + tmpQ2[21]*tmpFx[84] + tmpQ2[22]*tmpFx[93] + tmpQ2[23]*tmpFx[102];
tmpQ1[13] = + tmpQ2[12]*tmpFx[4] + tmpQ2[13]*tmpFx[13] + tmpQ2[14]*tmpFx[22] + tmpQ2[15]*tmpFx[31] + tmpQ2[16]*tmpFx[40] + tmpQ2[17]*tmpFx[49] + tmpQ2[18]*tmpFx[58] + tmpQ2[19]*tmpFx[67] + tmpQ2[20]*tmpFx[76] + tmpQ2[21]*tmpFx[85] + tmpQ2[22]*tmpFx[94] + tmpQ2[23]*tmpFx[103];
tmpQ1[14] = + tmpQ2[12]*tmpFx[5] + tmpQ2[13]*tmpFx[14] + tmpQ2[14]*tmpFx[23] + tmpQ2[15]*tmpFx[32] + tmpQ2[16]*tmpFx[41] + tmpQ2[17]*tmpFx[50] + tmpQ2[18]*tmpFx[59] + tmpQ2[19]*tmpFx[68] + tmpQ2[20]*tmpFx[77] + tmpQ2[21]*tmpFx[86] + tmpQ2[22]*tmpFx[95] + tmpQ2[23]*tmpFx[104];
tmpQ1[15] = + tmpQ2[12]*tmpFx[6] + tmpQ2[13]*tmpFx[15] + tmpQ2[14]*tmpFx[24] + tmpQ2[15]*tmpFx[33] + tmpQ2[16]*tmpFx[42] + tmpQ2[17]*tmpFx[51] + tmpQ2[18]*tmpFx[60] + tmpQ2[19]*tmpFx[69] + tmpQ2[20]*tmpFx[78] + tmpQ2[21]*tmpFx[87] + tmpQ2[22]*tmpFx[96] + tmpQ2[23]*tmpFx[105];
tmpQ1[16] = + tmpQ2[12]*tmpFx[7] + tmpQ2[13]*tmpFx[16] + tmpQ2[14]*tmpFx[25] + tmpQ2[15]*tmpFx[34] + tmpQ2[16]*tmpFx[43] + tmpQ2[17]*tmpFx[52] + tmpQ2[18]*tmpFx[61] + tmpQ2[19]*tmpFx[70] + tmpQ2[20]*tmpFx[79] + tmpQ2[21]*tmpFx[88] + tmpQ2[22]*tmpFx[97] + tmpQ2[23]*tmpFx[106];
tmpQ1[17] = + tmpQ2[12]*tmpFx[8] + tmpQ2[13]*tmpFx[17] + tmpQ2[14]*tmpFx[26] + tmpQ2[15]*tmpFx[35] + tmpQ2[16]*tmpFx[44] + tmpQ2[17]*tmpFx[53] + tmpQ2[18]*tmpFx[62] + tmpQ2[19]*tmpFx[71] + tmpQ2[20]*tmpFx[80] + tmpQ2[21]*tmpFx[89] + tmpQ2[22]*tmpFx[98] + tmpQ2[23]*tmpFx[107];
tmpQ1[18] = + tmpQ2[24]*tmpFx[0] + tmpQ2[25]*tmpFx[9] + tmpQ2[26]*tmpFx[18] + tmpQ2[27]*tmpFx[27] + tmpQ2[28]*tmpFx[36] + tmpQ2[29]*tmpFx[45] + tmpQ2[30]*tmpFx[54] + tmpQ2[31]*tmpFx[63] + tmpQ2[32]*tmpFx[72] + tmpQ2[33]*tmpFx[81] + tmpQ2[34]*tmpFx[90] + tmpQ2[35]*tmpFx[99];
tmpQ1[19] = + tmpQ2[24]*tmpFx[1] + tmpQ2[25]*tmpFx[10] + tmpQ2[26]*tmpFx[19] + tmpQ2[27]*tmpFx[28] + tmpQ2[28]*tmpFx[37] + tmpQ2[29]*tmpFx[46] + tmpQ2[30]*tmpFx[55] + tmpQ2[31]*tmpFx[64] + tmpQ2[32]*tmpFx[73] + tmpQ2[33]*tmpFx[82] + tmpQ2[34]*tmpFx[91] + tmpQ2[35]*tmpFx[100];
tmpQ1[20] = + tmpQ2[24]*tmpFx[2] + tmpQ2[25]*tmpFx[11] + tmpQ2[26]*tmpFx[20] + tmpQ2[27]*tmpFx[29] + tmpQ2[28]*tmpFx[38] + tmpQ2[29]*tmpFx[47] + tmpQ2[30]*tmpFx[56] + tmpQ2[31]*tmpFx[65] + tmpQ2[32]*tmpFx[74] + tmpQ2[33]*tmpFx[83] + tmpQ2[34]*tmpFx[92] + tmpQ2[35]*tmpFx[101];
tmpQ1[21] = + tmpQ2[24]*tmpFx[3] + tmpQ2[25]*tmpFx[12] + tmpQ2[26]*tmpFx[21] + tmpQ2[27]*tmpFx[30] + tmpQ2[28]*tmpFx[39] + tmpQ2[29]*tmpFx[48] + tmpQ2[30]*tmpFx[57] + tmpQ2[31]*tmpFx[66] + tmpQ2[32]*tmpFx[75] + tmpQ2[33]*tmpFx[84] + tmpQ2[34]*tmpFx[93] + tmpQ2[35]*tmpFx[102];
tmpQ1[22] = + tmpQ2[24]*tmpFx[4] + tmpQ2[25]*tmpFx[13] + tmpQ2[26]*tmpFx[22] + tmpQ2[27]*tmpFx[31] + tmpQ2[28]*tmpFx[40] + tmpQ2[29]*tmpFx[49] + tmpQ2[30]*tmpFx[58] + tmpQ2[31]*tmpFx[67] + tmpQ2[32]*tmpFx[76] + tmpQ2[33]*tmpFx[85] + tmpQ2[34]*tmpFx[94] + tmpQ2[35]*tmpFx[103];
tmpQ1[23] = + tmpQ2[24]*tmpFx[5] + tmpQ2[25]*tmpFx[14] + tmpQ2[26]*tmpFx[23] + tmpQ2[27]*tmpFx[32] + tmpQ2[28]*tmpFx[41] + tmpQ2[29]*tmpFx[50] + tmpQ2[30]*tmpFx[59] + tmpQ2[31]*tmpFx[68] + tmpQ2[32]*tmpFx[77] + tmpQ2[33]*tmpFx[86] + tmpQ2[34]*tmpFx[95] + tmpQ2[35]*tmpFx[104];
tmpQ1[24] = + tmpQ2[24]*tmpFx[6] + tmpQ2[25]*tmpFx[15] + tmpQ2[26]*tmpFx[24] + tmpQ2[27]*tmpFx[33] + tmpQ2[28]*tmpFx[42] + tmpQ2[29]*tmpFx[51] + tmpQ2[30]*tmpFx[60] + tmpQ2[31]*tmpFx[69] + tmpQ2[32]*tmpFx[78] + tmpQ2[33]*tmpFx[87] + tmpQ2[34]*tmpFx[96] + tmpQ2[35]*tmpFx[105];
tmpQ1[25] = + tmpQ2[24]*tmpFx[7] + tmpQ2[25]*tmpFx[16] + tmpQ2[26]*tmpFx[25] + tmpQ2[27]*tmpFx[34] + tmpQ2[28]*tmpFx[43] + tmpQ2[29]*tmpFx[52] + tmpQ2[30]*tmpFx[61] + tmpQ2[31]*tmpFx[70] + tmpQ2[32]*tmpFx[79] + tmpQ2[33]*tmpFx[88] + tmpQ2[34]*tmpFx[97] + tmpQ2[35]*tmpFx[106];
tmpQ1[26] = + tmpQ2[24]*tmpFx[8] + tmpQ2[25]*tmpFx[17] + tmpQ2[26]*tmpFx[26] + tmpQ2[27]*tmpFx[35] + tmpQ2[28]*tmpFx[44] + tmpQ2[29]*tmpFx[53] + tmpQ2[30]*tmpFx[62] + tmpQ2[31]*tmpFx[71] + tmpQ2[32]*tmpFx[80] + tmpQ2[33]*tmpFx[89] + tmpQ2[34]*tmpFx[98] + tmpQ2[35]*tmpFx[107];
tmpQ1[27] = + tmpQ2[36]*tmpFx[0] + tmpQ2[37]*tmpFx[9] + tmpQ2[38]*tmpFx[18] + tmpQ2[39]*tmpFx[27] + tmpQ2[40]*tmpFx[36] + tmpQ2[41]*tmpFx[45] + tmpQ2[42]*tmpFx[54] + tmpQ2[43]*tmpFx[63] + tmpQ2[44]*tmpFx[72] + tmpQ2[45]*tmpFx[81] + tmpQ2[46]*tmpFx[90] + tmpQ2[47]*tmpFx[99];
tmpQ1[28] = + tmpQ2[36]*tmpFx[1] + tmpQ2[37]*tmpFx[10] + tmpQ2[38]*tmpFx[19] + tmpQ2[39]*tmpFx[28] + tmpQ2[40]*tmpFx[37] + tmpQ2[41]*tmpFx[46] + tmpQ2[42]*tmpFx[55] + tmpQ2[43]*tmpFx[64] + tmpQ2[44]*tmpFx[73] + tmpQ2[45]*tmpFx[82] + tmpQ2[46]*tmpFx[91] + tmpQ2[47]*tmpFx[100];
tmpQ1[29] = + tmpQ2[36]*tmpFx[2] + tmpQ2[37]*tmpFx[11] + tmpQ2[38]*tmpFx[20] + tmpQ2[39]*tmpFx[29] + tmpQ2[40]*tmpFx[38] + tmpQ2[41]*tmpFx[47] + tmpQ2[42]*tmpFx[56] + tmpQ2[43]*tmpFx[65] + tmpQ2[44]*tmpFx[74] + tmpQ2[45]*tmpFx[83] + tmpQ2[46]*tmpFx[92] + tmpQ2[47]*tmpFx[101];
tmpQ1[30] = + tmpQ2[36]*tmpFx[3] + tmpQ2[37]*tmpFx[12] + tmpQ2[38]*tmpFx[21] + tmpQ2[39]*tmpFx[30] + tmpQ2[40]*tmpFx[39] + tmpQ2[41]*tmpFx[48] + tmpQ2[42]*tmpFx[57] + tmpQ2[43]*tmpFx[66] + tmpQ2[44]*tmpFx[75] + tmpQ2[45]*tmpFx[84] + tmpQ2[46]*tmpFx[93] + tmpQ2[47]*tmpFx[102];
tmpQ1[31] = + tmpQ2[36]*tmpFx[4] + tmpQ2[37]*tmpFx[13] + tmpQ2[38]*tmpFx[22] + tmpQ2[39]*tmpFx[31] + tmpQ2[40]*tmpFx[40] + tmpQ2[41]*tmpFx[49] + tmpQ2[42]*tmpFx[58] + tmpQ2[43]*tmpFx[67] + tmpQ2[44]*tmpFx[76] + tmpQ2[45]*tmpFx[85] + tmpQ2[46]*tmpFx[94] + tmpQ2[47]*tmpFx[103];
tmpQ1[32] = + tmpQ2[36]*tmpFx[5] + tmpQ2[37]*tmpFx[14] + tmpQ2[38]*tmpFx[23] + tmpQ2[39]*tmpFx[32] + tmpQ2[40]*tmpFx[41] + tmpQ2[41]*tmpFx[50] + tmpQ2[42]*tmpFx[59] + tmpQ2[43]*tmpFx[68] + tmpQ2[44]*tmpFx[77] + tmpQ2[45]*tmpFx[86] + tmpQ2[46]*tmpFx[95] + tmpQ2[47]*tmpFx[104];
tmpQ1[33] = + tmpQ2[36]*tmpFx[6] + tmpQ2[37]*tmpFx[15] + tmpQ2[38]*tmpFx[24] + tmpQ2[39]*tmpFx[33] + tmpQ2[40]*tmpFx[42] + tmpQ2[41]*tmpFx[51] + tmpQ2[42]*tmpFx[60] + tmpQ2[43]*tmpFx[69] + tmpQ2[44]*tmpFx[78] + tmpQ2[45]*tmpFx[87] + tmpQ2[46]*tmpFx[96] + tmpQ2[47]*tmpFx[105];
tmpQ1[34] = + tmpQ2[36]*tmpFx[7] + tmpQ2[37]*tmpFx[16] + tmpQ2[38]*tmpFx[25] + tmpQ2[39]*tmpFx[34] + tmpQ2[40]*tmpFx[43] + tmpQ2[41]*tmpFx[52] + tmpQ2[42]*tmpFx[61] + tmpQ2[43]*tmpFx[70] + tmpQ2[44]*tmpFx[79] + tmpQ2[45]*tmpFx[88] + tmpQ2[46]*tmpFx[97] + tmpQ2[47]*tmpFx[106];
tmpQ1[35] = + tmpQ2[36]*tmpFx[8] + tmpQ2[37]*tmpFx[17] + tmpQ2[38]*tmpFx[26] + tmpQ2[39]*tmpFx[35] + tmpQ2[40]*tmpFx[44] + tmpQ2[41]*tmpFx[53] + tmpQ2[42]*tmpFx[62] + tmpQ2[43]*tmpFx[71] + tmpQ2[44]*tmpFx[80] + tmpQ2[45]*tmpFx[89] + tmpQ2[46]*tmpFx[98] + tmpQ2[47]*tmpFx[107];
tmpQ1[36] = + tmpQ2[48]*tmpFx[0] + tmpQ2[49]*tmpFx[9] + tmpQ2[50]*tmpFx[18] + tmpQ2[51]*tmpFx[27] + tmpQ2[52]*tmpFx[36] + tmpQ2[53]*tmpFx[45] + tmpQ2[54]*tmpFx[54] + tmpQ2[55]*tmpFx[63] + tmpQ2[56]*tmpFx[72] + tmpQ2[57]*tmpFx[81] + tmpQ2[58]*tmpFx[90] + tmpQ2[59]*tmpFx[99];
tmpQ1[37] = + tmpQ2[48]*tmpFx[1] + tmpQ2[49]*tmpFx[10] + tmpQ2[50]*tmpFx[19] + tmpQ2[51]*tmpFx[28] + tmpQ2[52]*tmpFx[37] + tmpQ2[53]*tmpFx[46] + tmpQ2[54]*tmpFx[55] + tmpQ2[55]*tmpFx[64] + tmpQ2[56]*tmpFx[73] + tmpQ2[57]*tmpFx[82] + tmpQ2[58]*tmpFx[91] + tmpQ2[59]*tmpFx[100];
tmpQ1[38] = + tmpQ2[48]*tmpFx[2] + tmpQ2[49]*tmpFx[11] + tmpQ2[50]*tmpFx[20] + tmpQ2[51]*tmpFx[29] + tmpQ2[52]*tmpFx[38] + tmpQ2[53]*tmpFx[47] + tmpQ2[54]*tmpFx[56] + tmpQ2[55]*tmpFx[65] + tmpQ2[56]*tmpFx[74] + tmpQ2[57]*tmpFx[83] + tmpQ2[58]*tmpFx[92] + tmpQ2[59]*tmpFx[101];
tmpQ1[39] = + tmpQ2[48]*tmpFx[3] + tmpQ2[49]*tmpFx[12] + tmpQ2[50]*tmpFx[21] + tmpQ2[51]*tmpFx[30] + tmpQ2[52]*tmpFx[39] + tmpQ2[53]*tmpFx[48] + tmpQ2[54]*tmpFx[57] + tmpQ2[55]*tmpFx[66] + tmpQ2[56]*tmpFx[75] + tmpQ2[57]*tmpFx[84] + tmpQ2[58]*tmpFx[93] + tmpQ2[59]*tmpFx[102];
tmpQ1[40] = + tmpQ2[48]*tmpFx[4] + tmpQ2[49]*tmpFx[13] + tmpQ2[50]*tmpFx[22] + tmpQ2[51]*tmpFx[31] + tmpQ2[52]*tmpFx[40] + tmpQ2[53]*tmpFx[49] + tmpQ2[54]*tmpFx[58] + tmpQ2[55]*tmpFx[67] + tmpQ2[56]*tmpFx[76] + tmpQ2[57]*tmpFx[85] + tmpQ2[58]*tmpFx[94] + tmpQ2[59]*tmpFx[103];
tmpQ1[41] = + tmpQ2[48]*tmpFx[5] + tmpQ2[49]*tmpFx[14] + tmpQ2[50]*tmpFx[23] + tmpQ2[51]*tmpFx[32] + tmpQ2[52]*tmpFx[41] + tmpQ2[53]*tmpFx[50] + tmpQ2[54]*tmpFx[59] + tmpQ2[55]*tmpFx[68] + tmpQ2[56]*tmpFx[77] + tmpQ2[57]*tmpFx[86] + tmpQ2[58]*tmpFx[95] + tmpQ2[59]*tmpFx[104];
tmpQ1[42] = + tmpQ2[48]*tmpFx[6] + tmpQ2[49]*tmpFx[15] + tmpQ2[50]*tmpFx[24] + tmpQ2[51]*tmpFx[33] + tmpQ2[52]*tmpFx[42] + tmpQ2[53]*tmpFx[51] + tmpQ2[54]*tmpFx[60] + tmpQ2[55]*tmpFx[69] + tmpQ2[56]*tmpFx[78] + tmpQ2[57]*tmpFx[87] + tmpQ2[58]*tmpFx[96] + tmpQ2[59]*tmpFx[105];
tmpQ1[43] = + tmpQ2[48]*tmpFx[7] + tmpQ2[49]*tmpFx[16] + tmpQ2[50]*tmpFx[25] + tmpQ2[51]*tmpFx[34] + tmpQ2[52]*tmpFx[43] + tmpQ2[53]*tmpFx[52] + tmpQ2[54]*tmpFx[61] + tmpQ2[55]*tmpFx[70] + tmpQ2[56]*tmpFx[79] + tmpQ2[57]*tmpFx[88] + tmpQ2[58]*tmpFx[97] + tmpQ2[59]*tmpFx[106];
tmpQ1[44] = + tmpQ2[48]*tmpFx[8] + tmpQ2[49]*tmpFx[17] + tmpQ2[50]*tmpFx[26] + tmpQ2[51]*tmpFx[35] + tmpQ2[52]*tmpFx[44] + tmpQ2[53]*tmpFx[53] + tmpQ2[54]*tmpFx[62] + tmpQ2[55]*tmpFx[71] + tmpQ2[56]*tmpFx[80] + tmpQ2[57]*tmpFx[89] + tmpQ2[58]*tmpFx[98] + tmpQ2[59]*tmpFx[107];
tmpQ1[45] = + tmpQ2[60]*tmpFx[0] + tmpQ2[61]*tmpFx[9] + tmpQ2[62]*tmpFx[18] + tmpQ2[63]*tmpFx[27] + tmpQ2[64]*tmpFx[36] + tmpQ2[65]*tmpFx[45] + tmpQ2[66]*tmpFx[54] + tmpQ2[67]*tmpFx[63] + tmpQ2[68]*tmpFx[72] + tmpQ2[69]*tmpFx[81] + tmpQ2[70]*tmpFx[90] + tmpQ2[71]*tmpFx[99];
tmpQ1[46] = + tmpQ2[60]*tmpFx[1] + tmpQ2[61]*tmpFx[10] + tmpQ2[62]*tmpFx[19] + tmpQ2[63]*tmpFx[28] + tmpQ2[64]*tmpFx[37] + tmpQ2[65]*tmpFx[46] + tmpQ2[66]*tmpFx[55] + tmpQ2[67]*tmpFx[64] + tmpQ2[68]*tmpFx[73] + tmpQ2[69]*tmpFx[82] + tmpQ2[70]*tmpFx[91] + tmpQ2[71]*tmpFx[100];
tmpQ1[47] = + tmpQ2[60]*tmpFx[2] + tmpQ2[61]*tmpFx[11] + tmpQ2[62]*tmpFx[20] + tmpQ2[63]*tmpFx[29] + tmpQ2[64]*tmpFx[38] + tmpQ2[65]*tmpFx[47] + tmpQ2[66]*tmpFx[56] + tmpQ2[67]*tmpFx[65] + tmpQ2[68]*tmpFx[74] + tmpQ2[69]*tmpFx[83] + tmpQ2[70]*tmpFx[92] + tmpQ2[71]*tmpFx[101];
tmpQ1[48] = + tmpQ2[60]*tmpFx[3] + tmpQ2[61]*tmpFx[12] + tmpQ2[62]*tmpFx[21] + tmpQ2[63]*tmpFx[30] + tmpQ2[64]*tmpFx[39] + tmpQ2[65]*tmpFx[48] + tmpQ2[66]*tmpFx[57] + tmpQ2[67]*tmpFx[66] + tmpQ2[68]*tmpFx[75] + tmpQ2[69]*tmpFx[84] + tmpQ2[70]*tmpFx[93] + tmpQ2[71]*tmpFx[102];
tmpQ1[49] = + tmpQ2[60]*tmpFx[4] + tmpQ2[61]*tmpFx[13] + tmpQ2[62]*tmpFx[22] + tmpQ2[63]*tmpFx[31] + tmpQ2[64]*tmpFx[40] + tmpQ2[65]*tmpFx[49] + tmpQ2[66]*tmpFx[58] + tmpQ2[67]*tmpFx[67] + tmpQ2[68]*tmpFx[76] + tmpQ2[69]*tmpFx[85] + tmpQ2[70]*tmpFx[94] + tmpQ2[71]*tmpFx[103];
tmpQ1[50] = + tmpQ2[60]*tmpFx[5] + tmpQ2[61]*tmpFx[14] + tmpQ2[62]*tmpFx[23] + tmpQ2[63]*tmpFx[32] + tmpQ2[64]*tmpFx[41] + tmpQ2[65]*tmpFx[50] + tmpQ2[66]*tmpFx[59] + tmpQ2[67]*tmpFx[68] + tmpQ2[68]*tmpFx[77] + tmpQ2[69]*tmpFx[86] + tmpQ2[70]*tmpFx[95] + tmpQ2[71]*tmpFx[104];
tmpQ1[51] = + tmpQ2[60]*tmpFx[6] + tmpQ2[61]*tmpFx[15] + tmpQ2[62]*tmpFx[24] + tmpQ2[63]*tmpFx[33] + tmpQ2[64]*tmpFx[42] + tmpQ2[65]*tmpFx[51] + tmpQ2[66]*tmpFx[60] + tmpQ2[67]*tmpFx[69] + tmpQ2[68]*tmpFx[78] + tmpQ2[69]*tmpFx[87] + tmpQ2[70]*tmpFx[96] + tmpQ2[71]*tmpFx[105];
tmpQ1[52] = + tmpQ2[60]*tmpFx[7] + tmpQ2[61]*tmpFx[16] + tmpQ2[62]*tmpFx[25] + tmpQ2[63]*tmpFx[34] + tmpQ2[64]*tmpFx[43] + tmpQ2[65]*tmpFx[52] + tmpQ2[66]*tmpFx[61] + tmpQ2[67]*tmpFx[70] + tmpQ2[68]*tmpFx[79] + tmpQ2[69]*tmpFx[88] + tmpQ2[70]*tmpFx[97] + tmpQ2[71]*tmpFx[106];
tmpQ1[53] = + tmpQ2[60]*tmpFx[8] + tmpQ2[61]*tmpFx[17] + tmpQ2[62]*tmpFx[26] + tmpQ2[63]*tmpFx[35] + tmpQ2[64]*tmpFx[44] + tmpQ2[65]*tmpFx[53] + tmpQ2[66]*tmpFx[62] + tmpQ2[67]*tmpFx[71] + tmpQ2[68]*tmpFx[80] + tmpQ2[69]*tmpFx[89] + tmpQ2[70]*tmpFx[98] + tmpQ2[71]*tmpFx[107];
tmpQ1[54] = + tmpQ2[72]*tmpFx[0] + tmpQ2[73]*tmpFx[9] + tmpQ2[74]*tmpFx[18] + tmpQ2[75]*tmpFx[27] + tmpQ2[76]*tmpFx[36] + tmpQ2[77]*tmpFx[45] + tmpQ2[78]*tmpFx[54] + tmpQ2[79]*tmpFx[63] + tmpQ2[80]*tmpFx[72] + tmpQ2[81]*tmpFx[81] + tmpQ2[82]*tmpFx[90] + tmpQ2[83]*tmpFx[99];
tmpQ1[55] = + tmpQ2[72]*tmpFx[1] + tmpQ2[73]*tmpFx[10] + tmpQ2[74]*tmpFx[19] + tmpQ2[75]*tmpFx[28] + tmpQ2[76]*tmpFx[37] + tmpQ2[77]*tmpFx[46] + tmpQ2[78]*tmpFx[55] + tmpQ2[79]*tmpFx[64] + tmpQ2[80]*tmpFx[73] + tmpQ2[81]*tmpFx[82] + tmpQ2[82]*tmpFx[91] + tmpQ2[83]*tmpFx[100];
tmpQ1[56] = + tmpQ2[72]*tmpFx[2] + tmpQ2[73]*tmpFx[11] + tmpQ2[74]*tmpFx[20] + tmpQ2[75]*tmpFx[29] + tmpQ2[76]*tmpFx[38] + tmpQ2[77]*tmpFx[47] + tmpQ2[78]*tmpFx[56] + tmpQ2[79]*tmpFx[65] + tmpQ2[80]*tmpFx[74] + tmpQ2[81]*tmpFx[83] + tmpQ2[82]*tmpFx[92] + tmpQ2[83]*tmpFx[101];
tmpQ1[57] = + tmpQ2[72]*tmpFx[3] + tmpQ2[73]*tmpFx[12] + tmpQ2[74]*tmpFx[21] + tmpQ2[75]*tmpFx[30] + tmpQ2[76]*tmpFx[39] + tmpQ2[77]*tmpFx[48] + tmpQ2[78]*tmpFx[57] + tmpQ2[79]*tmpFx[66] + tmpQ2[80]*tmpFx[75] + tmpQ2[81]*tmpFx[84] + tmpQ2[82]*tmpFx[93] + tmpQ2[83]*tmpFx[102];
tmpQ1[58] = + tmpQ2[72]*tmpFx[4] + tmpQ2[73]*tmpFx[13] + tmpQ2[74]*tmpFx[22] + tmpQ2[75]*tmpFx[31] + tmpQ2[76]*tmpFx[40] + tmpQ2[77]*tmpFx[49] + tmpQ2[78]*tmpFx[58] + tmpQ2[79]*tmpFx[67] + tmpQ2[80]*tmpFx[76] + tmpQ2[81]*tmpFx[85] + tmpQ2[82]*tmpFx[94] + tmpQ2[83]*tmpFx[103];
tmpQ1[59] = + tmpQ2[72]*tmpFx[5] + tmpQ2[73]*tmpFx[14] + tmpQ2[74]*tmpFx[23] + tmpQ2[75]*tmpFx[32] + tmpQ2[76]*tmpFx[41] + tmpQ2[77]*tmpFx[50] + tmpQ2[78]*tmpFx[59] + tmpQ2[79]*tmpFx[68] + tmpQ2[80]*tmpFx[77] + tmpQ2[81]*tmpFx[86] + tmpQ2[82]*tmpFx[95] + tmpQ2[83]*tmpFx[104];
tmpQ1[60] = + tmpQ2[72]*tmpFx[6] + tmpQ2[73]*tmpFx[15] + tmpQ2[74]*tmpFx[24] + tmpQ2[75]*tmpFx[33] + tmpQ2[76]*tmpFx[42] + tmpQ2[77]*tmpFx[51] + tmpQ2[78]*tmpFx[60] + tmpQ2[79]*tmpFx[69] + tmpQ2[80]*tmpFx[78] + tmpQ2[81]*tmpFx[87] + tmpQ2[82]*tmpFx[96] + tmpQ2[83]*tmpFx[105];
tmpQ1[61] = + tmpQ2[72]*tmpFx[7] + tmpQ2[73]*tmpFx[16] + tmpQ2[74]*tmpFx[25] + tmpQ2[75]*tmpFx[34] + tmpQ2[76]*tmpFx[43] + tmpQ2[77]*tmpFx[52] + tmpQ2[78]*tmpFx[61] + tmpQ2[79]*tmpFx[70] + tmpQ2[80]*tmpFx[79] + tmpQ2[81]*tmpFx[88] + tmpQ2[82]*tmpFx[97] + tmpQ2[83]*tmpFx[106];
tmpQ1[62] = + tmpQ2[72]*tmpFx[8] + tmpQ2[73]*tmpFx[17] + tmpQ2[74]*tmpFx[26] + tmpQ2[75]*tmpFx[35] + tmpQ2[76]*tmpFx[44] + tmpQ2[77]*tmpFx[53] + tmpQ2[78]*tmpFx[62] + tmpQ2[79]*tmpFx[71] + tmpQ2[80]*tmpFx[80] + tmpQ2[81]*tmpFx[89] + tmpQ2[82]*tmpFx[98] + tmpQ2[83]*tmpFx[107];
tmpQ1[63] = + tmpQ2[84]*tmpFx[0] + tmpQ2[85]*tmpFx[9] + tmpQ2[86]*tmpFx[18] + tmpQ2[87]*tmpFx[27] + tmpQ2[88]*tmpFx[36] + tmpQ2[89]*tmpFx[45] + tmpQ2[90]*tmpFx[54] + tmpQ2[91]*tmpFx[63] + tmpQ2[92]*tmpFx[72] + tmpQ2[93]*tmpFx[81] + tmpQ2[94]*tmpFx[90] + tmpQ2[95]*tmpFx[99];
tmpQ1[64] = + tmpQ2[84]*tmpFx[1] + tmpQ2[85]*tmpFx[10] + tmpQ2[86]*tmpFx[19] + tmpQ2[87]*tmpFx[28] + tmpQ2[88]*tmpFx[37] + tmpQ2[89]*tmpFx[46] + tmpQ2[90]*tmpFx[55] + tmpQ2[91]*tmpFx[64] + tmpQ2[92]*tmpFx[73] + tmpQ2[93]*tmpFx[82] + tmpQ2[94]*tmpFx[91] + tmpQ2[95]*tmpFx[100];
tmpQ1[65] = + tmpQ2[84]*tmpFx[2] + tmpQ2[85]*tmpFx[11] + tmpQ2[86]*tmpFx[20] + tmpQ2[87]*tmpFx[29] + tmpQ2[88]*tmpFx[38] + tmpQ2[89]*tmpFx[47] + tmpQ2[90]*tmpFx[56] + tmpQ2[91]*tmpFx[65] + tmpQ2[92]*tmpFx[74] + tmpQ2[93]*tmpFx[83] + tmpQ2[94]*tmpFx[92] + tmpQ2[95]*tmpFx[101];
tmpQ1[66] = + tmpQ2[84]*tmpFx[3] + tmpQ2[85]*tmpFx[12] + tmpQ2[86]*tmpFx[21] + tmpQ2[87]*tmpFx[30] + tmpQ2[88]*tmpFx[39] + tmpQ2[89]*tmpFx[48] + tmpQ2[90]*tmpFx[57] + tmpQ2[91]*tmpFx[66] + tmpQ2[92]*tmpFx[75] + tmpQ2[93]*tmpFx[84] + tmpQ2[94]*tmpFx[93] + tmpQ2[95]*tmpFx[102];
tmpQ1[67] = + tmpQ2[84]*tmpFx[4] + tmpQ2[85]*tmpFx[13] + tmpQ2[86]*tmpFx[22] + tmpQ2[87]*tmpFx[31] + tmpQ2[88]*tmpFx[40] + tmpQ2[89]*tmpFx[49] + tmpQ2[90]*tmpFx[58] + tmpQ2[91]*tmpFx[67] + tmpQ2[92]*tmpFx[76] + tmpQ2[93]*tmpFx[85] + tmpQ2[94]*tmpFx[94] + tmpQ2[95]*tmpFx[103];
tmpQ1[68] = + tmpQ2[84]*tmpFx[5] + tmpQ2[85]*tmpFx[14] + tmpQ2[86]*tmpFx[23] + tmpQ2[87]*tmpFx[32] + tmpQ2[88]*tmpFx[41] + tmpQ2[89]*tmpFx[50] + tmpQ2[90]*tmpFx[59] + tmpQ2[91]*tmpFx[68] + tmpQ2[92]*tmpFx[77] + tmpQ2[93]*tmpFx[86] + tmpQ2[94]*tmpFx[95] + tmpQ2[95]*tmpFx[104];
tmpQ1[69] = + tmpQ2[84]*tmpFx[6] + tmpQ2[85]*tmpFx[15] + tmpQ2[86]*tmpFx[24] + tmpQ2[87]*tmpFx[33] + tmpQ2[88]*tmpFx[42] + tmpQ2[89]*tmpFx[51] + tmpQ2[90]*tmpFx[60] + tmpQ2[91]*tmpFx[69] + tmpQ2[92]*tmpFx[78] + tmpQ2[93]*tmpFx[87] + tmpQ2[94]*tmpFx[96] + tmpQ2[95]*tmpFx[105];
tmpQ1[70] = + tmpQ2[84]*tmpFx[7] + tmpQ2[85]*tmpFx[16] + tmpQ2[86]*tmpFx[25] + tmpQ2[87]*tmpFx[34] + tmpQ2[88]*tmpFx[43] + tmpQ2[89]*tmpFx[52] + tmpQ2[90]*tmpFx[61] + tmpQ2[91]*tmpFx[70] + tmpQ2[92]*tmpFx[79] + tmpQ2[93]*tmpFx[88] + tmpQ2[94]*tmpFx[97] + tmpQ2[95]*tmpFx[106];
tmpQ1[71] = + tmpQ2[84]*tmpFx[8] + tmpQ2[85]*tmpFx[17] + tmpQ2[86]*tmpFx[26] + tmpQ2[87]*tmpFx[35] + tmpQ2[88]*tmpFx[44] + tmpQ2[89]*tmpFx[53] + tmpQ2[90]*tmpFx[62] + tmpQ2[91]*tmpFx[71] + tmpQ2[92]*tmpFx[80] + tmpQ2[93]*tmpFx[89] + tmpQ2[94]*tmpFx[98] + tmpQ2[95]*tmpFx[107];
tmpQ1[72] = + tmpQ2[96]*tmpFx[0] + tmpQ2[97]*tmpFx[9] + tmpQ2[98]*tmpFx[18] + tmpQ2[99]*tmpFx[27] + tmpQ2[100]*tmpFx[36] + tmpQ2[101]*tmpFx[45] + tmpQ2[102]*tmpFx[54] + tmpQ2[103]*tmpFx[63] + tmpQ2[104]*tmpFx[72] + tmpQ2[105]*tmpFx[81] + tmpQ2[106]*tmpFx[90] + tmpQ2[107]*tmpFx[99];
tmpQ1[73] = + tmpQ2[96]*tmpFx[1] + tmpQ2[97]*tmpFx[10] + tmpQ2[98]*tmpFx[19] + tmpQ2[99]*tmpFx[28] + tmpQ2[100]*tmpFx[37] + tmpQ2[101]*tmpFx[46] + tmpQ2[102]*tmpFx[55] + tmpQ2[103]*tmpFx[64] + tmpQ2[104]*tmpFx[73] + tmpQ2[105]*tmpFx[82] + tmpQ2[106]*tmpFx[91] + tmpQ2[107]*tmpFx[100];
tmpQ1[74] = + tmpQ2[96]*tmpFx[2] + tmpQ2[97]*tmpFx[11] + tmpQ2[98]*tmpFx[20] + tmpQ2[99]*tmpFx[29] + tmpQ2[100]*tmpFx[38] + tmpQ2[101]*tmpFx[47] + tmpQ2[102]*tmpFx[56] + tmpQ2[103]*tmpFx[65] + tmpQ2[104]*tmpFx[74] + tmpQ2[105]*tmpFx[83] + tmpQ2[106]*tmpFx[92] + tmpQ2[107]*tmpFx[101];
tmpQ1[75] = + tmpQ2[96]*tmpFx[3] + tmpQ2[97]*tmpFx[12] + tmpQ2[98]*tmpFx[21] + tmpQ2[99]*tmpFx[30] + tmpQ2[100]*tmpFx[39] + tmpQ2[101]*tmpFx[48] + tmpQ2[102]*tmpFx[57] + tmpQ2[103]*tmpFx[66] + tmpQ2[104]*tmpFx[75] + tmpQ2[105]*tmpFx[84] + tmpQ2[106]*tmpFx[93] + tmpQ2[107]*tmpFx[102];
tmpQ1[76] = + tmpQ2[96]*tmpFx[4] + tmpQ2[97]*tmpFx[13] + tmpQ2[98]*tmpFx[22] + tmpQ2[99]*tmpFx[31] + tmpQ2[100]*tmpFx[40] + tmpQ2[101]*tmpFx[49] + tmpQ2[102]*tmpFx[58] + tmpQ2[103]*tmpFx[67] + tmpQ2[104]*tmpFx[76] + tmpQ2[105]*tmpFx[85] + tmpQ2[106]*tmpFx[94] + tmpQ2[107]*tmpFx[103];
tmpQ1[77] = + tmpQ2[96]*tmpFx[5] + tmpQ2[97]*tmpFx[14] + tmpQ2[98]*tmpFx[23] + tmpQ2[99]*tmpFx[32] + tmpQ2[100]*tmpFx[41] + tmpQ2[101]*tmpFx[50] + tmpQ2[102]*tmpFx[59] + tmpQ2[103]*tmpFx[68] + tmpQ2[104]*tmpFx[77] + tmpQ2[105]*tmpFx[86] + tmpQ2[106]*tmpFx[95] + tmpQ2[107]*tmpFx[104];
tmpQ1[78] = + tmpQ2[96]*tmpFx[6] + tmpQ2[97]*tmpFx[15] + tmpQ2[98]*tmpFx[24] + tmpQ2[99]*tmpFx[33] + tmpQ2[100]*tmpFx[42] + tmpQ2[101]*tmpFx[51] + tmpQ2[102]*tmpFx[60] + tmpQ2[103]*tmpFx[69] + tmpQ2[104]*tmpFx[78] + tmpQ2[105]*tmpFx[87] + tmpQ2[106]*tmpFx[96] + tmpQ2[107]*tmpFx[105];
tmpQ1[79] = + tmpQ2[96]*tmpFx[7] + tmpQ2[97]*tmpFx[16] + tmpQ2[98]*tmpFx[25] + tmpQ2[99]*tmpFx[34] + tmpQ2[100]*tmpFx[43] + tmpQ2[101]*tmpFx[52] + tmpQ2[102]*tmpFx[61] + tmpQ2[103]*tmpFx[70] + tmpQ2[104]*tmpFx[79] + tmpQ2[105]*tmpFx[88] + tmpQ2[106]*tmpFx[97] + tmpQ2[107]*tmpFx[106];
tmpQ1[80] = + tmpQ2[96]*tmpFx[8] + tmpQ2[97]*tmpFx[17] + tmpQ2[98]*tmpFx[26] + tmpQ2[99]*tmpFx[35] + tmpQ2[100]*tmpFx[44] + tmpQ2[101]*tmpFx[53] + tmpQ2[102]*tmpFx[62] + tmpQ2[103]*tmpFx[71] + tmpQ2[104]*tmpFx[80] + tmpQ2[105]*tmpFx[89] + tmpQ2[106]*tmpFx[98] + tmpQ2[107]*tmpFx[107];
}

void acado_setObjR1R2( real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*tmpObjS[0] + tmpFu[3]*tmpObjS[12] + tmpFu[6]*tmpObjS[24] + tmpFu[9]*tmpObjS[36] + tmpFu[12]*tmpObjS[48] + tmpFu[15]*tmpObjS[60] + tmpFu[18]*tmpObjS[72] + tmpFu[21]*tmpObjS[84] + tmpFu[24]*tmpObjS[96] + tmpFu[27]*tmpObjS[108] + tmpFu[30]*tmpObjS[120] + tmpFu[33]*tmpObjS[132];
tmpR2[1] = + tmpFu[0]*tmpObjS[1] + tmpFu[3]*tmpObjS[13] + tmpFu[6]*tmpObjS[25] + tmpFu[9]*tmpObjS[37] + tmpFu[12]*tmpObjS[49] + tmpFu[15]*tmpObjS[61] + tmpFu[18]*tmpObjS[73] + tmpFu[21]*tmpObjS[85] + tmpFu[24]*tmpObjS[97] + tmpFu[27]*tmpObjS[109] + tmpFu[30]*tmpObjS[121] + tmpFu[33]*tmpObjS[133];
tmpR2[2] = + tmpFu[0]*tmpObjS[2] + tmpFu[3]*tmpObjS[14] + tmpFu[6]*tmpObjS[26] + tmpFu[9]*tmpObjS[38] + tmpFu[12]*tmpObjS[50] + tmpFu[15]*tmpObjS[62] + tmpFu[18]*tmpObjS[74] + tmpFu[21]*tmpObjS[86] + tmpFu[24]*tmpObjS[98] + tmpFu[27]*tmpObjS[110] + tmpFu[30]*tmpObjS[122] + tmpFu[33]*tmpObjS[134];
tmpR2[3] = + tmpFu[0]*tmpObjS[3] + tmpFu[3]*tmpObjS[15] + tmpFu[6]*tmpObjS[27] + tmpFu[9]*tmpObjS[39] + tmpFu[12]*tmpObjS[51] + tmpFu[15]*tmpObjS[63] + tmpFu[18]*tmpObjS[75] + tmpFu[21]*tmpObjS[87] + tmpFu[24]*tmpObjS[99] + tmpFu[27]*tmpObjS[111] + tmpFu[30]*tmpObjS[123] + tmpFu[33]*tmpObjS[135];
tmpR2[4] = + tmpFu[0]*tmpObjS[4] + tmpFu[3]*tmpObjS[16] + tmpFu[6]*tmpObjS[28] + tmpFu[9]*tmpObjS[40] + tmpFu[12]*tmpObjS[52] + tmpFu[15]*tmpObjS[64] + tmpFu[18]*tmpObjS[76] + tmpFu[21]*tmpObjS[88] + tmpFu[24]*tmpObjS[100] + tmpFu[27]*tmpObjS[112] + tmpFu[30]*tmpObjS[124] + tmpFu[33]*tmpObjS[136];
tmpR2[5] = + tmpFu[0]*tmpObjS[5] + tmpFu[3]*tmpObjS[17] + tmpFu[6]*tmpObjS[29] + tmpFu[9]*tmpObjS[41] + tmpFu[12]*tmpObjS[53] + tmpFu[15]*tmpObjS[65] + tmpFu[18]*tmpObjS[77] + tmpFu[21]*tmpObjS[89] + tmpFu[24]*tmpObjS[101] + tmpFu[27]*tmpObjS[113] + tmpFu[30]*tmpObjS[125] + tmpFu[33]*tmpObjS[137];
tmpR2[6] = + tmpFu[0]*tmpObjS[6] + tmpFu[3]*tmpObjS[18] + tmpFu[6]*tmpObjS[30] + tmpFu[9]*tmpObjS[42] + tmpFu[12]*tmpObjS[54] + tmpFu[15]*tmpObjS[66] + tmpFu[18]*tmpObjS[78] + tmpFu[21]*tmpObjS[90] + tmpFu[24]*tmpObjS[102] + tmpFu[27]*tmpObjS[114] + tmpFu[30]*tmpObjS[126] + tmpFu[33]*tmpObjS[138];
tmpR2[7] = + tmpFu[0]*tmpObjS[7] + tmpFu[3]*tmpObjS[19] + tmpFu[6]*tmpObjS[31] + tmpFu[9]*tmpObjS[43] + tmpFu[12]*tmpObjS[55] + tmpFu[15]*tmpObjS[67] + tmpFu[18]*tmpObjS[79] + tmpFu[21]*tmpObjS[91] + tmpFu[24]*tmpObjS[103] + tmpFu[27]*tmpObjS[115] + tmpFu[30]*tmpObjS[127] + tmpFu[33]*tmpObjS[139];
tmpR2[8] = + tmpFu[0]*tmpObjS[8] + tmpFu[3]*tmpObjS[20] + tmpFu[6]*tmpObjS[32] + tmpFu[9]*tmpObjS[44] + tmpFu[12]*tmpObjS[56] + tmpFu[15]*tmpObjS[68] + tmpFu[18]*tmpObjS[80] + tmpFu[21]*tmpObjS[92] + tmpFu[24]*tmpObjS[104] + tmpFu[27]*tmpObjS[116] + tmpFu[30]*tmpObjS[128] + tmpFu[33]*tmpObjS[140];
tmpR2[9] = + tmpFu[0]*tmpObjS[9] + tmpFu[3]*tmpObjS[21] + tmpFu[6]*tmpObjS[33] + tmpFu[9]*tmpObjS[45] + tmpFu[12]*tmpObjS[57] + tmpFu[15]*tmpObjS[69] + tmpFu[18]*tmpObjS[81] + tmpFu[21]*tmpObjS[93] + tmpFu[24]*tmpObjS[105] + tmpFu[27]*tmpObjS[117] + tmpFu[30]*tmpObjS[129] + tmpFu[33]*tmpObjS[141];
tmpR2[10] = + tmpFu[0]*tmpObjS[10] + tmpFu[3]*tmpObjS[22] + tmpFu[6]*tmpObjS[34] + tmpFu[9]*tmpObjS[46] + tmpFu[12]*tmpObjS[58] + tmpFu[15]*tmpObjS[70] + tmpFu[18]*tmpObjS[82] + tmpFu[21]*tmpObjS[94] + tmpFu[24]*tmpObjS[106] + tmpFu[27]*tmpObjS[118] + tmpFu[30]*tmpObjS[130] + tmpFu[33]*tmpObjS[142];
tmpR2[11] = + tmpFu[0]*tmpObjS[11] + tmpFu[3]*tmpObjS[23] + tmpFu[6]*tmpObjS[35] + tmpFu[9]*tmpObjS[47] + tmpFu[12]*tmpObjS[59] + tmpFu[15]*tmpObjS[71] + tmpFu[18]*tmpObjS[83] + tmpFu[21]*tmpObjS[95] + tmpFu[24]*tmpObjS[107] + tmpFu[27]*tmpObjS[119] + tmpFu[30]*tmpObjS[131] + tmpFu[33]*tmpObjS[143];
tmpR2[12] = + tmpFu[1]*tmpObjS[0] + tmpFu[4]*tmpObjS[12] + tmpFu[7]*tmpObjS[24] + tmpFu[10]*tmpObjS[36] + tmpFu[13]*tmpObjS[48] + tmpFu[16]*tmpObjS[60] + tmpFu[19]*tmpObjS[72] + tmpFu[22]*tmpObjS[84] + tmpFu[25]*tmpObjS[96] + tmpFu[28]*tmpObjS[108] + tmpFu[31]*tmpObjS[120] + tmpFu[34]*tmpObjS[132];
tmpR2[13] = + tmpFu[1]*tmpObjS[1] + tmpFu[4]*tmpObjS[13] + tmpFu[7]*tmpObjS[25] + tmpFu[10]*tmpObjS[37] + tmpFu[13]*tmpObjS[49] + tmpFu[16]*tmpObjS[61] + tmpFu[19]*tmpObjS[73] + tmpFu[22]*tmpObjS[85] + tmpFu[25]*tmpObjS[97] + tmpFu[28]*tmpObjS[109] + tmpFu[31]*tmpObjS[121] + tmpFu[34]*tmpObjS[133];
tmpR2[14] = + tmpFu[1]*tmpObjS[2] + tmpFu[4]*tmpObjS[14] + tmpFu[7]*tmpObjS[26] + tmpFu[10]*tmpObjS[38] + tmpFu[13]*tmpObjS[50] + tmpFu[16]*tmpObjS[62] + tmpFu[19]*tmpObjS[74] + tmpFu[22]*tmpObjS[86] + tmpFu[25]*tmpObjS[98] + tmpFu[28]*tmpObjS[110] + tmpFu[31]*tmpObjS[122] + tmpFu[34]*tmpObjS[134];
tmpR2[15] = + tmpFu[1]*tmpObjS[3] + tmpFu[4]*tmpObjS[15] + tmpFu[7]*tmpObjS[27] + tmpFu[10]*tmpObjS[39] + tmpFu[13]*tmpObjS[51] + tmpFu[16]*tmpObjS[63] + tmpFu[19]*tmpObjS[75] + tmpFu[22]*tmpObjS[87] + tmpFu[25]*tmpObjS[99] + tmpFu[28]*tmpObjS[111] + tmpFu[31]*tmpObjS[123] + tmpFu[34]*tmpObjS[135];
tmpR2[16] = + tmpFu[1]*tmpObjS[4] + tmpFu[4]*tmpObjS[16] + tmpFu[7]*tmpObjS[28] + tmpFu[10]*tmpObjS[40] + tmpFu[13]*tmpObjS[52] + tmpFu[16]*tmpObjS[64] + tmpFu[19]*tmpObjS[76] + tmpFu[22]*tmpObjS[88] + tmpFu[25]*tmpObjS[100] + tmpFu[28]*tmpObjS[112] + tmpFu[31]*tmpObjS[124] + tmpFu[34]*tmpObjS[136];
tmpR2[17] = + tmpFu[1]*tmpObjS[5] + tmpFu[4]*tmpObjS[17] + tmpFu[7]*tmpObjS[29] + tmpFu[10]*tmpObjS[41] + tmpFu[13]*tmpObjS[53] + tmpFu[16]*tmpObjS[65] + tmpFu[19]*tmpObjS[77] + tmpFu[22]*tmpObjS[89] + tmpFu[25]*tmpObjS[101] + tmpFu[28]*tmpObjS[113] + tmpFu[31]*tmpObjS[125] + tmpFu[34]*tmpObjS[137];
tmpR2[18] = + tmpFu[1]*tmpObjS[6] + tmpFu[4]*tmpObjS[18] + tmpFu[7]*tmpObjS[30] + tmpFu[10]*tmpObjS[42] + tmpFu[13]*tmpObjS[54] + tmpFu[16]*tmpObjS[66] + tmpFu[19]*tmpObjS[78] + tmpFu[22]*tmpObjS[90] + tmpFu[25]*tmpObjS[102] + tmpFu[28]*tmpObjS[114] + tmpFu[31]*tmpObjS[126] + tmpFu[34]*tmpObjS[138];
tmpR2[19] = + tmpFu[1]*tmpObjS[7] + tmpFu[4]*tmpObjS[19] + tmpFu[7]*tmpObjS[31] + tmpFu[10]*tmpObjS[43] + tmpFu[13]*tmpObjS[55] + tmpFu[16]*tmpObjS[67] + tmpFu[19]*tmpObjS[79] + tmpFu[22]*tmpObjS[91] + tmpFu[25]*tmpObjS[103] + tmpFu[28]*tmpObjS[115] + tmpFu[31]*tmpObjS[127] + tmpFu[34]*tmpObjS[139];
tmpR2[20] = + tmpFu[1]*tmpObjS[8] + tmpFu[4]*tmpObjS[20] + tmpFu[7]*tmpObjS[32] + tmpFu[10]*tmpObjS[44] + tmpFu[13]*tmpObjS[56] + tmpFu[16]*tmpObjS[68] + tmpFu[19]*tmpObjS[80] + tmpFu[22]*tmpObjS[92] + tmpFu[25]*tmpObjS[104] + tmpFu[28]*tmpObjS[116] + tmpFu[31]*tmpObjS[128] + tmpFu[34]*tmpObjS[140];
tmpR2[21] = + tmpFu[1]*tmpObjS[9] + tmpFu[4]*tmpObjS[21] + tmpFu[7]*tmpObjS[33] + tmpFu[10]*tmpObjS[45] + tmpFu[13]*tmpObjS[57] + tmpFu[16]*tmpObjS[69] + tmpFu[19]*tmpObjS[81] + tmpFu[22]*tmpObjS[93] + tmpFu[25]*tmpObjS[105] + tmpFu[28]*tmpObjS[117] + tmpFu[31]*tmpObjS[129] + tmpFu[34]*tmpObjS[141];
tmpR2[22] = + tmpFu[1]*tmpObjS[10] + tmpFu[4]*tmpObjS[22] + tmpFu[7]*tmpObjS[34] + tmpFu[10]*tmpObjS[46] + tmpFu[13]*tmpObjS[58] + tmpFu[16]*tmpObjS[70] + tmpFu[19]*tmpObjS[82] + tmpFu[22]*tmpObjS[94] + tmpFu[25]*tmpObjS[106] + tmpFu[28]*tmpObjS[118] + tmpFu[31]*tmpObjS[130] + tmpFu[34]*tmpObjS[142];
tmpR2[23] = + tmpFu[1]*tmpObjS[11] + tmpFu[4]*tmpObjS[23] + tmpFu[7]*tmpObjS[35] + tmpFu[10]*tmpObjS[47] + tmpFu[13]*tmpObjS[59] + tmpFu[16]*tmpObjS[71] + tmpFu[19]*tmpObjS[83] + tmpFu[22]*tmpObjS[95] + tmpFu[25]*tmpObjS[107] + tmpFu[28]*tmpObjS[119] + tmpFu[31]*tmpObjS[131] + tmpFu[34]*tmpObjS[143];
tmpR2[24] = + tmpFu[2]*tmpObjS[0] + tmpFu[5]*tmpObjS[12] + tmpFu[8]*tmpObjS[24] + tmpFu[11]*tmpObjS[36] + tmpFu[14]*tmpObjS[48] + tmpFu[17]*tmpObjS[60] + tmpFu[20]*tmpObjS[72] + tmpFu[23]*tmpObjS[84] + tmpFu[26]*tmpObjS[96] + tmpFu[29]*tmpObjS[108] + tmpFu[32]*tmpObjS[120] + tmpFu[35]*tmpObjS[132];
tmpR2[25] = + tmpFu[2]*tmpObjS[1] + tmpFu[5]*tmpObjS[13] + tmpFu[8]*tmpObjS[25] + tmpFu[11]*tmpObjS[37] + tmpFu[14]*tmpObjS[49] + tmpFu[17]*tmpObjS[61] + tmpFu[20]*tmpObjS[73] + tmpFu[23]*tmpObjS[85] + tmpFu[26]*tmpObjS[97] + tmpFu[29]*tmpObjS[109] + tmpFu[32]*tmpObjS[121] + tmpFu[35]*tmpObjS[133];
tmpR2[26] = + tmpFu[2]*tmpObjS[2] + tmpFu[5]*tmpObjS[14] + tmpFu[8]*tmpObjS[26] + tmpFu[11]*tmpObjS[38] + tmpFu[14]*tmpObjS[50] + tmpFu[17]*tmpObjS[62] + tmpFu[20]*tmpObjS[74] + tmpFu[23]*tmpObjS[86] + tmpFu[26]*tmpObjS[98] + tmpFu[29]*tmpObjS[110] + tmpFu[32]*tmpObjS[122] + tmpFu[35]*tmpObjS[134];
tmpR2[27] = + tmpFu[2]*tmpObjS[3] + tmpFu[5]*tmpObjS[15] + tmpFu[8]*tmpObjS[27] + tmpFu[11]*tmpObjS[39] + tmpFu[14]*tmpObjS[51] + tmpFu[17]*tmpObjS[63] + tmpFu[20]*tmpObjS[75] + tmpFu[23]*tmpObjS[87] + tmpFu[26]*tmpObjS[99] + tmpFu[29]*tmpObjS[111] + tmpFu[32]*tmpObjS[123] + tmpFu[35]*tmpObjS[135];
tmpR2[28] = + tmpFu[2]*tmpObjS[4] + tmpFu[5]*tmpObjS[16] + tmpFu[8]*tmpObjS[28] + tmpFu[11]*tmpObjS[40] + tmpFu[14]*tmpObjS[52] + tmpFu[17]*tmpObjS[64] + tmpFu[20]*tmpObjS[76] + tmpFu[23]*tmpObjS[88] + tmpFu[26]*tmpObjS[100] + tmpFu[29]*tmpObjS[112] + tmpFu[32]*tmpObjS[124] + tmpFu[35]*tmpObjS[136];
tmpR2[29] = + tmpFu[2]*tmpObjS[5] + tmpFu[5]*tmpObjS[17] + tmpFu[8]*tmpObjS[29] + tmpFu[11]*tmpObjS[41] + tmpFu[14]*tmpObjS[53] + tmpFu[17]*tmpObjS[65] + tmpFu[20]*tmpObjS[77] + tmpFu[23]*tmpObjS[89] + tmpFu[26]*tmpObjS[101] + tmpFu[29]*tmpObjS[113] + tmpFu[32]*tmpObjS[125] + tmpFu[35]*tmpObjS[137];
tmpR2[30] = + tmpFu[2]*tmpObjS[6] + tmpFu[5]*tmpObjS[18] + tmpFu[8]*tmpObjS[30] + tmpFu[11]*tmpObjS[42] + tmpFu[14]*tmpObjS[54] + tmpFu[17]*tmpObjS[66] + tmpFu[20]*tmpObjS[78] + tmpFu[23]*tmpObjS[90] + tmpFu[26]*tmpObjS[102] + tmpFu[29]*tmpObjS[114] + tmpFu[32]*tmpObjS[126] + tmpFu[35]*tmpObjS[138];
tmpR2[31] = + tmpFu[2]*tmpObjS[7] + tmpFu[5]*tmpObjS[19] + tmpFu[8]*tmpObjS[31] + tmpFu[11]*tmpObjS[43] + tmpFu[14]*tmpObjS[55] + tmpFu[17]*tmpObjS[67] + tmpFu[20]*tmpObjS[79] + tmpFu[23]*tmpObjS[91] + tmpFu[26]*tmpObjS[103] + tmpFu[29]*tmpObjS[115] + tmpFu[32]*tmpObjS[127] + tmpFu[35]*tmpObjS[139];
tmpR2[32] = + tmpFu[2]*tmpObjS[8] + tmpFu[5]*tmpObjS[20] + tmpFu[8]*tmpObjS[32] + tmpFu[11]*tmpObjS[44] + tmpFu[14]*tmpObjS[56] + tmpFu[17]*tmpObjS[68] + tmpFu[20]*tmpObjS[80] + tmpFu[23]*tmpObjS[92] + tmpFu[26]*tmpObjS[104] + tmpFu[29]*tmpObjS[116] + tmpFu[32]*tmpObjS[128] + tmpFu[35]*tmpObjS[140];
tmpR2[33] = + tmpFu[2]*tmpObjS[9] + tmpFu[5]*tmpObjS[21] + tmpFu[8]*tmpObjS[33] + tmpFu[11]*tmpObjS[45] + tmpFu[14]*tmpObjS[57] + tmpFu[17]*tmpObjS[69] + tmpFu[20]*tmpObjS[81] + tmpFu[23]*tmpObjS[93] + tmpFu[26]*tmpObjS[105] + tmpFu[29]*tmpObjS[117] + tmpFu[32]*tmpObjS[129] + tmpFu[35]*tmpObjS[141];
tmpR2[34] = + tmpFu[2]*tmpObjS[10] + tmpFu[5]*tmpObjS[22] + tmpFu[8]*tmpObjS[34] + tmpFu[11]*tmpObjS[46] + tmpFu[14]*tmpObjS[58] + tmpFu[17]*tmpObjS[70] + tmpFu[20]*tmpObjS[82] + tmpFu[23]*tmpObjS[94] + tmpFu[26]*tmpObjS[106] + tmpFu[29]*tmpObjS[118] + tmpFu[32]*tmpObjS[130] + tmpFu[35]*tmpObjS[142];
tmpR2[35] = + tmpFu[2]*tmpObjS[11] + tmpFu[5]*tmpObjS[23] + tmpFu[8]*tmpObjS[35] + tmpFu[11]*tmpObjS[47] + tmpFu[14]*tmpObjS[59] + tmpFu[17]*tmpObjS[71] + tmpFu[20]*tmpObjS[83] + tmpFu[23]*tmpObjS[95] + tmpFu[26]*tmpObjS[107] + tmpFu[29]*tmpObjS[119] + tmpFu[32]*tmpObjS[131] + tmpFu[35]*tmpObjS[143];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[3] + tmpR2[2]*tmpFu[6] + tmpR2[3]*tmpFu[9] + tmpR2[4]*tmpFu[12] + tmpR2[5]*tmpFu[15] + tmpR2[6]*tmpFu[18] + tmpR2[7]*tmpFu[21] + tmpR2[8]*tmpFu[24] + tmpR2[9]*tmpFu[27] + tmpR2[10]*tmpFu[30] + tmpR2[11]*tmpFu[33];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[4] + tmpR2[2]*tmpFu[7] + tmpR2[3]*tmpFu[10] + tmpR2[4]*tmpFu[13] + tmpR2[5]*tmpFu[16] + tmpR2[6]*tmpFu[19] + tmpR2[7]*tmpFu[22] + tmpR2[8]*tmpFu[25] + tmpR2[9]*tmpFu[28] + tmpR2[10]*tmpFu[31] + tmpR2[11]*tmpFu[34];
tmpR1[2] = + tmpR2[0]*tmpFu[2] + tmpR2[1]*tmpFu[5] + tmpR2[2]*tmpFu[8] + tmpR2[3]*tmpFu[11] + tmpR2[4]*tmpFu[14] + tmpR2[5]*tmpFu[17] + tmpR2[6]*tmpFu[20] + tmpR2[7]*tmpFu[23] + tmpR2[8]*tmpFu[26] + tmpR2[9]*tmpFu[29] + tmpR2[10]*tmpFu[32] + tmpR2[11]*tmpFu[35];
tmpR1[3] = + tmpR2[12]*tmpFu[0] + tmpR2[13]*tmpFu[3] + tmpR2[14]*tmpFu[6] + tmpR2[15]*tmpFu[9] + tmpR2[16]*tmpFu[12] + tmpR2[17]*tmpFu[15] + tmpR2[18]*tmpFu[18] + tmpR2[19]*tmpFu[21] + tmpR2[20]*tmpFu[24] + tmpR2[21]*tmpFu[27] + tmpR2[22]*tmpFu[30] + tmpR2[23]*tmpFu[33];
tmpR1[4] = + tmpR2[12]*tmpFu[1] + tmpR2[13]*tmpFu[4] + tmpR2[14]*tmpFu[7] + tmpR2[15]*tmpFu[10] + tmpR2[16]*tmpFu[13] + tmpR2[17]*tmpFu[16] + tmpR2[18]*tmpFu[19] + tmpR2[19]*tmpFu[22] + tmpR2[20]*tmpFu[25] + tmpR2[21]*tmpFu[28] + tmpR2[22]*tmpFu[31] + tmpR2[23]*tmpFu[34];
tmpR1[5] = + tmpR2[12]*tmpFu[2] + tmpR2[13]*tmpFu[5] + tmpR2[14]*tmpFu[8] + tmpR2[15]*tmpFu[11] + tmpR2[16]*tmpFu[14] + tmpR2[17]*tmpFu[17] + tmpR2[18]*tmpFu[20] + tmpR2[19]*tmpFu[23] + tmpR2[20]*tmpFu[26] + tmpR2[21]*tmpFu[29] + tmpR2[22]*tmpFu[32] + tmpR2[23]*tmpFu[35];
tmpR1[6] = + tmpR2[24]*tmpFu[0] + tmpR2[25]*tmpFu[3] + tmpR2[26]*tmpFu[6] + tmpR2[27]*tmpFu[9] + tmpR2[28]*tmpFu[12] + tmpR2[29]*tmpFu[15] + tmpR2[30]*tmpFu[18] + tmpR2[31]*tmpFu[21] + tmpR2[32]*tmpFu[24] + tmpR2[33]*tmpFu[27] + tmpR2[34]*tmpFu[30] + tmpR2[35]*tmpFu[33];
tmpR1[7] = + tmpR2[24]*tmpFu[1] + tmpR2[25]*tmpFu[4] + tmpR2[26]*tmpFu[7] + tmpR2[27]*tmpFu[10] + tmpR2[28]*tmpFu[13] + tmpR2[29]*tmpFu[16] + tmpR2[30]*tmpFu[19] + tmpR2[31]*tmpFu[22] + tmpR2[32]*tmpFu[25] + tmpR2[33]*tmpFu[28] + tmpR2[34]*tmpFu[31] + tmpR2[35]*tmpFu[34];
tmpR1[8] = + tmpR2[24]*tmpFu[2] + tmpR2[25]*tmpFu[5] + tmpR2[26]*tmpFu[8] + tmpR2[27]*tmpFu[11] + tmpR2[28]*tmpFu[14] + tmpR2[29]*tmpFu[17] + tmpR2[30]*tmpFu[20] + tmpR2[31]*tmpFu[23] + tmpR2[32]*tmpFu[26] + tmpR2[33]*tmpFu[29] + tmpR2[34]*tmpFu[32] + tmpR2[35]*tmpFu[35];
}

void acado_setObjS1( real_t* const tmpFx, real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpS1 )
{
/** Matrix of size: 9 x 12 (row major format) */
real_t tmpS2[ 108 ];

tmpS2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[9]*tmpObjS[12] + tmpFx[18]*tmpObjS[24] + tmpFx[27]*tmpObjS[36] + tmpFx[36]*tmpObjS[48] + tmpFx[45]*tmpObjS[60] + tmpFx[54]*tmpObjS[72] + tmpFx[63]*tmpObjS[84] + tmpFx[72]*tmpObjS[96] + tmpFx[81]*tmpObjS[108] + tmpFx[90]*tmpObjS[120] + tmpFx[99]*tmpObjS[132];
tmpS2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[9]*tmpObjS[13] + tmpFx[18]*tmpObjS[25] + tmpFx[27]*tmpObjS[37] + tmpFx[36]*tmpObjS[49] + tmpFx[45]*tmpObjS[61] + tmpFx[54]*tmpObjS[73] + tmpFx[63]*tmpObjS[85] + tmpFx[72]*tmpObjS[97] + tmpFx[81]*tmpObjS[109] + tmpFx[90]*tmpObjS[121] + tmpFx[99]*tmpObjS[133];
tmpS2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[9]*tmpObjS[14] + tmpFx[18]*tmpObjS[26] + tmpFx[27]*tmpObjS[38] + tmpFx[36]*tmpObjS[50] + tmpFx[45]*tmpObjS[62] + tmpFx[54]*tmpObjS[74] + tmpFx[63]*tmpObjS[86] + tmpFx[72]*tmpObjS[98] + tmpFx[81]*tmpObjS[110] + tmpFx[90]*tmpObjS[122] + tmpFx[99]*tmpObjS[134];
tmpS2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[9]*tmpObjS[15] + tmpFx[18]*tmpObjS[27] + tmpFx[27]*tmpObjS[39] + tmpFx[36]*tmpObjS[51] + tmpFx[45]*tmpObjS[63] + tmpFx[54]*tmpObjS[75] + tmpFx[63]*tmpObjS[87] + tmpFx[72]*tmpObjS[99] + tmpFx[81]*tmpObjS[111] + tmpFx[90]*tmpObjS[123] + tmpFx[99]*tmpObjS[135];
tmpS2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[9]*tmpObjS[16] + tmpFx[18]*tmpObjS[28] + tmpFx[27]*tmpObjS[40] + tmpFx[36]*tmpObjS[52] + tmpFx[45]*tmpObjS[64] + tmpFx[54]*tmpObjS[76] + tmpFx[63]*tmpObjS[88] + tmpFx[72]*tmpObjS[100] + tmpFx[81]*tmpObjS[112] + tmpFx[90]*tmpObjS[124] + tmpFx[99]*tmpObjS[136];
tmpS2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[9]*tmpObjS[17] + tmpFx[18]*tmpObjS[29] + tmpFx[27]*tmpObjS[41] + tmpFx[36]*tmpObjS[53] + tmpFx[45]*tmpObjS[65] + tmpFx[54]*tmpObjS[77] + tmpFx[63]*tmpObjS[89] + tmpFx[72]*tmpObjS[101] + tmpFx[81]*tmpObjS[113] + tmpFx[90]*tmpObjS[125] + tmpFx[99]*tmpObjS[137];
tmpS2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[9]*tmpObjS[18] + tmpFx[18]*tmpObjS[30] + tmpFx[27]*tmpObjS[42] + tmpFx[36]*tmpObjS[54] + tmpFx[45]*tmpObjS[66] + tmpFx[54]*tmpObjS[78] + tmpFx[63]*tmpObjS[90] + tmpFx[72]*tmpObjS[102] + tmpFx[81]*tmpObjS[114] + tmpFx[90]*tmpObjS[126] + tmpFx[99]*tmpObjS[138];
tmpS2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[9]*tmpObjS[19] + tmpFx[18]*tmpObjS[31] + tmpFx[27]*tmpObjS[43] + tmpFx[36]*tmpObjS[55] + tmpFx[45]*tmpObjS[67] + tmpFx[54]*tmpObjS[79] + tmpFx[63]*tmpObjS[91] + tmpFx[72]*tmpObjS[103] + tmpFx[81]*tmpObjS[115] + tmpFx[90]*tmpObjS[127] + tmpFx[99]*tmpObjS[139];
tmpS2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[9]*tmpObjS[20] + tmpFx[18]*tmpObjS[32] + tmpFx[27]*tmpObjS[44] + tmpFx[36]*tmpObjS[56] + tmpFx[45]*tmpObjS[68] + tmpFx[54]*tmpObjS[80] + tmpFx[63]*tmpObjS[92] + tmpFx[72]*tmpObjS[104] + tmpFx[81]*tmpObjS[116] + tmpFx[90]*tmpObjS[128] + tmpFx[99]*tmpObjS[140];
tmpS2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[9]*tmpObjS[21] + tmpFx[18]*tmpObjS[33] + tmpFx[27]*tmpObjS[45] + tmpFx[36]*tmpObjS[57] + tmpFx[45]*tmpObjS[69] + tmpFx[54]*tmpObjS[81] + tmpFx[63]*tmpObjS[93] + tmpFx[72]*tmpObjS[105] + tmpFx[81]*tmpObjS[117] + tmpFx[90]*tmpObjS[129] + tmpFx[99]*tmpObjS[141];
tmpS2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[9]*tmpObjS[22] + tmpFx[18]*tmpObjS[34] + tmpFx[27]*tmpObjS[46] + tmpFx[36]*tmpObjS[58] + tmpFx[45]*tmpObjS[70] + tmpFx[54]*tmpObjS[82] + tmpFx[63]*tmpObjS[94] + tmpFx[72]*tmpObjS[106] + tmpFx[81]*tmpObjS[118] + tmpFx[90]*tmpObjS[130] + tmpFx[99]*tmpObjS[142];
tmpS2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[9]*tmpObjS[23] + tmpFx[18]*tmpObjS[35] + tmpFx[27]*tmpObjS[47] + tmpFx[36]*tmpObjS[59] + tmpFx[45]*tmpObjS[71] + tmpFx[54]*tmpObjS[83] + tmpFx[63]*tmpObjS[95] + tmpFx[72]*tmpObjS[107] + tmpFx[81]*tmpObjS[119] + tmpFx[90]*tmpObjS[131] + tmpFx[99]*tmpObjS[143];
tmpS2[12] = + tmpFx[1]*tmpObjS[0] + tmpFx[10]*tmpObjS[12] + tmpFx[19]*tmpObjS[24] + tmpFx[28]*tmpObjS[36] + tmpFx[37]*tmpObjS[48] + tmpFx[46]*tmpObjS[60] + tmpFx[55]*tmpObjS[72] + tmpFx[64]*tmpObjS[84] + tmpFx[73]*tmpObjS[96] + tmpFx[82]*tmpObjS[108] + tmpFx[91]*tmpObjS[120] + tmpFx[100]*tmpObjS[132];
tmpS2[13] = + tmpFx[1]*tmpObjS[1] + tmpFx[10]*tmpObjS[13] + tmpFx[19]*tmpObjS[25] + tmpFx[28]*tmpObjS[37] + tmpFx[37]*tmpObjS[49] + tmpFx[46]*tmpObjS[61] + tmpFx[55]*tmpObjS[73] + tmpFx[64]*tmpObjS[85] + tmpFx[73]*tmpObjS[97] + tmpFx[82]*tmpObjS[109] + tmpFx[91]*tmpObjS[121] + tmpFx[100]*tmpObjS[133];
tmpS2[14] = + tmpFx[1]*tmpObjS[2] + tmpFx[10]*tmpObjS[14] + tmpFx[19]*tmpObjS[26] + tmpFx[28]*tmpObjS[38] + tmpFx[37]*tmpObjS[50] + tmpFx[46]*tmpObjS[62] + tmpFx[55]*tmpObjS[74] + tmpFx[64]*tmpObjS[86] + tmpFx[73]*tmpObjS[98] + tmpFx[82]*tmpObjS[110] + tmpFx[91]*tmpObjS[122] + tmpFx[100]*tmpObjS[134];
tmpS2[15] = + tmpFx[1]*tmpObjS[3] + tmpFx[10]*tmpObjS[15] + tmpFx[19]*tmpObjS[27] + tmpFx[28]*tmpObjS[39] + tmpFx[37]*tmpObjS[51] + tmpFx[46]*tmpObjS[63] + tmpFx[55]*tmpObjS[75] + tmpFx[64]*tmpObjS[87] + tmpFx[73]*tmpObjS[99] + tmpFx[82]*tmpObjS[111] + tmpFx[91]*tmpObjS[123] + tmpFx[100]*tmpObjS[135];
tmpS2[16] = + tmpFx[1]*tmpObjS[4] + tmpFx[10]*tmpObjS[16] + tmpFx[19]*tmpObjS[28] + tmpFx[28]*tmpObjS[40] + tmpFx[37]*tmpObjS[52] + tmpFx[46]*tmpObjS[64] + tmpFx[55]*tmpObjS[76] + tmpFx[64]*tmpObjS[88] + tmpFx[73]*tmpObjS[100] + tmpFx[82]*tmpObjS[112] + tmpFx[91]*tmpObjS[124] + tmpFx[100]*tmpObjS[136];
tmpS2[17] = + tmpFx[1]*tmpObjS[5] + tmpFx[10]*tmpObjS[17] + tmpFx[19]*tmpObjS[29] + tmpFx[28]*tmpObjS[41] + tmpFx[37]*tmpObjS[53] + tmpFx[46]*tmpObjS[65] + tmpFx[55]*tmpObjS[77] + tmpFx[64]*tmpObjS[89] + tmpFx[73]*tmpObjS[101] + tmpFx[82]*tmpObjS[113] + tmpFx[91]*tmpObjS[125] + tmpFx[100]*tmpObjS[137];
tmpS2[18] = + tmpFx[1]*tmpObjS[6] + tmpFx[10]*tmpObjS[18] + tmpFx[19]*tmpObjS[30] + tmpFx[28]*tmpObjS[42] + tmpFx[37]*tmpObjS[54] + tmpFx[46]*tmpObjS[66] + tmpFx[55]*tmpObjS[78] + tmpFx[64]*tmpObjS[90] + tmpFx[73]*tmpObjS[102] + tmpFx[82]*tmpObjS[114] + tmpFx[91]*tmpObjS[126] + tmpFx[100]*tmpObjS[138];
tmpS2[19] = + tmpFx[1]*tmpObjS[7] + tmpFx[10]*tmpObjS[19] + tmpFx[19]*tmpObjS[31] + tmpFx[28]*tmpObjS[43] + tmpFx[37]*tmpObjS[55] + tmpFx[46]*tmpObjS[67] + tmpFx[55]*tmpObjS[79] + tmpFx[64]*tmpObjS[91] + tmpFx[73]*tmpObjS[103] + tmpFx[82]*tmpObjS[115] + tmpFx[91]*tmpObjS[127] + tmpFx[100]*tmpObjS[139];
tmpS2[20] = + tmpFx[1]*tmpObjS[8] + tmpFx[10]*tmpObjS[20] + tmpFx[19]*tmpObjS[32] + tmpFx[28]*tmpObjS[44] + tmpFx[37]*tmpObjS[56] + tmpFx[46]*tmpObjS[68] + tmpFx[55]*tmpObjS[80] + tmpFx[64]*tmpObjS[92] + tmpFx[73]*tmpObjS[104] + tmpFx[82]*tmpObjS[116] + tmpFx[91]*tmpObjS[128] + tmpFx[100]*tmpObjS[140];
tmpS2[21] = + tmpFx[1]*tmpObjS[9] + tmpFx[10]*tmpObjS[21] + tmpFx[19]*tmpObjS[33] + tmpFx[28]*tmpObjS[45] + tmpFx[37]*tmpObjS[57] + tmpFx[46]*tmpObjS[69] + tmpFx[55]*tmpObjS[81] + tmpFx[64]*tmpObjS[93] + tmpFx[73]*tmpObjS[105] + tmpFx[82]*tmpObjS[117] + tmpFx[91]*tmpObjS[129] + tmpFx[100]*tmpObjS[141];
tmpS2[22] = + tmpFx[1]*tmpObjS[10] + tmpFx[10]*tmpObjS[22] + tmpFx[19]*tmpObjS[34] + tmpFx[28]*tmpObjS[46] + tmpFx[37]*tmpObjS[58] + tmpFx[46]*tmpObjS[70] + tmpFx[55]*tmpObjS[82] + tmpFx[64]*tmpObjS[94] + tmpFx[73]*tmpObjS[106] + tmpFx[82]*tmpObjS[118] + tmpFx[91]*tmpObjS[130] + tmpFx[100]*tmpObjS[142];
tmpS2[23] = + tmpFx[1]*tmpObjS[11] + tmpFx[10]*tmpObjS[23] + tmpFx[19]*tmpObjS[35] + tmpFx[28]*tmpObjS[47] + tmpFx[37]*tmpObjS[59] + tmpFx[46]*tmpObjS[71] + tmpFx[55]*tmpObjS[83] + tmpFx[64]*tmpObjS[95] + tmpFx[73]*tmpObjS[107] + tmpFx[82]*tmpObjS[119] + tmpFx[91]*tmpObjS[131] + tmpFx[100]*tmpObjS[143];
tmpS2[24] = + tmpFx[2]*tmpObjS[0] + tmpFx[11]*tmpObjS[12] + tmpFx[20]*tmpObjS[24] + tmpFx[29]*tmpObjS[36] + tmpFx[38]*tmpObjS[48] + tmpFx[47]*tmpObjS[60] + tmpFx[56]*tmpObjS[72] + tmpFx[65]*tmpObjS[84] + tmpFx[74]*tmpObjS[96] + tmpFx[83]*tmpObjS[108] + tmpFx[92]*tmpObjS[120] + tmpFx[101]*tmpObjS[132];
tmpS2[25] = + tmpFx[2]*tmpObjS[1] + tmpFx[11]*tmpObjS[13] + tmpFx[20]*tmpObjS[25] + tmpFx[29]*tmpObjS[37] + tmpFx[38]*tmpObjS[49] + tmpFx[47]*tmpObjS[61] + tmpFx[56]*tmpObjS[73] + tmpFx[65]*tmpObjS[85] + tmpFx[74]*tmpObjS[97] + tmpFx[83]*tmpObjS[109] + tmpFx[92]*tmpObjS[121] + tmpFx[101]*tmpObjS[133];
tmpS2[26] = + tmpFx[2]*tmpObjS[2] + tmpFx[11]*tmpObjS[14] + tmpFx[20]*tmpObjS[26] + tmpFx[29]*tmpObjS[38] + tmpFx[38]*tmpObjS[50] + tmpFx[47]*tmpObjS[62] + tmpFx[56]*tmpObjS[74] + tmpFx[65]*tmpObjS[86] + tmpFx[74]*tmpObjS[98] + tmpFx[83]*tmpObjS[110] + tmpFx[92]*tmpObjS[122] + tmpFx[101]*tmpObjS[134];
tmpS2[27] = + tmpFx[2]*tmpObjS[3] + tmpFx[11]*tmpObjS[15] + tmpFx[20]*tmpObjS[27] + tmpFx[29]*tmpObjS[39] + tmpFx[38]*tmpObjS[51] + tmpFx[47]*tmpObjS[63] + tmpFx[56]*tmpObjS[75] + tmpFx[65]*tmpObjS[87] + tmpFx[74]*tmpObjS[99] + tmpFx[83]*tmpObjS[111] + tmpFx[92]*tmpObjS[123] + tmpFx[101]*tmpObjS[135];
tmpS2[28] = + tmpFx[2]*tmpObjS[4] + tmpFx[11]*tmpObjS[16] + tmpFx[20]*tmpObjS[28] + tmpFx[29]*tmpObjS[40] + tmpFx[38]*tmpObjS[52] + tmpFx[47]*tmpObjS[64] + tmpFx[56]*tmpObjS[76] + tmpFx[65]*tmpObjS[88] + tmpFx[74]*tmpObjS[100] + tmpFx[83]*tmpObjS[112] + tmpFx[92]*tmpObjS[124] + tmpFx[101]*tmpObjS[136];
tmpS2[29] = + tmpFx[2]*tmpObjS[5] + tmpFx[11]*tmpObjS[17] + tmpFx[20]*tmpObjS[29] + tmpFx[29]*tmpObjS[41] + tmpFx[38]*tmpObjS[53] + tmpFx[47]*tmpObjS[65] + tmpFx[56]*tmpObjS[77] + tmpFx[65]*tmpObjS[89] + tmpFx[74]*tmpObjS[101] + tmpFx[83]*tmpObjS[113] + tmpFx[92]*tmpObjS[125] + tmpFx[101]*tmpObjS[137];
tmpS2[30] = + tmpFx[2]*tmpObjS[6] + tmpFx[11]*tmpObjS[18] + tmpFx[20]*tmpObjS[30] + tmpFx[29]*tmpObjS[42] + tmpFx[38]*tmpObjS[54] + tmpFx[47]*tmpObjS[66] + tmpFx[56]*tmpObjS[78] + tmpFx[65]*tmpObjS[90] + tmpFx[74]*tmpObjS[102] + tmpFx[83]*tmpObjS[114] + tmpFx[92]*tmpObjS[126] + tmpFx[101]*tmpObjS[138];
tmpS2[31] = + tmpFx[2]*tmpObjS[7] + tmpFx[11]*tmpObjS[19] + tmpFx[20]*tmpObjS[31] + tmpFx[29]*tmpObjS[43] + tmpFx[38]*tmpObjS[55] + tmpFx[47]*tmpObjS[67] + tmpFx[56]*tmpObjS[79] + tmpFx[65]*tmpObjS[91] + tmpFx[74]*tmpObjS[103] + tmpFx[83]*tmpObjS[115] + tmpFx[92]*tmpObjS[127] + tmpFx[101]*tmpObjS[139];
tmpS2[32] = + tmpFx[2]*tmpObjS[8] + tmpFx[11]*tmpObjS[20] + tmpFx[20]*tmpObjS[32] + tmpFx[29]*tmpObjS[44] + tmpFx[38]*tmpObjS[56] + tmpFx[47]*tmpObjS[68] + tmpFx[56]*tmpObjS[80] + tmpFx[65]*tmpObjS[92] + tmpFx[74]*tmpObjS[104] + tmpFx[83]*tmpObjS[116] + tmpFx[92]*tmpObjS[128] + tmpFx[101]*tmpObjS[140];
tmpS2[33] = + tmpFx[2]*tmpObjS[9] + tmpFx[11]*tmpObjS[21] + tmpFx[20]*tmpObjS[33] + tmpFx[29]*tmpObjS[45] + tmpFx[38]*tmpObjS[57] + tmpFx[47]*tmpObjS[69] + tmpFx[56]*tmpObjS[81] + tmpFx[65]*tmpObjS[93] + tmpFx[74]*tmpObjS[105] + tmpFx[83]*tmpObjS[117] + tmpFx[92]*tmpObjS[129] + tmpFx[101]*tmpObjS[141];
tmpS2[34] = + tmpFx[2]*tmpObjS[10] + tmpFx[11]*tmpObjS[22] + tmpFx[20]*tmpObjS[34] + tmpFx[29]*tmpObjS[46] + tmpFx[38]*tmpObjS[58] + tmpFx[47]*tmpObjS[70] + tmpFx[56]*tmpObjS[82] + tmpFx[65]*tmpObjS[94] + tmpFx[74]*tmpObjS[106] + tmpFx[83]*tmpObjS[118] + tmpFx[92]*tmpObjS[130] + tmpFx[101]*tmpObjS[142];
tmpS2[35] = + tmpFx[2]*tmpObjS[11] + tmpFx[11]*tmpObjS[23] + tmpFx[20]*tmpObjS[35] + tmpFx[29]*tmpObjS[47] + tmpFx[38]*tmpObjS[59] + tmpFx[47]*tmpObjS[71] + tmpFx[56]*tmpObjS[83] + tmpFx[65]*tmpObjS[95] + tmpFx[74]*tmpObjS[107] + tmpFx[83]*tmpObjS[119] + tmpFx[92]*tmpObjS[131] + tmpFx[101]*tmpObjS[143];
tmpS2[36] = + tmpFx[3]*tmpObjS[0] + tmpFx[12]*tmpObjS[12] + tmpFx[21]*tmpObjS[24] + tmpFx[30]*tmpObjS[36] + tmpFx[39]*tmpObjS[48] + tmpFx[48]*tmpObjS[60] + tmpFx[57]*tmpObjS[72] + tmpFx[66]*tmpObjS[84] + tmpFx[75]*tmpObjS[96] + tmpFx[84]*tmpObjS[108] + tmpFx[93]*tmpObjS[120] + tmpFx[102]*tmpObjS[132];
tmpS2[37] = + tmpFx[3]*tmpObjS[1] + tmpFx[12]*tmpObjS[13] + tmpFx[21]*tmpObjS[25] + tmpFx[30]*tmpObjS[37] + tmpFx[39]*tmpObjS[49] + tmpFx[48]*tmpObjS[61] + tmpFx[57]*tmpObjS[73] + tmpFx[66]*tmpObjS[85] + tmpFx[75]*tmpObjS[97] + tmpFx[84]*tmpObjS[109] + tmpFx[93]*tmpObjS[121] + tmpFx[102]*tmpObjS[133];
tmpS2[38] = + tmpFx[3]*tmpObjS[2] + tmpFx[12]*tmpObjS[14] + tmpFx[21]*tmpObjS[26] + tmpFx[30]*tmpObjS[38] + tmpFx[39]*tmpObjS[50] + tmpFx[48]*tmpObjS[62] + tmpFx[57]*tmpObjS[74] + tmpFx[66]*tmpObjS[86] + tmpFx[75]*tmpObjS[98] + tmpFx[84]*tmpObjS[110] + tmpFx[93]*tmpObjS[122] + tmpFx[102]*tmpObjS[134];
tmpS2[39] = + tmpFx[3]*tmpObjS[3] + tmpFx[12]*tmpObjS[15] + tmpFx[21]*tmpObjS[27] + tmpFx[30]*tmpObjS[39] + tmpFx[39]*tmpObjS[51] + tmpFx[48]*tmpObjS[63] + tmpFx[57]*tmpObjS[75] + tmpFx[66]*tmpObjS[87] + tmpFx[75]*tmpObjS[99] + tmpFx[84]*tmpObjS[111] + tmpFx[93]*tmpObjS[123] + tmpFx[102]*tmpObjS[135];
tmpS2[40] = + tmpFx[3]*tmpObjS[4] + tmpFx[12]*tmpObjS[16] + tmpFx[21]*tmpObjS[28] + tmpFx[30]*tmpObjS[40] + tmpFx[39]*tmpObjS[52] + tmpFx[48]*tmpObjS[64] + tmpFx[57]*tmpObjS[76] + tmpFx[66]*tmpObjS[88] + tmpFx[75]*tmpObjS[100] + tmpFx[84]*tmpObjS[112] + tmpFx[93]*tmpObjS[124] + tmpFx[102]*tmpObjS[136];
tmpS2[41] = + tmpFx[3]*tmpObjS[5] + tmpFx[12]*tmpObjS[17] + tmpFx[21]*tmpObjS[29] + tmpFx[30]*tmpObjS[41] + tmpFx[39]*tmpObjS[53] + tmpFx[48]*tmpObjS[65] + tmpFx[57]*tmpObjS[77] + tmpFx[66]*tmpObjS[89] + tmpFx[75]*tmpObjS[101] + tmpFx[84]*tmpObjS[113] + tmpFx[93]*tmpObjS[125] + tmpFx[102]*tmpObjS[137];
tmpS2[42] = + tmpFx[3]*tmpObjS[6] + tmpFx[12]*tmpObjS[18] + tmpFx[21]*tmpObjS[30] + tmpFx[30]*tmpObjS[42] + tmpFx[39]*tmpObjS[54] + tmpFx[48]*tmpObjS[66] + tmpFx[57]*tmpObjS[78] + tmpFx[66]*tmpObjS[90] + tmpFx[75]*tmpObjS[102] + tmpFx[84]*tmpObjS[114] + tmpFx[93]*tmpObjS[126] + tmpFx[102]*tmpObjS[138];
tmpS2[43] = + tmpFx[3]*tmpObjS[7] + tmpFx[12]*tmpObjS[19] + tmpFx[21]*tmpObjS[31] + tmpFx[30]*tmpObjS[43] + tmpFx[39]*tmpObjS[55] + tmpFx[48]*tmpObjS[67] + tmpFx[57]*tmpObjS[79] + tmpFx[66]*tmpObjS[91] + tmpFx[75]*tmpObjS[103] + tmpFx[84]*tmpObjS[115] + tmpFx[93]*tmpObjS[127] + tmpFx[102]*tmpObjS[139];
tmpS2[44] = + tmpFx[3]*tmpObjS[8] + tmpFx[12]*tmpObjS[20] + tmpFx[21]*tmpObjS[32] + tmpFx[30]*tmpObjS[44] + tmpFx[39]*tmpObjS[56] + tmpFx[48]*tmpObjS[68] + tmpFx[57]*tmpObjS[80] + tmpFx[66]*tmpObjS[92] + tmpFx[75]*tmpObjS[104] + tmpFx[84]*tmpObjS[116] + tmpFx[93]*tmpObjS[128] + tmpFx[102]*tmpObjS[140];
tmpS2[45] = + tmpFx[3]*tmpObjS[9] + tmpFx[12]*tmpObjS[21] + tmpFx[21]*tmpObjS[33] + tmpFx[30]*tmpObjS[45] + tmpFx[39]*tmpObjS[57] + tmpFx[48]*tmpObjS[69] + tmpFx[57]*tmpObjS[81] + tmpFx[66]*tmpObjS[93] + tmpFx[75]*tmpObjS[105] + tmpFx[84]*tmpObjS[117] + tmpFx[93]*tmpObjS[129] + tmpFx[102]*tmpObjS[141];
tmpS2[46] = + tmpFx[3]*tmpObjS[10] + tmpFx[12]*tmpObjS[22] + tmpFx[21]*tmpObjS[34] + tmpFx[30]*tmpObjS[46] + tmpFx[39]*tmpObjS[58] + tmpFx[48]*tmpObjS[70] + tmpFx[57]*tmpObjS[82] + tmpFx[66]*tmpObjS[94] + tmpFx[75]*tmpObjS[106] + tmpFx[84]*tmpObjS[118] + tmpFx[93]*tmpObjS[130] + tmpFx[102]*tmpObjS[142];
tmpS2[47] = + tmpFx[3]*tmpObjS[11] + tmpFx[12]*tmpObjS[23] + tmpFx[21]*tmpObjS[35] + tmpFx[30]*tmpObjS[47] + tmpFx[39]*tmpObjS[59] + tmpFx[48]*tmpObjS[71] + tmpFx[57]*tmpObjS[83] + tmpFx[66]*tmpObjS[95] + tmpFx[75]*tmpObjS[107] + tmpFx[84]*tmpObjS[119] + tmpFx[93]*tmpObjS[131] + tmpFx[102]*tmpObjS[143];
tmpS2[48] = + tmpFx[4]*tmpObjS[0] + tmpFx[13]*tmpObjS[12] + tmpFx[22]*tmpObjS[24] + tmpFx[31]*tmpObjS[36] + tmpFx[40]*tmpObjS[48] + tmpFx[49]*tmpObjS[60] + tmpFx[58]*tmpObjS[72] + tmpFx[67]*tmpObjS[84] + tmpFx[76]*tmpObjS[96] + tmpFx[85]*tmpObjS[108] + tmpFx[94]*tmpObjS[120] + tmpFx[103]*tmpObjS[132];
tmpS2[49] = + tmpFx[4]*tmpObjS[1] + tmpFx[13]*tmpObjS[13] + tmpFx[22]*tmpObjS[25] + tmpFx[31]*tmpObjS[37] + tmpFx[40]*tmpObjS[49] + tmpFx[49]*tmpObjS[61] + tmpFx[58]*tmpObjS[73] + tmpFx[67]*tmpObjS[85] + tmpFx[76]*tmpObjS[97] + tmpFx[85]*tmpObjS[109] + tmpFx[94]*tmpObjS[121] + tmpFx[103]*tmpObjS[133];
tmpS2[50] = + tmpFx[4]*tmpObjS[2] + tmpFx[13]*tmpObjS[14] + tmpFx[22]*tmpObjS[26] + tmpFx[31]*tmpObjS[38] + tmpFx[40]*tmpObjS[50] + tmpFx[49]*tmpObjS[62] + tmpFx[58]*tmpObjS[74] + tmpFx[67]*tmpObjS[86] + tmpFx[76]*tmpObjS[98] + tmpFx[85]*tmpObjS[110] + tmpFx[94]*tmpObjS[122] + tmpFx[103]*tmpObjS[134];
tmpS2[51] = + tmpFx[4]*tmpObjS[3] + tmpFx[13]*tmpObjS[15] + tmpFx[22]*tmpObjS[27] + tmpFx[31]*tmpObjS[39] + tmpFx[40]*tmpObjS[51] + tmpFx[49]*tmpObjS[63] + tmpFx[58]*tmpObjS[75] + tmpFx[67]*tmpObjS[87] + tmpFx[76]*tmpObjS[99] + tmpFx[85]*tmpObjS[111] + tmpFx[94]*tmpObjS[123] + tmpFx[103]*tmpObjS[135];
tmpS2[52] = + tmpFx[4]*tmpObjS[4] + tmpFx[13]*tmpObjS[16] + tmpFx[22]*tmpObjS[28] + tmpFx[31]*tmpObjS[40] + tmpFx[40]*tmpObjS[52] + tmpFx[49]*tmpObjS[64] + tmpFx[58]*tmpObjS[76] + tmpFx[67]*tmpObjS[88] + tmpFx[76]*tmpObjS[100] + tmpFx[85]*tmpObjS[112] + tmpFx[94]*tmpObjS[124] + tmpFx[103]*tmpObjS[136];
tmpS2[53] = + tmpFx[4]*tmpObjS[5] + tmpFx[13]*tmpObjS[17] + tmpFx[22]*tmpObjS[29] + tmpFx[31]*tmpObjS[41] + tmpFx[40]*tmpObjS[53] + tmpFx[49]*tmpObjS[65] + tmpFx[58]*tmpObjS[77] + tmpFx[67]*tmpObjS[89] + tmpFx[76]*tmpObjS[101] + tmpFx[85]*tmpObjS[113] + tmpFx[94]*tmpObjS[125] + tmpFx[103]*tmpObjS[137];
tmpS2[54] = + tmpFx[4]*tmpObjS[6] + tmpFx[13]*tmpObjS[18] + tmpFx[22]*tmpObjS[30] + tmpFx[31]*tmpObjS[42] + tmpFx[40]*tmpObjS[54] + tmpFx[49]*tmpObjS[66] + tmpFx[58]*tmpObjS[78] + tmpFx[67]*tmpObjS[90] + tmpFx[76]*tmpObjS[102] + tmpFx[85]*tmpObjS[114] + tmpFx[94]*tmpObjS[126] + tmpFx[103]*tmpObjS[138];
tmpS2[55] = + tmpFx[4]*tmpObjS[7] + tmpFx[13]*tmpObjS[19] + tmpFx[22]*tmpObjS[31] + tmpFx[31]*tmpObjS[43] + tmpFx[40]*tmpObjS[55] + tmpFx[49]*tmpObjS[67] + tmpFx[58]*tmpObjS[79] + tmpFx[67]*tmpObjS[91] + tmpFx[76]*tmpObjS[103] + tmpFx[85]*tmpObjS[115] + tmpFx[94]*tmpObjS[127] + tmpFx[103]*tmpObjS[139];
tmpS2[56] = + tmpFx[4]*tmpObjS[8] + tmpFx[13]*tmpObjS[20] + tmpFx[22]*tmpObjS[32] + tmpFx[31]*tmpObjS[44] + tmpFx[40]*tmpObjS[56] + tmpFx[49]*tmpObjS[68] + tmpFx[58]*tmpObjS[80] + tmpFx[67]*tmpObjS[92] + tmpFx[76]*tmpObjS[104] + tmpFx[85]*tmpObjS[116] + tmpFx[94]*tmpObjS[128] + tmpFx[103]*tmpObjS[140];
tmpS2[57] = + tmpFx[4]*tmpObjS[9] + tmpFx[13]*tmpObjS[21] + tmpFx[22]*tmpObjS[33] + tmpFx[31]*tmpObjS[45] + tmpFx[40]*tmpObjS[57] + tmpFx[49]*tmpObjS[69] + tmpFx[58]*tmpObjS[81] + tmpFx[67]*tmpObjS[93] + tmpFx[76]*tmpObjS[105] + tmpFx[85]*tmpObjS[117] + tmpFx[94]*tmpObjS[129] + tmpFx[103]*tmpObjS[141];
tmpS2[58] = + tmpFx[4]*tmpObjS[10] + tmpFx[13]*tmpObjS[22] + tmpFx[22]*tmpObjS[34] + tmpFx[31]*tmpObjS[46] + tmpFx[40]*tmpObjS[58] + tmpFx[49]*tmpObjS[70] + tmpFx[58]*tmpObjS[82] + tmpFx[67]*tmpObjS[94] + tmpFx[76]*tmpObjS[106] + tmpFx[85]*tmpObjS[118] + tmpFx[94]*tmpObjS[130] + tmpFx[103]*tmpObjS[142];
tmpS2[59] = + tmpFx[4]*tmpObjS[11] + tmpFx[13]*tmpObjS[23] + tmpFx[22]*tmpObjS[35] + tmpFx[31]*tmpObjS[47] + tmpFx[40]*tmpObjS[59] + tmpFx[49]*tmpObjS[71] + tmpFx[58]*tmpObjS[83] + tmpFx[67]*tmpObjS[95] + tmpFx[76]*tmpObjS[107] + tmpFx[85]*tmpObjS[119] + tmpFx[94]*tmpObjS[131] + tmpFx[103]*tmpObjS[143];
tmpS2[60] = + tmpFx[5]*tmpObjS[0] + tmpFx[14]*tmpObjS[12] + tmpFx[23]*tmpObjS[24] + tmpFx[32]*tmpObjS[36] + tmpFx[41]*tmpObjS[48] + tmpFx[50]*tmpObjS[60] + tmpFx[59]*tmpObjS[72] + tmpFx[68]*tmpObjS[84] + tmpFx[77]*tmpObjS[96] + tmpFx[86]*tmpObjS[108] + tmpFx[95]*tmpObjS[120] + tmpFx[104]*tmpObjS[132];
tmpS2[61] = + tmpFx[5]*tmpObjS[1] + tmpFx[14]*tmpObjS[13] + tmpFx[23]*tmpObjS[25] + tmpFx[32]*tmpObjS[37] + tmpFx[41]*tmpObjS[49] + tmpFx[50]*tmpObjS[61] + tmpFx[59]*tmpObjS[73] + tmpFx[68]*tmpObjS[85] + tmpFx[77]*tmpObjS[97] + tmpFx[86]*tmpObjS[109] + tmpFx[95]*tmpObjS[121] + tmpFx[104]*tmpObjS[133];
tmpS2[62] = + tmpFx[5]*tmpObjS[2] + tmpFx[14]*tmpObjS[14] + tmpFx[23]*tmpObjS[26] + tmpFx[32]*tmpObjS[38] + tmpFx[41]*tmpObjS[50] + tmpFx[50]*tmpObjS[62] + tmpFx[59]*tmpObjS[74] + tmpFx[68]*tmpObjS[86] + tmpFx[77]*tmpObjS[98] + tmpFx[86]*tmpObjS[110] + tmpFx[95]*tmpObjS[122] + tmpFx[104]*tmpObjS[134];
tmpS2[63] = + tmpFx[5]*tmpObjS[3] + tmpFx[14]*tmpObjS[15] + tmpFx[23]*tmpObjS[27] + tmpFx[32]*tmpObjS[39] + tmpFx[41]*tmpObjS[51] + tmpFx[50]*tmpObjS[63] + tmpFx[59]*tmpObjS[75] + tmpFx[68]*tmpObjS[87] + tmpFx[77]*tmpObjS[99] + tmpFx[86]*tmpObjS[111] + tmpFx[95]*tmpObjS[123] + tmpFx[104]*tmpObjS[135];
tmpS2[64] = + tmpFx[5]*tmpObjS[4] + tmpFx[14]*tmpObjS[16] + tmpFx[23]*tmpObjS[28] + tmpFx[32]*tmpObjS[40] + tmpFx[41]*tmpObjS[52] + tmpFx[50]*tmpObjS[64] + tmpFx[59]*tmpObjS[76] + tmpFx[68]*tmpObjS[88] + tmpFx[77]*tmpObjS[100] + tmpFx[86]*tmpObjS[112] + tmpFx[95]*tmpObjS[124] + tmpFx[104]*tmpObjS[136];
tmpS2[65] = + tmpFx[5]*tmpObjS[5] + tmpFx[14]*tmpObjS[17] + tmpFx[23]*tmpObjS[29] + tmpFx[32]*tmpObjS[41] + tmpFx[41]*tmpObjS[53] + tmpFx[50]*tmpObjS[65] + tmpFx[59]*tmpObjS[77] + tmpFx[68]*tmpObjS[89] + tmpFx[77]*tmpObjS[101] + tmpFx[86]*tmpObjS[113] + tmpFx[95]*tmpObjS[125] + tmpFx[104]*tmpObjS[137];
tmpS2[66] = + tmpFx[5]*tmpObjS[6] + tmpFx[14]*tmpObjS[18] + tmpFx[23]*tmpObjS[30] + tmpFx[32]*tmpObjS[42] + tmpFx[41]*tmpObjS[54] + tmpFx[50]*tmpObjS[66] + tmpFx[59]*tmpObjS[78] + tmpFx[68]*tmpObjS[90] + tmpFx[77]*tmpObjS[102] + tmpFx[86]*tmpObjS[114] + tmpFx[95]*tmpObjS[126] + tmpFx[104]*tmpObjS[138];
tmpS2[67] = + tmpFx[5]*tmpObjS[7] + tmpFx[14]*tmpObjS[19] + tmpFx[23]*tmpObjS[31] + tmpFx[32]*tmpObjS[43] + tmpFx[41]*tmpObjS[55] + tmpFx[50]*tmpObjS[67] + tmpFx[59]*tmpObjS[79] + tmpFx[68]*tmpObjS[91] + tmpFx[77]*tmpObjS[103] + tmpFx[86]*tmpObjS[115] + tmpFx[95]*tmpObjS[127] + tmpFx[104]*tmpObjS[139];
tmpS2[68] = + tmpFx[5]*tmpObjS[8] + tmpFx[14]*tmpObjS[20] + tmpFx[23]*tmpObjS[32] + tmpFx[32]*tmpObjS[44] + tmpFx[41]*tmpObjS[56] + tmpFx[50]*tmpObjS[68] + tmpFx[59]*tmpObjS[80] + tmpFx[68]*tmpObjS[92] + tmpFx[77]*tmpObjS[104] + tmpFx[86]*tmpObjS[116] + tmpFx[95]*tmpObjS[128] + tmpFx[104]*tmpObjS[140];
tmpS2[69] = + tmpFx[5]*tmpObjS[9] + tmpFx[14]*tmpObjS[21] + tmpFx[23]*tmpObjS[33] + tmpFx[32]*tmpObjS[45] + tmpFx[41]*tmpObjS[57] + tmpFx[50]*tmpObjS[69] + tmpFx[59]*tmpObjS[81] + tmpFx[68]*tmpObjS[93] + tmpFx[77]*tmpObjS[105] + tmpFx[86]*tmpObjS[117] + tmpFx[95]*tmpObjS[129] + tmpFx[104]*tmpObjS[141];
tmpS2[70] = + tmpFx[5]*tmpObjS[10] + tmpFx[14]*tmpObjS[22] + tmpFx[23]*tmpObjS[34] + tmpFx[32]*tmpObjS[46] + tmpFx[41]*tmpObjS[58] + tmpFx[50]*tmpObjS[70] + tmpFx[59]*tmpObjS[82] + tmpFx[68]*tmpObjS[94] + tmpFx[77]*tmpObjS[106] + tmpFx[86]*tmpObjS[118] + tmpFx[95]*tmpObjS[130] + tmpFx[104]*tmpObjS[142];
tmpS2[71] = + tmpFx[5]*tmpObjS[11] + tmpFx[14]*tmpObjS[23] + tmpFx[23]*tmpObjS[35] + tmpFx[32]*tmpObjS[47] + tmpFx[41]*tmpObjS[59] + tmpFx[50]*tmpObjS[71] + tmpFx[59]*tmpObjS[83] + tmpFx[68]*tmpObjS[95] + tmpFx[77]*tmpObjS[107] + tmpFx[86]*tmpObjS[119] + tmpFx[95]*tmpObjS[131] + tmpFx[104]*tmpObjS[143];
tmpS2[72] = + tmpFx[6]*tmpObjS[0] + tmpFx[15]*tmpObjS[12] + tmpFx[24]*tmpObjS[24] + tmpFx[33]*tmpObjS[36] + tmpFx[42]*tmpObjS[48] + tmpFx[51]*tmpObjS[60] + tmpFx[60]*tmpObjS[72] + tmpFx[69]*tmpObjS[84] + tmpFx[78]*tmpObjS[96] + tmpFx[87]*tmpObjS[108] + tmpFx[96]*tmpObjS[120] + tmpFx[105]*tmpObjS[132];
tmpS2[73] = + tmpFx[6]*tmpObjS[1] + tmpFx[15]*tmpObjS[13] + tmpFx[24]*tmpObjS[25] + tmpFx[33]*tmpObjS[37] + tmpFx[42]*tmpObjS[49] + tmpFx[51]*tmpObjS[61] + tmpFx[60]*tmpObjS[73] + tmpFx[69]*tmpObjS[85] + tmpFx[78]*tmpObjS[97] + tmpFx[87]*tmpObjS[109] + tmpFx[96]*tmpObjS[121] + tmpFx[105]*tmpObjS[133];
tmpS2[74] = + tmpFx[6]*tmpObjS[2] + tmpFx[15]*tmpObjS[14] + tmpFx[24]*tmpObjS[26] + tmpFx[33]*tmpObjS[38] + tmpFx[42]*tmpObjS[50] + tmpFx[51]*tmpObjS[62] + tmpFx[60]*tmpObjS[74] + tmpFx[69]*tmpObjS[86] + tmpFx[78]*tmpObjS[98] + tmpFx[87]*tmpObjS[110] + tmpFx[96]*tmpObjS[122] + tmpFx[105]*tmpObjS[134];
tmpS2[75] = + tmpFx[6]*tmpObjS[3] + tmpFx[15]*tmpObjS[15] + tmpFx[24]*tmpObjS[27] + tmpFx[33]*tmpObjS[39] + tmpFx[42]*tmpObjS[51] + tmpFx[51]*tmpObjS[63] + tmpFx[60]*tmpObjS[75] + tmpFx[69]*tmpObjS[87] + tmpFx[78]*tmpObjS[99] + tmpFx[87]*tmpObjS[111] + tmpFx[96]*tmpObjS[123] + tmpFx[105]*tmpObjS[135];
tmpS2[76] = + tmpFx[6]*tmpObjS[4] + tmpFx[15]*tmpObjS[16] + tmpFx[24]*tmpObjS[28] + tmpFx[33]*tmpObjS[40] + tmpFx[42]*tmpObjS[52] + tmpFx[51]*tmpObjS[64] + tmpFx[60]*tmpObjS[76] + tmpFx[69]*tmpObjS[88] + tmpFx[78]*tmpObjS[100] + tmpFx[87]*tmpObjS[112] + tmpFx[96]*tmpObjS[124] + tmpFx[105]*tmpObjS[136];
tmpS2[77] = + tmpFx[6]*tmpObjS[5] + tmpFx[15]*tmpObjS[17] + tmpFx[24]*tmpObjS[29] + tmpFx[33]*tmpObjS[41] + tmpFx[42]*tmpObjS[53] + tmpFx[51]*tmpObjS[65] + tmpFx[60]*tmpObjS[77] + tmpFx[69]*tmpObjS[89] + tmpFx[78]*tmpObjS[101] + tmpFx[87]*tmpObjS[113] + tmpFx[96]*tmpObjS[125] + tmpFx[105]*tmpObjS[137];
tmpS2[78] = + tmpFx[6]*tmpObjS[6] + tmpFx[15]*tmpObjS[18] + tmpFx[24]*tmpObjS[30] + tmpFx[33]*tmpObjS[42] + tmpFx[42]*tmpObjS[54] + tmpFx[51]*tmpObjS[66] + tmpFx[60]*tmpObjS[78] + tmpFx[69]*tmpObjS[90] + tmpFx[78]*tmpObjS[102] + tmpFx[87]*tmpObjS[114] + tmpFx[96]*tmpObjS[126] + tmpFx[105]*tmpObjS[138];
tmpS2[79] = + tmpFx[6]*tmpObjS[7] + tmpFx[15]*tmpObjS[19] + tmpFx[24]*tmpObjS[31] + tmpFx[33]*tmpObjS[43] + tmpFx[42]*tmpObjS[55] + tmpFx[51]*tmpObjS[67] + tmpFx[60]*tmpObjS[79] + tmpFx[69]*tmpObjS[91] + tmpFx[78]*tmpObjS[103] + tmpFx[87]*tmpObjS[115] + tmpFx[96]*tmpObjS[127] + tmpFx[105]*tmpObjS[139];
tmpS2[80] = + tmpFx[6]*tmpObjS[8] + tmpFx[15]*tmpObjS[20] + tmpFx[24]*tmpObjS[32] + tmpFx[33]*tmpObjS[44] + tmpFx[42]*tmpObjS[56] + tmpFx[51]*tmpObjS[68] + tmpFx[60]*tmpObjS[80] + tmpFx[69]*tmpObjS[92] + tmpFx[78]*tmpObjS[104] + tmpFx[87]*tmpObjS[116] + tmpFx[96]*tmpObjS[128] + tmpFx[105]*tmpObjS[140];
tmpS2[81] = + tmpFx[6]*tmpObjS[9] + tmpFx[15]*tmpObjS[21] + tmpFx[24]*tmpObjS[33] + tmpFx[33]*tmpObjS[45] + tmpFx[42]*tmpObjS[57] + tmpFx[51]*tmpObjS[69] + tmpFx[60]*tmpObjS[81] + tmpFx[69]*tmpObjS[93] + tmpFx[78]*tmpObjS[105] + tmpFx[87]*tmpObjS[117] + tmpFx[96]*tmpObjS[129] + tmpFx[105]*tmpObjS[141];
tmpS2[82] = + tmpFx[6]*tmpObjS[10] + tmpFx[15]*tmpObjS[22] + tmpFx[24]*tmpObjS[34] + tmpFx[33]*tmpObjS[46] + tmpFx[42]*tmpObjS[58] + tmpFx[51]*tmpObjS[70] + tmpFx[60]*tmpObjS[82] + tmpFx[69]*tmpObjS[94] + tmpFx[78]*tmpObjS[106] + tmpFx[87]*tmpObjS[118] + tmpFx[96]*tmpObjS[130] + tmpFx[105]*tmpObjS[142];
tmpS2[83] = + tmpFx[6]*tmpObjS[11] + tmpFx[15]*tmpObjS[23] + tmpFx[24]*tmpObjS[35] + tmpFx[33]*tmpObjS[47] + tmpFx[42]*tmpObjS[59] + tmpFx[51]*tmpObjS[71] + tmpFx[60]*tmpObjS[83] + tmpFx[69]*tmpObjS[95] + tmpFx[78]*tmpObjS[107] + tmpFx[87]*tmpObjS[119] + tmpFx[96]*tmpObjS[131] + tmpFx[105]*tmpObjS[143];
tmpS2[84] = + tmpFx[7]*tmpObjS[0] + tmpFx[16]*tmpObjS[12] + tmpFx[25]*tmpObjS[24] + tmpFx[34]*tmpObjS[36] + tmpFx[43]*tmpObjS[48] + tmpFx[52]*tmpObjS[60] + tmpFx[61]*tmpObjS[72] + tmpFx[70]*tmpObjS[84] + tmpFx[79]*tmpObjS[96] + tmpFx[88]*tmpObjS[108] + tmpFx[97]*tmpObjS[120] + tmpFx[106]*tmpObjS[132];
tmpS2[85] = + tmpFx[7]*tmpObjS[1] + tmpFx[16]*tmpObjS[13] + tmpFx[25]*tmpObjS[25] + tmpFx[34]*tmpObjS[37] + tmpFx[43]*tmpObjS[49] + tmpFx[52]*tmpObjS[61] + tmpFx[61]*tmpObjS[73] + tmpFx[70]*tmpObjS[85] + tmpFx[79]*tmpObjS[97] + tmpFx[88]*tmpObjS[109] + tmpFx[97]*tmpObjS[121] + tmpFx[106]*tmpObjS[133];
tmpS2[86] = + tmpFx[7]*tmpObjS[2] + tmpFx[16]*tmpObjS[14] + tmpFx[25]*tmpObjS[26] + tmpFx[34]*tmpObjS[38] + tmpFx[43]*tmpObjS[50] + tmpFx[52]*tmpObjS[62] + tmpFx[61]*tmpObjS[74] + tmpFx[70]*tmpObjS[86] + tmpFx[79]*tmpObjS[98] + tmpFx[88]*tmpObjS[110] + tmpFx[97]*tmpObjS[122] + tmpFx[106]*tmpObjS[134];
tmpS2[87] = + tmpFx[7]*tmpObjS[3] + tmpFx[16]*tmpObjS[15] + tmpFx[25]*tmpObjS[27] + tmpFx[34]*tmpObjS[39] + tmpFx[43]*tmpObjS[51] + tmpFx[52]*tmpObjS[63] + tmpFx[61]*tmpObjS[75] + tmpFx[70]*tmpObjS[87] + tmpFx[79]*tmpObjS[99] + tmpFx[88]*tmpObjS[111] + tmpFx[97]*tmpObjS[123] + tmpFx[106]*tmpObjS[135];
tmpS2[88] = + tmpFx[7]*tmpObjS[4] + tmpFx[16]*tmpObjS[16] + tmpFx[25]*tmpObjS[28] + tmpFx[34]*tmpObjS[40] + tmpFx[43]*tmpObjS[52] + tmpFx[52]*tmpObjS[64] + tmpFx[61]*tmpObjS[76] + tmpFx[70]*tmpObjS[88] + tmpFx[79]*tmpObjS[100] + tmpFx[88]*tmpObjS[112] + tmpFx[97]*tmpObjS[124] + tmpFx[106]*tmpObjS[136];
tmpS2[89] = + tmpFx[7]*tmpObjS[5] + tmpFx[16]*tmpObjS[17] + tmpFx[25]*tmpObjS[29] + tmpFx[34]*tmpObjS[41] + tmpFx[43]*tmpObjS[53] + tmpFx[52]*tmpObjS[65] + tmpFx[61]*tmpObjS[77] + tmpFx[70]*tmpObjS[89] + tmpFx[79]*tmpObjS[101] + tmpFx[88]*tmpObjS[113] + tmpFx[97]*tmpObjS[125] + tmpFx[106]*tmpObjS[137];
tmpS2[90] = + tmpFx[7]*tmpObjS[6] + tmpFx[16]*tmpObjS[18] + tmpFx[25]*tmpObjS[30] + tmpFx[34]*tmpObjS[42] + tmpFx[43]*tmpObjS[54] + tmpFx[52]*tmpObjS[66] + tmpFx[61]*tmpObjS[78] + tmpFx[70]*tmpObjS[90] + tmpFx[79]*tmpObjS[102] + tmpFx[88]*tmpObjS[114] + tmpFx[97]*tmpObjS[126] + tmpFx[106]*tmpObjS[138];
tmpS2[91] = + tmpFx[7]*tmpObjS[7] + tmpFx[16]*tmpObjS[19] + tmpFx[25]*tmpObjS[31] + tmpFx[34]*tmpObjS[43] + tmpFx[43]*tmpObjS[55] + tmpFx[52]*tmpObjS[67] + tmpFx[61]*tmpObjS[79] + tmpFx[70]*tmpObjS[91] + tmpFx[79]*tmpObjS[103] + tmpFx[88]*tmpObjS[115] + tmpFx[97]*tmpObjS[127] + tmpFx[106]*tmpObjS[139];
tmpS2[92] = + tmpFx[7]*tmpObjS[8] + tmpFx[16]*tmpObjS[20] + tmpFx[25]*tmpObjS[32] + tmpFx[34]*tmpObjS[44] + tmpFx[43]*tmpObjS[56] + tmpFx[52]*tmpObjS[68] + tmpFx[61]*tmpObjS[80] + tmpFx[70]*tmpObjS[92] + tmpFx[79]*tmpObjS[104] + tmpFx[88]*tmpObjS[116] + tmpFx[97]*tmpObjS[128] + tmpFx[106]*tmpObjS[140];
tmpS2[93] = + tmpFx[7]*tmpObjS[9] + tmpFx[16]*tmpObjS[21] + tmpFx[25]*tmpObjS[33] + tmpFx[34]*tmpObjS[45] + tmpFx[43]*tmpObjS[57] + tmpFx[52]*tmpObjS[69] + tmpFx[61]*tmpObjS[81] + tmpFx[70]*tmpObjS[93] + tmpFx[79]*tmpObjS[105] + tmpFx[88]*tmpObjS[117] + tmpFx[97]*tmpObjS[129] + tmpFx[106]*tmpObjS[141];
tmpS2[94] = + tmpFx[7]*tmpObjS[10] + tmpFx[16]*tmpObjS[22] + tmpFx[25]*tmpObjS[34] + tmpFx[34]*tmpObjS[46] + tmpFx[43]*tmpObjS[58] + tmpFx[52]*tmpObjS[70] + tmpFx[61]*tmpObjS[82] + tmpFx[70]*tmpObjS[94] + tmpFx[79]*tmpObjS[106] + tmpFx[88]*tmpObjS[118] + tmpFx[97]*tmpObjS[130] + tmpFx[106]*tmpObjS[142];
tmpS2[95] = + tmpFx[7]*tmpObjS[11] + tmpFx[16]*tmpObjS[23] + tmpFx[25]*tmpObjS[35] + tmpFx[34]*tmpObjS[47] + tmpFx[43]*tmpObjS[59] + tmpFx[52]*tmpObjS[71] + tmpFx[61]*tmpObjS[83] + tmpFx[70]*tmpObjS[95] + tmpFx[79]*tmpObjS[107] + tmpFx[88]*tmpObjS[119] + tmpFx[97]*tmpObjS[131] + tmpFx[106]*tmpObjS[143];
tmpS2[96] = + tmpFx[8]*tmpObjS[0] + tmpFx[17]*tmpObjS[12] + tmpFx[26]*tmpObjS[24] + tmpFx[35]*tmpObjS[36] + tmpFx[44]*tmpObjS[48] + tmpFx[53]*tmpObjS[60] + tmpFx[62]*tmpObjS[72] + tmpFx[71]*tmpObjS[84] + tmpFx[80]*tmpObjS[96] + tmpFx[89]*tmpObjS[108] + tmpFx[98]*tmpObjS[120] + tmpFx[107]*tmpObjS[132];
tmpS2[97] = + tmpFx[8]*tmpObjS[1] + tmpFx[17]*tmpObjS[13] + tmpFx[26]*tmpObjS[25] + tmpFx[35]*tmpObjS[37] + tmpFx[44]*tmpObjS[49] + tmpFx[53]*tmpObjS[61] + tmpFx[62]*tmpObjS[73] + tmpFx[71]*tmpObjS[85] + tmpFx[80]*tmpObjS[97] + tmpFx[89]*tmpObjS[109] + tmpFx[98]*tmpObjS[121] + tmpFx[107]*tmpObjS[133];
tmpS2[98] = + tmpFx[8]*tmpObjS[2] + tmpFx[17]*tmpObjS[14] + tmpFx[26]*tmpObjS[26] + tmpFx[35]*tmpObjS[38] + tmpFx[44]*tmpObjS[50] + tmpFx[53]*tmpObjS[62] + tmpFx[62]*tmpObjS[74] + tmpFx[71]*tmpObjS[86] + tmpFx[80]*tmpObjS[98] + tmpFx[89]*tmpObjS[110] + tmpFx[98]*tmpObjS[122] + tmpFx[107]*tmpObjS[134];
tmpS2[99] = + tmpFx[8]*tmpObjS[3] + tmpFx[17]*tmpObjS[15] + tmpFx[26]*tmpObjS[27] + tmpFx[35]*tmpObjS[39] + tmpFx[44]*tmpObjS[51] + tmpFx[53]*tmpObjS[63] + tmpFx[62]*tmpObjS[75] + tmpFx[71]*tmpObjS[87] + tmpFx[80]*tmpObjS[99] + tmpFx[89]*tmpObjS[111] + tmpFx[98]*tmpObjS[123] + tmpFx[107]*tmpObjS[135];
tmpS2[100] = + tmpFx[8]*tmpObjS[4] + tmpFx[17]*tmpObjS[16] + tmpFx[26]*tmpObjS[28] + tmpFx[35]*tmpObjS[40] + tmpFx[44]*tmpObjS[52] + tmpFx[53]*tmpObjS[64] + tmpFx[62]*tmpObjS[76] + tmpFx[71]*tmpObjS[88] + tmpFx[80]*tmpObjS[100] + tmpFx[89]*tmpObjS[112] + tmpFx[98]*tmpObjS[124] + tmpFx[107]*tmpObjS[136];
tmpS2[101] = + tmpFx[8]*tmpObjS[5] + tmpFx[17]*tmpObjS[17] + tmpFx[26]*tmpObjS[29] + tmpFx[35]*tmpObjS[41] + tmpFx[44]*tmpObjS[53] + tmpFx[53]*tmpObjS[65] + tmpFx[62]*tmpObjS[77] + tmpFx[71]*tmpObjS[89] + tmpFx[80]*tmpObjS[101] + tmpFx[89]*tmpObjS[113] + tmpFx[98]*tmpObjS[125] + tmpFx[107]*tmpObjS[137];
tmpS2[102] = + tmpFx[8]*tmpObjS[6] + tmpFx[17]*tmpObjS[18] + tmpFx[26]*tmpObjS[30] + tmpFx[35]*tmpObjS[42] + tmpFx[44]*tmpObjS[54] + tmpFx[53]*tmpObjS[66] + tmpFx[62]*tmpObjS[78] + tmpFx[71]*tmpObjS[90] + tmpFx[80]*tmpObjS[102] + tmpFx[89]*tmpObjS[114] + tmpFx[98]*tmpObjS[126] + tmpFx[107]*tmpObjS[138];
tmpS2[103] = + tmpFx[8]*tmpObjS[7] + tmpFx[17]*tmpObjS[19] + tmpFx[26]*tmpObjS[31] + tmpFx[35]*tmpObjS[43] + tmpFx[44]*tmpObjS[55] + tmpFx[53]*tmpObjS[67] + tmpFx[62]*tmpObjS[79] + tmpFx[71]*tmpObjS[91] + tmpFx[80]*tmpObjS[103] + tmpFx[89]*tmpObjS[115] + tmpFx[98]*tmpObjS[127] + tmpFx[107]*tmpObjS[139];
tmpS2[104] = + tmpFx[8]*tmpObjS[8] + tmpFx[17]*tmpObjS[20] + tmpFx[26]*tmpObjS[32] + tmpFx[35]*tmpObjS[44] + tmpFx[44]*tmpObjS[56] + tmpFx[53]*tmpObjS[68] + tmpFx[62]*tmpObjS[80] + tmpFx[71]*tmpObjS[92] + tmpFx[80]*tmpObjS[104] + tmpFx[89]*tmpObjS[116] + tmpFx[98]*tmpObjS[128] + tmpFx[107]*tmpObjS[140];
tmpS2[105] = + tmpFx[8]*tmpObjS[9] + tmpFx[17]*tmpObjS[21] + tmpFx[26]*tmpObjS[33] + tmpFx[35]*tmpObjS[45] + tmpFx[44]*tmpObjS[57] + tmpFx[53]*tmpObjS[69] + tmpFx[62]*tmpObjS[81] + tmpFx[71]*tmpObjS[93] + tmpFx[80]*tmpObjS[105] + tmpFx[89]*tmpObjS[117] + tmpFx[98]*tmpObjS[129] + tmpFx[107]*tmpObjS[141];
tmpS2[106] = + tmpFx[8]*tmpObjS[10] + tmpFx[17]*tmpObjS[22] + tmpFx[26]*tmpObjS[34] + tmpFx[35]*tmpObjS[46] + tmpFx[44]*tmpObjS[58] + tmpFx[53]*tmpObjS[70] + tmpFx[62]*tmpObjS[82] + tmpFx[71]*tmpObjS[94] + tmpFx[80]*tmpObjS[106] + tmpFx[89]*tmpObjS[118] + tmpFx[98]*tmpObjS[130] + tmpFx[107]*tmpObjS[142];
tmpS2[107] = + tmpFx[8]*tmpObjS[11] + tmpFx[17]*tmpObjS[23] + tmpFx[26]*tmpObjS[35] + tmpFx[35]*tmpObjS[47] + tmpFx[44]*tmpObjS[59] + tmpFx[53]*tmpObjS[71] + tmpFx[62]*tmpObjS[83] + tmpFx[71]*tmpObjS[95] + tmpFx[80]*tmpObjS[107] + tmpFx[89]*tmpObjS[119] + tmpFx[98]*tmpObjS[131] + tmpFx[107]*tmpObjS[143];
tmpS1[0] = + tmpS2[0]*tmpFu[0] + tmpS2[1]*tmpFu[3] + tmpS2[2]*tmpFu[6] + tmpS2[3]*tmpFu[9] + tmpS2[4]*tmpFu[12] + tmpS2[5]*tmpFu[15] + tmpS2[6]*tmpFu[18] + tmpS2[7]*tmpFu[21] + tmpS2[8]*tmpFu[24] + tmpS2[9]*tmpFu[27] + tmpS2[10]*tmpFu[30] + tmpS2[11]*tmpFu[33];
tmpS1[1] = + tmpS2[0]*tmpFu[1] + tmpS2[1]*tmpFu[4] + tmpS2[2]*tmpFu[7] + tmpS2[3]*tmpFu[10] + tmpS2[4]*tmpFu[13] + tmpS2[5]*tmpFu[16] + tmpS2[6]*tmpFu[19] + tmpS2[7]*tmpFu[22] + tmpS2[8]*tmpFu[25] + tmpS2[9]*tmpFu[28] + tmpS2[10]*tmpFu[31] + tmpS2[11]*tmpFu[34];
tmpS1[2] = + tmpS2[0]*tmpFu[2] + tmpS2[1]*tmpFu[5] + tmpS2[2]*tmpFu[8] + tmpS2[3]*tmpFu[11] + tmpS2[4]*tmpFu[14] + tmpS2[5]*tmpFu[17] + tmpS2[6]*tmpFu[20] + tmpS2[7]*tmpFu[23] + tmpS2[8]*tmpFu[26] + tmpS2[9]*tmpFu[29] + tmpS2[10]*tmpFu[32] + tmpS2[11]*tmpFu[35];
tmpS1[3] = + tmpS2[12]*tmpFu[0] + tmpS2[13]*tmpFu[3] + tmpS2[14]*tmpFu[6] + tmpS2[15]*tmpFu[9] + tmpS2[16]*tmpFu[12] + tmpS2[17]*tmpFu[15] + tmpS2[18]*tmpFu[18] + tmpS2[19]*tmpFu[21] + tmpS2[20]*tmpFu[24] + tmpS2[21]*tmpFu[27] + tmpS2[22]*tmpFu[30] + tmpS2[23]*tmpFu[33];
tmpS1[4] = + tmpS2[12]*tmpFu[1] + tmpS2[13]*tmpFu[4] + tmpS2[14]*tmpFu[7] + tmpS2[15]*tmpFu[10] + tmpS2[16]*tmpFu[13] + tmpS2[17]*tmpFu[16] + tmpS2[18]*tmpFu[19] + tmpS2[19]*tmpFu[22] + tmpS2[20]*tmpFu[25] + tmpS2[21]*tmpFu[28] + tmpS2[22]*tmpFu[31] + tmpS2[23]*tmpFu[34];
tmpS1[5] = + tmpS2[12]*tmpFu[2] + tmpS2[13]*tmpFu[5] + tmpS2[14]*tmpFu[8] + tmpS2[15]*tmpFu[11] + tmpS2[16]*tmpFu[14] + tmpS2[17]*tmpFu[17] + tmpS2[18]*tmpFu[20] + tmpS2[19]*tmpFu[23] + tmpS2[20]*tmpFu[26] + tmpS2[21]*tmpFu[29] + tmpS2[22]*tmpFu[32] + tmpS2[23]*tmpFu[35];
tmpS1[6] = + tmpS2[24]*tmpFu[0] + tmpS2[25]*tmpFu[3] + tmpS2[26]*tmpFu[6] + tmpS2[27]*tmpFu[9] + tmpS2[28]*tmpFu[12] + tmpS2[29]*tmpFu[15] + tmpS2[30]*tmpFu[18] + tmpS2[31]*tmpFu[21] + tmpS2[32]*tmpFu[24] + tmpS2[33]*tmpFu[27] + tmpS2[34]*tmpFu[30] + tmpS2[35]*tmpFu[33];
tmpS1[7] = + tmpS2[24]*tmpFu[1] + tmpS2[25]*tmpFu[4] + tmpS2[26]*tmpFu[7] + tmpS2[27]*tmpFu[10] + tmpS2[28]*tmpFu[13] + tmpS2[29]*tmpFu[16] + tmpS2[30]*tmpFu[19] + tmpS2[31]*tmpFu[22] + tmpS2[32]*tmpFu[25] + tmpS2[33]*tmpFu[28] + tmpS2[34]*tmpFu[31] + tmpS2[35]*tmpFu[34];
tmpS1[8] = + tmpS2[24]*tmpFu[2] + tmpS2[25]*tmpFu[5] + tmpS2[26]*tmpFu[8] + tmpS2[27]*tmpFu[11] + tmpS2[28]*tmpFu[14] + tmpS2[29]*tmpFu[17] + tmpS2[30]*tmpFu[20] + tmpS2[31]*tmpFu[23] + tmpS2[32]*tmpFu[26] + tmpS2[33]*tmpFu[29] + tmpS2[34]*tmpFu[32] + tmpS2[35]*tmpFu[35];
tmpS1[9] = + tmpS2[36]*tmpFu[0] + tmpS2[37]*tmpFu[3] + tmpS2[38]*tmpFu[6] + tmpS2[39]*tmpFu[9] + tmpS2[40]*tmpFu[12] + tmpS2[41]*tmpFu[15] + tmpS2[42]*tmpFu[18] + tmpS2[43]*tmpFu[21] + tmpS2[44]*tmpFu[24] + tmpS2[45]*tmpFu[27] + tmpS2[46]*tmpFu[30] + tmpS2[47]*tmpFu[33];
tmpS1[10] = + tmpS2[36]*tmpFu[1] + tmpS2[37]*tmpFu[4] + tmpS2[38]*tmpFu[7] + tmpS2[39]*tmpFu[10] + tmpS2[40]*tmpFu[13] + tmpS2[41]*tmpFu[16] + tmpS2[42]*tmpFu[19] + tmpS2[43]*tmpFu[22] + tmpS2[44]*tmpFu[25] + tmpS2[45]*tmpFu[28] + tmpS2[46]*tmpFu[31] + tmpS2[47]*tmpFu[34];
tmpS1[11] = + tmpS2[36]*tmpFu[2] + tmpS2[37]*tmpFu[5] + tmpS2[38]*tmpFu[8] + tmpS2[39]*tmpFu[11] + tmpS2[40]*tmpFu[14] + tmpS2[41]*tmpFu[17] + tmpS2[42]*tmpFu[20] + tmpS2[43]*tmpFu[23] + tmpS2[44]*tmpFu[26] + tmpS2[45]*tmpFu[29] + tmpS2[46]*tmpFu[32] + tmpS2[47]*tmpFu[35];
tmpS1[12] = + tmpS2[48]*tmpFu[0] + tmpS2[49]*tmpFu[3] + tmpS2[50]*tmpFu[6] + tmpS2[51]*tmpFu[9] + tmpS2[52]*tmpFu[12] + tmpS2[53]*tmpFu[15] + tmpS2[54]*tmpFu[18] + tmpS2[55]*tmpFu[21] + tmpS2[56]*tmpFu[24] + tmpS2[57]*tmpFu[27] + tmpS2[58]*tmpFu[30] + tmpS2[59]*tmpFu[33];
tmpS1[13] = + tmpS2[48]*tmpFu[1] + tmpS2[49]*tmpFu[4] + tmpS2[50]*tmpFu[7] + tmpS2[51]*tmpFu[10] + tmpS2[52]*tmpFu[13] + tmpS2[53]*tmpFu[16] + tmpS2[54]*tmpFu[19] + tmpS2[55]*tmpFu[22] + tmpS2[56]*tmpFu[25] + tmpS2[57]*tmpFu[28] + tmpS2[58]*tmpFu[31] + tmpS2[59]*tmpFu[34];
tmpS1[14] = + tmpS2[48]*tmpFu[2] + tmpS2[49]*tmpFu[5] + tmpS2[50]*tmpFu[8] + tmpS2[51]*tmpFu[11] + tmpS2[52]*tmpFu[14] + tmpS2[53]*tmpFu[17] + tmpS2[54]*tmpFu[20] + tmpS2[55]*tmpFu[23] + tmpS2[56]*tmpFu[26] + tmpS2[57]*tmpFu[29] + tmpS2[58]*tmpFu[32] + tmpS2[59]*tmpFu[35];
tmpS1[15] = + tmpS2[60]*tmpFu[0] + tmpS2[61]*tmpFu[3] + tmpS2[62]*tmpFu[6] + tmpS2[63]*tmpFu[9] + tmpS2[64]*tmpFu[12] + tmpS2[65]*tmpFu[15] + tmpS2[66]*tmpFu[18] + tmpS2[67]*tmpFu[21] + tmpS2[68]*tmpFu[24] + tmpS2[69]*tmpFu[27] + tmpS2[70]*tmpFu[30] + tmpS2[71]*tmpFu[33];
tmpS1[16] = + tmpS2[60]*tmpFu[1] + tmpS2[61]*tmpFu[4] + tmpS2[62]*tmpFu[7] + tmpS2[63]*tmpFu[10] + tmpS2[64]*tmpFu[13] + tmpS2[65]*tmpFu[16] + tmpS2[66]*tmpFu[19] + tmpS2[67]*tmpFu[22] + tmpS2[68]*tmpFu[25] + tmpS2[69]*tmpFu[28] + tmpS2[70]*tmpFu[31] + tmpS2[71]*tmpFu[34];
tmpS1[17] = + tmpS2[60]*tmpFu[2] + tmpS2[61]*tmpFu[5] + tmpS2[62]*tmpFu[8] + tmpS2[63]*tmpFu[11] + tmpS2[64]*tmpFu[14] + tmpS2[65]*tmpFu[17] + tmpS2[66]*tmpFu[20] + tmpS2[67]*tmpFu[23] + tmpS2[68]*tmpFu[26] + tmpS2[69]*tmpFu[29] + tmpS2[70]*tmpFu[32] + tmpS2[71]*tmpFu[35];
tmpS1[18] = + tmpS2[72]*tmpFu[0] + tmpS2[73]*tmpFu[3] + tmpS2[74]*tmpFu[6] + tmpS2[75]*tmpFu[9] + tmpS2[76]*tmpFu[12] + tmpS2[77]*tmpFu[15] + tmpS2[78]*tmpFu[18] + tmpS2[79]*tmpFu[21] + tmpS2[80]*tmpFu[24] + tmpS2[81]*tmpFu[27] + tmpS2[82]*tmpFu[30] + tmpS2[83]*tmpFu[33];
tmpS1[19] = + tmpS2[72]*tmpFu[1] + tmpS2[73]*tmpFu[4] + tmpS2[74]*tmpFu[7] + tmpS2[75]*tmpFu[10] + tmpS2[76]*tmpFu[13] + tmpS2[77]*tmpFu[16] + tmpS2[78]*tmpFu[19] + tmpS2[79]*tmpFu[22] + tmpS2[80]*tmpFu[25] + tmpS2[81]*tmpFu[28] + tmpS2[82]*tmpFu[31] + tmpS2[83]*tmpFu[34];
tmpS1[20] = + tmpS2[72]*tmpFu[2] + tmpS2[73]*tmpFu[5] + tmpS2[74]*tmpFu[8] + tmpS2[75]*tmpFu[11] + tmpS2[76]*tmpFu[14] + tmpS2[77]*tmpFu[17] + tmpS2[78]*tmpFu[20] + tmpS2[79]*tmpFu[23] + tmpS2[80]*tmpFu[26] + tmpS2[81]*tmpFu[29] + tmpS2[82]*tmpFu[32] + tmpS2[83]*tmpFu[35];
tmpS1[21] = + tmpS2[84]*tmpFu[0] + tmpS2[85]*tmpFu[3] + tmpS2[86]*tmpFu[6] + tmpS2[87]*tmpFu[9] + tmpS2[88]*tmpFu[12] + tmpS2[89]*tmpFu[15] + tmpS2[90]*tmpFu[18] + tmpS2[91]*tmpFu[21] + tmpS2[92]*tmpFu[24] + tmpS2[93]*tmpFu[27] + tmpS2[94]*tmpFu[30] + tmpS2[95]*tmpFu[33];
tmpS1[22] = + tmpS2[84]*tmpFu[1] + tmpS2[85]*tmpFu[4] + tmpS2[86]*tmpFu[7] + tmpS2[87]*tmpFu[10] + tmpS2[88]*tmpFu[13] + tmpS2[89]*tmpFu[16] + tmpS2[90]*tmpFu[19] + tmpS2[91]*tmpFu[22] + tmpS2[92]*tmpFu[25] + tmpS2[93]*tmpFu[28] + tmpS2[94]*tmpFu[31] + tmpS2[95]*tmpFu[34];
tmpS1[23] = + tmpS2[84]*tmpFu[2] + tmpS2[85]*tmpFu[5] + tmpS2[86]*tmpFu[8] + tmpS2[87]*tmpFu[11] + tmpS2[88]*tmpFu[14] + tmpS2[89]*tmpFu[17] + tmpS2[90]*tmpFu[20] + tmpS2[91]*tmpFu[23] + tmpS2[92]*tmpFu[26] + tmpS2[93]*tmpFu[29] + tmpS2[94]*tmpFu[32] + tmpS2[95]*tmpFu[35];
tmpS1[24] = + tmpS2[96]*tmpFu[0] + tmpS2[97]*tmpFu[3] + tmpS2[98]*tmpFu[6] + tmpS2[99]*tmpFu[9] + tmpS2[100]*tmpFu[12] + tmpS2[101]*tmpFu[15] + tmpS2[102]*tmpFu[18] + tmpS2[103]*tmpFu[21] + tmpS2[104]*tmpFu[24] + tmpS2[105]*tmpFu[27] + tmpS2[106]*tmpFu[30] + tmpS2[107]*tmpFu[33];
tmpS1[25] = + tmpS2[96]*tmpFu[1] + tmpS2[97]*tmpFu[4] + tmpS2[98]*tmpFu[7] + tmpS2[99]*tmpFu[10] + tmpS2[100]*tmpFu[13] + tmpS2[101]*tmpFu[16] + tmpS2[102]*tmpFu[19] + tmpS2[103]*tmpFu[22] + tmpS2[104]*tmpFu[25] + tmpS2[105]*tmpFu[28] + tmpS2[106]*tmpFu[31] + tmpS2[107]*tmpFu[34];
tmpS1[26] = + tmpS2[96]*tmpFu[2] + tmpS2[97]*tmpFu[5] + tmpS2[98]*tmpFu[8] + tmpS2[99]*tmpFu[11] + tmpS2[100]*tmpFu[14] + tmpS2[101]*tmpFu[17] + tmpS2[102]*tmpFu[20] + tmpS2[103]*tmpFu[23] + tmpS2[104]*tmpFu[26] + tmpS2[105]*tmpFu[29] + tmpS2[106]*tmpFu[32] + tmpS2[107]*tmpFu[35];
}

void acado_setObjQN1QN2( real_t* const tmpFx, real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = + tmpFx[0]*tmpObjSEndTerm[0] + tmpFx[9]*tmpObjSEndTerm[7] + tmpFx[18]*tmpObjSEndTerm[14] + tmpFx[27]*tmpObjSEndTerm[21] + tmpFx[36]*tmpObjSEndTerm[28] + tmpFx[45]*tmpObjSEndTerm[35] + tmpFx[54]*tmpObjSEndTerm[42];
tmpQN2[1] = + tmpFx[0]*tmpObjSEndTerm[1] + tmpFx[9]*tmpObjSEndTerm[8] + tmpFx[18]*tmpObjSEndTerm[15] + tmpFx[27]*tmpObjSEndTerm[22] + tmpFx[36]*tmpObjSEndTerm[29] + tmpFx[45]*tmpObjSEndTerm[36] + tmpFx[54]*tmpObjSEndTerm[43];
tmpQN2[2] = + tmpFx[0]*tmpObjSEndTerm[2] + tmpFx[9]*tmpObjSEndTerm[9] + tmpFx[18]*tmpObjSEndTerm[16] + tmpFx[27]*tmpObjSEndTerm[23] + tmpFx[36]*tmpObjSEndTerm[30] + tmpFx[45]*tmpObjSEndTerm[37] + tmpFx[54]*tmpObjSEndTerm[44];
tmpQN2[3] = + tmpFx[0]*tmpObjSEndTerm[3] + tmpFx[9]*tmpObjSEndTerm[10] + tmpFx[18]*tmpObjSEndTerm[17] + tmpFx[27]*tmpObjSEndTerm[24] + tmpFx[36]*tmpObjSEndTerm[31] + tmpFx[45]*tmpObjSEndTerm[38] + tmpFx[54]*tmpObjSEndTerm[45];
tmpQN2[4] = + tmpFx[0]*tmpObjSEndTerm[4] + tmpFx[9]*tmpObjSEndTerm[11] + tmpFx[18]*tmpObjSEndTerm[18] + tmpFx[27]*tmpObjSEndTerm[25] + tmpFx[36]*tmpObjSEndTerm[32] + tmpFx[45]*tmpObjSEndTerm[39] + tmpFx[54]*tmpObjSEndTerm[46];
tmpQN2[5] = + tmpFx[0]*tmpObjSEndTerm[5] + tmpFx[9]*tmpObjSEndTerm[12] + tmpFx[18]*tmpObjSEndTerm[19] + tmpFx[27]*tmpObjSEndTerm[26] + tmpFx[36]*tmpObjSEndTerm[33] + tmpFx[45]*tmpObjSEndTerm[40] + tmpFx[54]*tmpObjSEndTerm[47];
tmpQN2[6] = + tmpFx[0]*tmpObjSEndTerm[6] + tmpFx[9]*tmpObjSEndTerm[13] + tmpFx[18]*tmpObjSEndTerm[20] + tmpFx[27]*tmpObjSEndTerm[27] + tmpFx[36]*tmpObjSEndTerm[34] + tmpFx[45]*tmpObjSEndTerm[41] + tmpFx[54]*tmpObjSEndTerm[48];
tmpQN2[7] = + tmpFx[1]*tmpObjSEndTerm[0] + tmpFx[10]*tmpObjSEndTerm[7] + tmpFx[19]*tmpObjSEndTerm[14] + tmpFx[28]*tmpObjSEndTerm[21] + tmpFx[37]*tmpObjSEndTerm[28] + tmpFx[46]*tmpObjSEndTerm[35] + tmpFx[55]*tmpObjSEndTerm[42];
tmpQN2[8] = + tmpFx[1]*tmpObjSEndTerm[1] + tmpFx[10]*tmpObjSEndTerm[8] + tmpFx[19]*tmpObjSEndTerm[15] + tmpFx[28]*tmpObjSEndTerm[22] + tmpFx[37]*tmpObjSEndTerm[29] + tmpFx[46]*tmpObjSEndTerm[36] + tmpFx[55]*tmpObjSEndTerm[43];
tmpQN2[9] = + tmpFx[1]*tmpObjSEndTerm[2] + tmpFx[10]*tmpObjSEndTerm[9] + tmpFx[19]*tmpObjSEndTerm[16] + tmpFx[28]*tmpObjSEndTerm[23] + tmpFx[37]*tmpObjSEndTerm[30] + tmpFx[46]*tmpObjSEndTerm[37] + tmpFx[55]*tmpObjSEndTerm[44];
tmpQN2[10] = + tmpFx[1]*tmpObjSEndTerm[3] + tmpFx[10]*tmpObjSEndTerm[10] + tmpFx[19]*tmpObjSEndTerm[17] + tmpFx[28]*tmpObjSEndTerm[24] + tmpFx[37]*tmpObjSEndTerm[31] + tmpFx[46]*tmpObjSEndTerm[38] + tmpFx[55]*tmpObjSEndTerm[45];
tmpQN2[11] = + tmpFx[1]*tmpObjSEndTerm[4] + tmpFx[10]*tmpObjSEndTerm[11] + tmpFx[19]*tmpObjSEndTerm[18] + tmpFx[28]*tmpObjSEndTerm[25] + tmpFx[37]*tmpObjSEndTerm[32] + tmpFx[46]*tmpObjSEndTerm[39] + tmpFx[55]*tmpObjSEndTerm[46];
tmpQN2[12] = + tmpFx[1]*tmpObjSEndTerm[5] + tmpFx[10]*tmpObjSEndTerm[12] + tmpFx[19]*tmpObjSEndTerm[19] + tmpFx[28]*tmpObjSEndTerm[26] + tmpFx[37]*tmpObjSEndTerm[33] + tmpFx[46]*tmpObjSEndTerm[40] + tmpFx[55]*tmpObjSEndTerm[47];
tmpQN2[13] = + tmpFx[1]*tmpObjSEndTerm[6] + tmpFx[10]*tmpObjSEndTerm[13] + tmpFx[19]*tmpObjSEndTerm[20] + tmpFx[28]*tmpObjSEndTerm[27] + tmpFx[37]*tmpObjSEndTerm[34] + tmpFx[46]*tmpObjSEndTerm[41] + tmpFx[55]*tmpObjSEndTerm[48];
tmpQN2[14] = + tmpFx[2]*tmpObjSEndTerm[0] + tmpFx[11]*tmpObjSEndTerm[7] + tmpFx[20]*tmpObjSEndTerm[14] + tmpFx[29]*tmpObjSEndTerm[21] + tmpFx[38]*tmpObjSEndTerm[28] + tmpFx[47]*tmpObjSEndTerm[35] + tmpFx[56]*tmpObjSEndTerm[42];
tmpQN2[15] = + tmpFx[2]*tmpObjSEndTerm[1] + tmpFx[11]*tmpObjSEndTerm[8] + tmpFx[20]*tmpObjSEndTerm[15] + tmpFx[29]*tmpObjSEndTerm[22] + tmpFx[38]*tmpObjSEndTerm[29] + tmpFx[47]*tmpObjSEndTerm[36] + tmpFx[56]*tmpObjSEndTerm[43];
tmpQN2[16] = + tmpFx[2]*tmpObjSEndTerm[2] + tmpFx[11]*tmpObjSEndTerm[9] + tmpFx[20]*tmpObjSEndTerm[16] + tmpFx[29]*tmpObjSEndTerm[23] + tmpFx[38]*tmpObjSEndTerm[30] + tmpFx[47]*tmpObjSEndTerm[37] + tmpFx[56]*tmpObjSEndTerm[44];
tmpQN2[17] = + tmpFx[2]*tmpObjSEndTerm[3] + tmpFx[11]*tmpObjSEndTerm[10] + tmpFx[20]*tmpObjSEndTerm[17] + tmpFx[29]*tmpObjSEndTerm[24] + tmpFx[38]*tmpObjSEndTerm[31] + tmpFx[47]*tmpObjSEndTerm[38] + tmpFx[56]*tmpObjSEndTerm[45];
tmpQN2[18] = + tmpFx[2]*tmpObjSEndTerm[4] + tmpFx[11]*tmpObjSEndTerm[11] + tmpFx[20]*tmpObjSEndTerm[18] + tmpFx[29]*tmpObjSEndTerm[25] + tmpFx[38]*tmpObjSEndTerm[32] + tmpFx[47]*tmpObjSEndTerm[39] + tmpFx[56]*tmpObjSEndTerm[46];
tmpQN2[19] = + tmpFx[2]*tmpObjSEndTerm[5] + tmpFx[11]*tmpObjSEndTerm[12] + tmpFx[20]*tmpObjSEndTerm[19] + tmpFx[29]*tmpObjSEndTerm[26] + tmpFx[38]*tmpObjSEndTerm[33] + tmpFx[47]*tmpObjSEndTerm[40] + tmpFx[56]*tmpObjSEndTerm[47];
tmpQN2[20] = + tmpFx[2]*tmpObjSEndTerm[6] + tmpFx[11]*tmpObjSEndTerm[13] + tmpFx[20]*tmpObjSEndTerm[20] + tmpFx[29]*tmpObjSEndTerm[27] + tmpFx[38]*tmpObjSEndTerm[34] + tmpFx[47]*tmpObjSEndTerm[41] + tmpFx[56]*tmpObjSEndTerm[48];
tmpQN2[21] = + tmpFx[3]*tmpObjSEndTerm[0] + tmpFx[12]*tmpObjSEndTerm[7] + tmpFx[21]*tmpObjSEndTerm[14] + tmpFx[30]*tmpObjSEndTerm[21] + tmpFx[39]*tmpObjSEndTerm[28] + tmpFx[48]*tmpObjSEndTerm[35] + tmpFx[57]*tmpObjSEndTerm[42];
tmpQN2[22] = + tmpFx[3]*tmpObjSEndTerm[1] + tmpFx[12]*tmpObjSEndTerm[8] + tmpFx[21]*tmpObjSEndTerm[15] + tmpFx[30]*tmpObjSEndTerm[22] + tmpFx[39]*tmpObjSEndTerm[29] + tmpFx[48]*tmpObjSEndTerm[36] + tmpFx[57]*tmpObjSEndTerm[43];
tmpQN2[23] = + tmpFx[3]*tmpObjSEndTerm[2] + tmpFx[12]*tmpObjSEndTerm[9] + tmpFx[21]*tmpObjSEndTerm[16] + tmpFx[30]*tmpObjSEndTerm[23] + tmpFx[39]*tmpObjSEndTerm[30] + tmpFx[48]*tmpObjSEndTerm[37] + tmpFx[57]*tmpObjSEndTerm[44];
tmpQN2[24] = + tmpFx[3]*tmpObjSEndTerm[3] + tmpFx[12]*tmpObjSEndTerm[10] + tmpFx[21]*tmpObjSEndTerm[17] + tmpFx[30]*tmpObjSEndTerm[24] + tmpFx[39]*tmpObjSEndTerm[31] + tmpFx[48]*tmpObjSEndTerm[38] + tmpFx[57]*tmpObjSEndTerm[45];
tmpQN2[25] = + tmpFx[3]*tmpObjSEndTerm[4] + tmpFx[12]*tmpObjSEndTerm[11] + tmpFx[21]*tmpObjSEndTerm[18] + tmpFx[30]*tmpObjSEndTerm[25] + tmpFx[39]*tmpObjSEndTerm[32] + tmpFx[48]*tmpObjSEndTerm[39] + tmpFx[57]*tmpObjSEndTerm[46];
tmpQN2[26] = + tmpFx[3]*tmpObjSEndTerm[5] + tmpFx[12]*tmpObjSEndTerm[12] + tmpFx[21]*tmpObjSEndTerm[19] + tmpFx[30]*tmpObjSEndTerm[26] + tmpFx[39]*tmpObjSEndTerm[33] + tmpFx[48]*tmpObjSEndTerm[40] + tmpFx[57]*tmpObjSEndTerm[47];
tmpQN2[27] = + tmpFx[3]*tmpObjSEndTerm[6] + tmpFx[12]*tmpObjSEndTerm[13] + tmpFx[21]*tmpObjSEndTerm[20] + tmpFx[30]*tmpObjSEndTerm[27] + tmpFx[39]*tmpObjSEndTerm[34] + tmpFx[48]*tmpObjSEndTerm[41] + tmpFx[57]*tmpObjSEndTerm[48];
tmpQN2[28] = + tmpFx[4]*tmpObjSEndTerm[0] + tmpFx[13]*tmpObjSEndTerm[7] + tmpFx[22]*tmpObjSEndTerm[14] + tmpFx[31]*tmpObjSEndTerm[21] + tmpFx[40]*tmpObjSEndTerm[28] + tmpFx[49]*tmpObjSEndTerm[35] + tmpFx[58]*tmpObjSEndTerm[42];
tmpQN2[29] = + tmpFx[4]*tmpObjSEndTerm[1] + tmpFx[13]*tmpObjSEndTerm[8] + tmpFx[22]*tmpObjSEndTerm[15] + tmpFx[31]*tmpObjSEndTerm[22] + tmpFx[40]*tmpObjSEndTerm[29] + tmpFx[49]*tmpObjSEndTerm[36] + tmpFx[58]*tmpObjSEndTerm[43];
tmpQN2[30] = + tmpFx[4]*tmpObjSEndTerm[2] + tmpFx[13]*tmpObjSEndTerm[9] + tmpFx[22]*tmpObjSEndTerm[16] + tmpFx[31]*tmpObjSEndTerm[23] + tmpFx[40]*tmpObjSEndTerm[30] + tmpFx[49]*tmpObjSEndTerm[37] + tmpFx[58]*tmpObjSEndTerm[44];
tmpQN2[31] = + tmpFx[4]*tmpObjSEndTerm[3] + tmpFx[13]*tmpObjSEndTerm[10] + tmpFx[22]*tmpObjSEndTerm[17] + tmpFx[31]*tmpObjSEndTerm[24] + tmpFx[40]*tmpObjSEndTerm[31] + tmpFx[49]*tmpObjSEndTerm[38] + tmpFx[58]*tmpObjSEndTerm[45];
tmpQN2[32] = + tmpFx[4]*tmpObjSEndTerm[4] + tmpFx[13]*tmpObjSEndTerm[11] + tmpFx[22]*tmpObjSEndTerm[18] + tmpFx[31]*tmpObjSEndTerm[25] + tmpFx[40]*tmpObjSEndTerm[32] + tmpFx[49]*tmpObjSEndTerm[39] + tmpFx[58]*tmpObjSEndTerm[46];
tmpQN2[33] = + tmpFx[4]*tmpObjSEndTerm[5] + tmpFx[13]*tmpObjSEndTerm[12] + tmpFx[22]*tmpObjSEndTerm[19] + tmpFx[31]*tmpObjSEndTerm[26] + tmpFx[40]*tmpObjSEndTerm[33] + tmpFx[49]*tmpObjSEndTerm[40] + tmpFx[58]*tmpObjSEndTerm[47];
tmpQN2[34] = + tmpFx[4]*tmpObjSEndTerm[6] + tmpFx[13]*tmpObjSEndTerm[13] + tmpFx[22]*tmpObjSEndTerm[20] + tmpFx[31]*tmpObjSEndTerm[27] + tmpFx[40]*tmpObjSEndTerm[34] + tmpFx[49]*tmpObjSEndTerm[41] + tmpFx[58]*tmpObjSEndTerm[48];
tmpQN2[35] = + tmpFx[5]*tmpObjSEndTerm[0] + tmpFx[14]*tmpObjSEndTerm[7] + tmpFx[23]*tmpObjSEndTerm[14] + tmpFx[32]*tmpObjSEndTerm[21] + tmpFx[41]*tmpObjSEndTerm[28] + tmpFx[50]*tmpObjSEndTerm[35] + tmpFx[59]*tmpObjSEndTerm[42];
tmpQN2[36] = + tmpFx[5]*tmpObjSEndTerm[1] + tmpFx[14]*tmpObjSEndTerm[8] + tmpFx[23]*tmpObjSEndTerm[15] + tmpFx[32]*tmpObjSEndTerm[22] + tmpFx[41]*tmpObjSEndTerm[29] + tmpFx[50]*tmpObjSEndTerm[36] + tmpFx[59]*tmpObjSEndTerm[43];
tmpQN2[37] = + tmpFx[5]*tmpObjSEndTerm[2] + tmpFx[14]*tmpObjSEndTerm[9] + tmpFx[23]*tmpObjSEndTerm[16] + tmpFx[32]*tmpObjSEndTerm[23] + tmpFx[41]*tmpObjSEndTerm[30] + tmpFx[50]*tmpObjSEndTerm[37] + tmpFx[59]*tmpObjSEndTerm[44];
tmpQN2[38] = + tmpFx[5]*tmpObjSEndTerm[3] + tmpFx[14]*tmpObjSEndTerm[10] + tmpFx[23]*tmpObjSEndTerm[17] + tmpFx[32]*tmpObjSEndTerm[24] + tmpFx[41]*tmpObjSEndTerm[31] + tmpFx[50]*tmpObjSEndTerm[38] + tmpFx[59]*tmpObjSEndTerm[45];
tmpQN2[39] = + tmpFx[5]*tmpObjSEndTerm[4] + tmpFx[14]*tmpObjSEndTerm[11] + tmpFx[23]*tmpObjSEndTerm[18] + tmpFx[32]*tmpObjSEndTerm[25] + tmpFx[41]*tmpObjSEndTerm[32] + tmpFx[50]*tmpObjSEndTerm[39] + tmpFx[59]*tmpObjSEndTerm[46];
tmpQN2[40] = + tmpFx[5]*tmpObjSEndTerm[5] + tmpFx[14]*tmpObjSEndTerm[12] + tmpFx[23]*tmpObjSEndTerm[19] + tmpFx[32]*tmpObjSEndTerm[26] + tmpFx[41]*tmpObjSEndTerm[33] + tmpFx[50]*tmpObjSEndTerm[40] + tmpFx[59]*tmpObjSEndTerm[47];
tmpQN2[41] = + tmpFx[5]*tmpObjSEndTerm[6] + tmpFx[14]*tmpObjSEndTerm[13] + tmpFx[23]*tmpObjSEndTerm[20] + tmpFx[32]*tmpObjSEndTerm[27] + tmpFx[41]*tmpObjSEndTerm[34] + tmpFx[50]*tmpObjSEndTerm[41] + tmpFx[59]*tmpObjSEndTerm[48];
tmpQN2[42] = + tmpFx[6]*tmpObjSEndTerm[0] + tmpFx[15]*tmpObjSEndTerm[7] + tmpFx[24]*tmpObjSEndTerm[14] + tmpFx[33]*tmpObjSEndTerm[21] + tmpFx[42]*tmpObjSEndTerm[28] + tmpFx[51]*tmpObjSEndTerm[35] + tmpFx[60]*tmpObjSEndTerm[42];
tmpQN2[43] = + tmpFx[6]*tmpObjSEndTerm[1] + tmpFx[15]*tmpObjSEndTerm[8] + tmpFx[24]*tmpObjSEndTerm[15] + tmpFx[33]*tmpObjSEndTerm[22] + tmpFx[42]*tmpObjSEndTerm[29] + tmpFx[51]*tmpObjSEndTerm[36] + tmpFx[60]*tmpObjSEndTerm[43];
tmpQN2[44] = + tmpFx[6]*tmpObjSEndTerm[2] + tmpFx[15]*tmpObjSEndTerm[9] + tmpFx[24]*tmpObjSEndTerm[16] + tmpFx[33]*tmpObjSEndTerm[23] + tmpFx[42]*tmpObjSEndTerm[30] + tmpFx[51]*tmpObjSEndTerm[37] + tmpFx[60]*tmpObjSEndTerm[44];
tmpQN2[45] = + tmpFx[6]*tmpObjSEndTerm[3] + tmpFx[15]*tmpObjSEndTerm[10] + tmpFx[24]*tmpObjSEndTerm[17] + tmpFx[33]*tmpObjSEndTerm[24] + tmpFx[42]*tmpObjSEndTerm[31] + tmpFx[51]*tmpObjSEndTerm[38] + tmpFx[60]*tmpObjSEndTerm[45];
tmpQN2[46] = + tmpFx[6]*tmpObjSEndTerm[4] + tmpFx[15]*tmpObjSEndTerm[11] + tmpFx[24]*tmpObjSEndTerm[18] + tmpFx[33]*tmpObjSEndTerm[25] + tmpFx[42]*tmpObjSEndTerm[32] + tmpFx[51]*tmpObjSEndTerm[39] + tmpFx[60]*tmpObjSEndTerm[46];
tmpQN2[47] = + tmpFx[6]*tmpObjSEndTerm[5] + tmpFx[15]*tmpObjSEndTerm[12] + tmpFx[24]*tmpObjSEndTerm[19] + tmpFx[33]*tmpObjSEndTerm[26] + tmpFx[42]*tmpObjSEndTerm[33] + tmpFx[51]*tmpObjSEndTerm[40] + tmpFx[60]*tmpObjSEndTerm[47];
tmpQN2[48] = + tmpFx[6]*tmpObjSEndTerm[6] + tmpFx[15]*tmpObjSEndTerm[13] + tmpFx[24]*tmpObjSEndTerm[20] + tmpFx[33]*tmpObjSEndTerm[27] + tmpFx[42]*tmpObjSEndTerm[34] + tmpFx[51]*tmpObjSEndTerm[41] + tmpFx[60]*tmpObjSEndTerm[48];
tmpQN2[49] = + tmpFx[7]*tmpObjSEndTerm[0] + tmpFx[16]*tmpObjSEndTerm[7] + tmpFx[25]*tmpObjSEndTerm[14] + tmpFx[34]*tmpObjSEndTerm[21] + tmpFx[43]*tmpObjSEndTerm[28] + tmpFx[52]*tmpObjSEndTerm[35] + tmpFx[61]*tmpObjSEndTerm[42];
tmpQN2[50] = + tmpFx[7]*tmpObjSEndTerm[1] + tmpFx[16]*tmpObjSEndTerm[8] + tmpFx[25]*tmpObjSEndTerm[15] + tmpFx[34]*tmpObjSEndTerm[22] + tmpFx[43]*tmpObjSEndTerm[29] + tmpFx[52]*tmpObjSEndTerm[36] + tmpFx[61]*tmpObjSEndTerm[43];
tmpQN2[51] = + tmpFx[7]*tmpObjSEndTerm[2] + tmpFx[16]*tmpObjSEndTerm[9] + tmpFx[25]*tmpObjSEndTerm[16] + tmpFx[34]*tmpObjSEndTerm[23] + tmpFx[43]*tmpObjSEndTerm[30] + tmpFx[52]*tmpObjSEndTerm[37] + tmpFx[61]*tmpObjSEndTerm[44];
tmpQN2[52] = + tmpFx[7]*tmpObjSEndTerm[3] + tmpFx[16]*tmpObjSEndTerm[10] + tmpFx[25]*tmpObjSEndTerm[17] + tmpFx[34]*tmpObjSEndTerm[24] + tmpFx[43]*tmpObjSEndTerm[31] + tmpFx[52]*tmpObjSEndTerm[38] + tmpFx[61]*tmpObjSEndTerm[45];
tmpQN2[53] = + tmpFx[7]*tmpObjSEndTerm[4] + tmpFx[16]*tmpObjSEndTerm[11] + tmpFx[25]*tmpObjSEndTerm[18] + tmpFx[34]*tmpObjSEndTerm[25] + tmpFx[43]*tmpObjSEndTerm[32] + tmpFx[52]*tmpObjSEndTerm[39] + tmpFx[61]*tmpObjSEndTerm[46];
tmpQN2[54] = + tmpFx[7]*tmpObjSEndTerm[5] + tmpFx[16]*tmpObjSEndTerm[12] + tmpFx[25]*tmpObjSEndTerm[19] + tmpFx[34]*tmpObjSEndTerm[26] + tmpFx[43]*tmpObjSEndTerm[33] + tmpFx[52]*tmpObjSEndTerm[40] + tmpFx[61]*tmpObjSEndTerm[47];
tmpQN2[55] = + tmpFx[7]*tmpObjSEndTerm[6] + tmpFx[16]*tmpObjSEndTerm[13] + tmpFx[25]*tmpObjSEndTerm[20] + tmpFx[34]*tmpObjSEndTerm[27] + tmpFx[43]*tmpObjSEndTerm[34] + tmpFx[52]*tmpObjSEndTerm[41] + tmpFx[61]*tmpObjSEndTerm[48];
tmpQN2[56] = + tmpFx[8]*tmpObjSEndTerm[0] + tmpFx[17]*tmpObjSEndTerm[7] + tmpFx[26]*tmpObjSEndTerm[14] + tmpFx[35]*tmpObjSEndTerm[21] + tmpFx[44]*tmpObjSEndTerm[28] + tmpFx[53]*tmpObjSEndTerm[35] + tmpFx[62]*tmpObjSEndTerm[42];
tmpQN2[57] = + tmpFx[8]*tmpObjSEndTerm[1] + tmpFx[17]*tmpObjSEndTerm[8] + tmpFx[26]*tmpObjSEndTerm[15] + tmpFx[35]*tmpObjSEndTerm[22] + tmpFx[44]*tmpObjSEndTerm[29] + tmpFx[53]*tmpObjSEndTerm[36] + tmpFx[62]*tmpObjSEndTerm[43];
tmpQN2[58] = + tmpFx[8]*tmpObjSEndTerm[2] + tmpFx[17]*tmpObjSEndTerm[9] + tmpFx[26]*tmpObjSEndTerm[16] + tmpFx[35]*tmpObjSEndTerm[23] + tmpFx[44]*tmpObjSEndTerm[30] + tmpFx[53]*tmpObjSEndTerm[37] + tmpFx[62]*tmpObjSEndTerm[44];
tmpQN2[59] = + tmpFx[8]*tmpObjSEndTerm[3] + tmpFx[17]*tmpObjSEndTerm[10] + tmpFx[26]*tmpObjSEndTerm[17] + tmpFx[35]*tmpObjSEndTerm[24] + tmpFx[44]*tmpObjSEndTerm[31] + tmpFx[53]*tmpObjSEndTerm[38] + tmpFx[62]*tmpObjSEndTerm[45];
tmpQN2[60] = + tmpFx[8]*tmpObjSEndTerm[4] + tmpFx[17]*tmpObjSEndTerm[11] + tmpFx[26]*tmpObjSEndTerm[18] + tmpFx[35]*tmpObjSEndTerm[25] + tmpFx[44]*tmpObjSEndTerm[32] + tmpFx[53]*tmpObjSEndTerm[39] + tmpFx[62]*tmpObjSEndTerm[46];
tmpQN2[61] = + tmpFx[8]*tmpObjSEndTerm[5] + tmpFx[17]*tmpObjSEndTerm[12] + tmpFx[26]*tmpObjSEndTerm[19] + tmpFx[35]*tmpObjSEndTerm[26] + tmpFx[44]*tmpObjSEndTerm[33] + tmpFx[53]*tmpObjSEndTerm[40] + tmpFx[62]*tmpObjSEndTerm[47];
tmpQN2[62] = + tmpFx[8]*tmpObjSEndTerm[6] + tmpFx[17]*tmpObjSEndTerm[13] + tmpFx[26]*tmpObjSEndTerm[20] + tmpFx[35]*tmpObjSEndTerm[27] + tmpFx[44]*tmpObjSEndTerm[34] + tmpFx[53]*tmpObjSEndTerm[41] + tmpFx[62]*tmpObjSEndTerm[48];
tmpQN1[0] = + tmpQN2[0]*tmpFx[0] + tmpQN2[1]*tmpFx[9] + tmpQN2[2]*tmpFx[18] + tmpQN2[3]*tmpFx[27] + tmpQN2[4]*tmpFx[36] + tmpQN2[5]*tmpFx[45] + tmpQN2[6]*tmpFx[54];
tmpQN1[1] = + tmpQN2[0]*tmpFx[1] + tmpQN2[1]*tmpFx[10] + tmpQN2[2]*tmpFx[19] + tmpQN2[3]*tmpFx[28] + tmpQN2[4]*tmpFx[37] + tmpQN2[5]*tmpFx[46] + tmpQN2[6]*tmpFx[55];
tmpQN1[2] = + tmpQN2[0]*tmpFx[2] + tmpQN2[1]*tmpFx[11] + tmpQN2[2]*tmpFx[20] + tmpQN2[3]*tmpFx[29] + tmpQN2[4]*tmpFx[38] + tmpQN2[5]*tmpFx[47] + tmpQN2[6]*tmpFx[56];
tmpQN1[3] = + tmpQN2[0]*tmpFx[3] + tmpQN2[1]*tmpFx[12] + tmpQN2[2]*tmpFx[21] + tmpQN2[3]*tmpFx[30] + tmpQN2[4]*tmpFx[39] + tmpQN2[5]*tmpFx[48] + tmpQN2[6]*tmpFx[57];
tmpQN1[4] = + tmpQN2[0]*tmpFx[4] + tmpQN2[1]*tmpFx[13] + tmpQN2[2]*tmpFx[22] + tmpQN2[3]*tmpFx[31] + tmpQN2[4]*tmpFx[40] + tmpQN2[5]*tmpFx[49] + tmpQN2[6]*tmpFx[58];
tmpQN1[5] = + tmpQN2[0]*tmpFx[5] + tmpQN2[1]*tmpFx[14] + tmpQN2[2]*tmpFx[23] + tmpQN2[3]*tmpFx[32] + tmpQN2[4]*tmpFx[41] + tmpQN2[5]*tmpFx[50] + tmpQN2[6]*tmpFx[59];
tmpQN1[6] = + tmpQN2[0]*tmpFx[6] + tmpQN2[1]*tmpFx[15] + tmpQN2[2]*tmpFx[24] + tmpQN2[3]*tmpFx[33] + tmpQN2[4]*tmpFx[42] + tmpQN2[5]*tmpFx[51] + tmpQN2[6]*tmpFx[60];
tmpQN1[7] = + tmpQN2[0]*tmpFx[7] + tmpQN2[1]*tmpFx[16] + tmpQN2[2]*tmpFx[25] + tmpQN2[3]*tmpFx[34] + tmpQN2[4]*tmpFx[43] + tmpQN2[5]*tmpFx[52] + tmpQN2[6]*tmpFx[61];
tmpQN1[8] = + tmpQN2[0]*tmpFx[8] + tmpQN2[1]*tmpFx[17] + tmpQN2[2]*tmpFx[26] + tmpQN2[3]*tmpFx[35] + tmpQN2[4]*tmpFx[44] + tmpQN2[5]*tmpFx[53] + tmpQN2[6]*tmpFx[62];
tmpQN1[9] = + tmpQN2[7]*tmpFx[0] + tmpQN2[8]*tmpFx[9] + tmpQN2[9]*tmpFx[18] + tmpQN2[10]*tmpFx[27] + tmpQN2[11]*tmpFx[36] + tmpQN2[12]*tmpFx[45] + tmpQN2[13]*tmpFx[54];
tmpQN1[10] = + tmpQN2[7]*tmpFx[1] + tmpQN2[8]*tmpFx[10] + tmpQN2[9]*tmpFx[19] + tmpQN2[10]*tmpFx[28] + tmpQN2[11]*tmpFx[37] + tmpQN2[12]*tmpFx[46] + tmpQN2[13]*tmpFx[55];
tmpQN1[11] = + tmpQN2[7]*tmpFx[2] + tmpQN2[8]*tmpFx[11] + tmpQN2[9]*tmpFx[20] + tmpQN2[10]*tmpFx[29] + tmpQN2[11]*tmpFx[38] + tmpQN2[12]*tmpFx[47] + tmpQN2[13]*tmpFx[56];
tmpQN1[12] = + tmpQN2[7]*tmpFx[3] + tmpQN2[8]*tmpFx[12] + tmpQN2[9]*tmpFx[21] + tmpQN2[10]*tmpFx[30] + tmpQN2[11]*tmpFx[39] + tmpQN2[12]*tmpFx[48] + tmpQN2[13]*tmpFx[57];
tmpQN1[13] = + tmpQN2[7]*tmpFx[4] + tmpQN2[8]*tmpFx[13] + tmpQN2[9]*tmpFx[22] + tmpQN2[10]*tmpFx[31] + tmpQN2[11]*tmpFx[40] + tmpQN2[12]*tmpFx[49] + tmpQN2[13]*tmpFx[58];
tmpQN1[14] = + tmpQN2[7]*tmpFx[5] + tmpQN2[8]*tmpFx[14] + tmpQN2[9]*tmpFx[23] + tmpQN2[10]*tmpFx[32] + tmpQN2[11]*tmpFx[41] + tmpQN2[12]*tmpFx[50] + tmpQN2[13]*tmpFx[59];
tmpQN1[15] = + tmpQN2[7]*tmpFx[6] + tmpQN2[8]*tmpFx[15] + tmpQN2[9]*tmpFx[24] + tmpQN2[10]*tmpFx[33] + tmpQN2[11]*tmpFx[42] + tmpQN2[12]*tmpFx[51] + tmpQN2[13]*tmpFx[60];
tmpQN1[16] = + tmpQN2[7]*tmpFx[7] + tmpQN2[8]*tmpFx[16] + tmpQN2[9]*tmpFx[25] + tmpQN2[10]*tmpFx[34] + tmpQN2[11]*tmpFx[43] + tmpQN2[12]*tmpFx[52] + tmpQN2[13]*tmpFx[61];
tmpQN1[17] = + tmpQN2[7]*tmpFx[8] + tmpQN2[8]*tmpFx[17] + tmpQN2[9]*tmpFx[26] + tmpQN2[10]*tmpFx[35] + tmpQN2[11]*tmpFx[44] + tmpQN2[12]*tmpFx[53] + tmpQN2[13]*tmpFx[62];
tmpQN1[18] = + tmpQN2[14]*tmpFx[0] + tmpQN2[15]*tmpFx[9] + tmpQN2[16]*tmpFx[18] + tmpQN2[17]*tmpFx[27] + tmpQN2[18]*tmpFx[36] + tmpQN2[19]*tmpFx[45] + tmpQN2[20]*tmpFx[54];
tmpQN1[19] = + tmpQN2[14]*tmpFx[1] + tmpQN2[15]*tmpFx[10] + tmpQN2[16]*tmpFx[19] + tmpQN2[17]*tmpFx[28] + tmpQN2[18]*tmpFx[37] + tmpQN2[19]*tmpFx[46] + tmpQN2[20]*tmpFx[55];
tmpQN1[20] = + tmpQN2[14]*tmpFx[2] + tmpQN2[15]*tmpFx[11] + tmpQN2[16]*tmpFx[20] + tmpQN2[17]*tmpFx[29] + tmpQN2[18]*tmpFx[38] + tmpQN2[19]*tmpFx[47] + tmpQN2[20]*tmpFx[56];
tmpQN1[21] = + tmpQN2[14]*tmpFx[3] + tmpQN2[15]*tmpFx[12] + tmpQN2[16]*tmpFx[21] + tmpQN2[17]*tmpFx[30] + tmpQN2[18]*tmpFx[39] + tmpQN2[19]*tmpFx[48] + tmpQN2[20]*tmpFx[57];
tmpQN1[22] = + tmpQN2[14]*tmpFx[4] + tmpQN2[15]*tmpFx[13] + tmpQN2[16]*tmpFx[22] + tmpQN2[17]*tmpFx[31] + tmpQN2[18]*tmpFx[40] + tmpQN2[19]*tmpFx[49] + tmpQN2[20]*tmpFx[58];
tmpQN1[23] = + tmpQN2[14]*tmpFx[5] + tmpQN2[15]*tmpFx[14] + tmpQN2[16]*tmpFx[23] + tmpQN2[17]*tmpFx[32] + tmpQN2[18]*tmpFx[41] + tmpQN2[19]*tmpFx[50] + tmpQN2[20]*tmpFx[59];
tmpQN1[24] = + tmpQN2[14]*tmpFx[6] + tmpQN2[15]*tmpFx[15] + tmpQN2[16]*tmpFx[24] + tmpQN2[17]*tmpFx[33] + tmpQN2[18]*tmpFx[42] + tmpQN2[19]*tmpFx[51] + tmpQN2[20]*tmpFx[60];
tmpQN1[25] = + tmpQN2[14]*tmpFx[7] + tmpQN2[15]*tmpFx[16] + tmpQN2[16]*tmpFx[25] + tmpQN2[17]*tmpFx[34] + tmpQN2[18]*tmpFx[43] + tmpQN2[19]*tmpFx[52] + tmpQN2[20]*tmpFx[61];
tmpQN1[26] = + tmpQN2[14]*tmpFx[8] + tmpQN2[15]*tmpFx[17] + tmpQN2[16]*tmpFx[26] + tmpQN2[17]*tmpFx[35] + tmpQN2[18]*tmpFx[44] + tmpQN2[19]*tmpFx[53] + tmpQN2[20]*tmpFx[62];
tmpQN1[27] = + tmpQN2[21]*tmpFx[0] + tmpQN2[22]*tmpFx[9] + tmpQN2[23]*tmpFx[18] + tmpQN2[24]*tmpFx[27] + tmpQN2[25]*tmpFx[36] + tmpQN2[26]*tmpFx[45] + tmpQN2[27]*tmpFx[54];
tmpQN1[28] = + tmpQN2[21]*tmpFx[1] + tmpQN2[22]*tmpFx[10] + tmpQN2[23]*tmpFx[19] + tmpQN2[24]*tmpFx[28] + tmpQN2[25]*tmpFx[37] + tmpQN2[26]*tmpFx[46] + tmpQN2[27]*tmpFx[55];
tmpQN1[29] = + tmpQN2[21]*tmpFx[2] + tmpQN2[22]*tmpFx[11] + tmpQN2[23]*tmpFx[20] + tmpQN2[24]*tmpFx[29] + tmpQN2[25]*tmpFx[38] + tmpQN2[26]*tmpFx[47] + tmpQN2[27]*tmpFx[56];
tmpQN1[30] = + tmpQN2[21]*tmpFx[3] + tmpQN2[22]*tmpFx[12] + tmpQN2[23]*tmpFx[21] + tmpQN2[24]*tmpFx[30] + tmpQN2[25]*tmpFx[39] + tmpQN2[26]*tmpFx[48] + tmpQN2[27]*tmpFx[57];
tmpQN1[31] = + tmpQN2[21]*tmpFx[4] + tmpQN2[22]*tmpFx[13] + tmpQN2[23]*tmpFx[22] + tmpQN2[24]*tmpFx[31] + tmpQN2[25]*tmpFx[40] + tmpQN2[26]*tmpFx[49] + tmpQN2[27]*tmpFx[58];
tmpQN1[32] = + tmpQN2[21]*tmpFx[5] + tmpQN2[22]*tmpFx[14] + tmpQN2[23]*tmpFx[23] + tmpQN2[24]*tmpFx[32] + tmpQN2[25]*tmpFx[41] + tmpQN2[26]*tmpFx[50] + tmpQN2[27]*tmpFx[59];
tmpQN1[33] = + tmpQN2[21]*tmpFx[6] + tmpQN2[22]*tmpFx[15] + tmpQN2[23]*tmpFx[24] + tmpQN2[24]*tmpFx[33] + tmpQN2[25]*tmpFx[42] + tmpQN2[26]*tmpFx[51] + tmpQN2[27]*tmpFx[60];
tmpQN1[34] = + tmpQN2[21]*tmpFx[7] + tmpQN2[22]*tmpFx[16] + tmpQN2[23]*tmpFx[25] + tmpQN2[24]*tmpFx[34] + tmpQN2[25]*tmpFx[43] + tmpQN2[26]*tmpFx[52] + tmpQN2[27]*tmpFx[61];
tmpQN1[35] = + tmpQN2[21]*tmpFx[8] + tmpQN2[22]*tmpFx[17] + tmpQN2[23]*tmpFx[26] + tmpQN2[24]*tmpFx[35] + tmpQN2[25]*tmpFx[44] + tmpQN2[26]*tmpFx[53] + tmpQN2[27]*tmpFx[62];
tmpQN1[36] = + tmpQN2[28]*tmpFx[0] + tmpQN2[29]*tmpFx[9] + tmpQN2[30]*tmpFx[18] + tmpQN2[31]*tmpFx[27] + tmpQN2[32]*tmpFx[36] + tmpQN2[33]*tmpFx[45] + tmpQN2[34]*tmpFx[54];
tmpQN1[37] = + tmpQN2[28]*tmpFx[1] + tmpQN2[29]*tmpFx[10] + tmpQN2[30]*tmpFx[19] + tmpQN2[31]*tmpFx[28] + tmpQN2[32]*tmpFx[37] + tmpQN2[33]*tmpFx[46] + tmpQN2[34]*tmpFx[55];
tmpQN1[38] = + tmpQN2[28]*tmpFx[2] + tmpQN2[29]*tmpFx[11] + tmpQN2[30]*tmpFx[20] + tmpQN2[31]*tmpFx[29] + tmpQN2[32]*tmpFx[38] + tmpQN2[33]*tmpFx[47] + tmpQN2[34]*tmpFx[56];
tmpQN1[39] = + tmpQN2[28]*tmpFx[3] + tmpQN2[29]*tmpFx[12] + tmpQN2[30]*tmpFx[21] + tmpQN2[31]*tmpFx[30] + tmpQN2[32]*tmpFx[39] + tmpQN2[33]*tmpFx[48] + tmpQN2[34]*tmpFx[57];
tmpQN1[40] = + tmpQN2[28]*tmpFx[4] + tmpQN2[29]*tmpFx[13] + tmpQN2[30]*tmpFx[22] + tmpQN2[31]*tmpFx[31] + tmpQN2[32]*tmpFx[40] + tmpQN2[33]*tmpFx[49] + tmpQN2[34]*tmpFx[58];
tmpQN1[41] = + tmpQN2[28]*tmpFx[5] + tmpQN2[29]*tmpFx[14] + tmpQN2[30]*tmpFx[23] + tmpQN2[31]*tmpFx[32] + tmpQN2[32]*tmpFx[41] + tmpQN2[33]*tmpFx[50] + tmpQN2[34]*tmpFx[59];
tmpQN1[42] = + tmpQN2[28]*tmpFx[6] + tmpQN2[29]*tmpFx[15] + tmpQN2[30]*tmpFx[24] + tmpQN2[31]*tmpFx[33] + tmpQN2[32]*tmpFx[42] + tmpQN2[33]*tmpFx[51] + tmpQN2[34]*tmpFx[60];
tmpQN1[43] = + tmpQN2[28]*tmpFx[7] + tmpQN2[29]*tmpFx[16] + tmpQN2[30]*tmpFx[25] + tmpQN2[31]*tmpFx[34] + tmpQN2[32]*tmpFx[43] + tmpQN2[33]*tmpFx[52] + tmpQN2[34]*tmpFx[61];
tmpQN1[44] = + tmpQN2[28]*tmpFx[8] + tmpQN2[29]*tmpFx[17] + tmpQN2[30]*tmpFx[26] + tmpQN2[31]*tmpFx[35] + tmpQN2[32]*tmpFx[44] + tmpQN2[33]*tmpFx[53] + tmpQN2[34]*tmpFx[62];
tmpQN1[45] = + tmpQN2[35]*tmpFx[0] + tmpQN2[36]*tmpFx[9] + tmpQN2[37]*tmpFx[18] + tmpQN2[38]*tmpFx[27] + tmpQN2[39]*tmpFx[36] + tmpQN2[40]*tmpFx[45] + tmpQN2[41]*tmpFx[54];
tmpQN1[46] = + tmpQN2[35]*tmpFx[1] + tmpQN2[36]*tmpFx[10] + tmpQN2[37]*tmpFx[19] + tmpQN2[38]*tmpFx[28] + tmpQN2[39]*tmpFx[37] + tmpQN2[40]*tmpFx[46] + tmpQN2[41]*tmpFx[55];
tmpQN1[47] = + tmpQN2[35]*tmpFx[2] + tmpQN2[36]*tmpFx[11] + tmpQN2[37]*tmpFx[20] + tmpQN2[38]*tmpFx[29] + tmpQN2[39]*tmpFx[38] + tmpQN2[40]*tmpFx[47] + tmpQN2[41]*tmpFx[56];
tmpQN1[48] = + tmpQN2[35]*tmpFx[3] + tmpQN2[36]*tmpFx[12] + tmpQN2[37]*tmpFx[21] + tmpQN2[38]*tmpFx[30] + tmpQN2[39]*tmpFx[39] + tmpQN2[40]*tmpFx[48] + tmpQN2[41]*tmpFx[57];
tmpQN1[49] = + tmpQN2[35]*tmpFx[4] + tmpQN2[36]*tmpFx[13] + tmpQN2[37]*tmpFx[22] + tmpQN2[38]*tmpFx[31] + tmpQN2[39]*tmpFx[40] + tmpQN2[40]*tmpFx[49] + tmpQN2[41]*tmpFx[58];
tmpQN1[50] = + tmpQN2[35]*tmpFx[5] + tmpQN2[36]*tmpFx[14] + tmpQN2[37]*tmpFx[23] + tmpQN2[38]*tmpFx[32] + tmpQN2[39]*tmpFx[41] + tmpQN2[40]*tmpFx[50] + tmpQN2[41]*tmpFx[59];
tmpQN1[51] = + tmpQN2[35]*tmpFx[6] + tmpQN2[36]*tmpFx[15] + tmpQN2[37]*tmpFx[24] + tmpQN2[38]*tmpFx[33] + tmpQN2[39]*tmpFx[42] + tmpQN2[40]*tmpFx[51] + tmpQN2[41]*tmpFx[60];
tmpQN1[52] = + tmpQN2[35]*tmpFx[7] + tmpQN2[36]*tmpFx[16] + tmpQN2[37]*tmpFx[25] + tmpQN2[38]*tmpFx[34] + tmpQN2[39]*tmpFx[43] + tmpQN2[40]*tmpFx[52] + tmpQN2[41]*tmpFx[61];
tmpQN1[53] = + tmpQN2[35]*tmpFx[8] + tmpQN2[36]*tmpFx[17] + tmpQN2[37]*tmpFx[26] + tmpQN2[38]*tmpFx[35] + tmpQN2[39]*tmpFx[44] + tmpQN2[40]*tmpFx[53] + tmpQN2[41]*tmpFx[62];
tmpQN1[54] = + tmpQN2[42]*tmpFx[0] + tmpQN2[43]*tmpFx[9] + tmpQN2[44]*tmpFx[18] + tmpQN2[45]*tmpFx[27] + tmpQN2[46]*tmpFx[36] + tmpQN2[47]*tmpFx[45] + tmpQN2[48]*tmpFx[54];
tmpQN1[55] = + tmpQN2[42]*tmpFx[1] + tmpQN2[43]*tmpFx[10] + tmpQN2[44]*tmpFx[19] + tmpQN2[45]*tmpFx[28] + tmpQN2[46]*tmpFx[37] + tmpQN2[47]*tmpFx[46] + tmpQN2[48]*tmpFx[55];
tmpQN1[56] = + tmpQN2[42]*tmpFx[2] + tmpQN2[43]*tmpFx[11] + tmpQN2[44]*tmpFx[20] + tmpQN2[45]*tmpFx[29] + tmpQN2[46]*tmpFx[38] + tmpQN2[47]*tmpFx[47] + tmpQN2[48]*tmpFx[56];
tmpQN1[57] = + tmpQN2[42]*tmpFx[3] + tmpQN2[43]*tmpFx[12] + tmpQN2[44]*tmpFx[21] + tmpQN2[45]*tmpFx[30] + tmpQN2[46]*tmpFx[39] + tmpQN2[47]*tmpFx[48] + tmpQN2[48]*tmpFx[57];
tmpQN1[58] = + tmpQN2[42]*tmpFx[4] + tmpQN2[43]*tmpFx[13] + tmpQN2[44]*tmpFx[22] + tmpQN2[45]*tmpFx[31] + tmpQN2[46]*tmpFx[40] + tmpQN2[47]*tmpFx[49] + tmpQN2[48]*tmpFx[58];
tmpQN1[59] = + tmpQN2[42]*tmpFx[5] + tmpQN2[43]*tmpFx[14] + tmpQN2[44]*tmpFx[23] + tmpQN2[45]*tmpFx[32] + tmpQN2[46]*tmpFx[41] + tmpQN2[47]*tmpFx[50] + tmpQN2[48]*tmpFx[59];
tmpQN1[60] = + tmpQN2[42]*tmpFx[6] + tmpQN2[43]*tmpFx[15] + tmpQN2[44]*tmpFx[24] + tmpQN2[45]*tmpFx[33] + tmpQN2[46]*tmpFx[42] + tmpQN2[47]*tmpFx[51] + tmpQN2[48]*tmpFx[60];
tmpQN1[61] = + tmpQN2[42]*tmpFx[7] + tmpQN2[43]*tmpFx[16] + tmpQN2[44]*tmpFx[25] + tmpQN2[45]*tmpFx[34] + tmpQN2[46]*tmpFx[43] + tmpQN2[47]*tmpFx[52] + tmpQN2[48]*tmpFx[61];
tmpQN1[62] = + tmpQN2[42]*tmpFx[8] + tmpQN2[43]*tmpFx[17] + tmpQN2[44]*tmpFx[26] + tmpQN2[45]*tmpFx[35] + tmpQN2[46]*tmpFx[44] + tmpQN2[47]*tmpFx[53] + tmpQN2[48]*tmpFx[62];
tmpQN1[63] = + tmpQN2[49]*tmpFx[0] + tmpQN2[50]*tmpFx[9] + tmpQN2[51]*tmpFx[18] + tmpQN2[52]*tmpFx[27] + tmpQN2[53]*tmpFx[36] + tmpQN2[54]*tmpFx[45] + tmpQN2[55]*tmpFx[54];
tmpQN1[64] = + tmpQN2[49]*tmpFx[1] + tmpQN2[50]*tmpFx[10] + tmpQN2[51]*tmpFx[19] + tmpQN2[52]*tmpFx[28] + tmpQN2[53]*tmpFx[37] + tmpQN2[54]*tmpFx[46] + tmpQN2[55]*tmpFx[55];
tmpQN1[65] = + tmpQN2[49]*tmpFx[2] + tmpQN2[50]*tmpFx[11] + tmpQN2[51]*tmpFx[20] + tmpQN2[52]*tmpFx[29] + tmpQN2[53]*tmpFx[38] + tmpQN2[54]*tmpFx[47] + tmpQN2[55]*tmpFx[56];
tmpQN1[66] = + tmpQN2[49]*tmpFx[3] + tmpQN2[50]*tmpFx[12] + tmpQN2[51]*tmpFx[21] + tmpQN2[52]*tmpFx[30] + tmpQN2[53]*tmpFx[39] + tmpQN2[54]*tmpFx[48] + tmpQN2[55]*tmpFx[57];
tmpQN1[67] = + tmpQN2[49]*tmpFx[4] + tmpQN2[50]*tmpFx[13] + tmpQN2[51]*tmpFx[22] + tmpQN2[52]*tmpFx[31] + tmpQN2[53]*tmpFx[40] + tmpQN2[54]*tmpFx[49] + tmpQN2[55]*tmpFx[58];
tmpQN1[68] = + tmpQN2[49]*tmpFx[5] + tmpQN2[50]*tmpFx[14] + tmpQN2[51]*tmpFx[23] + tmpQN2[52]*tmpFx[32] + tmpQN2[53]*tmpFx[41] + tmpQN2[54]*tmpFx[50] + tmpQN2[55]*tmpFx[59];
tmpQN1[69] = + tmpQN2[49]*tmpFx[6] + tmpQN2[50]*tmpFx[15] + tmpQN2[51]*tmpFx[24] + tmpQN2[52]*tmpFx[33] + tmpQN2[53]*tmpFx[42] + tmpQN2[54]*tmpFx[51] + tmpQN2[55]*tmpFx[60];
tmpQN1[70] = + tmpQN2[49]*tmpFx[7] + tmpQN2[50]*tmpFx[16] + tmpQN2[51]*tmpFx[25] + tmpQN2[52]*tmpFx[34] + tmpQN2[53]*tmpFx[43] + tmpQN2[54]*tmpFx[52] + tmpQN2[55]*tmpFx[61];
tmpQN1[71] = + tmpQN2[49]*tmpFx[8] + tmpQN2[50]*tmpFx[17] + tmpQN2[51]*tmpFx[26] + tmpQN2[52]*tmpFx[35] + tmpQN2[53]*tmpFx[44] + tmpQN2[54]*tmpFx[53] + tmpQN2[55]*tmpFx[62];
tmpQN1[72] = + tmpQN2[56]*tmpFx[0] + tmpQN2[57]*tmpFx[9] + tmpQN2[58]*tmpFx[18] + tmpQN2[59]*tmpFx[27] + tmpQN2[60]*tmpFx[36] + tmpQN2[61]*tmpFx[45] + tmpQN2[62]*tmpFx[54];
tmpQN1[73] = + tmpQN2[56]*tmpFx[1] + tmpQN2[57]*tmpFx[10] + tmpQN2[58]*tmpFx[19] + tmpQN2[59]*tmpFx[28] + tmpQN2[60]*tmpFx[37] + tmpQN2[61]*tmpFx[46] + tmpQN2[62]*tmpFx[55];
tmpQN1[74] = + tmpQN2[56]*tmpFx[2] + tmpQN2[57]*tmpFx[11] + tmpQN2[58]*tmpFx[20] + tmpQN2[59]*tmpFx[29] + tmpQN2[60]*tmpFx[38] + tmpQN2[61]*tmpFx[47] + tmpQN2[62]*tmpFx[56];
tmpQN1[75] = + tmpQN2[56]*tmpFx[3] + tmpQN2[57]*tmpFx[12] + tmpQN2[58]*tmpFx[21] + tmpQN2[59]*tmpFx[30] + tmpQN2[60]*tmpFx[39] + tmpQN2[61]*tmpFx[48] + tmpQN2[62]*tmpFx[57];
tmpQN1[76] = + tmpQN2[56]*tmpFx[4] + tmpQN2[57]*tmpFx[13] + tmpQN2[58]*tmpFx[22] + tmpQN2[59]*tmpFx[31] + tmpQN2[60]*tmpFx[40] + tmpQN2[61]*tmpFx[49] + tmpQN2[62]*tmpFx[58];
tmpQN1[77] = + tmpQN2[56]*tmpFx[5] + tmpQN2[57]*tmpFx[14] + tmpQN2[58]*tmpFx[23] + tmpQN2[59]*tmpFx[32] + tmpQN2[60]*tmpFx[41] + tmpQN2[61]*tmpFx[50] + tmpQN2[62]*tmpFx[59];
tmpQN1[78] = + tmpQN2[56]*tmpFx[6] + tmpQN2[57]*tmpFx[15] + tmpQN2[58]*tmpFx[24] + tmpQN2[59]*tmpFx[33] + tmpQN2[60]*tmpFx[42] + tmpQN2[61]*tmpFx[51] + tmpQN2[62]*tmpFx[60];
tmpQN1[79] = + tmpQN2[56]*tmpFx[7] + tmpQN2[57]*tmpFx[16] + tmpQN2[58]*tmpFx[25] + tmpQN2[59]*tmpFx[34] + tmpQN2[60]*tmpFx[43] + tmpQN2[61]*tmpFx[52] + tmpQN2[62]*tmpFx[61];
tmpQN1[80] = + tmpQN2[56]*tmpFx[8] + tmpQN2[57]*tmpFx[17] + tmpQN2[58]*tmpFx[26] + tmpQN2[59]*tmpFx[35] + tmpQN2[60]*tmpFx[44] + tmpQN2[61]*tmpFx[53] + tmpQN2[62]*tmpFx[62];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 15];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 15 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 15 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 15 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 15 + 4];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 15 + 5];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 15 + 6];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 15 + 7];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 15 + 8];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 15 + 9];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 15 + 10];
acadoWorkspace.objValueIn[23] = acadoVariables.od[runObj * 15 + 11];
acadoWorkspace.objValueIn[24] = acadoVariables.od[runObj * 15 + 12];
acadoWorkspace.objValueIn[25] = acadoVariables.od[runObj * 15 + 13];
acadoWorkspace.objValueIn[26] = acadoVariables.od[runObj * 15 + 14];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 12] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 12 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 12 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 12 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 12 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 12 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 12 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 12 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 12 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 12 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 12 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 12 + 11] = acadoWorkspace.objValueOut[11];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 12 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 81 ]), &(acadoWorkspace.Q2[ runObj * 108 ]) );

acado_setObjR1R2( &(acadoWorkspace.objValueOut[ 120 ]), acadoVariables.W, &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 36 ]) );

acado_setObjS1( &(acadoWorkspace.objValueOut[ 12 ]), &(acadoWorkspace.objValueOut[ 120 ]), acadoVariables.W, &(acadoWorkspace.S1[ runObj * 27 ]) );
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.x[183];
acadoWorkspace.objValueIn[4] = acadoVariables.x[184];
acadoWorkspace.objValueIn[5] = acadoVariables.x[185];
acadoWorkspace.objValueIn[6] = acadoVariables.x[186];
acadoWorkspace.objValueIn[7] = acadoVariables.x[187];
acadoWorkspace.objValueIn[8] = acadoVariables.x[188];
acadoWorkspace.objValueIn[9] = acadoVariables.od[300];
acadoWorkspace.objValueIn[10] = acadoVariables.od[301];
acadoWorkspace.objValueIn[11] = acadoVariables.od[302];
acadoWorkspace.objValueIn[12] = acadoVariables.od[303];
acadoWorkspace.objValueIn[13] = acadoVariables.od[304];
acadoWorkspace.objValueIn[14] = acadoVariables.od[305];
acadoWorkspace.objValueIn[15] = acadoVariables.od[306];
acadoWorkspace.objValueIn[16] = acadoVariables.od[307];
acadoWorkspace.objValueIn[17] = acadoVariables.od[308];
acadoWorkspace.objValueIn[18] = acadoVariables.od[309];
acadoWorkspace.objValueIn[19] = acadoVariables.od[310];
acadoWorkspace.objValueIn[20] = acadoVariables.od[311];
acadoWorkspace.objValueIn[21] = acadoVariables.od[312];
acadoWorkspace.objValueIn[22] = acadoVariables.od[313];
acadoWorkspace.objValueIn[23] = acadoVariables.od[314];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];

acado_setObjQN1QN2( &(acadoWorkspace.objValueOut[ 7 ]), acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26];
Gu2[3] = + Gx1[9]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[15] + Gx1[15]*Gu1[18] + Gx1[16]*Gu1[21] + Gx1[17]*Gu1[24];
Gu2[4] = + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[19] + Gx1[16]*Gu1[22] + Gx1[17]*Gu1[25];
Gu2[5] = + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[8] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[23] + Gx1[17]*Gu1[26];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[21]*Gu1[9] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[24];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[16] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[25];
Gu2[8] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[14] + Gx1[23]*Gu1[17] + Gx1[24]*Gu1[20] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[26];
Gu2[9] = + Gx1[27]*Gu1[0] + Gx1[28]*Gu1[3] + Gx1[29]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[31]*Gu1[12] + Gx1[32]*Gu1[15] + Gx1[33]*Gu1[18] + Gx1[34]*Gu1[21] + Gx1[35]*Gu1[24];
Gu2[10] = + Gx1[27]*Gu1[1] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[31]*Gu1[13] + Gx1[32]*Gu1[16] + Gx1[33]*Gu1[19] + Gx1[34]*Gu1[22] + Gx1[35]*Gu1[25];
Gu2[11] = + Gx1[27]*Gu1[2] + Gx1[28]*Gu1[5] + Gx1[29]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[31]*Gu1[14] + Gx1[32]*Gu1[17] + Gx1[33]*Gu1[20] + Gx1[34]*Gu1[23] + Gx1[35]*Gu1[26];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[3] + Gx1[38]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[41]*Gu1[15] + Gx1[42]*Gu1[18] + Gx1[43]*Gu1[21] + Gx1[44]*Gu1[24];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[41]*Gu1[16] + Gx1[42]*Gu1[19] + Gx1[43]*Gu1[22] + Gx1[44]*Gu1[25];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[41]*Gu1[17] + Gx1[42]*Gu1[20] + Gx1[43]*Gu1[23] + Gx1[44]*Gu1[26];
Gu2[15] = + Gx1[45]*Gu1[0] + Gx1[46]*Gu1[3] + Gx1[47]*Gu1[6] + Gx1[48]*Gu1[9] + Gx1[49]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[51]*Gu1[18] + Gx1[52]*Gu1[21] + Gx1[53]*Gu1[24];
Gu2[16] = + Gx1[45]*Gu1[1] + Gx1[46]*Gu1[4] + Gx1[47]*Gu1[7] + Gx1[48]*Gu1[10] + Gx1[49]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[51]*Gu1[19] + Gx1[52]*Gu1[22] + Gx1[53]*Gu1[25];
Gu2[17] = + Gx1[45]*Gu1[2] + Gx1[46]*Gu1[5] + Gx1[47]*Gu1[8] + Gx1[48]*Gu1[11] + Gx1[49]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[51]*Gu1[20] + Gx1[52]*Gu1[23] + Gx1[53]*Gu1[26];
Gu2[18] = + Gx1[54]*Gu1[0] + Gx1[55]*Gu1[3] + Gx1[56]*Gu1[6] + Gx1[57]*Gu1[9] + Gx1[58]*Gu1[12] + Gx1[59]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[61]*Gu1[21] + Gx1[62]*Gu1[24];
Gu2[19] = + Gx1[54]*Gu1[1] + Gx1[55]*Gu1[4] + Gx1[56]*Gu1[7] + Gx1[57]*Gu1[10] + Gx1[58]*Gu1[13] + Gx1[59]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[61]*Gu1[22] + Gx1[62]*Gu1[25];
Gu2[20] = + Gx1[54]*Gu1[2] + Gx1[55]*Gu1[5] + Gx1[56]*Gu1[8] + Gx1[57]*Gu1[11] + Gx1[58]*Gu1[14] + Gx1[59]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[61]*Gu1[23] + Gx1[62]*Gu1[26];
Gu2[21] = + Gx1[63]*Gu1[0] + Gx1[64]*Gu1[3] + Gx1[65]*Gu1[6] + Gx1[66]*Gu1[9] + Gx1[67]*Gu1[12] + Gx1[68]*Gu1[15] + Gx1[69]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[71]*Gu1[24];
Gu2[22] = + Gx1[63]*Gu1[1] + Gx1[64]*Gu1[4] + Gx1[65]*Gu1[7] + Gx1[66]*Gu1[10] + Gx1[67]*Gu1[13] + Gx1[68]*Gu1[16] + Gx1[69]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[71]*Gu1[25];
Gu2[23] = + Gx1[63]*Gu1[2] + Gx1[64]*Gu1[5] + Gx1[65]*Gu1[8] + Gx1[66]*Gu1[11] + Gx1[67]*Gu1[14] + Gx1[68]*Gu1[17] + Gx1[69]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[71]*Gu1[26];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[3] + Gx1[74]*Gu1[6] + Gx1[75]*Gu1[9] + Gx1[76]*Gu1[12] + Gx1[77]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[79]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[7] + Gx1[75]*Gu1[10] + Gx1[76]*Gu1[13] + Gx1[77]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[79]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[11] + Gx1[76]*Gu1[14] + Gx1[77]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[79]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 183] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + R11[0];
acadoWorkspace.H[iRow * 183 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + R11[1];
acadoWorkspace.H[iRow * 183 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + R11[2];
acadoWorkspace.H[iRow * 183 + 60] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + R11[3];
acadoWorkspace.H[iRow * 183 + 61] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + R11[4];
acadoWorkspace.H[iRow * 183 + 62] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + R11[5];
acadoWorkspace.H[iRow * 183 + 120] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + R11[6];
acadoWorkspace.H[iRow * 183 + 121] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + R11[7];
acadoWorkspace.H[iRow * 183 + 122] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + R11[8];
acadoWorkspace.H[iRow * 183] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 183 + 61] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 183 + 122] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[9]*Gu1[3] + Gx1[18]*Gu1[6] + Gx1[27]*Gu1[9] + Gx1[36]*Gu1[12] + Gx1[45]*Gu1[15] + Gx1[54]*Gu1[18] + Gx1[63]*Gu1[21] + Gx1[72]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[9]*Gu1[4] + Gx1[18]*Gu1[7] + Gx1[27]*Gu1[10] + Gx1[36]*Gu1[13] + Gx1[45]*Gu1[16] + Gx1[54]*Gu1[19] + Gx1[63]*Gu1[22] + Gx1[72]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[9]*Gu1[5] + Gx1[18]*Gu1[8] + Gx1[27]*Gu1[11] + Gx1[36]*Gu1[14] + Gx1[45]*Gu1[17] + Gx1[54]*Gu1[20] + Gx1[63]*Gu1[23] + Gx1[72]*Gu1[26];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[19]*Gu1[6] + Gx1[28]*Gu1[9] + Gx1[37]*Gu1[12] + Gx1[46]*Gu1[15] + Gx1[55]*Gu1[18] + Gx1[64]*Gu1[21] + Gx1[73]*Gu1[24];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[19]*Gu1[7] + Gx1[28]*Gu1[10] + Gx1[37]*Gu1[13] + Gx1[46]*Gu1[16] + Gx1[55]*Gu1[19] + Gx1[64]*Gu1[22] + Gx1[73]*Gu1[25];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[19]*Gu1[8] + Gx1[28]*Gu1[11] + Gx1[37]*Gu1[14] + Gx1[46]*Gu1[17] + Gx1[55]*Gu1[20] + Gx1[64]*Gu1[23] + Gx1[73]*Gu1[26];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[11]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[29]*Gu1[9] + Gx1[38]*Gu1[12] + Gx1[47]*Gu1[15] + Gx1[56]*Gu1[18] + Gx1[65]*Gu1[21] + Gx1[74]*Gu1[24];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[11]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[29]*Gu1[10] + Gx1[38]*Gu1[13] + Gx1[47]*Gu1[16] + Gx1[56]*Gu1[19] + Gx1[65]*Gu1[22] + Gx1[74]*Gu1[25];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[11]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[29]*Gu1[11] + Gx1[38]*Gu1[14] + Gx1[47]*Gu1[17] + Gx1[56]*Gu1[20] + Gx1[65]*Gu1[23] + Gx1[74]*Gu1[26];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[12]*Gu1[3] + Gx1[21]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[39]*Gu1[12] + Gx1[48]*Gu1[15] + Gx1[57]*Gu1[18] + Gx1[66]*Gu1[21] + Gx1[75]*Gu1[24];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[12]*Gu1[4] + Gx1[21]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[39]*Gu1[13] + Gx1[48]*Gu1[16] + Gx1[57]*Gu1[19] + Gx1[66]*Gu1[22] + Gx1[75]*Gu1[25];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[12]*Gu1[5] + Gx1[21]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[39]*Gu1[14] + Gx1[48]*Gu1[17] + Gx1[57]*Gu1[20] + Gx1[66]*Gu1[23] + Gx1[75]*Gu1[26];
Gu2[12] = + Gx1[4]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[22]*Gu1[6] + Gx1[31]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[49]*Gu1[15] + Gx1[58]*Gu1[18] + Gx1[67]*Gu1[21] + Gx1[76]*Gu1[24];
Gu2[13] = + Gx1[4]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[22]*Gu1[7] + Gx1[31]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[49]*Gu1[16] + Gx1[58]*Gu1[19] + Gx1[67]*Gu1[22] + Gx1[76]*Gu1[25];
Gu2[14] = + Gx1[4]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[22]*Gu1[8] + Gx1[31]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[49]*Gu1[17] + Gx1[58]*Gu1[20] + Gx1[67]*Gu1[23] + Gx1[76]*Gu1[26];
Gu2[15] = + Gx1[5]*Gu1[0] + Gx1[14]*Gu1[3] + Gx1[23]*Gu1[6] + Gx1[32]*Gu1[9] + Gx1[41]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[59]*Gu1[18] + Gx1[68]*Gu1[21] + Gx1[77]*Gu1[24];
Gu2[16] = + Gx1[5]*Gu1[1] + Gx1[14]*Gu1[4] + Gx1[23]*Gu1[7] + Gx1[32]*Gu1[10] + Gx1[41]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[59]*Gu1[19] + Gx1[68]*Gu1[22] + Gx1[77]*Gu1[25];
Gu2[17] = + Gx1[5]*Gu1[2] + Gx1[14]*Gu1[5] + Gx1[23]*Gu1[8] + Gx1[32]*Gu1[11] + Gx1[41]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[59]*Gu1[20] + Gx1[68]*Gu1[23] + Gx1[77]*Gu1[26];
Gu2[18] = + Gx1[6]*Gu1[0] + Gx1[15]*Gu1[3] + Gx1[24]*Gu1[6] + Gx1[33]*Gu1[9] + Gx1[42]*Gu1[12] + Gx1[51]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[69]*Gu1[21] + Gx1[78]*Gu1[24];
Gu2[19] = + Gx1[6]*Gu1[1] + Gx1[15]*Gu1[4] + Gx1[24]*Gu1[7] + Gx1[33]*Gu1[10] + Gx1[42]*Gu1[13] + Gx1[51]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[69]*Gu1[22] + Gx1[78]*Gu1[25];
Gu2[20] = + Gx1[6]*Gu1[2] + Gx1[15]*Gu1[5] + Gx1[24]*Gu1[8] + Gx1[33]*Gu1[11] + Gx1[42]*Gu1[14] + Gx1[51]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[69]*Gu1[23] + Gx1[78]*Gu1[26];
Gu2[21] = + Gx1[7]*Gu1[0] + Gx1[16]*Gu1[3] + Gx1[25]*Gu1[6] + Gx1[34]*Gu1[9] + Gx1[43]*Gu1[12] + Gx1[52]*Gu1[15] + Gx1[61]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[79]*Gu1[24];
Gu2[22] = + Gx1[7]*Gu1[1] + Gx1[16]*Gu1[4] + Gx1[25]*Gu1[7] + Gx1[34]*Gu1[10] + Gx1[43]*Gu1[13] + Gx1[52]*Gu1[16] + Gx1[61]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[79]*Gu1[25];
Gu2[23] = + Gx1[7]*Gu1[2] + Gx1[16]*Gu1[5] + Gx1[25]*Gu1[8] + Gx1[34]*Gu1[11] + Gx1[43]*Gu1[14] + Gx1[52]*Gu1[17] + Gx1[61]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[79]*Gu1[26];
Gu2[24] = + Gx1[8]*Gu1[0] + Gx1[17]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[35]*Gu1[9] + Gx1[44]*Gu1[12] + Gx1[53]*Gu1[15] + Gx1[62]*Gu1[18] + Gx1[71]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[8]*Gu1[1] + Gx1[17]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[35]*Gu1[10] + Gx1[44]*Gu1[13] + Gx1[53]*Gu1[16] + Gx1[62]*Gu1[19] + Gx1[71]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[8]*Gu1[2] + Gx1[17]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[35]*Gu1[11] + Gx1[44]*Gu1[14] + Gx1[53]*Gu1[17] + Gx1[62]*Gu1[20] + Gx1[71]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Q11[4]*Gu1[12] + Q11[5]*Gu1[15] + Q11[6]*Gu1[18] + Q11[7]*Gu1[21] + Q11[8]*Gu1[24] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Q11[4]*Gu1[13] + Q11[5]*Gu1[16] + Q11[6]*Gu1[19] + Q11[7]*Gu1[22] + Q11[8]*Gu1[25] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Q11[4]*Gu1[14] + Q11[5]*Gu1[17] + Q11[6]*Gu1[20] + Q11[7]*Gu1[23] + Q11[8]*Gu1[26] + Gu2[2];
Gu3[3] = + Q11[9]*Gu1[0] + Q11[10]*Gu1[3] + Q11[11]*Gu1[6] + Q11[12]*Gu1[9] + Q11[13]*Gu1[12] + Q11[14]*Gu1[15] + Q11[15]*Gu1[18] + Q11[16]*Gu1[21] + Q11[17]*Gu1[24] + Gu2[3];
Gu3[4] = + Q11[9]*Gu1[1] + Q11[10]*Gu1[4] + Q11[11]*Gu1[7] + Q11[12]*Gu1[10] + Q11[13]*Gu1[13] + Q11[14]*Gu1[16] + Q11[15]*Gu1[19] + Q11[16]*Gu1[22] + Q11[17]*Gu1[25] + Gu2[4];
Gu3[5] = + Q11[9]*Gu1[2] + Q11[10]*Gu1[5] + Q11[11]*Gu1[8] + Q11[12]*Gu1[11] + Q11[13]*Gu1[14] + Q11[14]*Gu1[17] + Q11[15]*Gu1[20] + Q11[16]*Gu1[23] + Q11[17]*Gu1[26] + Gu2[5];
Gu3[6] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[3] + Q11[20]*Gu1[6] + Q11[21]*Gu1[9] + Q11[22]*Gu1[12] + Q11[23]*Gu1[15] + Q11[24]*Gu1[18] + Q11[25]*Gu1[21] + Q11[26]*Gu1[24] + Gu2[6];
Gu3[7] = + Q11[18]*Gu1[1] + Q11[19]*Gu1[4] + Q11[20]*Gu1[7] + Q11[21]*Gu1[10] + Q11[22]*Gu1[13] + Q11[23]*Gu1[16] + Q11[24]*Gu1[19] + Q11[25]*Gu1[22] + Q11[26]*Gu1[25] + Gu2[7];
Gu3[8] = + Q11[18]*Gu1[2] + Q11[19]*Gu1[5] + Q11[20]*Gu1[8] + Q11[21]*Gu1[11] + Q11[22]*Gu1[14] + Q11[23]*Gu1[17] + Q11[24]*Gu1[20] + Q11[25]*Gu1[23] + Q11[26]*Gu1[26] + Gu2[8];
Gu3[9] = + Q11[27]*Gu1[0] + Q11[28]*Gu1[3] + Q11[29]*Gu1[6] + Q11[30]*Gu1[9] + Q11[31]*Gu1[12] + Q11[32]*Gu1[15] + Q11[33]*Gu1[18] + Q11[34]*Gu1[21] + Q11[35]*Gu1[24] + Gu2[9];
Gu3[10] = + Q11[27]*Gu1[1] + Q11[28]*Gu1[4] + Q11[29]*Gu1[7] + Q11[30]*Gu1[10] + Q11[31]*Gu1[13] + Q11[32]*Gu1[16] + Q11[33]*Gu1[19] + Q11[34]*Gu1[22] + Q11[35]*Gu1[25] + Gu2[10];
Gu3[11] = + Q11[27]*Gu1[2] + Q11[28]*Gu1[5] + Q11[29]*Gu1[8] + Q11[30]*Gu1[11] + Q11[31]*Gu1[14] + Q11[32]*Gu1[17] + Q11[33]*Gu1[20] + Q11[34]*Gu1[23] + Q11[35]*Gu1[26] + Gu2[11];
Gu3[12] = + Q11[36]*Gu1[0] + Q11[37]*Gu1[3] + Q11[38]*Gu1[6] + Q11[39]*Gu1[9] + Q11[40]*Gu1[12] + Q11[41]*Gu1[15] + Q11[42]*Gu1[18] + Q11[43]*Gu1[21] + Q11[44]*Gu1[24] + Gu2[12];
Gu3[13] = + Q11[36]*Gu1[1] + Q11[37]*Gu1[4] + Q11[38]*Gu1[7] + Q11[39]*Gu1[10] + Q11[40]*Gu1[13] + Q11[41]*Gu1[16] + Q11[42]*Gu1[19] + Q11[43]*Gu1[22] + Q11[44]*Gu1[25] + Gu2[13];
Gu3[14] = + Q11[36]*Gu1[2] + Q11[37]*Gu1[5] + Q11[38]*Gu1[8] + Q11[39]*Gu1[11] + Q11[40]*Gu1[14] + Q11[41]*Gu1[17] + Q11[42]*Gu1[20] + Q11[43]*Gu1[23] + Q11[44]*Gu1[26] + Gu2[14];
Gu3[15] = + Q11[45]*Gu1[0] + Q11[46]*Gu1[3] + Q11[47]*Gu1[6] + Q11[48]*Gu1[9] + Q11[49]*Gu1[12] + Q11[50]*Gu1[15] + Q11[51]*Gu1[18] + Q11[52]*Gu1[21] + Q11[53]*Gu1[24] + Gu2[15];
Gu3[16] = + Q11[45]*Gu1[1] + Q11[46]*Gu1[4] + Q11[47]*Gu1[7] + Q11[48]*Gu1[10] + Q11[49]*Gu1[13] + Q11[50]*Gu1[16] + Q11[51]*Gu1[19] + Q11[52]*Gu1[22] + Q11[53]*Gu1[25] + Gu2[16];
Gu3[17] = + Q11[45]*Gu1[2] + Q11[46]*Gu1[5] + Q11[47]*Gu1[8] + Q11[48]*Gu1[11] + Q11[49]*Gu1[14] + Q11[50]*Gu1[17] + Q11[51]*Gu1[20] + Q11[52]*Gu1[23] + Q11[53]*Gu1[26] + Gu2[17];
Gu3[18] = + Q11[54]*Gu1[0] + Q11[55]*Gu1[3] + Q11[56]*Gu1[6] + Q11[57]*Gu1[9] + Q11[58]*Gu1[12] + Q11[59]*Gu1[15] + Q11[60]*Gu1[18] + Q11[61]*Gu1[21] + Q11[62]*Gu1[24] + Gu2[18];
Gu3[19] = + Q11[54]*Gu1[1] + Q11[55]*Gu1[4] + Q11[56]*Gu1[7] + Q11[57]*Gu1[10] + Q11[58]*Gu1[13] + Q11[59]*Gu1[16] + Q11[60]*Gu1[19] + Q11[61]*Gu1[22] + Q11[62]*Gu1[25] + Gu2[19];
Gu3[20] = + Q11[54]*Gu1[2] + Q11[55]*Gu1[5] + Q11[56]*Gu1[8] + Q11[57]*Gu1[11] + Q11[58]*Gu1[14] + Q11[59]*Gu1[17] + Q11[60]*Gu1[20] + Q11[61]*Gu1[23] + Q11[62]*Gu1[26] + Gu2[20];
Gu3[21] = + Q11[63]*Gu1[0] + Q11[64]*Gu1[3] + Q11[65]*Gu1[6] + Q11[66]*Gu1[9] + Q11[67]*Gu1[12] + Q11[68]*Gu1[15] + Q11[69]*Gu1[18] + Q11[70]*Gu1[21] + Q11[71]*Gu1[24] + Gu2[21];
Gu3[22] = + Q11[63]*Gu1[1] + Q11[64]*Gu1[4] + Q11[65]*Gu1[7] + Q11[66]*Gu1[10] + Q11[67]*Gu1[13] + Q11[68]*Gu1[16] + Q11[69]*Gu1[19] + Q11[70]*Gu1[22] + Q11[71]*Gu1[25] + Gu2[22];
Gu3[23] = + Q11[63]*Gu1[2] + Q11[64]*Gu1[5] + Q11[65]*Gu1[8] + Q11[66]*Gu1[11] + Q11[67]*Gu1[14] + Q11[68]*Gu1[17] + Q11[69]*Gu1[20] + Q11[70]*Gu1[23] + Q11[71]*Gu1[26] + Gu2[23];
Gu3[24] = + Q11[72]*Gu1[0] + Q11[73]*Gu1[3] + Q11[74]*Gu1[6] + Q11[75]*Gu1[9] + Q11[76]*Gu1[12] + Q11[77]*Gu1[15] + Q11[78]*Gu1[18] + Q11[79]*Gu1[21] + Q11[80]*Gu1[24] + Gu2[24];
Gu3[25] = + Q11[72]*Gu1[1] + Q11[73]*Gu1[4] + Q11[74]*Gu1[7] + Q11[75]*Gu1[10] + Q11[76]*Gu1[13] + Q11[77]*Gu1[16] + Q11[78]*Gu1[19] + Q11[79]*Gu1[22] + Q11[80]*Gu1[25] + Gu2[25];
Gu3[26] = + Q11[72]*Gu1[2] + Q11[73]*Gu1[5] + Q11[74]*Gu1[8] + Q11[75]*Gu1[11] + Q11[76]*Gu1[14] + Q11[77]*Gu1[17] + Q11[78]*Gu1[20] + Q11[79]*Gu1[23] + Q11[80]*Gu1[26] + Gu2[26];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[9]*w11[1] + Gx1[18]*w11[2] + Gx1[27]*w11[3] + Gx1[36]*w11[4] + Gx1[45]*w11[5] + Gx1[54]*w11[6] + Gx1[63]*w11[7] + Gx1[72]*w11[8] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[10]*w11[1] + Gx1[19]*w11[2] + Gx1[28]*w11[3] + Gx1[37]*w11[4] + Gx1[46]*w11[5] + Gx1[55]*w11[6] + Gx1[64]*w11[7] + Gx1[73]*w11[8] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[11]*w11[1] + Gx1[20]*w11[2] + Gx1[29]*w11[3] + Gx1[38]*w11[4] + Gx1[47]*w11[5] + Gx1[56]*w11[6] + Gx1[65]*w11[7] + Gx1[74]*w11[8] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[12]*w11[1] + Gx1[21]*w11[2] + Gx1[30]*w11[3] + Gx1[39]*w11[4] + Gx1[48]*w11[5] + Gx1[57]*w11[6] + Gx1[66]*w11[7] + Gx1[75]*w11[8] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[13]*w11[1] + Gx1[22]*w11[2] + Gx1[31]*w11[3] + Gx1[40]*w11[4] + Gx1[49]*w11[5] + Gx1[58]*w11[6] + Gx1[67]*w11[7] + Gx1[76]*w11[8] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[14]*w11[1] + Gx1[23]*w11[2] + Gx1[32]*w11[3] + Gx1[41]*w11[4] + Gx1[50]*w11[5] + Gx1[59]*w11[6] + Gx1[68]*w11[7] + Gx1[77]*w11[8] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[15]*w11[1] + Gx1[24]*w11[2] + Gx1[33]*w11[3] + Gx1[42]*w11[4] + Gx1[51]*w11[5] + Gx1[60]*w11[6] + Gx1[69]*w11[7] + Gx1[78]*w11[8] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[16]*w11[1] + Gx1[25]*w11[2] + Gx1[34]*w11[3] + Gx1[43]*w11[4] + Gx1[52]*w11[5] + Gx1[61]*w11[6] + Gx1[70]*w11[7] + Gx1[79]*w11[8] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[17]*w11[1] + Gx1[26]*w11[2] + Gx1[35]*w11[3] + Gx1[44]*w11[4] + Gx1[53]*w11[5] + Gx1[62]*w11[6] + Gx1[71]*w11[7] + Gx1[80]*w11[8] + w12[8];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8];
}

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + w12[0];
w13[1] = + Q11[9]*w11[0] + Q11[10]*w11[1] + Q11[11]*w11[2] + Q11[12]*w11[3] + Q11[13]*w11[4] + Q11[14]*w11[5] + Q11[15]*w11[6] + Q11[16]*w11[7] + Q11[17]*w11[8] + w12[1];
w13[2] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + Q11[24]*w11[6] + Q11[25]*w11[7] + Q11[26]*w11[8] + w12[2];
w13[3] = + Q11[27]*w11[0] + Q11[28]*w11[1] + Q11[29]*w11[2] + Q11[30]*w11[3] + Q11[31]*w11[4] + Q11[32]*w11[5] + Q11[33]*w11[6] + Q11[34]*w11[7] + Q11[35]*w11[8] + w12[3];
w13[4] = + Q11[36]*w11[0] + Q11[37]*w11[1] + Q11[38]*w11[2] + Q11[39]*w11[3] + Q11[40]*w11[4] + Q11[41]*w11[5] + Q11[42]*w11[6] + Q11[43]*w11[7] + Q11[44]*w11[8] + w12[4];
w13[5] = + Q11[45]*w11[0] + Q11[46]*w11[1] + Q11[47]*w11[2] + Q11[48]*w11[3] + Q11[49]*w11[4] + Q11[50]*w11[5] + Q11[51]*w11[6] + Q11[52]*w11[7] + Q11[53]*w11[8] + w12[5];
w13[6] = + Q11[54]*w11[0] + Q11[55]*w11[1] + Q11[56]*w11[2] + Q11[57]*w11[3] + Q11[58]*w11[4] + Q11[59]*w11[5] + Q11[60]*w11[6] + Q11[61]*w11[7] + Q11[62]*w11[8] + w12[6];
w13[7] = + Q11[63]*w11[0] + Q11[64]*w11[1] + Q11[65]*w11[2] + Q11[66]*w11[3] + Q11[67]*w11[4] + Q11[68]*w11[5] + Q11[69]*w11[6] + Q11[70]*w11[7] + Q11[71]*w11[8] + w12[7];
w13[8] = + Q11[72]*w11[0] + Q11[73]*w11[1] + Q11[74]*w11[2] + Q11[75]*w11[3] + Q11[76]*w11[4] + Q11[77]*w11[5] + Q11[78]*w11[6] + Q11[79]*w11[7] + Q11[80]*w11[8] + w12[8];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
w12[4] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2];
w12[5] += + Gu1[15]*U1[0] + Gu1[16]*U1[1] + Gu1[17]*U1[2];
w12[6] += + Gu1[18]*U1[0] + Gu1[19]*U1[1] + Gu1[20]*U1[2];
w12[7] += + Gu1[21]*U1[0] + Gu1[22]*U1[1] + Gu1[23]*U1[2];
w12[8] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11];
RDy1[1] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5] + R2[18]*Dy1[6] + R2[19]*Dy1[7] + R2[20]*Dy1[8] + R2[21]*Dy1[9] + R2[22]*Dy1[10] + R2[23]*Dy1[11];
RDy1[2] = + R2[24]*Dy1[0] + R2[25]*Dy1[1] + R2[26]*Dy1[2] + R2[27]*Dy1[3] + R2[28]*Dy1[4] + R2[29]*Dy1[5] + R2[30]*Dy1[6] + R2[31]*Dy1[7] + R2[32]*Dy1[8] + R2[33]*Dy1[9] + R2[34]*Dy1[10] + R2[35]*Dy1[11];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11];
QDy1[1] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5] + Q2[18]*Dy1[6] + Q2[19]*Dy1[7] + Q2[20]*Dy1[8] + Q2[21]*Dy1[9] + Q2[22]*Dy1[10] + Q2[23]*Dy1[11];
QDy1[2] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7] + Q2[32]*Dy1[8] + Q2[33]*Dy1[9] + Q2[34]*Dy1[10] + Q2[35]*Dy1[11];
QDy1[3] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5] + Q2[42]*Dy1[6] + Q2[43]*Dy1[7] + Q2[44]*Dy1[8] + Q2[45]*Dy1[9] + Q2[46]*Dy1[10] + Q2[47]*Dy1[11];
QDy1[4] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5] + Q2[54]*Dy1[6] + Q2[55]*Dy1[7] + Q2[56]*Dy1[8] + Q2[57]*Dy1[9] + Q2[58]*Dy1[10] + Q2[59]*Dy1[11];
QDy1[5] = + Q2[60]*Dy1[0] + Q2[61]*Dy1[1] + Q2[62]*Dy1[2] + Q2[63]*Dy1[3] + Q2[64]*Dy1[4] + Q2[65]*Dy1[5] + Q2[66]*Dy1[6] + Q2[67]*Dy1[7] + Q2[68]*Dy1[8] + Q2[69]*Dy1[9] + Q2[70]*Dy1[10] + Q2[71]*Dy1[11];
QDy1[6] = + Q2[72]*Dy1[0] + Q2[73]*Dy1[1] + Q2[74]*Dy1[2] + Q2[75]*Dy1[3] + Q2[76]*Dy1[4] + Q2[77]*Dy1[5] + Q2[78]*Dy1[6] + Q2[79]*Dy1[7] + Q2[80]*Dy1[8] + Q2[81]*Dy1[9] + Q2[82]*Dy1[10] + Q2[83]*Dy1[11];
QDy1[7] = + Q2[84]*Dy1[0] + Q2[85]*Dy1[1] + Q2[86]*Dy1[2] + Q2[87]*Dy1[3] + Q2[88]*Dy1[4] + Q2[89]*Dy1[5] + Q2[90]*Dy1[6] + Q2[91]*Dy1[7] + Q2[92]*Dy1[8] + Q2[93]*Dy1[9] + Q2[94]*Dy1[10] + Q2[95]*Dy1[11];
QDy1[8] = + Q2[96]*Dy1[0] + Q2[97]*Dy1[1] + Q2[98]*Dy1[2] + Q2[99]*Dy1[3] + Q2[100]*Dy1[4] + Q2[101]*Dy1[5] + Q2[102]*Dy1[6] + Q2[103]*Dy1[7] + Q2[104]*Dy1[8] + Q2[105]*Dy1[9] + Q2[106]*Dy1[10] + Q2[107]*Dy1[11];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 27 ]), &(acadoWorkspace.E[ lRun3 * 27 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (9)) * (9)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (9)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (9)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 27 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 27 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 81 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 81 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 27 ]), acadoWorkspace.W1, lRun2 );
}

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];


}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
for (lRun1 = 0; lRun1 < 240; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 108 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 324 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 396 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 432 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 468 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 504 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 576 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 612 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 648 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 684 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.g[ 57 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 216 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 324 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 648 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 756 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 972 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1080 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1188 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1296 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1404 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1512 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1620 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1728 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1836 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 153 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1944 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 162 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2052 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.QDy[ 171 ]) );

acadoWorkspace.QDy[180] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[181] = + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[182] = + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[183] = + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[184] = + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[185] = + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[186] = + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[187] = + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[188] = + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[6];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[180];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[181];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[182];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[183];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[184];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[185];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[186];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[187];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[188];
acado_macBTw1( &(acadoWorkspace.evGu[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1539 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1458 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1377 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1215 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1134 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1053 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );
acado_macS1TSbar( acadoWorkspace.S1, acadoWorkspace.sbar, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 81 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 135 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.evGu[ 243 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.evGu[ 297 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.evGu[ 351 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.evGu[ 378 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.evGu[ 405 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.evGu[ 459 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.evGu[ 486 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.evGu[ 513 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );
for (lRun1 = 0; lRun1 < 189; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -7.8539816339744828e-01;
acadoVariables.lbValues[1] = -7.8539816339744828e-01;
acadoVariables.lbValues[2] = 4.9032999999999998e+00;
acadoVariables.lbValues[3] = -7.8539816339744828e-01;
acadoVariables.lbValues[4] = -7.8539816339744828e-01;
acadoVariables.lbValues[5] = 4.9032999999999998e+00;
acadoVariables.lbValues[6] = -7.8539816339744828e-01;
acadoVariables.lbValues[7] = -7.8539816339744828e-01;
acadoVariables.lbValues[8] = 4.9032999999999998e+00;
acadoVariables.lbValues[9] = -7.8539816339744828e-01;
acadoVariables.lbValues[10] = -7.8539816339744828e-01;
acadoVariables.lbValues[11] = 4.9032999999999998e+00;
acadoVariables.lbValues[12] = -7.8539816339744828e-01;
acadoVariables.lbValues[13] = -7.8539816339744828e-01;
acadoVariables.lbValues[14] = 4.9032999999999998e+00;
acadoVariables.lbValues[15] = -7.8539816339744828e-01;
acadoVariables.lbValues[16] = -7.8539816339744828e-01;
acadoVariables.lbValues[17] = 4.9032999999999998e+00;
acadoVariables.lbValues[18] = -7.8539816339744828e-01;
acadoVariables.lbValues[19] = -7.8539816339744828e-01;
acadoVariables.lbValues[20] = 4.9032999999999998e+00;
acadoVariables.lbValues[21] = -7.8539816339744828e-01;
acadoVariables.lbValues[22] = -7.8539816339744828e-01;
acadoVariables.lbValues[23] = 4.9032999999999998e+00;
acadoVariables.lbValues[24] = -7.8539816339744828e-01;
acadoVariables.lbValues[25] = -7.8539816339744828e-01;
acadoVariables.lbValues[26] = 4.9032999999999998e+00;
acadoVariables.lbValues[27] = -7.8539816339744828e-01;
acadoVariables.lbValues[28] = -7.8539816339744828e-01;
acadoVariables.lbValues[29] = 4.9032999999999998e+00;
acadoVariables.lbValues[30] = -7.8539816339744828e-01;
acadoVariables.lbValues[31] = -7.8539816339744828e-01;
acadoVariables.lbValues[32] = 4.9032999999999998e+00;
acadoVariables.lbValues[33] = -7.8539816339744828e-01;
acadoVariables.lbValues[34] = -7.8539816339744828e-01;
acadoVariables.lbValues[35] = 4.9032999999999998e+00;
acadoVariables.lbValues[36] = -7.8539816339744828e-01;
acadoVariables.lbValues[37] = -7.8539816339744828e-01;
acadoVariables.lbValues[38] = 4.9032999999999998e+00;
acadoVariables.lbValues[39] = -7.8539816339744828e-01;
acadoVariables.lbValues[40] = -7.8539816339744828e-01;
acadoVariables.lbValues[41] = 4.9032999999999998e+00;
acadoVariables.lbValues[42] = -7.8539816339744828e-01;
acadoVariables.lbValues[43] = -7.8539816339744828e-01;
acadoVariables.lbValues[44] = 4.9032999999999998e+00;
acadoVariables.lbValues[45] = -7.8539816339744828e-01;
acadoVariables.lbValues[46] = -7.8539816339744828e-01;
acadoVariables.lbValues[47] = 4.9032999999999998e+00;
acadoVariables.lbValues[48] = -7.8539816339744828e-01;
acadoVariables.lbValues[49] = -7.8539816339744828e-01;
acadoVariables.lbValues[50] = 4.9032999999999998e+00;
acadoVariables.lbValues[51] = -7.8539816339744828e-01;
acadoVariables.lbValues[52] = -7.8539816339744828e-01;
acadoVariables.lbValues[53] = 4.9032999999999998e+00;
acadoVariables.lbValues[54] = -7.8539816339744828e-01;
acadoVariables.lbValues[55] = -7.8539816339744828e-01;
acadoVariables.lbValues[56] = 4.9032999999999998e+00;
acadoVariables.lbValues[57] = -7.8539816339744828e-01;
acadoVariables.lbValues[58] = -7.8539816339744828e-01;
acadoVariables.lbValues[59] = 4.9032999999999998e+00;
acadoVariables.ubValues[0] = 7.8539816339744828e-01;
acadoVariables.ubValues[1] = 7.8539816339744828e-01;
acadoVariables.ubValues[2] = 1.4709899999999999e+01;
acadoVariables.ubValues[3] = 7.8539816339744828e-01;
acadoVariables.ubValues[4] = 7.8539816339744828e-01;
acadoVariables.ubValues[5] = 1.4709899999999999e+01;
acadoVariables.ubValues[6] = 7.8539816339744828e-01;
acadoVariables.ubValues[7] = 7.8539816339744828e-01;
acadoVariables.ubValues[8] = 1.4709899999999999e+01;
acadoVariables.ubValues[9] = 7.8539816339744828e-01;
acadoVariables.ubValues[10] = 7.8539816339744828e-01;
acadoVariables.ubValues[11] = 1.4709899999999999e+01;
acadoVariables.ubValues[12] = 7.8539816339744828e-01;
acadoVariables.ubValues[13] = 7.8539816339744828e-01;
acadoVariables.ubValues[14] = 1.4709899999999999e+01;
acadoVariables.ubValues[15] = 7.8539816339744828e-01;
acadoVariables.ubValues[16] = 7.8539816339744828e-01;
acadoVariables.ubValues[17] = 1.4709899999999999e+01;
acadoVariables.ubValues[18] = 7.8539816339744828e-01;
acadoVariables.ubValues[19] = 7.8539816339744828e-01;
acadoVariables.ubValues[20] = 1.4709899999999999e+01;
acadoVariables.ubValues[21] = 7.8539816339744828e-01;
acadoVariables.ubValues[22] = 7.8539816339744828e-01;
acadoVariables.ubValues[23] = 1.4709899999999999e+01;
acadoVariables.ubValues[24] = 7.8539816339744828e-01;
acadoVariables.ubValues[25] = 7.8539816339744828e-01;
acadoVariables.ubValues[26] = 1.4709899999999999e+01;
acadoVariables.ubValues[27] = 7.8539816339744828e-01;
acadoVariables.ubValues[28] = 7.8539816339744828e-01;
acadoVariables.ubValues[29] = 1.4709899999999999e+01;
acadoVariables.ubValues[30] = 7.8539816339744828e-01;
acadoVariables.ubValues[31] = 7.8539816339744828e-01;
acadoVariables.ubValues[32] = 1.4709899999999999e+01;
acadoVariables.ubValues[33] = 7.8539816339744828e-01;
acadoVariables.ubValues[34] = 7.8539816339744828e-01;
acadoVariables.ubValues[35] = 1.4709899999999999e+01;
acadoVariables.ubValues[36] = 7.8539816339744828e-01;
acadoVariables.ubValues[37] = 7.8539816339744828e-01;
acadoVariables.ubValues[38] = 1.4709899999999999e+01;
acadoVariables.ubValues[39] = 7.8539816339744828e-01;
acadoVariables.ubValues[40] = 7.8539816339744828e-01;
acadoVariables.ubValues[41] = 1.4709899999999999e+01;
acadoVariables.ubValues[42] = 7.8539816339744828e-01;
acadoVariables.ubValues[43] = 7.8539816339744828e-01;
acadoVariables.ubValues[44] = 1.4709899999999999e+01;
acadoVariables.ubValues[45] = 7.8539816339744828e-01;
acadoVariables.ubValues[46] = 7.8539816339744828e-01;
acadoVariables.ubValues[47] = 1.4709899999999999e+01;
acadoVariables.ubValues[48] = 7.8539816339744828e-01;
acadoVariables.ubValues[49] = 7.8539816339744828e-01;
acadoVariables.ubValues[50] = 1.4709899999999999e+01;
acadoVariables.ubValues[51] = 7.8539816339744828e-01;
acadoVariables.ubValues[52] = 7.8539816339744828e-01;
acadoVariables.ubValues[53] = 1.4709899999999999e+01;
acadoVariables.ubValues[54] = 7.8539816339744828e-01;
acadoVariables.ubValues[55] = 7.8539816339744828e-01;
acadoVariables.ubValues[56] = 1.4709899999999999e+01;
acadoVariables.ubValues[57] = 7.8539816339744828e-01;
acadoVariables.ubValues[58] = 7.8539816339744828e-01;
acadoVariables.ubValues[59] = 1.4709899999999999e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
state[0] = acadoVariables.x[index * 9];
state[1] = acadoVariables.x[index * 9 + 1];
state[2] = acadoVariables.x[index * 9 + 2];
state[3] = acadoVariables.x[index * 9 + 3];
state[4] = acadoVariables.x[index * 9 + 4];
state[5] = acadoVariables.x[index * 9 + 5];
state[6] = acadoVariables.x[index * 9 + 6];
state[7] = acadoVariables.x[index * 9 + 7];
state[8] = acadoVariables.x[index * 9 + 8];
state[117] = acadoVariables.u[index * 3];
state[118] = acadoVariables.u[index * 3 + 1];
state[119] = acadoVariables.u[index * 3 + 2];
state[120] = acadoVariables.od[index * 15];
state[121] = acadoVariables.od[index * 15 + 1];
state[122] = acadoVariables.od[index * 15 + 2];
state[123] = acadoVariables.od[index * 15 + 3];
state[124] = acadoVariables.od[index * 15 + 4];
state[125] = acadoVariables.od[index * 15 + 5];
state[126] = acadoVariables.od[index * 15 + 6];
state[127] = acadoVariables.od[index * 15 + 7];
state[128] = acadoVariables.od[index * 15 + 8];
state[129] = acadoVariables.od[index * 15 + 9];
state[130] = acadoVariables.od[index * 15 + 10];
state[131] = acadoVariables.od[index * 15 + 11];
state[132] = acadoVariables.od[index * 15 + 12];
state[133] = acadoVariables.od[index * 15 + 13];
state[134] = acadoVariables.od[index * 15 + 14];

acado_integrate(state, index == 0);

acadoVariables.x[index * 9 + 9] = state[0];
acadoVariables.x[index * 9 + 10] = state[1];
acadoVariables.x[index * 9 + 11] = state[2];
acadoVariables.x[index * 9 + 12] = state[3];
acadoVariables.x[index * 9 + 13] = state[4];
acadoVariables.x[index * 9 + 14] = state[5];
acadoVariables.x[index * 9 + 15] = state[6];
acadoVariables.x[index * 9 + 16] = state[7];
acadoVariables.x[index * 9 + 17] = state[8];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 9] = acadoVariables.x[index * 9 + 9];
acadoVariables.x[index * 9 + 1] = acadoVariables.x[index * 9 + 10];
acadoVariables.x[index * 9 + 2] = acadoVariables.x[index * 9 + 11];
acadoVariables.x[index * 9 + 3] = acadoVariables.x[index * 9 + 12];
acadoVariables.x[index * 9 + 4] = acadoVariables.x[index * 9 + 13];
acadoVariables.x[index * 9 + 5] = acadoVariables.x[index * 9 + 14];
acadoVariables.x[index * 9 + 6] = acadoVariables.x[index * 9 + 15];
acadoVariables.x[index * 9 + 7] = acadoVariables.x[index * 9 + 16];
acadoVariables.x[index * 9 + 8] = acadoVariables.x[index * 9 + 17];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[180] = xEnd[0];
acadoVariables.x[181] = xEnd[1];
acadoVariables.x[182] = xEnd[2];
acadoVariables.x[183] = xEnd[3];
acadoVariables.x[184] = xEnd[4];
acadoVariables.x[185] = xEnd[5];
acadoVariables.x[186] = xEnd[6];
acadoVariables.x[187] = xEnd[7];
acadoVariables.x[188] = xEnd[8];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[180];
state[1] = acadoVariables.x[181];
state[2] = acadoVariables.x[182];
state[3] = acadoVariables.x[183];
state[4] = acadoVariables.x[184];
state[5] = acadoVariables.x[185];
state[6] = acadoVariables.x[186];
state[7] = acadoVariables.x[187];
state[8] = acadoVariables.x[188];
if (uEnd != 0)
{
state[117] = uEnd[0];
state[118] = uEnd[1];
state[119] = uEnd[2];
}
else
{
state[117] = acadoVariables.u[57];
state[118] = acadoVariables.u[58];
state[119] = acadoVariables.u[59];
}
state[120] = acadoVariables.od[300];
state[121] = acadoVariables.od[301];
state[122] = acadoVariables.od[302];
state[123] = acadoVariables.od[303];
state[124] = acadoVariables.od[304];
state[125] = acadoVariables.od[305];
state[126] = acadoVariables.od[306];
state[127] = acadoVariables.od[307];
state[128] = acadoVariables.od[308];
state[129] = acadoVariables.od[309];
state[130] = acadoVariables.od[310];
state[131] = acadoVariables.od[311];
state[132] = acadoVariables.od[312];
state[133] = acadoVariables.od[313];
state[134] = acadoVariables.od[314];

acado_integrate(state, 1);

acadoVariables.x[180] = state[0];
acadoVariables.x[181] = state[1];
acadoVariables.x[182] = state[2];
acadoVariables.x[183] = state[3];
acadoVariables.x[184] = state[4];
acadoVariables.x[185] = state[5];
acadoVariables.x[186] = state[6];
acadoVariables.x[187] = state[7];
acadoVariables.x[188] = state[8];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[57] = uEnd[0];
acadoVariables.u[58] = uEnd[1];
acadoVariables.u[59] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 12 */
real_t tmpDy[ 12 ];

/** Row vector of size: 7 */
real_t tmpDyN[ 7 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 15];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 15 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 15 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 15 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 15 + 4];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 15 + 5];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 15 + 6];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 15 + 7];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 15 + 8];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 15 + 9];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 15 + 10];
acadoWorkspace.objValueIn[23] = acadoVariables.od[lRun1 * 15 + 11];
acadoWorkspace.objValueIn[24] = acadoVariables.od[lRun1 * 15 + 12];
acadoWorkspace.objValueIn[25] = acadoVariables.od[lRun1 * 15 + 13];
acadoWorkspace.objValueIn[26] = acadoVariables.od[lRun1 * 15 + 14];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 12] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 12];
acadoWorkspace.Dy[lRun1 * 12 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 12 + 1];
acadoWorkspace.Dy[lRun1 * 12 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 12 + 2];
acadoWorkspace.Dy[lRun1 * 12 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 12 + 3];
acadoWorkspace.Dy[lRun1 * 12 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 12 + 4];
acadoWorkspace.Dy[lRun1 * 12 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 12 + 5];
acadoWorkspace.Dy[lRun1 * 12 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 12 + 6];
acadoWorkspace.Dy[lRun1 * 12 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 12 + 7];
acadoWorkspace.Dy[lRun1 * 12 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 12 + 8];
acadoWorkspace.Dy[lRun1 * 12 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 12 + 9];
acadoWorkspace.Dy[lRun1 * 12 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 12 + 10];
acadoWorkspace.Dy[lRun1 * 12 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 12 + 11];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.x[183];
acadoWorkspace.objValueIn[4] = acadoVariables.x[184];
acadoWorkspace.objValueIn[5] = acadoVariables.x[185];
acadoWorkspace.objValueIn[6] = acadoVariables.x[186];
acadoWorkspace.objValueIn[7] = acadoVariables.x[187];
acadoWorkspace.objValueIn[8] = acadoVariables.x[188];
acadoWorkspace.objValueIn[9] = acadoVariables.od[300];
acadoWorkspace.objValueIn[10] = acadoVariables.od[301];
acadoWorkspace.objValueIn[11] = acadoVariables.od[302];
acadoWorkspace.objValueIn[12] = acadoVariables.od[303];
acadoWorkspace.objValueIn[13] = acadoVariables.od[304];
acadoWorkspace.objValueIn[14] = acadoVariables.od[305];
acadoWorkspace.objValueIn[15] = acadoVariables.od[306];
acadoWorkspace.objValueIn[16] = acadoVariables.od[307];
acadoWorkspace.objValueIn[17] = acadoVariables.od[308];
acadoWorkspace.objValueIn[18] = acadoVariables.od[309];
acadoWorkspace.objValueIn[19] = acadoVariables.od[310];
acadoWorkspace.objValueIn[20] = acadoVariables.od[311];
acadoWorkspace.objValueIn[21] = acadoVariables.od[312];
acadoWorkspace.objValueIn[22] = acadoVariables.od[313];
acadoWorkspace.objValueIn[23] = acadoVariables.od[314];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[13];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[26];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[39];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[52];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[65];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[78];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[91];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[104];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[117];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[130];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[143];
objVal += + acadoWorkspace.Dy[lRun1 * 12]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 12 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 12 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 12 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 12 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 12 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 12 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 12 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 12 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 12 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 12 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 12 + 11]*tmpDy[11];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[8];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[16];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[24];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[32];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[40];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[48];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6];

objVal *= 0.5;
return objVal;
}

