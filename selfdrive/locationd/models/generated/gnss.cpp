#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2877063747984525498) {
   out_2877063747984525498[0] = delta_x[0] + nom_x[0];
   out_2877063747984525498[1] = delta_x[1] + nom_x[1];
   out_2877063747984525498[2] = delta_x[2] + nom_x[2];
   out_2877063747984525498[3] = delta_x[3] + nom_x[3];
   out_2877063747984525498[4] = delta_x[4] + nom_x[4];
   out_2877063747984525498[5] = delta_x[5] + nom_x[5];
   out_2877063747984525498[6] = delta_x[6] + nom_x[6];
   out_2877063747984525498[7] = delta_x[7] + nom_x[7];
   out_2877063747984525498[8] = delta_x[8] + nom_x[8];
   out_2877063747984525498[9] = delta_x[9] + nom_x[9];
   out_2877063747984525498[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9070657437256373961) {
   out_9070657437256373961[0] = -nom_x[0] + true_x[0];
   out_9070657437256373961[1] = -nom_x[1] + true_x[1];
   out_9070657437256373961[2] = -nom_x[2] + true_x[2];
   out_9070657437256373961[3] = -nom_x[3] + true_x[3];
   out_9070657437256373961[4] = -nom_x[4] + true_x[4];
   out_9070657437256373961[5] = -nom_x[5] + true_x[5];
   out_9070657437256373961[6] = -nom_x[6] + true_x[6];
   out_9070657437256373961[7] = -nom_x[7] + true_x[7];
   out_9070657437256373961[8] = -nom_x[8] + true_x[8];
   out_9070657437256373961[9] = -nom_x[9] + true_x[9];
   out_9070657437256373961[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_2369205197323246875) {
   out_2369205197323246875[0] = 1.0;
   out_2369205197323246875[1] = 0;
   out_2369205197323246875[2] = 0;
   out_2369205197323246875[3] = 0;
   out_2369205197323246875[4] = 0;
   out_2369205197323246875[5] = 0;
   out_2369205197323246875[6] = 0;
   out_2369205197323246875[7] = 0;
   out_2369205197323246875[8] = 0;
   out_2369205197323246875[9] = 0;
   out_2369205197323246875[10] = 0;
   out_2369205197323246875[11] = 0;
   out_2369205197323246875[12] = 1.0;
   out_2369205197323246875[13] = 0;
   out_2369205197323246875[14] = 0;
   out_2369205197323246875[15] = 0;
   out_2369205197323246875[16] = 0;
   out_2369205197323246875[17] = 0;
   out_2369205197323246875[18] = 0;
   out_2369205197323246875[19] = 0;
   out_2369205197323246875[20] = 0;
   out_2369205197323246875[21] = 0;
   out_2369205197323246875[22] = 0;
   out_2369205197323246875[23] = 0;
   out_2369205197323246875[24] = 1.0;
   out_2369205197323246875[25] = 0;
   out_2369205197323246875[26] = 0;
   out_2369205197323246875[27] = 0;
   out_2369205197323246875[28] = 0;
   out_2369205197323246875[29] = 0;
   out_2369205197323246875[30] = 0;
   out_2369205197323246875[31] = 0;
   out_2369205197323246875[32] = 0;
   out_2369205197323246875[33] = 0;
   out_2369205197323246875[34] = 0;
   out_2369205197323246875[35] = 0;
   out_2369205197323246875[36] = 1.0;
   out_2369205197323246875[37] = 0;
   out_2369205197323246875[38] = 0;
   out_2369205197323246875[39] = 0;
   out_2369205197323246875[40] = 0;
   out_2369205197323246875[41] = 0;
   out_2369205197323246875[42] = 0;
   out_2369205197323246875[43] = 0;
   out_2369205197323246875[44] = 0;
   out_2369205197323246875[45] = 0;
   out_2369205197323246875[46] = 0;
   out_2369205197323246875[47] = 0;
   out_2369205197323246875[48] = 1.0;
   out_2369205197323246875[49] = 0;
   out_2369205197323246875[50] = 0;
   out_2369205197323246875[51] = 0;
   out_2369205197323246875[52] = 0;
   out_2369205197323246875[53] = 0;
   out_2369205197323246875[54] = 0;
   out_2369205197323246875[55] = 0;
   out_2369205197323246875[56] = 0;
   out_2369205197323246875[57] = 0;
   out_2369205197323246875[58] = 0;
   out_2369205197323246875[59] = 0;
   out_2369205197323246875[60] = 1.0;
   out_2369205197323246875[61] = 0;
   out_2369205197323246875[62] = 0;
   out_2369205197323246875[63] = 0;
   out_2369205197323246875[64] = 0;
   out_2369205197323246875[65] = 0;
   out_2369205197323246875[66] = 0;
   out_2369205197323246875[67] = 0;
   out_2369205197323246875[68] = 0;
   out_2369205197323246875[69] = 0;
   out_2369205197323246875[70] = 0;
   out_2369205197323246875[71] = 0;
   out_2369205197323246875[72] = 1.0;
   out_2369205197323246875[73] = 0;
   out_2369205197323246875[74] = 0;
   out_2369205197323246875[75] = 0;
   out_2369205197323246875[76] = 0;
   out_2369205197323246875[77] = 0;
   out_2369205197323246875[78] = 0;
   out_2369205197323246875[79] = 0;
   out_2369205197323246875[80] = 0;
   out_2369205197323246875[81] = 0;
   out_2369205197323246875[82] = 0;
   out_2369205197323246875[83] = 0;
   out_2369205197323246875[84] = 1.0;
   out_2369205197323246875[85] = 0;
   out_2369205197323246875[86] = 0;
   out_2369205197323246875[87] = 0;
   out_2369205197323246875[88] = 0;
   out_2369205197323246875[89] = 0;
   out_2369205197323246875[90] = 0;
   out_2369205197323246875[91] = 0;
   out_2369205197323246875[92] = 0;
   out_2369205197323246875[93] = 0;
   out_2369205197323246875[94] = 0;
   out_2369205197323246875[95] = 0;
   out_2369205197323246875[96] = 1.0;
   out_2369205197323246875[97] = 0;
   out_2369205197323246875[98] = 0;
   out_2369205197323246875[99] = 0;
   out_2369205197323246875[100] = 0;
   out_2369205197323246875[101] = 0;
   out_2369205197323246875[102] = 0;
   out_2369205197323246875[103] = 0;
   out_2369205197323246875[104] = 0;
   out_2369205197323246875[105] = 0;
   out_2369205197323246875[106] = 0;
   out_2369205197323246875[107] = 0;
   out_2369205197323246875[108] = 1.0;
   out_2369205197323246875[109] = 0;
   out_2369205197323246875[110] = 0;
   out_2369205197323246875[111] = 0;
   out_2369205197323246875[112] = 0;
   out_2369205197323246875[113] = 0;
   out_2369205197323246875[114] = 0;
   out_2369205197323246875[115] = 0;
   out_2369205197323246875[116] = 0;
   out_2369205197323246875[117] = 0;
   out_2369205197323246875[118] = 0;
   out_2369205197323246875[119] = 0;
   out_2369205197323246875[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6668905213867798645) {
   out_6668905213867798645[0] = dt*state[3] + state[0];
   out_6668905213867798645[1] = dt*state[4] + state[1];
   out_6668905213867798645[2] = dt*state[5] + state[2];
   out_6668905213867798645[3] = state[3];
   out_6668905213867798645[4] = state[4];
   out_6668905213867798645[5] = state[5];
   out_6668905213867798645[6] = dt*state[7] + state[6];
   out_6668905213867798645[7] = dt*state[8] + state[7];
   out_6668905213867798645[8] = state[8];
   out_6668905213867798645[9] = state[9];
   out_6668905213867798645[10] = state[10];
}
void F_fun(double *state, double dt, double *out_3396924666748295075) {
   out_3396924666748295075[0] = 1;
   out_3396924666748295075[1] = 0;
   out_3396924666748295075[2] = 0;
   out_3396924666748295075[3] = dt;
   out_3396924666748295075[4] = 0;
   out_3396924666748295075[5] = 0;
   out_3396924666748295075[6] = 0;
   out_3396924666748295075[7] = 0;
   out_3396924666748295075[8] = 0;
   out_3396924666748295075[9] = 0;
   out_3396924666748295075[10] = 0;
   out_3396924666748295075[11] = 0;
   out_3396924666748295075[12] = 1;
   out_3396924666748295075[13] = 0;
   out_3396924666748295075[14] = 0;
   out_3396924666748295075[15] = dt;
   out_3396924666748295075[16] = 0;
   out_3396924666748295075[17] = 0;
   out_3396924666748295075[18] = 0;
   out_3396924666748295075[19] = 0;
   out_3396924666748295075[20] = 0;
   out_3396924666748295075[21] = 0;
   out_3396924666748295075[22] = 0;
   out_3396924666748295075[23] = 0;
   out_3396924666748295075[24] = 1;
   out_3396924666748295075[25] = 0;
   out_3396924666748295075[26] = 0;
   out_3396924666748295075[27] = dt;
   out_3396924666748295075[28] = 0;
   out_3396924666748295075[29] = 0;
   out_3396924666748295075[30] = 0;
   out_3396924666748295075[31] = 0;
   out_3396924666748295075[32] = 0;
   out_3396924666748295075[33] = 0;
   out_3396924666748295075[34] = 0;
   out_3396924666748295075[35] = 0;
   out_3396924666748295075[36] = 1;
   out_3396924666748295075[37] = 0;
   out_3396924666748295075[38] = 0;
   out_3396924666748295075[39] = 0;
   out_3396924666748295075[40] = 0;
   out_3396924666748295075[41] = 0;
   out_3396924666748295075[42] = 0;
   out_3396924666748295075[43] = 0;
   out_3396924666748295075[44] = 0;
   out_3396924666748295075[45] = 0;
   out_3396924666748295075[46] = 0;
   out_3396924666748295075[47] = 0;
   out_3396924666748295075[48] = 1;
   out_3396924666748295075[49] = 0;
   out_3396924666748295075[50] = 0;
   out_3396924666748295075[51] = 0;
   out_3396924666748295075[52] = 0;
   out_3396924666748295075[53] = 0;
   out_3396924666748295075[54] = 0;
   out_3396924666748295075[55] = 0;
   out_3396924666748295075[56] = 0;
   out_3396924666748295075[57] = 0;
   out_3396924666748295075[58] = 0;
   out_3396924666748295075[59] = 0;
   out_3396924666748295075[60] = 1;
   out_3396924666748295075[61] = 0;
   out_3396924666748295075[62] = 0;
   out_3396924666748295075[63] = 0;
   out_3396924666748295075[64] = 0;
   out_3396924666748295075[65] = 0;
   out_3396924666748295075[66] = 0;
   out_3396924666748295075[67] = 0;
   out_3396924666748295075[68] = 0;
   out_3396924666748295075[69] = 0;
   out_3396924666748295075[70] = 0;
   out_3396924666748295075[71] = 0;
   out_3396924666748295075[72] = 1;
   out_3396924666748295075[73] = dt;
   out_3396924666748295075[74] = 0;
   out_3396924666748295075[75] = 0;
   out_3396924666748295075[76] = 0;
   out_3396924666748295075[77] = 0;
   out_3396924666748295075[78] = 0;
   out_3396924666748295075[79] = 0;
   out_3396924666748295075[80] = 0;
   out_3396924666748295075[81] = 0;
   out_3396924666748295075[82] = 0;
   out_3396924666748295075[83] = 0;
   out_3396924666748295075[84] = 1;
   out_3396924666748295075[85] = dt;
   out_3396924666748295075[86] = 0;
   out_3396924666748295075[87] = 0;
   out_3396924666748295075[88] = 0;
   out_3396924666748295075[89] = 0;
   out_3396924666748295075[90] = 0;
   out_3396924666748295075[91] = 0;
   out_3396924666748295075[92] = 0;
   out_3396924666748295075[93] = 0;
   out_3396924666748295075[94] = 0;
   out_3396924666748295075[95] = 0;
   out_3396924666748295075[96] = 1;
   out_3396924666748295075[97] = 0;
   out_3396924666748295075[98] = 0;
   out_3396924666748295075[99] = 0;
   out_3396924666748295075[100] = 0;
   out_3396924666748295075[101] = 0;
   out_3396924666748295075[102] = 0;
   out_3396924666748295075[103] = 0;
   out_3396924666748295075[104] = 0;
   out_3396924666748295075[105] = 0;
   out_3396924666748295075[106] = 0;
   out_3396924666748295075[107] = 0;
   out_3396924666748295075[108] = 1;
   out_3396924666748295075[109] = 0;
   out_3396924666748295075[110] = 0;
   out_3396924666748295075[111] = 0;
   out_3396924666748295075[112] = 0;
   out_3396924666748295075[113] = 0;
   out_3396924666748295075[114] = 0;
   out_3396924666748295075[115] = 0;
   out_3396924666748295075[116] = 0;
   out_3396924666748295075[117] = 0;
   out_3396924666748295075[118] = 0;
   out_3396924666748295075[119] = 0;
   out_3396924666748295075[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3860577417230869744) {
   out_3860577417230869744[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8831843007616136562) {
   out_8831843007616136562[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8831843007616136562[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8831843007616136562[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8831843007616136562[3] = 0;
   out_8831843007616136562[4] = 0;
   out_8831843007616136562[5] = 0;
   out_8831843007616136562[6] = 1;
   out_8831843007616136562[7] = 0;
   out_8831843007616136562[8] = 0;
   out_8831843007616136562[9] = 0;
   out_8831843007616136562[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3800497205520904063) {
   out_3800497205520904063[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_653013259064409177) {
   out_653013259064409177[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_653013259064409177[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_653013259064409177[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_653013259064409177[3] = 0;
   out_653013259064409177[4] = 0;
   out_653013259064409177[5] = 0;
   out_653013259064409177[6] = 1;
   out_653013259064409177[7] = 0;
   out_653013259064409177[8] = 0;
   out_653013259064409177[9] = 1;
   out_653013259064409177[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_267143536851933288) {
   out_267143536851933288[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_6387361506978117576) {
   out_6387361506978117576[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[6] = 0;
   out_6387361506978117576[7] = 1;
   out_6387361506978117576[8] = 0;
   out_6387361506978117576[9] = 0;
   out_6387361506978117576[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_267143536851933288) {
   out_267143536851933288[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_6387361506978117576) {
   out_6387361506978117576[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_6387361506978117576[6] = 0;
   out_6387361506978117576[7] = 1;
   out_6387361506978117576[8] = 0;
   out_6387361506978117576[9] = 0;
   out_6387361506978117576[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2877063747984525498) {
  err_fun(nom_x, delta_x, out_2877063747984525498);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_9070657437256373961) {
  inv_err_fun(nom_x, true_x, out_9070657437256373961);
}
void gnss_H_mod_fun(double *state, double *out_2369205197323246875) {
  H_mod_fun(state, out_2369205197323246875);
}
void gnss_f_fun(double *state, double dt, double *out_6668905213867798645) {
  f_fun(state,  dt, out_6668905213867798645);
}
void gnss_F_fun(double *state, double dt, double *out_3396924666748295075) {
  F_fun(state,  dt, out_3396924666748295075);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3860577417230869744) {
  h_6(state, sat_pos, out_3860577417230869744);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8831843007616136562) {
  H_6(state, sat_pos, out_8831843007616136562);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3800497205520904063) {
  h_20(state, sat_pos, out_3800497205520904063);
}
void gnss_H_20(double *state, double *sat_pos, double *out_653013259064409177) {
  H_20(state, sat_pos, out_653013259064409177);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_267143536851933288) {
  h_7(state, sat_pos_vel, out_267143536851933288);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6387361506978117576) {
  H_7(state, sat_pos_vel, out_6387361506978117576);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_267143536851933288) {
  h_21(state, sat_pos_vel, out_267143536851933288);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6387361506978117576) {
  H_21(state, sat_pos_vel, out_6387361506978117576);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
