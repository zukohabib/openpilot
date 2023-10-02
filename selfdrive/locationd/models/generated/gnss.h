#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2877063747984525498);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_9070657437256373961);
void gnss_H_mod_fun(double *state, double *out_2369205197323246875);
void gnss_f_fun(double *state, double dt, double *out_6668905213867798645);
void gnss_F_fun(double *state, double dt, double *out_3396924666748295075);
void gnss_h_6(double *state, double *sat_pos, double *out_3860577417230869744);
void gnss_H_6(double *state, double *sat_pos, double *out_8831843007616136562);
void gnss_h_20(double *state, double *sat_pos, double *out_3800497205520904063);
void gnss_H_20(double *state, double *sat_pos, double *out_653013259064409177);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_267143536851933288);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_6387361506978117576);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_267143536851933288);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_6387361506978117576);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}