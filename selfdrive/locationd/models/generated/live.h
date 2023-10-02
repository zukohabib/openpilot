#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2001638209905338966);
void live_err_fun(double *nom_x, double *delta_x, double *out_4090277704939598758);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5202557587600642285);
void live_H_mod_fun(double *state, double *out_7949871151437297473);
void live_f_fun(double *state, double dt, double *out_3703409087435433243);
void live_F_fun(double *state, double dt, double *out_4085847992067201093);
void live_h_4(double *state, double *unused, double *out_6781006617317371273);
void live_H_4(double *state, double *unused, double *out_8242877463465214800);
void live_h_9(double *state, double *unused, double *out_7537792390691149338);
void live_H_9(double *state, double *unused, double *out_4085709727110437317);
void live_h_10(double *state, double *unused, double *out_2492288544387048668);
void live_H_10(double *state, double *unused, double *out_8728770980102530045);
void live_h_12(double *state, double *unused, double *out_3416092933119690794);
void live_H_12(double *state, double *unused, double *out_8863976488512808467);
void live_h_35(double *state, double *unused, double *out_3897801298940732081);
void live_H_35(double *state, double *unused, double *out_6837204552871729440);
void live_h_32(double *state, double *unused, double *out_4297874594603971057);
void live_H_32(double *state, double *unused, double *out_1035900180664116882);
void live_h_13(double *state, double *unused, double *out_440301787937693022);
void live_H_13(double *state, double *unused, double *out_6891277052059464278);
void live_h_14(double *state, double *unused, double *out_7537792390691149338);
void live_H_14(double *state, double *unused, double *out_4085709727110437317);
void live_h_33(double *state, double *unused, double *out_5120077427427814513);
void live_H_33(double *state, double *unused, double *out_3686647548232871836);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}