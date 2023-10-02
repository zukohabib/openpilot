#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6690086502958356170);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1138017061106975654);
void car_H_mod_fun(double *state, double *out_8731402594447162530);
void car_f_fun(double *state, double dt, double *out_5880714312440560991);
void car_F_fun(double *state, double dt, double *out_6668270723209344525);
void car_h_25(double *state, double *unused, double *out_6309852841180432979);
void car_H_25(double *state, double *unused, double *out_6040165476284428233);
void car_h_24(double *state, double *unused, double *out_7716672714826793939);
void car_H_24(double *state, double *unused, double *out_2482239877482105097);
void car_h_30(double *state, double *unused, double *out_2554258583712588207);
void car_H_30(double *state, double *unused, double *out_7878882267297515185);
void car_h_26(double *state, double *unused, double *out_7555931851373433852);
void car_H_26(double *state, double *unused, double *out_8665075278551067159);
void car_h_27(double *state, double *unused, double *out_3273066789736100294);
void car_H_27(double *state, double *unused, double *out_5704118955497090274);
void car_h_29(double *state, double *unused, double *out_3548260852020606183);
void car_H_29(double *state, double *unused, double *out_8389113611611907369);
void car_h_28(double *state, double *unused, double *out_668661468927402643);
void car_H_28(double *state, double *unused, double *out_3306714594542376795);
void car_h_31(double *state, double *unused, double *out_6034658778895927090);
void car_H_31(double *state, double *unused, double *out_8038867176317715683);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}