#ifndef DATA_ASSOCIATION_H
#define DATA_ASSOCIATION_H

#include <Eigen/Dense>
#include "fusion_object.h"

// data association
void dataAssociation(map<uint8_t, FusionObject> &g_map, const vector<FusionObject> &new_obj, Eigen::MatrixXd &incidence_matrix);

float calIOU(const FusionObject &new_object, const FusionObject &global_object);

#endif