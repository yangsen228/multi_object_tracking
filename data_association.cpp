#include "data_association.h"

// 数据关联
void dataAssociation(map<uint8_t, FusionObject> &g_map, const vector<FusionObject> &new_obj, Eigen::MatrixXd &incidence_matrix)
{
    cout << "dataAssociation: start" << endl;
    int rows = new_obj.size();
    int cols = g_map.size();
    cout << "rows: " << rows << "cols: " << cols << endl;
    Eigen::MatrixXd matrix_0, matrix_1, matrix_2;
    matrix_0 = Eigen::MatrixXd::Zero(rows, cols);
    matrix_1 = Eigen::MatrixXd::Zero(rows, cols);
    matrix_2 = Eigen::MatrixXd::Zero(rows, cols);
    map<uint8_t, FusionObject>::iterator it;
    // 将matrix_0中全局目标跟新过的位置置0,其余填入IOU
    for(int i = 0; i < rows; i++)
    {
        // cout << "123456" << endl;
        int j = 0;
        for(it = g_map.begin(); it != g_map.end(); it++)
        {
            if((it->second).flash_flag_ == true)
            {
                cout << "flash_flag is true" << endl;
                matrix_0(i,j) = 0;
            }
            else
            {
                cout << "flash_flag is false, calIOU" << endl;
                float iou = calIOU(new_obj[i], it->second);
                if(iou > 0.7)
                {
                    matrix_0(i,j) = iou;
                }
                else
                {
                    matrix_0(i,j) = 0.0;
                }
                
            }
            j++;
        }
    }
    // 将matrix_1中的每行的最大IOU位置置1
    for(int i = 0; i < matrix_0.rows(); i++)
    {
        Eigen::MatrixXd::Index index;
        matrix_0.row(i).maxCoeff(&index);
        matrix_1(i, index) = 1;
    }
    // 将matrix_2中的每列的最大IOU位置置1
    for(int j = 0; j < matrix_0.cols(); j++)
    {
        Eigen::MatrixXd::Index index;
        matrix_0.col(j).maxCoeff(&index);
        matrix_2(index, j) = 1;
    }
    // 计算关联矩阵
    incidence_matrix = matrix_1.array() * matrix_2.array();

    cout << "dataAssociation: end" << endl;
}

// 计算IOU
float calIOU(const FusionObject &new_object, const FusionObject &global_object)
{
    cout << "calIOU: start" << endl;
    // 时间差
    double time_diff = new_object.timestamp_.toSec() - global_object.timestamp_.toSec();
    // 外推参考速度
    int vx = global_object.state_(2);
    int vy = global_object.state_(3);

    cout << "iou iou" << endl;
    
    // x1_min 第一个框x的最小值
    int x1_min, y1_min, x1_max, y1_max, x2_min, y2_min, x2_max, y2_max;
    x1_max = new_object.peak1_[0];
    y1_max = new_object.peak1_[1];
    x1_min = new_object.peak3_[0];
    y1_min = new_object.peak3_[1]; 
    // 外推后框点位置
    x2_max = global_object.peak1_[0] + time_diff * vx;
    y2_max = global_object.peak1_[1] + time_diff * vy;
    x2_min = global_object.peak3_[0] + time_diff * vx;
    y2_min = global_object.peak3_[1] + time_diff * vy;
    // x_min 重叠区域x的最小值
    int x_min, y_min, x_max, y_max;
    x_max = min(x1_max, x2_max);
    y_max = min(y1_max, y2_max);
    x_min = max(x1_min, x2_min);
    y_min = max(y1_min, y2_min);
    // 求交集面积
    int intersection_area = (x_max - x_min) * (y_max - y_min);
    // 求并集面积
    int union_area = (x1_max - x1_min) * (y1_max - y1_min) + (x2_max - x2_min) * (y2_max - y2_min) - intersection_area;
    
    cout << "calIOU: end" << endl;
    return intersection_area / float(union_area);
}