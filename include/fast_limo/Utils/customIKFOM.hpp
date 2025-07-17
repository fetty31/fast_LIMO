#include "IKFoM/use-ikfom.hpp"

namespace IKFoM {
    
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in);

Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in);

Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in);

void h_share_model(state_ikfom &, esekfom::dyn_share_datastruct<double> &);

} // namespace IKFOM