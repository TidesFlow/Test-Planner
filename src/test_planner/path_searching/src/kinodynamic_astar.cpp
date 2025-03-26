#include "path_searching/kinodynamic_astar.h"

void KinodynamicAStar::init()
{
    _state_trans_matrix = Eigen::Matrix<double,6,6>::Identity();
    /* ------------ node pool init ------------------ */
    _path_node_pool.resize(_allocated_num);
    for(int i = 0;i < _allocated_num;i++){
        _path_node_pool[i] = new PathNode;
    }

    _used_node_count = 0;
    _iteration_num = 0;
}

void KinodynamicAStar::reset()
{
    std::priority_queue<PathNodePtr,std::vector<PathNodePtr>,PathNodeComparator> empty_queue;
    _open_set.swap(empty_queue);
    
    _expanded_nodes.clear();

    for(int i = 0;i < _allocated_num;i++){
        _path_node_pool[i]->_parent = NULL;
        _path_node_pool[i]->_node_state = PathNode::NodeState::UNEXPANDED;
    }

    _used_node_count = 0;
    _iteration_num = 0;
}

void KinodynamicAStar::setEnvironment(EDTEnvironment::Ptr &env)
{
    _edt_environment = env;
}

int KinodynamicAStar::search(Eigen::Vector3d start_pos,Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,
            Eigen::Vector3d end_pos,Eigen::Vector3d end_vel,Eigen::Vector3d end_acc)
{
    _start_pos = start_pos;
    _start_vel = start_vel;
    _start_acc = start_acc;

    PathNodePtr cur_node = _path_node_pool[0];
    _used_node_count += 1;

    cur_node->_parent = NULL;
    cur_node->_state.head(3) = _start_pos;
    cur_node->_state.tail(3) = _start_vel;
    cur_node->_g_score = 0.0;
    cur_node->_index = posToIndex(_start_pos);

    _end_pos = end_pos;
    _end_vel = end_vel;
    _end_acc = end_acc;

    Eigen::Matrix<double,6,1> end_state;
    Eigen::Vector3i end_index;

    end_state.head(3) = end_pos;
    end_state.tail(3) = end_vel;
    end_index = posToIndex(end_pos);

    double time_to_goal;
    cur_node->_f_score = cur_node->_g_score + estimateHeuristic(cur_node->_state,end_state,time_to_goal);

    _open_set.push(cur_node);
    _expanded_nodes.insert(cur_node->_index,cur_node);

    PathNodePtr neighbor = NULL;
    PathNodePtr terminate_node = NULL;
    bool init_search = false;
    double tolerance = _inv_resolution;

    while (!_open_set.empty()){
        cur_node = _open_set.top();
        _open_set.pop();

        /* -----------------checkt if quadrotor reachs horizon or end ----------------*/
        bool reach_horizon = ((cur_node->_state.head(3) - _start_pos).norm() >= _horizon);
        bool near_end = abs(cur_node->_index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->_index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->_index(2) - end_index(2)) <= tolerance;

        if(reach_horizon || near_end){
            terminate_node = cur_node;
            retrivePath(terminate_node);
            if(near_end){
                estimateHeuristic(cur_node->_state,end_state,time_to_goal);
                computeShotTraj(cur_node->_state,end_state,time_to_goal);
            }
        }

        if(reach_horizon){
            if(_is_shot_succ){
                return SearchResult::REACH_END;
            }else {
                return SearchResult::REACH_HORIZON;
            }
        }

        if(near_end){
            if(_is_shot_succ){
                return SearchResult::REACH_END;
            }else if(cur_node->_parent != NULL){
                return SearchResult::NEAR_END;
            }else {
                return SearchResult::NO_PATH;
            }
        }

        cur_node->_node_state = PathNode::NodeState::INCLOSESET;
        _iteration_num += 1;

        std::vector<Eigen::Vector3d> inputs;
        std::vector<double> durations;
        
        std::vector<PathNodePtr> tmp_expand_nodes;
        Eigen::Matrix<double,6,1> cur_state = cur_node->_state;
        Eigen::Matrix<double,6,1> pro_state;

        if(init_search){
            inputs.push_back(start_acc);
            for(double t = -_max_tau;t < _max_tau + 1e-3;t += _time_resolution){
                durations.push_back(t);
            }

            init_search = false;
        } else {
            for(double x_acc = -_max_acc;x_acc < _max_acc + 1e-3;x_acc += _acc_resolution){
                for(double y_acc = -_max_acc;y_acc < _max_acc + 1e-3;y_acc += _acc_resolution){
                    for(double z_acc = -_max_acc;z_acc < _max_acc + 1e-3;z_acc += _acc_resolution){
                        Eigen::Vector3d acc(x_acc,y_acc,z_acc);
                        inputs.push_back(acc);
                    }
                }
            }

            for(double t = -_max_tau;t < _max_tau + 1e-3;t += _time_resolution){
                durations.push_back(t);
            }
        }

        for(int i = 0;i < inputs.size();i++){
            Eigen::Vector3d u = inputs[i];
            for(int j = 0;j < durations.size();j++){
                double t = durations[j];
                transitState(cur_state,pro_state,u,t);

                Eigen::Vector3d pro_index = posToIndex(pro_state.head(3));
                PathNodePtr pro_node = _expanded_nodes.find(pro_index);
                if(pro_node != NULL && pro_node->_node_state == PathNode::NodeState::INCLOSESET){
                    continue;
                }
                /* -------------- check feasible -------------- */
                // Check maximal velocity
                Eigen::Vector3d pro_vel = pro_state.tail(3);
                if (fabs(pro_vel(0)) > _max_vel || fabs(pro_vel(1)) > _max_vel || fabs(pro_vel(2)) > _max_vel){
                    continue;
                }

                // Check not in the same voxel
                Eigen::Vector3i diff = pro_index - cur_node->_index;
                if(diff.norm() == 0){
                    continue;
                }
                // check safety
                Eigen::Vector3d tmp_pos;
                Eigen::Matrix<double,6,1> xt;
                bool is_occ = true;
                for(int k = 1;k <= _check_num;k++){
                    double dt = t * double(k) / double(_check_num);
                    transitState(cur_state,xt,u,dt);
                    tmp_pos = xt.head(3);
                }

                if(is_occ){
                    continue;
                }

                /* ------------- update ------------*/
                double optimal_time;
                double tmp_g_score,tmp_f_score;
                tmp_g_score = cur_node->_g_score + (u.squaredNorm() + _time_weight) * t;
                tmp_f_score = tmp_g_score + estimateHeuristic(pro_state,end_state,optimal_time);
                // Compare nodes expanded from the same parent
                bool prune = false;
                for(int k = 0;k < tmp_expand_nodes.size();k++){
                    PathNodePtr expanded_node = tmp_expand_nodes[i];
                    if((expanded_node->_index - pro_index).norm() == 0){
                        prune = true;
                        if(tmp_f_score < expanded_node->_f_score){
                            expanded_node->_input = u;
                            expanded_node->_duration = t;

                            expanded_node->_state = pro_state;

                            expanded_node->_g_score = tmp_g_score;
                            expanded_node->_f_score = tmp_f_score;
                        }
                        break;
                    }
                }

                if(!prune){
                    if(pro_node == NULL){
                        pro_node = _path_node_pool[_used_node_count];
                        _used_node_count += 1;
                        
                        if(_used_node_count >= _allocated_num){

                        }

                        pro_node->_input = u;
                        pro_node->_duration = t;
                        pro_node->_index = pro_index;
                        pro_node->_state = pro_state;
                        pro_node->_g_score = tmp_g_score;
                        pro_node->_f_score = tmp_f_score;
                        pro_node->_parent = cur_node;
                        pro_node->_node_state = PathNode::NodeState::INOPENSET;

                        _open_set.push(pro_node);
                        _expanded_nodes.insert(pro_index,pro_node);

                        tmp_expand_nodes.push_back(pro_node);
                    } else if(pro_node->_node_state == PathNode::NodeState::INOPENSET){
                        if(tmp_f_score < pro_node->_f_score){
                            pro_node->_input = u;
                            pro_node->_duration = t;
                            pro_node->_state = pro_state;
                            pro_node->_g_score = tmp_g_score;
                            pro_node->_f_score = tmp_f_score;
                            pro_node->_parent = cur_node;
                        }
                    }
                }

            }
        }
    }
    
    return SearchResult::NO_PATH;
}

std::vector<Eigen::Vector3d> KinodynamicAStar::getKinoTraj(double delta_t)
{   
    std::vector<Eigen::Vector3d> state_list;

    PathNodePtr cur_node = _path_nodes.back();
    Eigen::Matrix<double,6,1> x0,xt;
    Eigen::Vector3d u;
    double tau;

    while (cur_node->_parent != NULL)
    {
        x0 = cur_node->_parent->_state;
        u = cur_node->_input;
        tau = cur_node->_duration; 
    
        for(double t = tau;t >= -1e-5;t -= delta_t){
            transitState(x0,xt,u,t);
            state_list.push_back(xt.head(3));
        }

        cur_node = cur_node->_parent;
    }

    std::reverse(state_list.begin(),state_list.end());
    if (_is_shot_succ){
        Eigen::Vector3d coord;
        Eigen::VectorXd poly1d, time(4);

        for (double t = delta_t;t <= _t_shot;t += delta_t){
            for (int j = 0;j < 4;j++)
                time(j) = pow(t, j);

            for (int dim = 0; dim < 3;dim++){
                poly1d = _coefficient_matrix_shot.row(dim);
                coord(dim) = poly1d.dot(time);
            }
        
            state_list.push_back(coord);
        }
    }

    return state_list;
}

void KinodynamicAStar::getSamples(double& ts, std::vector<Eigen::Vector3d>& point_set,
                            std::vector<Eigen::Vector3d>& start_end_derivatives)
{
    double T_sum = 0.0;
    if(_is_shot_succ){
        T_sum += _t_shot;
    }

    PathNodePtr tmp_node = _path_nodes.back();
    while (tmp_node->_parent != NULL){
        T_sum += tmp_node->_duration;
        tmp_node = tmp_node->_parent;
    }

    Eigen::Vector3d end_vel,end_acc;

    double t;

    if(_is_shot_succ){
        t = _t_shot;
        end_vel = _end_vel;
        for(int dim = 0;dim < 3;dim++){
            Eigen::Vector4d coe = _coefficient_matrix_shot.row(dim);
            end_acc(dim) = 2 * coe(2) + 6 / coe(3) * _t_shot;
        }
    } else {
        t = _path_nodes.back()->_duration;
        end_vel = _path_nodes.back()->_state.tail(3);
        end_acc = _path_nodes.back()->_input;
    }

    int seg_num = floor(T_sum / ts);
    seg_num = std::max(8,seg_num);
    ts = T_sum / seg_num;
    bool sample_shot_traj = _is_shot_succ;
    tmp_node = _path_nodes.back();

    for(double ti = T_sum;ti > -1e-5;ti -= ts){
        if(sample_shot_traj){
            Eigen::Vector3d coord;
            Eigen::Vector4d poly1d,time;

            for(int i = 0;i < 4;i++){
                time(i) = std::pow(t,i);
            }

            for(int dim = 0;dim < 3;dim++){
                poly1d = _coefficient_matrix_shot.row(dim);
                coord(dim) = poly1d.dot(time); 
            }

            point_set.push_back(coord);
            t -= ts;

            if(t < 1e-5){
                sample_shot_traj = false;
                if(tmp_node->_parent !=NULL)
                    t += tmp_node->_duration;
            }
        }else{
            Eigen::Matrix<double, 6, 1> x0 = tmp_node->_parent->_state;
            Eigen::Matrix<double, 6, 1> xt;
            Eigen::Vector3d u = tmp_node->_input;

            transitState(x0,xt,u,t);

            point_set.push_back(xt.head(3));
            t -= ts;

            if(t < 1e-5 && tmp_node->_parent != NULL){
                tmp_node = tmp_node->_parent;
                t += tmp_node->_duration;
            }
        }
    }

    std::reverse(point_set.begin(),point_set.end());

    Eigen::Vector3d start_acc;
    if(_path_nodes.back()->_parent == NULL){
        start_acc = 2 * _coefficient_matrix_shot.col(2);
    } else {
        start_acc = _start_acc;
    }

    start_end_derivatives.push_back(_start_vel);
    start_end_derivatives.push_back(end_vel);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(end_acc);
}

double KinodynamicAStar::estimateHeuristic(Eigen::Matrix<double,6,1> pro_state,Eigen::Matrix<double,6,1> end_state,double &optimal_time)
{
    Eigen::Vector3d dp = end_state.head(3) - pro_state.head(3);
    Eigen::Vector3d vc = pro_state.tail(3);
    Eigen::Vector3d ve = end_state.tail(3);

    double c4,c3,c2,c1,c0;

    c4 = _time_weight;
    c3 = 0;
    c2 = -4 * (vc.dot(vc) + vc.dot(ve) + ve.dot(ve));
    c1 = 24 * dp.dot(vc + ve);
    c0 = -36 * dp.dot(dp);

    std::vector<double> t_solution = quartic(c4,c3,c2,c1,c0);

    double vmax = _max_vel * 0.5;
    double t_bar = dp.lpNorm<Eigen::Infinity>() / vmax;

    double cost = 100000000;
    double t_tmp = t_bar;
    t_solution.push_back(t_bar);

    for(auto t:t_solution){
        if(t < t_bar)
            continue;
        
        double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + _time_weight * t;
        if(c < cost){
            t_tmp = t;
            cost = c;
        }
    }

    optimal_time = t_tmp;
    return 1.0 * (1 + _tie_breaker) * cost;
}

void KinodynamicAStar::retrivePath(PathNodePtr end_node)
{
    _path_nodes.clear();

    PathNodePtr tmp_node = end_node;

    while (end_node != NULL){
        _path_nodes.push_back(tmp_node);
        tmp_node = tmp_node->_parent;
    }

    std::reverse(_path_nodes.begin(),_path_nodes.end());
}

bool KinodynamicAStar::computeShotTraj(Eigen::Matrix<double,6,1> pro_state,Eigen::Matrix<double,6,1> end_state,double optimal_time)
{
    Eigen::Vector3d pc = pro_state.head(3);
    Eigen::Vector3d pe = end_state.head(3);
    Eigen::Vector3d dp = pe - pc;

    Eigen::Vector3d vc = pro_state.tail(3);
    Eigen::Vector3d ve = end_state.tail(3);
    Eigen::Vector3d dv = ve - vc;

    double t_d = optimal_time;

    Eigen::Matrix<double,3,4> coefficient;

    Eigen::Vector3d a = 1.0/6 * 1/std::pow(t_d,3) * (-12 * (dp - vc * t_d) + 6 * t_d * dv);
    Eigen::Vector3d b = 1.0/2 * 1/std::pow(t_d,3) * (6 * t_d * (dp - vc * t_d) - 6 * t_d * t_d * dv);
    Eigen::Vector3d c = vc;
    Eigen::Vector3d d = pc;

    coefficient.col(3) = a;
    coefficient.col(2) = b;
    coefficient.col(1) = c;
    coefficient.col(0) = d;

    Eigen::Vector3d coord,vel,acc;
    Eigen::Vector4d poly1d,t,polyv,polya;
    Eigen::Vector3d index;

    Eigen::MatrixXd Tm(4, 4);
    Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

    double t_delta = t_d / 10;
    for(double time = t_delta;time <= t_d;time += t_delta){
        t = Eigen::Vector4d::Zero(4);
        for(int i = 0;i < 4;i++){
            t(i) = pow(time,i);
        }

        for(int dim = 0;dim < 3;dim++){
            poly1d = coefficient.row(dim);
            coord(dim) = poly1d.dot(t);
            vel(dim) = (Tm * poly1d).dot(t);
            acc(dim) = (Tm * Tm * poly1d).dot(t);

            if (fabs(vel(dim)) > _max_vel || fabs(acc(dim)) > _max_acc){

            }

            if (coord(0) < _origin(0) || coord(0) >= _map_size_3d(0) || 
                coord(1) < _origin(1) || coord(1) >= _map_size_3d(1) ||
                coord(2) < _origin(2) || coord(2) >= _map_size_3d(2)){
                return false;
            }

            // chack collsion
        }
    }

    _coefficient_matrix_shot = coefficient;
    _t_shot = t_d;
    _is_shot_succ = true;

    return true;
}
 

Eigen::Vector3i KinodynamicAStar::posToIndex(Eigen::Vector3d pos)
{
    return ((pos - _origin) * _inv_resolution).array().floor().cast<int>();
}

Eigen::Vector3d KinodynamicAStar::indexToPos(Eigen::Vector3i index)
{

}

void KinodynamicAStar::transitState(Eigen::Matrix<double,6,1> &start_state,Eigen::Matrix<double,6,1> &pro_state,Eigen::Vector3d u,double tau)
{
    for(int i = 0;i < 3;i++){
        _state_trans_matrix(i,i + 3) = tau;
    }

    Eigen::Matrix<double, 6, 1> integral;
    integral.head(3) = 1/2 * u * std::pow(tau,2);
    integral.tail(3) = u * tau;

    pro_state = _state_trans_matrix * start_state + integral;
}

std::vector<double> KinodynamicAStar::cubic(double a, double b, double c, double d)
{
  std::vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

std::vector<double> KinodynamicAStar::quartic(double a, double b, double c, double d, double e)
{
  std::vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}
