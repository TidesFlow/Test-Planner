#ifndef __KINODYNAMIC_H__
#define __KINODYNAMIC_H__

#include <Eigen/Dense>

#include <vector>
#include <queue>
#include <unordered_map>

#include "plan_env/edt_environment.h"

class PathNode {
public:
    enum NodeState {UNEXPANDED,INOPENSET,INCLOSESET};
    typedef PathNode* Ptr;
    /* ------------------------- */
    Eigen::Vector3i _index;
    Eigen::Matrix<double,6,1> _state;
    double _f_score,_g_score;
    Eigen::Vector3d _input;
    double _duration;
    double _time;
    PathNode::Ptr _parent;
    NodeState _node_state;

    /* ------------------------- */

    PathNode(){
        _parent = NULL;
        _node_state = NodeState::UNEXPANDED;
    }
    ~PathNode(){};
private:
};

typedef PathNode* PathNodePtr;

class PathNodeComparator {
    bool operator()(PathNodePtr node1,PathNodePtr node2){
        return node1->_f_score > node2->_f_score;
    }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class PathNodeHashTable {
private:
    std::unordered_map<Eigen::Vector3d,PathNodePtr,matrix_hash<Eigen::Vector3d>> _data_3d;
    std::unordered_map<Eigen::Vector3d,PathNodePtr,matrix_hash<Eigen::Vector3d>> _data_4d;
public:
    PathNodeHashTable(){}
    ~PathNodeHashTable(){}

    void insert(Eigen::Vector3i index,PathNodePtr path_node) {
        _data_3d.insert(std::make_pair(index,path_node));
    }

    void insert(Eigen::Vector4i index,double time_index,PathNodePtr path_node) {
        _data_4d.insert(std::make_pair(Eigen::Vector4i(index(0), index(1), index(2), time_index),path_node));
    }

    PathNodePtr find(Eigen::Vector3i index) {
        auto iter = _data_3d.find(index);
        return iter == _data_3d.end()?NULL:iter->second;
    }

    PathNodePtr find(Eigen::Vector3i index,int time_index) {
        auto iter = _data_4d.find(Eigen::Vector4i(index(0), index(1), index(2), time_index));
        return iter == _data_4d.end()?NULL:iter->second;
    }

    void clear() {
        _data_3d.clear();
        _data_4d.clear();
    }
};


class KinodynamicAStar {
public:
    enum SearchResult {REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4};

    KinodynamicAStar(){}
    ~KinodynamicAStar(){}

    void init();
    void reset();
    int search(Eigen::Vector3d start_point,Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,
                Eigen::Vector3d end_point,Eigen::Vector3d end_vel,Eigen::Vector3d end_acc);
    std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
    void getSamples(double& ts, std::vector<Eigen::Vector3d>& point_set,
                                std::vector<Eigen::Vector3d>& start_end_derivatives);
    /* ---------------- some tool function ---------------- */
    double estimateHeuristic(Eigen::Matrix<double,6,1> pro_state,Eigen::Matrix<double,6,1> end_state,double &optimal_time);
    bool computeShotTraj(Eigen::Matrix<double,6,1> pro_state,Eigen::Matrix<double,6,1> end_state,double optimal_time);
    void retrivePath(PathNodePtr end_node);
    void setEnvironment(EDTEnvironment::Ptr &env);


    Eigen::Vector3i posToIndex(Eigen::Vector3d pos);
    Eigen::Vector3d indexToPos(Eigen::Vector3i index);
    void transitState(Eigen::Matrix<double,6,1> &start_state,Eigen::Matrix<double,6,1> &pro_state,Eigen::Vector3d u,double tau);

    /* ----------------- some math function --------------- */
    std::vector<double> cubic(double a, double b, double c, double d);
    std::vector<double> quartic(double a, double b, double c, double d, double e);
private:
    /* --------- main data structure ------------- */
    std::vector<PathNodePtr> _path_node_pool;
    int _allocated_num;
    int _used_node_count;

    std::priority_queue<PathNodePtr,std::vector<PathNodePtr>,PathNodeComparator> _open_set;
    PathNodeHashTable _expanded_nodes;
    std::vector<PathNodePtr> _path_nodes;
    /* --------- map data ------------------------ */
    EDTEnvironment::Ptr _edt_environment;
    Eigen::Vector3d _origin,_map_size_3d;
    double _resolution,_inv_resolution;

    /* --------- init state ---------------------- */
    Eigen::Vector3d _start_pos,_start_vel,_start_acc;
    Eigen::Vector3d _end_pos,_end_vel,_end_acc;
    Eigen::Matrix<double,6,6> _state_trans_matrix;
    
    /* ----------- computation -------------- */
    int _iteration_num;

    /* --------- shot data ----------------------- */
    Eigen::MatrixXd _coefficient_matrix_shot;
    double _t_shot;
    bool _is_shot_succ;
    /* ---------- parameters --------------------- */
    int _check_num;
    
    double _max_acc,_acc_resolution;
    double _max_tau,_time_resolution;
    double _max_vel;
    double _time_weight;
    double _tie_breaker;
    /* ----------- RHD --------------------------- */
    double _horizon;
    
};
#endif