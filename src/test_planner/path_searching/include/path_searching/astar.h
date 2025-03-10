#ifndef _ASTAR_H__
#define _ASTAR_H__

#include <queue>
#include <vector>
#include <iostream>

#include <Eigen/Dense>

class Node {
public:
    enum NODE_STATE {NOT_EXPANDED,IN_OPEN_SET,IN_CLOSE_SET};

    Node(){}
    ~Node(){}

    Eigen::Vector3i _index;
    Eigen::Vector3d _position;
    double _g_score,_f_score;
    NODE_STATE _state;
    Node *_parent;
};

typedef Node* NodePtr;

class AStar {
public:
    enum PLAN_RESULT {REACH_END,NO_PATH};

    int search(Eigen::Vector3d start_point,Eigen::Vector3d end_point);
    void retrievePath(NodePtr end_node);
    std::vector<Eigen::Vector3d> getPath(void);
    Eigen::Vector3i posToIdx(Eigen::Vector3d position);
    Eigen::Vector3d idxToPos(Eigen::Vector3i index);
    double getHeuristic(Eigen::Vector3d cur_pos,Eigen::Vector3d goal_pos);
    void reset(void);
private:
    double _resolution;

    double _lambda_heu;

    std::vector<NodePtr> _nodes_pool;
    int _allocate_num;
    int _nodes_pool_count;

    Eigen::Vector3d _origin,_map_size_3d;

    std::priority_queue<NodePtr,std::vector<NodePtr>> _open_set;
    std::vector<NodePtr> _path_nodes;
};

#endif