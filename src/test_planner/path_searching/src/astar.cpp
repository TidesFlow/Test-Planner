#include "path_searching/astar.h"

int AStar::search(Eigen::Vector3d start_point,Eigen::Vector3d end_point)
{   
    NodePtr cur_node = _nodes_pool[0];
    _nodes_pool_count += 1;

    cur_node->_parent = NULL;
    cur_node->_position = start_point;
    cur_node->_index = posToIdx(cur_node->_position);
    cur_node->_g_score = 0.0;

    _open_set.push(cur_node);

    Eigen::Vector3i end_index = posToIdx(end_point);

    NodePtr neighbor_node = NULL;
    NodePtr terminate_node = NULL;

    while (!_open_set.empty()){
        cur_node = _open_set.top();
        _open_set.pop();

        bool reach_end = std::abs(cur_node->_index[0] - end_index(0)) <= 1 && std::abs(cur_node->_index[1] - end_index(1)) <= 1
                        && std::abs(cur_node->_index[2] - end_index(2)) <= 1;

        if(reach_end){
            terminate_node = cur_node;
            return PLAN_RESULT::REACH_END;
        }

        cur_node->_state = Node::IN_CLOSE_SET;

        Eigen::Vector3d cur_pos = cur_node->_position;
        Eigen::Vector3d pro_pos;

        Eigen::Vector3d delta_pos;

        for(double dx = -_resolution;dx <= _resolution;dx += _resolution){
            for(double dy = -_resolution;dy <= _resolution;dy += _resolution){
                for(double dz = -_resolution;dz <= _resolution;dz += _resolution){
                    delta_pos << dx,dy,dz;

                    if(delta_pos.norm() < 1e-3) 
                        continue;

                    pro_pos = cur_pos + delta_pos;

                    if(pro_pos(0) <= _origin(0) || pro_pos(0) >= _map_size_3d(0) || pro_pos(1) <= _origin(1) || pro_pos(1) >= _map_size_3d(1)
                        || pro_pos(2) <= _origin(2) || pro_pos(2) >= _map_size_3d(2))
                        continue;

                    Eigen::Vector3i pro_idx = posToIdx(pro_pos);

                    NodePtr pro_node;

                    double tmp_g_score,tmp_f_score;

                    tmp_g_score = delta_pos.squaredNorm() + cur_node->_g_score;
                    tmp_f_score = tmp_g_score + _lambda_heu * getHeuristic(pro_pos,end_point);

                    if(pro_node == NULL){ // unexpanded
                        pro_node = _nodes_pool[_nodes_pool_count];
                        _nodes_pool_count += 1;

                        if(_nodes_pool_count == _allocate_num){
                            std::cout << "run out of memory" << std::endl;
                        }

                        pro_node->_index = pro_idx;
                        pro_node->_position = pro_pos;
                        pro_node->_g_score = tmp_g_score;
                        pro_node->_f_score = tmp_f_score;
                        pro_node->_parent = cur_node;
                        pro_node->_state = Node::NODE_STATE::IN_OPEN_SET;

                        _open_set.push(pro_node);
                    } else if(pro_node->_state == Node::NODE_STATE::IN_OPEN_SET) {
                        if(tmp_g_score < pro_node->_g_score) {
                            pro_node->_f_score = tmp_f_score;
                            pro_node->_g_score = tmp_g_score;
                            pro_node->_parent = cur_node;
                        }
                    }
                }
            }
        }
    }

    return PLAN_RESULT::NO_PATH;
    
}

void AStar::retrievePath(NodePtr end_node)
{
    NodePtr cur_node = end_node;

    while (cur_node != NULL){
        _path_nodes.push_back(cur_node);
        cur_node = cur_node->_parent;
    }

    std::reverse(_path_nodes.begin(),_path_nodes.end());
}

std::vector<Eigen::Vector3d> AStar::getPath(void)
{
    std::vector<Eigen::Vector3d> path;
    for(int i = 0;i < _path_nodes.size();i++){
        path.push_back(_path_nodes[i]->_position);
    }

    return path;
}

void AStar::reset(void)
{

}