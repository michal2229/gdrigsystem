#ifndef GDRIGSYSTEM_H
#define GDRIGSYSTEM_H

#include "rigsystem_lib/src/rigsystem_common.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <vector>
#include <thread>
#include <future>


namespace godot {

class MeshInstance3D;
class Node3D;
struct Vector3;

inline rigsystem::vec3 v(const Vector3& v);
inline rigsystem::vec3 av(const Array& v);
inline Vector3 v(const rigsystem::vec3& v);

class GDRigSystem : public Node {
	GDCLASS(GDRigSystem, Node)

protected:
	static void _bind_methods();

public:
	GDRigSystem();
	virtual ~GDRigSystem();

	void _process(double delta) override;
	void _physics_process(double delta) override;

	void simulate(bool active);
	void visualize(bool active);
	void clear();
	void add_node(Dictionary node_dict, Dictionary node_defaults, Node* visual);
	void add_conn(Dictionary conn_dict, Dictionary conn_defaults, Node* visual);

	void align_visuals();
	void initialize_nodes(Node* parent);
	void initialize_connections(Node* parent);
private:
	rigsystem::RigSystemCommon m_rs;
	std::thread t;
	std::future<void> fut;
	
	int m_init_done = 0;
	bool state_sim_active = false;
	bool state_visuals_active = false;
	//bool thread_running = true;
	double last_delta = 0.01;

	std::vector<Node*> m_p_nodes;
	std::vector<Node*> m_p_conns;
	std::vector<std::pair<Node3D*, Node3D*>> m_p_conns_nodes;
	std::vector<MeshInstance3D*> m_p_conns_visual;

};

}

#endif

