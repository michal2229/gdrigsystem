#ifndef GDRIGSYSTEM_H
#define GDRIGSYSTEM_H

#include "rigsystem_lib/src/rigsystem_common.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/node.hpp>

#include <vector>


namespace godot {

class MeshInstance3D;
class Node3D;
struct Vector3;

inline rigsystem::vec3 v(const Vector3& v);
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

	void align_visuals();

	void initialize_nodes();
	void initialize_connections();
private:
	rigsystem::RigSystemCommon m_rs;
	
	int m_init_done = 0;

	std::vector<Node*> m_p_nodes;
	std::vector<Node*> m_p_conns;
	std::vector<std::pair<Node3D*, Node3D*>> m_p_conns_nodes;
	std::vector<MeshInstance3D*> m_p_conns_visual;

};

}

#endif

