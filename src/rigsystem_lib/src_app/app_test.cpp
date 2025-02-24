// our lib
#include "deps/ascii-graphics/src/agm.hpp"
#include <rigsystem_common.hpp>

#include <iostream>
#include <chrono>
#include <cstdlib>

// ascii-graphics lib
#include <Screen.hpp>
#include <agm.hpp>
#include <Camera.hpp>
#include <Lights.hpp>
#include <Mesh.hpp>

using namespace rigsystem;

namespace app_test 
{
    
struct RigWireMesh
{
    void update_wire(int id, const Vert& a, const Vert& b, bool broken);
    std::vector<std::pair<Vert, Vert>> wires;
    std::vector<char> wbroken;
};

const int gScreenWidth = 200; // Define the screens width and height
const int gScreenHeight = 42;
const float gAspect = (float)gScreenWidth / (float)gScreenHeight;    

const Vert v_trans = {-3.8f,-1.25f,-10.0f};
Mat4 t_rot_y = rotY(-50);
Mat4 t_rot_z = rotZ(20);

const float stiffness_factor = 10000.f;
const float damping_factor = 100.f;
const float break_threshold = 0.2;

const int num_iter = 1000;
inline constexpr float dt = 1.0f / 60.f / static_cast<float>(num_iter);
const char perf_note[] = "[PERF] %d nodes, %d conns, %d iter time ms = %.3f";

void create_tower(RigSystemCommon& rs, int num_levels);

void project(std::pair<Vert, Vert>& w, Mat4 m);
void centerFlipY(std::pair<Vert, Vert>& w, Screen& s);
void drawWire(std::pair<Vert, Vert>& w, char c, Screen& s);
void drawMeshWire(RigWireMesh m, char c, Screen& s);

}


int main(int argc, char* argv[])
{
    srand (42);

    RigSystemCommon rs;
    app_test::create_tower(rs, 8);

    Camera camera(0.0f, 0.0f, 0.0f, app_test::gAspect, 12, 0.1, 1000);
	LightD light;
	Screen screen(app_test::gScreenWidth, app_test::gScreenHeight, camera, light);

    app_test::RigWireMesh wm;

	while (1) { 
		screen.start();
        
        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < app_test::num_iter; ++i)
            rs.integrate_system_radau2(app_test::dt);  // computing phys step
        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms = t2 - t1;

        for (int i = 0; i < rs.get_conns_num(); ++i)
        {
            vec3 pa = rs.m_s.nodes_pos[rs.m_s.conns_node_a[i]];
            vec3 pb = rs.m_s.nodes_pos[rs.m_s.conns_node_b[i]];

            Vert va = { pa.x, pa.y, pa.z };
            Vert vb = { pb.x, pb.y, pb.z };

            va = mult4(va, app_test::t_rot_z);
            vb = mult4(vb, app_test::t_rot_z);

            va = mult4(va, app_test::t_rot_y);
            vb = mult4(vb, app_test::t_rot_y);

            va = va + app_test::v_trans;
            vb = vb + app_test::v_trans;

            // updating wire mesh
            wm.update_wire(i, va, vb, rs.m_s.conns_broken[i]);
        }

        // drawing with ascii in the terminal
        // if not visible, decrease font size and maximize terminal window
        // should look like below
		app_test::drawMeshWire(wm, 'o', screen);  

        sprintf(&screen.buffer[app_test::gScreenHeight - 2][2], app_test::perf_note, rs.get_nodes_num(), rs.get_conns_num(), app_test::num_iter, ms.count());

		screen.print(); // Print the entire screen
		//std::cout << "[PERF] integrate_system_radau2 iter time ms = " << ms.count() << std::endl;
		screen.clear(); // Clear the screen
	}
}


/*  example view which should be visible in the terminal (a frame from animation)
                                                                                                                                 ooooo                                                                  
                                                                                                                       oooooooooo  ooo                           oooooooooooooo                         
                                                                                                             oooooooooo           o  o oooooooooooooooooooooooooooooooooo oooo                          
                                                                                                   oooooooooooooooooooooooooooooooooooo              oooooooooo        ooooo                            
                                                                                          ....oooooooooooooo......             oo    oo    oooooooooo               ooo  o                              
                                                                                 .........   oooooooo             ...................oooooo                      ooo   oo                               
                                                                        .........   oooooooooooooooooooooooooooooooooooooooooo   oooooooo                     ooo     o                                 
                                                                ooooooooooooooooooooooooooooooooooooooooooooooo            oo ooo o  oo  ooo               ooo      oo                                  
                                                         ooooooo  oooooooo  oooooooooooooooooooooooooooooo   ooooo        oooo   o    o     oo          ooo       oo                                    
                                                   ooooooooooooooooooooooooooo.      oooooooo  o oo  oooo oo      oooo  ooo    oo     oo      oo     ooo         o                                      
                                            ooooooooooooooooooooooooooooooooooooooooo  oo     ooo o o o .oo oo       ooooo    o       oo        ooooo          oo                                       
                                    oooooooo oo.ooooooooooooooooooooooooooooooooooooooo    ooo o   o  o .  oo oo  ooo o   oooo        oo       ooo oo         o                                         
                               oooooooooooooooo.ooooooooooooooooooooo   o ooo  oo o ooooooo    o  oo  o .    ooooo   o     oo ooooo   oo    ooo      ooo    oo                                          
                           ooooooo.ooooooooooooo.ooooooooooooooooo  o  ooo  oooo  o oooo  oooo  oo  oo  .   ooooo ooo     o        ooooo ooo            oooo                                            
                       oooo oooooooooooooooooooo.ooooo   ooo oooooooooo   o  ooo  ooo   oo    oooo   o   ooo     oo oo   o           oooooo              ooo                                            
                   oooooooooooooooooo.oooooooooo.ooo oooo   oo  o ooooo    oo o ooo       oo   oo ooooooo.       o o  ooo         ooo  oo  oooo        oo   ooo                                         
                ooo ooooooooooooooo  o. oooooo o.ooooo oo  oo  ooo  o oooooooooo ooo        ooo o  oooooooo    oo   ooo oo     ooo     oo      ooooo  o        oo                                       
             ooooooooooooooooooooooooooo.ooo ooo.oooooooooooooo  o   o  ooooooooo  oo       o o ooo o o  . oooo     oooo  ooooo         o           oooo         ooo                                    
          oooooooooooooooooooooooooooooooo  o ooo.o    oooooo    o   o oooo o  ooooo o     o ooooo  o  o  .  o ooooo    oooooo          o         oo    oooo        oo                                  
       ooo.oo.ooooooooooooooooooooo  ooooo.oooo  . o  ooo.oooooooo   oo  o oo o o   oooo  ooo    oo o   o .oo     ooooooo oo  oo        o        o          oooo      ooo                               
    ooooooooooooooooooooo  ooooo oooooo oooooo   . ooo o.o oooo  ooooo  o o  oo  o  o  ooooo     o oo   o o     oo ooo oooo oo  oo      o      oo               ooooo    oo                             
  ooooooo.  o  o.o    o  oooooooooo o oooo oo.ooo.oo  o. o  ooooooo  ooooo oo  oo o ooo o o oooooo o oo  o.    oooo        ooooo  oo    oo    o                      oooo  oo                           
  ooo ooooooo   oo..  o ooo  oooooo ooooo   oooo . oooooo   ooooo o  oo  oooooo  oooo  o   oo    oooo  oo o. ooo               oooo oo   o  oo                           ooooooo                        
  o  o  .   ooooooo ..oo o ooo o ooooooo   ooooo..  oo ooooo   oooo  oo oo    ooooooooo      o   o o ooooooooo                    oooooo ooo                                 ooooo                      
  o   oo     o  o  ooooooooo  ooo  ooo  oooooo ooo. o ooo   ooooooooo oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo                    
  o    .o    o  ooo o  oooooooooooooooooo o   oooo.oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo                                         
   o  .  oo  oooo oooooooooo  ooooooo.............oo..................ooooooooooooooooooooooooooooooooooooooooooooooooo                 ooo                                                             
   o .     ooo o ooo ooooooooooooooooooooooooooo                                                                      ooooooooooooooooooooo                                                             
   o .    ooooooooooooooooooooooooooo  oo                                                                                                                                                               
   o.  ooo  ooooooooooooooo     ..oo oo                                                                                                                                                                 
   o.ooooooooooooo  oooooooooooooo.oo                                                                                                                                                                   
   ooooooooooooooooo
*/

namespace app_test 
{

// create tower rig
void create_tower(RigSystemCommon& rs, int num_levels)
{
    std::vector<std::pair<int, int> > conns = { {1, 0}, {2,0}, {3,1} };

    for (int i = 0; i < num_levels; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            for (int k = 0; k < 2; ++k) 
            {
                Node n = {
                    .id = i*4 + j*2 + k,
                    .mass = 1,
                    .pos = vec3( 1.0f + i, j-0.5f ,  (j^k)-0.5f ),
                    .vel = vec3(0,0,0),
                    .acc = vec3(0,0,0), 
                    .frc = vec3(0,0,0), 
                    .pinned = i == 0 
                };
                rs.add_node(n);
                std::cout << "[NODE] id = " << n.id << ", pos = (" << n.pos.x << ", " << n.pos.y << ", " << n.pos.z << " )\n";
                
            }
        }
        if (i >= num_levels - 1)
        {
            continue;
        }

        std::vector<std::pair<int, int> > conns_i = {
            {0 +i*4, 1 +i*4},
            {0 +i*4, 3 +i*4},
            {0 +i*4, 4 +i*4},
            {0 +i*4, 5 +i*4},
            {0 +i*4, 6 +i*4},
            {0 +i*4, 7 +i*4},

            {1 +i*4, 2 +i*4},
            {1 +i*4, 4 +i*4},
            {1 +i*4, 5 +i*4},
            {1 +i*4, 6 +i*4},
            {1 +i*4, 7 +i*4},

            {2 +i*4, 3 +i*4},
            {2 +i*4, 4 +i*4},
            {2 +i*4, 5 +i*4},
            {2 +i*4, 6 +i*4},
            {2 +i*4, 7 +i*4},

            {3 +i*4, 4 +i*4},
            {3 +i*4, 5 +i*4},
            {3 +i*4, 6 +i*4},
            {3 +i*4, 7 +i*4} };

        for (int ci = 0; ci < conns_i.size(); ++ci)
        {
            const auto& p = conns_i[ci];
            Conn c = {
                .id = i * static_cast<int>( conns_i.size() ) + ci,
                .i = p.first,
                .j = p.second,
                .len = -1.0f,
                .stiff = stiffness_factor,
                .damp = damping_factor,
                .brk_thr = break_threshold,
                .broken = false
            };

            std::cout << "[CONN] id = " << c.id << ", node_a = " << c.i << ", node_b = " << c.j << "\n";
            rs.add_conn(c);
        }
    }

    for (int i = 0; i < rs.get_conns_num(); ++i)
    {
        const vec3& pa = rs.m_s.nodes_pos[rs.m_s.conns_node_a[i]];
        const vec3& pb = rs.m_s.nodes_pos[rs.m_s.conns_node_b[i]];  
        const auto dist = pa.distance_to(pb);
        rs.m_s.conns_len[i] = dist;
    }

    for (int i = 0; i < rs.get_nodes_num(); ++i)
    {
        rigsystem::vec3& p = rs.m_s.nodes_pos[i];

        p.x += (static_cast<float>(rand() % 10 + 1) / 10.0f - 0.5f) / 4.0f;
        p.y += (static_cast<float>(rand() % 10 + 1) / 10.0f - 0.5f) / 4.0f;
        p.z += (static_cast<float>(rand() % 10 + 1) / 10.0f - 0.5f) / 4.0f;
    }
}


void RigWireMesh::update_wire(int id, const Vert& a, const Vert& b, bool broken)
{
    if (id + 1> wires.size()) wires.resize(id+1);
    if (id + 1> wbroken.size()) wbroken.resize(id+1);
    wires[id] = { a, b };
    wbroken[id] = broken;
}


/// functions below were adapted from ascii-graphics lib to draw lines instead of triangles

void project(std::pair<Vert, Vert>& w, Mat4 m) 
{
	w.first  = mult4(w.first, m);
	w.second = mult4(w.second, m);
}

void centerFlipY(std::pair<Vert, Vert>& w, Screen& s)
{
	// Flip triangle verts
	w.first.y *= -1.f;
	w.second.y *= -1.f;

	// Scale by aspect ratio
	w.first.x *= s.camera.a*2.5;
	w.second.x *= s.camera.a*2.5;

	// Move the very center of screen
	w.first.x += (float)s.width/2;
	w.first.y += (float)s.height/2.f;
	w.second.x += (float)s.width/2;
	w.second.y += (float)s.height/2.f;
}

void drawWire(std::pair<Vert, Vert>& w, char c, Screen& s)
{
    s.drawLine(w.first, w.second, c);
}

void drawMeshWire(RigWireMesh m, char c, Screen& s) 
{
    s.renderMode = 0;  // no z-buffer
	for (int i = 0; i < m.wires.size(); ++i) 
    { 
        //if (m.wbroken[i]) continue;

        auto &wire = m.wires[i];

		project(wire, s.camera.projMat);
		centerFlipY(wire, s);
		drawWire(wire, m.wbroken[i] ? '.' : c, s);
	}
}

}
