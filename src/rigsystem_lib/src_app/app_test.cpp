// our lib
#include <rigsystem_common.hpp>

#include <iostream>

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
    void update_wire(int id, const Vert& a, const Vert& b);
    std::vector<std::pair<Vert, Vert>> wires;
};

int gScreenWidth = 200; // Define the screens width and height
int gScreenHeight = 50;
float gAspect = (float)gScreenWidth / (float)gScreenHeight;
float dt = 1.0f / 240.0f;

void create_tower(RigSystemCommon& rs, int num_levels);

void project(std::pair<Vert, Vert>& w, Mat4 m);
void centerFlipY(std::pair<Vert, Vert>& w, Screen& s);
void drawWire(std::pair<Vert, Vert>& w, char c, Screen& s);
void drawMeshWire(RigWireMesh m, char c, Screen& s);

}


int main(int argc, char* argv[])
{
    RigSystemCommon rs;
    app_test::create_tower(rs, 8);

    Camera camera(0.0f, 0.0f, 0.0f, app_test::gAspect, 12, 0.1, 1000);
	LightD light;
	Screen screen(app_test::gScreenWidth, app_test::gScreenHeight, camera, light);

    app_test::RigWireMesh wm;
    
    vec3 t = {-4.5f,1.0f,-10.0f};

	while (1) { 
		screen.start();

        rs.integrate_system_radau2(app_test::dt);  // computing phys step

        for (int i = 0; i < rs.get_conns_num(); ++i)
        {
            vec3 pa = rs.m_s.nodes_pos[rs.m_s.conns_node_a[i]];
            vec3 pb = rs.m_s.nodes_pos[rs.m_s.conns_node_b[i]];

            // updating wire mesh
            wm.update_wire(i, Vert(pa.x + t.x, pa.y + t.y, pa.z + t.z), Vert(pb.x + t.x, pb.y + t.y, pb.z + t.z));
        }

        // drawing with ascii in the terminal
        // if not visible, decrease font size and maximize terminal window
        // should look like below
		app_test::drawMeshWire(wm, 'x', screen);  

		screen.print(); // Print the entire screen
		screen.clear(); // Clear the screen
	}
}


/*  example view which should be visible in the terminal (a frame from animation)
xxxxxxxxxxxxxx                                                                                                                                                                              
x xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx                                                                                                                                                          
x   xxx  xxxx        xxxxxxxxxxxxxxxxxxxxxxxxxx                                                                                                                                             
x     xxxx  xxx     xxxxx xxxxxx xxx   xxxxxxxxxxxxxxxxxxxxx                                                                                                                                
x       xxxxxxxxxxxx xxxxxxxx   xx xxx          xxxxxxxxxxxxxxxxxxxxxxxx                                                                                                                    
x        xxx xxxxxxxxxxxxxx     xxxx xx     xxxxx xxxxxxxxx   xxxxxxxxxxxxxxxxxxx                                                                                                           
x        x xxxxxxxxxxxx   x     x  xxxxxxxxxxxxxxxxx   x xxx             xxxxxxxxxxxxxx                                                                                                     
x       xxxxxxxxxxxxxxx  x     x     xxxxxxxxxx    x  x    xxx       xxxxxxxxxx xx xxxxxxxxxx                                                                                               
x    xxxxxx  xxxxxxxxxxxxx     x  xxxxxxxxxx      x   x     xxx  xxxxxxxxx  xx   xx       xxxxxxxxx                                                                                         
x xxxxx  x xxx     xxx  xxx   xxxxxxxxxxxxxxxx   x   x       xxxxxxxxx     x x     x              xxxxxxxx                                                                                  
xxxxxxxxxxxxxxxxxxxx xxxx xxxxx xxxx      xxxxx  x   x   xxxxxxxxx         xx       x         xxxxxxxxxx xxxxxxxxx                                                                          
            xxxxxxxxxxxxxxxxxxxxxxxx       xxxxxx   xxxxxxxxxx   xx       xx         x   xxxxxx    xx  xxx      xxxxxxxxxx                                                                  
                                xxxxxxxxxxxxxxxxx xxxxxxxxxx        xxx    xx         xxxxxx         x    xxx            xxxxxxxxxxx                                                          
                                            xxxxxxxxxxxxxxx           xx  x x    xxxxxx   x         x       xx        xxxxxxxxxxxxxxxxxxxxxx                                                  
                                                        xxxxxxxxxxxxxx  xxxxxxxxxxx         x       xx        xxx xxxxxxxx x   xx  x    xxxxxxxxxxxxx                                          
                                                                    xxxxxxxxxxx              x     xx        xxxxxxx       x  x xxx xx          xxxxxxxxxxxx                                   
                                                                            xxxxxxxx        xx   x     xxxxxx xxx       x  x    xxx x     xxxxxxxxxxxxxxxxxxxxxx                             
                                                                                    xxxxxxxx  x x xxxxxx       xxxx    x   x     xxx xxxxxxxxxxx   xxx  xxxxxxxxxxxxxx                       
                                                                                            xxxxxxx             xx x  x   x      xxxxxxxxx    x     xxx  xx    xxxxxxxxxxxxx                 
                                                                                                    xxxxxxxx       x x x  x  xxxxxxxxxx x     x     x x x x x        xxx    xxxx              
                                                                                                            xxxxxxxxxx xxxxxxxxxxx    x xxx    x     x  x xxx x xxxxxxxxxxxxxx                 
                                                                                                                xxxxxxxxx           xxxxx  x     x    xxxxxxxxxxxxxxxx                      
                                                                                                                        xxxxxxxx       x xxx     xxxxxxxxxxxxxxxx                            
                                                                                                                                xxxxxxxx  x xxxxxxxxxxxxxxxxxxxxx x                            
                                                                                                                                    xxxxxxxxx x xxxxxx    xx  x x                           
                                                                                                                                            xxxxxxxxx        xx  xxx                          
                                                                                                                                                    xxxxxxxxx  x    xx                         
                                                                                                                                                            xxxxxxxx xx                        
                                                                                                                                                                    xxxx
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
                .stiff = 6400.f,
                .damp = 32.f,
                .brk_thr = 0.8f,
                .broken = false
            };

            std::cout << "[CONN] id = " << c.id << ", node_a = " << c.i << ", node_b = " << c.j << "\n";
            rs.add_conn(c);
        }
    }

    for (int i = 0; i < rs.get_conns_num(); ++i)
    {
        vec3 pa = rs.m_s.nodes_pos[rs.m_s.conns_node_a[i]];
        vec3 pb = rs.m_s.nodes_pos[rs.m_s.conns_node_b[i]];  
        auto dist = pa.distance_to(pb);
        rs.m_s.conns_len[i] = dist;
    }

}


void RigWireMesh::update_wire(int id, const Vert& a, const Vert& b)
{
    if (id + 1> wires.size()) wires.resize(id+1);
    wires[id] = { a, b };
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
	for (auto &wire : m.wires) 
    {
		project(wire, s.camera.projMat);
		centerFlipY(wire, s);
		drawWire(wire, c, s);
	}
}

}