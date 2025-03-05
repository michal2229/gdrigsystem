
#include <cstddef>
#include <rigsystem_common.hpp>

#include <rigsystem_vec3.hpp>

#include <assert.h>
#include <chrono>
#include <cstdlib>
#include <immintrin.h> // portable
#include <iostream>
#include <signal.h>
#include <vector>
// #include <x86intrin.h>
// #include <intrin.h> // msvc

// using namespace rigsystem;
// using namespace adr;

namespace app_perf_test {

bool compare_floats(float f1, float f2)
{
    return (fabs(f1 - f2) < std::numeric_limits<float>::epsilon());
}

void siginthandler(int param);

union vec3_x8_avx256_u_t {
    float flt38[3][8]; // [ [xxx...][yyy...][zzz...] ]
    __m256 regs[3]; // [ x8, y8, z8 ]

    void set_item(size_t n, const rigsystem::vec3& v)
    {
        flt38[0][n] = v.x;
        flt38[1][n] = v.y;
        flt38[2][n] = v.z;
    }

    void get_item(size_t n, rigsystem::vec3& v) const
    {
        // v = rigsystem::vec3(flt38[0][n], flt38[1][n], flt38[2][n]);
        v.x = flt38[0][n];
        v.y = flt38[1][n];
        v.z = flt38[2][n];
    }

    void add(const vec3_x8_avx256_u_t& other)
    {
        for (int i = 0; i < 3; ++i) {
            regs[i] = _mm256_add_ps(regs[i], other.regs[i]);
        }
    }

    void add(float val)
    {
        __m256 tmp = _mm256_set1_ps(val);
        for (int i = 0; i < 3; ++i) {
            regs[i] = _mm256_add_ps(regs[i], tmp);
        }
    }

    void sub(const vec3_x8_avx256_u_t& other)
    {
        for (int i = 0; i < 3; ++i) {
            regs[i] = _mm256_sub_ps(regs[i], other.regs[i]);
        }
    }

    void sub(float val)
    {
        __m256 tmp = _mm256_set1_ps(val);
        for (int i = 0; i < 3; ++i) {
            regs[i] = _mm256_sub_ps(regs[i], tmp);
        }
    }

    void mul(const vec3_x8_avx256_u_t& other)
    {
        for (int i = 0; i < 3; ++i) {
            regs[i] = _mm256_mul_ps(regs[i], other.regs[i]);
        }
    }

    void mul(float val)
    {
        __m256 tmp = _mm256_set1_ps(val);
        for (int i = 0; i < 3; ++i) {
            regs[i] = _mm256_mul_ps(regs[i], tmp);
        }
    }

    void div(const vec3_x8_avx256_u_t& other)
    {
        for (int i = 0; i < 3; ++i) {
            regs[i] = _mm256_div_ps(regs[i], other.regs[i]);
        }
    }

    void div(float val)
    {
        __m256 tmp = _mm256_set1_ps(val);
        for (int i = 0; i < 3; ++i) {
            regs[i] = _mm256_div_ps(regs[i], tmp);
        }
    }
};

struct vec3_x8_avx256_a_t {
    __m256 xyz1xyz2xy;
    __m256 z3xyz4xyz5x;
    __m256 yz6xyz7xyz8;

    void set_item(size_t n, const rigsystem::vec3& v)
    {
        static const __m256 idx = _mm256_setr_ps(0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f);
        __m256* regs = reinterpret_cast<__m256*>(this); // regs[0]=xyz1xyz2xy, regs[1]=z3xyz4xyz5x, regs[2]=yz6xyz7xyz8
        const float* vec_data = reinterpret_cast<const float*>(&v);

        for (size_t i = 0; i < 3; ++i) {
            size_t offset = n * 3 + i;

            size_t reg_idx = offset / 8;
            size_t lane_idx = offset % 8;
            __m256 curr = regs[reg_idx];
            __m256 mask = _mm256_cmp_ps(idx, _mm256_set1_ps((float)lane_idx), _CMP_EQ_OQ);
            curr = _mm256_blendv_ps(curr, _mm256_set1_ps(vec_data[i]), mask);
            regs[reg_idx] = curr;
        }
    }

    void get_item(size_t i, rigsystem::vec3& v_out) const
    {
        const __m256* regs = reinterpret_cast<const __m256*>(this);
        float data[24];
        _mm256_store_ps(&data[0], regs[0]);
        _mm256_store_ps(&data[8], regs[1]);
        _mm256_store_ps(&data[16], regs[2]);
        size_t offset = i * 3;
        v_out.x = data[offset];
        v_out.y = data[offset + 1];
        v_out.z = data[offset + 2];
    }

    void add(const vec3_x8_avx256_a_t& other)
    {
        xyz1xyz2xy = _mm256_add_ps(xyz1xyz2xy, other.xyz1xyz2xy);
        z3xyz4xyz5x = _mm256_add_ps(z3xyz4xyz5x, other.z3xyz4xyz5x);
        yz6xyz7xyz8 = _mm256_add_ps(yz6xyz7xyz8, other.yz6xyz7xyz8);
    }

    void sub(const vec3_x8_avx256_a_t& other)
    {
        xyz1xyz2xy = _mm256_sub_ps(xyz1xyz2xy, other.xyz1xyz2xy);
        z3xyz4xyz5x = _mm256_sub_ps(z3xyz4xyz5x, other.z3xyz4xyz5x);
        yz6xyz7xyz8 = _mm256_sub_ps(yz6xyz7xyz8, other.yz6xyz7xyz8);
    }

    void mul(const vec3_x8_avx256_a_t& other)
    {
        xyz1xyz2xy = _mm256_mul_ps(xyz1xyz2xy, other.xyz1xyz2xy);
        z3xyz4xyz5x = _mm256_mul_ps(z3xyz4xyz5x, other.z3xyz4xyz5x);
        yz6xyz7xyz8 = _mm256_mul_ps(yz6xyz7xyz8, other.yz6xyz7xyz8);
    }

    void div(const vec3_x8_avx256_a_t& other)
    {
        xyz1xyz2xy = _mm256_div_ps(xyz1xyz2xy, other.xyz1xyz2xy);
        z3xyz4xyz5x = _mm256_div_ps(z3xyz4xyz5x, other.z3xyz4xyz5x);
        yz6xyz7xyz8 = _mm256_div_ps(yz6xyz7xyz8, other.yz6xyz7xyz8);
    }

    void add(float val)
    {
        vec3_x8_avx256_a_t tmp {
            _mm256_set1_ps(val),
            _mm256_set1_ps(val),
            _mm256_set1_ps(val)
        };
        add(tmp);
    }

    void sub(float val)
    {
        vec3_x8_avx256_a_t tmp {
            _mm256_set1_ps(val),
            _mm256_set1_ps(val),
            _mm256_set1_ps(val)
        };
        sub(tmp);
    }

    void mul(float val)
    {
        vec3_x8_avx256_a_t tmp {
            _mm256_set1_ps(val),
            _mm256_set1_ps(val),
            _mm256_set1_ps(val)
        };
        mul(tmp);
    }

    void div(float val)
    {
        vec3_x8_avx256_a_t tmp {
            _mm256_set1_ps(val),
            _mm256_set1_ps(val),
            _mm256_set1_ps(val)
        };
        div(tmp);
    }
};

struct vec3x2_t {
    float v[8] = { 1.111111f };

    vec3x2_t() = default;

    vec3x2_t(const rigsystem::vec3& v1, const rigsystem::vec3& v2)
    {
        v[0] = v1.x;
        v[1] = v1.y;
        v[2] = v1.z;

        v[4] = v2.x;
        v[5] = v2.y;
        v[6] = v2.z;
    }

    void get(rigsystem::vec3& v1_out, rigsystem::vec3& v2_out)
    {
        v1_out.x = v[0];
        v1_out.y = v[1];
        v1_out.z = v[2];

        v2_out.x = v[4];
        v2_out.y = v[5];
        v2_out.z = v[6];
    }

    void mul(const vec3x2_t& b)
    {
        for (size_t i = 0; i < 8; ++i)
            v[i] *= b.v[i];
    }

    void mul(float b)
    {
        for (size_t i = 0; i < 8; ++i)
            v[i] *= b;
    }

    void div(const vec3x2_t& b)
    {
        for (size_t i = 0; i < 8; ++i)
            v[i] *= b.v[i];
    }

    void div(float b)
    {
        float invb = 1.f / b;
        for (size_t i = 0; i < 8; ++i)
            v[i] *= invb;
    }

    void add(const vec3x2_t& b)
    {
        for (size_t i = 0; i < 8; ++i)
            v[i] += b.v[i];
    }

    void add(float b)
    {
        for (size_t i = 0; i < 8; ++i)
            v[i] += b;
    }

    void sub(const vec3x2_t& b)
    {
        for (size_t i = 0; i < 8; ++i)
            v[i] -= b.v[i];
    }

    void sub(float b)
    {
        for (size_t i = 0; i < 8; ++i)
            v[i] -= b;
    }
};

struct vec3x5_t {
    float v[16] = { 1.111111f };

    float* get_item(size_t i)
    {
        return &v[i * 3];
    }

    void set_item(size_t i, rigsystem::vec3 e)
    {
        get_item(i)[0] = e.x;
        get_item(i)[1] = e.y;
        get_item(i)[2] = e.z;
    }

    void mul(const vec3x5_t& b)
    {
        for (size_t i = 0; i < 16; ++i)
            v[i] *= b.v[i];
    }

    void mul(float b)
    {
        for (size_t i = 0; i < 16; ++i)
            v[i] *= b;
    }

    void div(const vec3x5_t& b)
    {
        for (size_t i = 0; i < 16; ++i)
            v[i] *= b.v[i];
    }

    void div(float b)
    {
        float invb = 1.f / b;
        for (size_t i = 0; i < 16; ++i)
            v[i] *= invb;
    }

    void add(const vec3x5_t& b)
    {
        for (size_t i = 0; i < 16; ++i)
            v[i] += b.v[i];
    }

    void add(float b)
    {
        for (size_t i = 0; i < 16; ++i)
            v[i] += b;
    }

    void sub(const vec3x5_t& b)
    {
        for (size_t i = 0; i < 16; ++i)
            v[i] -= b.v[i];
    }

    void sub(float b)
    {
        for (size_t i = 0; i < 16; ++i)
            v[i] -= b;
    }
};

struct TestNodeItemx2_t {
    vec3x2_t pos;
    vec3x2_t vel;
    vec3x2_t acc;
    vec3x2_t frc;
    float mass[2];
};

struct TestNodeItem_t {
    rigsystem::vec3 pos; // 4 bytes * 3 = 12 bytes
    rigsystem::vec3 vel; // 4 bytes * 3 = 12 bytes
    rigsystem::vec3 acc; // 4 bytes * 3 = 12 bytes
    rigsystem::vec3 frc; // 4 bytes * 3 = 12 bytes
    float mass; // 4 bytes
};

struct TestNodeItemx8_t {
    vec3_x8_avx256_u_t pos; // 8 vec3's (24 floats)
    vec3_x8_avx256_u_t vel;
    vec3_x8_avx256_u_t acc;
    vec3_x8_avx256_u_t frc;
    vec3_x8_avx256_u_t mass;

    void set_item(size_t i, const TestNodeItem_t& e)
    {
        pos.set_item(i, e.pos);
        vel.set_item(i, e.vel);
        acc.set_item(i, e.acc);
        frc.set_item(i, e.frc);
        rigsystem::vec3 massVal = { e.mass, e.mass, e.mass };
        mass.set_item(i, massVal);
    }

    void get_item(size_t i, TestNodeItem_t& e) const
    {
        pos.get_item(i, e.pos);
        vel.get_item(i, e.vel);
        acc.get_item(i, e.acc);
        frc.get_item(i, e.frc);
        rigsystem::vec3 massVal;
        mass.get_item(i, massVal);
        e.mass = massVal.x;
    }
};

struct TestNodeItemx5_t {

    vec3x5_t pos;
    vec3x5_t vel;
    vec3x5_t acc;
    vec3x5_t frc;
    float mass[5];

    void set_item(size_t i, const TestNodeItem_t& e)
    {
        pos.set_item(i, e.pos);
        vel.set_item(i, e.vel);
        acc.set_item(i, e.acc);
        frc.set_item(i, e.frc);
        mass[i] = e.mass;
    }
};

struct TestNodeVecs_t {
    std::vector<rigsystem::vec3> pos;
    std::vector<rigsystem::vec3> vel;
    std::vector<rigsystem::vec3> acc;
    std::vector<rigsystem::vec3> frc;
    std::vector<float> mass;

    void clear()
    {
        pos.clear();
        vel.clear();
        acc.clear();
        frc.clear();
        mass.clear();
    }

    void resize(size_t n)
    {
        pos.resize(n);
        vel.resize(n);
        acc.resize(n);
        frc.resize(n);
        mass.resize(n);
    }

    void set_item(size_t i, const TestNodeItem_t& e)
    {
        pos[i] = e.pos;
        vel[i] = e.vel;
        acc[i] = e.acc;
        frc[i] = e.frc;
        mass[i] = e.mass;
    }
};

struct TestNodex2Vecs_t {
    std::vector<vec3x2_t> pos;
    std::vector<vec3x2_t> vel;
    std::vector<vec3x2_t> acc;
    std::vector<vec3x2_t> frc;
    std::vector<float> mass;

    void clear()
    {
        pos.clear();
        vel.clear();
        acc.clear();
        frc.clear();
        mass.clear();
    }

    void resize(size_t n)
    {
        pos.resize(n);
        vel.resize(n);
        acc.resize(n);
        frc.resize(n);
        mass.resize(n * 2);
    }

    void set_item(size_t i, const TestNodeItem_t& e)
    {
        pos[i / 2].v[(i % 2) * 4 + 0] = e.pos.x;
        pos[i / 2].v[(i % 2) * 4 + 1] = e.pos.y;
        pos[i / 2].v[(i % 2) * 4 + 2] = e.pos.z;

        vel[i / 2].v[(i % 2) * 4 + 0] = e.vel.x;
        vel[i / 2].v[(i % 2) * 4 + 1] = e.vel.y;
        vel[i / 2].v[(i % 2) * 4 + 2] = e.vel.z;

        acc[i / 2].v[(i % 2) * 4 + 0] = e.acc.x;
        acc[i / 2].v[(i % 2) * 4 + 1] = e.acc.y;
        acc[i / 2].v[(i % 2) * 4 + 2] = e.acc.z;

        frc[i / 2].v[(i % 2) * 4 + 0] = e.frc.x;
        frc[i / 2].v[(i % 2) * 4 + 1] = e.frc.y;
        frc[i / 2].v[(i % 2) * 4 + 2] = e.frc.z;

        mass[i] = e.mass;
    }
};

struct TestNodex8Vecs_t {
    std::vector<vec3_x8_avx256_u_t> pos;
    std::vector<vec3_x8_avx256_u_t> vel;
    std::vector<vec3_x8_avx256_u_t> acc;
    std::vector<vec3_x8_avx256_u_t> frc;
    std::vector<vec3_x8_avx256_u_t> mass;

    void clear()
    {
        pos.clear();
        vel.clear();
        acc.clear();
        frc.clear();
        mass.clear();
    }

    void resize(size_t n)
    {
        pos.resize(n);
        vel.resize(n);
        acc.resize(n);
        frc.resize(n);
        mass.resize(n);
    }

    void set_item(size_t i, const TestNodeItem_t& e)
    {
        size_t block = i / 8;
        size_t lane = i % 8;
        pos[block].set_item(lane, e.pos);
        vel[block].set_item(lane, e.vel);
        acc[block].set_item(lane, e.acc);
        frc[block].set_item(lane, e.frc);
        // For mass, replicate the scalar across all components.
        rigsystem::vec3 massVal = { e.mass, e.mass, e.mass };
        mass[block].set_item(lane, massVal);
    }

    void get_item(size_t i, TestNodeItem_t& e) const
    {
        size_t block = i / 8;
        size_t lane = i % 8;
        pos[block].get_item(lane, e.pos);
        vel[block].get_item(lane, e.vel);
        acc[block].get_item(lane, e.acc);
        frc[block].get_item(lane, e.frc);
        rigsystem::vec3 massVal;
        mass[block].get_item(lane, massVal);
        e.mass = massVal.x; // since mass was replicated, any component is valid.
    }
};

const float dt = 0.001f;
const size_t num_items = 1e4;
const size_t num_iters = 1e5;
std::vector<TestNodeItem_t> vec_of_structs;
std::vector<TestNodeItemx2_t> vec_of_structs_x2;
std::vector<TestNodeItemx5_t> vec_of_structs_x5;
std::vector<TestNodeItemx8_t> vec_of_structs_x8;
TestNodeVecs_t struct_of_vecs;
TestNodex2Vecs_t struct_of_vecs_x2;
TestNodex8Vecs_t struct_of_vecs_x8;

rigsystem::vec3 random_vec();
rigsystem::vec3 zero_vec();
rigsystem::vec3 one_vec();
void data_init(size_t);

void test_vec3();
void test_vec_of_structs(size_t);
void test_struct_of_vecs(size_t);
void test_vos_avx256_2(size_t n);
void test_vos_avx256_5(size_t n);
void test_sov_avx256_2(size_t n);
void test_vos_avx256_8(size_t n);
void test_sov_avx256_8(size_t n);
void verify_data();

}

using namespace app_perf_test;

int main(int argc, char** argv)
{
    srand(42);

    __m256 avx256_zero = _mm256_set1_ps(0.0f);

    union {
        float f[8];
        __m256 m;
    } u;

    u.m = avx256_zero;
    u.f[3] = 234.0f;

    float f2[8];
    _mm256_store_ps(f2, u.m);

    for (size_t i = 0; i < 8; ++i) {
        std::cout << f2[i] << " ";
    }
    std::cout << "\n";

    // notes:
    //   * it seems that for low number of elements (<1e3), vec of structs wins slightly
    //     but for high number of elements (>1e5), struct of vecs wins - in both cases there's over 3ops/cycle
    //   * for num elements efficiency drops quite hard in both cases (cache misses)
    //     ops/cycle is below 1 in this case, ~15% L1 misses
    // 768 KB

    std::cout << "[info] argc = " << argc << "\n";
    for (size_t i = 0; i < argc; ++i)
        std::cout << "[info] argv[" << i << "] = " << argv[i] << "\n";

    std::cout << "[info] sizeof(TestNodeItem) = " << sizeof(TestNodeItem_t) << "\n";

    //signal(SIGINT, siginthandler);
    data_init(num_items);
    for (size_t i = 0; i < 3; ++i) {
        test_vec_of_structs(num_iters);
        test_struct_of_vecs(num_iters);
        test_vos_avx256_2(num_iters);
        test_sov_avx256_2(num_iters);
        test_vos_avx256_5(num_iters);
        test_vos_avx256_8(num_iters);
        test_sov_avx256_8(num_iters);
        verify_data();
    }

    return 0;
}

namespace app_perf_test {

// void siginthandler(int param)
// {
//     state_handle_sigint = true;
//     printf("User pressed Ctrl+C\n");
// }

rigsystem::vec3 random_vec()
{
    return rigsystem::vec3(
        (static_cast<float>(rand() % 1000 + 1) / 1000.0f - 0.5f) / 100.0f + 1.0f,
        (static_cast<float>(rand() % 1000 + 1) / 1000.0f - 0.5f) / 100.0f + 1.0f,
        (static_cast<float>(rand() % 1000 + 1) / 1000.0f - 0.5f) / 100.0f + 1.0f);
}

rigsystem::vec3 zero_vec()
{
    return rigsystem::vec3(0.0f, 0.0f, 0.0f);
}

rigsystem::vec3 one_vec()
{
    return rigsystem::vec3(1.0f, 1.0f, 1.0f);
}

void data_init(size_t n)
{
    auto t1 = std::chrono::high_resolution_clock::now(); // perf
    vec_of_structs.clear();
    struct_of_vecs.clear();
    vec_of_structs_x2.clear();
    struct_of_vecs_x2.clear();
    struct_of_vecs_x8.clear();
    vec_of_structs.resize(n);
    struct_of_vecs.resize(n);
    vec_of_structs_x2.resize(n / 2 + 1);
    struct_of_vecs_x2.resize(n / 2 + 1);
    struct_of_vecs_x8.resize(n / 8 + 1);
    vec_of_structs_x5.resize(n / 5 + 1);
    vec_of_structs_x8.resize(n / 8 + 1);

    for (size_t i = 0; i < n; ++i) {
        vec_of_structs[i] = TestNodeItem_t {
            .pos = zero_vec(),
            .vel = zero_vec(),
            .acc = zero_vec(),
            .frc = one_vec(),
            .mass = 1.0f
        };

        struct_of_vecs.set_item(i, vec_of_structs[i]);
        struct_of_vecs_x2.set_item(i, vec_of_structs[i]);
        struct_of_vecs_x8.set_item(i, vec_of_structs[i]);

        vec_of_structs_x5[i / 5].set_item(i % 5, vec_of_structs[i]);
        vec_of_structs_x8[i / 8].set_item(i % 8, vec_of_structs[i]);
    }

    for (size_t i = 0; i < n / 2; ++i) {
        vec_of_structs_x2[i] = TestNodeItemx2_t {
            .pos = vec3x2_t(vec_of_structs[i * 2].pos, vec_of_structs[i * 2 + 1].pos),
            .vel = vec3x2_t(vec_of_structs[i * 2].vel, vec_of_structs[i * 2 + 1].vel),
            .acc = vec3x2_t(vec_of_structs[i * 2].acc, vec_of_structs[i * 2 + 1].acc),
            .frc = vec3x2_t(vec_of_structs[i * 2].frc, vec_of_structs[i * 2 + 1].frc),
            .mass { vec_of_structs[i * 2].mass, vec_of_structs[i * 2 + 1].mass }
        };
    }

    auto t2 = std::chrono::high_resolution_clock::now(); // perf
    std::chrono::duration<double, std::milli> ms = t2 - t1;
    std::cout << "[init] data init: size = " << num_items << ", time = " << ms.count() << "ms\n";
}

void test_vec_of_structs(size_t n)
{
    auto t1 = std::chrono::high_resolution_clock::now(); // perf
    for (size_t iter = 0; iter < n; ++iter)
        for (size_t i = 0; i < vec_of_structs.size(); ++i) {
            TestNodeItem_t& n = vec_of_structs[i];

            n.acc = n.frc / n.mass;
            n.vel += n.acc * dt;
            n.pos += n.vel * dt;
        }

    auto t2 = std::chrono::high_resolution_clock::now(); // perf
    std::chrono::duration<double, std::nano> ms = t2 - t1;
    std::cout << "[perf] vec of structs, len = " << vec_of_structs.size() << ", iter time = " << ms.count() / n / vec_of_structs.size() << "ns\n";
}

void test_struct_of_vecs(size_t n)
{
    auto t1 = std::chrono::high_resolution_clock::now(); // perf
    for (size_t iter = 0; iter < n; ++iter)
        for (size_t i = 0; i < struct_of_vecs.pos.size(); ++i) {
            rigsystem::vec3& pos = struct_of_vecs.pos[i];
            rigsystem::vec3& vel = struct_of_vecs.vel[i];
            rigsystem::vec3& acc = struct_of_vecs.acc[i];
            rigsystem::vec3& frc = struct_of_vecs.frc[i];
            float mass = struct_of_vecs.mass[i];

            acc = frc / mass;
            vel += acc * dt;
            pos += vel * dt;
        }

    auto t2 = std::chrono::high_resolution_clock::now(); // perf
    std::chrono::duration<double, std::nano> ms = t2 - t1;
    std::cout << "[perf] struct of vecs, len = " << struct_of_vecs.pos.size() << ", iter time = " << ms.count() / n / struct_of_vecs.pos.size() << "ns\n";
}

void test_vos_avx256_2(size_t n)
{
    auto t1 = std::chrono::high_resolution_clock::now(); // perf
    for (size_t iter = 0; iter < n; ++iter)
        for (size_t i = 0; i < vec_of_structs_x2.size(); ++i) {
            TestNodeItemx2_t& nn = vec_of_structs_x2[i];

            // n.acc = n.frc / n.mass;
            nn.acc = nn.frc;
            for (size_t i = 0; i < 2; ++i) {
                nn.acc.v[i * 4 + 0] /= nn.mass[i];
                nn.acc.v[i * 4 + 1] /= nn.mass[i];
                nn.acc.v[i * 4 + 2] /= nn.mass[i];
                nn.acc.v[i * 4 + 3] /= nn.mass[i];
            }

            // n.vel += n.acc * dt;
            vec3x2_t t = nn.acc;
            t.mul(dt);
            nn.vel.add(t);

            // n.pos += n.vel * dt;
            t = nn.vel;
            t.mul(dt);
            nn.pos.add(t);
        }

    auto t2 = std::chrono::high_resolution_clock::now(); // perf
    std::chrono::duration<double, std::nano> ms = t2 - t1;
    std::cout << "[perf] test_vos_avx256, len = " << (vec_of_structs_x2.size() * 2) << ", iter time = " << ms.count() / n / (vec_of_structs_x2.size() * 2) << "ns\n";
}

void test_sov_avx256_2(size_t n)
{
    auto t1 = std::chrono::high_resolution_clock::now(); // perf
    for (size_t iter = 0; iter < n; ++iter)
        for (size_t i = 0; i < struct_of_vecs_x2.pos.size(); ++i) {
            vec3x2_t& pos = struct_of_vecs_x2.pos[i];
            vec3x2_t& vel = struct_of_vecs_x2.vel[i];
            vec3x2_t& acc = struct_of_vecs_x2.acc[i];
            vec3x2_t& frc = struct_of_vecs_x2.frc[i];
            float mass[] = { struct_of_vecs_x2.mass[i * 2], struct_of_vecs_x2.mass[i * 2 + 1] };

            // n.acc = n.frc / n.mass;
            acc = frc;
            for (size_t i = 0; i < 2; ++i) {
                acc.v[i * 4 + 0] /= mass[i];
                acc.v[i * 4 + 1] /= mass[i];
                acc.v[i * 4 + 2] /= mass[i];
                acc.v[i * 4 + 3] /= mass[i];
            }

            // n.vel += n.acc * dt;
            vec3x2_t t = acc;
            t.mul(dt);
            vel.add(t);

            // n.pos += n.vel * dt;
            t = vel;
            t.mul(dt);
            pos.add(t);
        }

    auto t2 = std::chrono::high_resolution_clock::now(); // perf
    std::chrono::duration<double, std::nano> ms = t2 - t1;
    std::cout << "[perf] test_sov_avx256, len = " << (struct_of_vecs_x2.pos.size() * 2) << ", iter time = " << ms.count() / n / (struct_of_vecs_x2.pos.size() * 2) << "ns\n";
}
void test_sov_avx256_8(size_t n)
{
    auto t1 = std::chrono::high_resolution_clock::now(); // perf
    for (size_t iter = 0; iter < n; ++iter)
        for (size_t i = 0; i < struct_of_vecs_x8.pos.size(); ++i) {
            vec3_x8_avx256_u_t& pos = struct_of_vecs_x8.pos[i];
            vec3_x8_avx256_u_t& vel = struct_of_vecs_x8.vel[i];
            vec3_x8_avx256_u_t& acc = struct_of_vecs_x8.acc[i];
            vec3_x8_avx256_u_t& frc = struct_of_vecs_x8.frc[i];
            vec3_x8_avx256_u_t& mass = struct_of_vecs_x8.mass[i];

            // n.acc = n.frc / n.mass;
            acc = frc;
            acc.div(mass);

            // n.vel += n.acc * dt;
            vec3_x8_avx256_u_t t = acc;
            t.mul(dt);
            vel.add(t);

            // n.pos += n.vel * dt;
            t = vel;
            t.mul(dt);
            pos.add(t);
        }

    auto t2 = std::chrono::high_resolution_clock::now(); // perf
    std::chrono::duration<double, std::nano> ms = t2 - t1;
    std::cout << "[perf] test_sov_avx256_8, len = " << (struct_of_vecs_x8.pos.size() * 8) << ", iter time = " << ms.count() / n / (struct_of_vecs_x8.pos.size() * 8) << "ns\n";
}

void test_vos_avx256_5(size_t n)
{
    auto t1 = std::chrono::high_resolution_clock::now(); // perf
    for (size_t iter = 0; iter < n; ++iter)
        for (size_t i = 0; i < vec_of_structs_x5.size(); ++i) {
            TestNodeItemx5_t& nn = vec_of_structs_x5[i];

            // n.acc = n.frc / n.mass;
            nn.acc = nn.frc;
            for (size_t i = 0; i < 16; ++i) {
                nn.acc.v[i] /= nn.mass[i / 3 % 5];
            }

            // n.vel += n.acc * dt;
            vec3x5_t t = nn.acc;
            t.mul(dt);
            nn.vel.add(t);

            // n.pos += n.vel * dt;
            t = nn.vel;
            t.mul(dt);
            nn.pos.add(t);
        }

    auto t2 = std::chrono::high_resolution_clock::now(); // perf
    std::chrono::duration<double, std::nano> ms = t2 - t1;
    std::cout << "[perf] test_vos_avx256_5, len = " << (vec_of_structs_x5.size() * 5) << ", iter time = " << ms.count() / n / (vec_of_structs_x5.size() * 5) << "ns\n";
}

void test_vos_avx256_8(size_t n)
{
    // Broadcast dt into a __m256 register.
    __m256 dt_v = _mm256_set1_ps(dt);

    auto t1 = std::chrono::high_resolution_clock::now();

    for (size_t iter = 0; iter < n; ++iter) {
        for (size_t i = 0; i < vec_of_structs_x8.size(); ++i) {
            TestNodeItemx8_t& nn = vec_of_structs_x8[i];

            // Compute acceleration: n.acc = n.frc / n.mass
            nn.acc.regs[0] = _mm256_div_ps(nn.frc.regs[0], nn.mass.regs[0]);
            nn.acc.regs[1] = _mm256_div_ps(nn.frc.regs[1], nn.mass.regs[1]);
            nn.acc.regs[2] = _mm256_div_ps(nn.frc.regs[2], nn.mass.regs[2]);

            // Update velocity: n.vel += n.acc * dt
            nn.vel.regs[0] = _mm256_add_ps(nn.vel.regs[0], _mm256_mul_ps(nn.acc.regs[0], dt_v));
            nn.vel.regs[1] = _mm256_add_ps(nn.vel.regs[1], _mm256_mul_ps(nn.acc.regs[1], dt_v));
            nn.vel.regs[2] = _mm256_add_ps(nn.vel.regs[2], _mm256_mul_ps(nn.acc.regs[2], dt_v));

            // Update position: n.pos += n.vel * dt
            nn.pos.regs[0] = _mm256_add_ps(nn.pos.regs[0], _mm256_mul_ps(nn.vel.regs[0], dt_v));
            nn.pos.regs[1] = _mm256_add_ps(nn.pos.regs[1], _mm256_mul_ps(nn.vel.regs[1], dt_v));
            nn.pos.regs[2] = _mm256_add_ps(nn.pos.regs[2], _mm256_mul_ps(nn.vel.regs[2], dt_v));
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::nano> ns = t2 - t1;
    std::cout << "[perf] test_vos_avx256_8, len = "
              << (vec_of_structs_x8.size() * 8)
              << ", iter time = "
              << ns.count() / n / (vec_of_structs_x8.size() * 8)
              << "ns\n";
}

void verify_data()
{
    for (size_t i = 0; i < vec_of_structs.size(); ++i) {
        rigsystem::vec3& a = vec_of_structs[i].pos;
        rigsystem::vec3& b = struct_of_vecs.pos[i];
        vec3x2_t& cc = vec_of_structs_x2[i / 2].pos;
        rigsystem::vec3 c = rigsystem::vec3(cc.v[0 + (i % 2) * 4], cc.v[1 + (i % 2) * 4], cc.v[2 + (i % 2) * 4]);

        vec3x2_t& dd = struct_of_vecs_x2.pos[i / 2];
        rigsystem::vec3 d = rigsystem::vec3(dd.v[0 + (i % 2) * 4], dd.v[1 + (i % 2) * 4], dd.v[2 + (i % 2) * 4]);

        vec3x5_t& ee = vec_of_structs_x5[i / 5].pos;
        rigsystem::vec3 e = rigsystem::vec3(ee.get_item(i % 5)[0], ee.get_item(i % 5)[1], ee.get_item(i % 5)[2]);

        vec3_x8_avx256_u_t ff = vec_of_structs_x8[i / 8].pos; // we have 8 vecs here
        rigsystem::vec3 f;
        ff.get_item(i % 8, f);

        vec3_x8_avx256_u_t gg = struct_of_vecs_x8.pos[i / 8];
        rigsystem::vec3 g;
        gg.get_item(i % 8, g);

        assert(a.distance_to(b) < 0.000001f);
        assert(a.distance_to(c) < 0.000001f);
        assert(a.distance_to(d) < 0.000001f);
        assert(a.distance_to(e) < 0.000001f);
        assert(a.distance_to(f) < 0.000001f);
        assert(a.distance_to(g) < 0.000001f);
    }
}

}
