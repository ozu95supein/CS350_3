#include <gtest/gtest.h>
#include "common.hpp"
#include "bvh_indexed.hpp"
#include <fstream>

constexpr float c_testEpsilon = 0.001f;

/**
 * @brief 
 *  Retrieve all the indices of a node (recursively)
 */
std::vector<unsigned> bvh_flat_map(bvh_indexed::node const* n)
{
    std::vector<unsigned> indices;
    bvh_traverse_level_order(
        [&indices](bvh_indexed::node const* c) {
            if (c && c->is_leaf())
                for (unsigned i = 0; i < c->object_count; ++i)
                    indices.push_back(c->objects[i]);
        },
        n);
    return indices;
}

/**
 * @brief 
 *  Ensures that all the BVs in the BVH are tight (with respect to the objects)
 */
void assert_tight(bvh_indexed const& bvh, std::vector<aabb> const& bvs)
{
    bvh.traverse_level_order([&bvs](bvh_indexed::node const* n) {
        auto indices = bvh_flat_map(n);          // Retrieve all indices of the current nodes children (recursively)
        ASSERT_FALSE(indices.empty());           // All nodes should contain at least a point
        auto expected_bv = bvs[indices.front()]; // First point should be part of the BV

        // Ensure that all vertices lie inside the current bv
        for (auto triangle_index : indices) {
            auto const& bv = bvs[triangle_index];

            // Update boundaries
            expected_bv.min = glm::min(expected_bv.min, bv.min);
            expected_bv.max = glm::max(expected_bv.max, bv.max);
        }

        ASSERT_NEAR(expected_bv.min, n->bv.min, c_testEpsilon); // Ensure tight bv
        ASSERT_NEAR(expected_bv.max, n->bv.max, c_testEpsilon); // Ensure tight bv

        // Sanity check
        if (!n->is_leaf()) {
            ASSERT_EQ(n->objects, nullptr);
            ASSERT_EQ(n->object_count, 0u);
            ASSERT_NE(n->children[0], nullptr);
            ASSERT_NE(n->children[1], nullptr);
        } else {
            ASSERT_NE(n->objects, nullptr);
            ASSERT_NE(n->object_count, 0u);
            ASSERT_EQ(n->children[0], nullptr);
            ASSERT_EQ(n->children[1], nullptr);
        }
    });
}

TEST(bvh, unused)
{
    ASSERT_NO_FATAL_FAILURE({ bvh_indexed bvh; }); // Constructor + destructor of unused
}

TEST(bvh, empty_create)
{
    bvh_indexed bvh;
    ASSERT_NO_FATAL_FAILURE(bvh.build_top_down(nullptr, 0, 0));
    ASSERT_NO_FATAL_FAILURE(bvh.build_bottom_up(nullptr, 0));
}

TEST(bvh, top_down_single_aabb)
{
    // Scene BVs
    aabb const bvs[]    = {aabb{{0, 0, 0}, {1, 1, 1}}};
    auto const bv_count = sizeof(bvs) / sizeof(*bvs);

    // BVH
    bvh_indexed bvh;
    bvh.build_top_down(bvs, bv_count, std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);
    auto bvh_objects = bvh_flat_map(bvh.root());

    //
    ASSERT_NEAR(bvh.root()->bv, bvs[0], c_testEpsilon);
    ASSERT_EQ(bvh.root()->depth(), 0);
    ASSERT_EQ(bvh.root()->size(), 1);
    ASSERT_EQ(bvh_objects.size(), 1u);
    ASSERT_EQ(bvh_objects.front(), 0u);
}

TEST(bvh, top_down_pair_aabb)
{
    // Scene BVs
    std::vector<aabb> bvs = {
        aabb{{0, 0, 0}, {1, 1, 1}},
        aabb{{1, 0, 0}, {2, 1, 1}},
    };

    // BVH
    bvh_indexed bvh;
    bvh.build_top_down(bvs.data(), bvs.size(), std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);
    auto bvh_objects = bvh_flat_map(bvh.root());

    //
    aabb full = {{0, 0, 0}, {2, 1, 1}};       // Full scene aabb
    ASSERT_NEAR(bvh.root()->bv, full, c_testEpsilon); // Should match
    ASSERT_EQ(bvh.root()->depth(), 1);
    ASSERT_EQ(bvh.root()->size(), 3);
    ASSERT_EQ(bvh_objects.size(), 2u);
}

TEST(bvh, top_down_clear_check)
{
    // Scene BVs
    std::vector<aabb> bvs = {aabb{{0, 0, 0}, {1, 1, 1}}};

    // BVH
    bvh_indexed bvh;
    bvh.build_top_down(bvs.data(), bvs.size(), std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);
    bvh.clear();

    //
    ASSERT_NO_FATAL_FAILURE(bvh.depth());
    ASSERT_NO_FATAL_FAILURE(bvh.size());
    ASSERT_EQ(bvh.depth(), -1);
    ASSERT_EQ(bvh.size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST(bvh, top_down_rebuild)
{
    // Scene BVs
    std::vector<aabb> bvs = {aabb{{0, 0, 0}, {1, 1, 1}}};

    // BVH
    bvh_indexed bvh;
    bvh.build_top_down(bvs.data(), bvs.size(), std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);

    //
    ASSERT_NO_FATAL_FAILURE(bvh.build_top_down(bvs.data(), bvs.size(), 0));
}

TEST(bvh, top_down_avocado)
{
    // Scene
    auto tris = load_triangles_cs350(WORKDIR "test/bvh_data/avocado.cs350");
    auto bvs  = triangles_to_aabbs(tris);
    ASSERT_TRUE(!tris.empty()) << "Could not parse triangles";

    // BVH
    bvh_indexed bvh;
    bvh.build_top_down(bvs.data(), bvs.size(), std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);

    //
    assert_tight(bvh, bvs);
}

TEST(bvh, bottom_up_single_aabb)
{
    // Scene BVs
    std::vector<aabb> bvs = {aabb{{0, 0, 0}, {1, 1, 1}}};

    // BVH
    bvh_indexed bvh;
    bvh.build_bottom_up(bvs.data(), bvs.size());
    bvh.dump_info(std::cerr);

    //
    ASSERT_NEAR(bvh.root()->bv, bvs[0], c_testEpsilon);
    assert_tight(bvh, bvs);
}

TEST(bvh, bottom_up_pair_aabb)
{
    // Scene BVs
    std::vector<aabb> bvs = {
        aabb{{0, 0, 0}, {1, 1, 1}},
        aabb{{1, 0, 0}, {2, 1, 1}},
    };

    // BVH
    bvh_indexed bvh;
    bvh.build_bottom_up(bvs.data(), bvs.size());
    bvh.dump_info(std::cerr);
    auto bvh_objects = bvh_flat_map(bvh.root());

    //
    aabb full = {{0, 0, 0}, {2, 1, 1}};       // Full scene aabb
    ASSERT_NEAR(bvh.root()->bv, full, c_testEpsilon); // Should match
    ASSERT_EQ(bvh.root()->depth(), 1);
    ASSERT_EQ(bvh.root()->size(), 3);
    ASSERT_EQ(bvh_objects.size(), 2u);
}

TEST(bvh, bottom_up_clear_check)
{
    // Scene BVs
    std::vector<aabb> bvs = {aabb{{0, 0, 0}, {1, 1, 1}}};

    // BVH
    bvh_indexed bvh;
    bvh.build_bottom_up(bvs.data(), bvs.size());
    bvh.dump_info(std::cerr);
    bvh.clear();

    //
    ASSERT_NO_FATAL_FAILURE(bvh.depth());
    ASSERT_NO_FATAL_FAILURE(bvh.size());
    ASSERT_EQ(bvh.depth(), -1);
    ASSERT_EQ(bvh.size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST(bvh, bottom_up_rebuild)
{
    // Scene BVs
    std::vector<aabb> bvs = {aabb{{0, 0, 0}, {1, 1, 1}}};

    // BVH
    bvh_indexed bvh;
    bvh.build_bottom_up(bvs.data(), bvs.size());
    bvh.dump_info(std::cerr);
    //
    ASSERT_NO_FATAL_FAILURE(bvh.build_top_down(bvs.data(), bvs.size(), 0));
}

TEST(bvh, bottom_up_avocado)
{
    // Scene
    auto tris = load_triangles_cs350(WORKDIR "test/bvh_data/avocado.cs350");
    auto bvs  = triangles_to_aabbs(tris);
    ASSERT_TRUE(!tris.empty()) << "Could not parse triangles";

    // BVH
    bvh_indexed bvh;
    bvh.build_bottom_up(bvs.data(), bvs.size()); // Takes a while
    bvh.dump_info(std::cerr);

    //
    assert_tight(bvh, bvs);
}

TEST(bvh, insert_single_aabb)
{
    // Scene BVs
    std::vector<aabb> bvs = {aabb{{0, 0, 0}, {1, 1, 1}}};

    // BVH
    bvh_indexed bvh;
    for (unsigned i = 0; i < bvs.size(); ++i)
        bvh.insert(bvs[i], i, std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);

    //
    ASSERT_NEAR(bvh.root()->bv, bvs[0], c_testEpsilon);
    assert_tight(bvh, bvs);
}

TEST(bvh, insert_pair_aabb)
{
    // Scene BVs
    std::vector<aabb> bvs = {
        aabb{{0, 0, 0}, {1, 1, 1}},
        aabb{{1, 0, 0}, {2, 1, 1}},
    };

    // BVH
    bvh_indexed bvh;
    for (unsigned i = 0; i < bvs.size(); ++i)
        bvh.insert(bvs[i], i, std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);
    auto bvh_objects = bvh_flat_map(bvh.root());

    //
    aabb full = {{0, 0, 0}, {2, 1, 1}};       // Full scene aabb
    ASSERT_NEAR(bvh.root()->bv, full, c_testEpsilon); // Should match
    ASSERT_EQ(bvh.root()->depth(), 1);
    ASSERT_EQ(bvh.root()->size(), 3);
    ASSERT_EQ(bvh_objects.size(), 2u);
}

TEST(bvh, insert_clear_check)
{
    // Scene BVs
    std::vector<aabb> bvs = {aabb{{0, 0, 0}, {1, 1, 1}}};

    // BVH
    bvh_indexed bvh;
    for (unsigned i = 0; i < bvs.size(); ++i)
        bvh.insert(bvs[i], i, std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);
    bvh.clear();

    //
    ASSERT_NO_FATAL_FAILURE(bvh.depth());
    ASSERT_NO_FATAL_FAILURE(bvh.size());
    ASSERT_EQ(bvh.depth(), -1);
    ASSERT_EQ(bvh.size(), 0);
    ASSERT_EQ(bvh.root(), nullptr);
}

TEST(bvh, insert_rebuild)
{
    // Scene BVs
    std::vector<aabb> bvs = {aabb{{0, 0, 0}, {1, 1, 1}}};

    // BVH
    bvh_indexed bvh;
    for (unsigned i = 0; i < bvs.size(); ++i)
        bvh.insert(bvs[i], i, std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);
    //
    ASSERT_NO_FATAL_FAILURE(bvh.build_top_down(bvs.data(), bvs.size(), 0));
}

TEST(bvh, insert_avocado)
{
    // Scene
    auto tris = load_triangles_cs350(WORKDIR "test/bvh_data/avocado.cs350");
    auto bvs  = triangles_to_aabbs(tris);
    ASSERT_TRUE(!tris.empty()) << "Could not parse triangles";

    // BVH
    bvh_indexed bvh;
    for (unsigned i = 0; i < bvs.size(); ++i)
        bvh.insert(bvs[i], i, std::numeric_limits<unsigned>::max());
    bvh.dump_info(std::cerr);

    //
    assert_tight(bvh, bvs);
}
