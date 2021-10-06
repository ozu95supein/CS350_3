#ifndef __BVH_INDEXED_HPP__
#define __BVH_INDEXED_HPP__

#include "shapes.hpp"
#include <ostream>

class bvh_indexed
{
  public: // Public for testing reasons
    struct node
    {
        aabb      bv;
        node*     children[2];
        unsigned* objects;
        unsigned  object_count;

        int  depth() const;
        int  size() const;
        bool is_leaf() const;
    };

  private:
    node* m_root;

  public:
    bvh_indexed();
    ~bvh_indexed();
    bvh_indexed(bvh_indexed const&) = delete;
    bvh_indexed& operator=(bvh_indexed const&) = delete;

    void build_top_down(aabb const* bvs, unsigned bv_count, unsigned max_depth);
    void build_bottom_up(aabb const* bvs, unsigned bv_count);
    void insert(aabb const& bv, unsigned id, unsigned max_depth);

    void        clear();
    bool        empty() const;
    int         depth() const;
    int         size() const;
    node const* root() const;                                     // Debug
    void        dump_info(std::ostream& os) const;                // Debug
    void        dump_info(std::ostream& os, node const* n) const; // Debug

    /**
     * @tparam T 
     *  A predicate of the form void(bvh_indexed::node const*) 
     */
    template <typename T>
    void traverse_level_order(T func) const; // Debug

  private:
};

#endif // __BVH_INDEXED_HPP__
